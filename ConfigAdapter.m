function [analysis_params, sim_params] = ConfigAdapter(preConfig, identifiedParams)

if nargin < 2
        identifiedParams = [];
    end
    
    %% 1. 生成参数识别所需参数
    analysis_params = struct();
    analysis_params.fs_target = preConfig.signal.fs_target;
    analysis_params.cutoff_freq = preConfig.signal.cutoff_freq;
    analysis_params.filter_order = preConfig.signal.filter_order;
    analysis_params.nfft = preConfig.signal.nfft;
    analysis_params.freq_range = [preConfig.signal.freq_range_min, preConfig.signal.freq_range_max];
    analysis_params.snr_threshold = preConfig.signal.snr_threshold;
    analysis_params.n_nodes = 3;
    analysis_params.n_dof = 6;
    analysis_params.node_labels = {'Root', 'Mid', 'Tip'};
    analysis_params.direction_labels = {'X', 'Z'};
    analysis_params.topology = preConfig.topology;

    %% 2. 检查是否需要生成仿真参数
    if isempty(identifiedParams)
        sim_params = [];
        fprintf('ConfigAdapter: 仅返回分析参数，未生成仿真参数 (identifiedParams 为空)。\n');
        return; 
    end
    
    %% 3. 生成仿真所需参数
    sim_params = struct();
    
    % --- 基础设置 ---
    sim_params.parallel_max_workers = preConfig.basic.parallel_max_workers; 
    sim_params.workFolder = preConfig.basic.workFolder;
    
    if isfield(preConfig.basic, 'modelName')
        sim_params.model_name = preConfig.basic.modelName;
    else
        sim_params.model_name = 'MDOF_Hierarchical_Vibration_Sim'; 
    end
    
    sim_params.gravity_g = preConfig.basic.gravity_g;
    sim_params.use_parallel = preConfig.basic.useParallel;
    
    % =========================================================================
    % 拓扑结构自动补全
    % =========================================================================
    sim_params.config = preConfig.topology;
    
    % 补全 num_primary_branches 及计数数组
    if isfield(sim_params.config, 'structure')
        structure = sim_params.config.structure;
        num_p = length(structure);
        
        sim_params.config.num_primary_branches = num_p;
        sim_params.config.secondary_branches_count = zeros(1, num_p);
        sim_params.config.tertiary_branches_count = cell(1, num_p);
        
        for i = 1:num_p
            vec = structure{i};
            if isequal(vec, -1) || (length(vec)==1 && vec(1) == -1)
                sim_params.config.secondary_branches_count(i) = 0;
                sim_params.config.tertiary_branches_count{i} = [];
            else
                sim_params.config.secondary_branches_count(i) = length(vec);
                sim_params.config.tertiary_branches_count{i} = vec;
            end
        end
    end
    
    if isfield(preConfig, 'trunk')
        sim_params.config.trunk = preConfig.trunk;
    end
    
    % --- 构建主干参数 ---
    sim_params.trunk = buildTrunkParams(preConfig, identifiedParams);
    
    % --- 构建分枝参数 ---
    sim_params.predefined_params = generatePredefinedParams(preConfig, identifiedParams);

    % --- 显式调用验证函数，确保生成的分枝参数覆盖了所有拓扑节点---
    validatePredefinedParams(sim_params.predefined_params, preConfig);
    
    % --- 构建果实配置 ---
    sim_params.fruit_config = struct();
    sim_params.fruit_config.attach_secondary_mid = preConfig.fruit.attach_secondary_mid;
    sim_params.fruit_config.attach_secondary_tip = preConfig.fruit.attach_secondary_tip;
    sim_params.fruit_config.attach_tertiary_mid = preConfig.fruit.attach_tertiary_mid;
    sim_params.fruit_config.attach_tertiary_tip = preConfig.fruit.attach_tertiary_tip;
    
    if isfield(preConfig.fruit, 'fruits_per_node')
        sim_params.fruit_config.fruits_per_node = preConfig.fruit.fruits_per_node;
    else
        sim_params.fruit_config.fruits_per_node = 1;
    end
    
    % 生成默认果实参数
    sim_params.default_fruit_params = buildFruitParamsStrict(preConfig, identifiedParams);
    
    % 激励参数
    sim_params.excitation = preConfig.excitation;
    
    % 更新激励频率
    found_freq = false;
    if isfield(identifiedParams, 'branches') && isfield(identifiedParams.branches, 'Trunk') && ...
       isfield(identifiedParams.branches.Trunk, 'linear') && ...
       isfield(identifiedParams.branches.Trunk.linear, 'natural_freqs_x')
        freqs = identifiedParams.branches.Trunk.linear.natural_freqs_x;
        if ~isempty(freqs)
            sim_params.excitation.frequency_hz = freqs(1);
            found_freq = true;
        end
    end
    
    if ~found_freq && isfield(identifiedParams, 'linear') && ...
       isfield(identifiedParams.linear, 'natural_freqs_x')
        freqs = identifiedParams.linear.natural_freqs_x;
        if ~isempty(freqs)
            sim_params.excitation.frequency_hz = freqs(1);
        end
    end

    % 仿真控制
    sim_params.sim_stop_time = preConfig.simulation.stop_time;
    sim_params.sim_fixed_step = preConfig.simulation.fixed_step;
    sim_params.has_identified_params = true;
    
    fprintf('ConfigAdapter: 参数适配完成。\n');
end

%% ==================== 构建主干参数 (Trunk) ====================
function trunk = buildTrunkParams(preConfig, identifiedParams)
    trunk = struct();
    
    % 1. 几何与质量
    trunk.length = preConfig.trunk.length;
    trunk.diameter_base = preConfig.trunk.diameter_base;
    trunk.diameter_tip = preConfig.trunk.diameter_tip;
    
    m_total = preConfig.trunk.total_mass;
    m_dist = preConfig.trunk.mass_distribution;
    trunk.root.m = m_total * m_dist(1);
    trunk.mid.m = m_total * m_dist(2);
    trunk.tip.m = m_total * m_dist(3);
    
    % 2. 准备源数据 (Source Data Preparation)
    % 这一步将源数据的“非对称性”抹平，后续逻辑即可对称处理
    
    src_data_x = [];
    src_data_z = [];
    src_nonlin_x = [];
    src_nonlin_z = [];
    
    % 获取 Trunk 分枝专用数据
    if isfield(identifiedParams, 'branches') && isfield(identifiedParams.branches, 'Trunk')
        b_data = identifiedParams.branches.Trunk;
        
        % 读取线性 X (优先找 linear_x, 兼容旧版 linear)
        if isfield(b_data, 'linear_x'), src_data_x = b_data.linear_x;
        elseif isfield(b_data, 'linear'), src_data_x = b_data.linear; end
        
        % 读取线性 Z
        if isfield(b_data, 'linear_z'), src_data_z = b_data.linear_z; end
        
        % 读取非线性 X
        if isfield(b_data, 'nonlinear_x'), src_nonlin_x = b_data.nonlinear_x;
        elseif isfield(b_data, 'nonlinear'), src_nonlin_x = b_data.nonlinear; end
        
        % 读取非线性 Z
        if isfield(b_data, 'nonlinear_z'), src_nonlin_z = b_data.nonlinear_z; end
    
    else
        % 回退到全局数据
        if isfield(identifiedParams, 'linear_x'), src_data_x = identifiedParams.linear_x;
        elseif isfield(identifiedParams, 'linear'), src_data_x = identifiedParams.linear; end
        
        if isfield(identifiedParams, 'linear_z'), src_data_z = identifiedParams.linear_z; end
        
        if isfield(identifiedParams, 'nonlinear_x'), src_nonlin_x = identifiedParams.nonlinear_x;
        elseif isfield(identifiedParams, 'nonlinear'), src_nonlin_x = identifiedParams.nonlinear; end
        
        if isfield(identifiedParams, 'nonlinear_z'), src_nonlin_z = identifiedParams.nonlinear_z; end
    end
    
    if isempty(src_data_x)
        error('ConfigAdapter:NoTrunkParams', '错误：未在 identifiedParams 中找到主干 X 方向参数。');
    end

    % 3. 初始化所有参数为 0 (安全默认值)
    segments = {'root', 'mid', 'tip'};
    directions = {'x', 'z'};
    params = {'k', 'c', 'k3', 'c2'};
    
    for s = 1:length(segments)
        seg = segments{s};
        for d = 1:length(directions)
            dir = directions{d};
            % 连接到基座(root) 或 连接到上一级(mid/tip)
            suffix = '_conn';
            if strcmp(seg, 'root'), suffix = '_conn_to_base'; end
            
            for p = 1:length(params)
                par = params{p};
                trunk.(seg).([par '_' dir suffix]) = 0;
            end
        end
    end

    % 4. 映射: 传感器 X -> 仿真模型 Y (Coordinate Mapping: Sensor X => Model Y)
    if ~isempty(src_data_x) && isfield(src_data_x, 'identified_params_x')
        p_vec = src_data_x.identified_params_x;
        trunk.root.k_x_conn_to_base = p_vec(1); trunk.root.c_x_conn_to_base = p_vec(2);
        trunk.root.k_x_conn = p_vec(3);         trunk.root.c_x_conn = p_vec(4);
        trunk.mid.k_x_conn  = p_vec(5);         trunk.mid.c_x_conn  = p_vec(6);
    else
        error('ConfigAdapter:MissingXData', '主干参数缺少 X 方向数据 (identified_params_x)。');
    end
    
    if ~isempty(src_nonlin_x) && isfield(src_nonlin_x, 'k3_coeffs')
        k3 = src_nonlin_x.k3_coeffs; c2 = src_nonlin_x.c2_coeffs;
        if length(k3)>=1, trunk.root.k3_x_conn_to_base = k3(1); trunk.root.c2_x_conn_to_base = c2(1); end
        if length(k3)>=2, trunk.root.k3_x_conn = k3(2);         trunk.root.c2_x_conn = c2(2); end
        if length(k3)>=3, trunk.mid.k3_x_conn  = k3(3);         trunk.mid.c2_x_conn  = c2(3); end
        fprintf('    [ConfigAdapter] 主干 X 数据 -> X 模型参数 (非线性) 已应用。\n');
    end

    % 5. 映射: 传感器 Z -> 仿真模型 Z (Direct Mapping)
    if ~isempty(src_data_z) && isfield(src_data_z, 'identified_params_z')
        p_vec = src_data_z.identified_params_z;
        trunk.root.k_z_conn_to_base = p_vec(1); trunk.root.c_z_conn_to_base = p_vec(2);
        trunk.root.k_z_conn = p_vec(3);         trunk.root.c_z_conn = p_vec(4);
        trunk.mid.k_z_conn  = p_vec(5);         trunk.mid.c_z_conn  = p_vec(6);
    else
        error('ConfigAdapter:NoZParams', '严重错误：主干缺少 Z 方向识别参数 (identified_params_z)。');
    end

    if ~isempty(src_nonlin_z) && isfield(src_nonlin_z, 'k3_coeffs')
        k3 = src_nonlin_z.k3_coeffs; c2 = src_nonlin_z.c2_coeffs;
        if length(k3)>=1, trunk.root.k3_z_conn_to_base = k3(1); trunk.root.c2_z_conn_to_base = c2(1); end
        if length(k3)>=2, trunk.root.k3_z_conn = k3(2);         trunk.root.c2_z_conn = c2(2); end
        if length(k3)>=3, trunk.mid.k3_z_conn  = k3(3);         trunk.mid.c2_z_conn  = c2(3); end
        fprintf('    [ConfigAdapter] 主干 Z 数据 -> Z 模型参数 (非线性) 已应用。\n');
    else
        fprintf('    [ConfigAdapter] 主干 Z 非线性参数未找到 (linear default)。\n');
    end
end

%% ==================== 验证预定义参数完整性 ====================
function validatePredefinedParams(predefined, preConfig)
    missingBranches = {};
    structure = preConfig.topology.structure;
    numP = length(structure);
    for p = 1:numP
        bid_p = sprintf('P%d', p);
        if ~isfield(predefined, bid_p), missingBranches{end+1} = bid_p; end
        
        vec = structure{p};
        if isequal(vec, -1) || (length(vec)==1 && vec(1) == -1), continue; end
        
        numS = length(vec);
        for s = 1:numS
            bid_s = sprintf('P%d_S%d', p, s);
            if ~isfield(predefined, bid_s), missingBranches{end+1} = bid_s; end
            
            numT = vec(s);
            for t = 1:numT
                bid_t = sprintf('P%d_S%d_T%d', p, s, t);
                if ~isfield(predefined, bid_t), missingBranches{end+1} = bid_t; end
            end
        end
    end
    
    if ~isempty(missingBranches)
        error('ConfigAdapter:MissingData', '以下分枝在预配置中缺少几何参数:\n  %s', strjoin(missingBranches, ', '));
    end
end

%% ==================== 为分枝生成默认段参数 ====================
function predefined = generatePredefinedParams(preConfig, identifiedParams)
    
    if isempty(preConfig) || isempty(identifiedParams)
        error('ConfigAdapter:MissingData', '缺少配置或识别参数');
    end
    
    structure = preConfig.topology.structure;
    predefined = struct();
    fruitConfig = preConfig.fruit;
    
    function processBranch(name, type_struct)
        lvl = determineBranchLevel(name);
        if ~isfield(type_struct, name), error('缺少分枝 %s 的几何参数', name); end
        geom = type_struct.(name);
        validateBranchGeometry(geom, name);
        
        [k_b, c_b, k3_b, c2_b, specific_taper, z_params] = estimateStiffnessDamping(geom, identifiedParams, name);
        
        predefined.(name) = generateBranchSegmentParams(geom, k_b, c_b, k3_b, c2_b, specific_taper, z_params);
        
        predefined.(name).branch_level = lvl;
        if shouldAttachFruit(name, lvl, fruitConfig)
            if shouldAttachAtPosition(lvl, 'mid', fruitConfig)
                predefined.(name).fruit_at_mid = buildFruitParamsStrict(preConfig, identifiedParams);
            end
            if shouldAttachAtPosition(lvl, 'tip', fruitConfig)
                predefined.(name).fruit_at_tip = buildFruitParamsStrict(preConfig, identifiedParams);
            end
        end
    end

    numP = length(structure);
    for p = 1:numP
        branch_name_p = sprintf('P%d', p);
        processBranch(branch_name_p, preConfig.primary);
        
        vec = structure{p};
        if isequal(vec, -1) || (length(vec)==1 && vec(1) == -1), continue; end
        
        numS = length(vec);
        for s = 1:numS
            branch_name_s = sprintf('P%d_S%d', p, s);
            processBranch(branch_name_s, preConfig.secondary);
            
            numT = vec(s);
            for t = 1:numT
                branch_name_t = sprintf('P%d_S%d_T%d', p, s, t);
                processBranch(branch_name_t, preConfig.tertiary);
            end
        end
    end
end

%% ==================== 生成单个分枝的段参数 ====================
function params = generateBranchSegmentParams(branchGeom, k_base, c_base, k3_base, c2_base, identified_taper, z_params)
    
    if isempty(identified_taper), error('ConfigAdapter:MissingTaper', '缺少 Y 方向分布因子 identified_taper。'); end
    if isempty(z_params)
        error('ConfigAdapter:MissingZData', '严重错误：Z 方向实测数据为空。');
    end
    
    k_taper = identified_taper.k; c_taper = identified_taper.c;
    k3_taper = identified_taper.k3; c2_taper = identified_taper.c2;
    
    params = struct();
    m_total = branchGeom.total_mass;
    m_dist = branchGeom.mass_dist;
    
    % Root段
    params.root.m = m_total * m_dist(1);
    params.root.k_x_conn = k_base * k_taper(1);
    params.root.c_x_conn = c_base * c_taper(1);
    params.root.k3_x_conn = k3_base * k3_taper(1);
    params.root.c2_x_conn = c2_base * c2_taper(1);
    params.root.k_z_conn = z_params.k_base * z_params.k_taper(1);
    params.root.c_z_conn = z_params.c_base * z_params.c_taper(1);
    params.root.k3_z_conn = z_params.k3_base * z_params.k3_taper(1);
    params.root.c2_z_conn = z_params.c2_base * z_params.c2_taper(1);
    
    % Mid段
    params.mid.m = m_total * m_dist(2);
    params.mid.k_x_conn = k_base * k_taper(2);
    params.mid.c_x_conn = c_base * c_taper(2);
    params.mid.k3_x_conn = k3_base * k3_taper(2);
    params.mid.c2_x_conn = c2_base * c2_taper(2);
    params.mid.k_z_conn = z_params.k_base * z_params.k_taper(2);
    params.mid.c_z_conn = z_params.c_base * z_params.c_taper(2);
    params.mid.k3_z_conn = z_params.k3_base * z_params.k3_taper(2);
    params.mid.c2_z_conn = z_params.c2_base * z_params.c2_taper(2);
    
    % Tip段
    params.tip.m = m_total * m_dist(3);
    params.tip.k_x_conn = k_base * k_taper(3);
    params.tip.c_x_conn = c_base * c_taper(3);
    params.tip.k3_x_conn = k3_base * k3_taper(3);
    params.tip.c2_x_conn = c2_base * c2_taper(3);
    params.tip.k_z_conn = z_params.k_base * z_params.k_taper(3);
    params.tip.c_z_conn = z_params.c_base * z_params.c_taper(3);
    params.tip.k3_z_conn = z_params.k3_base * z_params.k3_taper(3);
    params.tip.c2_z_conn = z_params.c2_base * z_params.c2_taper(3);
    
    params.geometry.length = branchGeom.length;
    params.geometry.diameter_base = branchGeom.diameter_base;
    params.geometry.diameter_tip = branchGeom.diameter_tip;
    params.taper_factors = identified_taper;
end

%% ==================== 辅助函数: 提取基准值和分布 ====================
function [k_base, c_base, k3_base, c2_base, branch_taper, z_params] = estimateStiffnessDamping(branchGeom, identifiedParams, branchName)
    
    % 1. 准备本地数据容器 (X/Z 独立)
    src_lin_x = []; src_nonlin_x = [];
    src_lin_z = []; src_nonlin_z = [];
    
    % 2. 尝试从分枝专用数据读取 (Branch Specific)
    if isfield(identifiedParams, 'branches') && isfield(identifiedParams.branches, branchName)
        branch_data = identifiedParams.branches.(branchName);
        
        % 读取 X (支持 linear_x 和旧版 linear)
        if isfield(branch_data, 'linear_x'), src_lin_x = branch_data.linear_x;
        elseif isfield(branch_data, 'linear'), src_lin_x = branch_data.linear; end
        
        % 读取 Z
        if isfield(branch_data, 'linear_z'), src_lin_z = branch_data.linear_z; end
        
        % 读取非线性 X
        if isfield(branch_data, 'nonlinear_x'), src_nonlin_x = branch_data.nonlinear_x;
        elseif isfield(branch_data, 'nonlinear'), src_nonlin_x = branch_data.nonlinear; end
        
        % 读取非线性 Z
        if isfield(branch_data, 'nonlinear_z'), src_nonlin_z = branch_data.nonlinear_z; end
    end
    
    % 3. 尝试从全局数据读取 (Global Fallback)
    glob_lin_x = []; glob_nonlin_x = [];
    glob_lin_z = []; glob_nonlin_z = [];
    
    % 全局 X 读取
    if isfield(identifiedParams, 'linear_x'), glob_lin_x = identifiedParams.linear_x;
    elseif isfield(identifiedParams, 'linear'), glob_lin_x = identifiedParams.linear; end
    
    % 全局 Z 读取
    if isfield(identifiedParams, 'linear_z'), glob_lin_z = identifiedParams.linear_z; end
    
    % 兼容旧结构: Z 可能藏在 linear 中
    if isempty(glob_lin_z) && isfield(identifiedParams, 'linear') && isfield(identifiedParams.linear, 'identified_params_z')
         glob_lin_z = identifiedParams.linear;
    end
    
    % 全局非线性
    if isfield(identifiedParams, 'nonlinear_x'), glob_nonlin_x = identifiedParams.nonlinear_x;
    elseif isfield(identifiedParams, 'nonlinear'), glob_nonlin_x = identifiedParams.nonlinear; end
    
    if isfield(identifiedParams, 'nonlinear_z'), glob_nonlin_z = identifiedParams.nonlinear_z; end
    
    % 4. 融合数据
    final_lin_x    = src_lin_x;    if isempty(final_lin_x),    final_lin_x = glob_lin_x;       end
    final_lin_z    = src_lin_z;    if isempty(final_lin_z),    final_lin_z = glob_lin_z;       end
    final_nonlin_x = src_nonlin_x; if isempty(final_nonlin_x), final_nonlin_x = glob_nonlin_x; end
    final_nonlin_z = src_nonlin_z; if isempty(final_nonlin_z), final_nonlin_z = glob_nonlin_z; end
    
    % 5. 提取 X 参数 -> 映射为模型的 Y 参数 (Coordinate Mapping: Sensor X => Model Y)
    if isempty(final_lin_x)
        error('ConfigAdapter:MissingXData', '分枝 "%s" 缺少 X 方向线性数据。', branchName);
    end
    
    % 处理 K_x / C_x
    if isfield(final_lin_x, 'K_x'), K_mat = final_lin_x.K_x; C_mat = final_lin_x.C_x;
    elseif isfield(final_lin_x, 'K'), K_mat = final_lin_x.K; C_mat = final_lin_x.C; 
    else
        % 向量格式回退
        vec = final_lin_x.identified_params_x;
        k_d = [vec(1)+vec(3), vec(3)+vec(5), vec(5)];
        c_d = [vec(2)+vec(4), vec(4)+vec(6), vec(6)];
        K_mat = diag(k_d); C_mat = diag(c_d);
    end
    
    K_vals = diag(K_mat); C_vals = diag(C_mat);
    k_base = max(K_vals); k_taper = K_vals / k_base; 
    c_base = max(C_vals); c_taper = C_vals / c_base;
    [k3_base, c2_base, k3_taper, c2_taper] = extractNonlinearBaseTaper(final_nonlin_x);
    
    % 6. 提取 Z 参数 -> 映射为模型的 Z 参数
    if isempty(final_lin_z)
        warning('ConfigAdapter:MissingZData', '分枝 "%s" 缺少 Z 方向线性数据，尝试使用 X 参数作为近似。', branchName);
        % 这里的回退是为了保证代码不报错，但在您的场景中应尽量避免
        final_lin_z = final_lin_x; 
        K_mat_z = K_mat; C_mat_z = C_mat;
    else
        if isfield(final_lin_z, 'K_z'), K_mat_z = final_lin_z.K_z; C_mat_z = final_lin_z.C_z;
        elseif isfield(final_lin_z, 'K'), K_mat_z = final_lin_z.K; C_mat_z = final_lin_z.C;
        else
             vec = final_lin_z.identified_params_z;
             k_d = [vec(1)+vec(3), vec(3)+vec(5), vec(5)];
             c_d = [vec(2)+vec(4), vec(4)+vec(6), vec(6)];
             K_mat_z = diag(k_d); C_mat_z = diag(c_d);
        end
    end
    
    z_params = struct();
    K_vals_z = diag(K_mat_z); C_vals_z = diag(C_mat_z);
    z_params.k_base = max(K_vals_z); z_params.c_base = max(C_vals_z);
    z_params.k_taper = K_vals_z / z_params.k_base; z_params.c_taper = C_vals_z / z_params.c_base;
    [z_params.k3_base, z_params.c2_base, z_params.k3_taper, z_params.c2_taper] = extractNonlinearBaseTaper(final_nonlin_z);
    
    branch_taper = struct('k', k_taper, 'c', c_taper, 'k3', k3_taper, 'c2', c2_taper);
end

function [k3_base, c2_base, k3_taper, c2_taper] = extractNonlinearBaseTaper(nl_struct)
    % 提取非线性参数
    if ~isempty(nl_struct) && isfield(nl_struct, 'k3_coeffs')
        k3_vals = nl_struct.k3_coeffs(:); c2_vals = nl_struct.c2_coeffs(:);
        if length(k3_vals) < 3, k3_vals(end+1:3) = 0; end
        if length(c2_vals) < 3, c2_vals(end+1:3) = 0; end
        [max_k3, idx_k3] = max(abs(k3_vals(1:3)));
        if max_k3 > 1e-12, k3_base = k3_vals(idx_k3); k3_taper = k3_vals(1:3)/k3_base;
        else, k3_base=0; k3_taper=[0;0;0]; end
        [max_c2, idx_c2] = max(abs(c2_vals(1:3)));
        if max_c2 > 1e-12, c2_base = c2_vals(idx_c2); c2_taper = c2_vals(1:3)/c2_base;
        else, c2_base=0; c2_taper=[0;0;0]; end
    else
        k3_base=0; c2_base=0; k3_taper=[0;0;0]; c2_taper=[0;0;0];
    end
end

function level = determineBranchLevel(branchName)
    underscores = strfind(branchName, '_');
    if isempty(underscores)
        level = 1;
    elseif length(underscores) == 1
        level = 2;
    else
        level = 3; 
    end
end

function attach = shouldAttachFruit(branchName, level, fruitConfig)
    if level == 1
        attach = false;
    elseif level == 2
        attach = fruitConfig.attach_secondary_mid || fruitConfig.attach_secondary_tip;
    else 
        attach = fruitConfig.attach_tertiary_mid || fruitConfig.attach_tertiary_tip;
    end
end

function attach = shouldAttachAtPosition(level, position, fruitConfig)
    if level == 2
        if strcmp(position, 'mid')
            attach = fruitConfig.attach_secondary_mid; 
        else
            attach = fruitConfig.attach_secondary_tip;
        end
    elseif level == 3
        if strcmp(position, 'mid')
            attach = fruitConfig.attach_tertiary_mid;
        else
            attach = fruitConfig.attach_tertiary_tip; 
        end
    else
        attach = false; 
    end
end

function validateBranchGeometry(branchGeom, branchName)
    if ~isfield(branchGeom, 'mass_dist') || length(branchGeom.mass_dist)~=3
        error('ConfigAdapter:InvalidData', '分枝 %s 的mass_dist无效', branchName);
    end
end

function fruit_params = buildFruitParamsStrict(preConfig, identifiedParams)
    fruit_params = struct();
    fruit_params.m = preConfig.fruit.mass;
    fruit_params.diameter = preConfig.fruit.diameter;
    k_val = 2000; c_val = 0.5;
    if isfield(preConfig.fruit, 'k_pedicel'), k_val = preConfig.fruit.k_pedicel; c_val = preConfig.fruit.c_pedicel; end
    fruit_params.k_pedicel_x = k_val; fruit_params.c_pedicel_x = c_val;
    fruit_params.k_pedicel_z = k_val; fruit_params.c_pedicel_z = c_val;
    fruit_params.F_break = 5.0; 
    
    det_model = [];
    if isfield(identifiedParams, 'branches')
        fn = fieldnames(identifiedParams.branches);
        for i = 1:length(fn)
            if isfield(identifiedParams.branches.(fn{i}), 'detachment_model')
                det_model = identifiedParams.branches.(fn{i}).detachment_model; break;
            end
        end
    elseif isfield(identifiedParams, 'detachment_model')
        det_model = identifiedParams.detachment_model;
    end
    
    if ~isempty(det_model)
        try
            pred = det_model.predict(1.5, 1.0, preConfig.fruit.diameter*100, 0);
            fruit_params.F_break = max(1.0, pred);
        catch
            fprintf('    [Warn] 脱落模型预测失败，使用默认值。\n');
        end
    end
end