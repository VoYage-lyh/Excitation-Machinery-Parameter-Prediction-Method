function [analysis_params, sim_params] = ConfigAdapter(preConfig, identifiedParams)
% ConfigAdapter_v2 - 参数适配器（修正版）
%
% 工作流程：
%   1. preConfig: 来自GUI的预配置（拓扑、几何、质量）
%   2. identifiedParams: 来自参数识别的结果（刚度、阻尼、非线性）- 可选
%
% 输出：
%   analysis_params: 用于参数识别代码的参数
%   sim_params: 用于仿真代码的参数（包含识别结果）

    if nargin < 2
        identifiedParams = [];
    end
    
    %% 1. 严格性检查：必须提供参数识别结果
    if isempty(identifiedParams)
        error('ConfigAdapter:NoIdentifiedParams', ...
              '错误：未提供 identifiedParams（参数识别结果）。\n' + ...
              '为了避免硬编码数值，仿真必须基于 analyse_chibi_data.m 的识别结果运行。');
    end
    
    %% 2. 生成参数识别所需参数 (保持不变)
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
    analysis_params.direction_labels = {'Y', 'Z'};
    analysis_params.topology = preConfig.topology;
    
    %% 3. 生成仿真所需参数
    sim_params = struct();
    
    % 基础设置
    sim_params.parallel_execution_max_workers = preConfig.basic.parallel_max_workers;
    sim_params.workFolder = preConfig.basic.workFolder;
    sim_params.model_name = preConfig.basic.modelName;
    sim_params.gravity_g = preConfig.basic.gravity_g;
    sim_params.use_parallel = preConfig.basic.useParallel;
    sim_params.config = preConfig.topology;
    
    % --- 构建主干参数 (严格依赖识别结果) ---
    sim_params.trunk = buildTrunkParams(preConfig, identifiedParams);
    
    % --- 构建分枝参数 (严格依赖识别结果 + 激活完整性验证) ---
    sim_params.predefined_params = generatePredefinedParams(preConfig, identifiedParams);
    
    % --- 构建果实配置 ---
    sim_params.fruit_config = struct();
    sim_params.fruit_config.attach_secondary_mid = preConfig.fruit.attach_secondary_mid;
    sim_params.fruit_config.attach_secondary_tip = preConfig.fruit.attach_secondary_tip;
    sim_params.fruit_config.attach_tertiary_mid = preConfig.fruit.attach_tertiary_mid;
    sim_params.fruit_config.attach_tertiary_tip = preConfig.fruit.attach_tertiary_tip;
    
    % [修复] 使用 GUI 配置的每节点果实数量，移除硬编码
    if isfield(preConfig.fruit, 'fruits_per_node')
        sim_params.fruit_config.fruits_per_node = preConfig.fruit.fruits_per_node;
    else
        sim_params.fruit_config.fruits_per_node = 1;
    end
    
    % 生成默认果实参数 (用于未特定指明位置的果实)
    sim_params.default_fruit_params = buildFruitParamsStrict(preConfig, identifiedParams);
    
    % 激励参数
    sim_params.excitation = preConfig.excitation;
    
    % [修复] 更新激励频率为第一阶固有频率 (优先从Trunk获取，适配聚合结构体)
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
    
    % 兼容旧版单一结构体
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
    
    fprintf('ConfigAdapter: 参数适配完成。模型参数已基于实验数据生成。\n');
end

%% ==================== 构建主干参数 ====================
%% buildTrunkParams (修改版 - 支持非线性参数提取)
function trunk = buildTrunkParams(preConfig, identifiedParams)
    trunk = struct();
    
    % --- 1. 几何与质量 (保持不变) ---
    trunk.length = preConfig.trunk.length;
    trunk.diameter_base = preConfig.trunk.diameter_base;
    trunk.diameter_tip = preConfig.trunk.diameter_tip;
    
    m_total = preConfig.trunk.total_mass;
    m_dist = preConfig.trunk.mass_distribution;
    trunk.root.m = m_total * m_dist(1);
    trunk.mid.m = m_total * m_dist(2);
    trunk.tip.m = m_total * m_dist(3);
    
    % --- 2. 刚度和阻尼 (线性 + 非线性) ---
    
    % A. 定位参数源 (Linear & Nonlinear)
    target_lin = [];
    target_nonlin = [];
    
    % 尝试从 branches.Trunk 结构获取
    if isfield(identifiedParams, 'branches') && isfield(identifiedParams.branches, 'Trunk')
        branch_data = identifiedParams.branches.Trunk;
        if isfield(branch_data, 'linear'), target_lin = branch_data.linear; end
        if isfield(branch_data, 'nonlinear'), target_nonlin = branch_data.nonlinear; end
        fprintf('    [ConfigAdapter] 主干：加载 Trunk 分枝识别参数。\n');
    % 尝试从全局结构获取 (兼容旧版)
    elseif isfield(identifiedParams, 'linear')
        target_lin = identifiedParams.linear;
        if isfield(identifiedParams, 'nonlinear'), target_nonlin = identifiedParams.nonlinear; end
        fprintf('    [ConfigAdapter] 主干：加载全局参数。\n');
    else
        error('ConfigAdapter:NoTrunkParams', ...
              '错误：未在 identifiedParams 中找到主干(Trunk)或全局(linear)参数。');
    end
    
    % B. 提取线性参数 (Y方向)
    if ~isfield(target_lin, 'identified_params_x') || isempty(target_lin.identified_params_x) || length(target_lin.identified_params_x) < 6
        error('ConfigAdapter:PhysicalParamsMissingY', ...
              '严重错误：参数识别结果中缺少有效的 Y 方向物理参数向量 "identified_params_x"。');
    end
    p_vec_y = target_lin.identified_params_x;
    
    % 线性赋值 (Y)
    trunk.root.k_y_conn_to_base = p_vec_y(1); % k_g
    trunk.root.c_y_conn_to_base = p_vec_y(2); % c_g
    
    trunk.root.k_y_conn = p_vec_y(3); % k_rm (连接 Root-Mid)
    trunk.mid.k_y_conn  = p_vec_y(5); % k_mt (连接 Mid-Tip)
    trunk.tip.k_y_conn  = 0;        
    
    trunk.root.c_y_conn = p_vec_y(4);
    trunk.mid.c_y_conn  = p_vec_y(6);
    trunk.tip.c_y_conn  = 0;

    fprintf('    [ConfigAdapter] 主干 Y 线性参数应用: k_g=%.1f, k_rm=%.1f。\n', p_vec_y(1), p_vec_y(3));

    % C. [新增] 提取非线性参数 (Y方向)
    % 假设非线性系数向量 k3_coeffs, c2_coeffs 的顺序为 [Root, Mid, Tip]
    % 注意：build_branch_recursively 中，root.k3 对应的是 Root 与 Parent 的连接 (k_g)，
    % 而 root.k3_conn 对应的是 Root 与 Mid 的连接。
    % 这里需要根据 SAD 模型定义进行映射。通常 k3_coeffs(1) 对应 k_g 的非线性修正。
    
    % 初始化默认值 (线性)
    trunk.root.k3_y_conn_to_base = 0; trunk.root.c2_y_conn_to_base = 0;
    trunk.root.k3_y_conn = 0;         trunk.root.c2_y_conn = 0;
    trunk.mid.k3_y_conn  = 0;         trunk.mid.c2_y_conn  = 0;
    
    if ~isempty(target_nonlin) && isfield(target_nonlin, 'k3_coeffs') && isfield(target_nonlin, 'c2_coeffs')
        k3 = target_nonlin.k3_coeffs;
        c2 = target_nonlin.c2_coeffs;
        
        % 映射逻辑 (假设索引 1:Root节点(对地), 2:Mid节点(Root-Mid连接), 3:Tip节点(Mid-Tip连接))
        if length(k3) >= 1
            trunk.root.k3_y_conn_to_base = k3(1); % Root对地非线性
            trunk.root.c2_y_conn_to_base = c2(1);
        end
        if length(k3) >= 2
            trunk.root.k3_y_conn = k3(2); % Root-Mid连接非线性 (存储在Root上)
            trunk.root.c2_y_conn = c2(2);
        end
        if length(k3) >= 3
            trunk.mid.k3_y_conn = k3(3);  % Mid-Tip连接非线性 (存储在Mid上)
            trunk.mid.c2_y_conn = c2(3);
        end
        fprintf('    [ConfigAdapter] 主干 Y 非线性参数已应用 (k3_root=%.2e)。\n', trunk.root.k3_y_conn_to_base);
    end

    % D. 提取 Z 方向参数 (线性 + 非线性)
    if isfield(target_lin, 'identified_params_z') && length(target_lin.identified_params_z) >= 6
        p_vec_z = target_lin.identified_params_z;
        
        % 计算 Z-factor (仅供参考)
        if p_vec_y(1) ~= 0
            trunk.z_factor = p_vec_z(1) / p_vec_y(1); 
        else
            trunk.z_factor = 1.0; 
        end

        % 线性赋值 (Z)
        trunk.root.k_z_conn_to_base = p_vec_z(1);
        trunk.root.c_z_conn_to_base = p_vec_z(2);
        
        trunk.root.k_z_conn = p_vec_z(3);
        trunk.mid.k_z_conn  = p_vec_z(5);
        trunk.tip.k_conn_z  = 0;
        
        trunk.root.c_z_conn = p_vec_z(4);
        trunk.mid.c_z_conn  = p_vec_z(6);
        trunk.tip.c_z_conn  = 0;
        
        % [新增] 非线性赋值 (Z)
        % 假设 Z 方向非线性参数结构类似，或者暂时假设各向同性/无非线性
        % 如果识别结果中有独立的 Z 向非线性参数 (e.g. k3_coeffs_z)，应在此处提取
        % 目前 analyse_chibi_data.m 似乎只输出了一组 k3_coeffs，通常基于主振动方向(X/Y)
        % 这里我们可以选择：
        % 1. 忽略 Z 向非线性 (设为0)
        % 2. 应用各向同性假设 (使用 Y 向参数)
        % 3. 应用 z_factor 缩放
        
        % 策略：使用 z_factor 缩放 Y 向非线性参数 (简化的物理假设)
        z_nl_factor = trunk.z_factor; % 或者使用 sqrt(trunk.z_factor)
        
        trunk.root.k3_z_conn_to_base = trunk.root.k3_y_conn_to_base * z_nl_factor;
        trunk.root.c2_z_conn_to_base = trunk.root.c2_y_conn_to_base * z_nl_factor;
        
        trunk.root.k3_z_conn = trunk.root.k3_y_conn * z_nl_factor;
        trunk.root.c2_z_conn = trunk.root.c2_y_conn * z_nl_factor;
        
        trunk.mid.k3_z_conn = trunk.mid.k3_y_conn * z_nl_factor;
        trunk.mid.c2_z_conn = trunk.mid.c2_y_conn * z_nl_factor;
        
    else
        error('ConfigAdapter:NoZParams', ...
              '严重错误：缺少 Z 方向识别参数 (identified_params_z)。');
    end
end

%% ==================== 验证预定义参数完整性 ====================
%% validatePredefinedParams (完整修改版 - 增加非线性参数完整性检查)
% 功能: 验证生成的预定义参数库是否包含所有必需的分枝，以及分枝参数是否完整。
%       新增对非线性参数字段的检查。
%
% 输入:
%   predefined: 生成好的参数结构体
%   preConfig: 预配置结构体 (用于对照拓扑)
function validatePredefinedParams(predefined, preConfig)
    % 验证所有必需的分枝参数是否都已生成
    
    missingBranches = {};
    incompleteParams = {}; % [新增] 记录参数不完整的分枝
    
    % --- 1. 检查一级分枝 ---
    num_p = preConfig.topology.num_primary_branches;
    for p = 1:num_p
        branch_id = sprintf('P%d', p);
        if ~isfield(predefined, branch_id)
            missingBranches{end+1} = branch_id;
        else
            % [新增] 检查参数完整性
            if ~checkBranchParamIntegrity(predefined.(branch_id))
                incompleteParams{end+1} = branch_id;
            end
        end
    end
    
    % --- 2. 检查二级分枝 ---
    for p = 1:num_p
        num_s = preConfig.topology.secondary_branches_count(p);
        for s = 1:num_s
            branch_id = sprintf('P%d_S%d', p, s);
            if ~isfield(predefined, branch_id)
                missingBranches{end+1} = branch_id;
            else
                if ~checkBranchParamIntegrity(predefined.(branch_id))
                    incompleteParams{end+1} = branch_id;
                end
            end
        end
    end
    
    % --- 3. 检查三级分枝 ---
    for p = 1:num_p
        if p <= length(preConfig.topology.tertiary_branches_count)
            tertiary_for_p = preConfig.topology.tertiary_branches_count{p};
            num_s = preConfig.topology.secondary_branches_count(p);
            for s = 1:num_s
                if s <= length(tertiary_for_p)
                    num_t = tertiary_for_p(s);
                    for t = 1:num_t
                        branch_id = sprintf('P%d_S%d_T%d', p, s, t);
                        if ~isfield(predefined, branch_id)
                            missingBranches{end+1} = branch_id;
                        else
                            if ~checkBranchParamIntegrity(predefined.(branch_id))
                                incompleteParams{end+1} = branch_id;
                            end
                        end
                    end
                end
            end
        end
    end
    
    % --- 4. 报告错误 ---
    if ~isempty(missingBranches)
        error('ConfigAdapter:MissingData', ...
              '以下分枝在预配置中缺少几何参数，请在GUI中完整配置:\n  %s', ...
              strjoin(missingBranches, ', '));
    end
    
    % [新增] 报告参数不完整 (通常意味着 generateBranchSegmentParams 未正确执行)
    if ~isempty(incompleteParams)
        warning('ConfigAdapter:IncompleteParams', ...
                ['以下分枝的物理参数不完整 (可能缺少非线性 k3/c2 字段):\n  %s\n' ...
                 '虽然仿真可能继续运行 (默认为0)，但这表明参数生成过程可能存在问题。'], ...
                strjoin(incompleteParams, ', '));
    end
end

% [新增] 内部辅助函数：检查单个分枝的参数字段
function is_ok = checkBranchParamIntegrity(branch_data)
    is_ok = true;
    segments = {'root', 'mid', 'tip'};
    % 检查的关键字段 (包含新增的非线性项)
    required_fields = {'m', 'k_y_conn', 'c_y_conn', 'k3_y_conn', 'c2_y_conn'};
    
    for i = 1:length(segments)
        seg = segments{i};
        if isfield(branch_data, seg)
            seg_data = branch_data.(seg);
            for k = 1:length(required_fields)
                if ~isfield(seg_data, required_fields{k})
                    is_ok = false;
                    return;
                end
            end
        else
            is_ok = false; % 缺少 root/mid/tip 段
            return;
        end
    end
end

%% ==================== 为分枝生成默认段参数 ====================
%% generatePredefinedParams (完整修改版 - 同步非线性参数流)
% 功能: 遍历所有分枝配置，调用子函数生成包含线性和非线性特征的完整物理参数。
%       此函数是连接 "参数识别结果" 与 "仿真模型参数" 的核心调度器。
%
% 输入:
%   preConfig: 预配置结构体 (包含拓扑、几何、果实配置)
%   identifiedParams: 识别参数结构体 (包含实验识别的刚度、阻尼、非线性系数)
% 输出:
%   predefined: 分枝参数库结构体
function predefined = generatePredefinedParams(preConfig, identifiedParams)
    
    % 基础检查
    if isempty(preConfig) || isempty(identifiedParams)
        error('ConfigAdapter:MissingData', '缺少配置或识别参数');
    end
    
    predefined = struct();
    fruitConfig = preConfig.fruit;
    
    % --- 定义内部辅助函数：处理单个分枝的通用逻辑 ---
    function processBranch(name, type_struct)
        % 1. 确定分枝等级 (1=Primary, 2=Secondary, 3=Tertiary)
        lvl = determineBranchLevel(name);
        
        % 2. 获取几何参数 (来自GUI预配置)
        if ~isfield(type_struct, name)
            error('缺少分枝 %s 的几何参数', name);
        end
        geom = type_struct.(name);
        validateBranchGeometry(geom, name);
        
        % 3. [关键步骤] 获取基准刚度阻尼及非线性参数
        % 调用更新后的 estimateStiffnessDamping，获取4个基准值和分布因子
        [k_b, c_b, k3_b, c2_b, specific_taper] = estimateStiffnessDamping(geom, identifiedParams, name);
        
        % 4. [关键步骤] 生成分段参数 (Root/Mid/Tip)
        % 将线性和非线性基准值传递给生成函数
        predefined.(name) = generateBranchSegmentParams(geom, k_b, c_b, k3_b, c2_b, specific_taper);
        
        % 标记分枝等级
        predefined.(name).branch_level = lvl;
        
        % 5. 挂果逻辑 (根据配置决定是否添加果实参数)
        % 注意: 果实参数生成函数 buildFruitParamsStrict 也已支持严格模式
        if shouldAttachFruit(name, lvl, fruitConfig)
            if shouldAttachAtPosition(lvl, 'mid', fruitConfig)
                predefined.(name).fruit_at_mid = buildFruitParamsStrict(preConfig, identifiedParams);
            end
            if shouldAttachAtPosition(lvl, 'tip', fruitConfig)
                predefined.(name).fruit_at_tip = buildFruitParamsStrict(preConfig, identifiedParams);
            end
        end
    end

    % --- 遍历生成所有分枝参数 ---
    
    % 1. 一级分枝 (Primary)
    fprintf('    [ConfigAdapter] 正在生成一级分枝参数...\n');
    for p = 1:preConfig.topology.num_primary_branches
        branch_name = sprintf('P%d', p);
        processBranch(branch_name, preConfig.primary);
    end
    
    % 2. 二级分枝 (Secondary)
    fprintf('    [ConfigAdapter] 正在生成二级分枝参数...\n');
    for p = 1:preConfig.topology.num_primary_branches
        for s = 1:preConfig.topology.secondary_branches_count(p)
            branch_name = sprintf('P%d_S%d', p, s);
            processBranch(branch_name, preConfig.secondary);
        end
    end
    
    % 3. 三级分枝 (Tertiary)
    fprintf('    [ConfigAdapter] 正在生成三级分枝参数...\n');
    for p = 1:preConfig.topology.num_primary_branches
        if p <= length(preConfig.topology.tertiary_branches_count)
            tertiary_counts = preConfig.topology.tertiary_branches_count{p};
            % 注意 tertiary_branches_count 可能是 cell array 也可能是 vector，需根据实际格式处理
            % 这里假设经过 GUI 验证后是标准的数值数组或 cell 元素
            for s = 1:length(tertiary_counts)
                for t = 1:tertiary_counts(s)
                    branch_name = sprintf('P%d_S%d_T%d', p, s, t);
                    processBranch(branch_name, preConfig.tertiary);
                end
            end
        end
    end

    % === 完整性验证 ===
    validatePredefinedParams(predefined, preConfig);
    
    fprintf('  分枝参数库生成完成 (已包含非线性特征)。\n');
end

%% ==================== 生成单个分枝的段参数 ====================
%% generateBranchSegmentParams (完整修改版 - 支持非线性生成)
% 功能: 根据基准值和分布因子，生成分枝各段(Root/Mid/Tip)的具体物理参数
%
% 输入:
%   branchGeom: 几何参数
%   k_base, c_base: 线性基准值
%   k3_base, c2_base: 非线性基准值 [新增]
%   identified_taper: 分布因子结构体 (含 k, c, k3, c2)
function params = generateBranchSegmentParams(branchGeom, k_base, c_base, k3_base, c2_base, identified_taper)
    
    % --- 验证递减因子 ---
    if nargin < 6 || isempty(identified_taper)
        error('ConfigAdapter:MissingTaper', '缺少分布因子 identified_taper。');
    end
    
    required_fields = {'k', 'c', 'k3', 'c2'};
    for i = 1:length(required_fields)
        if ~isfield(identified_taper, required_fields{i})
            % 如果缺少非线性taper，尝试补全为0
            if strcmp(required_fields{i}, 'k3') || strcmp(required_fields{i}, 'c2')
                identified_taper.(required_fields{i}) = [0; 0; 0];
            else
                error('ConfigAdapter:InvalidTaper', 'identified_taper 缺少字段 %s', required_fields{i});
            end
        end
    end
    
    k_taper = identified_taper.k;
    c_taper = identified_taper.c;
    k3_taper = identified_taper.k3;
    c2_taper = identified_taper.c2;
    
    params = struct();
    
    % --- 提取几何配置 ---
    if ~isfield(branchGeom, 'z_factor'), error('分枝几何缺少 z_factor'); end
    z_factor = branchGeom.z_factor;
    
    if ~isfield(branchGeom, 'mass_dist'), error('分枝几何缺少 mass_dist'); end
    m_dist = branchGeom.mass_dist;
    
    if ~isfield(branchGeom, 'total_mass'), error('分枝几何缺少 total_mass'); end
    m_total = branchGeom.total_mass;
    
    % --- 生成各段参数 (Root, Mid, Tip) ---
    
    % 1. Root段
    params.root.m = m_total * m_dist(1);
    % Y方向 (主方向)
    params.root.k_y_conn = k_base * k_taper(1);
    params.root.c_y_conn = c_base * c_taper(1);
    params.root.k3_y_conn = k3_base * k3_taper(1); % [新增]
    params.root.c2_y_conn = c2_base * c2_taper(1); % [新增]
    % Z方向 (应用 z_factor)
    params.root.k_z_conn = params.root.k_y_conn * z_factor;
    params.root.c_z_conn = params.root.c_y_conn * z_factor;
    params.root.k3_z_conn = params.root.k3_y_conn * z_factor; % [新增] 假设非线性各项异性与线性一致
    params.root.c2_z_conn = params.root.c2_y_conn * z_factor; % [新增]
    
    % 2. Mid段
    params.mid.m = m_total * m_dist(2);
    % Y方向
    params.mid.k_y_conn = k_base * k_taper(2);
    params.mid.c_y_conn = c_base * c_taper(2);
    params.mid.k3_y_conn = k3_base * k3_taper(2); % [新增]
    params.mid.c2_y_conn = c2_base * c2_taper(2); % [新增]
    % Z方向
    params.mid.k_z_conn = params.mid.k_y_conn * z_factor;
    params.mid.c_z_conn = params.mid.c_y_conn * z_factor;
    params.mid.k3_z_conn = params.mid.k3_y_conn * z_factor; % [新增]
    params.mid.c2_z_conn = params.mid.c2_y_conn * z_factor; % [新增]
    
    % 3. Tip段
    params.tip.m = m_total * m_dist(3);
    % Y方向
    params.tip.k_y_conn = k_base * k_taper(3);
    params.tip.c_y_conn = c_base * c_taper(3);
    params.tip.k3_y_conn = k3_base * k3_taper(3); % [新增]
    params.tip.c2_y_conn = c2_base * c2_taper(3); % [新增]
    % Z方向
    params.tip.k_z_conn = params.tip.k_y_conn * z_factor;
    params.tip.c_z_conn = params.tip.c_y_conn * z_factor;
    params.tip.k3_z_conn = params.tip.k3_y_conn * z_factor; % [新增]
    params.tip.c2_z_conn = params.tip.c2_y_conn * z_factor; % [新增]
    
    % --- 保存几何信息 ---
    params.geometry.length = branchGeom.length;
    params.geometry.diameter_base = branchGeom.diameter_base;
    params.geometry.diameter_tip = branchGeom.diameter_tip;
    
    % 保存因子供参考
    params.taper_factors = identified_taper;
end

%% ==================== 辅助函数 ====================
%% estimateStiffnessDamping (完整修改版 - 支持非线性提取)
% 功能: 获取分枝的刚度、阻尼及非线性参数的基准值和分布因子
%       从实验识别结果中提取，并进行归一化处理
%
% 输出:
%   k_base, c_base: 线性基准值
%   k3_base, c2_base: 非线性基准值 [新增]
%   branch_taper: 包含 k, c, k3, c2 分布因子的结构体
function [k_base, c_base, k3_base, c2_base, branch_taper] = estimateStiffnessDamping(branchGeom, identifiedParams, branchName)
    
    target_linear_params = [];
    target_nonlinear_params = [];
    
    % 1. 定位参数源 (Linear & Nonlinear)
    % 优先查找 branches.(branchName)
    if isfield(identifiedParams, 'branches') && isfield(identifiedParams.branches, branchName)
        branch_data = identifiedParams.branches.(branchName);
        if isfield(branch_data, 'linear'), target_linear_params = branch_data.linear; end
        if isfield(branch_data, 'nonlinear'), target_nonlinear_params = branch_data.nonlinear; end
    % 回退全局
    elseif isfield(identifiedParams, 'linear')
        target_linear_params = identifiedParams.linear;
        if isfield(identifiedParams, 'nonlinear'), target_nonlinear_params = identifiedParams.nonlinear; end
    end
    
    % 2. 线性参数处理 (严格校验)
    if isempty(target_linear_params) || ~isfield(target_linear_params, 'K') || ~isfield(target_linear_params, 'C')
        error('ConfigAdapter:MissingExperimentData', ...
              '严重错误：分枝 "%s" 缺少线性识别数据 (K/C矩阵)。', branchName);
    end
    
    % 提取对角线元素 (假设对应 Root/Mid/Tip)
    K_vals = diag(target_linear_params.K); 
    C_vals = diag(target_linear_params.C);
    
    % 计算线性基准值 (Base) 和 递减因子 (Taper)
    k_base = max(K_vals);
    if k_base <= 0, error('ConfigAdapter:BadData', '分枝 "%s" 线性刚度无效。', branchName); end
    k_taper = K_vals / k_base; 
    
    c_base = max(C_vals);
    if c_base <= 0, c_base = 1e-3; warning('分枝 "%s" 阻尼异常，使用默认小值。', branchName); end
    c_taper = C_vals / c_base;
    
    % 3. [新增] 非线性参数处理
    k3_base = 0;
    c2_base = 0;
    k3_taper = [0; 0; 0];
    c2_taper = [0; 0; 0];
    
    if ~isempty(target_nonlinear_params) && ...
       isfield(target_nonlinear_params, 'k3_coeffs') && ...
       isfield(target_nonlinear_params, 'c2_coeffs')
   
        k3_vals = target_nonlinear_params.k3_coeffs(:); % 确保列向量
        c2_vals = target_nonlinear_params.c2_coeffs(:);
        
        % 补齐数据长度 (防止识别结果少于3个节点)
        if length(k3_vals) < 3, k3_vals(end+1:3) = 0; end
        if length(c2_vals) < 3, c2_vals(end+1:3) = 0; end
        k3_vals = k3_vals(1:3);
        c2_vals = c2_vals(1:3);
        
        % 计算非线性基准值 (取绝对值最大者，保留符号信息在 taper 中)
        [max_k3_abs, idx_k3] = max(abs(k3_vals));
        if max_k3_abs > 1e-12
            k3_base = k3_vals(idx_k3); % 基准值带符号
            k3_taper = k3_vals / k3_base;
        else
            k3_base = 0;
            k3_taper = [0; 0; 0];
        end
        
        [max_c2_abs, idx_c2] = max(abs(c2_vals));
        if max_c2_abs > 1e-12
            c2_base = c2_vals(idx_c2);
            c2_taper = c2_vals / c2_base;
        else
            c2_base = 0;
            c2_taper = [0; 0; 0];
        end
        
        fprintf('    [√] 分枝 %s 参数: k_lin=%.1f, k3_nl=%.2e\n', branchName, k_base, k3_base);
    else
        fprintf('    [!] 分枝 %s 无非线性数据，使用线性模型。\n', branchName);
    end
    
    % 4. 打包输出
    branch_taper = struct();
    branch_taper.k = k_taper;
    branch_taper.c = c_taper;
    branch_taper.k3 = k3_taper; % [新增]
    branch_taper.c2 = c2_taper; % [新增]
end

function level = determineBranchLevel(branchName)
    % 根据分枝名称确定层级
    % P1, P2, P3 -> 1 (一级)
    % P1_S1, P2_S1 -> 2 (二级)
    % P1_S1_T1 -> 3 (三级)
    
    underscores = strfind(branchName, '_');
    if isempty(underscores)
        level = 1;  % 一级分枝 (P1, P2, ...)
    elseif length(underscores) == 1
        level = 2;  % 二级分枝 (P1_S1, ...)
    else
        level = 3;  % 三级分枝 (P1_S1_T1, ...)
    end
end

function attach = shouldAttachFruit(branchName, level, fruitConfig)
    % 根据配置确定是否需要挂果
    % 一级分枝不直接挂果
    % 二级和三级根据配置决定
    
    if level == 1
        attach = false;
    elseif level == 2
        attach = fruitConfig.attach_secondary_mid || fruitConfig.attach_secondary_tip;
    else  % level == 3
        attach = fruitConfig.attach_tertiary_mid || fruitConfig.attach_tertiary_tip;
    end
end

function attach = shouldAttachAtPosition(level, position, fruitConfig)
    % 确定特定位置是否挂果
    
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
    % 验证分枝几何参数完整性 - 严格模式
    
    required_fields = {'total_mass', 'length', 'diameter_base', 'diameter_tip', 'mass_dist'};
    
    for i = 1:length(required_fields)
        field = required_fields{i};
        if ~isfield(branchGeom, field)
            error('ConfigAdapter:MissingData', ...
                  '分枝 %s 几何参数缺少字段: %s，请在GUI中完整配置', branchName, field);
        end
        
        value = branchGeom.(field);
        
        if strcmp(field, 'mass_dist')
            if length(value) ~= 3
                error('ConfigAdapter:InvalidData', ...
                      '分枝 %s 的mass_dist必须是长度为3的向量 [root, mid, tip]', branchName);
            end
            if abs(sum(value) - 1) > 0.01
                error('ConfigAdapter:InvalidData', ...
                      '分枝 %s 的mass_dist之和必须为1，当前为%.4f', branchName, sum(value));
            end
        else
            if isempty(value) || (isnumeric(value) && value <= 0)
                error('ConfigAdapter:InvalidData', ...
                      '分枝 %s 的 %s 值无效(空或非正)', branchName, field);
            end
        end
    end
end

function fruit_params = buildFruitParamsStrict(preConfig, identifiedParams)
    % 功能：基于识别出的统计模型生成果实参数
    % 核心原则：严禁使用预设的固定值 (如 F=5N)，必须由模型预测得出
    
    fruit_params = struct();
    
    % 1. 物理属性 (来自 GUI 预配置，这些是几何输入)
    fruit_params.m = preConfig.fruit.mass;
    fruit_params.diameter = preConfig.fruit.diameter;
    % 检查 preConfig 是否包含新字段（兼容旧版配置）
    if isfield(preConfig.fruit, 'k_pedicel')
        k_val = preConfig.fruit.k_pedicel;
        c_val = preConfig.fruit.c_pedicel;
    else
        warning('ConfigAdapter:LegacyConfig', '预配置中缺少果柄动力学参数，使用默认值。请更新 GUI 配置。');
        k_val = 2000;
        c_val = 0.5;
    end
    % 应用到 Y/Z 两个方向 (假设各向同性，或者也可以在GUI分开配置)
    fruit_params.k_pedicel_y = k_val;
    fruit_params.c_pedicel_y = c_val;
    fruit_params.k_pedicel_z = k_val;
    fruit_params.c_pedicel_z = c_val;

    % 2. 核心：计算断裂力 F_break
    % 需要从 identifiedParams 中提取脱落模型
    % 假设所有分枝共享同一个脱落机制模型，我们取第一个有效分枝的模型即可
    det_model = [];
    % [修复] 适配聚合结构体
    if isfield(identifiedParams, 'branches')
        % 遍历寻找含有 detachment_model 的分枝
        fn = fieldnames(identifiedParams.branches);
        for i = 1:length(fn)
            if isfield(identifiedParams.branches.(fn{i}), 'detachment_model')
                det_model = identifiedParams.branches.(fn{i}).detachment_model;
                break;
            end
        end
    elseif isfield(identifiedParams, 'detachment_model')
        det_model = identifiedParams.detachment_model;
    end
    
    if ~isempty(det_model)
        % 提取 GUI 中配置的平均特征作为输入
        % 注意：这里将直径从 m 转换为 cm 以匹配标定数据的单位
        D_input_cm = (preConfig.fruit.diameter * 100); 
        
        % 假设场景：中等高度，末端挂果，无开裂
        H_input_m = 1.5; 
        P_input_idx = 1.0; % 末端
        S_input_crack = 0; % 无开裂
        
        % 调用模型的预测接口
        predicted_F = det_model.predict(H_input_m, P_input_idx, D_input_cm, S_input_crack);
        
        % 加上随机波动 (模拟果实间的差异，基于识别出的误差 sigma)
        % 如果希望确定性仿真，去掉 randn 部分
        final_F_break = predicted_F + det_model.sigma_epsilon * randn();
        
        fruit_params.F_break = max(1.0, final_F_break); % 确保力为正值
        
        fprintf('    [ConfigAdapter] 果实断裂力 F_break 已由数据模型计算: %.2f N (模型预测值)\n', fruit_params.F_break);
    else
        % 如果没有识别结果，为了不让程序崩溃，报错提示
        error('ConfigAdapter:NoDetachmentModel', ...
              '未找到果实脱落力模型。请确保在参数识别阶段成功运行了 Stage 4 (基于内置的20组数据)。');
    end
end