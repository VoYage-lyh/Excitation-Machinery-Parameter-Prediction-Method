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
%
% 使用示例：
%   % 第一阶段：仅用于参数识别
%   [analysis_params, ~] = ConfigAdapter_v2(preConfig, []);
%
%   % 第二阶段：用于仿真（带识别结果）
%   [~, sim_params] = ConfigAdapter_v2(preConfig, identifiedParams);

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
    sim_params.fruit_config.fruits_per_node = 1;
    
    % 生成默认果实参数 (用于未特定指明位置的果实)
    sim_params.default_fruit_params = buildFruitParamsStrict(preConfig, identifiedParams);
    
    % 激励参数
    sim_params.excitation = preConfig.excitation;
    
    % 更新激励频率为第一阶固有频率
    if isfield(identifiedParams, 'linear') && ...
       isfield(identifiedParams.linear, 'natural_freqs_x') && ...
       ~isempty(identifiedParams.linear.natural_freqs_x)
        sim_params.excitation.frequency_hz = identifiedParams.linear.natural_freqs_x(1);
    end

    % 仿真控制
    sim_params.sim_stop_time = preConfig.simulation.stop_time;
    sim_params.sim_fixed_step = preConfig.simulation.fixed_step;
    sim_params.has_identified_params = true;
    
    fprintf('ConfigAdapter: 参数适配完成。模型参数已基于实验数据生成。\n');
end

%% ==================== 构建主干参数 ====================
function trunk = buildTrunkParams(preConfig, identifiedParams)
    trunk = struct();
    
    % 几何与质量 (来自 GUI)
    trunk.length = preConfig.trunk.length;
    trunk.diameter_base = preConfig.trunk.diameter_base;
    trunk.diameter_tip = preConfig.trunk.diameter_tip;
    trunk.z_factor = preConfig.trunk.z_factor;
    
    m_total = preConfig.trunk.total_mass;
    m_dist = preConfig.trunk.mass_distribution;
    trunk.root.m = m_total * m_dist(1);
    trunk.mid.m = m_total * m_dist(2);
    trunk.tip.m = m_total * m_dist(3);
    
    % 刚度和阻尼 (严格来自 实验识别)
    if ~isfield(identifiedParams, 'linear')
        error('ConfigAdapter:MissingData', '缺少线性识别参数(linear)，无法构建主干。');
    end
    
    K = identifiedParams.linear.K; % 识别到的刚度矩阵 (3x3)
    C = identifiedParams.linear.C; % 识别到的阻尼矩阵 (3x3)
    
    % 获取刚度递减趋势 (如果有的话，用于节点间微调，否则直接用矩阵对角元)
    % 这里假设 K 矩阵的对角元分别代表 Root, Mid, Tip 的等效刚度
    
    % Y方向 (主振动方向)
    trunk.root.k_y_conn_to_base = K(1,1); % 根部对地
    trunk.root.c_y_conn_to_base = C(1,1);
    
    trunk.root.k_y_conn = K(1,1); % Root节点连接刚度
    trunk.mid.k_y_conn  = K(2,2); % Mid节点连接刚度
    trunk.tip.k_y_conn  = K(3,3); % Tip节点连接刚度
    
    trunk.root.c_y_conn = C(1,1);
    trunk.mid.c_y_conn  = C(2,2);
    trunk.tip.c_y_conn  = C(3,3);
    
    % Z方向 (应用几何异向性因子 z_factor)
    z_fac = trunk.z_factor;
    
    trunk.root.k_z_conn_to_base = K(1,1) * z_fac;
    trunk.root.c_z_conn_to_base = C(1,1) * z_fac;
    
    trunk.root.k_z_conn = K(1,1) * z_fac;
    trunk.mid.k_z_conn  = K(2,2) * z_fac;
    trunk.tip.k_z_conn  = K(3,3) * z_fac;
    
    trunk.root.c_z_conn = C(1,1) * z_fac;
    trunk.mid.c_z_conn  = C(2,2) * z_fac;
    trunk.tip.c_z_conn  = C(3,3) * z_fac;
end


%% ==================== 验证预定义参数完整性 ====================
function validatePredefinedParams(predefined, preConfig)
    % 验证所有必需的分枝参数是否都已生成
    
    missingBranches = {};
    
    % 检查一级分枝
    num_p = preConfig.topology.num_primary_branches;
    for p = 1:num_p
        branch_id = sprintf('P%d', p);
        if ~isfield(predefined, branch_id)
            missingBranches{end+1} = branch_id;
        end
    end
    
    % 检查二级分枝
    for p = 1:num_p
        num_s = preConfig.topology.secondary_branches_count(p);
        for s = 1:num_s
            branch_id = sprintf('P%d_S%d', p, s);
            if ~isfield(predefined, branch_id)
                missingBranches{end+1} = branch_id;
            end
        end
    end
    
    % 检查三级分枝
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
                        end
                    end
                end
            end
        end
    end
    
    % 报告缺失的分枝
    if ~isempty(missingBranches)
        error('ConfigAdapter:MissingData', ...
              '以下分枝在预配置中缺少几何参数，请在GUI中完整配置:\n  %s', ...
              strjoin(missingBranches, ', '));
    end
end

%% ==================== 为分枝生成默认段参数 ====================
function predefined = generatePredefinedParams(preConfig, identifiedParams)
    % 生成预定义分枝参数
    % 包含：严格的数据依赖 + 激活完整性验证 + 几何参数校验
    
    % 基础检查
    if isempty(preConfig) || isempty(identifiedParams)
        error('ConfigAdapter:MissingData', '缺少配置或识别参数');
    end
    
    % 获取递减因子 (必须存在)
    if ~isfield(identifiedParams.linear, 'taper_factors')
        error('ConfigAdapter:MissingData', '识别结果中缺少 taper_factors (刚度递减因子)');
    end
    identified_taper = identifiedParams.linear.taper_factors;
    
    predefined = struct();
    fruitConfig = preConfig.fruit;
    
    % --- 定义内部辅助函数：处理单个分枝的通用逻辑 ---
    function processBranch(name, type_struct)
        % 1. 确定等级
        lvl = determineBranchLevel(name);
        
        % 2. 获取几何
        if ~isfield(type_struct, name)
            error('缺少分枝 %s 的几何参数', name);
        end
        geom = type_struct.(name);
        
        % === 修复点：在此处调用 validateBranchGeometry ===
        % 确保几何参数（如质量分布、直径）是合法的
        validateBranchGeometry(geom, name);
        
        % 3. 估算基准刚度 (从实验数据)
        [k_b, c_b] = estimateStiffnessDamping(geom, identifiedParams, name);
        
        % 4. 生成分段参数
        predefined.(name) = generateBranchSegmentParams(geom, k_b, c_b, identified_taper);
        predefined.(name).branch_level = lvl;
        
        % 5. 挂果逻辑 (统一使用 buildFruitParamsStrict)
        if shouldAttachFruit(name, lvl, fruitConfig)
            if shouldAttachAtPosition(lvl, 'mid', fruitConfig)
                predefined.(name).fruit_at_mid = buildFruitParamsStrict(preConfig, identifiedParams);
            end
            if shouldAttachAtPosition(lvl, 'tip', fruitConfig)
                predefined.(name).fruit_at_tip = buildFruitParamsStrict(preConfig, identifiedParams);
            end
        end
    end

    % --- 遍历生成 ---
    % 一级分枝
    fprintf('    正在处理一级分枝...\n');
    for p = 1:preConfig.topology.num_primary_branches
        processBranch(sprintf('P%d', p), preConfig.primary);
    end
    
    % 二级分枝
    fprintf('    正在处理二级分枝...\n');
    for p = 1:preConfig.topology.num_primary_branches
        for s = 1:preConfig.topology.secondary_branches_count(p)
            processBranch(sprintf('P%d_S%d', p, s), preConfig.secondary);
        end
    end
    
    % 三级分枝
    fprintf('    正在处理三级分枝...\n');
    for p = 1:preConfig.topology.num_primary_branches
        if p <= length(preConfig.topology.tertiary_branches_count)
            tertiary_counts = preConfig.topology.tertiary_branches_count{p};
            for s = 1:length(tertiary_counts)
                for t = 1:tertiary_counts(s)
                    processBranch(sprintf('P%d_S%d_T%d', p, s, t), preConfig.tertiary);
                end
            end
        end
    end

    % === 完整性验证 ===
    validatePredefinedParams(predefined, preConfig);
    
    fprintf('  分枝参数生成完成 (已通过几何校验与完整性验证)。\n');
end

%% ==================== 生成单个分枝的段参数 ====================
function params = generateBranchSegmentParams(branchGeom, k_base, c_base, identified_taper)
    % 生成单个分枝的段参数 - 严格模式
    % 递减因子必须从实验数据获取，无数据则报错
    % 输入:
    %   branchGeom - 分枝几何结构体（来自GUI配置）
    %   k_base - 基础刚度（从实验识别）
    %   c_base - 基础阻尼（从实验识别）
    %   identified_taper - 从实验识别的递减因子结构体（必须提供）
    
   % 验证递减因子必须存在且有效
    if nargin < 4 || isempty(identified_taper)
        error('ConfigAdapter:MissingTaperFactors', ...
              ['严重错误：生成分枝参数时缺少 "identified_taper" (刚度/阻尼递减因子)。\n' ...
               'ConfigAdapter 无法推断分枝沿长度方向的刚度变化。\n' ...
               '请检查 analyse_chibi_data.m 是否成功识别并输出了 taper_factors。']);
    end
    
    if ~isfield(identified_taper, 'k') || ~isfield(identified_taper, 'c') || ...
       length(identified_taper.k) ~= 3 || length(identified_taper.c) ~= 3
        error('ConfigAdapter:InvalidTaperFactors', ...
              ['严重错误：提供的 "identified_taper" 格式不正确。\n' ...
               '必须包含 "k" 和 "c" 字段，且每个字段必须是长度为 3 的向量 (对应 Root/Mid/Tip)。']);
    end
    
    if ~isstruct(identified_taper)
        error('ConfigAdapter:InvalidData', ...
              'identified_taper必须是结构体');
    end
    
    if ~isfield(identified_taper, 'k') || ~isfield(identified_taper, 'c')
        error('ConfigAdapter:MissingData', ...
              'identified_taper必须包含k和c字段（从实验FRF识别的刚度阻尼递减因子）');
    end
    
    if length(identified_taper.k) ~= 3 || length(identified_taper.c) ~= 3
        error('ConfigAdapter:InvalidData', ...
              '递减因子必须是长度为3的向量 [root, mid, tip]');
    end
    
    k_taper = identified_taper.k;
    c_taper = identified_taper.c;
    
    params = struct();
    
    % Z方向因子
    if ~isfield(branchGeom, 'z_factor')
        error('ConfigAdapter:MissingData', ...
              '分枝几何缺少z_factor，请在GUI中配置');
    end
    z_factor = branchGeom.z_factor;
    
    % 质量分配
    if ~isfield(branchGeom, 'mass_dist')
        error('ConfigAdapter:MissingData', ...
              '分枝几何缺少mass_dist（质量分配），请在GUI中配置');
    end
    m_dist = branchGeom.mass_dist;
    
    if ~isfield(branchGeom, 'total_mass')
        error('ConfigAdapter:MissingData', ...
              '分枝几何缺少total_mass，请在GUI中配置');
    end
    m_total = branchGeom.total_mass;
    
    % Root段
    params.root.m = m_total * m_dist(1);
    params.root.k_y_conn = k_base * k_taper(1);
    params.root.c_y_conn = c_base * c_taper(1);
    params.root.k_z_conn = k_base * k_taper(1) * z_factor;
    params.root.c_z_conn = c_base * c_taper(1) * z_factor;
    
    % Mid段
    params.mid.m = m_total * m_dist(2);
    params.mid.k_y_conn = k_base * k_taper(2);
    params.mid.c_y_conn = c_base * c_taper(2);
    params.mid.k_z_conn = k_base * k_taper(2) * z_factor;
    params.mid.c_z_conn = c_base * c_taper(2) * z_factor;
    
    % Tip段
    params.tip.m = m_total * m_dist(3);
    params.tip.k_y_conn = k_base * k_taper(3);
    params.tip.c_y_conn = c_base * c_taper(3);
    params.tip.k_z_conn = k_base * k_taper(3) * z_factor;
    params.tip.c_z_conn = c_base * c_taper(3) * z_factor;
    
    % 保存几何信息（必须有）
    if ~isfield(branchGeom, 'length')
        error('ConfigAdapter:MissingData', '分枝几何缺少length');
    end
    if ~isfield(branchGeom, 'diameter_base')
        error('ConfigAdapter:MissingData', '分枝几何缺少diameter_base');
    end
    if ~isfield(branchGeom, 'diameter_tip')
        error('ConfigAdapter:MissingData', '分枝几何缺少diameter_tip');
    end
    
    params.geometry.length = branchGeom.length;
    params.geometry.diameter_base = branchGeom.diameter_base;
    params.geometry.diameter_tip = branchGeom.diameter_tip;
    params.taper_factors.k = k_taper;
    params.taper_factors.c = c_taper;
end


%% ==================== 辅助函数 ====================
function [k_est, c_est] = estimateStiffnessDamping(branchGeom, identifiedParams, branchName)
    % 获取分枝刚度和阻尼 - 严格模式
    % 必须从实验识别结果获取，无数据则报错
    % 输入:
    %   branchGeom - 分枝几何（用于验证）
    %   identifiedParams - 参数识别结果（必须提供）
    %   branchName - 分枝名称（如 'P1', 'P1_S1'）
    
    % 验证识别参数必须存在
    if isempty(identifiedParams)
        error('ConfigAdapter:MissingData', ...
              '缺少参数识别结果(identifiedParams)，请先运行 analyse_chibi_data_v8.m 进行参数识别');
    end
    
    % 检查是否包含必要的线性参数和递减因子
    if ~isfield(identifiedParams, 'linear')
        error('ConfigAdapter:MissingLinearParams', '识别结果中缺少 "linear" 结构体。');
    end
    
    if ~isfield(identifiedParams.linear, 'taper_factors')
        error('ConfigAdapter:MissingTaperFactors', ...
              ['识别结果中缺少 "linear.taper_factors"。\n' ...
               '请确保你使用的是最新版的 "analyse_chibi_data.m"，它会计算并输出递减因子。']);
    end

    % 首先尝试从分枝特定的识别结果获取
    if isfield(identifiedParams, 'branches') && isfield(identifiedParams.branches, branchName)
        branch_params = identifiedParams.branches.(branchName);
        
        if ~isfield(branch_params, 'k_base') || ~isfield(branch_params, 'c_base')
            error('ConfigAdapter:MissingData', ...
                  '分枝 %s 的识别参数缺少 k_base 或 c_base', branchName);
        end
        
        k_est = branch_params.k_base;
        c_est = branch_params.c_base;
        return;
    end
    
    % 如果没有分枝特定参数，从全局线性参数计算
    if ~isfield(identifiedParams, 'linear')
        error('ConfigAdapter:MissingData', ...
              '识别参数中缺少 linear 字段，请检查参数识别是否成功完成');
    end
    
    linear = identifiedParams.linear;
    
    % 获取识别的刚度（使用X方向，因为更可靠）
    if ~isfield(linear, 'identified_params_x') || isempty(linear.identified_params_x)
        error('ConfigAdapter:MissingData', ...
              '缺少X方向识别参数(identified_params_x)，无法获取刚度值');
    end
    
    if length(linear.identified_params_x) < 6
        error('ConfigAdapter:InvalidData', ...
              'X方向识别参数不完整，需要6个值: [k_g, c_g, k_rm, c_rm, k_mt, c_mt]');
    end
    
    params_x = linear.identified_params_x;
    
    % 根据分枝级别选择对应的刚度阻尼
    level = determineBranchLevel(branchName);
    
    switch level
        case 1  % 一级分枝 - 使用 k_rm (Root-Mid连接刚度)
            k_est = params_x(3);
            c_est = params_x(4);
        case 2  % 二级分枝 - 使用 k_mt (Mid-Tip连接刚度)
            k_est = params_x(5);
            c_est = params_x(6);
        case 3  % 三级分枝 - 基于识别出的层级间衰减趋势进行推算
            % 获取一级和二级分枝的特征刚度/阻尼
            k_L1 = params_x(3); % k_rm (一级分枝特征值)
            k_L2 = params_x(5); % k_mt (二级分枝特征值)
    
            c_L1 = params_x(4); % c_rm
            c_L2 = params_x(6); % c_mt
    
            % 计算层级间的自然衰减率
            % 逻辑：假设 Level 3 相对于 Level 2 的衰减比例，与 Level 2 相对于 Level 1 的比例相似
            if k_L1 > 0
                decay_ratio_k = k_L2 / k_L1;
            else
                error('ConfigAdapter:InvalidStiffness', '识别出的一级分枝刚度(k_rm)非正，无法计算衰减率。');
            end
    
            if c_L1 > 0
                decay_ratio_c = c_L2 / c_L1;
            else
                decay_ratio_c = 0.8; % 阻尼测量通常波动大，如果c_rm异常，给一个保守的衰减估计警告
                warning('识别出的一级分枝阻尼(c_rm)非正，使用保守衰减率 0.8。');
            end
    
            % 应用衰减率计算三级分枝参数
            k_est = k_L2 * decay_ratio_k;
            c_est = c_L2 * decay_ratio_c;
    
            fprintf('    [推算] 三级分枝 %s: 基于层级衰减率 (k_decay=%.2f, c_decay=%.2f) 计算参数\n', ...
                    branchName, decay_ratio_k, decay_ratio_c);
    end
    
    % 验证值的有效性
    if k_est <= 0
        error('ConfigAdapter:InvalidData', ...
              '分枝 %s 计算的刚度为非正值(%.4f)，请检查实验数据', branchName, k_est);
    end
    if c_est <= 0
        error('ConfigAdapter:InvalidData', ...
              '分枝 %s 计算的阻尼为非正值(%.4f)，请检查实验数据', branchName, c_est);
    end
    
    fprintf('    分枝 %s (level=%d): k=%.2f N/m, c=%.4f Ns/m (从实验识别)\n', ...
            branchName, level, k_est, c_est);
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
    
    required_fields = {'total_mass', 'length', 'diameter_base', 'diameter_tip', 'mass_dist', 'z_factor'};
    
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