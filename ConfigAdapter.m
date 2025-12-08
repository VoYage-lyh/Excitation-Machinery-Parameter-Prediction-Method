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
    
    %% ==================== 生成参数识别所需参数 ====================
    analysis_params = struct();
    
    % 信号处理参数
    analysis_params.fs_target = preConfig.signal.fs_target;
    analysis_params.cutoff_freq = preConfig.signal.cutoff_freq;
    analysis_params.filter_order = preConfig.signal.filter_order;
    analysis_params.nfft = preConfig.signal.nfft;
    analysis_params.freq_range = [preConfig.signal.freq_range_min, preConfig.signal.freq_range_max];
    analysis_params.snr_threshold = preConfig.signal.snr_threshold;
    
    % 节点配置
    analysis_params.n_nodes = 3;
    analysis_params.n_dof = 6;
    analysis_params.node_labels = {'Root', 'Mid', 'Tip'};
    analysis_params.direction_labels = {'Y', 'Z'};
    
    % 传递拓扑信息（识别代码可能需要）
    analysis_params.topology = preConfig.topology;
    
    %% ==================== 生成仿真所需参数 ====================
    sim_params = struct();
    
    % 基础设置
    sim_params.workFolder = preConfig.basic.workFolder;
    sim_params.model_name = preConfig.basic.modelName;
    sim_params.gravity_g = preConfig.basic.gravity_g;
    sim_params.use_parallel = preConfig.basic.useParallel;
    
    % 拓扑结构
    sim_params.config = preConfig.topology;
    
    % 主干参数（几何+质量，刚度阻尼待填充）
    sim_params.trunk = buildTrunkParams(preConfig, identifiedParams);
    
    % 分枝参数（几何+质量，刚度阻尼待填充）
    sim_params.predefined_params = buildAllBranchParams(preConfig, identifiedParams);
    
    % 果实参数
    % 构建果实配置，确保包含所有必需字段
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
    sim_params.default_fruit_params = buildFruitParams(preConfig, identifiedParams);
    
    % 激励参数
    sim_params.excitation = preConfig.excitation;
    
    % 如果有识别结果，更新激励频率为第一阶固有频率
    if ~isempty(identifiedParams) && isfield(identifiedParams, 'linear')
        if isfield(identifiedParams.linear, 'natural_freqs_x') && ...
           ~isempty(identifiedParams.linear.natural_freqs_x)
            sim_params.excitation.frequency_hz = identifiedParams.linear.natural_freqs_x(1);
            fprintf('  激励频率已更新为第一阶固有频率: %.2f Hz\n', ...
                    sim_params.excitation.frequency_hz);
        end
    end

    % 验证 sim_params 包含所有必需字段
    required_sim_fields = {'config', 'trunk', 'predefined_params', 'fruit_config', ...
                          'default_fruit_params', 'excitation', 'sim_stop_time', 'sim_fixed_step'};
    for i_f = 1:length(required_sim_fields)
        if ~isfield(sim_params, required_sim_fields{i_f})
            error('ConfigAdapter 内部错误：sim_params 缺少字段 %s', required_sim_fields{i_f});
        end
    end

    % 仿真控制
    sim_params.sim_stop_time = preConfig.simulation.stop_time;
    sim_params.sim_fixed_step = preConfig.simulation.fixed_step;
    
    % 标记参数来源
    sim_params.has_identified_params = ~isempty(identifiedParams);
    
    % 输出摘要
    fprintf('\n===== 参数适配摘要 =====\n');
    fprintf('预配置:\n');
    fprintf('  拓扑: %d个一级分枝\n', preConfig.topology.num_primary_branches);
    fprintf('  主干质量: %.2f kg\n', preConfig.trunk.total_mass);
    fprintf('  果实位置: 二级[mid=%d,tip=%d] 三级[mid=%d,tip=%d]\n', ...
            preConfig.fruit.attach_secondary_mid, preConfig.fruit.attach_secondary_tip, ...
            preConfig.fruit.attach_tertiary_mid, preConfig.fruit.attach_tertiary_tip);
    
    if ~isempty(identifiedParams)
        fprintf('识别参数: 已加载\n');
        if isfield(identifiedParams.linear, 'natural_freqs_x')
            fprintf('  固有频率(X): [%s] Hz\n', num2str(identifiedParams.linear.natural_freqs_x', '%.2f '));
        end
    else
        fprintf('识别参数: 未加载（将使用估算值）\n');
    end
    fprintf('========================\n\n');
end

%% ==================== 构建主干参数 ====================
function trunk = buildTrunkParams(preConfig, identifiedParams)
    trunk = struct();
    
    % 几何参数
    trunk.length = preConfig.trunk.length;
    trunk.diameter_base = preConfig.trunk.diameter_base;
    trunk.diameter_tip = preConfig.trunk.diameter_tip;
    trunk.z_factor = preConfig.trunk.z_factor;
    
    % 质量参数
    m_total = preConfig.trunk.total_mass;
    m_dist = preConfig.trunk.mass_distribution;
    
    trunk.root.m = m_total * m_dist(1);
    trunk.mid.m = m_total * m_dist(2);
    trunk.tip.m = m_total * m_dist(3);
    
    % 刚度和阻尼参数
    if ~isempty(identifiedParams) && isfield(identifiedParams, 'linear')
        % 使用识别结果
        K = identifiedParams.linear.K;
        C = identifiedParams.linear.C;
        
        % 提取对角元素作为各节点的连接刚度/阻尼
        if size(K, 1) >= 3
            trunk.root.k_y_conn_to_base = K(1,1);
            trunk.root.k_y_conn = K(1,1) * 0.8;
            trunk.mid.k_y_conn = K(2,2);
            trunk.tip.k_y_conn = K(3,3);
            
            trunk.root.c_y_conn_to_base = C(1,1);
            trunk.root.c_y_conn = C(1,1) * 0.8;
            trunk.mid.c_y_conn = C(2,2);
            trunk.tip.c_y_conn = C(3,3);
            
            % Z方向（应用z_factor）
            z_factor = trunk.z_factor;
            trunk.root.k_z_conn_to_base = K(1,1) * z_factor;
            trunk.root.k_z_conn = K(1,1) * 0.8 * z_factor;
            trunk.mid.k_z_conn = K(2,2) * z_factor;
            trunk.tip.k_z_conn = K(3,3) * z_factor;
            
            trunk.root.c_z_conn_to_base = C(1,1) * z_factor;
            trunk.root.c_z_conn = C(1,1) * 0.8 * z_factor;
            trunk.mid.c_z_conn = C(2,2) * z_factor;
            trunk.tip.c_z_conn = C(3,3) * z_factor;
        end
    else
        % 使用基于几何的估算值
        [k_est, c_est] = estimateStiffnessDamping(preConfig.trunk);
        
        z_factor = trunk.z_factor;
        
        trunk.root.k_y_conn_to_base = k_est;
        trunk.root.c_y_conn_to_base = c_est;
        trunk.root.k_z_conn_to_base = k_est * z_factor;
        trunk.root.c_z_conn_to_base = c_est * z_factor;
        
        trunk.root.k_y_conn = k_est * 0.8;
        trunk.root.c_y_conn = c_est * 0.8;
        trunk.root.k_z_conn = k_est * 0.8 * z_factor;
        trunk.root.c_z_conn = c_est * 0.8 * z_factor;
        
        trunk.mid.k_y_conn = k_est * 0.5;
        trunk.mid.c_y_conn = c_est * 0.8;
        trunk.mid.k_z_conn = k_est * 0.5 * z_factor;
        trunk.mid.c_z_conn = c_est * 0.8 * z_factor;
        
        trunk.tip.k_y_conn = k_est * 0.2;
        trunk.tip.c_y_conn = c_est * 0.8;
        trunk.tip.k_z_conn = k_est * 0.2 * z_factor;
        trunk.tip.c_z_conn = c_est * 0.8 * z_factor;
    end
end

%% ==================== 构建所有分枝参数 ====================
function predefined = buildAllBranchParams(preConfig, identifiedParams)
    % 构建所有分枝的物理参数（root, mid, tip 段）
    % 输入:
    %   preConfig - 来自GUI的预配置（包含primary, secondary, tertiary几何参数）
    %   identifiedParams - 识别的参数（可选，用于更新刚度阻尼）
    % 输出:
    %   predefined - 包含所有分枝段参数的结构体
    
    predefined = struct();
    
    % --- 处理一级分枝 ---
    if isfield(preConfig, 'primary') && isstruct(preConfig.primary)
        pFields = fieldnames(preConfig.primary);
        for i = 1:length(pFields)
            name = pFields{i};
            b = preConfig.primary.(name);
            
            % 基于几何估算刚度阻尼
            [k_base, c_base] = estimateStiffnessDamping(b, identifiedParams);
            
            % 如果有识别参数，尝试使用识别值
            if ~isempty(identifiedParams) && isfield(identifiedParams, 'branches')
                if isfield(identifiedParams.branches, name)
                    k_base = identifiedParams.branches.(name).k_base;
                    c_base = identifiedParams.branches.(name).c_base;
                end
            end
            
            % 生成分枝段参数
            predefined.(name) = generateBranchSegmentParams(b, k_base, c_base);
            predefined.(name).branch_level = 1;
        end
    end
    
    % --- 处理二级分枝 ---
    if isfield(preConfig, 'secondary') && isstruct(preConfig.secondary)
        sFields = fieldnames(preConfig.secondary);
        for i = 1:length(sFields)
            name = sFields{i};
            b = preConfig.secondary.(name);
            
            [k_base, c_base] = estimateStiffnessDamping(b, identifiedParams);
            
            if ~isempty(identifiedParams) && isfield(identifiedParams, 'branches')
                if isfield(identifiedParams.branches, name)
                    k_base = identifiedParams.branches.(name).k_base;
                    c_base = identifiedParams.branches.(name).c_base;
                end
            end
            
            predefined.(name) = generateBranchSegmentParams(b, k_base, c_base);
            predefined.(name).branch_level = 2;

            % 为二级分枝添加果实附着信息
            if shouldAttachAtPosition(2, 'mid', preConfig.fruit)
                predefined.(name).fruit_at_mid = buildFruitParamsForBranch(preConfig, identifiedParams);
            end
            if shouldAttachAtPosition(2, 'tip', preConfig.fruit)
                predefined.(name).fruit_at_tip = buildFruitParamsForBranch(preConfig, identifiedParams);
            end
        end
    end
    
    % --- 处理三级分枝 ---
    if isfield(preConfig, 'tertiary') && isstruct(preConfig.tertiary)
        tFields = fieldnames(preConfig.tertiary);
        for i = 1:length(tFields)
            name = tFields{i};
            b = preConfig.tertiary.(name);
            
            [k_base, c_base] = estimateStiffnessDamping(b, identifiedParams);
            
            if ~isempty(identifiedParams) && isfield(identifiedParams, 'branches')
                if isfield(identifiedParams.branches, name)
                    k_base = identifiedParams.branches.(name).k_base;
                    c_base = identifiedParams.branches.(name).c_base;
                end
            end
            
            predefined.(name) = generateBranchSegmentParams(b, k_base, c_base);
            predefined.(name).branch_level = 3;

            % 【新增】为三级分枝添加果实附着信息
            if shouldAttachAtPosition(3, 'mid', preConfig.fruit)
                predefined.(name).fruit_at_mid = buildFruitParamsForBranch(preConfig, identifiedParams);
            end
            if shouldAttachAtPosition(3, 'tip', preConfig.fruit)
                predefined.(name).fruit_at_tip = buildFruitParamsForBranch(preConfig, identifiedParams);
            end
        end
    end
    
    % 验证生成的参数
    validatePredefinedParams(predefined, preConfig);
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
        warning('ConfigAdapter:MissingBranches', ...
                '以下分枝在预配置中缺少几何参数，将使用默认值:\n  %s', ...
                strjoin(missingBranches, ', '));
        
        % 为缺失的分枝生成默认参数
        for i = 1:length(missingBranches)
            branch_id = missingBranches{i};
            predefined.(branch_id) = generateDefaultBranchSegment(branch_id);
        end
    end
end

%% ==================== 为分枝生成默认段参数 ====================
function predefined = generatePredefinedParams(preConfig, identifiedParams)
    % 生成预定义分枝参数 
    % 使用所有已定义的辅助函数，无数据则报错
    
    fprintf('  生成预定义分枝参数（严格模式）...\n');
    
    % ===== 验证必需输入 =====
    if isempty(preConfig)
        error('ConfigAdapter:MissingData', '缺少预配置(preConfig)');
    end
    
    if isempty(identifiedParams)
        error('ConfigAdapter:MissingData', ...
              '缺少参数识别结果(identifiedParams)，请先运行参数识别程序');
    end
    
    % 验证拓扑配置
    if ~isfield(preConfig, 'topology')
        error('ConfigAdapter:MissingData', '预配置缺少拓扑结构(topology)');
    end
    
    if ~isfield(preConfig.topology, 'num_primary_branches')
        error('ConfigAdapter:MissingData', '拓扑配置缺少一级分枝数量');
    end
    
    if ~isfield(preConfig.topology, 'secondary_branches_count')
        error('ConfigAdapter:MissingData', '拓扑配置缺少二级分枝数量');
    end
    
    if ~isfield(preConfig.topology, 'tertiary_branches_count')
        error('ConfigAdapter:MissingData', '拓扑配置缺少三级分枝数量');
    end
    
    % 验证果实配置
    if ~isfield(preConfig, 'fruit')
        error('ConfigAdapter:MissingData', '预配置缺少果实配置(fruit)');
    end
    
    fruitConfig = preConfig.fruit;
    required_fruit_fields = {'attach_secondary_mid', 'attach_secondary_tip', ...
                             'attach_tertiary_mid', 'attach_tertiary_tip', ...
                             'mass', 'F_break_mean', 'F_break_std'};
    for i = 1:length(required_fruit_fields)
        if ~isfield(fruitConfig, required_fruit_fields{i})
            error('ConfigAdapter:MissingData', '果实配置缺少字段: %s', required_fruit_fields{i});
        end
    end
    
    % 验证递减因子
    if ~isfield(identifiedParams, 'linear') || ...
       ~isfield(identifiedParams.linear, 'taper_factors')
        error('ConfigAdapter:MissingData', ...
              '识别参数缺少递减因子(taper_factors)，请确保参数识别程序计算了递减因子');
    end
    
    identified_taper = identifiedParams.linear.taper_factors;
    
    if ~isfield(identified_taper, 'k') || ~isfield(identified_taper, 'c')
        error('ConfigAdapter:MissingData', ...
              '递减因子结构不完整，缺少k或c字段');
    end
    
    predefined = struct();
    
    num_p = preConfig.topology.num_primary_branches;
    secondary_count = preConfig.topology.secondary_branches_count;
    tertiary_count = preConfig.topology.tertiary_branches_count;
    
    fprintf('    拓扑结构: %d个一级分枝\n', num_p);
    
    % ===== 生成一级分枝参数 =====
    fprintf('    处理一级分枝...\n');
    for p = 1:num_p
        name = sprintf('P%d', p);
        
        % 使用 determineBranchLevel 函数
        branch_level = determineBranchLevel(name);
        
        % 获取分枝几何参数（必须存在）
        if ~isfield(preConfig, 'primary') || ~isfield(preConfig.primary, name)
            error('ConfigAdapter:MissingData', ...
                  '预配置缺少一级分枝 %s 的几何参数，请在GUI中配置', name);
        end
        branchGeom = preConfig.primary.(name);
        
        % 验证几何参数完整性
        validateBranchGeometry(branchGeom, name);
        
        % 使用 estimateStiffnessDamping 函数（从实验数据）
        [k_base, c_base] = estimateStiffnessDamping(branchGeom, identifiedParams, name);
        
        % 生成段参数（传入识别的递减因子）
        predefined.(name) = generateBranchSegmentParams(branchGeom, k_base, c_base, identified_taper);
        predefined.(name).branch_level = branch_level;
        
        % 使用 shouldAttachFruit 函数
        if shouldAttachFruit(name, branch_level, fruitConfig)
            fprintf('      警告: 一级分枝 %s 被配置为挂果，但一级分枝通常不直接挂果\n', name);
        end
        
        fprintf('      %s: level=%d, k_base=%.2f, c_base=%.4f\n', name, branch_level, k_base, c_base);
    end
    
    % ===== 生成二级分枝参数 =====
    fprintf('    处理二级分枝...\n');
    for p = 1:num_p
        if p > length(secondary_count)
            error('ConfigAdapter:InvalidTopology', ...
                  '二级分枝数量配置与一级分枝数量不匹配');
        end
        
        num_s = secondary_count(p);
        for s = 1:num_s
            name = sprintf('P%d_S%d', p, s);
            
            % 使用 determineBranchLevel 函数
            branch_level = determineBranchLevel(name);
            
            % 获取几何参数
            if ~isfield(preConfig, 'secondary') || ~isfield(preConfig.secondary, name)
                error('ConfigAdapter:MissingData', ...
                      '预配置缺少二级分枝 %s 的几何参数，请在GUI中配置', name);
            end
            branchGeom = preConfig.secondary.(name);
            
            validateBranchGeometry(branchGeom, name);
            
            [k_base, c_base] = estimateStiffnessDamping(branchGeom, identifiedParams, name);
            
            predefined.(name) = generateBranchSegmentParams(branchGeom, k_base, c_base, identified_taper);
            predefined.(name).branch_level = branch_level;
            
            % 使用 shouldAttachFruit 和 shouldAttachAtPosition 函数
            if shouldAttachFruit(name, branch_level, fruitConfig)
                if shouldAttachAtPosition(branch_level, 'mid', fruitConfig)
                    predefined.(name).fruit_at_mid = buildFruitParamsStrict(preConfig, identifiedParams);
                    fprintf('      %s: Mid段挂果\n', name);
                end
                if shouldAttachAtPosition(branch_level, 'tip', fruitConfig)
                    predefined.(name).fruit_at_tip = buildFruitParamsStrict(preConfig, identifiedParams);
                    fprintf('      %s: Tip段挂果\n', name);
                end
            end
            
            fprintf('      %s: level=%d, k_base=%.2f, c_base=%.4f\n', name, branch_level, k_base, c_base);
        end
    end
    
    % ===== 生成三级分枝参数 =====
    fprintf('    处理三级分枝...\n');
    for p = 1:num_p
        if p > length(tertiary_count)
            continue;
        end
        
        tertiary_for_p = tertiary_count{p};
        num_s = secondary_count(p);
        
        for s = 1:num_s
            if s > length(tertiary_for_p)
                continue;
            end
            
            num_t = tertiary_for_p(s);
            for t = 1:num_t
                name = sprintf('P%d_S%d_T%d', p, s, t);
                
                % 使用 determineBranchLevel 函数
                branch_level = determineBranchLevel(name);
                
                % 获取几何参数
                if ~isfield(preConfig, 'tertiary') || ~isfield(preConfig.tertiary, name)
                    error('ConfigAdapter:MissingData', ...
                          '预配置缺少三级分枝 %s 的几何参数，请在GUI中配置', name);
                end
                branchGeom = preConfig.tertiary.(name);
                
                validateBranchGeometry(branchGeom, name);
                
                [k_base, c_base] = estimateStiffnessDamping(branchGeom, identifiedParams, name);
                
                predefined.(name) = generateBranchSegmentParams(branchGeom, k_base, c_base, identified_taper);
                predefined.(name).branch_level = branch_level;
                
                % 使用 shouldAttachFruit 和 shouldAttachAtPosition 函数
                if shouldAttachFruit(name, branch_level, fruitConfig)
                    if shouldAttachAtPosition(branch_level, 'mid', fruitConfig)
                        predefined.(name).fruit_at_mid = buildFruitParamsStrict(preConfig, identifiedParams);
                        fprintf('      %s: Mid段挂果\n', name);
                    end
                    if shouldAttachAtPosition(branch_level, 'tip', fruitConfig)
                        predefined.(name).fruit_at_tip = buildFruitParamsStrict(preConfig, identifiedParams);
                        fprintf('      %s: Tip段挂果\n', name);
                    end
                end
                
                fprintf('      %s: level=%d, k_base=%.2f, c_base=%.4f\n', name, branch_level, k_base, c_base);
            end
        end
    end
    
    fprintf('  预定义参数生成完成。\n');
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
    
    % 验证递减因子必须存在
    if nargin < 4 || isempty(identified_taper)
        error('ConfigAdapter:MissingData', ...
              '缺少递减因子(identified_taper)，必须先运行参数识别程序从实验数据计算递减因子');
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

%% ==================== 构建果实参数 ====================
function fruit = buildFruitParams(preConfig, identifiedParams)
    fruit = struct();
    
    % 物理参数
    fruit.m = preConfig.fruit.mass;
    fruit.diameter = preConfig.fruit.diameter;
    fruit.pedicel_length = preConfig.fruit.pedicel_length;
    fruit.pedicel_diameter = preConfig.fruit.pedicel_diameter;
    
    % 断裂力
    fruit.F_break = preConfig.fruit.F_break_mean;
    fruit.F_break_std = preConfig.fruit.F_break_std;
    
    % 果柄刚度阻尼
    if ~isempty(identifiedParams) && isfield(identifiedParams, 'detachment_model')
        % 使用识别的果实参数（如果有）
        % TODO: 从识别结果中提取
        fruit.k_pedicel_y = 8;
        fruit.c_pedicel_y = 0.2;
        fruit.k_pedicel_z = 12;
        fruit.c_pedicel_z = 0.2;
    else
        % 基于几何估算
        d = preConfig.fruit.pedicel_diameter;
        L = preConfig.fruit.pedicel_length;
        E = 1e7;  % 估算弹性模量 (Pa)
        
        % 简化悬臂梁模型: k = 3EI/L^3
        I = pi * d^4 / 64;
        k_est = 3 * E * I / L^3;
        
        fruit.k_pedicel_y = min(k_est, 20);  % 限制范围
        fruit.k_pedicel_z = min(k_est * 1.5, 30);
        fruit.c_pedicel_y = 0.2;
        fruit.c_pedicel_z = 0.2;
    end
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
        case 3  % 三级分枝 - 使用缩放后的 k_mt
            k_est = params_x(5) * 0.7;  % 三级比二级刚度小
            c_est = params_x(6) * 0.8;
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

function fruitParams = buildFruitParamsForBranch(preConfig, identifiedParams)
    % 为单个分枝位置构建果实参数结构体
    % 使用统一的字段命名: fruit_at_mid, fruit_at_tip
    
    fruitParams = struct();
    
    % 基础物理参数从预配置读取
    fruitParams.m = preConfig.fruit.mass;
    
    % 果柄刚度和阻尼 - 优先使用识别值
    if ~isempty(identifiedParams) && isfield(identifiedParams, 'fruit_pedicel')
        fruitParams.k_pedicel_y = identifiedParams.fruit_pedicel.k_y;
        fruitParams.c_pedicel_y = identifiedParams.fruit_pedicel.c_y;
        fruitParams.k_pedicel_z = identifiedParams.fruit_pedicel.k_z;
        fruitParams.c_pedicel_z = identifiedParams.fruit_pedicel.c_z;
    else
        % 基于果柄几何估算刚度
        L_ped = preConfig.fruit.pedicel_length;
        D_ped = preConfig.fruit.pedicel_diameter;
        E_wood = 1e9;  % 木材弹性模量近似值 (Pa)
        I_ped = pi * D_ped^4 / 64;  % 惯性矩
        
        % 悬臂梁刚度估算: k = 3EI/L^3
        k_estimated = 3 * E_wood * I_ped / (L_ped^3);
        k_estimated = max(5, min(k_estimated, 100));  % 限制在合理范围
        
        fruitParams.k_pedicel_y = k_estimated;
        fruitParams.k_pedicel_z = k_estimated * 1.2;  % Z方向略硬
        
        % 阻尼估算 - 基于阻尼比
        zeta_pedicel = 0.05;  % 果柄阻尼比经验值
        omega_ped = sqrt(k_estimated / preConfig.fruit.mass);
        c_estimated = 2 * zeta_pedicel * preConfig.fruit.mass * omega_ped;
        
        fruitParams.c_pedicel_y = c_estimated;
        fruitParams.c_pedicel_z = c_estimated * 1.2;
    end
    
    % 断裂力 - 考虑随机性
    fruitParams.F_break = preConfig.fruit.F_break_mean + ...
                          preConfig.fruit.F_break_std * randn();
    fruitParams.F_break = max(2, fruitParams.F_break);  % 最小断裂力
end

function fruitParams = buildFruitParamsStrict(preConfig, identifiedParams)
    % 构建果实参数 - 严格模式，所有参数必须有明确来源
    % 无数据则报错，绝不使用默认值
    
    fruitParams = struct();
    
    % 果实质量 - 必须从预配置获取
    if ~isfield(preConfig, 'fruit') || ~isfield(preConfig.fruit, 'mass')
        error('ConfigAdapter:MissingData', ...
              '预配置中缺少果实质量(preConfig.fruit.mass)，请在GUI中配置');
    end
    fruitParams.m = preConfig.fruit.mass;
    
    % 果柄刚度和阻尼 - 必须从识别参数获取
    if isempty(identifiedParams)
        error('ConfigAdapter:MissingData', ...
              '缺少参数识别结果(identifiedParams)，请先运行参数识别程序');
    end
    
    if ~isfield(identifiedParams, 'fruit_pedicel')
        error('ConfigAdapter:MissingData', ...
              '识别参数中缺少果柄参数(identifiedParams.fruit_pedicel)，请确保参数识别包含果柄刚度阻尼识别');
    end
    
    if ~isfield(identifiedParams.fruit_pedicel, 'k_y') || ...
       ~isfield(identifiedParams.fruit_pedicel, 'c_y') || ...
       ~isfield(identifiedParams.fruit_pedicel, 'k_z') || ...
       ~isfield(identifiedParams.fruit_pedicel, 'c_z')
        error('ConfigAdapter:MissingData', ...
              '果柄参数不完整，需要: k_y, c_y, k_z, c_z');
    end
    
    fruitParams.k_pedicel_y = identifiedParams.fruit_pedicel.k_y;
    fruitParams.c_pedicel_y = identifiedParams.fruit_pedicel.c_y;
    fruitParams.k_pedicel_z = identifiedParams.fruit_pedicel.k_z;
    fruitParams.c_pedicel_z = identifiedParams.fruit_pedicel.c_z;
    
    % 断裂力 - 必须从预配置获取
    if ~isfield(preConfig.fruit, 'F_break_mean') || ~isfield(preConfig.fruit, 'F_break_std')
        error('ConfigAdapter:MissingData', ...
              '预配置中缺少断裂力参数(F_break_mean, F_break_std)，请在GUI中配置');
    end
    
    fruitParams.F_break = preConfig.fruit.F_break_mean + ...
                          preConfig.fruit.F_break_std * randn();
    fruitParams.F_break = max(1, fruitParams.F_break);
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