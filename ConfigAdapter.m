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
    predefined = struct();
    
    % 获取分枝几何数据
    branches = preConfig.branches;
    branchNames = fieldnames(branches);
    
    for i = 1:length(branchNames)
        name = branchNames{i};
        b = branches.(name);
        
        % 基于几何估算刚度阻尼（如果没有识别结果）
        [k_base, c_base] = estimateStiffnessDamping(b);
        
        % 生成分枝参数
        predefined.(name) = generateBranchSegmentParams(b, k_base, c_base);
        
        % 确定是否需要挂果
        branchLevel = determineBranchLevel(name);
        predefined.(name).branch_level = branchLevel;
        
    end
end

%% ==================== 生成单个分枝的段参数 ====================
function params = generateBranchSegmentParams(branchGeom, k_base, c_base)
    params = struct();
    
    z_factor = 1.0;
    m_dist = branchGeom.mass_dist;
    m_total = branchGeom.total_mass;
    
    % 刚度阻尼递减因子
    k_taper = [0.5, 0.3, 0.2];
    c_taper = [1.0, 0.8, 0.6];
    
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
    
    % 保存几何信息
    params.geometry.length = branchGeom.length;
    params.geometry.diameter_base = branchGeom.diameter_base;
    params.geometry.diameter_tip = branchGeom.diameter_tip;
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
function [k_est, c_est] = estimateStiffnessDamping(geom)
    % 基于几何参数估算刚度和阻尼
    % 使用简化的悬臂梁模型
    
    if isfield(geom, 'diameter_base') && isfield(geom, 'length')
        d = geom.diameter_base;
        L = geom.length;
        E = 8e9;  % 木材弹性模量估值 (Pa)
        
        % 惯性矩 I = pi*d^4/64
        I = pi * d^4 / 64;
        
        % 悬臂梁刚度 k = 3EI/L^3
        k_est = 3 * E * I / L^3;
        
        % 限制在合理范围内
        k_est = max(100, min(k_est, 1e6));
        
        % 阻尼估算（阻尼比约0.02-0.05）
        if isfield(geom, 'total_mass')
            m = geom.total_mass;
            omega = sqrt(k_est / m);
            zeta = 0.03;  % 假设阻尼比
            c_est = 2 * zeta * m * omega;
        else
            c_est = k_est * 0.001;  % 简化估算
        end
        
        c_est = max(1, min(c_est, 1000));
    else
        % 默认值
        k_est = 5000;
        c_est = 10;
    end
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
