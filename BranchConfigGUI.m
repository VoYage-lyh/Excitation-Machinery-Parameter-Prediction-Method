function config = BranchConfigGUI()
% BranchConfigGUI_v2 - 统一参数配置界面（修正版）
% 
% 工作流程：
%   1. 用户在此GUI中预定义：拓扑结构、几何参数、质量分配、仿真参数
%   2. 运行参数识别代码 -> 识别刚度、阻尼、非线性参数
%   3. 运行仿真代码 -> 读取识别结果进行仿真
%
% 注意：刚度和阻尼由参数识别代码从实验数据中获取，不在此处配置
%
% 果实配置逻辑：
%   - 二级分枝：mid和tip位置都挂果
%   - 三级分枝：mid和tip位置都挂果
%   - 一级分枝：不直接挂果（通过子分枝挂果）

    config = [];
    
    % 创建主窗口
    fig = figure('Name', '果树振动分析 - 预配置界面 v2.0', ...
                 'NumberTitle', 'off', ...
                 'MenuBar', 'none', ...
                 'ToolBar', 'none', ...
                 'Position', [100 50 950 750], ...
                 'Resize', 'on', ...
                 'CloseRequestFcn', @onClose);
    
    % 初始化配置
    configData = getDefaultConfig();
    userData.configData = configData;
    userData.confirmed = false;
    set(fig, 'UserData', userData);
    
    % 创建标签页
    tabGroup = uitabgroup(fig, 'Position', [0.01 0.08 0.98 0.91]);
    
    % Tab 1: 工作流程说明
    tab0 = uitab(tabGroup, 'Title', '📋 使用说明');
    createInstructionPanel(tab0);
    
    % Tab 2: 基础设置
    tab1 = uitab(tabGroup, 'Title', '1. 基础设置');
    createBasicSettingsPanel(tab1, configData);
    
    % Tab 3: 拓扑结构
    tab2 = uitab(tabGroup, 'Title', '2. 拓扑结构');
    createTopologyPanel(tab2, configData);
    
    % Tab 4: 几何与质量参数
    tab3 = uitab(tabGroup, 'Title', '3. 几何与质量');
    createGeometryMassPanel(tab3, configData);
    
    % Tab 5: 果实配置
    tab4 = uitab(tabGroup, 'Title', '4. 果实配置');
    createFruitConfigPanel(tab4, configData);
    
    % Tab 6: 激励参数
    tab5 = uitab(tabGroup, 'Title', '5. 激励参数');
    createExcitationPanel(tab5, configData);
    
    % Tab 7: 仿真参数
    tab6 = uitab(tabGroup, 'Title', '6. 仿真参数');
    createSimulationPanel(tab6, configData);
    
    % Tab 8: 信号处理参数
    tab7 = uitab(tabGroup, 'Title', '7. 信号处理');
    createSignalProcessingPanel(tab7, configData);
    
    % 底部按钮
    uicontrol(fig, 'Style', 'pushbutton', ...
              'String', '加载配置', ...
              'Position', [50 15 100 30], ...
              'Callback', @(~,~) loadConfig(fig));
    
    uicontrol(fig, 'Style', 'pushbutton', ...
              'String', '保存配置', ...
              'Position', [170 15 100 30], ...
              'Callback', @(~,~) saveConfig(fig));
    
    uicontrol(fig, 'Style', 'pushbutton', ...
              'String', '恢复默认', ...
              'Position', [290 15 100 30], ...
              'Callback', @(~,~) resetToDefault(fig));
    
    uicontrol(fig, 'Style', 'pushbutton', ...
              'String', '预览拓扑', ...
              'Position', [410 15 100 30], ...
              'Callback', @(~,~) previewTopology(fig));
    
    uicontrol(fig, 'Style', 'pushbutton', ...
              'String', '✓ 确认并继续', ...
              'Position', [700 15 130 30], ...
              'BackgroundColor', [0.3 0.7 0.3], ...
              'FontWeight', 'bold', ...
              'Callback', @(~,~) confirmConfig(fig));
    
    uicontrol(fig, 'Style', 'pushbutton', ...
              'String', '取消', ...
              'Position', [850 15 80 30], ...
              'Callback', @(~,~) cancelConfig(fig));
    
    uiwait(fig);
    
    if isvalid(fig)
        userData = get(fig, 'UserData');
        if userData.confirmed
            config = collectAllParameters(fig);
        end
        delete(fig);
    end
end

%% ==================== 默认配置 ====================
function config = getDefaultConfig()
    config = struct();
    
    % 基础设置
    config.basic.workFolder = pwd;
    config.basic.modelName = 'MDOF_Hierarchical_Vibration_Sim';
    config.basic.gravity_g = 9.81;
    config.basic.useParallel = true;
    config.basic.parallel_max_workers = 4;
    
    % 信号处理参数（用于参数识别）
    config.signal.fs_target = 1000;
    config.signal.cutoff_freq = 65;
    config.signal.filter_order = 4;
    config.signal.freq_range_min = 3;
    config.signal.freq_range_max = 50;
    config.signal.snr_threshold = 10;
    config.signal.nfft = 2048;
        
    % 拓扑结构
    config.topology.num_primary_branches = 3;
    config.topology.secondary_branches_count = [2, 1, 2];
    config.topology.tertiary_branches_count = {[1, 2], [0], [1, 0]};
    
    % 主干几何与质量参数
    config.trunk.total_mass = 11.23;           % 总质量 (kg)
    config.trunk.length = 1.5;                  % 长度 (m)
    config.trunk.diameter_base = 0.15;          % 基部直径 (m)
    config.trunk.diameter_tip = 0.08;           % 顶部直径 (m)
    config.trunk.mass_distribution = [0.4, 0.35, 0.25];  % root/mid/tip质量分配
    config.trunk.z_factor = 1.0;                % Z方向刚度因子
    
    % 根据拓扑结构动态生成分枝参数
    [config.primary, config.secondary, config.tertiary] = generateDefaultBranchParams(...
        config.topology.num_primary_branches, ...
        config.topology.secondary_branches_count, ...
        config.topology.tertiary_branches_count);
    
    % 果实参数（物理属性）
    config.fruit.mass = 0.08;                   % 单个果实质量 (kg)
    config.fruit.diameter = 0.06;               % 果实直径 (m)
    config.fruit.pedicel_length = 0.02;         % 果柄长度 (m)
    config.fruit.pedicel_diameter = 0.003;      % 果柄直径 (m)
    config.fruit.F_break_mean = 5.0;            % 平均断裂力 (N)
    config.fruit.F_break_std = 1.0;             % 断裂力标准差 (N)
    
    % 果实位置配置（新逻辑：二级和三级分枝的mid和tip都挂果）
    config.fruit.attach_secondary_mid = true;   % 二级分枝mid挂果
    config.fruit.attach_secondary_tip = true;   % 二级分枝tip挂果
    config.fruit.attach_tertiary_mid = true;    % 三级分枝mid挂果
    config.fruit.attach_tertiary_tip = true;    % 三级分枝tip挂果
    config.fruit.fruits_per_node = 1;           % 每个节点挂果数量
    
    % 激励参数
    config.excitation.type = 'impulse';
    config.excitation.sine_amplitude_y = 355;
    config.excitation.sine_amplitude_z = 275;
    config.excitation.frequency_hz = 7;         % 将由识别结果更新
    config.excitation.phase_y_rad = 0;
    config.excitation.phase_z_rad = pi/2;
    config.excitation.impulse_gain_y = 14500;
    config.excitation.impulse_gain_z = 15500;
    config.excitation.pulse_period_s = 20;
    config.excitation.pulse_width_percent = 0.025;
    config.excitation.pulse_delay_y_s = 0;
    config.excitation.pulse_delay_z_s = 0;
    config.excitation.start_time = 0.5;
    config.excitation.end_time = 3.0;
    
    % 仿真参数
    config.simulation.stop_time = 20;
    config.simulation.fixed_step = 0.001;
end

%% ==================== 使用说明面板 ====================
function createInstructionPanel(parent)
    panel = uipanel(parent, 'Title', '工作流程说明', ...
                    'Position', [0.02 0.02 0.96 0.96]);
    
    instructionText = sprintf([...
        '【重要】正确的工作流程\n\n' ...
        '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n' ...
        '第一步：预配置（本界面）\n' ...
        '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n' ...
        '  • 定义拓扑结构（分枝数量和层级关系）\n' ...
        '  • 定义几何参数（长度、直径）\n' ...
        '  • 定义质量参数（总质量、分配比例）\n' ...
        '  • 定义果实物理属性和挂果位置\n' ...
        '  • 定义仿真参数（时间、步长）\n' ...
        '  • 定义信号处理参数（采样率、滤波器）\n\n' ...
        '  ⚠️ 注意：刚度(k)和阻尼(c)不在此处配置！\n' ...
        '     这些参数将由参数识别代码从实验数据中获取。\n\n' ...
        '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n' ...
        '第二步：参数识别\n' ...
        '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n' ...
        '  • 运行 analyse_chibi_data 代码\n' ...
        '  • 从锤击试验数据中识别：\n' ...
        '    - 线性参数：质量矩阵M、刚度矩阵K、阻尼矩阵C\n' ...
        '    - 固有频率和阻尼比\n' ...
        '    - 非线性参数：k3系数、c2系数\n' ...
        '  • 结果保存到 IdentifiedParameters.mat\n\n' ...
        '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n' ...
        '第三步：仿真\n' ...
        '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n' ...
        '  • 运行 Build_Extended_MDOF_model 代码\n' ...
        '  • 读取预配置（拓扑、几何、质量）\n' ...
        '  • 读取识别结果（刚度、阻尼）\n' ...
        '  • 构建Simulink模型并仿真\n' ...
        '  • 分析果实脱落效果\n\n' ...
        '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n' ...
        '果实配置说明（新逻辑）\n' ...
        '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n' ...
        '  • 二级分枝：mid段和tip段都可挂果\n' ...
        '  • 三级分枝：mid段和tip段都可挂果\n' ...
        '  • 一级分枝：不直接挂果（仅作为连接）\n' ...
        '  • 主干：不挂果\n']);
    
    uicontrol(panel, 'Style', 'text', ...
              'String', instructionText, ...
              'Units', 'normalized', ...
              'Position', [0.02 0.02 0.96 0.96], ...
              'HorizontalAlignment', 'left', ...
              'FontSize', 10, ...
              'FontName', 'FixedWidth');
end

%% ==================== 基础设置面板 ====================
function createBasicSettingsPanel(parent, config)
    panel = uipanel(parent, 'Title', '基础设置', ...
                    'Position', [0.02 0.02 0.96 0.96]);
    
    y = 0.85;
    dy = 0.12;
    
    % 工作目录
    uicontrol(panel, 'Style', 'text', 'String', '工作目录:', ...
              'Units', 'normalized', 'Position', [0.02 y 0.15 0.06], ...
              'HorizontalAlignment', 'left');
    uicontrol(panel, 'Style', 'edit', 'String', config.basic.workFolder, ...
              'Units', 'normalized', 'Position', [0.18 y 0.6 0.06], ...
              'Tag', 'edit_workFolder', 'HorizontalAlignment', 'left');
    uicontrol(panel, 'Style', 'pushbutton', 'String', '浏览...', ...
              'Units', 'normalized', 'Position', [0.8 y 0.15 0.06], ...
              'Callback', @(~,~) browseFolder(panel));
    
    y = y - dy;
    uicontrol(panel, 'Style', 'text', 'String', '模型名称:', ...
              'Units', 'normalized', 'Position', [0.02 y 0.15 0.06], ...
              'HorizontalAlignment', 'left');
    uicontrol(panel, 'Style', 'edit', 'String', config.basic.modelName, ...
              'Units', 'normalized', 'Position', [0.18 y 0.4 0.06], ...
              'Tag', 'edit_modelName');
    
    y = y - dy;
    uicontrol(panel, 'Style', 'text', 'String', '重力加速度 (m/s²):', ...
              'Units', 'normalized', 'Position', [0.02 y 0.2 0.06], ...
              'HorizontalAlignment', 'left');
    uicontrol(panel, 'Style', 'edit', 'String', num2str(config.basic.gravity_g), ...
              'Units', 'normalized', 'Position', [0.23 y 0.15 0.06], ...
              'Tag', 'edit_gravity');
    
    y = y - dy;
    uicontrol(panel, 'Style', 'checkbox', 'String', '使用并行计算', ...
              'Units', 'normalized', 'Position', [0.02 y 0.3 0.06], ...
              'Value', config.basic.useParallel, ...
              'Tag', 'check_parallel');
    uicontrol(panel, 'Style', 'text', 'String', '最大Worker数:', ...
              'Units', 'normalized', 'Position', [0.02 0.4 0.2 0.1], ...
              'HorizontalAlignment', 'left');
    uicontrol(panel, 'Style', 'edit', ...
              'String', num2str(config.basic.parallel_max_workers), ...
              'Units', 'normalized', 'Position', [0.23 0.4 0.1 0.1], ...
              'Tag', 'edit_parallel_workers');
end

%% ==================== 拓扑结构面板 ====================
function createTopologyPanel(parent, config)
    panel = uipanel(parent, 'Title', '分枝拓扑结构', ...
                    'Position', [0.02 0.02 0.96 0.96]);
    
    y = 0.88;
    dy = 0.1;
    
    uicontrol(panel, 'Style', 'text', 'String', '一级分枝数量:', ...
              'Units', 'normalized', 'Position', [0.02 y 0.2 0.06], ...
              'HorizontalAlignment', 'left');
    uicontrol(panel, 'Style', 'edit', ...
              'String', num2str(config.topology.num_primary_branches), ...
              'Units', 'normalized', 'Position', [0.23 y 0.1 0.06], ...
              'Tag', 'edit_numPrimary');
    
    y = y - dy;
    uicontrol(panel, 'Style', 'text', ...
              'String', '二级分枝数量 [P1下, P2下, P3下, ...]:', ...
              'Units', 'normalized', 'Position', [0.02 y 0.4 0.06], ...
              'HorizontalAlignment', 'left');
    uicontrol(panel, 'Style', 'edit', ...
              'String', mat2str(config.topology.secondary_branches_count), ...
              'Units', 'normalized', 'Position', [0.43 y 0.3 0.06], ...
              'Tag', 'edit_secondaryCount');
    
    y = y - dy;
    uicontrol(panel, 'Style', 'text', ...
              'String', '三级分枝数量 (用;分隔每个一级分枝):', ...
              'Units', 'normalized', 'Position', [0.02 y 0.4 0.06], ...
              'HorizontalAlignment', 'left');
    
    tertiary_str = '';
    for i = 1:length(config.topology.tertiary_branches_count)
        tertiary_str = [tertiary_str, mat2str(config.topology.tertiary_branches_count{i})];
        if i < length(config.topology.tertiary_branches_count)
            tertiary_str = [tertiary_str, '; '];
        end
    end
    uicontrol(panel, 'Style', 'edit', ...
              'String', tertiary_str, ...
              'Units', 'normalized', 'Position', [0.43 y 0.5 0.06], ...
              'Tag', 'edit_tertiaryCount');
    
    % 拓扑图示
    y = y - dy * 1.5;
    uicontrol(panel, 'Style', 'text', ...
              'String', sprintf([...
                  '拓扑结构示意（当前配置）:\n\n' ...
                  '主干 ─┬─ P1 ─┬─ P1_S1 ─── P1_S1_T1 [果]\n' ...
                  '      │      └─ P1_S2 ─┬─ P1_S2_T1 [果]\n' ...
                  '      │                └─ P1_S2_T2 [果]\n' ...
                  '      ├─ P2 ─── P2_S1 [果]\n' ...
                  '      └─ P3 ─┬─ P3_S1 ─── P3_S1_T1 [果]\n' ...
                  '             └─ P3_S2 [果]\n\n' ...
                  '[果] = 该分枝mid和tip位置挂果']), ...
              'Units', 'normalized', 'Position', [0.02 0.1 0.96 y-0.12], ...
              'HorizontalAlignment', 'left', 'FontName', 'FixedWidth', 'FontSize', 10);
end

%% ==================== 几何与质量参数面板 ====================
function createGeometryMassPanel(parent, config)
    panel = uipanel(parent, 'Title', '几何与质量参数（不包含刚度阻尼 - 由识别获得）', ...
                    'Position', [0.02 0.02 0.96 0.96]);
    
    % 主干参数
    trunkPanel = uipanel(panel, 'Title', '主干参数', ...
                         'Position', [0.02 0.7 0.96 0.28]);
    
    trunkParams = {
        '总质量 (kg):', 'edit_trunk_mass', num2str(config.trunk.total_mass);
        '长度 (m):', 'edit_trunk_length', num2str(config.trunk.length);
        '基部直径 (m):', 'edit_trunk_dBase', num2str(config.trunk.diameter_base);
        '顶部直径 (m):', 'edit_trunk_dTip', num2str(config.trunk.diameter_tip);
        'Z方向因子:', 'edit_trunk_zfactor', num2str(config.trunk.z_factor);
    };
    
    for i = 1:size(trunkParams, 1)
        col = mod(i-1, 3);
        row = floor((i-1) / 3);
        x = 0.02 + col * 0.33;
        y = 0.55 - row * 0.45;
        uicontrol(trunkPanel, 'Style', 'text', 'String', trunkParams{i,1}, ...
                  'Units', 'normalized', 'Position', [x y 0.15 0.3], ...
                  'HorizontalAlignment', 'left');
        uicontrol(trunkPanel, 'Style', 'edit', 'String', trunkParams{i,3}, ...
                  'Units', 'normalized', 'Position', [x+0.15 y 0.12 0.3], ...
                  'Tag', trunkParams{i,2});
    end
    
    % 质量分配
    uicontrol(trunkPanel, 'Style', 'text', 'String', '质量分配 [root,mid,tip]:', ...
              'Units', 'normalized', 'Position', [0.02 0.1 0.25 0.3], ...
              'HorizontalAlignment', 'left');
    uicontrol(trunkPanel, 'Style', 'edit', ...
              'String', mat2str(config.trunk.mass_distribution), ...
              'Units', 'normalized', 'Position', [0.28 0.1 0.25 0.3], ...
              'Tag', 'edit_trunk_massDist');
    
    % 分枝参数表格
    branchPanel = uipanel(panel, 'Title', '分枝几何与质量参数', ...
                          'Position', [0.02 0.02 0.96 0.65]);
    
    columnNames = {'分枝ID', '总质量(kg)', '长度(m)', '基部直径(m)', '顶部直径(m)', '质量分配[r,m,t]'};
    columnFormat = {'char', 'numeric', 'numeric', 'numeric', 'numeric', 'char'};
    columnEditable = [false, true, true, true, true, true];
    columnWidth = {80, 80, 70, 90, 90, 120};
    
    % 收集所有分枝数据
    data = {};
    
    % 一级分枝
    pFields = fieldnames(config.primary);
    for i = 1:length(pFields)
        p = config.primary.(pFields{i});
        data(end+1, :) = {pFields{i}, p.total_mass, p.length, ...
                          p.diameter_base, p.diameter_tip, mat2str(p.mass_dist)};
    end
    
    % 二级分枝
    sFields = fieldnames(config.secondary);
    for i = 1:length(sFields)
        s = config.secondary.(sFields{i});
        data(end+1, :) = {sFields{i}, s.total_mass, s.length, ...
                          s.diameter_base, s.diameter_tip, mat2str(s.mass_dist)};
    end
    
    % 三级分枝
    tFields = fieldnames(config.tertiary);
    for i = 1:length(tFields)
        t = config.tertiary.(tFields{i});
        data(end+1, :) = {tFields{i}, t.total_mass, t.length, ...
                          t.diameter_base, t.diameter_tip, mat2str(t.mass_dist)};
    end
    
    uitable(branchPanel, 'Data', data, ...
            'ColumnName', columnNames, ...
            'ColumnFormat', columnFormat, ...
            'ColumnEditable', columnEditable, ...
            'ColumnWidth', columnWidth, ...
            'Units', 'normalized', ...
            'Position', [0.02 0.05 0.96 0.9], ...
            'Tag', 'table_branches', ...
            'RowName', 'numbered');
end

%% ==================== 果实配置面板 ====================
function createFruitConfigPanel(parent, config)
    panel = uipanel(parent, 'Title', '果实配置', ...
                    'Position', [0.02 0.02 0.96 0.96]);
    
    % 果实物理属性
    physPanel = uipanel(panel, 'Title', '果实物理属性', ...
                        'Position', [0.02 0.55 0.96 0.43]);
    
    physParams = {
        '单果质量 (kg):', 'edit_fruit_mass', num2str(config.fruit.mass);
        '果实直径 (m):', 'edit_fruit_diameter', num2str(config.fruit.diameter);
        '果柄长度 (m):', 'edit_fruit_pedicel_length', num2str(config.fruit.pedicel_length);
        '果柄直径 (m):', 'edit_fruit_pedicel_diameter', num2str(config.fruit.pedicel_diameter);
        '平均断裂力 (N):', 'edit_fruit_Fbreak_mean', num2str(config.fruit.F_break_mean);
        '断裂力标准差 (N):', 'edit_fruit_Fbreak_std', num2str(config.fruit.F_break_std);
    };
    
    for i = 1:size(physParams, 1)
        col = mod(i-1, 3);
        row = floor((i-1) / 3);
        x = 0.02 + col * 0.33;
        y = 0.6 - row * 0.45;
        uicontrol(physPanel, 'Style', 'text', 'String', physParams{i,1}, ...
                  'Units', 'normalized', 'Position', [x y 0.2 0.25], ...
                  'HorizontalAlignment', 'left');
        uicontrol(physPanel, 'Style', 'edit', 'String', physParams{i,3}, ...
                  'Units', 'normalized', 'Position', [x+0.18 y 0.12 0.25], ...
                  'Tag', physParams{i,2});
    end
    
    % 挂果位置配置
    posPanel = uipanel(panel, 'Title', '挂果位置配置（新逻辑）', ...
                       'Position', [0.02 0.1 0.96 0.42]);
    
    uicontrol(posPanel, 'Style', 'text', ...
              'String', '二级分枝挂果位置:', ...
              'Units', 'normalized', 'Position', [0.02 0.7 0.2 0.15], ...
              'HorizontalAlignment', 'left', 'FontWeight', 'bold');
    
    uicontrol(posPanel, 'Style', 'checkbox', ...
              'String', 'Mid段挂果', ...
              'Units', 'normalized', 'Position', [0.25 0.7 0.15 0.15], ...
              'Value', config.fruit.attach_secondary_mid, ...
              'Tag', 'check_secondary_mid');
    
    uicontrol(posPanel, 'Style', 'checkbox', ...
              'String', 'Tip段挂果', ...
              'Units', 'normalized', 'Position', [0.42 0.7 0.15 0.15], ...
              'Value', config.fruit.attach_secondary_tip, ...
              'Tag', 'check_secondary_tip');
    
    uicontrol(posPanel, 'Style', 'text', ...
              'String', '三级分枝挂果位置:', ...
              'Units', 'normalized', 'Position', [0.02 0.45 0.2 0.15], ...
              'HorizontalAlignment', 'left', 'FontWeight', 'bold');
    
    uicontrol(posPanel, 'Style', 'checkbox', ...
              'String', 'Mid段挂果', ...
              'Units', 'normalized', 'Position', [0.25 0.45 0.15 0.15], ...
              'Value', config.fruit.attach_tertiary_mid, ...
              'Tag', 'check_tertiary_mid');
    
    uicontrol(posPanel, 'Style', 'checkbox', ...
              'String', 'Tip段挂果', ...
              'Units', 'normalized', 'Position', [0.42 0.45 0.15 0.15], ...
              'Value', config.fruit.attach_tertiary_tip, ...
              'Tag', 'check_tertiary_tip');
    
    uicontrol(posPanel, 'Style', 'text', 'String', '每节点挂果数:', ...
              'Units', 'normalized', 'Position', [0.02 0.2 0.2 0.15], ...
              'HorizontalAlignment', 'left');
    uicontrol(posPanel, 'Style', 'edit', ...
              'String', num2str(config.fruit.fruits_per_node), ...
              'Units', 'normalized', 'Position', [0.23 0.2 0.1 0.15], ...
              'Tag', 'edit_fruits_per_node');
    
    uicontrol(posPanel, 'Style', 'text', ...
              'String', '（提示：一级分枝不挂果，仅作为结构连接）', ...
              'Units', 'normalized', 'Position', [0.4 0.2 0.55 0.15], ...
              'HorizontalAlignment', 'left', 'FontAngle', 'italic');
end

%% ==================== 激励参数面板 ====================
function createExcitationPanel(parent, config)
    panel = uipanel(parent, 'Title', '激励参数', ...
                    'Position', [0.02 0.02 0.96 0.96]);
    
    % 激励类型
    uicontrol(panel, 'Style', 'text', 'String', '激励类型:', ...
              'Units', 'normalized', 'Position', [0.02 0.88 0.12 0.05], ...
              'HorizontalAlignment', 'left');
    
    bg = uibuttongroup(panel, 'Units', 'normalized', ...
                       'Position', [0.15 0.85 0.4 0.08], ...
                       'Tag', 'bg_excitationType');
    uicontrol(bg, 'Style', 'radiobutton', 'String', '正弦 (sine)', ...
              'Units', 'normalized', 'Position', [0.05 0.2 0.4 0.6], ...
              'Tag', 'radio_sine', ...
              'Value', strcmp(config.excitation.type, 'sine'));
    uicontrol(bg, 'Style', 'radiobutton', 'String', '脉冲 (impulse)', ...
              'Units', 'normalized', 'Position', [0.5 0.2 0.45 0.6], ...
              'Tag', 'radio_impulse', ...
              'Value', strcmp(config.excitation.type, 'impulse'));
    
    % 正弦参数
    sinePanel = uipanel(panel, 'Title', '正弦激励参数', ...
                        'Position', [0.02 0.45 0.46 0.38]);
    
    sineParams = {
        'Y幅值 (N):', 'edit_sine_ampY', num2str(config.excitation.sine_amplitude_y);
        'Z幅值 (N):', 'edit_sine_ampZ', num2str(config.excitation.sine_amplitude_z);
        '频率 (Hz):', 'edit_sine_freq', num2str(config.excitation.frequency_hz);
        'Y相位 (rad):', 'edit_sine_phaseY', num2str(config.excitation.phase_y_rad);
        'Z相位 (rad):', 'edit_sine_phaseZ', num2str(config.excitation.phase_z_rad);
    };
    
    y = 0.8;
    for i = 1:size(sineParams, 1)
        uicontrol(sinePanel, 'Style', 'text', 'String', sineParams{i,1}, ...
                  'Units', 'normalized', 'Position', [0.05 y 0.45 0.12], ...
                  'HorizontalAlignment', 'left');
        uicontrol(sinePanel, 'Style', 'edit', 'String', sineParams{i,3}, ...
                  'Units', 'normalized', 'Position', [0.52 y 0.4 0.12], ...
                  'Tag', sineParams{i,2});
        y = y - 0.16;
    end
    
    % 脉冲参数
    impulsePanel = uipanel(panel, 'Title', '脉冲激励参数', ...
                           'Position', [0.52 0.45 0.46 0.38]);
    
    impulseParams = {
        'Y峰值力 (N):', 'edit_impulse_gainY', num2str(config.excitation.impulse_gain_y);
        'Z峰值力 (N):', 'edit_impulse_gainZ', num2str(config.excitation.impulse_gain_z);
        '脉冲周期 (s):', 'edit_pulse_period', num2str(config.excitation.pulse_period_s);
        '脉宽 (%):', 'edit_pulse_width', num2str(config.excitation.pulse_width_percent);
        'Y延迟 (s):', 'edit_pulse_delayY', num2str(config.excitation.pulse_delay_y_s);
        'Z延迟 (s):', 'edit_pulse_delayZ', num2str(config.excitation.pulse_delay_z_s);
    };
    
    y = 0.85;
    for i = 1:size(impulseParams, 1)
        uicontrol(impulsePanel, 'Style', 'text', 'String', impulseParams{i,1}, ...
                  'Units', 'normalized', 'Position', [0.05 y 0.45 0.1], ...
                  'HorizontalAlignment', 'left');
        uicontrol(impulsePanel, 'Style', 'edit', 'String', impulseParams{i,3}, ...
                  'Units', 'normalized', 'Position', [0.52 y 0.4 0.1], ...
                  'Tag', impulseParams{i,2});
        y = y - 0.14;
    end
    
    % 时间窗口
    timePanel = uipanel(panel, 'Title', '激励时间窗口', ...
                        'Position', [0.02 0.02 0.96 0.4]);
    
    uicontrol(timePanel, 'Style', 'text', 'String', '开始时间 (s):', ...
              'Units', 'normalized', 'Position', [0.05 0.65 0.15 0.2], ...
              'HorizontalAlignment', 'left');
    uicontrol(timePanel, 'Style', 'edit', ...
              'String', num2str(config.excitation.start_time), ...
              'Units', 'normalized', 'Position', [0.22 0.65 0.12 0.2], ...
              'Tag', 'edit_excite_start');
    
    uicontrol(timePanel, 'Style', 'text', 'String', '结束时间 (s):', ...
              'Units', 'normalized', 'Position', [0.4 0.65 0.15 0.2], ...
              'HorizontalAlignment', 'left');
    uicontrol(timePanel, 'Style', 'edit', ...
              'String', num2str(config.excitation.end_time), ...
              'Units', 'normalized', 'Position', [0.57 0.65 0.12 0.2], ...
              'Tag', 'edit_excite_end');
    
    uicontrol(timePanel, 'Style', 'text', ...
              'String', sprintf(['提示：\n' ...
                  '• 激励频率可在参数识别后自动更新为第一阶固有频率\n' ...
                  '• 激励时间窗口应确保脉冲发生时刻在范围内']), ...
              'Units', 'normalized', 'Position', [0.05 0.1 0.9 0.45], ...
              'HorizontalAlignment', 'left');
end

%% ==================== 仿真参数面板 ====================
function createSimulationPanel(parent, config)
    panel = uipanel(parent, 'Title', '仿真控制参数', ...
                    'Position', [0.02 0.02 0.96 0.96]);
    
    y = 0.8;
    dy = 0.12;
    
    uicontrol(panel, 'Style', 'text', 'String', '仿真停止时间 (s):', ...
              'Units', 'normalized', 'Position', [0.05 y 0.25 0.06], ...
              'HorizontalAlignment', 'left');
    uicontrol(panel, 'Style', 'edit', ...
              'String', num2str(config.simulation.stop_time), ...
              'Units', 'normalized', 'Position', [0.32 y 0.15 0.06], ...
              'Tag', 'edit_sim_stop');
    
    y = y - dy;
    uicontrol(panel, 'Style', 'text', 'String', '求解器固定步长 (s):', ...
              'Units', 'normalized', 'Position', [0.05 y 0.25 0.06], ...
              'HorizontalAlignment', 'left');
    uicontrol(panel, 'Style', 'edit', ...
              'String', num2str(config.simulation.fixed_step), ...
              'Units', 'normalized', 'Position', [0.32 y 0.15 0.06], ...
              'Tag', 'edit_sim_step');
end

%% ==================== 信号处理参数面板 ====================
function createSignalProcessingPanel(parent, config)
    panel = uipanel(parent, 'Title', '信号处理参数（用于参数识别阶段）', ...
                    'Position', [0.02 0.02 0.96 0.96]);
    
    % 采样与滤波
    filterPanel = uipanel(panel, 'Title', '采样与滤波', ...
                          'Position', [0.02 0.5 0.96 0.48]);
    
    filterParams = {
        '目标采样率 (Hz):', 'edit_fs', num2str(config.signal.fs_target);
        '滤波截止频率 (Hz):', 'edit_cutoff', num2str(config.signal.cutoff_freq);
        '滤波器阶数:', 'edit_filterOrder', num2str(config.signal.filter_order);
        'FFT点数:', 'edit_nfft', num2str(config.signal.nfft);
    };
    
    y = 0.75;
    for i = 1:size(filterParams, 1)
        uicontrol(filterPanel, 'Style', 'text', 'String', filterParams{i,1}, ...
                  'Units', 'normalized', 'Position', [0.05 y 0.3 0.15], ...
                  'HorizontalAlignment', 'left');
        uicontrol(filterPanel, 'Style', 'edit', 'String', filterParams{i,3}, ...
                  'Units', 'normalized', 'Position', [0.35 y 0.2 0.15], ...
                  'Tag', filterParams{i,2});
        y = y - 0.22;
    end
    
    % 频率分析
    analysisPanel = uipanel(panel, 'Title', '频率分析范围', ...
                            'Position', [0.02 0.02 0.96 0.45]);
    
    uicontrol(analysisPanel, 'Style', 'text', 'String', '分析频率范围:', ...
              'Units', 'normalized', 'Position', [0.05 0.65 0.2 0.2], ...
              'HorizontalAlignment', 'left');
    uicontrol(analysisPanel, 'Style', 'edit', ...
              'String', num2str(config.signal.freq_range_min), ...
              'Units', 'normalized', 'Position', [0.26 0.65 0.1 0.2], ...
              'Tag', 'edit_freqMin');
    uicontrol(analysisPanel, 'Style', 'text', 'String', ' ~ ', ...
              'Units', 'normalized', 'Position', [0.37 0.65 0.05 0.2]);
    uicontrol(analysisPanel, 'Style', 'edit', ...
              'String', num2str(config.signal.freq_range_max), ...
              'Units', 'normalized', 'Position', [0.42 0.65 0.1 0.2], ...
              'Tag', 'edit_freqMax');
    uicontrol(analysisPanel, 'Style', 'text', 'String', 'Hz', ...
              'Units', 'normalized', 'Position', [0.53 0.65 0.05 0.2]);
    
    uicontrol(analysisPanel, 'Style', 'text', 'String', 'SNR阈值 (dB):', ...
              'Units', 'normalized', 'Position', [0.05 0.35 0.2 0.2], ...
              'HorizontalAlignment', 'left');
    uicontrol(analysisPanel, 'Style', 'edit', ...
              'String', num2str(config.signal.snr_threshold), ...
              'Units', 'normalized', 'Position', [0.26 0.35 0.1 0.2], ...
              'Tag', 'edit_snrThreshold');
end

%% ==================== 回调函数 ====================
function browseFolder(panel)
    folder = uigetdir(pwd, '选择工作目录');
    if folder ~= 0
        h = findobj(panel, 'Tag', 'edit_workFolder');
        set(h, 'String', folder);
    end
end

function previewTopology(fig)
    % 预览拓扑结构图
    try
        config = collectAllParameters(fig);
        if isempty(config), return; end
        
        figure('Name', '拓扑结构预览', 'NumberTitle', 'off');
        hold on;
        
        % 简单绘制树状结构
        drawTreeTopology(config);
        
        title('果树拓扑结构预览');
        axis equal;
        axis off;
        hold off;
    catch ME
        errordlg(['预览失败: ' ME.message], '错误');
    end
end

function drawTreeTopology(config)
    % 绘制简化的树状拓扑图
    
    % 主干
    plot([0 0], [0 3], 'k-', 'LineWidth', 8);
    text(0.1, 1.5, '主干', 'FontSize', 10);
    
    % 一级分枝
    numP = config.topology.num_primary_branches;
    pAngles = linspace(30, 150, numP);
    
    for p = 1:numP
        angle = pAngles(p) * pi / 180;
        px = 2 * cos(angle);
        py = 2.5 + 0.5 * sin(angle);
        
        plot([0 px], [2.5 py], 'b-', 'LineWidth', 4);
        text(px, py + 0.2, sprintf('P%d', p), 'FontSize', 9, 'Color', 'b');
        
        % 二级分枝
        numS = config.topology.secondary_branches_count(p);
        for s = 1:numS
            sx = px + 0.8 * cos(angle - 0.3 + 0.3*s);
            sy = py + 0.5;
            
            plot([px sx], [py sy], 'g-', 'LineWidth', 2);
            text(sx, sy + 0.15, sprintf('S%d', s), 'FontSize', 8, 'Color', [0 0.6 0]);
            
            % 标记挂果位置
            plot(sx, sy, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
            
            % 三级分枝
            if p <= length(config.topology.tertiary_branches_count)
                numT = config.topology.tertiary_branches_count{p};
                if s <= length(numT) && numT(s) > 0
                    for t = 1:numT(s)
                        tx = sx + 0.4 * cos(angle + 0.2*t);
                        ty = sy + 0.3;
                        plot([sx tx], [sy ty], 'm-', 'LineWidth', 1);
                        plot(tx, ty, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
                    end
                end
            end
        end
    end
    
    % 图例
    plot(NaN, NaN, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    legend('挂果位置', 'Location', 'southeast');
end

function loadConfig(fig)
    [filename, pathname] = uigetfile('*.mat', '选择配置文件');
    if filename ~= 0
        try
            S = load(fullfile(pathname, filename));
            if isfield(S, 'preConfig')
                updateUIFromConfig(fig, S.preConfig);
                msgbox('配置加载成功!', '成功');
            else
                errordlg('无效的配置文件', '错误');
            end
        catch ME
            errordlg(['加载失败: ' ME.message], '错误');
        end
    end
end

function saveConfig(fig)
    config = collectAllParameters(fig);
    if isempty(config), return; end
    
    [filename, pathname] = uiputfile('*.mat', '保存配置', 'tree_preconfig.mat');
    if filename ~= 0
        preConfig = config;
        save(fullfile(pathname, filename), 'preConfig');
        msgbox('配置保存成功!', '成功');
    end
end

function resetToDefault(fig)
    answer = questdlg('确定恢复默认值?', '确认', '确定', '取消', '取消');
    if strcmp(answer, '确定')
        defaultConfig = getDefaultConfig();
        updateUIFromConfig(fig, defaultConfig);
        msgbox('已恢复默认值', '成功');
    end
end

function confirmConfig(fig)
    config = collectAllParameters(fig);
    if isempty(config), return; end
    
    [valid, msg] = validateConfig(config);
    if ~valid
        errordlg(sprintf('配置验证失败:\n%s', msg), '验证错误');
        return;
    end
    
    userData = get(fig, 'UserData');
    userData.confirmed = true;
    set(fig, 'UserData', userData);
    uiresume(fig);
end

function cancelConfig(fig)
    userData = get(fig, 'UserData');
    userData.confirmed = false;
    set(fig, 'UserData', userData);
    uiresume(fig);
end

function onClose(fig, ~)
    userData = get(fig, 'UserData');
    userData.confirmed = false;
    set(fig, 'UserData', userData);
    uiresume(fig);
end

%% ==================== 参数收集函数 ====================
function config = collectAllParameters(fig)
    try
        config = struct();
        
        % 基础设置
        config.basic.workFolder = getEditValue(fig, 'edit_workFolder', 'string');
        config.basic.modelName = getEditValue(fig, 'edit_modelName', 'string');
        config.basic.gravity_g = getEditValue(fig, 'edit_gravity', 'double');
        config.basic.useParallel = getCheckValue(fig, 'check_parallel');
        parallel_workers = getEditValue(fig, 'edit_parallel_workers', 'double');
        if isnan(parallel_workers) || parallel_workers < 1
            error('BranchConfigGUI:InvalidInput', '并行Worker数必须是大于0的整数');
        end
        config.basic.parallel_max_workers = round(parallel_workers);
        
        % 信号处理
        config.signal.fs_target = getEditValue(fig, 'edit_fs', 'double');
        config.signal.cutoff_freq = getEditValue(fig, 'edit_cutoff', 'double');
        config.signal.filter_order = getEditValue(fig, 'edit_filterOrder', 'double');
        config.signal.nfft = getEditValue(fig, 'edit_nfft', 'double');
        config.signal.freq_range_min = getEditValue(fig, 'edit_freqMin', 'double');
        config.signal.freq_range_max = getEditValue(fig, 'edit_freqMax', 'double');
        config.signal.snr_threshold = getEditValue(fig, 'edit_snrThreshold', 'double');
        
        % 拓扑
        config.topology.num_primary_branches = getEditValue(fig, 'edit_numPrimary', 'double');
        config.topology.secondary_branches_count = eval(getEditValue(fig, 'edit_secondaryCount', 'string'));
        
        tertiaryStr = getEditValue(fig, 'edit_tertiaryCount', 'string');
        parts = strsplit(tertiaryStr, ';');
        config.topology.tertiary_branches_count = cell(1, length(parts));
        for i = 1:length(parts)
            config.topology.tertiary_branches_count{i} = eval(strtrim(parts{i}));
        end
        
        % 主干
        config.trunk.total_mass = getEditValue(fig, 'edit_trunk_mass', 'double');
        config.trunk.length = getEditValue(fig, 'edit_trunk_length', 'double');
        config.trunk.diameter_base = getEditValue(fig, 'edit_trunk_dBase', 'double');
        config.trunk.diameter_tip = getEditValue(fig, 'edit_trunk_dTip', 'double');
        config.trunk.z_factor = getEditValue(fig, 'edit_trunk_zfactor', 'double');
        config.trunk.mass_distribution = eval(getEditValue(fig, 'edit_trunk_massDist', 'string'));
        
        % 分枝参数（从表格）
        hTable = findobj(fig, 'Tag', 'table_branches');
        if ~isempty(hTable)
            tableData = get(hTable, 'Data');
            config.branches = struct();
            for i = 1:size(tableData, 1)
                branchId = tableData{i, 1};
                config.branches.(branchId) = struct(...
                    'total_mass', tableData{i, 2}, ...
                    'length', tableData{i, 3}, ...
                    'diameter_base', tableData{i, 4}, ...
                    'diameter_tip', tableData{i, 5}, ...
                    'mass_dist', eval(tableData{i, 6}));
            end
        end
        
        % 果实参数
        config.fruit.mass = getEditValue(fig, 'edit_fruit_mass', 'double');
        config.fruit.diameter = getEditValue(fig, 'edit_fruit_diameter', 'double');
        config.fruit.pedicel_length = getEditValue(fig, 'edit_fruit_pedicel_length', 'double');
        config.fruit.pedicel_diameter = getEditValue(fig, 'edit_fruit_pedicel_diameter', 'double');
        config.fruit.F_break_mean = getEditValue(fig, 'edit_fruit_Fbreak_mean', 'double');
        config.fruit.F_break_std = getEditValue(fig, 'edit_fruit_Fbreak_std', 'double');
        config.fruit.attach_secondary_mid = getCheckValue(fig, 'check_secondary_mid');
        config.fruit.attach_secondary_tip = getCheckValue(fig, 'check_secondary_tip');
        config.fruit.attach_tertiary_mid = getCheckValue(fig, 'check_tertiary_mid');
        config.fruit.attach_tertiary_tip = getCheckValue(fig, 'check_tertiary_tip');
        config.fruit.fruits_per_node = getEditValue(fig, 'edit_fruits_per_node', 'double');
        
        % 激励参数
        bg = findobj(fig, 'Tag', 'bg_excitationType');
        if ~isempty(bg)
            selectedBtn = get(bg, 'SelectedObject');
            if ~isempty(selectedBtn) && strcmp(get(selectedBtn, 'Tag'), 'radio_impulse')
                config.excitation.type = 'impulse';
            else
                config.excitation.type = 'sine';
            end
        else
            config.excitation.type = 'impulse';
        end
        
        config.excitation.sine_amplitude_y = getEditValue(fig, 'edit_sine_ampY', 'double');
        config.excitation.sine_amplitude_z = getEditValue(fig, 'edit_sine_ampZ', 'double');
        config.excitation.frequency_hz = getEditValue(fig, 'edit_sine_freq', 'double');
        config.excitation.phase_y_rad = getEditValue(fig, 'edit_sine_phaseY', 'double');
        config.excitation.phase_z_rad = getEditValue(fig, 'edit_sine_phaseZ', 'double');
        config.excitation.impulse_gain_y = getEditValue(fig, 'edit_impulse_gainY', 'double');
        config.excitation.impulse_gain_z = getEditValue(fig, 'edit_impulse_gainZ', 'double');
        config.excitation.pulse_period_s = getEditValue(fig, 'edit_pulse_period', 'double');
        config.excitation.pulse_width_percent = getEditValue(fig, 'edit_pulse_width', 'double');
        config.excitation.pulse_delay_y_s = getEditValue(fig, 'edit_pulse_delayY', 'double');
        config.excitation.pulse_delay_z_s = getEditValue(fig, 'edit_pulse_delayZ', 'double');
        config.excitation.start_time = getEditValue(fig, 'edit_excite_start', 'double');
        config.excitation.end_time = getEditValue(fig, 'edit_excite_end', 'double');
        
        % 仿真
        config.simulation.stop_time = getEditValue(fig, 'edit_sim_stop', 'double');
        config.simulation.fixed_step = getEditValue(fig, 'edit_sim_step', 'double');
        
    catch ME
        errordlg(['收集参数失败: ' ME.message], '错误');
        config = [];
    end
end

function val = getEditValue(fig, tag, type)
    h = findobj(fig, 'Tag', tag);
    if isempty(h)
        if strcmp(type, 'double')
            val = 0;
        else
            val = '';
        end
        return;
    end
    str = get(h(1), 'String');
    if strcmp(type, 'double')
        val = str2double(str);
    else
        val = str;
    end
end

function val = getCheckValue(fig, tag)
    h = findobj(fig, 'Tag', tag);
    if isempty(h)
        val = false;
        return;
    end
    val = get(h(1), 'Value') == 1;
end

%% ==================== 从配置更新UI ====================
function updateUIFromConfig(fig, config)
    % 从配置结构体更新所有UI控件的值
    
    % --- 基础设置 ---
    setEditValue(fig, 'edit_workFolder', config.basic.workFolder);
    setEditValue(fig, 'edit_modelName', config.basic.modelName);
    setEditValue(fig, 'edit_gravity', num2str(config.basic.gravity_g));
    setCheckValue(fig, 'check_parallel', config.basic.useParallel);
    
    % --- 信号处理参数 ---
    setEditValue(fig, 'edit_fs', num2str(config.signal.fs_target));
    setEditValue(fig, 'edit_cutoff', num2str(config.signal.cutoff_freq));
    setEditValue(fig, 'edit_filterOrder', num2str(config.signal.filter_order));
    setEditValue(fig, 'edit_nfft', num2str(config.signal.nfft));
    setEditValue(fig, 'edit_freqMin', num2str(config.signal.freq_range_min));
    setEditValue(fig, 'edit_freqMax', num2str(config.signal.freq_range_max));
    setEditValue(fig, 'edit_snrThreshold', num2str(config.signal.snr_threshold));
    
    % --- 拓扑结构 ---
    setEditValue(fig, 'edit_numPrimary', num2str(config.topology.num_primary_branches));
    setEditValue(fig, 'edit_secondaryCount', mat2str(config.topology.secondary_branches_count));
    
    % 三级分枝数量转换为字符串格式
    tertiaryParts = cell(1, length(config.topology.tertiary_branches_count));
    for i = 1:length(config.topology.tertiary_branches_count)
        tertiaryParts{i} = mat2str(config.topology.tertiary_branches_count{i});
    end
    setEditValue(fig, 'edit_tertiaryCount', strjoin(tertiaryParts, '; '));
    
    % --- 主干参数 ---
    setEditValue(fig, 'edit_trunk_mass', num2str(config.trunk.total_mass));
    setEditValue(fig, 'edit_trunk_length', num2str(config.trunk.length));
    setEditValue(fig, 'edit_trunk_dBase', num2str(config.trunk.diameter_base));
    setEditValue(fig, 'edit_trunk_dTip', num2str(config.trunk.diameter_tip));
    setEditValue(fig, 'edit_trunk_zfactor', num2str(config.trunk.z_factor));
    setEditValue(fig, 'edit_trunk_massDist', mat2str(config.trunk.mass_distribution));
    
    % --- 分枝参数表格 ---
    hTable = findobj(fig, 'Tag', 'table_branches');
    if ~isempty(hTable)
        tableData = {};
        
        % 一级分枝
        if isfield(config, 'primary') && isstruct(config.primary)
            pFields = fieldnames(config.primary);
            for i = 1:length(pFields)
                p = config.primary.(pFields{i});
                tableData(end+1, :) = {pFields{i}, p.total_mass, p.length, ...
                                       p.diameter_base, p.diameter_tip, mat2str(p.mass_dist)};
            end
        end
        
        % 二级分枝
        if isfield(config, 'secondary') && isstruct(config.secondary)
            sFields = fieldnames(config.secondary);
            for i = 1:length(sFields)
                s = config.secondary.(sFields{i});
                tableData(end+1, :) = {sFields{i}, s.total_mass, s.length, ...
                                       s.diameter_base, s.diameter_tip, mat2str(s.mass_dist)};
            end
        end
        
        % 三级分枝
        if isfield(config, 'tertiary') && isstruct(config.tertiary)
            tFields = fieldnames(config.tertiary);
            for i = 1:length(tFields)
                t = config.tertiary.(tFields{i});
                tableData(end+1, :) = {tFields{i}, t.total_mass, t.length, ...
                                       t.diameter_base, t.diameter_tip, mat2str(t.mass_dist)};
            end
        end
        
        set(hTable, 'Data', tableData);
    end
    
    % --- 果实参数 ---
    setEditValue(fig, 'edit_fruit_mass', num2str(config.fruit.mass));
    setEditValue(fig, 'edit_fruit_diameter', num2str(config.fruit.diameter));
    setEditValue(fig, 'edit_fruit_pedicel_length', num2str(config.fruit.pedicel_length));
    setEditValue(fig, 'edit_fruit_pedicel_diameter', num2str(config.fruit.pedicel_diameter));
    setEditValue(fig, 'edit_fruit_Fbreak_mean', num2str(config.fruit.F_break_mean));
    setEditValue(fig, 'edit_fruit_Fbreak_std', num2str(config.fruit.F_break_std));
    
    % 挂果位置配置
    setCheckValue(fig, 'check_secondary_mid', config.fruit.attach_secondary_mid);
    setCheckValue(fig, 'check_secondary_tip', config.fruit.attach_secondary_tip);
    setCheckValue(fig, 'check_tertiary_mid', config.fruit.attach_tertiary_mid);
    setCheckValue(fig, 'check_tertiary_tip', config.fruit.attach_tertiary_tip);
    setEditValue(fig, 'edit_fruits_per_node', num2str(config.fruit.fruits_per_node));
    
    % --- 激励参数 ---
    % 激励类型单选按钮
    bg = findobj(fig, 'Tag', 'bg_excitationType');
    if ~isempty(bg)
        if strcmp(config.excitation.type, 'sine')
            btnTag = 'radio_sine';
        else
            btnTag = 'radio_impulse';
        end
        btn = findobj(fig, 'Tag', btnTag);
        if ~isempty(btn)
            set(bg, 'SelectedObject', btn);
        end
    end
    
    setEditValue(fig, 'edit_sine_amp_y', num2str(config.excitation.sine_amplitude_y));
    setEditValue(fig, 'edit_sine_amp_z', num2str(config.excitation.sine_amplitude_z));
    setEditValue(fig, 'edit_frequency', num2str(config.excitation.frequency_hz));
    setEditValue(fig, 'edit_phase_y', num2str(config.excitation.phase_y_rad));
    setEditValue(fig, 'edit_phase_z', num2str(config.excitation.phase_z_rad));
    setEditValue(fig, 'edit_impulse_gain_y', num2str(config.excitation.impulse_gain_y));
    setEditValue(fig, 'edit_impulse_gain_z', num2str(config.excitation.impulse_gain_z));
    setEditValue(fig, 'edit_pulse_period', num2str(config.excitation.pulse_period_s));
    setEditValue(fig, 'edit_pulse_width', num2str(config.excitation.pulse_width_percent));
    setEditValue(fig, 'edit_pulse_delay_y', num2str(config.excitation.pulse_delay_y_s));
    setEditValue(fig, 'edit_pulse_delay_z', num2str(config.excitation.pulse_delay_z_s));
    
    if isfield(config.excitation, 'start_time')
        setEditValue(fig, 'edit_excite_start', num2str(config.excitation.start_time));
    end
    if isfield(config.excitation, 'end_time')
        setEditValue(fig, 'edit_excite_end', num2str(config.excitation.end_time));
    end
    
    % --- 仿真参数 ---
    if isfield(config, 'simulation')
        setEditValue(fig, 'edit_stopTime', num2str(config.simulation.stop_time));
        setEditValue(fig, 'edit_fixedStep', num2str(config.simulation.fixed_step));
    end
end

%% ==================== 设置UI控件值的辅助函数 ====================
function setEditValue(fig, tag, value)
    h = findobj(fig, 'Tag', tag);
    if ~isempty(h)
        set(h, 'String', value);
    end
end

function setCheckValue(fig, tag, value)
    h = findobj(fig, 'Tag', tag);
    if ~isempty(h)
        set(h, 'Value', value);
    end
end

function [valid, msg] = validateConfig(config)
    valid = true;
    msg = '';
    
    % 验证拓扑
    numP = config.topology.num_primary_branches;
    if length(config.topology.secondary_branches_count) ~= numP
        valid = false;
        msg = [msg '二级分枝数量与一级分枝数量不匹配\n'];
    end
    
    % 验证时间
    if config.excitation.end_time >= config.simulation.stop_time
        valid = false;
        msg = [msg '激励结束时间必须小于仿真停止时间\n'];
    end
    
    % 验证正数
    if config.trunk.total_mass <= 0
        valid = false;
        msg = [msg '主干质量必须为正数\n'];
    end
    
    % 验证质量分配
    if abs(sum(config.trunk.mass_distribution) - 1) > 0.01
        valid = false;
        msg = [msg '主干质量分配之和必须为1\n'];
    end
end

%% ==================== 动态生成默认分枝参数 ====================
function [primary, secondary, tertiary] = generateDefaultBranchParams(num_primary, secondary_count, tertiary_count)
    % 根据拓扑配置动态生成默认的分枝几何与质量参数
    % 这避免了硬编码特定数量的分枝
    
    primary = struct();
    secondary = struct();
    tertiary = struct();
    
    % 基础参数模板（可根据分枝级别缩放）
    base_mass_p = 5.0;        % 一级分枝基础质量
    base_length_p = 0.5;      % 一级分枝基础长度
    base_diam_base_p = 0.045; % 一级分枝基础直径
    base_diam_tip_p = 0.028;  % 一级分枝尖端直径
    
    base_mass_s = 2.0;        % 二级分枝基础质量
    base_length_s = 0.35;     % 二级分枝基础长度
    base_diam_base_s = 0.025; % 二级分枝基础直径
    base_diam_tip_s = 0.015;  % 二级分枝尖端直径
    
    base_mass_t = 0.5;        % 三级分枝基础质量
    base_length_t = 0.25;     % 三级分枝基础长度
    base_diam_base_t = 0.012; % 三级分枝基础直径
    base_diam_tip_t = 0.006;  % 三级分枝尖端直径
    
    default_mass_dist = [0.5, 0.3, 0.2];
    
    % 生成一级分枝参数
    for p = 1:num_primary
        branch_id = sprintf('P%d', p);
        variation = 0.8 + 0.4 * rand();
        primary.(branch_id) = struct(...
            'total_mass', base_mass_p * variation, ...
            'length', base_length_p * (0.9 + 0.2 * rand()), ...
            'diameter_base', base_diam_base_p * variation, ...
            'diameter_tip', base_diam_tip_p * variation, ...
            'mass_dist', default_mass_dist);
    end
    
    % 生成二级分枝参数
    for p = 1:num_primary
        num_s = secondary_count(p);
        for s = 1:num_s
            branch_id = sprintf('P%d_S%d', p, s);
            variation = 0.7 + 0.6 * rand();
            secondary.(branch_id) = struct(...
                'total_mass', base_mass_s * variation, ...
                'length', base_length_s * (0.85 + 0.3 * rand()), ...
                'diameter_base', base_diam_base_s * variation, ...
                'diameter_tip', base_diam_tip_s * variation, ...
                'mass_dist', default_mass_dist);
        end
    end
    
    % 生成三级分枝参数
    for p = 1:num_primary
        if p <= length(tertiary_count)
            tertiary_for_p = tertiary_count{p};
            num_s = secondary_count(p);
            for s = 1:num_s
                if s <= length(tertiary_for_p)
                    num_t = tertiary_for_p(s);
                    for t = 1:num_t
                        branch_id = sprintf('P%d_S%d_T%d', p, s, t);
                        variation = 0.6 + 0.8 * rand();
                        tertiary.(branch_id) = struct(...
                            'total_mass', base_mass_t * variation, ...
                            'length', base_length_t * (0.8 + 0.4 * rand()), ...
                            'diameter_base', base_diam_base_t * variation, ...
                            'diameter_tip', base_diam_tip_t * variation, ...
                            'mass_dist', default_mass_dist);
                    end
                end
            end
        end
    end
end