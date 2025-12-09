%% 多自由度分层振动模型动态生成与仿真分析脚本
% 脚本功能:
% 1. 动态创建具有分层树状结构的多自由度振动Simulink模型。
% 2. 定义激励参数和仿真参数。
% 3. 根据拓扑结构和预定义参数填充模型。
% 4. 支持对模型中不同质量点施加激励，并进行批量仿真。
% 5. 分析仿真结果，评估不同激励点下的果实脱落效果，找出最佳激励点。
% 该版本最终完善了能想到的所有问题！果实的脱落正常，分枝参数正常！

%% --- 脚本初始化 ---
% 关闭所有已打开的图形窗口，防止干扰
close all;
% 清除命令行窗口的显示内容
clc;
disp('=== 脚本初始化完成：已关闭图形窗口、清空工作空间和命令行 ===');
disp(newline); % 增加空行，使输出更易读

%% === 检测预配置参数 ===
if ~evalin('base', 'exist(''params_from_gui'', ''var'')') || ~evalin('base', 'params_from_gui')
    error(['错误：未检测到GUI预配置参数！\n' ...
           '请先运行 IntegratedTreeVibrationAnalysis.m 进行预配置后再运行此脚本。\n' ...
           '或者确保以下变量已存在于工作区：\n' ...
           '  - params_from_gui = true\n' ...
           '  - config (拓扑配置)\n' ...
           '  - params_struct (含主干参数)\n' ...
           '  - predefined_params (分枝参数)\n' ...
           '  - fruit_config (果实配置)\n' ...
           '  - default_fruit_params (默认果实参数)\n' ...
           '  - 所有激励参数和仿真控制参数']);
end

fprintf('═══════════════════════════════════════════════════════\n');
fprintf('检测到GUI预配置参数，正在加载预设值...\n');
fprintf('═══════════════════════════════════════════════════════\n\n');

required_vars = {'config', 'params_struct', 'predefined_params', 'fruit_config', ...
                 'default_fruit_params', 'gravity_g', 'sim_stop_time', 'sim_fixed_step', ...
                 'excitation_type', 'F_excite_y_amplitude', 'F_excite_z_amplitude', ...
                 'excitation_frequency_hz', 'excitation_phase_y_rad', 'excitation_phase_z_rad', ...
                 'impulse_force_gain_y', 'impulse_force_gain_z', 'pulse_period_s', ...
                 'pulse_width_percent', 'pulse_phase_delay_y_s', 'pulse_phase_delay_z_s', ...
                 'excitation_start_time', 'excitation_end_time'};

missing_vars = {};
for i_var = 1:length(required_vars)
    if ~evalin('base', sprintf('exist(''%s'', ''var'')', required_vars{i_var}))
        missing_vars{end+1} = required_vars{i_var};
    end
end

if ~isempty(missing_vars)
    error('缺少必需的工作区变量:\n  %s\n请先运行预配置程序。', strjoin(missing_vars, '\n  '));
end

fprintf('所有必需参数验证通过。\n\n');

config = evalin('base', 'config');
params_struct = evalin('base', 'params_struct');
predefined_params = evalin('base', 'predefined_params');
fruit_config = evalin('base', 'fruit_config');
default_fruit_params = evalin('base', 'default_fruit_params');
gravity_g = evalin('base', 'gravity_g');
sim_stop_time = evalin('base', 'sim_stop_time');
sim_fixed_step = evalin('base', 'sim_fixed_step');
excitation_type = evalin('base', 'excitation_type');
F_excite_y_amplitude = evalin('base', 'F_excite_y_amplitude');
F_excite_z_amplitude = evalin('base', 'F_excite_z_amplitude');
excitation_frequency_hz = evalin('base', 'excitation_frequency_hz');
excitation_phase_y_rad = evalin('base', 'excitation_phase_y_rad');
excitation_phase_z_rad = evalin('base', 'excitation_phase_z_rad');
impulse_force_gain_y = evalin('base', 'impulse_force_gain_y');
impulse_force_gain_z = evalin('base', 'impulse_force_gain_z');
pulse_period_s = evalin('base', 'pulse_period_s');
pulse_width_percent = evalin('base', 'pulse_width_percent');
pulse_phase_delay_y_s = evalin('base', 'pulse_phase_delay_y_s');
pulse_phase_delay_z_s = evalin('base', 'pulse_phase_delay_z_s');
excitation_start_time = evalin('base', 'excitation_start_time');
excitation_end_time = evalin('base', 'excitation_end_time');

% 指定文件夹路径
if use_gui_params && exist('workFolder', 'var') && ~isempty(workFolder)
    folderPath = workFolder;
else
    folderPath = pwd;  % 使用当前目录
end

% 确保目录存在
if ~exist(folderPath, 'dir')
    mkdir(folderPath);
end

fileName = 'MDOF_Hierarchical_Vibration_Sim.slx';                % 要查找并删除的文件名
filePath = fullfile(folderPath, fileName);  % 构造完整路径

% 检查文件是否存在
if exist(filePath, 'file') == 2
    delete(filePath);  % 删除文件
    fprintf('已删除文件：%s\n', filePath);
else
    fprintf('未找到文件：%s\n', filePath);
end


%% === 1. 模型与仿真核心设置 ===
% 本部分定义Simulink模型的基本属性和全局仿真常量。

% --- 模型名称定义 ---
% 定义Simulink模型的名称。
model_name = 'MDOF_Hierarchical_Vibration_Sim'; 
disp(['模型名称已设定为: ', model_name]);

% --- 物理常量定义 ---
% 定义重力加速度常量 (单位: m/s^2)。
% 如果设置为0，则模型将不考虑重力效应。仿真速度加快方便调试。
gravity_g = 9.81; 
if gravity_g == 0
    disp('重力加速度 (gravity_g) 设置为 0 (m/s^2)，模型将忽略重力效应。');
else
    disp(['重力加速度 (gravity_g) 设置为: ', num2str(gravity_g), ' m/s^2。']);
end

disp('--- 1.1 模型与仿真核心参数初始化完成 ---');
disp(newline);

% --- 清理与创建新模型 ---
% 该过程确保每次运行脚本时，都从一个全新的、干净的模型开始，避免旧模型状态的干扰。

% 步骤1: 检查指定名称的Simulink模型是否已经加载到内存中
disp(['检查模型 "', model_name, '" 是否已加载...']);
if bdIsLoaded(model_name)
    % 如果模型已加载，则关闭它。参数 'false' 表示不保存任何未保存的更改即关闭。
    close_system(model_name, 0); % 使用数字0代替false，更符合Simulink API习惯
    disp(['已关闭已加载的Simulink模型: "', model_name, '" (未保存更改)。']);
else
    disp(['模型 "', model_name, '" 未加载，无需关闭。']);
end

% 步骤2: 检查同名Simulink模型文件 (.slx) 是否已存在于当前工作路径
model_file_path = [model_name, '.slx']; % 构建模型文件的完整路径
disp(['检查模型文件 "', model_file_path, '" 是否已存在于当前路径...']);
if exist(model_file_path, 'file') == 4 % 'file' 参数检查文件或文件夹，返回4表示为SLX或MDL文件
    disp(['发现已存在的模型文件: "', model_file_path, '"。尝试删除...']);
    try
        delete(model_file_path); % 删除已存在的模型文件
        disp(['旧模型文件 "', model_file_path, '" 已成功删除。']);
    catch e_delete_file % 捕获删除操作可能发生的错误
        % 显示更详细的错误信息，并提示用户手动操作
        warning('删除旧模型文件 "%s" 时发生错误: %s', model_file_path, e_delete_file.message);
        disp('请检查文件权限或是否有其他程序正在使用该文件。');
        disp('如果问题持续，请尝试手动删除该文件，然后重新运行脚本。');
        % 根据错误严重性，可以选择停止脚本执行
        % return; % 如果删除失败是致命的，则取消注释此行
    end
else
    disp(['模型文件 "', model_file_path, '" 不存在，无需删除。']);
end

% 步骤3: 创建一个新的、空的Simulink模型
disp(['正在创建新的空Simulink模型: "', model_name, '"...']);
new_system(model_name);
disp(['新模型 "', model_name, '" 已成功创建。']);

% 步骤4: 打开（并加载到内存）新创建的模型，以便后续编辑
disp(['正在打开模型: "', model_name, '"...']);
open_system(model_name);
disp(['模型 "', model_name, '" 已打开并加载。']);

% 步骤5: 清理新模型中的默认内容 (例如，Simulink自动添加的某些元素)
% 虽然new_system创建的是空模型，但有时可能仍需确保绝对干净。
% 首先删除所有默认连线 (通常新模型没有，但以防万一)
all_lines = find_system(model_name, 'FindAll', 'on', 'Type', 'Line');
if ~isempty(all_lines)
    delete_line(all_lines);
    disp('已删除新模型中的所有默认连线 (如果存在)。');
else
    disp('新模型中无默认连线需要删除。');
end

% 然后删除顶层所有默认模块 (除了模型本身)
% 'SearchDepth', 1 表示只查找模型顶层的模块
all_block_paths = find_system(model_name, 'SearchDepth', 1);
% 从列表中排除模型本身，因为它也会被find_system找到
all_block_paths = all_block_paths(~strcmp(all_block_paths, model_name));

if ~isempty(all_block_paths)
    disp('正在删除新模型中的默认模块 (如果存在)...');
    for i = 1:length(all_block_paths)
        block_path_to_delete = all_block_paths{i};
        % 再次确认获取到的是模块类型，避免尝试删除非模块对象
        if strcmp(get_param(block_path_to_delete, 'Type'), 'block')
            try
                block_name_to_delete = get_param(block_path_to_delete, 'Name'); % 获取模块名称用于显示
                delete_block(block_path_to_delete);
                disp(['  已删除默认模块: "', block_name_to_delete, '"。']);
            catch e_delete_block
                 warning('删除默认模块 "%s" 时发生警告: %s', ...
                         get_param(block_path_to_delete,'Name'), e_delete_block.message);
                 % 这种错误通常不致命，继续执行
            end
        end
    end
    disp('新模型默认模块清理完成。');
else
    disp('新模型中无默认模块需要删除。');
end

disp(['--- 1.2 Simulink模型 "', model_name, '" 已清空并重新创建 ---']);
disp('=== 1. 模型与仿真核心设置完成 ===');
disp(newline);

%% === 2. 激励与仿真参数定义 ===
disp(newline);
disp('=== 2. 开始定义激励与仿真参数 ===');

% --- 检测是否使用GUI预配置参数 ---
if evalin('base', 'exist(''params_from_gui'', ''var'')') && evalin('base', 'params_from_gui')
    use_gui_params = true;
    fprintf('检测到GUI预配置参数，将优先使用预设值...\n');
else
    use_gui_params = false;
    fprintf('未检测到GUI预配置，使用默认参数...\n');
end

% --- 激励类型选择 ---
if use_gui_params && evalin('base', 'exist(''excitation_type'', ''var'')')
    excitation_type = evalin('base', 'excitation_type');
else
    excitation_type = 'sine';  % 默认值
end
disp(['激励类型 (excitation_type): ', excitation_type]);

if ~ismember(excitation_type, {'sine', 'impulse'})
    error('无效的 excitation_type。请选择 ''sine'' 或 ''impulse''。');
end

% --- 正弦激励参数 ---
% 幅值
if use_gui_params && evalin('base', 'exist(''F_excite_y_amplitude'', ''var'')')
    F_excite_y_amplitude = evalin('base', 'F_excite_y_amplitude');
else
    F_excite_y_amplitude = 355;  % 默认值 (N)
end

if use_gui_params && evalin('base', 'exist(''F_excite_z_amplitude'', ''var'')')
    F_excite_z_amplitude = evalin('base', 'F_excite_z_amplitude');
else
    F_excite_z_amplitude = 275;  % 默认值 (N)
end
disp(['正弦激励幅值 - Y: ', num2str(F_excite_y_amplitude), ' N, Z: ', num2str(F_excite_z_amplitude), ' N']);

% 频率
if use_gui_params && evalin('base', 'exist(''excitation_frequency_hz'', ''var'')')
    excitation_frequency_hz = evalin('base', 'excitation_frequency_hz');
else
    excitation_frequency_hz = 7;  % 默认值 (Hz)
end
disp(['激励频率: ', num2str(excitation_frequency_hz), ' Hz']);

% 相位
if use_gui_params && evalin('base', 'exist(''excitation_phase_y_rad'', ''var'')')
    excitation_phase_y_rad = evalin('base', 'excitation_phase_y_rad');
else
    excitation_phase_y_rad = 0;  % 默认值 (rad)
end

if use_gui_params && evalin('base', 'exist(''excitation_phase_z_rad'', ''var'')')
    excitation_phase_z_rad = evalin('base', 'excitation_phase_z_rad');
else
    excitation_phase_z_rad = pi / 2;  % 默认值 (rad)
end

% --- 脉冲激励参数 ---
pulse_amplitude_y = 1;
pulse_amplitude_z = 1;

if use_gui_params && evalin('base', 'exist(''pulse_period_s'', ''var'')')
    pulse_period_s = evalin('base', 'pulse_period_s');
else
    pulse_period_s = 20;  % 默认值 (s)
end

if use_gui_params && evalin('base', 'exist(''pulse_width_percent'', ''var'')')
    pulse_width_percent = evalin('base', 'pulse_width_percent');
else
    pulse_width_percent = 0.025;  % 默认值 (%)
end

if use_gui_params && evalin('base', 'exist(''pulse_phase_delay_y_s'', ''var'')')
    pulse_phase_delay_y_s = evalin('base', 'pulse_phase_delay_y_s');
else
    pulse_phase_delay_y_s = 0;  % 默认值 (s)
end

if use_gui_params && evalin('base', 'exist(''pulse_phase_delay_z_s'', ''var'')')
    pulse_phase_delay_z_s = evalin('base', 'pulse_phase_delay_z_s');
else
    pulse_phase_delay_z_s = 0;  % 默认值 (s)
end

if use_gui_params && evalin('base', 'exist(''impulse_force_gain_y'', ''var'')')
    impulse_force_gain_y = evalin('base', 'impulse_force_gain_y');
else
    impulse_force_gain_y = 14500;  % 默认值 (N)
end

if use_gui_params && evalin('base', 'exist(''impulse_force_gain_z'', ''var'')')
    impulse_force_gain_z = evalin('base', 'impulse_force_gain_z');
else
    impulse_force_gain_z = 15500;  % 默认值 (N)
end

disp('脉冲激励参数已配置');

% --- 激励时间窗口 ---
if use_gui_params && evalin('base', 'exist(''excitation_start_time'', ''var'')')
    excitation_start_time = evalin('base', 'excitation_start_time');
else
    excitation_start_time = 5.0;  % 默认值 (s)
end

if use_gui_params && evalin('base', 'exist(''excitation_end_time'', ''var'')')
    excitation_end_time = evalin('base', 'excitation_end_time');
else
    excitation_end_time = 8.0;  % 默认值 (s)
end
disp(['激励时间窗口: ', num2str(excitation_start_time), ' - ', num2str(excitation_end_time), ' s']);

% --- 仿真控制参数 ---
if use_gui_params && evalin('base', 'exist(''sim_stop_time'', ''var'')')
    sim_stop_time = evalin('base', 'sim_stop_time');
else
    sim_stop_time = 20;  % 默认值 (s)
end

if use_gui_params && evalin('base', 'exist(''sim_fixed_step'', ''var'')')
    sim_fixed_step = evalin('base', 'sim_fixed_step');
else
    sim_fixed_step = 0.001;  % 默认值 (s)
end
disp(['仿真停止时间: ', num2str(sim_stop_time), ' s, 步长: ', num2str(sim_fixed_step), ' s']);

% --- 其他参数 ---
excitation_target_idx_base = 0;
excitation_type_selector_value = 1;
if strcmp(excitation_type, 'impulse')
    excitation_type_selector_value = 2;
end

if use_gui_params && evalin('base', 'exist(''use_parallel'', ''var'')')
    use_parallel = evalin('base', 'use_parallel');
else
    use_parallel = true;  % 默认值
end

disp('--- 2.激励与仿真参数定义完成 ---');
disp(newline);

%% === 3. 系统拓扑与参数定义 (模块化) ===
% 本部分定义了振动系统的层级拓扑结构以及各组成部分 (主干、各级分枝、果实) 的物理参数。
% 采用模块化和预定义库的方式管理参数，便于扩展和修改。
%
% =========================================================================
% === 注意：此版本参数经过大幅调整，旨在使模型更符合物理实际，避免数值问题 ===
% =========================================================================
%
disp(newline);
disp('=== 3. 开始定义系统拓扑与参数 (模块化 - 合理化参数版) ===');

% --- 尝试加载识别出的参数 ---
if exist('UpdatedTreeParameters.m', 'file')
    disp('检测到 UpdatedTreeParameters.m，正在加载识别参数...');
    % 为了防止 generate_branch_segment_params 函数未定义错误，先定义它
    % 注意：UpdatedTreeParameters.m 可能会调用它，所以必须确保它在路径中或在此之前定义
    % 由于脚本执行顺序问题，我们先定义 helper 函数 generate_branch_segment_params (见脚本末尾)
    % 这里我们假设 UpdatedTreeParameters.m 内部直接赋值，或者我们先运行脚本末尾的函数定义部分
    
    try
        run('UpdatedTreeParameters.m');
        disp('  -> 参数加载成功。');
    catch ME
        warning(E.identifier, '加载 UpdatedTreeParameters.m 失败: %s。将使用默认参数。', ME.message);
        use_sad_params = false;
    end
else
    use_sad_params = false;
    disp('未找到识别参数文件，使用默认参数。');
end

% --- A. 定义分枝拓扑结构 (Branching Topology Configuration) ---
fprintf('使用预配置的拓扑结构: %d个一级分枝\n', config.num_primary_branches);

disp('分枝拓扑结构 (config) 定义如下 (保持不变):');
disp(config);
% (拓扑校验部分保持不变)
if length(config.secondary_branches_count) ~= config.num_primary_branches, error('拓扑错误: secondary_branches_count 的长度与 num_primary_branches 不匹配。'); end
if length(config.tertiary_branches_count) ~= config.num_primary_branches, error('拓扑错误: tertiary_branches_count 的外层元胞长度与 num_primary_branches 不匹配。'); end
for p_idx_check = 1:config.num_primary_branches
if length(config.tertiary_branches_count{p_idx_check}) ~= config.secondary_branches_count(p_idx_check), error('拓扑错误: 对于一级分枝 P%d, tertiary_branches_count 内层数组长度与 secondary_branches_count 不匹配。', p_idx_check); end
end
disp('分枝拓扑结构初步校验通过。');
disp(newline);

% --- B. 预定义所有可能分枝的参数 (Parameter Pre-definition Library) ---
predefined_params = struct();
disp('初始化空的预定义参数库 (predefined_params)。');

disp(default_fruit_params);
disp(newline);

% --- 模板参数 (Parameter Templates) ---
% 检查 default_fruit_params 是否已从工作区正确加载
if ~exist('default_fruit_params', 'var') || isempty(default_fruit_params)
    error('Build:MissingFruitParams', ...
          ['严重错误：变量 "default_fruit_params" 未定义或为空。\n' ...
           '严禁使用脚本内的默认硬编码值。\n' ...
           '请先运行 "Integratedtreevibrationanalysis.m" 进行参数配置和识别。']);
end

% 验证果实参数的必要字段是否存在
required_fruit_fields = {'m', 'k_pedicel_y', 'c_pedicel_y', 'k_pedicel_z', 'c_pedicel_z', 'F_break'};
missing_fields = setdiff(required_fruit_fields, fieldnames(default_fruit_params));
if ~isempty(missing_fields)
    error('Build:InvalidFruitParams', ...
          '错误：传入的 "default_fruit_params" 结构体不完整，缺少以下必要字段：\n%s', ...
          strjoin(missing_fields, ', '));
end

fprintf('  已验证果实参数：质量=%.3f kg, 断裂力=%.2f N\n', default_fruit_params.m, default_fruit_params.F_break);
disp(newline);

% --- 主干参数 (Trunk Parameters) ---
params_struct = struct();
params_struct.trunk = struct(); 

if ~isfield(params_struct, 'trunk') || ~isfield(params_struct.trunk, 'root')
    error('参数错误：params_struct.trunk 不完整，请确保预配置正确提供了主干参数。');
end
fprintf('使用预配置的主干参数。\n');

disp('主干 (Trunk) 各段参数已基于新的基础值定义。');
disp(newline);

% --- 预定义一级分枝参数 (Primary Branch Parameters) ---
disp('验证一级分枝参数...');
if ~exist('predefined_params', 'var') || isempty(predefined_params)
    error('参数错误：predefined_params 不存在或为空，请确保预配置正确提供了分枝参数。');
end
for p_check = 1:config.num_primary_branches
    branch_id_check = ['P', num2str(p_check)];
    if ~isfield(predefined_params, branch_id_check)
        error('参数错误：未找到一级分枝 %s 的参数，请确保预配置包含所有分枝。', branch_id_check);
    end
end
fprintf('使用预配置的分枝参数，共 %d 个一级分枝。\n', config.num_primary_branches);
disp(newline);

% --- 验证二级和三级分枝参数 ---
disp('验证二级和三级分枝参数...');
for p_idx_v = 1:config.num_primary_branches
    for s_idx_v = 1:config.secondary_branches_count(p_idx_v)
        branch_id_s_v = ['P', num2str(p_idx_v), '_S', num2str(s_idx_v)];
        if ~isfield(predefined_params, branch_id_s_v)
            error('参数错误：未找到二级分枝 %s 的参数。', branch_id_s_v);
        end
        for t_idx_v = 1:config.tertiary_branches_count{p_idx_v}(s_idx_v)
            branch_id_t_v = [branch_id_s_v, '_T', num2str(t_idx_v)];
            if ~isfield(predefined_params, branch_id_t_v)
                error('参数错误：未找到三级分枝 %s 的参数。', branch_id_t_v);
            end
        end
    end
end
disp('所有分枝参数验证通过。');
disp(newline);

disp('--- 3.1 预定义分枝参数库 (predefined_params) 构建完成 ---');
disp(newline);

% 验证果实配置
if ~exist('fruit_config', 'var') || isempty(fruit_config)
    error('参数错误：fruit_config 不存在或为空，请确保预配置正确提供了果实配置。');
end
required_fruit_fields = {'attach_secondary_mid', 'attach_secondary_tip', ...
                         'attach_tertiary_mid', 'attach_tertiary_tip', 'fruits_per_node'};
for i_fc = 1:length(required_fruit_fields)
    if ~isfield(fruit_config, required_fruit_fields{i_fc})
        error('参数错误：fruit_config 缺少字段 %s。', required_fruit_fields{i_fc});
    end
end
fprintf('果实配置: 二级[mid=%d,tip=%d] 三级[mid=%d,tip=%d]\n', ...
        fruit_config.attach_secondary_mid, fruit_config.attach_secondary_tip, ...
        fruit_config.attach_tertiary_mid, fruit_config.attach_tertiary_tip);

params_struct.primary = cell(1, config.num_primary_branches); 
disp('开始根据拓扑配置 (config) 和预定义参数库 (predefined_params) 填充最终的 params_struct 结构体...');
for p_idx = 1:config.num_primary_branches 
branch_id_p = ['P', num2str(p_idx)]; 

fprintf('  处理一级分枝: %s...\n', branch_id_p);
if ~isfield(predefined_params, branch_id_p)
    error('参数缺失错误: 未找到一级分枝 "%s" 的参数。', branch_id_p); 
end
current_P_branch_data = predefined_params.(branch_id_p); 

if config.secondary_branches_count(p_idx) > 0
    current_P_branch_data.secondary_branches = cell(1, config.secondary_branches_count(p_idx));
    for s_idx = 1:config.secondary_branches_count(p_idx)
    branch_id_s = [branch_id_p, '_S', num2str(s_idx)];
    current_S_branch_data = predefined_params.(branch_id_s);
    
    % 检查是否有三级分枝
    num_tertiary = config.tertiary_branches_count{p_idx}(s_idx);
    
    if num_tertiary == 0
        % 无三级分枝，此二级分枝为末梢
        % 根据配置在mid和tip挂果
        if fruit_config.attach_secondary_mid
            current_S_branch_data.fruit_at_mid = default_fruit_params;
        end
        if fruit_config.attach_secondary_tip
            current_S_branch_data.fruit_at_tip = default_fruit_params;
        end
    else
        % 有三级分枝，处理三级分枝
        current_S_branch_data.tertiary_branches = cell(1, num_tertiary);
        
        for t_idx = 1:num_tertiary
            branch_id_t = [branch_id_s, '_T', num2str(t_idx)];
            current_T_branch_data = predefined_params.(branch_id_t);
            
            % 三级分枝在mid和tip挂果
            if fruit_config.attach_tertiary_mid
                current_T_branch_data.fruit_at_mid = default_fruit_params;
            end
            if fruit_config.attach_tertiary_tip
                current_T_branch_data.fruit_at_tip = default_fruit_params;
            end
            
            current_S_branch_data.tertiary_branches{t_idx} = current_T_branch_data;
        end
    end
    
    current_P_branch_data.secondary_branches{s_idx} = current_S_branch_data;
    end
end
params_struct.primary{p_idx} = current_P_branch_data; 
end
disp('--- 3.2 最终参数结构体 (params_struct) 动态填充完成 ---');
disp(newline);

% --- 将所有构建模型所需的参数打包到 model_build_params 结构体中 ---
model_build_params = struct();
model_build_params.config = config; 
model_build_params.parameters = params_struct; 
model_build_params.gravity_g = gravity_g; 
disp('已将拓扑、物理参数和重力加速度打包到 model_build_params 结构体中。');

clear predefined_params;
disp('已清理临时的预定义参数库 (predefined_params)。');

disp('--- 3.3 所有系统参数定义和打包完成 ---');
disp('=== 3. 系统拓扑与参数定义完成 ===');
disp(newline);    
%% === 3.5 静态分析：计算重力引起的初始位移 ===
% 本部分根据系统拓扑和参数，建立静态有限元模型，求解在重力作用下的平衡位移。
% 这些位移将作为Simulink模型中各质量点位移积分器的初始条件。
disp(newline);
disp('=== 3.5 开始进行静态分析以计算重力初始位移 ===');

initial_conditions_map = containers.Map('KeyType', 'char', 'ValueType', 'any');

if gravity_g ~= 0
    disp('重力不为零，开始构建并求解静态平衡方程 K * U = F...');

    % --- 步骤 1: 建立自由度(DOF)映射表 ---
    disp('  步骤 1/4: 正在建立系统自由度(DOF)映射...');
    % 调用辅助函数，该函数应定义在脚本末尾
    dof_map = map_dofs_recursively_static(model_build_params.parameters);
    num_masses = length(dof_map);
    num_dofs = num_masses * 2;
    disp(['  DOF映射完成，共发现 ', num2str(num_masses), ' 个质量点 (', num2str(num_dofs), ' 个自由度)。']);
    
    % --- 步骤 2: 构建全局力和刚度矩阵 ---
    disp('  步骤 2/4: 正在构建全局力向量 F 和刚度矩阵 K...');
    
    % 构建力向量 F (重力沿-Y方向作用)
    F = zeros(num_dofs, 1);
    for i = 1:num_masses
        mass_val = dof_map{i}{2}; % 从DOF映射中获取质量
        y_dof_idx = 2*i - 1;     % Y方向自由度的索引
        F(y_dof_idx) = -mass_val * gravity_g; % 假设Y为竖直方向
    end
    
    % 调用辅助函数构建全局刚度矩阵 K
    K = populate_K_matrix_static(dof_map, model_build_params.parameters);
    disp('  K 和 F 矩阵构建完成。');

    % --- 步骤 3: 求解静态位移 ---
    disp('  步骤 3/4: 正在求解线性方程组 K * U = F...');
    if rcond(full(K)) < eps
        warning('静态分析警告：全局刚度矩阵 K 是奇异的或接近奇异的。这可能意味着系统结构不稳定（存在自由浮动的部件）。求解结果可能不准确或出错。');
        U_static = pinv(full(K)) * F; % 使用伪逆尝试求解
        disp('  由于矩阵奇异，已使用伪逆(pinv)进行求解。');
    else
        U_static = K \ F;
        disp('  求解完成。');
    end

    % --- 步骤 4: 存储初始条件到Map中 ---
    disp('  步骤 4/4: 正在将静态位移结果存入 initial_conditions_map...');
    mass_id_to_idx_map = containers.Map(cellfun(@(c) c{1}, dof_map, 'UniformOutput', false), 1:num_masses);
    
    all_mass_ids = keys(mass_id_to_idx_map);
    for i = 1:length(all_mass_ids)
        mass_id = all_mass_ids{i};
        idx = mass_id_to_idx_map(mass_id);
        y_ic = U_static(2*idx - 1);
        z_ic = U_static(2*idx);
        initial_conditions_map(mass_id) = struct('y_ic', y_ic, 'z_ic', z_ic);
    end
    disp('  初始条件映射表构建完成。');

else
    disp('重力加速度为零，跳过静态分析。所有初始条件将默认为0。');
end

disp('=== 3.5 静态分析完成 ===');
disp(newline);

% 确保将 model_build_params 更新，包含新的初始条件映射
model_build_params.initial_conditions = initial_conditions_map;
disp('已将计算出的 initial_conditions_map 添加到 model_build_params 结构体中。');

%% === 4. (新增功能) 绘制果树拓扑结构 ===
disp(newline);
disp('=== 9. 开始绘制果树拓扑结构 ===');

if exist('model_build_params', 'var') && isfield(model_build_params, 'config')
    disp('正在根据config结构绘制果树拓扑...');
    try
        figure('Name', '果树拓扑结构示意图', 'NumberTitle', 'off');
        hold on;
        plot_tree_topology(model_build_params.config);
        hold off;
        title('果树分枝拓扑结构');
        xlabel('X 坐标 (示意)');
        ylabel('Y 坐标 (示意)');
        axis equal;
        grid on;
        disp('拓扑结构图绘制完成。');
    catch e_plot_topo
        warning(e_plot_topo.identifier, '绘制果树拓扑结构时发生错误: %s', e_plot_topo.message);
    end
else
    disp('警告: 未找到 model_build_params.config 结构，无法绘制拓扑图。');
end

%% === 5. 构建Simulink模型结构 (调用辅助函数) ===
% 本部分根据第3部分定义的参数 (model_build_params)，动态地在Simulink中创建模型拓扑。
% 主要包括：固定基座、主干、各级分枝、果实以及它们之间的连接。
% 同时，还会设置顶层激励源和激励路由逻辑。
% 核心构建逻辑封装在辅助函数 build_branch_recursively 中。
disp(newline);
disp('=== 4. 开始构建Simulink模型结构 ===');

% --- 布局参数定义 (Layout Parameters) ---
% 定义Simulink模型中各个模块和子系统的大小、间距等视觉布局参数。
% 将这些参数集中管理在 'layout_params' 结构体中，方便统一调整模型外观。
layout_params = struct();
layout_params.x_start = 50;         % 模型元素在X方向的起始绘制坐标
layout_params.y_start = 50;         % 模型元素在Y方向的起始绘制坐标

% 子系统尺寸与间距
layout_params.subsystem_width = 1800; % 主干和一级分枝子系统的宽度
layout_params.subsystem_height = layout_params.subsystem_width / 2.5; % 子系统高度，保持一定宽高比
layout_params.subsystem_height_spacing = layout_params.subsystem_height + 150; % 一级分枝子系统间的垂直间距

% 分枝段 (segment) 尺寸与间距 (指root, mid, tip质量块子系统)
layout_params.segment_width = 200;     % 分枝段质量块子系统的宽度
layout_params.segment_height = 220;    % 分枝段质量块子系统的高度
layout_params.segment_spacing_x = 350; % 分枝段之间的水平间距

% 连接模块 (弹簧阻尼器等) 尺寸 (这部分可能在辅助函数中更具体地使用)
layout_params.conn_block_width = 150;  % 连接逻辑中模块的大致宽度
layout_params.conn_block_height = 100; % 连接逻辑中模块的大致高度

% 果实模块偏移
layout_params.fruit_offset_x = 300;    % 果实模块相对于其所连接的Tip模块的X方向偏移

% 总线创建器和端口尺寸
layout_params.bus_creator_width = 5;   % 总线创建器模块的宽度 (非常窄)
layout_params.bus_creator_height = 100;% 总线创建器模块的高度
layout_params.outport_width = 30;      % 标准输出端口模块的宽度
layout_params.outport_height = 14;     % 标准输出端口模块的高度
layout_params.general_outport_width = 120; % 通用或命名输出端口的宽度 (如用于对外暴露信号)
layout_params.general_outport_height = 20; % 通用或命名输出端口的高度
layout_params.inport_width = 30;       % 标准输入端口模块的宽度
layout_params.inport_height = 14;      % 标准输入端口模块的高度
layout_params.general_inport_width = 120;  % 通用或命名输入端口的宽度
layout_params.general_inport_height = 20;  % 通用或命名输入端口的高度

disp('Simulink模块布局参数 (layout_params) 已定义。');
disp(layout_params);
disp(newline);

% --- 全局变量初始化 (用于在递归函数 build_branch_recursively 间传递信息) ---
% 警告: 全局变量的使用应谨慎。在此场景中，它们用于在递归调用栈中共享状态，
%       例如收集所有可激励目标的信息，或管理布局的动态Y坐标。
%       确保在使用完毕后清除它们，以避免潜在的副作用。

% excitation_targets_global: 元胞数组，用于存储模型中所有可被外部激励的质量块子系统的信息 (路径和名称)。
%                            由 build_branch_recursively 递归填充。
global excitation_targets_global;
excitation_targets_global = {}; % 初始化为空元胞

% current_y_level_global: 用于在 build_branch_recursively 中动态管理一级分枝子系统的Y轴布局位置。
global current_y_level_global;
current_y_level_global = [];    % 初始化为空，将在首次调用 build_branch_recursively (构建主干时) 设置初始值。

% fruit_signal_manager_global: 元胞数组，用于存储所有果实相关信号 (如脱落状态、果柄力) 的名称和唯一标识符。
%                              由 connect_fruit_with_detachment_2D 填充，并在 build_branch_recursively 中管理。
global fruit_signal_manager_global;
fruit_signal_manager_global = {}; % 初始化为空元胞

disp('全局变量 (excitation_targets_global, current_y_level_global, fruit_signal_manager_global) 已初始化。');
disp(newline);

% --- 4.1 创建 "固定基座 (FixedBase)" 子系统 ---
% 固定基座代表模型的固定参考点，通常输出零位移和零速度。
% 它也接收来自主干根部的反作用力 (虽然在本模型中可能仅用于观察或校验，不影响基座状态)。
disp('--- 4.1 开始创建 "固定基座 (FixedBase)" 子系统 ---');
fixed_base_subsystem_name = 'FixedBase'; % 固定基座子系统的名称
fixed_base_path = [model_name, '/', fixed_base_subsystem_name]; % 固定基座子系统的完整路径

% 创建前先检查并删除同名已存在的基座子系统，确保幂等性
if ~isempty(find_system(model_name, 'SearchDepth', 1, 'Name', fixed_base_subsystem_name))
    disp(['发现已存在的固定基座子系统 "', fixed_base_subsystem_name, '"，正在删除...']);
    delete_block(fixed_base_path);
    disp('旧的固定基座子系统已删除。');
end

% 添加新的子系统模块作为固定基座
disp(['正在添加新的固定基座子系统到: ', fixed_base_path]);
add_block('simulink/Ports & Subsystems/Subsystem', fixed_base_path, ...
    'Position', [layout_params.x_start, layout_params.y_start, ...
                 layout_params.x_start + 150, layout_params.y_start + 250]); % 基座的尺寸和位置

% 清理子系统内部的默认 Inport/Outport 和连线
clean_subsystem_internals(fixed_base_path); % 调用辅助函数 (需确保已定义或在路径上)
disp(['固定基座子系统 "', fixed_base_subsystem_name, '" 内部已清理。']);

% 在固定基座子系统内部添加模块：
% 定义固定基座的状态 (零位移，零速度)
add_block('simulink/Sources/Constant', [fixed_base_path, '/Zero_y_Displacement'], ...
          'Value', '0', 'SampleTime', '-1', ... % '-1' 表示继承采样时间
          'Position', [30, 30, 60, 50]); % Y方向位移输出 (常数0)
add_block('simulink/Sources/Constant', [fixed_base_path, '/Zero_y_Velocity'], ...
          'Value', '0', 'SampleTime', '-1', ...
          'Position', [30, 70, 60, 90]); % Y方向速度输出 (常数0)
add_block('simulink/Sources/Constant', [fixed_base_path, '/Zero_z_Displacement'], ...
          'Value', '0', 'SampleTime', '-1', ...
          'Position', [30, 110, 60, 130]);% Z方向位移输出 (常数0)
add_block('simulink/Sources/Constant', [fixed_base_path, '/Zero_z_Velocity'], ...
          'Value', '0', 'SampleTime', '-1', ...
          'Position', [30, 150, 60, 170]);% Z方向速度输出 (常数0)

% 为可能来自主干的反作用力创建输入端口 (主要用于连接完整性或分析)
add_block('simulink/Sources/In1', [fixed_base_path, '/F_react_y_in'], ...
          'Position', [30, 190, 60, 210], 'Port', '1'); % 接收Y方向反作用力
add_block('simulink/Sources/In1', [fixed_base_path, '/F_react_z_in'], ...
          'Position', [30, 230, 60, 250], 'Port', '2'); % 接收Z方向反作用力
% 注意: 这些输入力实际上并不改变固定基座的状态 (因为它被定义为恒定的零运动)。

% 使用 Bus Creator 将状态信号 (y, vy, z, vz) 合成一个总线信号输出
bus_creator_fb_internal_path = [fixed_base_path, '/StateBusCreator_Internal'];
add_block('simulink/Signal Routing/Bus Creator', bus_creator_fb_internal_path, ...
          'Inputs', '4', ... % 4个输入信号: y, vy, z, vz
          'DisplayOption', 'bar', ... % 显示为条形，更紧凑
          'Position', [100, 70, 105, 170]); % 总线创建器的位置和大小
% 连接Constant模块到Bus Creator的输入端口
add_line(fixed_base_path, 'Zero_y_Displacement/1',  'StateBusCreator_Internal/1');
add_line(fixed_base_path, 'Zero_y_Velocity/1', 'StateBusCreator_Internal/2');
add_line(fixed_base_path, 'Zero_z_Displacement/1',  'StateBusCreator_Internal/3');
add_line(fixed_base_path, 'Zero_z_Velocity/1', 'StateBusCreator_Internal/4');

% 创建固定基座的状态输出端口 (输出总线信号)
add_block('simulink/Sinks/Out1', [fixed_base_path, '/State_Out'], ...
          'Position', [180, 100, 210, 114], 'Port', '1'); % 输出端口1
% 连接Bus Creator的输出到子系统的Outport
add_line(fixed_base_path, 'StateBusCreator_Internal/1', 'State_Out/1');

disp(['固定基座子系统 "', fixed_base_subsystem_name, '" 内部结构构建完毕。']);
disp('--- 4.1 "固定基座" 子系统创建完成 ---');
disp(newline);

% --- 4.2 构建主干 (Trunk) ---
% 主干是系统的第一级振动结构，它连接到固定基座。
% 调用递归函数 build_branch_recursively 来创建主干。
disp('--- 4.2 开始构建主干 (Trunk) ---');

% 主干子系统的初始Y轴布局位置
% trunk_initial_y_level = layout_params.y_start + layout_params.subsystem_height_spacing / 2.0; % 原计算方式
% 调整主干的Y位置，使其与基座有一定对齐或视觉协调
trunk_initial_y_level = layout_params.y_start + ( (250 - layout_params.subsystem_height)/2 ); % 尝试让主干与基座垂直中心大致对齐
                                                                                           % 250是基座高度

% 主干子系统的建议路径 (tentative path)
% build_branch_recursively 内部会使用 'MakeNameUnique'='on' 来确保实际路径的唯一性。
trunk_subsystem_path_tentative = [model_name, '/Trunk_Branch']; % 主干子系统的推荐名称

% 调用递归构建函数来创建主干:
%   model_base_path: 顶层模型的名称 (model_name)
%   parent_subsystem_full_path_providing_state: 提供状态输入的父级模块路径 (此处为固定基座)
%   parent_reaction_force_target_full_path: 反作用力指向的父级模块路径 (此处也为固定基座)
%   parent_reaction_force_port_spec_y/z: 固定基座上接收反作用力的Inport的 *名称* (非端口号)
%                                        对于连接到固定基座，这些是 'F_react_y_in', 'F_react_z_in'
%   branch_level: 当前构建的分枝层级 (主干为0)
%   branch_indices: 当前分枝在同级中的索引 (主干通常为空或单一索引如 [1])
%   model_build_params_struct: 包含所有参数的结构体
%   layout_params_struct: 包含布局参数的结构体
%   base_pos_xy: 当前子系统左上角的建议起始坐标 [x, y]
%   current_branch_subsystem_full_path_tentative_in: 传递给下一级递归的子系统路径建议
build_branch_recursively(model_name, ...
                         fixed_base_path, ...  % 父级状态提供者：固定基座
                         fixed_base_path, ...  % 反作用力目标：固定基座
                         'F_react_y_in', ...   % 固定基座上接收Y反作用力的Inport名称
                         'F_react_z_in', ...   % 固定基座上接收Z反作用力的Inport名称
                         0, ...                % branch_level: 0 for Trunk
                         [], ...               % branch_indices: empty for Trunk (or could be [1])
                         model_build_params, ... % 包含拓扑、参数、重力的结构体
                         layout_params, ...    % 布局参数
                         [layout_params.x_start + 150 + 100, trunk_initial_y_level], ... % 主干起始X位置 (基座右侧+间距), Y位置
                         trunk_subsystem_path_tentative); % 传递主干子系统的建议路径
disp('主干 (Trunk) 结构已通过 build_branch_recursively 调用完成构建 (具体内部结构由辅助函数生成)。');
disp('--- 4.2 主干 (Trunk) 构建完成 ---');
disp(newline);

% --- 4.3 添加顶层激励控制 (正弦波源、时间门控) ---
% 这部分在模型顶层创建激励信号源 (如正弦波发生器) 和控制激励作用时间的逻辑。
disp('--- 4.3 开始添加顶层激励控制模块 (正弦波源、时间门控等) ---');

% Simulink模块的名称 (确保这些名称在模型顶层是唯一的)
sine_wave_y_source_block_name = 'SineWave_Y_ExcitationSource'; % Y方向正弦波源
sine_wave_z_source_block_name = 'SineWave_Z_ExcitationSource'; % Z方向正弦波源

pulse_gen_y_block_name = 'PulseGenerator_Y_Source';
pulse_gen_z_block_name = 'PulseGenerator_Z_Source';
impulse_gain_y_block_name = 'ImpulseForceGain_Y';
impulse_gain_z_block_name = 'ImpulseForceGain_Z';
excitation_type_switch_y_block_name = 'Switch_ExcitationType_Y';
excitation_type_switch_z_block_name = 'Switch_ExcitationType_Z';
excitation_type_selector_const_block_path = [model_name, '/ExcitationTypeSelector_Const']; % 用于选择激励类型的Constant

time_gated_y_exc_block_name = 'TimeGated_Y_Excitation';      % Y方向经时间门控后的实际激励信号
time_gated_z_exc_block_name = 'TimeGated_Z_Excitation';      % Z方向经时间门控后的实际激励信号

% 时间控制逻辑所需的模块路径
clock_block_path = [model_name, '/ClockForExcitationTiming']; % 时钟模块，提供当前仿真时间
zero_force_for_time_switch_block_path = [model_name, '/ZeroForce_ForTimeSwitch']; % 用于在激励非作用时段输出零力
% 用于选择激励目标的Constant模块 (其值将在仿真循环中被Simulink.SimulationInput覆盖)
exc_target_idx_const_block_path = [model_name, '/ExcitationTargetIndex_Selector'];

% 创建时钟模块 (Clock)
add_block('simulink/Sources/Clock', clock_block_path, ...
    'Position', [20, layout_params.y_start - 100, 50, layout_params.y_start - 80]); % 放置在左上角区域

% 创建零力常数模块 (Constant)
add_block('simulink/Sources/Constant', zero_force_for_time_switch_block_path, ...
    'Value', '0', 'SampleTime', '-1', ...
    'Position', [20, layout_params.y_start - 70, 50, layout_params.y_start - 50]);

% 创建激励类型选择器Constant模块
add_block('simulink/Sources/Constant', excitation_type_selector_const_block_path, ...
    'Value', 'excitation_type_selector_value', ... % 引用基础工作区变量
    'SampleTime', '-1', ...
    'Position', [20, layout_params.y_start - 180, 120, layout_params.y_start - 160]); % 放在左上角
disp(['激励类型选择器模块 "', strrep(excitation_type_selector_const_block_path, [model_name '/'], ''), '" 已添加。']);
excitation_type_selector_const_short_name = strrep(excitation_type_selector_const_block_path, [model_name '/'], '');

% --- Y方向激励链 (Y-Direction Excitation Chain) ---
disp('  创建Y方向激励链...');
% 1. Y方向正弦波源 (Sine Wave)
add_block('simulink/Sources/Sine Wave', [model_name, '/', sine_wave_y_source_block_name], ...
    'Amplitude', 'F_excite_y_amplitude', ...        % 幅值 (引用基础工作区变量)
    'Frequency', 'excitation_frequency_hz * 2 * pi', ... % 频率 (Hz转换为rad/s)
    'Phase', 'excitation_phase_y_rad', ...          % 相位 (引用基础工作区变量)
    'SampleTime', '0', ...                          % '0' 表示连续采样时间
    'Position', [20, layout_params.y_start - 20, 120, layout_params.y_start + 10]);

% 2. Y方向脉冲源 (Pulse Generator + Gain)
add_block('simulink/Sources/Pulse Generator', [model_name, '/', pulse_gen_y_block_name], ...
    'Amplitude', 'pulse_amplitude_y', ...
    'Period', 'pulse_period_s', ...
    'PulseWidth', 'pulse_width_percent', ...
    'PhaseDelay', 'pulse_phase_delay_y_s', ...
    'SampleTime', '0', ... % 连续时间脉冲
    'Position', [20, layout_params.y_start + 20, 120, layout_params.y_start + 50]); % 正弦波下方

add_block('simulink/Math Operations/Gain', [model_name, '/', impulse_gain_y_block_name], ...
    'Gain', 'impulse_force_gain_y', ...
    'Position', [140, layout_params.y_start + 25, 170, layout_params.y_start + 45]);
add_line(model_name, [pulse_gen_y_block_name, '/1'], [impulse_gain_y_block_name, '/1']);

% 3. Y方向激励类型选择开关 (Switch to select Sine or Impulse)
%    u1: Impulse, u2: Control (1 for sine, 2 for impulse), u3: Sine
switch_type_y_pos_x = 190;
add_block('simulink/Signal Routing/Switch', [model_name, '/', excitation_type_switch_y_block_name], ...
    'Criteria', 'u2 >= Threshold', ... % 如果 excitation_type_selector_value (u2) > 1.5 (即为2, impulse), 选择 u1 (impulse)
    'Threshold', '1.5', ... % 否则 (u2=1, sine), 选择 u3 (sine)
    'Position', [switch_type_y_pos_x, layout_params.y_start, ...
                 switch_type_y_pos_x + 40, layout_params.y_start + 40]);
add_line(model_name, [impulse_gain_y_block_name, '/1'], [excitation_type_switch_y_block_name, '/1']);     % Impulse to u1
add_line(model_name, [excitation_type_selector_const_short_name, '/1'], [excitation_type_switch_y_block_name, '/2']); % Selector to u2 (control)
add_line(model_name, [sine_wave_y_source_block_name, '/1'], [excitation_type_switch_y_block_name, '/3']); % Sine to u3
disp('  Y方向激励源 (正弦、脉冲) 及类型选择开关已创建。');

% 4. 时间门控逻辑 (Time Gating Logic for Y)
%    使用比较器 (Relational Operator) 和逻辑与 (Logical Operator) 来判断当前时间是否在激励窗口内。
compare_start_y_block_path = [model_name, '/Compare_Time_vs_ExcStartTime_Y'];
compare_end_y_block_path   = [model_name, '/Compare_Time_vs_ExcEndTime_Y'];
logical_and_y_block_path   = [model_name, '/LogicalAND_TimeWindow_Y'];
%    创建用于比较的激励起止时间常数模块
exc_start_time_const_y_block_path = [model_name, '/ExcitationStartTime_Const_Y'];
exc_end_time_const_y_block_path   = [model_name, '/ExcitationEndTime_Const_Y'];

add_block('simulink/Sources/Constant', exc_start_time_const_y_block_path, ...
    'Value', 'excitation_start_time', 'SampleTime', '-1', ... % 引用基础工作区变量
    'Position', [20, layout_params.y_start - 160, 50, layout_params.y_start - 140]);
add_block('simulink/Sources/Constant', exc_end_time_const_y_block_path, ...
    'Value', 'excitation_end_time', 'SampleTime', '-1', ...   % 引用基础工作区变量
    'Position', [20, layout_params.y_start - 130, 50, layout_params.y_start - 110]);

add_block('simulink/Logic and Bit Operations/Relational Operator', compare_start_y_block_path, ...
    'Operator', '>=', ... % 时间 >= 开始时间
    'Position', [70, layout_params.y_start - 100, 100, layout_params.y_start - 70]);
add_block('simulink/Logic and Bit Operations/Relational Operator', compare_end_y_block_path, ...
    'Operator', '<',  ...  % 时间 < 结束时间
    'Position', [70, layout_params.y_start - 60, 100, layout_params.y_start - 30]);
add_block('simulink/Logic and Bit Operations/Logical Operator', logical_and_y_block_path, ...
    'Operator', 'AND', 'Inputs', '2', ... % 两个输入
    'Position', [120, layout_params.y_start - 80, 150, layout_params.y_start - 50]);

% 连接时间门控逻辑模块
% 使用 strrep 去掉 model_name 前缀，因为 add_line 的第一个参数是父系统路径
clock_short_name = strrep(clock_block_path, [model_name, '/'], '');
start_time_const_short_name = strrep(exc_start_time_const_y_block_path, [model_name, '/'], '');
end_time_const_short_name = strrep(exc_end_time_const_y_block_path, [model_name, '/'], '');
compare_start_y_short_name = strrep(compare_start_y_block_path, [model_name, '/'], '');
compare_end_y_short_name = strrep(compare_end_y_block_path, [model_name, '/'], '');
logical_and_y_short_name = strrep(logical_and_y_block_path, [model_name, '/'], '');

add_line(model_name, [clock_short_name, '/1'], [compare_start_y_short_name, '/1']);    % Clock -> CompareStartTime (Input1)
add_line(model_name, [start_time_const_short_name, '/1'], [compare_start_y_short_name, '/2']); % ExcStartTimeConst -> CompareStartTime (Input2)
add_line(model_name, [clock_short_name, '/1'], [compare_end_y_short_name, '/1']);      % Clock -> CompareEndTime (Input1)
add_line(model_name, [end_time_const_short_name, '/1'], [compare_end_y_short_name, '/2']);   % ExcEndTimeConst -> CompareEndTime (Input2)
add_line(model_name, [compare_start_y_short_name, '/1'], [logical_and_y_short_name, '/1']); % CompareStartTimeOutput -> LogicalAND (Input1)
add_line(model_name, [compare_end_y_short_name, '/1'], [logical_and_y_short_name, '/2']);   % CompareEndTimeOutput -> LogicalAND (Input2)

% 5. Y方向时间门控开关 (Switch)
%    根据时间门控逻辑的输出，选择输出正弦激励或零力。
add_block('simulink/Signal Routing/Switch', [model_name, '/', time_gated_y_exc_block_name], ...
    'Criteria', 'u2 >= Threshold', 'Threshold', '0.5', ... % 当控制信号u2 (来自LogicalAND) > 0.5 时，输出u1
    'Position', [170, layout_params.y_start - 20, 210, layout_params.y_start + 30]); % Switch位置调整以适应输入
% 连接Switch模块
zero_force_switch_short_name = strrep(zero_force_for_time_switch_block_path, [model_name, '/'], '');
add_line(model_name, [excitation_type_switch_y_block_name, '/1'], [time_gated_y_exc_block_name, '/1']); % 输出从类型选择开关获取
add_line(model_name, [logical_and_y_short_name, '/1'], [time_gated_y_exc_block_name, '/2']);    % LogicalAND_Output -> Switch (Input2 - 控制)
add_line(model_name, [zero_force_switch_short_name, '/1'], [time_gated_y_exc_block_name, '/3']);% ZeroForce -> Switch (Input3 - 不通过时)
disp('  Y方向激励链 (正弦波源 -> 时间比较 -> 逻辑与 -> 开关) 构建完成。');

% --- Z方向激励链 (Z-Direction Excitation Chain) ---
% Z方向的激励链与Y方向类似，但可以复用Y方向的时间门控逻辑输出信号。
disp('  创建Z方向激励链...');
% 1. Z方向正弦波源 (Sine Wave)
add_block('simulink/Sources/Sine Wave', [model_name, '/', sine_wave_z_source_block_name], ...
    'Amplitude', 'F_excite_z_amplitude', ...
    'Frequency', 'excitation_frequency_hz * 2 * pi', ...
    'Phase', 'excitation_phase_z_rad', ...
    'SampleTime', '0', ...
    'Position', [20, layout_params.y_start + 50, 120, layout_params.y_start + 80]); % Z方向正弦波位置

% 2. Z方向脉冲源
add_block('simulink/Sources/Pulse Generator', [model_name, '/', pulse_gen_z_block_name], ...
    'Amplitude', 'pulse_amplitude_z', ...
    'Period', 'pulse_period_s', ...
    'PulseWidth', 'pulse_width_percent', ...
    'PhaseDelay', 'pulse_phase_delay_z_s', ...
    'SampleTime', '0', ...
    'Position', [20, layout_params.y_start + 110, 120, layout_params.y_start + 140]);

add_block('simulink/Math Operations/Gain', [model_name, '/', impulse_gain_z_block_name], ...
    'Gain', 'impulse_force_gain_z', ...
    'Position', [140, layout_params.y_start + 115, 170, layout_params.y_start + 135]);
add_line(model_name, [pulse_gen_z_block_name, '/1'], [impulse_gain_z_block_name, '/1']);

% 3. Z方向激励类型选择开关
add_block('simulink/Signal Routing/Switch', [model_name, '/', excitation_type_switch_z_block_name], ...
    'Criteria', 'u2 >= Threshold', ...
    'Threshold', '1.5', ...
    'Position', [switch_type_y_pos_x, layout_params.y_start + 90, ... % 与Y向类型开关对齐，但在下方
                 switch_type_y_pos_x + 40, layout_params.y_start + 90 + 40]);
add_line(model_name, [impulse_gain_z_block_name, '/1'], [excitation_type_switch_z_block_name, '/1']);     % Impulse to u1
add_line(model_name, [excitation_type_selector_const_short_name, '/1'], [excitation_type_switch_z_block_name, '/2']); % Selector to u2 (control)
add_line(model_name, [sine_wave_z_source_block_name, '/1'], [excitation_type_switch_z_block_name, '/3']); % Sine to u3
disp('  Z方向激励源 (正弦、脉冲) 及类型选择开关已创建。');

% 4. Z方向时间门控开关 (Switch) - 复用Y方向的时间控制信号
add_block('simulink/Signal Routing/Switch', [model_name, '/', time_gated_z_exc_block_name], ...
    'Criteria', 'u2 >= Threshold', 'Threshold', '0.5', ...
    'Position', [170, layout_params.y_start + 50, 210, layout_params.y_start + 100]); % Z方向Switch位置
% 连接Switch模块
add_line(model_name, [excitation_type_switch_z_block_name, '/1'], [time_gated_z_exc_block_name, '/1']); % 输出从类型选择开关获取
add_line(model_name, [logical_and_y_short_name, '/1'], [time_gated_z_exc_block_name, '/2']);    % LogicalAND_Output (from Y chain) -> Switch (Input2)
add_line(model_name, [zero_force_switch_short_name, '/1'], [time_gated_z_exc_block_name, '/3']);% ZeroForce -> Switch (Input3)
disp('  Z方向激励链 (正弦波源 -> 开关，复用Y向时间控制) 构建完成。');

% --- 激励目标选择器模块 ---
% 创建一个Constant模块，其值 (excitation_target_idx_base) 将在批量仿真时
% 通过 Simulink.SimulationInput().setVariable() 被动态修改，用于选择当前仿真运行的激励目标。
add_block('simulink/Sources/Constant', exc_target_idx_const_block_path, ...
    'Value', 'excitation_target_idx_base', ... % 引用基础工作区变量，将被SimIn覆盖
    'SampleTime', '-1', ...
    'Position', [20, layout_params.y_start + 120, 120, layout_params.y_start + 140]); % 放置在激励控制下方
disp(['激励目标选择器模块 "', strrep(exc_target_idx_const_block_path, [model_name '/'], ''), '" 已添加。']);

disp('--- 4.3 顶层激励控制模块添加完成 ---');
disp(newline);

% --- 4.4 收集激励目标并创建激励路由 ---
% 从全局变量中获取由 build_branch_recursively 收集到的所有可激励目标 (质量块子系统)
% 和所有果实相关的信号记录信息。
excitation_targets = excitation_targets_global; % 从全局变量中检索激励目标列表
all_fruit_signals_info = fruit_signal_manager_global; % 从全局变量中检索果实信号信息

% 清理全局变量 (非常重要，避免在后续脚本执行或多次运行中产生意外的累积效应)
clear global excitation_targets_global current_y_level_global fruit_signal_manager_global;
disp('已从全局变量中获取激励目标和果实信号信息，并已清理全局变量。');

% 显示收集到的可激励目标数量
if ~isempty(excitation_targets)
    disp(['--- 共收集到 ', num2str(length(excitation_targets)), ' 个可作为激励点的质量块子系统 ---']);
    disp('可激励目标列表 (索引号. 完整Simulink路径 (子系统名称)):');
    for i_et = 1:length(excitation_targets)
        % 确保 excitation_targets{i_et} 是结构体并且有 'path' 和 'name' 字段
        if isstruct(excitation_targets{i_et}) && isfield(excitation_targets{i_et}, 'path') && isfield(excitation_targets{i_et}, 'name')
            fprintf('  %d. %s (%s)\n', i_et, excitation_targets{i_et}.path, excitation_targets{i_et}.name);
        else
            fprintf('  %d. (无效的目标条目，缺少path或name字段)\n', i_et);
        end
    end
else
    warning('未收集到任何可激励目标 (excitation_targets为空)。激励路由将不会被创建。');
end

% 显示收集到的果实信号记录配置数量
if ~isempty(all_fruit_signals_info)
    disp(['--- 共收集到 ', num2str(length(all_fruit_signals_info)), ' 组果实信号记录配置 (用于后续分析) ---']);
    % 可以选择性地显示果实信号信息详情
    % for i_fs = 1:length(all_fruit_signals_info)
    %     disp(all_fruit_signals_info{i_fs});
    % end
else
    disp('未收集到任何果实信号记录配置 (all_fruit_signals_info为空)。这可能是因为模型中没有果实，或者果实信号记录逻辑未正确执行。');
end
disp(newline);

% --- 创建激励路由逻辑 (Excitation Routing Logic) ---
% 如果存在可激励目标，则创建逻辑：根据 ExcitationTargetIndex_Selector 的值，
% 将时间门控后的激励信号 (TimeGated_Y_Excitation, TimeGated_Z_Excitation)
% 路由到选定的目标质量块子系统的 F_excite_y_in 和 F_excite_z_in 端口。
% 对于未被选中的目标，其激励输入将接收零力。
if ~isempty(excitation_targets)
    disp('开始创建激励路由逻辑...');
    % 路由逻辑的起始布局位置和基本尺寸参数
    route_x_start = 250; % 路由逻辑块的X起始位置 (在激励源右侧)
    route_y_start = layout_params.y_start - 80; % Y起始位置，与时间控制逻辑对齐或稍作调整
    block_width_route = 100; % 路由中Constant和Compare模块的宽度
    block_height_route = 25; % 路由中这些模块的高度
    switch_width_route = 40;  % 路由中Switch模块的宽度
    switch_height_per_target_pair = 60; % 为每个目标的Y和Z激励对分配的Switch模块大致垂直空间
    y_spacing_route = 40;     % 路由中不同目标逻辑之间的垂直间距

    % 为路由逻辑创建共用的零力源 (避免重复创建多个零力Constant)
    zero_force_y_const_route_path = [model_name, '/ZeroForce_Routing_Y'];
    zero_force_z_const_route_path = [model_name, '/ZeroForce_Routing_Z'];
    add_block('simulink/Sources/Constant', zero_force_y_const_route_path, 'Value', '0', 'SampleTime', '-1', ...
        'Position', [route_x_start - 60, route_y_start - 60, route_x_start - 20, route_y_start - 40]);
    add_block('simulink/Sources/Constant', zero_force_z_const_route_path, 'Value', '0', 'SampleTime', '-1', ...
        'Position', [route_x_start - 60, route_y_start - 30, route_x_start - 20, route_y_start - 10]);
    % 获取这些零力源的短名称，用于add_line
    zero_force_y_src_short_name = [strrep(zero_force_y_const_route_path, [model_name, '/'], ''), '/1'];
    zero_force_z_src_short_name = [strrep(zero_force_z_const_route_path, [model_name, '/'], ''), '/1'];

    current_y_offset_route = 0; % 用于垂直排列每个目标的路由逻辑

    for k_target = 1:length(excitation_targets)
        target_info = excitation_targets{k_target}; % 当前处理的激励目标信息
        
        % 检查 target_info 的有效性
        if ~isstruct(target_info) || ~isfield(target_info, 'path') || ~isfield(target_info, 'name') || isempty(target_info.path)
            warning('激励目标列表中的条目 %d 无效 (缺少path或name字段，或path为空)，跳过此目标的路由创建。', k_target);
            continue;
        end
        
        target_mass_subsystem_full_path = target_info.path; % 目标质量块子系统的完整路径
        target_mass_local_name = target_info.name; % 目标质量块子系统的局部名称 (在父系统中的名称)
        
        fprintf('  为目标 %d/%d [%s (%s)] 创建激励路由...\n', ...
                k_target, length(excitation_targets), target_mass_local_name, target_mass_subsystem_full_path);

        % 解析目标质量块子系统的父级子系统路径
        path_parts = strsplit(target_mass_subsystem_full_path, '/');
        if length(path_parts) < 2 % 目标直接在顶层模型 (这种情况不应发生，因为质量块都在Branch子系统内)
            parent_subsystem_of_target_full_path = model_name;
            % target_mass_local_name 已经是正确的
            warning('目标 %s 似乎直接位于顶层模型，这不符合预期结构。请检查 build_branch_recursively 的实现。', target_mass_subsystem_full_path);
        else
            parent_subsystem_of_target_full_path = strjoin(path_parts(1:end-1), '/');
            % target_mass_local_name 已从 target_info.name 获取
        end
        
        % 获取父级子系统相对于顶层模型的路径 (用于add_line等)
        if strcmp(parent_subsystem_of_target_full_path, model_name)
            parent_subsystem_rel_path_for_line = model_name; % 如果父级是顶层模型
        else
            parent_subsystem_rel_path_for_line = strrep(parent_subsystem_of_target_full_path, [model_name, '/'], '');
        end

        % --- Y方向激励到此目标的路由 (Y-direction routing for this target) ---
        % 1. 创建一个Constant模块，其值为当前目标的索引 k_target
        idx_const_y_name = ['TargetIndexConst_Y_ForTarget_', num2str(k_target)];
        add_block('simulink/Sources/Constant', [model_name, '/', idx_const_y_name], 'Value', num2str(k_target), 'SampleTime', '-1', ...
            'Position', [route_x_start, route_y_start + current_y_offset_route, ...
                         route_x_start + block_width_route, route_y_start + current_y_offset_route + block_height_route]);

        % 2. 创建比较器 (Relational Operator)，比较 ExcitationTargetIndex_Selector 的输出与此目标的索引
        compare_y_name = ['Compare_ExcIndex_vs_TargetIndex_Y_', num2str(k_target)];
        compare_y_pos_x = route_x_start + block_width_route + 20;
        add_block('simulink/Logic and Bit Operations/Relational Operator', [model_name, '/', compare_y_name], 'Operator', '==', ...
            'Position', [compare_y_pos_x, route_y_start + current_y_offset_route, ...
                         compare_y_pos_x + block_height_route, route_y_start + current_y_offset_route + block_height_route]); % 调整为方形以匹配高度
        % 连接比较器输入
        exc_target_idx_const_short_name = strrep(exc_target_idx_const_block_path, [model_name '/'], '');
        add_line(model_name, [exc_target_idx_const_short_name, '/1'], [compare_y_name, '/1']); % ExcitationTargetIndex_Selector -> Compare (Input1)
        add_line(model_name, [idx_const_y_name, '/1'], [compare_y_name, '/2']);                % TargetIndexConst_Y -> Compare (Input2)

        % 3. 创建Switch模块，根据比较结果选择激励或零力
        switch_y_name = ['Switch_Y_Excitation_ToTarget_', num2str(k_target)];
        switch_y_pos_x = compare_y_pos_x + block_height_route + 20; % 调整X位置
        add_block('simulink/Signal Routing/Switch', [model_name, '/', switch_y_name], ...
            'Criteria', 'u2 >= Threshold', 'Threshold', '0.5', ...
            'Position', [switch_y_pos_x, route_y_start + current_y_offset_route - (switch_height_per_target_pair/4 - block_height_route)/2 , ... % 垂直居中调整
                         switch_y_pos_x + switch_width_route, route_y_start + current_y_offset_route + switch_height_per_target_pair/2 - (switch_height_per_target_pair/4 - block_height_route)/2]);
        % 连接Switch输入
        add_line(model_name, [time_gated_y_exc_block_name, '/1'], [switch_y_name, '/1']); % TimeGated_Y_Excitation -> Switch (Input1 - 通过)
        add_line(model_name, [compare_y_name, '/1'], [switch_y_name, '/2']);              % Compare_Output -> Switch (Input2 - 控制)
        add_line(model_name, zero_force_y_src_short_name, [switch_y_name, '/3']);         % ZeroForce_Routing_Y -> Switch (Input3 - 不通过)

        % 4. 将Switch的输出连接到目标质量块子系统的父级子系统的新Inport上
        %    这个新的Inport将把激励信号传入到目标质量块所在的Branch子系统内部。
        parent_inport_for_exc_y_name = matlab.lang.makeValidName(['Exc_Y_To_', strrep(target_mass_local_name, '_Mass', ''), '_In']);
        parent_inport_for_exc_y_full_path = [parent_subsystem_of_target_full_path, '/', parent_inport_for_exc_y_name];
        
        % 获取父级子系统上已存在的Inport数量，以确定新Inport的位置和端口号
        % 注意: find_system 的 SearchDepth 应该为1，只查找父级子系统直接包含的Inport
        num_existing_inports_on_parent = length(find_system(parent_subsystem_of_target_full_path, 'SearchDepth',1,'BlockType','Inport'));
        parent_inport_y_pos_for_exc_y = 50 + num_existing_inports_on_parent * (layout_params.general_inport_height + 20); % 垂直排列
        
        add_block('simulink/Sources/In1', parent_inport_for_exc_y_full_path, ...
            'Port', num2str(num_existing_inports_on_parent + 1), ... % 端口号是现有数量+1
            'Position', [20, parent_inport_y_pos_for_exc_y, ...
                         20 + layout_params.general_inport_width, parent_inport_y_pos_for_exc_y + layout_params.general_inport_height]);
        % 获取这个新创建的Inport在父级子系统接口上的端口号字符串 (例如 "3" for port 3)
        parent_subsystem_interface_port_for_new_exc_y_in_str = get_param(parent_inport_for_exc_y_full_path, 'Port');

        % 连接顶层的Switch输出到父级子系统的新Inport
        % 目标路径的格式：'ParentSubsystemRelativePath/PortNumber' 或 'ModelName/PortNumber' (如果父是顶层)
        if strcmp(parent_subsystem_rel_path_for_line, model_name) % 如果父级子系统就是顶层模型
             dest_block_for_line_y = parent_subsystem_interface_port_for_new_exc_y_in_str; % 直接用端口号
        else
             dest_block_for_line_y = [parent_subsystem_rel_path_for_line, '/', parent_subsystem_interface_port_for_new_exc_y_in_str];
        end
        add_line(model_name, [switch_y_name, '/1'], dest_block_for_line_y, 'autorouting','on');

        % 5. 在目标质量块子系统所在的父级子系统内部，将新创建的Inport连接到目标质量块的 F_excite_y_in
        target_mass_f_excite_y_in_block_full_path = [target_mass_subsystem_full_path, '/F_excite_y_in'];
        % 获取目标质量块上 F_excite_y_in 这个Inport的端口号字符串
        target_mass_subsystem_interface_port_for_f_excite_y_str = get_param(target_mass_f_excite_y_in_block_full_path, 'Port');
        % 连接线是在 parent_subsystem_of_target_full_path 内部画的
        add_line(parent_subsystem_of_target_full_path, ...
                 [parent_inport_for_exc_y_name, '/1'], ... % 源: 父级的新Inport
                 [target_mass_local_name, '/', target_mass_subsystem_interface_port_for_f_excite_y_str], ... % 目标: 目标质量块的对应激励输入端口
                 'autorouting','on');
        fprintf('    Y方向激励路由已连接至 %s 的 F_excite_y_in。\n', target_mass_local_name);

        % --- Z方向激励到此目标的路由 (Z-direction routing for this target) ---
        % 逻辑与Y方向完全类似，只是目标端口和源信号不同
        idx_const_z_name = ['TargetIndexConst_Z_ForTarget_', num2str(k_target)];
        add_block('simulink/Sources/Constant', [model_name, '/', idx_const_z_name], 'Value', num2str(k_target), 'SampleTime', '-1', ...
            'Position', [route_x_start, route_y_start + current_y_offset_route + switch_height_per_target_pair/2 + y_spacing_route/2, ...
                         route_x_start + block_width_route, route_y_start + current_y_offset_route + switch_height_per_target_pair/2 + y_spacing_route/2 + block_height_route]);
        
        compare_z_name = ['Compare_ExcIndex_vs_TargetIndex_Z_', num2str(k_target)];
        add_block('simulink/Logic and Bit Operations/Relational Operator', [model_name, '/', compare_z_name], 'Operator', '==', ...
            'Position', [compare_y_pos_x, route_y_start + current_y_offset_route + switch_height_per_target_pair/2 + y_spacing_route/2, ...
                         compare_y_pos_x + block_height_route, route_y_start + current_y_offset_route + switch_height_per_target_pair/2 + y_spacing_route/2 + block_height_route]);
        add_line(model_name, [exc_target_idx_const_short_name, '/1'], [compare_z_name, '/1']);
        add_line(model_name, [idx_const_z_name, '/1'], [compare_z_name, '/2']);

        switch_z_name = ['Switch_Z_Excitation_ToTarget_', num2str(k_target)];
        add_block('simulink/Signal Routing/Switch', [model_name, '/', switch_z_name], ...
            'Criteria', 'u2 >= Threshold', 'Threshold', '0.5', ...
            'Position', [switch_y_pos_x, route_y_start + current_y_offset_route + switch_height_per_target_pair/2 + y_spacing_route/2 - (switch_height_per_target_pair/4 - block_height_route)/2, ...
                         switch_y_pos_x + switch_width_route, route_y_start + current_y_offset_route + switch_height_per_target_pair + y_spacing_route/2 - (switch_height_per_target_pair/4 - block_height_route)/2]);
        add_line(model_name, [time_gated_z_exc_block_name, '/1'], [switch_z_name, '/1']); % 源是TimeGated_Z_Excitation
        add_line(model_name, [compare_z_name, '/1'], [switch_z_name, '/2']);
        add_line(model_name, zero_force_z_src_short_name, [switch_z_name, '/3']); % Z方向零力

        parent_inport_for_exc_z_name = matlab.lang.makeValidName(['Exc_Z_To_', strrep(target_mass_local_name, '_Mass', ''), '_In']);
        parent_inport_for_exc_z_full_path = [parent_subsystem_of_target_full_path, '/', parent_inport_for_exc_z_name];
        % 重新获取父级子系统上已存在的Inport数量 (因为Y方向刚加了一个)
        num_existing_inports_on_parent_after_y = length(find_system(parent_subsystem_of_target_full_path, 'SearchDepth',1,'BlockType','Inport'));
        parent_inport_z_pos_for_exc_y = 50 + num_existing_inports_on_parent_after_y * (layout_params.general_inport_height + 20);
        
        add_block('simulink/Sources/In1', parent_inport_for_exc_z_full_path, ...
            'Port', num2str(num_existing_inports_on_parent_after_y + 1), ...
            'Position', [20, parent_inport_z_pos_for_exc_y, ...
                         20 + layout_params.general_inport_width, parent_inport_z_pos_for_exc_y + layout_params.general_inport_height]);
        parent_subsystem_interface_port_for_new_exc_z_in_str = get_param(parent_inport_for_exc_z_full_path, 'Port');

        if strcmp(parent_subsystem_rel_path_for_line, model_name)
             dest_block_for_line_z = parent_subsystem_interface_port_for_new_exc_z_in_str;
        else
             dest_block_for_line_z = [parent_subsystem_rel_path_for_line, '/', parent_subsystem_interface_port_for_new_exc_z_in_str];
        end
        add_line(model_name, [switch_z_name, '/1'], dest_block_for_line_z, 'autorouting','on');

        target_mass_f_excite_z_in_block_full_path = [target_mass_subsystem_full_path, '/F_excite_z_in'];
        target_mass_subsystem_interface_port_for_f_excite_z_str = get_param(target_mass_f_excite_z_in_block_full_path, 'Port');
        add_line(parent_subsystem_of_target_full_path, ...
                 [parent_inport_for_exc_z_name, '/1'], ...
                 [target_mass_local_name, '/', target_mass_subsystem_interface_port_for_f_excite_z_str], ...
                 'autorouting','on');
        fprintf('    Z方向激励路由已连接至 %s 的 F_excite_z_in。\n', target_mass_local_name);

        % 更新下一个目标路由逻辑的垂直偏移
        current_y_offset_route = current_y_offset_route + switch_height_per_target_pair + y_spacing_route;
        fprintf('  为目标 %d [%s] 的激励路由逻辑 (包括父级子系统Inport创建和内部连线) 已全部创建。\n', k_target, target_mass_local_name);
        disp(' '); % 添加一些空行让输出更清晰
    end
    disp('--- 4.4 所有目标的激励路由逻辑自动创建完成 ---');
else
    disp('警告: 由于 excitation_targets 为空，未创建任何激励路由逻辑。仿真时所有质量块的外部激励输入将保持默认值 (通常为0，除非其内部有其他连接)。');
end
disp(newline);
disp('--- 模型结构构建完成 (已包含固定基座、主干、分枝、果实及顶层激励源和路由) ---');
disp('=== 4. Simulink模型结构构建完成 ===');
disp(newline);
%% === 6. 定义激励目标和仿真列表 ===
% 本部分基于第4部分收集到的可激励目标 (excitation_targets)，为每个目标配置一个Simulink仿真实例。
% 使用 Simulink.SimulationInput 对象来管理每次仿真的特定设置，例如要激励哪个目标。
disp(newline);
disp('=== 5. 开始定义激励目标和仿真列表 ===');

% 检查是否存在可激励目标，这些目标是在模型构建过程中 (主要是build_branch_recursively) 收集的。
if ~exist('excitation_targets', 'var') || isempty(excitation_targets)
    warning('变量 "excitation_targets" 未定义或为空。这通常意味着模型构建步骤未能成功收集到任何可激励的质量点。');
    disp('因此，无法定义仿真列表，后续的仿真将不会运行。');
    num_simulations = 0;         % 仿真次数为0
    simulation_inputs = [];      % 初始化空的仿真输入数组
else
    % 如果存在可激励目标，则为每个目标创建一个仿真配置
    num_simulations = length(excitation_targets); % 总仿真次数等于可激励目标的数量
    disp(['根据收集到的 ', num2str(num_simulations), ' 个可激励目标，准备配置相应的仿真运行。']);

    % 初始化一个 Simulink.SimulationInput 对象数组，每个对象对应一次仿真
    % repmat 创建一个重复的 SimulationInput 对象数组，指向同一个模型 model_name
    simulation_inputs = repmat(Simulink.SimulationInput(model_name), 1, num_simulations);
    disp(['已为模型 "', model_name, '" 初始化 ', num2str(num_simulations), ' 个 Simulink.SimulationInput 对象。']);

    % 循环遍历每个仿真配置 (即每个激励目标)
    for i_sim = 1:num_simulations
        fprintf('  配置第 %d/%d 个仿真输入对象 (对应激励目标 %d: %s)...\n', ...
                i_sim, num_simulations, i_sim, excitation_targets{i_sim}.name);

        % --- 设置本次仿真要激励的目标 ---
        % 通过 setVariable 方法，为当前 SimulationInput 对象设置模型工作区 (Model Workspace) 中的变量。
        % 'excitation_target_idx_base' 是在Simulink模型顶层由一个Constant模块引用的变量名。
        % 在每次仿真运行时，parsim 或 sim 会将这里设置的值传递给模型，从而激活对应的激励路由。
        % 注意：变量名 'excitation_target_idx_base' 必须与模型中Constant模块引用的基础工作区变量名一致。
        %        这里我们将目标索引 i_sim 赋值给它。
        simulation_inputs(i_sim) = simulation_inputs(i_sim).setVariable(...
            'excitation_target_idx_base', i_sim, ...       % 变量值：当前激励目标的索引 (1 到 num_simulations)
            'Workspace', model_name ...                   % 作用域：指定模型的工作区
        );
        fprintf('    变量 "excitation_target_idx_base" 在模型工作区中被设置为: %d\n', i_sim);

        % --- (可选) 为每次仿真单独设置其他模型参数 ---
        % 如果需要对每次仿真（例如，针对不同激励目标）使用不同的激励幅值、频率等，
        % 可以在这里通过 setVariable 修改这些参数。
        % 例如，假设有一个 F_excite_y_amplitude_scan 数组，包含了每次仿真要用的Y方向幅值:
        % if exist('F_excite_y_amplitude_scan', 'var') && length(F_excite_y_amplitude_scan) >= i_sim
        %     simulation_inputs(i_sim) = simulation_inputs(i_sim).setVariable('F_excite_y_amplitude', F_excite_y_amplitude_scan(i_sim), 'Workspace', model_name);
        %     fprintf('    (可选) 变量 "F_excite_y_amplitude" 被设置为: %f\n', F_excite_y_amplitude_scan(i_sim));
        % end
        % 当前脚本中，激励幅值和频率等参数是全局固定的，在第2部分已定义，并由Simulink模块直接引用。

        % --- 设置通用的模型仿真参数 ---
        % 这些参数通常对于所有批处理仿真都是相同的。
        simulation_inputs(i_sim) = simulation_inputs(i_sim).setModelParameter('StopTime', num2str(sim_stop_time)); % 仿真停止时间
%         simulation_inputs(i_sim) = simulation_inputs(i_sim).setModelParameter('SolverType','Fixed-step');      % 求解器类型：定步长
%         simulation_inputs(i_sim) = simulation_inputs(i_sim).setModelParameter('FixedStep', num2str(sim_fixed_step));   % 定步长大小
        simulation_inputs(i_sim) = simulation_inputs(i_sim).setModelParameter('SolverType', 'Variable-step');
        simulation_inputs(i_sim) = simulation_inputs(i_sim).setModelParameter('Solver', 'ode23t'); % ode15s, ode23t为变步长，ode45为定步长
        simulation_inputs(i_sim) = simulation_inputs(i_sim).setModelParameter('RelTol', '1e-4'); % 默认是1e-3
        simulation_inputs(i_sim) = simulation_inputs(i_sim).setModelParameter('AbsTol', '1e-5'); % 默认是auto
        % --- 设置数据保存和输出参数 ---
        simulation_inputs(i_sim) = simulation_inputs(i_sim).setModelParameter('SaveFormat', 'Dataset'); % 推荐的数据保存格式 (Dataset)
                                                                                                      % Dataset格式便于后续数据处理和访问。
        simulation_inputs(i_sim) = simulation_inputs(i_sim).setModelParameter('ReturnWorkspaceOutputs','on'); % 确保To Workspace模块的数据能够返回到sim函数输出中
                                                                                                            % 对于Dataset格式，To Workspace的输出会成为Dataset中的元素。
        simulation_inputs(i_sim) = simulation_inputs(i_sim).setModelParameter('SignalLogging', 'on');         % 开启信号记录功能
                                                                                                            % 这会将标记为记录的信号数据保存下来。
        simulation_inputs(i_sim) = simulation_inputs(i_sim).setModelParameter('SignalLoggingName', 'logsout'); % 指定记录信号的Dataset对象在输出结构体中的名称
                                                                                                              % 当SaveFormat为Dataset时，logsout是Simulink.SimulationOutput对象的一个字段。
        fprintf('    通用仿真参数 (StopTime, SolverType, FixedStep, SaveFormat, SignalLogging等) 已配置。\n');
    
        
        % 告诉求解器，我们希望在精炼的步长点上获得输出
        simulation_inputs(i_sim) = simulation_inputs(i_sim).setModelParameter('OutputOption', 'RefineOutputTimes');
        
        % 设置精炼因子。例如，4表示在每个求解器计算的步长之间，再内插出3个点。
        % 这会使数据点数量增加4倍，曲线会平滑得多。
        simulation_inputs(i_sim) = simulation_inputs(i_sim).setModelParameter('Refine', '4'); % <<-- 这个值可以调整，如2, 4, 8

        % (可选) 如果您之前设置了 Decimation，可以考虑减小或移除它
        % 因为 Refine 提供了更平滑的曲线，可能不再需要强制降采样。
    end
    disp('所有 Simulink.SimulationInput 对象配置完毕。');
end

disp('--- 5.1 仿真列表定义完成 ---');
disp('=== 5. 定义激励目标和仿真列表完成 ===');
disp(newline);
%% === 7. 运行仿真 (可配置为并行模式) ===
% 本部分执行在第5部分中定义的所有仿真配置 (simulation_inputs)。
% 支持使用并行计算工具箱 (Parallel Computing Toolbox) 来加速仿真过程。
% 同时，包含对并行池的智能管理逻辑。
disp(newline);
disp('=== 6. 开始运行仿真 ===');

simulation_results = []; % 初始化仿真结果数组 (将存储Simulink.SimulationOutput对象)
simulation_metadata_store = []; % 初始化元数据存储 (将存储每次仿真的配置信息和状态)

% 定义并行执行时请求的最大worker数量。
% 这个值可以根据系统CPU核心数、可用内存以及模型复杂度进行调整。
% 也可以考虑从外部配置文件读取或在脚本顶部设置为可调参数。
if ~evalin('base', 'exist(''parallel_execution_max_workers'', ''var'')')
    error('Build:MissingData', ...
          '工作区缺少 parallel_execution_max_workers，请先运行预配置程序');
end
parallel_execution_max_workers = evalin('base', 'parallel_execution_max_workers');
fprintf('并行计算最大Worker数: %d (从预配置读取)\n', parallel_execution_max_workers);
disp(['并行执行时请求的最大worker数 (parallel_execution_max_workers) 设置为: ', num2str(parallel_execution_max_workers)]);

% 检查是否有已配置的仿真任务
if exist('simulation_inputs', 'var') && ~isempty(simulation_inputs) && num_simulations > 0
    disp(['准备执行 ', num2str(num_simulations), ' 次仿真运行...']);

    % 在运行任何仿真之前，保存当前模型。
    % 这对于并行仿真尤其重要，因为它确保所有worker加载的是相同且最新的模型版本。
    try
        disp(['正在保存模型 "', model_name, '"...']);
        save_system(model_name);
        disp(['模型 "', model_name, '" 已成功保存。']);
    catch e_save_model
        warning('保存模型 "%s" 失败: %s', model_name, e_save_model.message);
        disp('并行仿真可能加载旧版本的模型，或者顺序仿真可能在未保存的更改上运行。');
        % 根据严重性，可以考虑是否继续。对于开发阶段，通常可以继续并提示用户。
    end
    disp(newline);

    % --- 并行池管理 ---
    if use_parallel && license('test', 'Distrib_Computing_Toolbox') % 检查并行计算工具箱许可证是否有效
        disp('检测到有效的并行计算工具箱许可证。');
        use_parallel = true;

        current_pool = gcp('nocreate'); % 获取当前并行池对象，如果不存在则不创建 ('nocreate')
        if isempty(current_pool)
            disp('当前没有活动的并行池。尝试启动新的并行池...');
            try
                % 获取系统物理核心数作为参考
                max_workers_system_cores = feature('numcores');
                disp(['系统检测到的物理CPU核心数: ', num2str(max_workers_system_cores)]);

                % 确定要请求的worker数量：取 系统核心数、仿真总数、用户定义的最大worker数 三者中的最小值。
                % 这样可以避免请求过多不必要的worker。
                num_workers_to_request = min([max_workers_system_cores, num_simulations, parallel_execution_max_workers]);
                
                % 简单的内存检查：估算每个worker所需内存，并与可用物理内存比较。
                % 注意: 这是一个非常粗略的估计，实际内存消耗取决于模型复杂度和Simulink版本。
                [~, system_memory_info] = memory; % 获取系统内存信息
                available_physical_mem_gb = system_memory_info.PhysicalMemory.Available / (1024^3); % 可用物理内存 (GB)
                % 假设每个worker运行一个Simulink实例大约需要0.5GB到2GB的额外内存 (需要根据实际模型调整)
                estimated_mem_per_worker_gb = 1.0; % 假设每个worker需要1GB (可调整)
                
                disp(['可用物理内存: ', num2str(available_physical_mem_gb, '%.2f'), ' GB.']);
                disp(['估计每个worker所需内存: ', num2str(estimated_mem_per_worker_gb), ' GB.']);

                if available_physical_mem_gb < num_workers_to_request * estimated_mem_per_worker_gb
                    % 如果可用内存不足以支持请求的worker数量，则自动减少请求数量
                    auto_adjusted_workers = floor(available_physical_mem_gb / estimated_mem_per_worker_gb);
                    if auto_adjusted_workers < 1
                        warning('可用内存不足以启动至少一个并行worker (需要 %.2f GB，可用 %.2f GB)。将转为顺序执行仿真。', ...
                                estimated_mem_per_worker_gb, available_physical_mem_gb);
                        use_parallel = false; % 内存不足，强制顺序执行
                    else
                        warning('可用内存可能不足以支持原计划的 %d 个worker。自动调整请求的worker数量从 %d 到 %d。', ...
                                num_workers_to_request, num_workers_to_request, auto_adjusted_workers);
                        num_workers_to_request = auto_adjusted_workers;
                    end
                end

                if use_parallel % 再次检查 use_parallel 标志 (可能因内存不足而被设为false)
                    fprintf('请求启动包含 %d 个worker的并行池。\n', num_workers_to_request);
                    % parpool(num_workers_to_request, 'AttachedFiles', {model_file_path}); % 可以附加模型文件，但通常Simulink会自动处理依赖
                    parpool_obj = parpool(num_workers_to_request);
                    current_pool = gcp('nocreate'); % 再次获取并行池对象
                    if isempty(current_pool)
                        warning('启动并行池失败。可能是由于配置问题或资源限制。将转为顺序执行仿真。');
                        use_parallel = false;
                    else
                        disp(['并行池已成功启动，包含 ', num2str(current_pool.NumWorkers), ' 个worker。']);
                        % 可选: 将模型路径添加到并行worker的路径中，确保worker能找到模型文件和相关函数
                        % addAttachedFiles(parpool_obj, {which(model_name)}); % 附加模型文件
                        % addAttachedFiles(parpool_obj, {'helper_function1.m', 'helper_function2.m'}); % 附加辅助函数
                        % 对于Simulink项目或设置了MATLAB路径的情况，这可能不是必需的。
                    end
                end
            catch e_parpool
                warning(E.identifier, '启动并行池时发生错误: %s', e_parpool.message);
                disp('将转为顺序执行仿真。');
                use_parallel = false;
            end
        else
            disp(['检测到已存在的并行池，包含 ', num2str(current_pool.NumWorkers), ' 个worker。将复用此池。']);
            % 如果需要，可以调整现有池的大小，但这通常更复杂，复用现有池是简单做法。
        end
    else
        disp('并行计算工具箱许可证不可用或用户手动关闭并行运算，将按顺序执行仿真。');
        use_parallel = false;
    end
    disp(newline);

    % --- 填充仿真元数据存储 (Simulation Metadata Store) ---
    % 在运行仿真之前，为每次仿真记录其关键配置参数 (主要是激励目标索引)。
    % 这有助于后续结果分析时将输出与输入配置对应起来。
    disp('正在填充仿真元数据存储...');
    temp_metadata_array = cell(1, num_simulations); % 临时元胞数组
    for i_meta = 1:num_simulations
        current_sim_input_obj = simulation_inputs(i_meta);
        % 从 SimulationInput 对象的 Variables 属性中提取 'excitation_target_idx_base' 的值
        vars_in_current_simIn = current_sim_input_obj.Variables;
        current_excitation_idx_val = NaN; % 默认值
        if ~isempty(vars_in_current_simIn)
            % 查找名为 'excitation_target_idx_base' 的变量
            idx_field_simIn = arrayfun(@(x) strcmp(x.Name, 'excitation_target_idx_base'), vars_in_current_simIn);
            if any(idx_field_simIn)
                current_excitation_idx_val = vars_in_current_simIn(idx_field_simIn).Value;
            else
                warning('在第 %d 个 SimulationInput 对象中未找到变量 "excitation_target_idx_base"。元数据中的激励索引可能不正确。', i_meta);
            end
        else
             warning('第 %d 个 SimulationInput 对象中的 Variables 属性为空。元数据中的激励索引可能不正确。', i_meta);
        end
        
        temp_metadata_array{i_meta} = struct(...
            'excitation_target_original_idx', current_excitation_idx_val, ... % 实际传递给模型的激励目标索引
            'excitation_target_name', excitation_targets{current_excitation_idx_val}.name, ... % 从excitation_targets获取名称
            'sim_input_index', i_meta, ...                  % 对应 simulation_inputs 数组中的索引
            'simulation_status', 'Pending' ...              % 初始状态
        );
    end
    simulation_metadata_store = [temp_metadata_array{:}]; % 将元胞数组转换为结构体数组
    disp('仿真元数据存储已填充。');
    % disp(simulation_metadata_store); % 可选：显示元数据内容
    disp(newline);

    sim_out_objects_temp = []; % 临时存储仿真输出对象

    % --- 执行仿真 ---
    if use_parallel && ~isempty(gcp('nocreate')) % 如果可以使用并行且并行池已成功启动或存在
        disp('使用 parsim (并行仿真) 运行仿真批处理...');
        try
            % parsim会自动处理SimulationInput对象数组，并将基础工作区变量传递给worker。
            % 'ShowProgress', 'on' 会在命令行显示仿真进度条。
            % 'TransferBaseWorkspaceVariables', 'on' (或 'auto') 确保基础工作区中被模型引用的变量被传递。
            %   通常 setVariable 设置的变量已在模型工作区，但这个选项可以作为保险。
            sim_out_objects_temp = parsim(simulation_inputs, ...
                                       'ShowProgress', 'on', ...
                                       'TransferBaseWorkspaceVariables', 'on', ...
                                       'AttachedFiles', {which(model_name)}); % 明确附加模型文件给worker
                                       % 'SetupFcn', @() load_system(model_name) % 可以在worker上执行的预加载函数
            disp('parsim 仿真批处理已完成。');

            % 检查返回结果的数量是否与预期一致
            if length(sim_out_objects_temp) ~= num_simulations
                warning('parsim 返回的仿真结果数量 (%d) 与预期的仿真次数 (%d) 不符。可能部分仿真失败或未返回结果。', ...
                        length(sim_out_objects_temp), num_simulations);
            end
            
            % 更新元数据中的仿真状态 (parsim的输出顺序与输入一致)
            for i_res = 1:length(sim_out_objects_temp)
                if i_res <= length(simulation_metadata_store)
                    if ~isempty(sim_out_objects_temp(i_res).ErrorMessage)
                        simulation_metadata_store(i_res).simulation_status = 'FailedInParsim';
                        simulation_metadata_store(i_res).error_details = sim_out_objects_temp(i_res).ErrorMessage;
                        fprintf('  并行仿真 %d (激励目标 %s) 失败: %s\n', ...
                                simulation_metadata_store(i_res).sim_input_index, ...
                                simulation_metadata_store(i_res).excitation_target_name, ...
                                sim_out_objects_temp(i_res).ErrorMessage);
                    else
                        simulation_metadata_store(i_res).simulation_status = 'Success';
                    end
                end
            end

        catch e_parsim % 捕获 parsim 执行期间的整体错误
            warning(E.identifier, '!!! parsim 仿真批处理期间发生严重错误: %s', e_parsim.message);
            disp('错误详情:');
            disp(e_parsim);
            % 如果 parsim 错误对象中包含了部分仿真结果 (e_parsim.SimulationOutput)
            if isprop(e_parsim, 'SimulationOutput') && ~isempty(e_parsim.SimulationOutput)
                 sim_out_objects_temp = e_parsim.SimulationOutput; % 尝试获取部分结果
                 disp('已从 parsim 错误对象中捕获到部分仿真结果。');
                 % 更新元数据状态
                 for i_meta_err = 1:length(simulation_metadata_store)
                     if i_meta_err <= length(sim_out_objects_temp) && ~isempty(sim_out_objects_temp(i_meta_err).ErrorMessage)
                         simulation_metadata_store(i_meta_err).simulation_status = 'FailedInParsimBatchError';
                         simulation_metadata_store(i_meta_err).error_details = sim_out_objects_temp(i_meta_err).ErrorMessage;
                     elseif i_meta_err > length(sim_out_objects_temp) || isempty(sim_out_objects_temp(i_meta_err))
                         simulation_metadata_store(i_meta_err).simulation_status = 'FailedInParsimBatchError_NoOutput';
                         simulation_metadata_store(i_meta_err).error_details = ['parsim批处理失败: ', e_parsim.message, ' (无此条目输出)'];
                     else
                         simulation_metadata_store(i_meta_err).simulation_status = 'SuccessInParsimBatchError'; % 可能有成功条目
                     end
                 end
            else
                sim_out_objects_temp = []; % 没有捕获到任何结果
                % 标记所有仿真的元数据为失败
                for k_err_meta = 1:num_simulations
                     simulation_metadata_store(k_err_meta).simulation_status = 'BatchFailed_ParsimError';
                     simulation_metadata_store(k_err_meta).error_details = MException('PARSIM:BatchError', ['parsim批处理失败: ', e_parsim.message]);
                end
            end
            disp('由于 parsim 批处理错误，结果可能不完整或不存在。');
        end
    else % 如果不使用并行，或者并行池启动失败，则按顺序执行仿真
        if use_parallel && isempty(gcp('nocreate'))
            disp('警告: 原计划使用并行仿真，但并行池未能成功启动。现转为按顺序执行仿真。');
        elseif ~use_parallel
            disp('按顺序执行仿真...');
        end
        
        temp_results_array_seq = cell(1, num_simulations); % 使用元胞数组存储顺序仿真的结果
        for k_sim_idx = 1:num_simulations
            target_name_disp = simulation_metadata_store(k_sim_idx).excitation_target_name;
            original_target_idx_disp = simulation_metadata_store(k_sim_idx).excitation_target_original_idx;

            fprintf('  运行仿真 %d/%d (顺序执行): 激励目标 %s (原始索引 %d)\n', ...
                    k_sim_idx, num_simulations, target_name_disp, original_target_idx_disp);
            try
                current_sim_output_seq = sim(simulation_inputs(k_sim_idx)); % 执行单次仿真
                temp_results_array_seq{k_sim_idx} = current_sim_output_seq; % 存储结果
                simulation_metadata_store(k_sim_idx).simulation_status = 'Success';
                fprintf('    仿真 %d (%s) 成功完成。\n', k_sim_idx, target_name_disp);
            catch e_single_sim_run % 捕获单次仿真运行的错误
                fprintf('  !!! 仿真 %d (%s) 失败: %s\n', k_sim_idx, target_name_disp, e_single_sim_run.message);
                disp('    错误详情:');
                disp(e_single_sim_run);
                simulation_metadata_store(k_sim_idx).simulation_status = 'FailedInSequential';
                simulation_metadata_store(k_sim_idx).error_details = e_single_sim_run;
                temp_results_array_seq{k_sim_idx} = []; % 标记为失败 (空结果)
            end
        end
        % 将元胞数组中的有效结果合并到 sim_out_objects_temp 数组中
        % isobject 检查是否为有效的Simulink.SimulationOutput对象，cellfun 更适合检查是否为空[]
        successful_indices_seq = ~cellfun(@isempty, temp_results_array_seq);
        if any(successful_indices_seq)
            sim_out_objects_temp = [temp_results_array_seq{successful_indices_seq}];
        else
            sim_out_objects_temp = []; % 如果所有顺序仿真都失败
        end
    end

    simulation_results = sim_out_objects_temp; % 将临时结果赋给最终的 simulation_results 变量

    % 总结仿真运行情况
    if ~isempty(simulation_results)
        num_successful_sims = 0;
        if isa(simulation_results, 'Simulink.SimulationOutput') % 确保是 SimulationOutput 对象数组
            for sr_idx = 1:length(simulation_results)
                if isempty(simulation_results(sr_idx).ErrorMessage) % parsim/sim 会在ErrorMessage字段记录错误
                    num_successful_sims = num_successful_sims + 1;
                end
            end
        end
        fprintf('共 %d 次仿真运行中，有 %d 次成功收集到结果对象 (无错误信息)。\n', num_simulations, num_successful_sims);
    elseif ~isempty(simulation_metadata_store) && ...
           any(arrayfun(@(x) contains(x.simulation_status, 'Failed', 'IgnoreCase', true), simulation_metadata_store))
        disp('所有或部分仿真运行失败。请检查 simulation_metadata_store 中的状态和错误详情。');
    else
        disp('未运行任何仿真，或者仿真未产生任何结果或元数据。');
    end

else
    disp('由于 "simulation_inputs" 为空或 "num_simulations" 为0，没有配置好的仿真任务，不运行仿真。');
end

disp('--- 6.1 仿真运行（或尝试运行）完成 ---');
disp('=== 6. 仿真运行阶段结束 ===');
disp(newline);
%% === 8. 分析结果并确定最佳夹持点 (多果实综合评估版) ===
% 本部分处理第6部分运行仿真后得到的 simulation_results 和 simulation_metadata_store。
% 主要目标是：
% 1. 从每次成功的仿真结果中提取关键性能指标 (KPI)，特别是与果实脱落相关的数据
%    (如脱落时间、脱落数量、果柄力等)。
% 2. 基于这些KPI，为每个激励点（即每次仿真运行）计算一个综合得分。
% 3. 找出得分最高的激励点作为“最佳夹持点”。
% 4. (可选) 绘制最佳激励点下各果实的响应曲线。
%
% 此版本考虑模型中可能存在多个果实，并对它们的整体脱落情况进行评估。
disp(newline);
disp('=== 7. 开始分析仿真结果 (多果实综合评估版) ===');

% --- 初始化分析所需变量 ---
best_excitation_target_info = []; % 用于存储找到的最佳激励点的详细信息和结果
analysis_summary = [];            % 用于存储每个激励点（仿真运行）的分析摘要

% --- 前置检查：确保分析所需的核心数据存在 ---
disp('进行分析前置检查...');
critical_vars_exist = true;
if ~exist('excitation_targets', 'var') || isempty(excitation_targets)
    warning('分析中止: 核心数据 "excitation_targets" 未定义或为空。无法将结果与激励点对应。');
    critical_vars_exist = false;
end
if ~exist('simulation_metadata_store', 'var') || isempty(simulation_metadata_store)
    warning('分析中止: 核心数据 "simulation_metadata_store" 未定义或为空。无法获取仿真的元数据和状态。');
    critical_vars_exist = false;
end
if ~exist('all_fruit_signals_info', 'var') % 这个变量可能为空 (如果没有果实)，但必须存在
    warning('分析中止: 核心数据 "all_fruit_signals_info" 未定义。无法获取果实信号的名称。');
    critical_vars_exist = false;
end
if ~exist('simulation_results','var') % 这个变量可能为空 (如果所有仿真失败)，但应存在
    disp('警告: 变量 "simulation_results" 不存在，已初始化为空数组。这可能表示所有仿真都失败了。');
    simulation_results = []; % 确保它存在，即使是空的
end
if ~exist('sim_stop_time', 'var')
    warning('全局仿真参数 "sim_stop_time" 未定义，将使用默认值 2.0 秒进行分析。');
    sim_stop_time = 2.0; % 如果未定义，则提供一个默认值
end

if ~critical_vars_exist
    disp('--- 由于核心数据缺失，结果分析提前结束 ---');
    disp('=== 7. 结果分析未能完成 ===');
    disp(newline);
    % 如果这是一个函数，这里应该 return; 对于脚本，后续部分可能无法正确执行。
    return; % 提前退出脚本的这一部分
end
disp('前置检查通过，核心数据变量存在。');
disp(newline);

% --- 初始化分析摘要结构体 ---
num_excitation_points = length(excitation_targets); % 总的潜在激励点数量
num_fruits_total = 0; % 初始化模型中果实的总数量
if ~isempty(all_fruit_signals_info) % 仅当 all_fruit_signals_info 非空时才计算长度
    num_fruits_total = length(all_fruit_signals_info);
end
disp(['检测到模型中总共有 ', num2str(num_fruits_total), ' 个果实被追踪。']);

% 定义分析摘要条目的模板结构体
analysis_summary_template = struct(...
    'excitation_target_original_idx', 0, ...  % 对应 excitation_targets 中的原始索引 (即传递给模型的激励选择值)
    'excitation_target_name', '', ...         % 该激励点的名称 (例如 "P1_S1_T1_Root_Mass")
    'excitation_target_path', '', ...         % 该激励点在Simulink模型中的简化路径
    'sim_input_index', NaN, ...               % 对应 simulation_inputs 和 simulation_metadata_store 中的索引
    'simulation_attempted', false, ...        % 是否尝试了对此激励点的仿真
    'simulation_succeeded_technically', false, ... % 仿真是否成功运行并返回了Simulink.SimulationOutput对象 (不代表逻辑成功)
    'sim_output_data_available', false, ...   % 是否能从Simulink.SimulationOutput对象中提取到有效数据
    'earliest_detachment_time_s', inf, ...    % 在此激励下，所有果实中最早的脱落时间 (秒)
    'first_fruit_to_detach_id', '', ...       % 最早脱落的那个果实的唯一ID
    'num_fruits_detached', 0, ...             % 在此激励下，成功脱落的果实数量
    'overall_score', -inf, ...                % 基于KPI计算的综合得分 (越高越好)
    'analysis_notes_or_error', '', ...        % 分析此结果时的备注或错误信息
    'sim_output_handle', [], ...              % (可选) 指向原始Simulink.SimulationOutput对象的句柄，便于调试
    'fruit_kpi_data', [] ...                  % (可选) 存储每个果实详细KPI的结构体数组
);
% 根据激励点数量，复制模板以创建分析摘要数组
analysis_summary = repmat(analysis_summary_template, num_excitation_points, 1);
disp(['已为 ', num2str(num_excitation_points), ' 个激励点初始化分析摘要结构体数组。']);

% --- 步骤1: 整合元数据和初始化分析摘要 ---
% 将 simulation_metadata_store 中的信息填充到 analysis_summary 中。
disp('步骤1: 开始整合仿真元数据到分析摘要中...');
current_sim_output_idx_counter = 0; % 用于正确关联 simulation_results 中的对象

if ~isempty(simulation_metadata_store)
    for i_meta = 1:length(simulation_metadata_store)
        meta_entry = simulation_metadata_store(i_meta);
        original_target_idx_from_meta = meta_entry.excitation_target_original_idx;

        % 校验从元数据中获取的原始激励点索引是否有效
        if isnan(original_target_idx_from_meta) || original_target_idx_from_meta < 1 || original_target_idx_from_meta > num_excitation_points
            warning('元数据条目 %d (sim_input_index: %d) 包含无效的 excitation_target_original_idx (%s)。该条目将被跳过。', ...
                    i_meta, meta_entry.sim_input_index, num2str(original_target_idx_from_meta));
            continue; % 跳过无效的元数据条目
        end
        
        % 将元数据信息填充到对应的 analysis_summary 条目中
        % 注意：analysis_summary 的索引是 original_target_idx_from_meta
        analysis_summary(original_target_idx_from_meta).excitation_target_original_idx = original_target_idx_from_meta;
        analysis_summary(original_target_idx_from_meta).excitation_target_name = excitation_targets{original_target_idx_from_meta}.name;
        
        % 简化路径显示 (去掉顶层模型名称前缀)
        full_path = excitation_targets{original_target_idx_from_meta}.path;
        prefix_to_remove = [model_name, '/'];
        if startsWith(full_path, prefix_to_remove)
            analysis_summary(original_target_idx_from_meta).excitation_target_path = full_path(length(prefix_to_remove)+1:end);
        else
            analysis_summary(original_target_idx_from_meta).excitation_target_path = full_path; % 如果没有前缀，直接使用
        end
        
        analysis_summary(original_target_idx_from_meta).sim_input_index = meta_entry.sim_input_index;
        analysis_summary(original_target_idx_from_meta).simulation_attempted = true; % 只要有元数据，就表示尝试了

        % 检查仿真状态 (来自元数据)
        if contains(meta_entry.simulation_status, 'Failed', 'IgnoreCase', true) || ...
           (isfield(meta_entry, 'error_details') && ~isempty(meta_entry.error_details))
            analysis_summary(original_target_idx_from_meta).simulation_succeeded_technically = false;
            error_msg = '仿真失败。';
            if isfield(meta_entry, 'error_details')
                if isa(meta_entry.error_details, 'MException')
                    error_msg = [error_msg, '原因: ', meta_entry.error_details.message];
                elseif ischar(meta_entry.error_details) || isstring(meta_entry.error_details)
                    error_msg = [error_msg, '原因: ', char(meta_entry.error_details)];
                end
            elseif isfield(meta_entry, 'simulation_status')
                 error_msg = [error_msg, '状态: ', meta_entry.simulation_status];
            end
            analysis_summary(original_target_idx_from_meta).analysis_notes_or_error = error_msg;
        else % 如果元数据状态不是 'Failed'
            analysis_summary(original_target_idx_from_meta).simulation_succeeded_technically = true; % 初步认为技术上成功
            current_sim_output_idx_counter = current_sim_output_idx_counter + 1; % 指向下一个有效的 sim_output 对象
            
            % 检查 simulation_results 中是否有对应的、有效的输出对象
            if ~isempty(simulation_results) && current_sim_output_idx_counter <= length(simulation_results) && ...
               isa(simulation_results(current_sim_output_idx_counter), 'Simulink.SimulationOutput') && ...
               isempty(simulation_results(current_sim_output_idx_counter).ErrorMessage) % 再次确认无错误信息
                
                analysis_summary(original_target_idx_from_meta).sim_output_handle = simulation_results(current_sim_output_idx_counter);
                analysis_summary(original_target_idx_from_meta).sim_output_data_available = true; % 标记数据可用于分析
            else
                % 虽然元数据未标记失败，但 simulation_results 中没有对应的有效输出
                analysis_summary(original_target_idx_from_meta).sim_output_data_available = false;
                analysis_summary(original_target_idx_from_meta).simulation_succeeded_technically = false; % 更正为技术失败
                error_msg_no_output = '元数据指示仿真可能成功，但在 simulation_results 中未找到对应的有效输出对象或该对象包含错误。';
                if current_sim_output_idx_counter > length(simulation_results)
                    error_msg_no_output = [error_msg_no_output, ' (计数器超出结果数组长度)'];
                elseif ~isempty(simulation_results) && current_sim_output_idx_counter <= length(simulation_results) && ...
                       ~isempty(simulation_results(current_sim_output_idx_counter).ErrorMessage)
                    error_msg_no_output = [error_msg_no_output, ' 关联的SimOutput对象包含错误: ', simulation_results(current_sim_output_idx_counter).ErrorMessage];
                end
                analysis_summary(original_target_idx_from_meta).analysis_notes_or_error = error_msg_no_output;
            end
        end
    end
else
    disp('警告: simulation_metadata_store 为空，无法将仿真结果与配置对应。分析将基于空的 analysis_summary。');
end
num_outputs_to_analyze = sum([analysis_summary.sim_output_data_available]);
disp(['步骤1完成: 已整合元数据。准备分析 ', num2str(num_outputs_to_analyze), ' 个有效的仿真输出。']);
disp(newline);

% --- 步骤2: 从每个成功的仿真结果中提取KPI ---
disp(['步骤2: 开始从 ', num2str(num_outputs_to_analyze), ' 个有效仿真输出中提取KPI...']);
for i_exc_point_analysis = 1:num_excitation_points % 遍历所有原始激励点
    % 只处理那些仿真成功且数据可用的条目
    if ~analysis_summary(i_exc_point_analysis).sim_output_data_available
        if analysis_summary(i_exc_point_analysis).simulation_attempted
            fprintf('  跳过激励点 %d (%s): 仿真未成功或输出数据不可用。备注: %s\n', ...
                    analysis_summary(i_exc_point_analysis).excitation_target_original_idx, ...
                    analysis_summary(i_exc_point_analysis).excitation_target_name, ...
                    analysis_summary(i_exc_point_analysis).analysis_notes_or_error);
        end
        continue; % 跳到下一个激励点
    end

    fprintf('  分析激励点 %d/%d: %s (原始索引 %d)...\n', ...
            i_exc_point_analysis, num_excitation_points, ...
            analysis_summary(i_exc_point_analysis).excitation_target_name, ...
            analysis_summary(i_exc_point_analysis).excitation_target_original_idx);
            
    current_sim_output_obj = analysis_summary(i_exc_point_analysis).sim_output_handle;
    
    try
        % --- 数据提取策略 ---
        % Simulink的输出 (Simulink.SimulationOutput 对象) 可以通过多种方式包含数据：
        % 1. logsout: 如果使用了信号记录 (Signal Logging) 并将 SaveFormat 设置为 'Dataset'，
        %    那么记录的信号会存储在 current_sim_output_obj.logsout (一个Dataset对象) 中。
        % 2. To Workspace 模块: 如果模型中使用了 To Workspace 模块，并且 SaveFormat 设置为
        %    'Timeseries' 或 'Array'，它们的数据会作为 current_sim_output_obj 的顶层属性出现，
        %    属性名即为 To Workspace 模块中指定的变量名。
        %
        % 优先从 logsout (Dataset) 中获取信号，因为它通常更结构化。
        
        signal_data_map = containers.Map('KeyType','char','ValueType','any'); % 用于存储提取到的 timeseries 数据
        
        logsout_data_extracted_successfully = false;
        if isprop(current_sim_output_obj, 'logsout') && ...
           ~isempty(current_sim_output_obj.logsout) && ...
           isa(current_sim_output_obj.logsout, 'Simulink.SimulationData.Dataset')
            
            sim_logsout_dataset = current_sim_output_obj.logsout;
            num_elements_in_logsout = sim_logsout_dataset.numElements;
            fprintf('    发现 logsout (Dataset) 包含 %d 个元素。尝试提取信号...\n', num_elements_in_logsout);
            
            for k_elem = 1:num_elements_in_logsout
                element = sim_logsout_dataset.getElement(k_elem);
                element_name = element.Name;
                
                % 确保元素名称不为空且为MATLAB有效变量名，再作为Map的键
                if ~isempty(element_name) && isvarname(element_name)
                    if isa(element, 'Simulink.SimulationData.Signal') && isa(element.Values, 'timeseries')
                        signal_data_map(element_name) = element.Values; % 存储 timeseries 对象
                        % fprintf('      从logsout提取到信号: %s (类型: timeseries from Signal object)\n', element_name);
                    elseif isa(element.Values, 'timeseries') % 有时Dataset元素直接就是timeseries
                         signal_data_map(element_name) = element.Values;
                         % fprintf('      从logsout提取到信号: %s (类型: timeseries direct)\n', element_name);
                    else
                         % fprintf('      logsout中元素 %s 不是期望的timeseries格式，跳过。\n', element_name);
                    end
                elseif ~isempty(element_name)
                    fprintf('      logsout中元素名 "%s" 不是有效的MATLAB变量名，无法作为Map的键，跳过。\n', element_name);
                % else: 元素名为空，无法处理
                end
            end
            if ~isempty(signal_data_map)
                logsout_data_extracted_successfully = true;
                fprintf('    已成功从 logsout Dataset 中提取 %d 个信号到 signal_data_map。\n', length(signal_data_map.keys));
            else
                fprintf('    logsout Dataset 中未找到可提取的 timeseries 信号。\n');
            end
        else
            fprintf('    未找到有效的 logsout Dataset (logsout属性不存在、为空或非Dataset类型)。\n');
        end

        % 如果未能从 logsout 成功提取数据 (或者 logsout 为空但模型可能用了 To Workspace)，
        % 则尝试遍历 SimulationOutput 对象的顶层属性，查找由 To Workspace 模块产生的 timeseries 数据。
        if ~logsout_data_extracted_successfully
            fprintf('    尝试从 SimulationOutput 对象的顶层属性中提取 To Workspace 模块数据...\n');
            all_prop_names_sim_output = properties(current_sim_output_obj);
            for k_prop = 1:length(all_prop_names_sim_output)
                prop_name = all_prop_names_sim_output{k_prop};
                % 排除已知的非信号属性 (如 SimulationMetadata, ErrorMessage, tout, logsout)
                % 以及我们刚刚尝试过的 logsout
                if any(strcmp(prop_name, {'SimulationMetadata', 'ErrorMessage', 'tout', 'logsout'}))
                    continue;
                end
                
                prop_value = current_sim_output_obj.(prop_name);
                if isa(prop_value,'timeseries')
                    if ~isKey(signal_data_map, prop_name) % 避免覆盖从logsout中可能已提取的同名信号
                        signal_data_map(prop_name) = prop_value;
                        % fprintf('      从顶层属性提取到To Workspace信号: %s (类型: timeseries)\n', prop_name);
                    end
                elseif isa(prop_value,'Simulink.SimulationData.Signal') && isa(prop_value.Values, 'timeseries')
                     if ~isKey(signal_data_map, prop_name)
                        signal_data_map(prop_name) = prop_value.Values; % 提取 Signal 对象中的 timeseries 数据
                        % fprintf('      从顶层属性提取到To Workspace信号: %s (类型: timeseries from Signal object)\n', prop_name);
                     end
                end
            end
             if ~isempty(signal_data_map) && ~logsout_data_extracted_successfully % 仅当之前未从logsout提取到时才报告
                fprintf('    已成功从顶层属性中提取 %d 个信号到 signal_data_map。\n', length(signal_data_map.keys));
            elseif isempty(signal_data_map)
                 fprintf('    在顶层属性中也未找到可提取的 timeseries 信号。\n');
            end
        end

        % 最终检查是否成功提取到任何信号数据
        if isempty(signal_data_map)
            % 检查是否存在时间向量 tout，如果连 tout 都没有，那基本没数据了
            if ~(isprop(current_sim_output_obj,'tout') && ~isempty(current_sim_output_obj.tout))
                error('关键错误: SimulationOutput 对象中未找到任何有效的信号数据 (signal_data_map 为空，且 tout 也缺失)。无法进行KPI分析。');
            else
                warning('未从 logsout 或 To Workspace 模块提取到任何信号数据 (signal_data_map 为空)，但存在时间向量 tout。KPI分析可能受限。');
            end
        end

        % --- 分析每个果实的KPI ---
        current_sim_fruit_kpi_data = []; % 用于存储此仿真下所有果实的KPI
        if num_fruits_total > 0 && ~isempty(all_fruit_signals_info) % 只有在模型中有果实且有信号信息时才进行
            fruit_kpi_template_single = struct('fruit_id','', 'is_detached',false, 'detachment_time_s',inf, 'f_pedicel_max_N',0);
            current_sim_fruit_kpi_data = repmat(fruit_kpi_template_single, num_fruits_total, 1);
        end
        
        overall_earliest_detachment_time_this_sim_s = inf; % 本次仿真中所有果实的最早脱落时间
        first_fruit_id_to_detach_this_sim = '';            % 最早脱落的果实ID
        num_fruits_detached_this_sim = 0;                  % 本次仿真中脱落的果实总数

        if num_fruits_total > 0 && ~isempty(all_fruit_signals_info)
            fprintf('    开始分析 %d 个果实的KPI...\n', num_fruits_total);
            for i_fruit = 1:num_fruits_total
                fruit_signal_info_entry = all_fruit_signals_info{i_fruit}; % 获取当前果实的信号名称信息
                current_sim_fruit_kpi_data(i_fruit).fruit_id = fruit_signal_info_entry.unique_id;
                
                % 检查脱落状态信号 (Detached Status)
                detached_status_var_name = fruit_signal_info_entry.detached_status_varname;
                if isKey(signal_data_map, detached_status_var_name)
                    detached_ts = signal_data_map(detached_status_var_name);
                    if ~isempty(detached_ts) && isa(detached_ts, 'timeseries') && ~isempty(detached_ts.Data)
                        % 脱落条件：信号值 > 0.5 (假设脱落信号是阶跃或布尔型，0表示未脱落，1表示脱落)
                        if any(detached_ts.Data > 0.5) 
                            current_sim_fruit_kpi_data(i_fruit).is_detached = true;
                            num_fruits_detached_this_sim = num_fruits_detached_this_sim + 1;
                            
                            % 找到首次脱落的时间点
                            first_detach_idx_in_ts = find(detached_ts.Data > 0.5, 1, 'first');
                            if ~isempty(first_detach_idx_in_ts)
                                current_fruit_detach_time_s = detached_ts.Time(first_detach_idx_in_ts);
                                current_sim_fruit_kpi_data(i_fruit).detachment_time_s = current_fruit_detach_time_s;
                                
                                % 更新全局最早脱落时间和对应果实ID
                                if current_fruit_detach_time_s < overall_earliest_detachment_time_this_sim_s
                                    overall_earliest_detachment_time_this_sim_s = current_fruit_detach_time_s;
                                    first_fruit_id_to_detach_this_sim = fruit_signal_info_entry.unique_id;
                                end
                            else
                                % 逻辑上不应发生：any(>0.5)为真但find找不到第一个。可能数据有问题。
                                current_sim_fruit_kpi_data(i_fruit).detachment_time_s = sim_stop_time; % 或标记为错误
                                analysis_summary(i_exc_point_analysis).analysis_notes_or_error = ...
                                    [analysis_summary(i_exc_point_analysis).analysis_notes_or_error, ...
                                     sprintf(' [果实 %s 的脱落信号 %s 数据异常: any>0.5 但无法找到首次脱落点]', ...
                                     fruit_signal_info_entry.unique_id, detached_status_var_name)];
                            end
                        else
                             current_sim_fruit_kpi_data(i_fruit).is_detached = false;
                             current_sim_fruit_kpi_data(i_fruit).detachment_time_s = sim_stop_time; % 未脱落，脱落时间设为仿真结束时间
                        end
                    else
                        analysis_summary(i_exc_point_analysis).analysis_notes_or_error = ...
                            [analysis_summary(i_exc_point_analysis).analysis_notes_or_error, ...
                             sprintf(' [果实 %s 的脱落信号 %s (来自signal_data_map) 为空或非timeseries]', ...
                             fruit_signal_info_entry.unique_id, detached_status_var_name)];
                    end
                else
                    analysis_summary(i_exc_point_analysis).analysis_notes_or_error = ...
                        [analysis_summary(i_exc_point_analysis).analysis_notes_or_error, ...
                         sprintf(' [在signal_data_map中未找到果实 %s 的脱落状态信号 %s]', ...
                         fruit_signal_info_entry.unique_id, detached_status_var_name)];
                     current_sim_fruit_kpi_data(i_fruit).detachment_time_s = sim_stop_time; % 无法判断，保守认为未脱落
                end

                % 检查果柄力幅值信号 (Pedicel Force Magnitude)
                fped_mag_var_name = fruit_signal_info_entry.fped_mag_varname;
                if isKey(signal_data_map, fped_mag_var_name)
                    fped_mag_ts = signal_data_map(fped_mag_var_name);
                    if ~isempty(fped_mag_ts) && isa(fped_mag_ts, 'timeseries') && ~isempty(fped_mag_ts.Data)
                        current_sim_fruit_kpi_data(i_fruit).f_pedicel_max_N = max(fped_mag_ts.Data);
                    else
                         analysis_summary(i_exc_point_analysis).analysis_notes_or_error = ...
                            [analysis_summary(i_exc_point_analysis).analysis_notes_or_error, ...
                             sprintf(' [果实 %s 的果柄力信号 %s (来自signal_data_map) 为空或非timeseries]', ...
                             fruit_signal_info_entry.unique_id, fped_mag_var_name)];
                    end
                else
                     analysis_summary(i_exc_point_analysis).analysis_notes_or_error = ...
                        [analysis_summary(i_exc_point_analysis).analysis_notes_or_error, ...
                         sprintf(' [在signal_data_map中未找到果实 %s 的果柄力信号 %s]', ...
                         fruit_signal_info_entry.unique_id, fped_mag_var_name)];
                end
            end % 结束单个果实的KPI分析循环
            fprintf('    所有 %d 个果实的KPI分析完成。\n', num_fruits_total);
        end % 结束 if num_fruits_total > 0
        
        % 将本次仿真的汇总KPI存入 analysis_summary
        analysis_summary(i_exc_point_analysis).fruit_kpi_data = current_sim_fruit_kpi_data;
        analysis_summary(i_exc_point_analysis).earliest_detachment_time_s = overall_earliest_detachment_time_this_sim_s;
        analysis_summary(i_exc_point_analysis).first_fruit_to_detach_id = first_fruit_id_to_detach_this_sim;
        analysis_summary(i_exc_point_analysis).num_fruits_detached = num_fruits_detached_this_sim;

        % --- 计算综合得分 (Scoring Logic) ---
        % 得分策略示例：
        % - 主要目标：尽快脱落尽可能多的果实。
        % - 时间分量：脱落越早，得分越高。
        % - 数量分量：脱落数量越多，得分越高。
        % - 权重：可以调整时间和数量分量的权重。
        current_score = -inf; % 默认极低分
        if num_fruits_total > 0 % 只有在有果实的情况下才计算有意义的得分
            if num_fruits_detached_this_sim > 0 % 至少有一个果实脱落
                % 时间得分: 100 - (最早脱落时间 / 总仿真时间) * 100。越早脱落，此项越高。
                % 修正：原 score_time_component = 100 - (overall_earliest_detachment_time_this_sim_s * 10) 不太好，
                % 改为基于仿真总时间的相对值。
                score_time_component = (1 - (overall_earliest_detachment_time_this_sim_s / sim_stop_time)) * 100;
                score_time_component = max(0, score_time_component); %确保不为负

                % 数量得分: (脱落数量 / 总果实数量) * 100
                score_quantity_component = (num_fruits_detached_this_sim / num_fruits_total) * 100;
                
                % 综合得分: 加权平均，例如 70% 时间，30% 数量
                time_weight = 0.7;
                quantity_weight = 0.3;
                current_score = time_weight * score_time_component + quantity_weight * score_quantity_component;
            else % 如果没有果实脱落
                % 给一个惩罚分，例如与总果实数量相关的负分
                current_score = -10 * num_fruits_total; % 例如，每个未脱落的果实扣10分
            end
        else % 如果模型中没有果实
            current_score = 0; % 没有果实，得分为0 (或根据需求定义)
        end
        analysis_summary(i_exc_point_analysis).overall_score = current_score;
        fprintf('    激励点 %s 的综合得分为: %.2f\n', ...
                analysis_summary(i_exc_point_analysis).excitation_target_name, current_score);

    catch e_extract_kpi
        % 如果在提取或分析单个仿真结果时出错
        error_message_kpi = sprintf('提取或分析激励点 %s (原始索引 %d) 的KPI时发生错误: %s', ...
                                 analysis_summary(i_exc_point_analysis).excitation_target_name, ...
                                 analysis_summary(i_exc_point_analysis).excitation_target_original_idx, ...
                                 e_extract_kpi.message);
        warning('%s', error_message_kpi);
        disp('    错误详情:');
        disp(e_extract_kpi);
        analysis_summary(i_exc_point_analysis).analysis_notes_or_error = ...
            [analysis_summary(i_exc_point_analysis).analysis_notes_or_error, ' [KPI提取/分析失败: ', e_extract_kpi.message, ']'];
        analysis_summary(i_exc_point_analysis).overall_score = -inf; % 标记为无效得分
    end
    disp(' '); % 增加间距
end % 结束对所有有效仿真输出的分析循环
disp(['步骤2完成: 所有 ', num2str(num_outputs_to_analyze), ' 个有效仿真输出的KPI提取和评分已完成。']);
disp(newline);


% --- 步骤3: 显示KPI摘要表并确定最佳激励点 ---
disp('步骤3: 显示KPI摘要并确定最佳激励点...');
disp(newline);
disp('--- KPI 综合分析摘要表 ---');
% 定义表头格式和行格式字符串
header_format_str  = '%-6s | %-40s | %-8s | %-10s | %-12s | %-10s | %-25s | %-10s | %s\n';
row_format_str     = '%-6d | %-40.40s | %-8s | %-10s | %-12.3f | %-10s | %-25.25s | %-10.2f | %s\n';
error_row_format_str = '%-6d | %-40.40s | %-8s | %-10s | %-12s | %-10s | %-25s | %-10s | %s\n';

fprintf(header_format_str, ...
    'ExcID', '激励点路径', '尝试仿真', '仿真成功', '最早脱落时间(s)', '脱落数/总数', '首个脱落果实ID', '综合得分', '分析备注/错误');
disp(repmat('-', 1, 160)); % 打印分隔线

for i_summary_row = 1:num_excitation_points
    summary_item = analysis_summary(i_summary_row);
    
    % 清理和截断备注信息，使其在表格中不过长
    remarks_to_print = strtrim(summary_item.analysis_notes_or_error);
    if length(remarks_to_print) > 50 % 如果备注太长，截断
        remarks_to_print = [remarks_to_print(1:47), '...'];
    end
    
    num_detached_display_str = sprintf('%d/%d', summary_item.num_fruits_detached, num_fruits_total);

    if summary_item.sim_output_data_available % 如果数据可用且分析过
        fprintf(row_format_str, ...
                summary_item.excitation_target_original_idx, ...
                summary_item.excitation_target_path, ...
                mat2str(summary_item.simulation_attempted), ...
                mat2str(summary_item.simulation_succeeded_technically), ...
                summary_item.earliest_detachment_time_s, ...
                num_detached_display_str, ...
                summary_item.first_fruit_to_detach_id, ...
                summary_item.overall_score, ...
                remarks_to_print);
    else % 如果仿真未成功或数据不可用
        fprintf(error_row_format_str, ...
                summary_item.excitation_target_original_idx, ...
                summary_item.excitation_target_path, ...
                mat2str(summary_item.simulation_attempted), ...
                mat2str(summary_item.simulation_succeeded_technically), ... % 可能尝试了但技术上失败
                'N/A', ... % 最早脱落时间
                'N/A', ... % 脱落数量
                'N/A', ... % 首个脱落果实ID
                'N/A', ... % 得分
                remarks_to_print); % 显示失败原因或备注
    end
end
disp(repmat('-', 1, 160)); % 打印分隔线
disp(newline);

% 找出最佳激励点 (基于最高的 overall_score)
% 只考虑那些仿真数据可用且评分有效的条目
valid_scores_for_max = -inf(num_excitation_points, 1); % 初始化一个得分向量
for i_score_check = 1:num_excitation_points
    if analysis_summary(i_score_check).sim_output_data_available && ...
       isfinite(analysis_summary(i_score_check).overall_score) % 确保得分是有限的数字
        valid_scores_for_max(i_score_check) = analysis_summary(i_score_check).overall_score;
    end
end

best_kpi_score_value = max(valid_scores_for_max);
best_excitation_point_analysis_idx_list = find(best_kpi_score_value == valid_scores_for_max);

if ~isempty(best_excitation_point_analysis_idx_list) && best_kpi_score_value > -inf
    fprintf('\n--- 最佳激励配置 (基于综合得分) ---\n');

    if length(best_excitation_point_analysis_idx_list) > 1
        fprintf('发现 %d 个并列最佳激励点，详细信息如下:\n', length(best_excitation_point_analysis_idx_list));
    else
        fprintf('找到唯一的最佳激励点，详细信息如下:\n');
    end
    disp(repmat('-',1,40)); % Separator for each best point details

    % --- 修改开始：遍历所有并列的最佳激励点 ---
    for i_best = 1:length(best_excitation_point_analysis_idx_list)
        current_best_idx = best_excitation_point_analysis_idx_list(i_best);
        best_excitation_target_info = analysis_summary(current_best_idx); % 获取当前最佳激励点的信息

        if length(best_excitation_point_analysis_idx_list) > 1
            fprintf('\n并列最佳点 %d/%d:\n', i_best, length(best_excitation_point_analysis_idx_list));
        end

        fprintf('激励点原始ID (ExcID):      %d\n', best_excitation_target_info.excitation_target_original_idx);
        fprintf('激励点名称:                %s\n', best_excitation_target_info.excitation_target_name);
        fprintf('综合得分:                  %.2f\n', best_excitation_target_info.overall_score);
        fprintf('最早脱落时间 (s):          %.3f\n', best_excitation_target_info.earliest_detachment_time_s);
        fprintf('脱落果实数量:              %d / %d\n', best_excitation_target_info.num_fruits_detached, num_fruits_total);

        % 获取并显示当前最佳激励点下全部脱落果实的ID
        all_detached_fruit_ids_list_current_best = {};
        if ~isempty(best_excitation_target_info.fruit_kpi_data)
            for k_fruit = 1:length(best_excitation_target_info.fruit_kpi_data)
                if best_excitation_target_info.fruit_kpi_data(k_fruit).is_detached
                    all_detached_fruit_ids_list_current_best{end+1} = best_excitation_target_info.fruit_kpi_data(k_fruit).fruit_id;
                end
            end
        end
        
        if isempty(all_detached_fruit_ids_list_current_best)
            all_detached_fruits_str_current_best = '无';
        else
            all_detached_fruits_str_current_best = strjoin(all_detached_fruit_ids_list_current_best, ', ');
        end
        fprintf('全部脱落果实ID:            %s\n', all_detached_fruits_str_current_best);

        if ~isempty(best_excitation_target_info.analysis_notes_or_error)
            fprintf('分析备注/错误:             %s\n', best_excitation_target_info.analysis_notes_or_error);
        end
        if i_best < length(best_excitation_point_analysis_idx_list)
             disp(repmat('-',1,40)); % Separator between multiple best points
        end
    end
    
    % --- (可选) 绘制最佳激励点下各果实的响应曲线 ---
    if best_excitation_target_info.sim_output_data_available && num_fruits_total > 0 && ~isempty(all_fruit_signals_info)
        disp(newline);
        disp('--- 正在为最佳激励点下的各果实绘制响应曲线 ---');
        best_result_sim_output_obj_for_plot = best_excitation_target_info.sim_output_handle;
        plot_signal_data_map = containers.Map('KeyType','char','ValueType','any'); % 用于绘图的数据

        % 重新提取一次数据到 plot_signal_data_map，确保与之前分析时的一致
        % (或者直接复用之前分析时生成的 signal_data_map，如果它被妥善保存了)
        % 这里为了独立性，重新提取一遍：
        if isprop(best_result_sim_output_obj_for_plot, 'logsout') && ...
           ~isempty(best_result_sim_output_obj_for_plot.logsout) && ...
           isa(best_result_sim_output_obj_for_plot.logsout, 'Simulink.SimulationData.Dataset')
            dataset_plot = best_result_sim_output_obj_for_plot.logsout;
            for k_elem_p = 1:dataset_plot.numElements
                element_p = dataset_plot.getElement(k_elem_p);
                if ~isempty(element_p.Name) && isvarname(element_p.Name)
                    if isa(element_p, 'Simulink.SimulationData.Signal') && isa(element_p.Values, 'timeseries')
                        plot_signal_data_map(element_p.Name) = element_p.Values;
                    elseif isa(element_p.Values, 'timeseries')
                        plot_signal_data_map(element_p.Name) = element_p.Values;
                    end
                end
            end
        end
        if isempty(plot_signal_data_map) % 如果logsout没有或为空，尝试从顶层属性提取
            all_props_plot = properties(best_result_sim_output_obj_for_plot);
            for k_p = 1:length(all_props_plot)
                p_name = all_props_plot{k_p};
                if any(strcmp(p_name,{'SimulationMetadata','ErrorMessage','tout','logsout'})), continue; end
                p_val = best_result_sim_output_obj_for_plot.(p_name);
                if isa(p_val,'timeseries'), plot_signal_data_map(p_name) = p_val;
                elseif isa(p_val,'Simulink.SimulationData.Signal') && isa(p_val.Values, 'timeseries'), plot_signal_data_map(p_name) = p_val.Values;
                end
            end
        end
        
        % 获取时间向量，优先从 tout 获取
        time_vector_for_plot = [];
        if isprop(best_result_sim_output_obj_for_plot,'tout') && ~isempty(best_result_sim_output_obj_for_plot.tout)
            time_vector_for_plot = best_result_sim_output_obj_for_plot.tout;
        end

        for i_fruit_plot = 1:num_fruits_total
            current_fruit_signal_info_for_plot = all_fruit_signals_info{i_fruit_plot};
            fruit_id_for_plot_title = strrep(current_fruit_signal_info_for_plot.unique_id, '_', '\_');

            % --- 获取KPI数据 ---
            current_fruit_kpi = best_excitation_target_info.fruit_kpi_data(i_fruit_plot);
            is_detached = current_fruit_kpi.is_detached;
            detachment_time = current_fruit_kpi.detachment_time_s;
            fruit_detach_info = "未脱落";
            if ismember(current_fruit_signal_info_for_plot.unique_id, all_detached_fruit_ids_list_current_best), fruit_detach_info = "脱落"; end

            % --- 创建图形和标题 ---
            figure('Name', ['果实响应: ', current_fruit_signal_info_for_plot.unique_id, ' (最佳激励)'], 'NumberTitle', 'off');
            tcl = tiledlayout(2, 2);
            tcl.Padding = 'compact'; tcl.TileSpacing = 'compact';
            title(tcl, sprintf('最佳激励点: %s (ExcID: %d)\n当前果实: %s (%s)', ...
                strrep(best_excitation_target_info.excitation_target_name, '_', '\_'), ...
                best_excitation_target_info.excitation_target_original_idx, ...
                fruit_id_for_plot_title, fruit_detach_info), 'Interpreter', 'tex');
            
            % --- 绘制四个子图 ---
            
            % Y-位移 (显示动态位移)
            ax1 = nexttile; hold(ax1, 'on'); grid on;
            plot_dynamic_signal(ax1, plot_signal_data_map, current_fruit_signal_info_for_plot.y_disp_varname, '果实', '-', 'b', true); % <-- 修改点
            if isfield(current_fruit_signal_info_for_plot, 'parent_tip_y_disp_varname')
                plot_dynamic_signal(ax1, plot_signal_data_map, current_fruit_signal_info_for_plot.parent_tip_y_disp_varname, '分枝尖端', '--', 'r', true); % <-- 修改点
            end
            title(ax1, 'Y方向动态位移'); ylabel(ax1, '位移 (m)'); xlabel(ax1, '时间 (s)'); legend(ax1, 'show', 'Location','best');
            
            % Z-位移 (显示动态位移)
            ax2 = nexttile; hold(ax2, 'on'); grid on;
            plot_dynamic_signal(ax2, plot_signal_data_map, current_fruit_signal_info_for_plot.z_disp_varname, '果实', '-', 'b', true); % <-- 修改点
            if isfield(current_fruit_signal_info_for_plot, 'parent_tip_z_disp_varname')
                plot_dynamic_signal(ax2, plot_signal_data_map, current_fruit_signal_info_for_plot.parent_tip_z_disp_varname, '分枝尖端', '--', 'r', true); % <-- 修改点
            end
            title(ax2, 'Z方向动态位移'); ylabel(ax2, '位移 (m)'); xlabel(ax2, '时间 (s)'); legend(ax2, 'show', 'Location','best');

            % Y-加速度 (显示绝对加速度)
            ax3 = nexttile; hold(ax3, 'on'); grid on;
            plot_dynamic_signal(ax3, plot_signal_data_map, current_fruit_signal_info_for_plot.y_accel_varname, '果实', '-', 'b', false); % <-- 修改点
            if isfield(current_fruit_signal_info_for_plot, 'parent_tip_y_accel_varname')
                plot_dynamic_signal(ax3, plot_signal_data_map, current_fruit_signal_info_for_plot.parent_tip_y_accel_varname, '分枝尖端', '--', 'r', false); % <-- 修改点
            end
            title(ax3, 'Y方向加速度'); ylabel(ax3, '加速度 (m/s^2)'); xlabel(ax3, '时间 (s)'); legend(ax3, 'show', 'Location','best');

            % Z-加速度 (显示绝对加速度)
            ax4 = nexttile; hold(ax4, 'on'); grid on;
            plot_dynamic_signal(ax4, plot_signal_data_map, current_fruit_signal_info_for_plot.z_accel_varname, '果实', '-', 'b', false); % <-- 修改点
            if isfield(current_fruit_signal_info_for_plot, 'parent_tip_z_accel_varname')
                plot_dynamic_signal(ax4, plot_signal_data_map, current_fruit_signal_info_for_plot.parent_tip_z_accel_varname, '分枝尖端', '--', 'r', false); % <-- 修改点
            end
            title(ax4, 'Z方向加速度'); ylabel(ax4, '加速度 (m/s^2)'); xlabel(ax4, '时间 (s)'); legend(ax4, 'show', 'Location','best');

            % 在所有子图上标记脱落时间
            if is_detached && isfinite(detachment_time)
                all_axes_for_line = [ax1, ax2, ax3, ax4];
                for i_ax = 1:length(all_axes_for_line)
                    hold(all_axes_for_line(i_ax), 'on');
                    y_limits = get(all_axes_for_line(i_ax), 'YLim');
                    plot(all_axes_for_line(i_ax), [detachment_time, detachment_time], y_limits, 'k:', 'LineWidth', 1.5, 'DisplayName', sprintf('脱落时刻 (t=%.2fs)', detachment_time));
                    hold(all_axes_for_line(i_ax), 'off');
                    legend(all_axes_for_line(i_ax), 'show');
                end
            end
            
            arrayfun(@(ax) hold(ax, 'off'), [ax1, ax2, ax3, ax4]);
            drawnow;

            % --- 绘制二维运动轨迹（增强版） ---
            figure('Name', ['运动轨迹: ', current_fruit_signal_info_for_plot.unique_id], 'NumberTitle', 'off');
            main_ax = gca;
            hold(main_ax, 'on');      

            % 提取并绘制分枝尖端轨迹
            ts_tip_y = []; ts_tip_z = [];
            if isfield(current_fruit_signal_info_for_plot, 'parent_tip_y_disp_varname') && isKey(plot_signal_data_map, current_fruit_signal_info_for_plot.parent_tip_y_disp_varname), ts_tip_y = plot_signal_data_map(current_fruit_signal_info_for_plot.parent_tip_y_disp_varname); end
            if isfield(current_fruit_signal_info_for_plot, 'parent_tip_z_disp_varname') && isKey(plot_signal_data_map, current_fruit_signal_info_for_plot.parent_tip_z_disp_varname), ts_tip_z = plot_signal_data_map(current_fruit_signal_info_for_plot.parent_tip_z_disp_varname); end
            if isa(ts_tip_y, 'timeseries') && isa(ts_tip_z, 'timeseries') && length(ts_tip_y.Data) == length(ts_tip_z.Data)
                 plot(ts_tip_z.Data, ts_tip_y.Data, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'DisplayName', '分枝尖端轨迹');
            end
            
            % 提取并绘制果实轨迹
            ts_fruit_y = []; ts_fruit_z = [];
            if isKey(plot_signal_data_map, current_fruit_signal_info_for_plot.y_disp_varname)
                ts_fruit_y = plot_signal_data_map(current_fruit_signal_info_for_plot.y_disp_varname);
            end
            if isKey(plot_signal_data_map, current_fruit_signal_info_for_plot.z_disp_varname)
                ts_fruit_z = plot_signal_data_map(current_fruit_signal_info_for_plot.z_disp_varname);
            end

            if isa(ts_fruit_y, 'timeseries') && isa(ts_fruit_z, 'timeseries') && length(ts_fruit_y.Time) == length(ts_fruit_y.Data) && length(ts_fruit_y.Data) == length(ts_fruit_z.Data)
                x_data = ts_fruit_z.Data(:)'; y_data = ts_fruit_y.Data(:)';
                z_data = zeros(size(x_data)); color_data = ts_fruit_y.Time(:)';
                % --- 新增：为彩色轨迹线创建代理图例条目 ---
                % surface对象本身不支持DisplayName，所以我们画一个看不见的plot对象来生成图例
                plot(NaN, NaN, '-k', 'LineWidth', 2, 'DisplayName', '果实轨迹');

                surface([x_data;x_data], [y_data;y_data], [z_data;z_data], [color_data;color_data], 'facecol', 'no', 'edgecol', 'interp', 'linew', 2, 'HandleVisibility', 'off');
                view(2); 
                cb = colorbar; 
                ylabel(cb, '时间 (s)');

                plot(x_data(1), y_data(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'DisplayName', '起点');
                plot(x_data(end), y_data(end), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', '终点');
                
                if is_detached && isfinite(detachment_time)
                    [~, detach_idx] = min(abs(ts_fruit_y.Time - detachment_time));
                    if ~isempty(detach_idx)
                        plot(x_data(detach_idx), y_data(detach_idx), 'r*', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', '脱落点');
                    end
                    
                    % =========================================================================
                    % ======================== 全新的标记代码 ============================
                    %
                    % --- 新增：直接在Colorbar的时间轴上添加脱落时间刻度 ---
                    
                    % 1. 获取Colorbar当前的刻度和标签
                    original_ticks = cb.Ticks;
                    original_tick_labels = cb.TickLabels;
                    
                    % 2. 将脱落时间加入刻度列表
                    new_ticks = [original_ticks, detachment_time];
                    
                    % 3. 创建新的标签列表
                    % 首先，将原有的数字标签转换为字符串cell数组
                    new_tick_labels = cellstr(num2str(original_ticks(:))); 
                    % 然后，添加我们自定义的“脱落”标签
                    new_tick_labels{end+1} = sprintf('脱落 (%.2f s)', detachment_time);
                    
                    % 4. 对新的刻度和标签进行排序，以保持一致性
                    [sorted_ticks, sort_order] = sort(new_ticks);
                    sorted_tick_labels = new_tick_labels(sort_order);
                    
                    % 5. 移除可能因浮点数精度问题产生的重复刻度
                    [unique_ticks, unique_idx] = unique(sorted_ticks, 'stable');
                    unique_tick_labels = sorted_tick_labels(unique_idx);

                    % 6. 将更新后的刻度和标签应用到Colorbar
                    cb.Ticks = unique_ticks;
                    cb.TickLabels = unique_tick_labels;
                    
                    % (可选) 调整标签旋转角度以防重叠
                    cb.TickLabelInterpreter = 'tex'; % 确保可以正常显示
                    % cb.TickLabelRotation = 45; % 如果标签太密集，可以取消注释此行
                    %
                    % =========================================================================
                end
            end
            
            hold(main_ax, 'off');
            title(sprintf('果实 %s 及分枝尖端的二维运动轨迹', fruit_id_for_plot_title), 'Interpreter', 'tex');
            xlabel('Z-位移 (m)'); ylabel('Y-位移 (m)');
            axis equal; grid on; legend('show', 'Location', 'best');
            
            drawnow;            
        end
        disp('已为最佳激励点下的每个果实绘制了位移和加速度响应图。');
    else
        if num_fruits_total == 0
            disp('模型中未定义或未记录任何果实信号，无法为最佳激励运行情况绘图。');
        elseif ~best_excitation_target_info.sim_output_data_available
            disp('警告: 最佳激励目标的仿真输出数据不可用，因此无法绘制其果实响应。');
            if ~isempty(best_excitation_target_info) && ~isempty(best_excitation_target_info.analysis_notes_or_error)
                disp(['  可能的原因: ', best_excitation_target_info.analysis_notes_or_error]);
            end
        end
    end
else
    disp('所有仿真未能有效评分或所有得分均为-inf (可能所有仿真都未导致果实脱落，或数据分析存在问题)。');
    disp('因此，未能确定一个有效的最佳激励配置。');
end

disp('--- 7.1 结果分析和最佳激励点确定完成 ---');
disp('=== 7. 结果分析阶段结束 ===');
disp(newline);

%% === 9. 清理 (可选) ===
% 脚本执行完毕后的可选清理操作。
disp(newline);
disp('=== 8. 脚本执行完毕后的可选操作 ===');

% 提示模型状态
disp(['模型 "', model_name, '" 当前已构建并（可能）已保存（在第6部分仿真运行前执行了 save_system）。']);
disp('如果需要关闭模型并保存最新更改，请取消注释以下命令:');
% disp(['close_system(''', model_name, ''', 1);']); % 1 表示保存更改后关闭

% 提示关闭并行池 (如果之前开启了)
if license('test', 'Distrib_Computing_Toolbox') && ~isempty(gcp('nocreate'))
    disp('检测到活动的并行池。如果不再需要，可以手动关闭它，或取消注释以下命令:');
    % disp('delete(gcp(''nocreate''));');
    % disp('disp(''并行池已关闭。'');');
end

disp('--- 脚本主要流程执行完毕 ---');

%% === START OF AUXILIARY FUNCTION DEFINITIONS ===
% #########################################################################
% ####### 所有辅助函数定义从脚本的这个标记之后开始，直到文件末尾 ###########
% #########################################################################
% 这些函数被主脚本调用，以执行特定的、可复用的任务，如清理子系统、创建质量块、连接元素等。
disp(newline);
%% clean_subsystem_internals
% 功能: 清理指定Simulink子系统内部的所有连线和模块。
%       用于在创建新子系统后，确保其内部是干净的，没有Simulink自动添加的默认元素。
% 输入参数:
%   subsystem_path (char/string): 需要被清理的子系统的完整Simulink路径。
% 输出参数: 无
% 注意: 此函数会修改传入路径的子系统。
function clean_subsystem_internals(subsystem_path)
    % 输入参数校验 (基本)
    if ~ischar(subsystem_path) && ~isstring(subsystem_path)
        error('clean_subsystem_internals: 输入的 subsystem_path 必须是字符向量或字符串。');
    end
    if isempty(subsystem_path)
        error('clean_subsystem_internals: 输入的 subsystem_path 不能为空。');
    end
    if ~bdIsLoaded(bdroot(subsystem_path)) || getSimulinkBlockHandle(subsystem_path) == -1
        warning('clean_subsystem_internals: 子系统 "%s" 未找到或其父模型未加载。清理操作可能失败或无效果。', subsystem_path);
        % return; % 可以选择直接返回，或者让后续 find_system 自然失败
    end

    % fprintf('  辅助函数 clean_subsystem_internals: 开始清理子系统 "%s"...\n', subsystem_path);

    % 步骤1: 删除子系统内部所有连线
    % 'FindAll', 'on' 确保找到所有类型的句柄 (而不仅仅是第一个)
    % 'SearchDepth', 1 只查找子系统内部顶层的连线 (不深入到更内层的子系统)
    % 'Type', 'line' 指定查找对象类型为连线
    all_lines_inside = find_system(subsystem_path, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line');
    if ~isempty(all_lines_inside)
        % fprintf('    发现 %d 条内部连线，正在删除...\n', length(all_lines_inside));
        for i_line = 1:length(all_lines_inside)
            try
                delete_line(all_lines_inside(i_line)); % 使用句柄删除连线
            catch e_line_delete
                warning('clean_subsystem_internals: 无法删除子系统 "%s" 内的连接线 (句柄: %f): %s', ...
                        subsystem_path, all_lines_inside(i_line), e_line_delete.message);
                % 通常删除连线失败不应阻止脚本继续，但应记录警告
            end
        end
        % fprintf('    内部连线删除完毕。\n');
    else
        % fprintf('    子系统 "%s" 内部无连线需要删除。\n', subsystem_path);
    end

    % 步骤2: 删除子系统内部所有模块
    % 'SearchDepth', 1 只查找子系统内部顶层的模块
    % 'LookUnderMasks', 'all' 查看封装子系统内部 (如果需要，但对于清理新子系统通常不需要)
    % 'FollowLinks', 'off' 不跟随库链接 (对于清理，我们只想删除此子系统内的实例)
    all_blocks_inside_paths_raw = find_system(subsystem_path, 'SearchDepth', 1, ...
                                              'LookUnderMasks', 'all', 'FollowLinks', 'off');
    % find_system 返回的路径中会包含子系统本身，需要将其排除
    blocks_to_delete_paths = all_blocks_inside_paths_raw(~strcmp(all_blocks_inside_paths_raw, subsystem_path));

    if ~isempty(blocks_to_delete_paths)
        % fprintf('    发现 %d 个内部模块，正在删除...\n', length(blocks_to_delete_paths));
        for k_block = 1:length(blocks_to_delete_paths)
            block_to_delete_path = blocks_to_delete_paths{k_block};
            try
                % 获取模块名称用于日志记录
                % block_to_delete_name = get_param(block_to_delete_path, 'Name');
                delete_block(block_to_delete_path);
                % fprintf('      已删除内部模块: "%s"\n', block_to_delete_name);
            catch e_block_delete
                warning('clean_subsystem_internals: 无法删除子系统 "%s" 内的模块 "%s": %s', ...
                        subsystem_path, block_to_delete_path, e_block_delete.message);
            end
        end
        % fprintf('    内部模块删除完毕。\n');
    else
        % fprintf('    子系统 "%s" 内部无模块需要删除。\n', subsystem_path);
    end
    % fprintf('  辅助函数 clean_subsystem_internals: 子系统 "%s" 清理完成。\n', subsystem_path);
end % 结束函数 clean_subsystem_internals

%% create_mass_subsystem_2D
% 功能: 在指定的父路径下创建一个代表二维点质量的Simulink子系统。
%       该子系统内部包含牛顿第二定律的积分 (F=ma -> a -> v -> p)，
%       可以接收外部连接力、激励力。
%       注意：此版本假设重力效应在模型外部被补偿，因此内部不包含重力项。
% 输入参数:
%   parent_path (char/string):    新质量块子系统将被创建于此父级路径下。
%   mass_name (char/string):      新质量块子系统的名称。
%   mass_val (double):            质量块的质量值 (kg)。
%   gravity_g_val (double):       (此版本中已忽略) 重力加速度值。
%   num_conn_force_pairs (int):   期望接收的连接力对数量。
%   has_excitation_ports (logical):是否创建外部激励力输入端口。
%   position_vec (double array):  [x, y, width, height] 定义子系统位置和大小。
%   y_initial_condition (char/string): Y方向位移积分器的初始条件值 (字符串形式)。
%   z_initial_condition (char/string): Z方向位移积分器的初始条件值 (字符串形式)。
% 输出参数:
%   mass_sys_actual_path (char):  实际创建的质量块子系统的完整路径。
function mass_sys_actual_path = create_mass_subsystem_2D(parent_path, mass_name, mass_val, ...
                                                       gravity_g_val, num_conn_force_pairs, ...
                                                       has_excitation_ports, position_vec, ...
                                                       y_initial_condition, z_initial_condition)
    
    % --- 为新参数提供默认值，以保持向后兼容 ---
    if nargin < 8
        y_initial_condition = '0'; 
    end
    if nargin < 9
        z_initial_condition = '0';
    end
    
    % 输入参数校验
    if mass_val <= 0
        error('create_mass_subsystem_2D: 质量值 (mass_val) 必须为正数。收到: %f', mass_val);
    end
    if num_conn_force_pairs < 0
        error('create_mass_subsystem_2D: 连接力对数量 (num_conn_force_pairs) 不能为负。收到: %d', num_conn_force_pairs);
    end

    mass_sys_path_tentative = [parent_path, '/', mass_name];
    add_block('simulink/Ports & Subsystems/Subsystem', mass_sys_path_tentative, ...
              'Position', position_vec, 'MakeNameUnique', 'on');
    mass_sys_actual_path = get_param(mass_sys_path_tentative, 'Object').getFullName();
    clean_subsystem_internals(mass_sys_actual_path);

    % --- 定义子系统内部模块的布局参数 ---
    inport_x_pos = 20; inport_y_start = 30;
    inport_width_s = 30; inport_height_s = 20; inport_v_spacing = 30;
    sum_block_x_pos = 80; sum_block_width = 20;
    sum_block_min_height = 20; sum_block_height_per_input = 15;
    gain_inv_mass_x_pos = 130;
    integrator1_x_pos = 190; integrator2_x_pos = 250;
    dynamics_block_width = 40; dynamics_block_height = 30;
    outport_x_pos = 310; outport_y_start = 30;
    outport_width_s = 30; outport_height_s = 20; outport_v_spacing = 30;

    % --- 创建输入端口 ---
    current_inport_number_overall = 1;
    num_y_force_inports = num_conn_force_pairs + has_excitation_ports;
    num_z_force_inports = num_conn_force_pairs + has_excitation_ports;
    
    current_y_pos = inport_y_start;
    % Y方向力输入
    for i_conn = 1:num_conn_force_pairs
        in_name = ['F_conn', num2str(i_conn), '_y_in'];
        add_block('simulink/Sources/In1', [mass_sys_actual_path, '/', in_name], 'Port', num2str(current_inport_number_overall), 'Position', [inport_x_pos, current_y_pos, inport_x_pos + inport_width_s, current_y_pos + inport_height_s]);
        current_y_pos = current_y_pos + inport_v_spacing; current_inport_number_overall = current_inport_number_overall + 1;
    end
    if has_excitation_ports
        add_block('simulink/Sources/In1', [mass_sys_actual_path, '/F_excite_y_in'], 'Port', num2str(current_inport_number_overall), 'Position', [inport_x_pos, current_y_pos, inport_x_pos + inport_width_s, current_y_pos + inport_height_s]);
        current_y_pos = current_y_pos + inport_v_spacing; current_inport_number_overall = current_inport_number_overall + 1;
    end
    y_force_inputs_y_center = inport_y_start + (num_y_force_inports * inport_v_spacing - inport_v_spacing) / 2;
    
    current_y_pos = current_y_pos + 10; % 留出Y和Z的间隔
    z_inport_initial_y = current_y_pos;
    % Z方向力输入
    for i_conn = 1:num_conn_force_pairs
        in_name = ['F_conn', num2str(i_conn), '_z_in'];
        add_block('simulink/Sources/In1', [mass_sys_actual_path, '/', in_name], 'Port', num2str(current_inport_number_overall), 'Position', [inport_x_pos, current_y_pos, inport_x_pos + inport_width_s, current_y_pos + inport_height_s]);
        current_y_pos = current_y_pos + inport_v_spacing; current_inport_number_overall = current_inport_number_overall + 1;
    end
    if has_excitation_ports
        add_block('simulink/Sources/In1', [mass_sys_actual_path, '/F_excite_z_in'], 'Port', num2str(current_inport_number_overall), 'Position', [inport_x_pos, current_y_pos, inport_x_pos + inport_width_s, current_y_pos + inport_height_s]);
    end
    z_force_inputs_y_center = z_inport_initial_y + (num_z_force_inports * inport_v_spacing - inport_v_spacing) / 2;

    % --- Y方向动力学链 ---
    y_chain_vertical_center = y_force_inputs_y_center - dynamics_block_height / 2;
    y_sum_output_source_block = '';
    if num_y_force_inports == 0
        y_sum_output_source_block = 'simulink/Sources/Constant';
        add_block(y_sum_output_source_block, [mass_sys_actual_path, '/ZeroForce_Y_Input'], 'Value', '0', 'Position', [sum_block_x_pos, y_chain_vertical_center, sum_block_x_pos+30, y_chain_vertical_center+30]);
        y_sum_output_source_block = 'ZeroForce_Y_Input'; % For add_line
    else
        y_sum_output_source_block = 'SumForces_Y';
        sum_inputs_str_y = repmat('+', 1, num_y_force_inports);
        sum_y_height = max(sum_block_min_height, num_y_force_inports * sum_block_height_per_input * 0.8);
        sum_y_y_pos = y_force_inputs_y_center - sum_y_height / 2;
        add_block('simulink/Math Operations/Sum', [mass_sys_actual_path, '/', y_sum_output_source_block], 'Inputs', sum_inputs_str_y, 'IconShape', 'rectangular', 'Position', [sum_block_x_pos, sum_y_y_pos, sum_block_x_pos + sum_block_width, sum_y_y_pos + sum_y_height]);
        current_sum_input_idx_y = 1;
        for i_conn = 1:num_conn_force_pairs
            add_line(mass_sys_actual_path, ['F_conn', num2str(i_conn), '_y_in/1'], [y_sum_output_source_block, '/', num2str(current_sum_input_idx_y)]);
            current_sum_input_idx_y = current_sum_input_idx_y + 1;
        end
        if has_excitation_ports
            add_line(mass_sys_actual_path, 'F_excite_y_in/1', [y_sum_output_source_block, '/', num2str(current_sum_input_idx_y)]);
        end
    end
    
    add_block('simulink/Commonly Used Blocks/Gain', [mass_sys_actual_path, '/InverseMass_Y'], 'Gain', ['1/', num2str(mass_val, '%.4g')], 'Position', [gain_inv_mass_x_pos, y_chain_vertical_center, gain_inv_mass_x_pos + dynamics_block_width, y_chain_vertical_center + dynamics_block_height]);
    add_line(mass_sys_actual_path, [y_sum_output_source_block, '/1'], 'InverseMass_Y/1');
    add_block('simulink/Continuous/Integrator', [mass_sys_actual_path, '/Integrator_Velocity_Y'], 'Position', [integrator1_x_pos, y_chain_vertical_center, integrator1_x_pos + dynamics_block_width, y_chain_vertical_center + dynamics_block_height]);
    add_line(mass_sys_actual_path, 'InverseMass_Y/1', 'Integrator_Velocity_Y/1');
    % *** 修改点: 为位移积分器设置初始条件 ***
    add_block('simulink/Continuous/Integrator', [mass_sys_actual_path, '/Integrator_Displacement_Y'], ...
              'Position', [integrator2_x_pos, y_chain_vertical_center, integrator2_x_pos + dynamics_block_width, y_chain_vertical_center + dynamics_block_height], ...
              'InitialCondition', y_initial_condition); % <-- 新增    
    add_line(mass_sys_actual_path, 'Integrator_Velocity_Y/1', 'Integrator_Displacement_Y/1');

    % --- Z方向动力学链 --- (逻辑与Y方向对称)
    z_chain_vertical_center = z_force_inputs_y_center - dynamics_block_height / 2;
    z_sum_output_source_block = '';
    if num_z_force_inports == 0
        z_sum_output_source_block = 'simulink/Sources/Constant';
        add_block(z_sum_output_source_block, [mass_sys_actual_path, '/ZeroForce_Z_Input'], 'Value', '0', 'Position', [sum_block_x_pos, z_chain_vertical_center, sum_block_x_pos+30, z_chain_vertical_center+30]);
        z_sum_output_source_block = 'ZeroForce_Z_Input';
    else
        z_sum_output_source_block = 'SumForces_Z';
        sum_inputs_str_z = repmat('+', 1, num_z_force_inports);
        sum_z_height = max(sum_block_min_height, num_z_force_inports * sum_block_height_per_input * 0.8);
        sum_z_y_pos = z_force_inputs_y_center - sum_z_height / 2;
        add_block('simulink/Math Operations/Sum', [mass_sys_actual_path, '/', z_sum_output_source_block], 'Inputs', sum_inputs_str_z, 'IconShape', 'rectangular', 'Position', [sum_block_x_pos, sum_z_y_pos, sum_block_x_pos + sum_block_width, sum_z_y_pos + sum_z_height]);
        current_sum_input_idx_z = 1;
        for i_conn = 1:num_conn_force_pairs
            add_line(mass_sys_actual_path, ['F_conn', num2str(i_conn), '_z_in/1'], [z_sum_output_source_block, '/', num2str(current_sum_input_idx_z)]);
            current_sum_input_idx_z = current_sum_input_idx_z + 1;
        end
        if has_excitation_ports
            add_line(mass_sys_actual_path, 'F_excite_z_in/1', [z_sum_output_source_block, '/', num2str(current_sum_input_idx_z)]);
        end
    end
    
    add_block('simulink/Commonly Used Blocks/Gain', [mass_sys_actual_path, '/InverseMass_Z'], 'Gain', ['1/', num2str(mass_val, '%.4g')], 'Position', [gain_inv_mass_x_pos, z_chain_vertical_center, gain_inv_mass_x_pos + dynamics_block_width, z_chain_vertical_center + dynamics_block_height]);
    add_line(mass_sys_actual_path, [z_sum_output_source_block, '/1'], 'InverseMass_Z/1');
    add_block('simulink/Continuous/Integrator', [mass_sys_actual_path, '/Integrator_Velocity_Z'], 'Position', [integrator1_x_pos, z_chain_vertical_center, integrator1_x_pos + dynamics_block_width, z_chain_vertical_center + dynamics_block_height]);
    add_line(mass_sys_actual_path, 'InverseMass_Z/1', 'Integrator_Velocity_Z/1');
    % *** 修改点: 为位移积分器设置初始条件 ***
    add_block('simulink/Continuous/Integrator', [mass_sys_actual_path, '/Integrator_Displacement_Z'], ...
              'Position', [integrator2_x_pos, z_chain_vertical_center, integrator2_x_pos + dynamics_block_width, z_chain_vertical_center + dynamics_block_height], ...
              'InitialCondition', z_initial_condition); % <-- 新增
    add_line(mass_sys_actual_path, 'Integrator_Velocity_Z/1', 'Integrator_Displacement_Z/1');

    % --- 创建输出端口 ---
    current_outport_number = 1;
    current_y_pos_for_outports = outport_y_start;
    
    add_block('simulink/Sinks/Out1', [mass_sys_actual_path, '/y_out'], 'Port', num2str(current_outport_number), 'Position', [outport_x_pos, current_y_pos_for_outports, outport_x_pos + outport_width_s, current_y_pos_for_outports + outport_height_s]);
    add_line(mass_sys_actual_path, 'Integrator_Displacement_Y/1', 'y_out/1');
    current_y_pos_for_outports = current_y_pos_for_outports + outport_v_spacing; current_outport_number = current_outport_number + 1;

    add_block('simulink/Sinks/Out1', [mass_sys_actual_path, '/vy_out'], 'Port', num2str(current_outport_number), 'Position', [outport_x_pos, current_y_pos_for_outports, outport_x_pos + outport_width_s, current_y_pos_for_outports + outport_height_s]);
    add_line(mass_sys_actual_path, 'Integrator_Velocity_Y/1', 'vy_out/1');
    current_y_pos_for_outports = current_y_pos_for_outports + outport_v_spacing; current_outport_number = current_outport_number + 1;

    add_block('simulink/Sinks/Out1', [mass_sys_actual_path, '/z_out'], 'Port', num2str(current_outport_number), 'Position', [outport_x_pos, current_y_pos_for_outports, outport_x_pos + outport_width_s, current_y_pos_for_outports + outport_height_s]);
    add_line(mass_sys_actual_path, 'Integrator_Displacement_Z/1', 'z_out/1');
    current_y_pos_for_outports = current_y_pos_for_outports + outport_v_spacing; current_outport_number = current_outport_number + 1;

    add_block('simulink/Sinks/Out1', [mass_sys_actual_path, '/vz_out'], 'Port', num2str(current_outport_number), 'Position', [outport_x_pos, current_y_pos_for_outports, outport_x_pos + outport_width_s, current_y_pos_for_outports + outport_height_s]);
    add_line(mass_sys_actual_path, 'Integrator_Velocity_Z/1', 'vz_out/1');
    current_y_pos_for_outports = current_y_pos_for_outports + outport_v_spacing; current_outport_number = current_outport_number + 1;

    add_block('simulink/Sinks/Out1', [mass_sys_actual_path, '/ay_out'], 'Port', num2str(current_outport_number), 'Position', [outport_x_pos, current_y_pos_for_outports, outport_x_pos + outport_width_s, current_y_pos_for_outports + outport_height_s]);
    add_line(mass_sys_actual_path, 'InverseMass_Y/1', 'ay_out/1');
    current_y_pos_for_outports = current_y_pos_for_outports + outport_v_spacing; current_outport_number = current_outport_number + 1;

    add_block('simulink/Sinks/Out1', [mass_sys_actual_path, '/az_out'], 'Port', num2str(current_outport_number), 'Position', [outport_x_pos, current_y_pos_for_outports, outport_x_pos + outport_width_s, current_y_pos_for_outports + outport_height_s]);
    add_line(mass_sys_actual_path, 'InverseMass_Z/1', 'az_out/1');
end
%% connect_elements_2D
% 功能: 在指定的父模型/子系统内部，创建连接两个元件 (通常是一个上游元件和一个下游质量块)
%       的二维 (Y和Z方向) 弹簧-阻尼器连接。
%       计算相对位移和相对速度，然后乘以刚度(k)和阻尼(c)得到连接力。
%       此力作用于下游质量块，其反作用力作用于上游元件。
% 输入参数:
%   parent_model_path (char/string):  连接元件将被创建于此父级路径下。
%   upstream_state_provider_path (char/string): 提供上游元件状态 (y,vy,z,vz) 的模块的完整路径。
%   upstream_state_provider_type (char/string): 上游状态提供模块的类型 ('BusSelector' 或 'Mass')。
%   downstream_mass_path (char/string): 需要接收连接力的下游质量块子系统的完整路径。
%   downstream_mass_name_prefix (char/string): 下游质量块的名称前缀，用于生成连接块的名称。
%   k_y, c_y (double):                Y方向连接的刚度和阻尼系数。
%   k_z, c_z (double):                Z方向连接的刚度和阻尼系数。
%   upstream_reaction_target_path (char/string): 
%                                     - 如果 is_upstream_fixed=true: 固定基座(FixedBase)的路径。
%                                     - 如果 is_cross_boundary_reaction=true (且非fixed): 当前parent_model_path (反作用力通过Outport)。
%                                     - 否则: 上游质量块子系统的路径。
%   upstream_F_react_port_spec_y (char/string or double):
%   upstream_F_react_port_spec_z (char/string or double):
%                                     - 如果 is_upstream_fixed=true or is_cross_boundary_reaction=true:
%                                       它们是 parent_model_path 中用于输出反作用力的 Outport 模块的 *期望名称*。
%                                     - 否则: 它们是上游质量块上 F_connX_y/z_in 的连接力对 *索引号* (double)。
%   downstream_F_conn_pair_idx (double): 下游质量块上 F_connX_y/z_in 的连接力对 *索引号*。
%   conn_block_base_pos_xy (double array): [x, y] 连接逻辑模块组在父级中的大致基准位置。
%   is_upstream_fixed (logical):      布尔值，指示上游元件是否为固定基座。
%   is_cross_boundary_reaction (logical):布尔值，指示此连接是否跨越一个主要子系统边界。
%   layout_params_struct (struct):    包含Simulink模块布局参数的结构体。
% 输出参数: 无
function connect_elements_2D(parent_model_path, ...
                               upstream_state_provider_path, upstream_state_provider_type, ...
                               downstream_mass_path, downstream_mass_name_prefix, ...
                               k_y, c_y, k_z, c_z, ...
                               upstream_reaction_target_path, ...
                               upstream_F_react_port_spec_y, upstream_F_react_port_spec_z, ...
                               downstream_F_conn_pair_idx, conn_block_base_pos_xy, ...
                               is_upstream_fixed, is_cross_boundary_reaction, layout_params_struct)

    top_level_model_name = bdroot(parent_model_path);

    [~, upstream_provider_name_part] = fileparts(upstream_state_provider_path); 
    conn_block_name_base = matlab.lang.makeValidName( ...
        sprintf('%s_connTo_%s_idx%d', downstream_mass_name_prefix, ...
                upstream_provider_name_part, downstream_F_conn_pair_idx) ...
    );
    % fprintf('  connect_elements_2D: 创建连接 "%s" in "%s"...\n', conn_block_name_base, parent_model_path);

    downstream_mass_rel_path = strrep(downstream_mass_path, [parent_model_path, '/'], '');
    upstream_provider_rel_path = strrep(upstream_state_provider_path, [parent_model_path, '/'], '');

    upstream_y_source_relport = ''; upstream_vy_source_relport = '';
    upstream_z_source_relport = ''; upstream_vz_source_relport = '';

    switch lower(upstream_state_provider_type)
        case 'busselector'
            upstream_y_source_relport  = [upstream_provider_rel_path, '/1'];
            upstream_vy_source_relport = [upstream_provider_rel_path, '/2'];
            upstream_z_source_relport  = [upstream_provider_rel_path, '/3'];
            upstream_vz_source_relport = [upstream_provider_rel_path, '/4'];
        case 'mass' 
            upstream_y_source_relport  = [upstream_provider_rel_path, '/1'];
            upstream_vy_source_relport = [upstream_provider_rel_path, '/2'];
            upstream_z_source_relport  = [upstream_provider_rel_path, '/3'];
            upstream_vz_source_relport = [upstream_provider_rel_path, '/4'];
        otherwise
            error('connect_elements_2D: 不支持的上游状态提供者类型: %s。请使用 "BusSelector" 或 "Mass"。', upstream_state_provider_type);
    end
    
    cb_x = conn_block_base_pos_xy(1); 
    cb_y_start_y_dir = conn_block_base_pos_xy(2) - 80; % Y方向连接Y起始调整
    cb_y_start_z_dir = conn_block_base_pos_xy(2) + 70; % Z方向连接Y起始调整 (留出约150间距)
    
    sum_w = 20; sum_h = 40; 
    gain_w = 30; gain_h = 20; 
    h_spacing = 25; % 水平间距调整         
    
    % --- Y方向连接逻辑 ---
    dy_sum_name = [conn_block_name_base, '_dy_sum'];
    add_block('simulink/Math Operations/Sum', [parent_model_path, '/', dy_sum_name], ...
              'Inputs', '+-', 'IconShape', 'rectangular', ... 
              'Position', [cb_x, cb_y_start_y_dir, cb_x + sum_w, cb_y_start_y_dir + sum_h]);
    add_line(parent_model_path, upstream_y_source_relport, [dy_sum_name, '/1']); 
    add_line(parent_model_path, [downstream_mass_rel_path, '/1'], [dy_sum_name, '/2']); 

    dvy_sum_name = [conn_block_name_base, '_dvy_sum'];
    add_block('simulink/Math Operations/Sum', [parent_model_path, '/', dvy_sum_name], ...
              'Inputs', '+-', 'IconShape', 'rectangular', ...
              'Position', [cb_x, cb_y_start_y_dir + sum_h + 15, cb_x + sum_w, cb_y_start_y_dir + sum_h*2 + 15]);
    add_line(parent_model_path, upstream_vy_source_relport, [dvy_sum_name, '/1']); 
    add_line(parent_model_path, [downstream_mass_rel_path, '/2'], [dvy_sum_name, '/2']); 

    ky_gain_name = [conn_block_name_base, '_ky_gain'];
    add_block('simulink/Commonly Used Blocks/Gain', [parent_model_path, '/', ky_gain_name], ...
              'Gain', num2str(k_y, '%.4g'), ...
              'Position', [cb_x + sum_w + h_spacing, cb_y_start_y_dir + (sum_h-gain_h)/2, ...
                           cb_x + sum_w + h_spacing + gain_w, cb_y_start_y_dir + (sum_h+gain_h)/2]);
    add_line(parent_model_path, [dy_sum_name, '/1'], [ky_gain_name, '/1']);

    cy_gain_name = [conn_block_name_base, '_cy_gain'];
    add_block('simulink/Commonly Used Blocks/Gain', [parent_model_path, '/', cy_gain_name], ...
              'Gain', num2str(c_y, '%.4g'), ...
              'Position', [cb_x + sum_w + h_spacing, cb_y_start_y_dir + sum_h + 15 + (sum_h-gain_h)/2, ...
                           cb_x + sum_w + h_spacing + gain_w, cb_y_start_y_dir + sum_h + 15 + (sum_h+gain_h)/2]);
    add_line(parent_model_path, [dvy_sum_name, '/1'], [cy_gain_name, '/1']);

    f_conn_y_sum_name = [conn_block_name_base, '_F_conn_y_sum'];
    f_conn_y_sum_x_pos = cb_x + sum_w + h_spacing + gain_w + h_spacing;
    add_block('simulink/Math Operations/Sum', [parent_model_path, '/', f_conn_y_sum_name], ...
              'Inputs', '++', 'IconShape', 'rectangular', ...
              'Position', [f_conn_y_sum_x_pos, cb_y_start_y_dir + sum_h + (15-sum_h)/2, ... 
                           f_conn_y_sum_x_pos + sum_w, cb_y_start_y_dir + sum_h + (15+sum_h)/2]);
    add_line(parent_model_path, [ky_gain_name, '/1'], [f_conn_y_sum_name, '/1']);
    add_line(parent_model_path, [cy_gain_name, '/1'], [f_conn_y_sum_name, '/2']);
    
    downstream_inport_y_name_local = ['F_conn', num2str(downstream_F_conn_pair_idx), '_y_in'];
    downstream_inport_y_full_path_local = [downstream_mass_path, '/', downstream_inport_y_name_local];
    downstream_port_num_y_str_local = get_param(downstream_inport_y_full_path_local, 'Port');
    downstream_F_conn_y_target_relport_str = [downstream_mass_rel_path, '/', downstream_port_num_y_str_local];
    add_line(parent_model_path, [f_conn_y_sum_name, '/1'], downstream_F_conn_y_target_relport_str);

    f_react_y_gain_name = [conn_block_name_base, '_F_react_y_negain']; 
    f_react_y_gain_x_pos = f_conn_y_sum_x_pos; 
    f_react_y_gain_y_pos = cb_y_start_y_dir - gain_h - 10; 
    add_block('simulink/Commonly Used Blocks/Gain', [parent_model_path, '/', f_react_y_gain_name], ...
              'Gain', '-1', ...
              'Position', [f_react_y_gain_x_pos, f_react_y_gain_y_pos, ...
                           f_react_y_gain_x_pos + gain_w, f_react_y_gain_y_pos + gain_h]);
    add_line(parent_model_path, [f_conn_y_sum_name, '/1'], [f_react_y_gain_name, '/1']); 

    if is_upstream_fixed || is_cross_boundary_reaction
        outport_for_react_y_name_in_parent = upstream_F_react_port_spec_y; % 这是期望的Outport名称

        existing_outports_in_parent_y = find_system(parent_model_path, 'SearchDepth', 1, 'BlockType', 'Outport', 'Name', outport_for_react_y_name_in_parent);
        if isempty(existing_outports_in_parent_y)
            all_outports_in_parent = find_system(parent_model_path, 'SearchDepth', 1, 'BlockType', 'Outport');
            next_outport_num_y = length(all_outports_in_parent) + 1;
            outport_y_layout_pos_x = f_react_y_gain_x_pos + gain_w + 30; % Gain右侧
            outport_y_layout_pos_y = f_react_y_gain_y_pos + (gain_h - layout_params_struct.general_outport_height)/2;
            
            add_block('simulink/Sinks/Out1', [parent_model_path, '/', outport_for_react_y_name_in_parent], ...
                      'Port', num2str(next_outport_num_y), ...
                      'Position', [outport_y_layout_pos_x, outport_y_layout_pos_y, ...
                                   outport_y_layout_pos_x + layout_params_struct.general_outport_width, ...
                                   outport_y_layout_pos_y + layout_params_struct.general_outport_height]);
        end
        add_line(parent_model_path, [f_react_y_gain_name, '/1'], [outport_for_react_y_name_in_parent, '/1']);
        
        if is_upstream_fixed 
            parent_model_rel_path_for_top = strrep(parent_model_path, [top_level_model_name, '/'], '');
            fixed_base_rel_path_for_top = strrep(upstream_reaction_target_path, [top_level_model_name, '/'], ''); 
            
            parent_outport_block_y_fullname_local = [parent_model_path, '/', outport_for_react_y_name_in_parent];
            parent_outport_port_num_str_y_local = get_param(parent_outport_block_y_fullname_local, 'Port');
            
            fixed_base_inport_y_name = 'F_react_y_in'; 
            fixed_base_inport_y_port_num_str = get_param([upstream_reaction_target_path, '/', fixed_base_inport_y_name], 'Port');
            
            src_y_top = [parent_model_rel_path_for_top, '/', parent_outport_port_num_str_y_local];
            dst_y_top = [fixed_base_rel_path_for_top, '/', fixed_base_inport_y_port_num_str];
            add_line(top_level_model_name, src_y_top, dst_y_top, 'autorouting', 'on');
        end
    else
        upstream_reaction_target_rel_path = strrep(upstream_reaction_target_path, [parent_model_path, '/'], '');
        upstream_inport_y_react_name_local = ['F_conn', num2str(upstream_F_react_port_spec_y), '_y_in'];
        upstream_inport_y_react_full_path_local = [upstream_reaction_target_path, '/', upstream_inport_y_react_name_local];
        upstream_port_num_y_react_str_local = get_param(upstream_inport_y_react_full_path_local, 'Port');
        add_line(parent_model_path, [f_react_y_gain_name, '/1'], ...
                 [upstream_reaction_target_rel_path, '/', upstream_port_num_y_react_str_local]);
    end

    % --- Z方向连接逻辑 --- (与Y方向对称)
    dz_sum_name = [conn_block_name_base, '_dz_sum'];
    add_block('simulink/Math Operations/Sum', [parent_model_path, '/', dz_sum_name], 'Inputs', '+-', 'IconShape', 'rectangular', ...
              'Position', [cb_x, cb_y_start_z_dir, cb_x + sum_w, cb_y_start_z_dir + sum_h]);
    add_line(parent_model_path, upstream_z_source_relport, [dz_sum_name, '/1']); 
    add_line(parent_model_path, [downstream_mass_rel_path, '/3'], [dz_sum_name, '/2']); 

    dvz_sum_name = [conn_block_name_base, '_dvz_sum'];
    add_block('simulink/Math Operations/Sum', [parent_model_path, '/', dvz_sum_name], 'Inputs', '+-', 'IconShape', 'rectangular', ...
              'Position', [cb_x, cb_y_start_z_dir + sum_h + 15, cb_x + sum_w, cb_y_start_z_dir + sum_h*2 + 15]);
    add_line(parent_model_path, upstream_vz_source_relport, [dvz_sum_name, '/1']); 
    add_line(parent_model_path, [downstream_mass_rel_path, '/4'], [dvz_sum_name, '/2']); 

    kz_gain_name = [conn_block_name_base, '_kz_gain'];
    add_block('simulink/Commonly Used Blocks/Gain', [parent_model_path, '/', kz_gain_name], 'Gain', num2str(k_z, '%.4g'), ...
              'Position', [cb_x + sum_w + h_spacing, cb_y_start_z_dir + (sum_h-gain_h)/2, ...
                           cb_x + sum_w + h_spacing + gain_w, cb_y_start_z_dir + (sum_h+gain_h)/2]);
    add_line(parent_model_path, [dz_sum_name, '/1'], [kz_gain_name, '/1']);

    cz_gain_name = [conn_block_name_base, '_cz_gain'];
    add_block('simulink/Commonly Used Blocks/Gain', [parent_model_path, '/', cz_gain_name], 'Gain', num2str(c_z, '%.4g'), ...
              'Position', [cb_x + sum_w + h_spacing, cb_y_start_z_dir + sum_h + 15 + (sum_h-gain_h)/2, ...
                           cb_x + sum_w + h_spacing + gain_w, cb_y_start_z_dir + sum_h + 15 + (sum_h+gain_h)/2]);
    add_line(parent_model_path, [dvz_sum_name, '/1'], [cz_gain_name, '/1']);

    f_conn_z_sum_name = [conn_block_name_base, '_F_conn_z_sum'];
    f_conn_z_sum_x_pos = cb_x + sum_w + h_spacing + gain_w + h_spacing;
    add_block('simulink/Math Operations/Sum', [parent_model_path, '/', f_conn_z_sum_name], 'Inputs', '++', 'IconShape', 'rectangular', ...
              'Position', [f_conn_z_sum_x_pos, cb_y_start_z_dir + sum_h + (15-sum_h)/2, ...
                           f_conn_z_sum_x_pos + sum_w, cb_y_start_z_dir + sum_h + (15+sum_h)/2]);
    add_line(parent_model_path, [kz_gain_name, '/1'], [f_conn_z_sum_name, '/1']);
    add_line(parent_model_path, [cz_gain_name, '/1'], [f_conn_z_sum_name, '/2']);
    
    downstream_inport_z_name_local = ['F_conn', num2str(downstream_F_conn_pair_idx), '_z_in'];
    downstream_inport_z_full_path_local = [downstream_mass_path, '/', downstream_inport_z_name_local];
    downstream_port_num_z_str_local = get_param(downstream_inport_z_full_path_local, 'Port');
    downstream_F_conn_z_target_relport_str = [downstream_mass_rel_path, '/', downstream_port_num_z_str_local];
    add_line(parent_model_path, [f_conn_z_sum_name, '/1'], downstream_F_conn_z_target_relport_str);

    f_react_z_gain_name = [conn_block_name_base, '_F_react_z_negain'];
    f_react_z_gain_x_pos = f_conn_z_sum_x_pos;
    f_react_z_gain_y_pos = cb_y_start_z_dir - gain_h - 10;
    add_block('simulink/Commonly Used Blocks/Gain', [parent_model_path, '/', f_react_z_gain_name], 'Gain', '-1', ...
              'Position', [f_react_z_gain_x_pos, f_react_z_gain_y_pos, ...
                           f_react_z_gain_x_pos + gain_w, f_react_z_gain_y_pos + gain_h]);
    add_line(parent_model_path, [f_conn_z_sum_name, '/1'], [f_react_z_gain_name, '/1']);

    if is_upstream_fixed || is_cross_boundary_reaction
        outport_for_react_z_name_in_parent = upstream_F_react_port_spec_z;

        existing_outports_in_parent_z = find_system(parent_model_path, 'SearchDepth', 1, 'BlockType', 'Outport', 'Name', outport_for_react_z_name_in_parent);
        if isempty(existing_outports_in_parent_z)
            all_outports_in_parent_z = find_system(parent_model_path, 'SearchDepth', 1, 'BlockType', 'Outport');
            next_outport_num_z = length(all_outports_in_parent_z) + 1;
            outport_z_layout_pos_x = f_react_z_gain_x_pos + gain_w + 30;
            outport_z_layout_pos_y = f_react_z_gain_y_pos + (gain_h - layout_params_struct.general_outport_height)/2;
            
            add_block('simulink/Sinks/Out1', [parent_model_path, '/', outport_for_react_z_name_in_parent], ...
                      'Port', num2str(next_outport_num_z), ...
                      'Position', [outport_z_layout_pos_x, outport_z_layout_pos_y, ...
                                   outport_z_layout_pos_x + layout_params_struct.general_outport_width, ...
                                   outport_z_layout_pos_y + layout_params_struct.general_outport_height]);
        end
        add_line(parent_model_path, [f_react_z_gain_name, '/1'], [outport_for_react_z_name_in_parent, '/1']);
        
        if is_upstream_fixed
            parent_model_rel_path_for_top = strrep(parent_model_path, [top_level_model_name, '/'], '');
            fixed_base_rel_path_for_top = strrep(upstream_reaction_target_path, [top_level_model_name, '/'], '');
            
            parent_outport_block_z_fullname_local = [parent_model_path, '/', outport_for_react_z_name_in_parent];
            parent_outport_port_num_str_z_local = get_param(parent_outport_block_z_fullname_local, 'Port');
            
            fixed_base_inport_z_name = 'F_react_z_in';
            fixed_base_inport_z_port_num_str = get_param([upstream_reaction_target_path, '/', fixed_base_inport_z_name], 'Port');
            
            src_z_top = [parent_model_rel_path_for_top, '/', parent_outport_port_num_str_z_local];
            dst_z_top = [fixed_base_rel_path_for_top, '/', fixed_base_inport_z_port_num_str];
            add_line(top_level_model_name, src_z_top, dst_z_top, 'autorouting', 'on');
        end
    else
        upstream_reaction_target_rel_path = strrep(upstream_reaction_target_path, [parent_model_path, '/'], '');
        upstream_inport_z_react_name_local = ['F_conn', num2str(upstream_F_react_port_spec_z), '_z_in'];
        upstream_inport_z_react_full_path_local = [upstream_reaction_target_path, '/', upstream_inport_z_react_name_local];
        upstream_port_num_z_react_str_local = get_param(upstream_inport_z_react_full_path_local, 'Port');
        add_line(parent_model_path, [f_react_z_gain_name, '/1'], ...
                 [upstream_reaction_target_rel_path, '/', upstream_port_num_z_react_str_local]);
    end
    % fprintf('  connect_elements_2D: 连接 "%s" 构建完成。\n', conn_block_name_base);
end

%% connect_fruit_with_detachment_2D (最终语法修正版)
% 功能: 创建一个包含物理上更稳健、语法正确的脱落逻辑的果柄连接。
function connect_fruit_with_detachment_2D(parent_model_path, tip_mass_path, fruit_mass_path, ...
                                          fruit_unique_id, fruit_params, gravity_g_val, ...
                                          tip_F_react_conn_pair_idx, fruit_F_conn_pair_idx, ...
                                          conn_block_base_pos_xy)
    global fruit_signal_manager_global;

    % --- 创建脱落逻辑子系统 ---
    logic_subsystem_name_base = matlab.lang.makeValidName([fruit_unique_id, '_DetachmentLogic']);
    logic_subsystem_path_tentative = [parent_model_path, '/', logic_subsystem_name_base];
    logic_ss_width = 450; logic_ss_height = 350; % 稍微调大一点高度
    logic_ss_pos = [conn_block_base_pos_xy(1), conn_block_base_pos_xy(2), conn_block_base_pos_xy(1) + logic_ss_width, conn_block_base_pos_xy(2) + logic_ss_height];
    add_block('simulink/Ports & Subsystems/Subsystem', logic_subsystem_path_tentative, 'Position', logic_ss_pos, 'MakeNameUnique', 'on');
    logic_subsystem_actual_path = get_param(logic_subsystem_path_tentative, 'Object').getFullName();
    clean_subsystem_internals(logic_subsystem_actual_path);

    % --- 定义脱落逻辑子系统的输入和输出端口 ---
    input_port_names_logic  = {'y_tip_in', 'vy_tip_in', 'z_tip_in', 'vz_tip_in', 'y_fruit_in', 'vy_fruit_in', 'z_fruit_in', 'vz_fruit_in'};
    output_port_names_logic = {'F_to_fruit_y_out', 'F_to_fruit_z_out', 'F_to_tip_y_out', 'F_to_tip_z_out', 'Detached_Status_out', 'F_pedicel_magnitude_out'};
    in_port_x_logic = 30; in_port_y_start_logic = 30; in_port_w_logic = 30; in_port_h_logic = 18; in_port_v_spacing_logic = 25;
    for i_in = 1:length(input_port_names_logic)
        add_block('simulink/Sources/In1', [logic_subsystem_actual_path, '/', input_port_names_logic{i_in}], 'Port', num2str(i_in), 'Position', [in_port_x_logic, in_port_y_start_logic + (i_in-1)*in_port_v_spacing_logic, in_port_x_logic + in_port_w_logic, in_port_y_start_logic + (i_in-1)*in_port_v_spacing_logic + in_port_h_logic]);
    end
    out_port_x_logic = logic_ss_width - 50; out_port_y_start_logic = 30; out_port_w_logic = 30; out_port_h_logic = 18; out_port_v_spacing_logic = 25;
    for i_out = 1:length(output_port_names_logic)
        add_block('simulink/Sinks/Out1', [logic_subsystem_actual_path, '/', output_port_names_logic{i_out}], 'Port', num2str(i_out), 'Position', [out_port_x_logic, out_port_y_start_logic + (i_out-1)*out_port_v_spacing_logic, out_port_x_logic + out_port_w_logic, out_port_y_start_logic + (i_out-1)*out_port_v_spacing_logic + out_port_h_logic]);
    end

    % --- 构建脱落逻辑子系统内部的详细逻辑 ---
    
    % 1. 计算相对位移和速度 (dy, dvy, dz, dvz)
    add_block('simulink/Math Operations/Sum', [logic_subsystem_actual_path, '/dy_sum'], 'Inputs', '+-', 'Position', [100 40 120 60]); add_line(logic_subsystem_actual_path, 'y_tip_in/1', 'dy_sum/1'); add_line(logic_subsystem_actual_path, 'y_fruit_in/1', 'dy_sum/2');
    add_block('simulink/Math Operations/Sum', [logic_subsystem_actual_path, '/dvy_sum'], 'Inputs', '+-', 'Position', [100 70 120 90]); add_line(logic_subsystem_actual_path, 'vy_tip_in/1', 'dvy_sum/1'); add_line(logic_subsystem_actual_path, 'vy_fruit_in/1', 'dvy_sum/2');
    add_block('simulink/Math Operations/Sum', [logic_subsystem_actual_path, '/dz_sum'], 'Inputs', '+-', 'Position', [100 100 120 120]); add_line(logic_subsystem_actual_path, 'z_tip_in/1', 'dz_sum/1'); add_line(logic_subsystem_actual_path, 'z_fruit_in/1', 'dz_sum/2');
    add_block('simulink/Math Operations/Sum', [logic_subsystem_actual_path, '/dvz_sum'], 'Inputs', '+-', 'Position', [100 130 120 150]); add_line(logic_subsystem_actual_path, 'vz_tip_in/1', 'dvz_sum/1'); add_line(logic_subsystem_actual_path, 'vz_fruit_in/1', 'dvz_sum/2');

    % 2. 无条件计算Y和Z方向的潜在连接力 (F_unbroken)
    add_block('simulink/Commonly Used Blocks/Gain', [logic_subsystem_actual_path, '/ky_gain'], 'Gain', num2str(fruit_params.k_pedicel_y)); add_line(logic_subsystem_actual_path, 'dy_sum/1', 'ky_gain/1');
    add_block('simulink/Commonly Used Blocks/Gain', [logic_subsystem_actual_path, '/cy_gain'], 'Gain', num2str(fruit_params.c_pedicel_y)); add_line(logic_subsystem_actual_path, 'dvy_sum/1', 'cy_gain/1');
    add_block('simulink/Math Operations/Sum', [logic_subsystem_actual_path, '/Fy_unbroken'], 'Inputs', '++'); add_line(logic_subsystem_actual_path, 'ky_gain/1', 'Fy_unbroken/1'); add_line(logic_subsystem_actual_path, 'cy_gain/1', 'Fy_unbroken/2');
    
    add_block('simulink/Commonly Used Blocks/Gain', [logic_subsystem_actual_path, '/kz_gain'], 'Gain', num2str(fruit_params.k_pedicel_z)); add_line(logic_subsystem_actual_path, 'dz_sum/1', 'kz_gain/1');
    add_block('simulink/Commonly Used Blocks/Gain', [logic_subsystem_actual_path, '/cz_gain'], 'Gain', num2str(fruit_params.c_pedicel_z)); add_line(logic_subsystem_actual_path, 'dvz_sum/1', 'cz_gain/1');
    add_block('simulink/Math Operations/Sum', [logic_subsystem_actual_path, '/Fz_unbroken'], 'Inputs', '++'); add_line(logic_subsystem_actual_path, 'kz_gain/1', 'Fz_unbroken/1'); add_line(logic_subsystem_actual_path, 'cz_gain/1', 'Fz_unbroken/2');

    % 3. 计算总力的幅值，用于脱落判断
    add_block('simulink/Math Operations/Math Function', [logic_subsystem_actual_path, '/Fy_sq'], 'Function', 'square'); add_line(logic_subsystem_actual_path, 'Fy_unbroken/1', 'Fy_sq/1');
    add_block('simulink/Math Operations/Math Function', [logic_subsystem_actual_path, '/Fz_sq'], 'Function', 'square'); add_line(logic_subsystem_actual_path, 'Fz_unbroken/1', 'Fz_sq/1');
    add_block('simulink/Math Operations/Sum', [logic_subsystem_actual_path, '/Sum_sq'], 'Inputs', '++'); add_line(logic_subsystem_actual_path, 'Fy_sq/1', 'Sum_sq/1'); add_line(logic_subsystem_actual_path, 'Fz_sq/1', 'Sum_sq/2');
    add_block('simulink/Math Operations/Math Function', [logic_subsystem_actual_path, '/F_mag_sqrt'], 'Function', 'sqrt'); add_line(logic_subsystem_actual_path, 'Sum_sq/1', 'F_mag_sqrt/1');
    
    % 4. 脱落状态判断与锁定 (Latch)
    f_break_val = fruit_params.F_break;
    add_block('simulink/Discontinuities/Relay', [logic_subsystem_actual_path, '/Detachment_Relay'], 'OnSwitchValue', num2str(f_break_val), 'OffSwitchValue', num2str(f_break_val * 0.95), 'OnOutputValue', '1', 'OffOutputValue', '0', 'Position', [180 250 220 280]);
    add_block('simulink/Discrete/Memory', [logic_subsystem_actual_path, '/Detached_Status_Latch'], 'InheritSampleTime', 'on', 'Position', [240 250 270 280]);
    add_line(logic_subsystem_actual_path, 'F_mag_sqrt/1', 'Detachment_Relay/1');
    add_line(logic_subsystem_actual_path, 'Detachment_Relay/1', 'Detached_Status_Latch/1');
    
    % 5. 将状态和力幅值连接到输出
    add_line(logic_subsystem_actual_path, 'Detached_Status_Latch/1', 'Detached_Status_out/1');
    add_line(logic_subsystem_actual_path, 'F_mag_sqrt/1', 'F_pedicel_magnitude_out/1');

    % 6. 【核心修正】使用开关直接切换最终输出的力
    % 创建一个Constant模块，其值为脱落后作用在Y方向的力，即重力
    add_block('simulink/Sources/Constant', [logic_subsystem_actual_path, '/GravityForce_Y'], ...
          'Value', num2str(-fruit_params.m * gravity_g_val, '%.4g')); % <-- m*g, 注意是负值
    zero_force_name = 'ZeroForce';
    add_block('simulink/Sources/Constant', [logic_subsystem_actual_path, '/', zero_force_name], 'Value', '0');
    
    % Y方向力的开关
    add_block('simulink/Signal Routing/Switch', [logic_subsystem_actual_path, '/Switch_Force_Y'], 'Criteria', 'u2 > Threshold', 'Threshold', '0.5', 'Position', [300 40 340 80]);
    add_line(logic_subsystem_actual_path, 'GravityForce_Y/1', 'Switch_Force_Y/1');  % u1: 如果脱落(u2>0.5)，通过u1 (GravityForce_Y)
    add_line(logic_subsystem_actual_path, 'Detached_Status_Latch/1', 'Switch_Force_Y/2'); % u2: 控制信号
    add_line(logic_subsystem_actual_path, 'Fy_unbroken/1', 'Switch_Force_Y/3');          % u3: 如果未脱落(u2<=0.5)，通过u3 (F_unbroken)
    
    % Z方向力的开关 (脱落后Z方向不受力)
    add_block('simulink/Signal Routing/Switch', [logic_subsystem_actual_path, '/Switch_Force_Z'], 'Criteria', 'u2 > Threshold', 'Threshold', '0.5', 'Position', [300 100 340 140]);
    add_line(logic_subsystem_actual_path, 'ZeroForce/1', 'Switch_Force_Z/1');         % u1: 如果脱落(u2>0.5)，通过u1 (ZeroForce)
    add_line(logic_subsystem_actual_path, 'Detached_Status_Latch/1', 'Switch_Force_Z/2'); % u2: 控制信号
    add_line(logic_subsystem_actual_path, 'Fz_unbroken/1', 'Switch_Force_Z/3');         % u3: 如果未脱落(u2<=0.5)，通过u3 (F_unbroken)

    % 7. 将切换后的最终作用力连接到输出
    add_line(logic_subsystem_actual_path, 'Switch_Force_Y/1', 'F_to_fruit_y_out/1');
    add_line(logic_subsystem_actual_path, 'Switch_Force_Z/1', 'F_to_fruit_z_out/1');
    
    % 8. 计算并连接反作用力
    add_block('simulink/Commonly Used Blocks/Gain', [logic_subsystem_actual_path, '/Negate_Fy'], 'Gain', '-1'); add_line(logic_subsystem_actual_path, 'Switch_Force_Y/1', 'Negate_Fy/1');
    add_line(logic_subsystem_actual_path, 'Negate_Fy/1', 'F_to_tip_y_out/1');
    add_block('simulink/Commonly Used Blocks/Gain', [logic_subsystem_actual_path, '/Negate_Fz'], 'Gain', '-1'); add_line(logic_subsystem_actual_path, 'Switch_Force_Z/1', 'Negate_Fz/1');
    add_line(logic_subsystem_actual_path, 'Negate_Fz/1', 'F_to_tip_z_out/1');

    % 自动整理子系统内部布局
    Simulink.BlockDiagram.arrangeSystem(logic_subsystem_actual_path);

    % --- 外部连接和记录 (这部分代码保持不变) ---
    logic_subsystem_rel_path = strrep(logic_subsystem_actual_path, [parent_model_path, '/'], '');
    tip_mass_rel_path = strrep(tip_mass_path, [parent_model_path, '/'], '');
    fruit_mass_rel_path = strrep(fruit_mass_path, [parent_model_path, '/'], '');
    for i_state = 1:4
        add_line(parent_model_path, [tip_mass_rel_path, '/', num2str(i_state)], [logic_subsystem_rel_path, '/', num2str(i_state)]);
        add_line(parent_model_path, [fruit_mass_rel_path, '/', num2str(i_state)], [logic_subsystem_rel_path, '/', num2str(i_state + 4)]);
    end
    fruit_inport_y_name_f = ['F_conn', num2str(fruit_F_conn_pair_idx), '_y_in']; fruit_port_num_y_f = get_param([fruit_mass_path, '/', fruit_inport_y_name_f], 'Port');
    add_line(parent_model_path, [logic_subsystem_rel_path, '/1'], [fruit_mass_rel_path, '/', fruit_port_num_y_f]);
    fruit_inport_z_name_f = ['F_conn', num2str(fruit_F_conn_pair_idx), '_z_in']; fruit_port_num_z_f = get_param([fruit_mass_path, '/', fruit_inport_z_name_f], 'Port');
    add_line(parent_model_path, [logic_subsystem_rel_path, '/2'], [fruit_mass_rel_path, '/', fruit_port_num_z_f]);
    tip_inport_y_name_t = ['F_conn', num2str(tip_F_react_conn_pair_idx), '_y_in']; tip_port_num_y_t = get_param([tip_mass_path, '/', tip_inport_y_name_t], 'Port');
    add_line(parent_model_path, [logic_subsystem_rel_path, '/3'], [tip_mass_rel_path, '/', tip_port_num_y_t]);
    tip_inport_z_name_t = ['F_conn', num2str(tip_F_react_conn_pair_idx), '_z_in']; tip_port_num_z_t = get_param([tip_mass_path, '/', tip_inport_z_name_t], 'Port');
    add_line(parent_model_path, [logic_subsystem_rel_path, '/4'], [tip_mass_rel_path, '/', tip_port_num_z_t]);
    ws_block_y_start_fruit = conn_block_base_pos_xy(2) + logic_ss_height + 10;
    ws_block_width = 150; ws_block_height = 20; ws_block_spacing = 5;
    detached_status_varname_ws = matlab.lang.makeValidName([fruit_unique_id, '_DetachedStatus']);
    ws_block_path_detached_status = [parent_model_path, '/', detached_status_varname_ws, '_ToWs'];
    add_block('simulink/Sinks/To Workspace', ws_block_path_detached_status, 'VariableName', detached_status_varname_ws, 'SaveFormat', 'Timeseries', 'SampleTime', '-1', 'Position', [conn_block_base_pos_xy(1) + logic_ss_width/2 - ws_block_width/2, ws_block_y_start_fruit, conn_block_base_pos_xy(1) + logic_ss_width/2 + ws_block_width/2, ws_block_y_start_fruit + ws_block_height]);
    add_line(parent_model_path, [logic_subsystem_rel_path, '/5'], [strrep(ws_block_path_detached_status, [parent_model_path, '/'], ''), '/1']);
    fped_mag_varname_ws = matlab.lang.makeValidName([fruit_unique_id, '_PedicelForceMag']);
    ws_block_path_fped_mag = [parent_model_path, '/', fped_mag_varname_ws, '_ToWs'];
    add_block('simulink/Sinks/To Workspace', ws_block_path_fped_mag, 'VariableName', fped_mag_varname_ws, 'SaveFormat', 'Timeseries', 'SampleTime', '-1', 'Position', [conn_block_base_pos_xy(1) + logic_ss_width/2 - ws_block_width/2, ws_block_y_start_fruit + ws_block_height + ws_block_spacing, conn_block_base_pos_xy(1) + logic_ss_width/2 + ws_block_width/2, ws_block_y_start_fruit + ws_block_height*2 + ws_block_spacing]);
    add_line(parent_model_path, [logic_subsystem_rel_path, '/6'], [strrep(ws_block_path_fped_mag, [parent_model_path, '/'], ''), '/1']);
    current_fruit_signal_entry = struct('unique_id', fruit_unique_id, 'y_disp_varname', matlab.lang.makeValidName([fruit_unique_id, '_y_displacement']), 'z_disp_varname', matlab.lang.makeValidName([fruit_unique_id, '_z_displacement']), 'y_accel_varname', matlab.lang.makeValidName([fruit_unique_id, '_y_acceleration']), 'z_accel_varname', matlab.lang.makeValidName([fruit_unique_id, '_z_acceleration']), 'detached_status_varname', detached_status_varname_ws, 'fped_mag_varname', fped_mag_varname_ws);
    existing_ids = cellfun(@(x) x.unique_id, fruit_signal_manager_global, 'UniformOutput', false);
    if ~any(strcmp(existing_ids, fruit_unique_id))
        fruit_signal_manager_global{end+1} = current_fruit_signal_entry;
    else
        warning('connect_fruit_with_detachment_2D: 果实唯一ID "%s" 的信号信息已存在于 fruit_signal_manager_global。未重复添加。', fruit_unique_id);
    end
end

%% build_branch_recursively (已修正)
% 功能: (核心递归函数) 构建一个分枝子系统 (主干、一级、二级或三级分枝)。
%       一个分枝通常由根(root)、中(mid)、尖(tip)三段质量块组成，它们之间通过弹簧阻尼器连接。
%       分枝的根段连接到其父级元件 (固定基座、或上层分枝的尖端)。
%       分枝的尖端可以进一步分出下一级子分枝，或者直接挂果。
%       此函数会递归调用自身来构建子分枝。
% 输入参数:
%   model_base_path (char/string): 顶层Simulink模型的名称。
%   parent_subsystem_full_path_providing_state (char/string):
%                                 提供状态输入 (y,vy,z,vz via a Bus) 的父级模块/子系统的完整路径。
%                                 对于主干，这是固定基座FixedBase的路径。
%                                 对于子分枝，这是其父分枝子系统的路径 (指向其TipState_ExternalOut总线)。
%   parent_reaction_force_target_full_path (char/string):
%                                 当前构建的分枝的根段所产生的反作用力应该施加到的父级元件的完整路径。
%                                 对于主干，这是固定基座FixedBase的路径。
%                                 对于子分枝，这是其父分枝的尖端质量块(Tip_Mass)的路径。
%   parent_reaction_force_port_spec_y (char/string or double):
%   parent_reaction_force_port_spec_z (char/string or double):
%                                 指定如何将反作用力连接回父级。
%                                 - 如果连接到FixedBase (branch_level=0): (此参数现在主要用于指引FixedBase上的Inport名)
%                                   在旧逻辑中，这曾被用作Trunk内Outport的名称。新逻辑中，Trunk会定义自己的Outport名。
%                                 - 如果连接到父分枝的Tip_Mass (branch_level>0): 它们是父Tip_Mass上F_connX_y/z_in的*索引号*。
%   branch_level (int):           当前正在构建的分枝的层级 (0=主干, 1=一级, 2=二级, 3=三级)。
%   branch_indices (double array):一个数组，包含从主干到当前分枝的各层级索引。
%                                 例如，[1, 2] 表示一级分枝P1下的二级分枝S2。主干时为空[]。
%   model_build_params_struct (struct): 包含完整模型参数的结构体 (config, parameters.trunk/primary/..., gravity_g)。
%   layout_params_struct (struct):      包含Simulink模块布局参数的结构体。
%   base_pos_xy (double array):   [x, y] 当前分枝子系统左上角的建议绘制起始坐标。
%   current_branch_subsystem_full_path_tentative_in (char/string):
%                                                                 当前分枝子系统的建议完整路径。函数内部会用此创建。
% 输出参数: 无
% 全局变量:
%   excitation_targets_global: (修改)
%   current_y_level_global:    (修改/使用)
%   fruit_signal_manager_global: (修改)
function build_branch_recursively(model_base_path, ...
                                  parent_subsystem_full_path_providing_state, ...
                                  parent_reaction_force_target_full_path, ...
                                  parent_reaction_force_port_spec_y, parent_reaction_force_port_spec_z, ...
                                  branch_level, branch_indices, ...
                                  model_build_params_struct, layout_params_struct, ...
                                  base_pos_xy, current_branch_subsystem_full_path_tentative_in)
    % 声明使用全局变量
    global excitation_targets_global;
    global current_y_level_global; 
    global fruit_signal_manager_global;

    % *** 新增: 获取初始条件映射表 ***
    ic_map = containers.Map(); % 默认空表
    if isfield(model_build_params_struct, 'initial_conditions')
        ic_map = model_build_params_struct.initial_conditions;
    end
    
     % *** 核心修正：在此处定义 get_ic_strings 作为嵌套函数 ***
    % 这样它就可以访问其父函数 build_branch_recursively 的工作区，特别是 ic_map 变量。
    function [y_ic_str, z_ic_str] = get_ic_strings(mass_id)
        if isKey(ic_map, mass_id)
            ic_struct = ic_map(mass_id);
            % 使用 %.8g 格式化，确保数值精度且避免不必要的科学记数法
            y_ic_str = num2str(ic_struct.y_ic, '%.8g');
            z_ic_str = num2str(ic_struct.z_ic, '%.8g');
        else
            y_ic_str = '0';
            z_ic_str = '0';
            warning('build_branch_recursively: 在 initial_conditions_map 中未找到质量点 "%s" 的初始条件，将使用默认值 0。', mass_id);
        end
    end

    % --- 初始化和参数提取 ---
    if isempty(current_y_level_global) && branch_level == 0
        current_y_level_global = base_pos_xy(2); 
    end

    branch_segment_names = {'root', 'mid', 'tip'}; 
    current_branch_params = struct(); 
    valid_params_found = true;        
    path_id_str_for_names = '';       

    if branch_level == 0
        path_id_str_for_names = 'Trunk';
        current_branch_params = model_build_params_struct.parameters.trunk;
    else
        % 从 branch_indices 重新构建路径前缀
        prefix_parts = {};
        if length(branch_indices) >= 1
            prefix_parts{end+1} = ['P' num2str(branch_indices(1))];
        end
        if length(branch_indices) >= 2
            prefix_parts{end+1} = ['S' num2str(branch_indices(2))];
        end
        if length(branch_indices) == 3
            prefix_parts{end+1} = ['T' num2str(branch_indices(3))];
        end
        path_id_str_for_names = strjoin(prefix_parts, '_');
        
        % 提取参数 (这部分逻辑可以简化)
        temp_ptr = model_build_params_struct.parameters.primary{branch_indices(1)};
        if length(branch_indices) >= 2
            temp_ptr = temp_ptr.secondary_branches{branch_indices(2)};
        end
        if length(branch_indices) == 3
            temp_ptr = temp_ptr.tertiary_branches{branch_indices(3)};
        end
        current_branch_params = temp_ptr;
    end 

    if ~valid_params_found || isempty(fieldnames(current_branch_params)) || ~isfield(current_branch_params, 'root')
        fprintf('警告 build_branch_recursively: 跳过分枝构建 - 层级: %d, 索引: %s (ID: %s) - 原因: 参数缺失或无效。\n', ...
                branch_level, mat2str(branch_indices), path_id_str_for_names);
        return; 
    end
    fprintf('  build_branch_recursively: 开始构建分枝 "%s" (层级 %d, 索引 %s)...\n', ...
            path_id_str_for_names, branch_level, mat2str(branch_indices));

    % --- 创建当前分枝的子系统 ---
    actual_subsystem_base_pos_y = base_pos_xy(2); 
    current_branch_subsystem_height = layout_params_struct.subsystem_height; 

    if branch_level == 1 
        actual_subsystem_base_pos_y = current_y_level_global;
        current_y_level_global = current_y_level_global + layout_params_struct.subsystem_height_spacing; 
    elseif branch_level > 1 
        current_branch_subsystem_height = layout_params_struct.subsystem_height * 0.8;
    end

    current_branch_subsystem_local_name = matlab.lang.makeValidName([path_id_str_for_names, '_Branch']);
    if isempty(current_branch_subsystem_full_path_tentative_in) 
        current_branch_subsystem_path_to_create = [model_base_path, '/', current_branch_subsystem_local_name];
    else 
        current_branch_subsystem_path_to_create = current_branch_subsystem_full_path_tentative_in;
    end
    
    add_block('simulink/Ports & Subsystems/Subsystem', current_branch_subsystem_path_to_create, ...
              'Position', [base_pos_xy(1), actual_subsystem_base_pos_y, ...
                           base_pos_xy(1) + layout_params_struct.subsystem_width, ...
                           actual_subsystem_base_pos_y + current_branch_subsystem_height], ...
              'MakeNameUnique', 'on'); % 移除了 'BackgroundColor'
    current_branch_subsystem_actual_full_path = get_param(current_branch_subsystem_path_to_create, 'Object').getFullName();
    clean_subsystem_internals(current_branch_subsystem_actual_full_path);

    % --- 创建父级状态输入端口 (Parent_State_In) 和 Bus Selector ---
    parent_state_inport_name_inside_branch = 'Parent_State_In'; 
    add_block('simulink/Sources/In1', [current_branch_subsystem_actual_full_path, '/', parent_state_inport_name_inside_branch], ...
              'Position', [20, 50, 20 + layout_params_struct.inport_width, 50 + layout_params_struct.inport_height], ...
              'Port', '1'); 

    parent_provider_rel_path_for_line = strrep(parent_subsystem_full_path_providing_state, [model_base_path, '/'], '');
    current_branch_subsys_rel_path_for_line = strrep(current_branch_subsystem_actual_full_path, [model_base_path, '/'], '');
    
    source_block_port_str_for_parent_state = [parent_provider_rel_path_for_line, '/1'];
    destination_inport_str_on_current_branch = [current_branch_subsys_rel_path_for_line, '/1']; 
    add_line(model_base_path, source_block_port_str_for_parent_state, destination_inport_str_on_current_branch, 'autorouting', 'on');

    bus_selector_for_parent_state_name_inside_branch_base = 'Selector_From_ParentStateBus';
    add_block('simulink/Signal Routing/Bus Selector', ...
              [current_branch_subsystem_actual_full_path, '/', bus_selector_for_parent_state_name_inside_branch_base], ...
              'OutputSignals', 'signal1,signal2,signal3,signal4', ... 
              'Position', [70, 40, 75, 100], 'MakeNameUnique', 'on');
    bus_selector_for_parent_state_actual_name_inside_branch = get_param([current_branch_subsystem_actual_full_path, '/', bus_selector_for_parent_state_name_inside_branch_base], 'Name');
    add_line(current_branch_subsystem_actual_full_path, ...
             [parent_state_inport_name_inside_branch, '/1'], ...
             [bus_selector_for_parent_state_actual_name_inside_branch, '/1']);
    
    upstream_state_provider_for_root_segment_conn = [current_branch_subsystem_actual_full_path, '/', bus_selector_for_parent_state_actual_name_inside_branch];

    % --- 创建当前分支的反作用力输出端口 ---
    outport_for_reaction_to_parent_Y_name = ''; 
    outport_for_reaction_to_parent_Z_name = ''; 
    % TipState_ExternalOut 将会是第1个Outport，所以其他Outport从Port '2' 开始。
    % current_branch_next_available_outport_num 在此上下文中指除了TipState_ExternalOut之外的下一个可用端口号。
    next_general_outport_number_in_branch = 1; % Start with 1 for TipState_ExternalOut, then increment

    react_outport_x_pos_inside_branch = layout_params_struct.subsystem_width - layout_params_struct.general_outport_width - 50;
    react_outport_y_pos_start_inside_branch = 150; 

    if branch_level == 0 
        % 主干的反作用力Outport名称定义。这些将在connect_elements_2D中被创建。
        outport_for_reaction_to_parent_Y_name = 'Trunk_ReactForce_Y_Out'; 
        outport_for_reaction_to_parent_Z_name = 'Trunk_ReactForce_Z_Out'; 
        % TipState_ExternalOut将使用端口 '1'. connect_elements_2D(is_upstream_fixed=true) 
        % 会负责创建名为Trunk_ReactForce_Y/Z_Out的Outport，并自动获取后续可用端口号。
        % 所以此处无需预先增加 next_general_outport_number_in_branch
    elseif branch_level > 0 
        outport_for_reaction_to_parent_Y_name = matlab.lang.makeValidName([path_id_str_for_names, '_ReactToParent_Y_Out']);
        outport_for_reaction_to_parent_Z_name = matlab.lang.makeValidName([path_id_str_for_names, '_ReactToParent_Z_Out']);
        
        add_block('simulink/Sinks/Out1', [current_branch_subsystem_actual_full_path, '/', outport_for_reaction_to_parent_Y_name], ...
                  'Port', num2str(next_general_outport_number_in_branch + 1), ... % Port '2'
                  'Position', [react_outport_x_pos_inside_branch, react_outport_y_pos_start_inside_branch, ...
                               react_outport_x_pos_inside_branch + layout_params_struct.general_outport_width, ...
                               react_outport_y_pos_start_inside_branch + layout_params_struct.general_outport_height]);
        add_block('simulink/Sinks/Out1', [current_branch_subsystem_actual_full_path, '/', outport_for_reaction_to_parent_Z_name], ...
                  'Port', num2str(next_general_outport_number_in_branch + 2), ... % Port '3'
                  'Position', [react_outport_x_pos_inside_branch, react_outport_y_pos_start_inside_branch + layout_params_struct.general_outport_height + 10, ...
                               react_outport_x_pos_inside_branch + layout_params_struct.general_outport_width, ...
                               react_outport_y_pos_start_inside_branch + 2*layout_params_struct.general_outport_height + 10]);
        % next_general_outport_number_in_branch += 2; % 如果后面还有其他通用Outport
    end
    
    % --- 构建分枝的三个段: Root, Mid, Tip ---
    segment_mass_paths = cell(1, 3); 
    segment_layout_x_start_inside_branch = 100; 
    segment_layout_y_pos_inside_branch = 200;   

    % --- 1. 根段 (Root Segment) ---
    root_segment_params = current_branch_params.root;
    root_mass_local_name = matlab.lang.makeValidName([path_id_str_for_names, '_', branch_segment_names{1}, '_Mass']);
    [y_ic_root, z_ic_root] = get_ic_strings(root_mass_local_name); % *** 获取初始条件 ***
    % *** 新增：健壮性检查 ***
    if isempty(y_ic_root) || ~ischar(y_ic_root), y_ic_root = '0'; end
    if isempty(z_ic_root) || ~ischar(z_ic_root), z_ic_root = '0'; end
    
    segment_mass_paths{1} = create_mass_subsystem_2D(...
        current_branch_subsystem_actual_full_path, ... 
        root_mass_local_name, ...
        root_segment_params.m, ...
        model_build_params_struct.gravity_g, ...
        2, ... 
        true, ... 
        [segment_layout_x_start_inside_branch, segment_layout_y_pos_inside_branch, ...
        segment_layout_x_start_inside_branch + layout_params_struct.segment_width, ...
        segment_layout_y_pos_inside_branch + layout_params_struct.segment_height], ...
        '0', '0' ... % <-- 核心修改点：强制初始条件为0
    );

    if branch_level < 3
        excitation_targets_global{end+1} = struct('path', segment_mass_paths{1}, 'name', root_mass_local_name); 
    end

    conn_to_parent_base_pos_xy = [segment_layout_x_start_inside_branch - layout_params_struct.segment_spacing_x / 1.5, ...
                                  segment_layout_y_pos_inside_branch + layout_params_struct.segment_height / 2];
    if branch_level == 0 
        connect_elements_2D(...
            current_branch_subsystem_actual_full_path, ... 
            upstream_state_provider_for_root_segment_conn, 'BusSelector', ... 
            segment_mass_paths{1}, strrep(root_mass_local_name, '_Mass', ''), ... 
            root_segment_params.k_y_conn_to_base, root_segment_params.c_y_conn_to_base, ... 
            root_segment_params.k_z_conn_to_base, root_segment_params.c_z_conn_to_base, ...
            parent_reaction_force_target_full_path, ... 
            outport_for_reaction_to_parent_Y_name, ...  
            outport_for_reaction_to_parent_Z_name, ...  
            1, ... 
            conn_to_parent_base_pos_xy, ...
            true, ...  
            false, ... 
            layout_params_struct);
    else 
        connect_elements_2D(...
            current_branch_subsystem_actual_full_path, ... 
            upstream_state_provider_for_root_segment_conn, 'BusSelector', ...
            segment_mass_paths{1}, strrep(root_mass_local_name, '_Mass', ''), ...
            root_segment_params.k_y_conn, root_segment_params.c_y_conn, ... 
            root_segment_params.k_z_conn, root_segment_params.c_z_conn, ...
            current_branch_subsystem_actual_full_path, ... 
            outport_for_reaction_to_parent_Y_name, ...  
            outport_for_reaction_to_parent_Z_name, ...  
            1, ... 
            conn_to_parent_base_pos_xy, ...
            false, ... 
            true,  ...  
            layout_params_struct);
        
        parent_branch_subsystem_of_tip_path = fileparts(parent_reaction_force_target_full_path); 
        parent_tip_mass_local_name_in_its_branch = strrep(parent_reaction_force_target_full_path, [parent_branch_subsystem_of_tip_path, '/'], '');

        child_branch_rel_path_to_model_base = strrep(current_branch_subsystem_actual_full_path, [model_base_path, '/'], '');
        child_react_outport_Y_port_num_str = get_param([current_branch_subsystem_actual_full_path, '/', outport_for_reaction_to_parent_Y_name], 'Port');
        child_react_outport_Z_port_num_str = get_param([current_branch_subsystem_actual_full_path, '/', outport_for_reaction_to_parent_Z_name], 'Port');
        
        parent_branch_rel_path_to_model_base = strrep(parent_branch_subsystem_of_tip_path, [model_base_path, '/'], '');
        
        parent_branch_inport_for_child_react_Y_name = matlab.lang.makeValidName(['From_', path_id_str_for_names, '_React_Y_In']);
        parent_branch_inport_for_child_react_Z_name = matlab.lang.makeValidName(['From_', path_id_str_for_names, '_React_Z_In']);
        
        num_existing_inports_on_parent_branch = length(find_system(parent_branch_subsystem_of_tip_path, 'SearchDepth', 1, 'BlockType', 'Inport'));
        parent_inport_layout_x = 20; 
        parent_inport_layout_y_start = 150 + num_existing_inports_on_parent_branch * (layout_params_struct.general_inport_height + 10); 
        
        add_block('simulink/Sources/In1', [parent_branch_subsystem_of_tip_path, '/', parent_branch_inport_for_child_react_Y_name], ...
                  'Port', num2str(num_existing_inports_on_parent_branch + 1), ...
                  'Position', [parent_inport_layout_x, parent_inport_layout_y_start, ...
                               parent_inport_layout_x + layout_params_struct.general_inport_width, ...
                               parent_inport_layout_y_start + layout_params_struct.general_inport_height]);
        parent_branch_inport_for_child_react_Y_port_num_str = get_param([parent_branch_subsystem_of_tip_path, '/', parent_branch_inport_for_child_react_Y_name], 'Port');

        add_block('simulink/Sources/In1', [parent_branch_subsystem_of_tip_path, '/', parent_branch_inport_for_child_react_Z_name], ...
                  'Port', num2str(num_existing_inports_on_parent_branch + 2), ...
                  'Position', [parent_inport_layout_x, parent_inport_layout_y_start + layout_params_struct.general_inport_height + 10, ...
                               parent_inport_layout_x + layout_params_struct.general_inport_width, ...
                               parent_inport_layout_y_start + 2*layout_params_struct.general_inport_height + 10]);
        parent_branch_inport_for_child_react_Z_port_num_str = get_param([parent_branch_subsystem_of_tip_path, '/', parent_branch_inport_for_child_react_Z_name], 'Port');
        
        add_line(model_base_path, ...
                 [child_branch_rel_path_to_model_base, '/', child_react_outport_Y_port_num_str], ...
                 [parent_branch_rel_path_to_model_base, '/', parent_branch_inport_for_child_react_Y_port_num_str], 'autorouting', 'on');
        add_line(model_base_path, ...
                 [child_branch_rel_path_to_model_base, '/', child_react_outport_Z_port_num_str], ...
                 [parent_branch_rel_path_to_model_base, '/', parent_branch_inport_for_child_react_Z_port_num_str], 'autorouting', 'on');

        parent_tip_mass_fconn_idx_y = parent_reaction_force_port_spec_y; 
        parent_tip_mass_fconn_idx_z = parent_reaction_force_port_spec_z;
        parent_tip_mass_internal_inport_Y_name = ['F_conn', num2str(parent_tip_mass_fconn_idx_y), '_y_in'];
        parent_tip_mass_internal_inport_Z_name = ['F_conn', num2str(parent_tip_mass_fconn_idx_z), '_z_in'];
        parent_tip_mass_internal_inport_Y_port_num_str = get_param([parent_reaction_force_target_full_path, '/', parent_tip_mass_internal_inport_Y_name], 'Port');
        parent_tip_mass_internal_inport_Z_port_num_str = get_param([parent_reaction_force_target_full_path, '/', parent_tip_mass_internal_inport_Z_name], 'Port');
        
        add_line(parent_branch_subsystem_of_tip_path, ...
                 [parent_branch_inport_for_child_react_Y_name, '/1'], ...
                 [parent_tip_mass_local_name_in_its_branch, '/', parent_tip_mass_internal_inport_Y_port_num_str], 'autorouting', 'on');
        add_line(parent_branch_subsystem_of_tip_path, ...
                 [parent_branch_inport_for_child_react_Z_name, '/1'], ...
                 [parent_tip_mass_local_name_in_its_branch, '/', parent_tip_mass_internal_inport_Z_port_num_str], 'autorouting', 'on');
    end

    % --- 3.2 中段 (Mid Segment) ---
    mid_segment_layout_x = segment_layout_x_start_inside_branch + layout_params_struct.segment_width + layout_params_struct.segment_spacing_x;
    mid_segment_params = current_branch_params.mid;
    mid_mass_local_name = matlab.lang.makeValidName([path_id_str_for_names, '_', branch_segment_names{2}, '_Mass']);
    [y_ic_mid, z_ic_mid] = get_ic_strings(mid_mass_local_name);
    if isempty(y_ic_mid) || ~ischar(y_ic_mid), y_ic_mid = '0'; end
    if isempty(z_ic_mid) || ~ischar(z_ic_mid), z_ic_mid = '0'; end
    
    % ========== 统一果实检测（Mid和Tip一起检测）==========
    % 检测 Mid 位置是否有果实
    has_fruit_at_mid = false;
    if isfield(current_branch_params, 'fruit_at_mid')
        if isstruct(current_branch_params.fruit_at_mid) && ~isempty(fieldnames(current_branch_params.fruit_at_mid))
            has_fruit_at_mid = true;
        end
    end
    
    % 检测 Tip 位置是否有果实
    has_fruit_at_tip = false;
    if isfield(current_branch_params, 'fruit_at_tip')
        if isstruct(current_branch_params.fruit_at_tip) && ~isempty(fieldnames(current_branch_params.fruit_at_tip))
            has_fruit_at_tip = true;
        end
    end
    
    % 向后兼容：旧字段名 'fruit' 映射到 fruit_at_tip
    if ~has_fruit_at_tip && isfield(current_branch_params, 'fruit')
        if isstruct(current_branch_params.fruit) && ~isempty(fieldnames(current_branch_params.fruit))
            has_fruit_at_tip = true;
            current_branch_params.fruit_at_tip = current_branch_params.fruit;
        end
    end
    
    % 计算 Mid 段的连接端口数量：基础2个（来自root和去往tip）+ 果实1个（如果有）
    num_conn_pairs_for_mid = 2;
    if has_fruit_at_mid
        num_conn_pairs_for_mid = num_conn_pairs_for_mid + 1;
    end
    
    segment_mass_paths{2} = create_mass_subsystem_2D(...
        current_branch_subsystem_actual_full_path, ...
        mid_mass_local_name, ...
        mid_segment_params.m, ...
        model_build_params_struct.gravity_g, ...
        num_conn_pairs_for_mid, ...
        true, ...
        [mid_segment_layout_x, segment_layout_y_pos_inside_branch, ...
        mid_segment_layout_x + layout_params_struct.segment_width, ...
        segment_layout_y_pos_inside_branch + layout_params_struct.segment_height], ...
        '0', '0'...
    );

    if branch_level < 3
        excitation_targets_global{end+1} = struct('path', segment_mass_paths{2}, 'name', mid_mass_local_name);
    end

    conn_mid_to_root_base_pos_xy = [mid_segment_layout_x - layout_params_struct.segment_spacing_x / 1.5, ...
                                    segment_layout_y_pos_inside_branch + layout_params_struct.segment_height / 2];
    connect_elements_2D(...
        current_branch_subsystem_actual_full_path, ...
        segment_mass_paths{1}, 'Mass', ...
        segment_mass_paths{2}, strrep(mid_mass_local_name, '_Mass', ''), ...
        root_segment_params.k_y_conn, root_segment_params.c_y_conn, ...
        root_segment_params.k_z_conn, root_segment_params.c_z_conn, ...
        segment_mass_paths{1}, 2, 2, ...
        1, ...
        conn_mid_to_root_base_pos_xy, ...
        false, false, ...
        layout_params_struct);

    % --- 3.2.1 Mid 位置果实连接（新增）---
    mid_next_available_fconn_idx = 2 + 1;
    
    if has_fruit_at_mid
        fruit_parameters_for_this_mid = current_branch_params.fruit_at_mid;
        fruit_unique_id_for_this_mid = matlab.lang.makeValidName([path_id_str_for_names, '_mid_Fruit']);
        fruit_mass_local_name_on_mid = [fruit_unique_id_for_this_mid, '_Mass'];
        [y_ic_fruit_mid, z_ic_fruit_mid] = get_ic_strings(fruit_mass_local_name_on_mid);
        if isempty(y_ic_fruit_mid) || ~ischar(y_ic_fruit_mid), y_ic_fruit_mid = '0'; end
        if isempty(z_ic_fruit_mid) || ~ischar(z_ic_fruit_mid), z_ic_fruit_mid = '0'; end
        
        fruit_mid_layout_x_pos = mid_segment_layout_x + layout_params_struct.segment_width + layout_params_struct.fruit_offset_x * 0.6;
        fruit_mid_layout_y_pos = segment_layout_y_pos_inside_branch - layout_params_struct.segment_height * 0.3;
        
        fruit_mass_path_on_mid = create_mass_subsystem_2D(...
            current_branch_subsystem_actual_full_path, ...
            fruit_mass_local_name_on_mid, ...
            fruit_parameters_for_this_mid.m, ...
            model_build_params_struct.gravity_g, ...
            1, ...
            true, ...
            [fruit_mid_layout_x_pos, fruit_mid_layout_y_pos, ...
            fruit_mid_layout_x_pos + layout_params_struct.segment_width, ...
            fruit_mid_layout_y_pos + layout_params_struct.segment_height], ...
            '0', '0' ...
        );
        
        conn_fruit_mid_to_mid_base_pos_xy = [fruit_mid_layout_x_pos - layout_params_struct.fruit_offset_x * 0.3, ...
                                             fruit_mid_layout_y_pos + layout_params_struct.segment_height / 2];
        connect_fruit_with_detachment_2D(...
            current_branch_subsystem_actual_full_path, ...
            segment_mass_paths{2}, ...
            fruit_mass_path_on_mid, ...
            fruit_unique_id_for_this_mid, ...
            fruit_parameters_for_this_mid, ...
            model_build_params_struct.gravity_g, ...
            mid_next_available_fconn_idx, ...
            1, ...
            conn_fruit_mid_to_mid_base_pos_xy ...
        );
        
        idx_fruit_mid_signal_in_manager = -1;
        for i_fsm_mid = 1:length(fruit_signal_manager_global)
            if strcmp(fruit_signal_manager_global{i_fsm_mid}.unique_id, fruit_unique_id_for_this_mid)
                idx_fruit_mid_signal_in_manager = i_fsm_mid;
                break;
            end
        end
        
        parent_mid_id = strrep(fruit_unique_id_for_this_mid, '_Fruit', '');
        parent_mid_y_disp_varname  = matlab.lang.makeValidName([parent_mid_id, '_y_displacement']);
        parent_mid_z_disp_varname  = matlab.lang.makeValidName([parent_mid_id, '_z_displacement']);
        parent_mid_y_accel_varname = matlab.lang.makeValidName([parent_mid_id, '_y_acceleration']);
        parent_mid_z_accel_varname = matlab.lang.makeValidName([parent_mid_id, '_z_acceleration']);
        
        if idx_fruit_mid_signal_in_manager > 0
            y_disp_varname_fruit_mid  = fruit_signal_manager_global{idx_fruit_mid_signal_in_manager}.y_disp_varname;
            z_disp_varname_fruit_mid  = fruit_signal_manager_global{idx_fruit_mid_signal_in_manager}.z_disp_varname;
            y_accel_varname_fruit_mid = fruit_signal_manager_global{idx_fruit_mid_signal_in_manager}.y_accel_varname;
            z_accel_varname_fruit_mid = fruit_signal_manager_global{idx_fruit_mid_signal_in_manager}.z_accel_varname;

            fruit_mid_mass_rel_path_in_branch = strrep(fruit_mass_path_on_mid, [current_branch_subsystem_actual_full_path, '/'], '');
            mid_mass_rel_path_in_branch = strrep(segment_mass_paths{2}, [current_branch_subsystem_actual_full_path, '/'], '');
            
            ws_block_layout_x_mid = fruit_mid_layout_x_pos + layout_params_struct.segment_width + 20;
            ws_block_layout_y_offset_mid = 50;
            ws_v_spacing_mid = 30; ws_height_mid = 20; ws_width_mid = 200;

            add_block('simulink/Sinks/To Workspace', [current_branch_subsystem_actual_full_path, '/', y_disp_varname_fruit_mid, '_ToWs'], ...
                      'VariableName', y_disp_varname_fruit_mid, 'SaveFormat', 'Timeseries', 'SampleTime', '-1', ...
                      'Position', [ws_block_layout_x_mid, fruit_mid_layout_y_pos + ws_block_layout_y_offset_mid, ...
                                   ws_block_layout_x_mid + ws_width_mid, fruit_mid_layout_y_pos + ws_block_layout_y_offset_mid + ws_height_mid]);
            add_line(current_branch_subsystem_actual_full_path, [fruit_mid_mass_rel_path_in_branch, '/1'], [y_disp_varname_fruit_mid, '_ToWs/1']);
            ws_block_layout_y_offset_mid = ws_block_layout_y_offset_mid + ws_v_spacing_mid;

            add_block('simulink/Sinks/To Workspace', [current_branch_subsystem_actual_full_path, '/', z_disp_varname_fruit_mid, '_ToWs'], ...
                      'VariableName', z_disp_varname_fruit_mid, 'SaveFormat', 'Timeseries', 'SampleTime', '-1', ...
                      'Position', [ws_block_layout_x_mid, fruit_mid_layout_y_pos + ws_block_layout_y_offset_mid, ...
                                   ws_block_layout_x_mid + ws_width_mid, fruit_mid_layout_y_pos + ws_block_layout_y_offset_mid + ws_height_mid]);
            add_line(current_branch_subsystem_actual_full_path, [fruit_mid_mass_rel_path_in_branch, '/3'], [z_disp_varname_fruit_mid, '_ToWs/1']);
            ws_block_layout_y_offset_mid = ws_block_layout_y_offset_mid + ws_v_spacing_mid;
            
            add_block('simulink/Sinks/To Workspace', [current_branch_subsystem_actual_full_path, '/', y_accel_varname_fruit_mid, '_ToWs'], ...
                      'VariableName', y_accel_varname_fruit_mid, 'SaveFormat', 'Timeseries', 'SampleTime', '-1', ...
                      'Position', [ws_block_layout_x_mid, fruit_mid_layout_y_pos + ws_block_layout_y_offset_mid, ...
                                   ws_block_layout_x_mid + ws_width_mid, fruit_mid_layout_y_pos + ws_block_layout_y_offset_mid + ws_height_mid]);
            add_line(current_branch_subsystem_actual_full_path, [fruit_mid_mass_rel_path_in_branch, '/5'], [y_accel_varname_fruit_mid, '_ToWs/1']);
            ws_block_layout_y_offset_mid = ws_block_layout_y_offset_mid + ws_v_spacing_mid;

            add_block('simulink/Sinks/To Workspace', [current_branch_subsystem_actual_full_path, '/', z_accel_varname_fruit_mid, '_ToWs'], ...
                      'VariableName', z_accel_varname_fruit_mid, 'SaveFormat', 'Timeseries', 'SampleTime', '-1', ...
                      'Position', [ws_block_layout_x_mid, fruit_mid_layout_y_pos + ws_block_layout_y_offset_mid, ...
                                   ws_block_layout_x_mid + ws_width_mid, fruit_mid_layout_y_pos + ws_block_layout_y_offset_mid + ws_height_mid]);
            add_line(current_branch_subsystem_actual_full_path, [fruit_mid_mass_rel_path_in_branch, '/6'], [z_accel_varname_fruit_mid, '_ToWs/1']);
            
            ws_block_layout_y_offset_mid = ws_block_layout_y_offset_mid + 20;
            add_block('simulink/Sinks/To Workspace', [current_branch_subsystem_actual_full_path, '/', parent_mid_y_disp_varname, '_ToWs'], ...
                'VariableName', parent_mid_y_disp_varname, 'SaveFormat', 'Timeseries', ...
                'Position', [ws_block_layout_x_mid, ws_block_layout_y_offset_mid, ws_block_layout_x_mid+ws_width_mid, ws_block_layout_y_offset_mid+ws_height_mid]);
            add_line(current_branch_subsystem_actual_full_path, [mid_mass_rel_path_in_branch, '/1'], [parent_mid_y_disp_varname, '_ToWs/1']);
            ws_block_layout_y_offset_mid = ws_block_layout_y_offset_mid + ws_v_spacing_mid;
            
            add_block('simulink/Sinks/To Workspace', [current_branch_subsystem_actual_full_path, '/', parent_mid_z_disp_varname, '_ToWs'], ...
                'VariableName', parent_mid_z_disp_varname, 'SaveFormat', 'Timeseries', ...
                'Position', [ws_block_layout_x_mid, ws_block_layout_y_offset_mid, ws_block_layout_x_mid+ws_width_mid, ws_block_layout_y_offset_mid+ws_height_mid]);
            add_line(current_branch_subsystem_actual_full_path, [mid_mass_rel_path_in_branch, '/3'], [parent_mid_z_disp_varname, '_ToWs/1']);
            ws_block_layout_y_offset_mid = ws_block_layout_y_offset_mid + ws_v_spacing_mid;
            
            add_block('simulink/Sinks/To Workspace', [current_branch_subsystem_actual_full_path, '/', parent_mid_y_accel_varname, '_ToWs'], ...
                'VariableName', parent_mid_y_accel_varname, 'SaveFormat', 'Timeseries', ...
                'Position', [ws_block_layout_x_mid, ws_block_layout_y_offset_mid, ws_block_layout_x_mid+ws_width_mid, ws_block_layout_y_offset_mid+ws_height_mid]);
            add_line(current_branch_subsystem_actual_full_path, [mid_mass_rel_path_in_branch, '/5'], [parent_mid_y_accel_varname, '_ToWs/1']);
            ws_block_layout_y_offset_mid = ws_block_layout_y_offset_mid + ws_v_spacing_mid;
            
            add_block('simulink/Sinks/To Workspace', [current_branch_subsystem_actual_full_path, '/', parent_mid_z_accel_varname, '_ToWs'], ...
                'VariableName', parent_mid_z_accel_varname, 'SaveFormat', 'Timeseries', ...
                'Position', [ws_block_layout_x_mid, ws_block_layout_y_offset_mid, ws_block_layout_x_mid+ws_width_mid, ws_block_layout_y_offset_mid+ws_height_mid]);
            add_line(current_branch_subsystem_actual_full_path, [mid_mass_rel_path_in_branch, '/6'], [parent_mid_z_accel_varname, '_ToWs/1']);
            
            fruit_signal_manager_global{idx_fruit_mid_signal_in_manager}.parent_mid_y_disp_varname  = parent_mid_y_disp_varname;
            fruit_signal_manager_global{idx_fruit_mid_signal_in_manager}.parent_mid_z_disp_varname  = parent_mid_z_disp_varname;
            fruit_signal_manager_global{idx_fruit_mid_signal_in_manager}.parent_mid_y_accel_varname = parent_mid_y_accel_varname;
            fruit_signal_manager_global{idx_fruit_mid_signal_in_manager}.parent_mid_z_accel_varname = parent_mid_z_accel_varname;
        else
            warning('build_branch_recursively: 未能在 fruit_signal_manager_global 中找到Mid果实ID "%s" 的条目。', fruit_unique_id_for_this_mid);
        end
    end

    % --- 3.3 尖端 (Tip Segment) ---
    tip_segment_layout_x = mid_segment_layout_x + layout_params_struct.segment_width + layout_params_struct.segment_spacing_x;
    tip_segment_params = current_branch_params.tip;
    tip_mass_local_name = matlab.lang.makeValidName([path_id_str_for_names, '_', branch_segment_names{3}, '_Mass']);
    [y_ic_tip, z_ic_tip] = get_ic_strings(tip_mass_local_name); % *** 获取初始条件 ***
    % *** 新增：健壮性检查 ***
    if isempty(y_ic_tip) || ~ischar(y_ic_tip), y_ic_tip = '0'; end
    if isempty(z_ic_tip) || ~ischar(z_ic_tip), z_ic_tip = '0'; end
    num_sub_branches_from_this_tip = 0;
    if branch_level == 0 
        num_sub_branches_from_this_tip = model_build_params_struct.config.num_primary_branches;
    elseif branch_level == 1 
        if branch_indices(1) <= length(model_build_params_struct.config.secondary_branches_count)
            num_sub_branches_from_this_tip = model_build_params_struct.config.secondary_branches_count(branch_indices(1));
        end
    elseif branch_level == 2 
        if branch_indices(1) <= length(model_build_params_struct.config.tertiary_branches_count) && ...
           (iscell(model_build_params_struct.config.tertiary_branches_count{branch_indices(1)}) || ...
            isvector(model_build_params_struct.config.tertiary_branches_count{branch_indices(1)})) && ...
           branch_indices(2) <= length(model_build_params_struct.config.tertiary_branches_count{branch_indices(1)})
            num_sub_branches_from_this_tip = model_build_params_struct.config.tertiary_branches_count{branch_indices(1)}(branch_indices(2));
        end
    end 
    
    num_total_conn_pairs_for_tip = 1 + num_sub_branches_from_this_tip; 

    if has_fruit_at_tip
        num_total_conn_pairs_for_tip = num_total_conn_pairs_for_tip + 1; 
    end
    
    segment_mass_paths{3} = create_mass_subsystem_2D(...
        current_branch_subsystem_actual_full_path, ...
        tip_mass_local_name, ...
        tip_segment_params.m, ...
        model_build_params_struct.gravity_g, ...
        num_total_conn_pairs_for_tip, ... 
        true, ...
        [tip_segment_layout_x, segment_layout_y_pos_inside_branch, ...
        tip_segment_layout_x + layout_params_struct.segment_width, ...
        segment_layout_y_pos_inside_branch + layout_params_struct.segment_height], ...
        '0', '0' ... % <-- 核心修改点：强制初始条件为0
    );
    
    if branch_level < 3
        excitation_targets_global{end+1} = struct('path', segment_mass_paths{3}, 'name', tip_mass_local_name);
    end

    conn_tip_to_mid_base_pos_xy = [tip_segment_layout_x - layout_params_struct.segment_spacing_x / 1.5, ...
                                   segment_layout_y_pos_inside_branch + layout_params_struct.segment_height / 2];
    connect_elements_2D(...
        current_branch_subsystem_actual_full_path, ...
        segment_mass_paths{2}, 'Mass', ... 
        segment_mass_paths{3}, strrep(tip_mass_local_name, '_Mass', ''), ... 
        mid_segment_params.k_y_conn, mid_segment_params.c_y_conn, ...
        mid_segment_params.k_z_conn, mid_segment_params.c_z_conn, ...
        segment_mass_paths{2}, 2, 2, ... 
        1, ... 
        conn_tip_to_mid_base_pos_xy, ...
        false, false, ...
        layout_params_struct);

    % --- 果实连接 ---
    tip_next_available_fconn_idx = 1 + 1; 
    
    if has_fruit_at_tip
        fruit_parameters_for_this_tip = current_branch_params.fruit_at_tip;
        fruit_unique_id_for_this_tip = matlab.lang.makeValidName([path_id_str_for_names, '_Fruit']);
        fruit_mass_local_name_on_tip = [fruit_unique_id_for_this_tip, '_Mass'];
        [y_ic_fruit, z_ic_fruit] = get_ic_strings(fruit_mass_local_name_on_tip); % *** 获取初始条件 ***

        % *** 新增：健壮性检查 ***
        if isempty(y_ic_fruit) || ~ischar(y_ic_fruit), y_ic_fruit = '0'; end
        if isempty(z_ic_fruit) || ~ischar(z_ic_fruit), z_ic_fruit = '0'; end
        fruit_layout_x_pos = tip_segment_layout_x + layout_params_struct.segment_width + layout_params_struct.fruit_offset_x;
        fruit_mass_path_on_tip = create_mass_subsystem_2D(...
            current_branch_subsystem_actual_full_path, ...
            fruit_mass_local_name_on_tip, ...
            fruit_parameters_for_this_tip.m, ...
            model_build_params_struct.gravity_g, ...
            1, ... 
            true, ... 
            [fruit_layout_x_pos, segment_layout_y_pos_inside_branch, ... 
            fruit_layout_x_pos + layout_params_struct.segment_width, ...
            segment_layout_y_pos_inside_branch + layout_params_struct.segment_height], ...
            '0', '0' ... % <-- 核心修改点：强制初始条件为0
        );
        % 把这行注释掉代表着不添加果实位置到激励点中
        % excitation_targets_global{end+1} = struct('path', fruit_mass_path_on_tip, 'name', fruit_mass_local_name_on_tip);
        
        conn_fruit_to_tip_base_pos_xy = [fruit_layout_x_pos - layout_params_struct.fruit_offset_x / 1.5, ...
                                         segment_layout_y_pos_inside_branch + layout_params_struct.segment_height / 2];
        connect_fruit_with_detachment_2D(...
            current_branch_subsystem_actual_full_path, ...
            segment_mass_paths{3}, ...      
            fruit_mass_path_on_tip, ...     
            fruit_unique_id_for_this_tip, ...
            fruit_parameters_for_this_tip, ... 
            model_build_params_struct.gravity_g, ...
            tip_next_available_fconn_idx, ... 
            1, ...                            
            conn_fruit_to_tip_base_pos_xy ...
        );
        tip_next_available_fconn_idx = tip_next_available_fconn_idx + 1; 

        % --- 在全局信号管理器中查找或创建此果实条目 ---
        idx_fruit_signal_in_manager = -1;
        for i_fsm = 1:length(fruit_signal_manager_global)
            if strcmp(fruit_signal_manager_global{i_fsm}.unique_id, fruit_unique_id_for_this_tip)
                idx_fruit_signal_in_manager = i_fsm;
                break;
            end
        end
        
        % --- 新增：为父级分枝尖端也定义信号变量名 ---
        parent_tip_id = strrep(fruit_unique_id_for_this_tip, '_Fruit', '_Tip');
        parent_tip_y_disp_varname  = matlab.lang.makeValidName([parent_tip_id, '_y_displacement']);
        parent_tip_z_disp_varname  = matlab.lang.makeValidName([parent_tip_id, '_z_displacement']);
        parent_tip_y_accel_varname = matlab.lang.makeValidName([parent_tip_id, '_y_acceleration']);
        parent_tip_z_accel_varname = matlab.lang.makeValidName([parent_tip_id, '_z_acceleration']);
        
        if idx_fruit_signal_in_manager > 0
            y_disp_varname_fruit  = fruit_signal_manager_global{idx_fruit_signal_in_manager}.y_disp_varname;
            z_disp_varname_fruit  = fruit_signal_manager_global{idx_fruit_signal_in_manager}.z_disp_varname;
            y_accel_varname_fruit = fruit_signal_manager_global{idx_fruit_signal_in_manager}.y_accel_varname;
            z_accel_varname_fruit = fruit_signal_manager_global{idx_fruit_signal_in_manager}.z_accel_varname;

            fruit_mass_rel_path_in_branch = strrep(fruit_mass_path_on_tip, [current_branch_subsystem_actual_full_path, '/'], '');
            tip_mass_rel_path_in_branch   = strrep(segment_mass_paths{3}, [current_branch_subsystem_actual_full_path, '/'], '');
            
            % 定义布局参数
            ws_block_layout_x = fruit_layout_x_pos + layout_params_struct.segment_width + 20;
            ws_block_layout_y_offset = 50; 
            ws_v_spacing = 30; ws_height = 20; ws_width = 200;  

            % 创建果实的To Workspace模块
            add_block('simulink/Sinks/To Workspace', [current_branch_subsystem_actual_full_path, '/', y_disp_varname_fruit, '_ToWs'], ...
                      'VariableName', y_disp_varname_fruit, 'SaveFormat', 'Timeseries', 'SampleTime', '-1', ...
                      'Position', [ws_block_layout_x, segment_layout_y_pos_inside_branch + ws_block_layout_y_offset, ...
                                   ws_block_layout_x + ws_width, segment_layout_y_pos_inside_branch + ws_block_layout_y_offset + ws_height]);
            add_line(current_branch_subsystem_actual_full_path, [fruit_mass_rel_path_in_branch, '/1'], [y_disp_varname_fruit, '_ToWs/1']); 
            ws_block_layout_y_offset = ws_block_layout_y_offset + ws_v_spacing;

            add_block('simulink/Sinks/To Workspace', [current_branch_subsystem_actual_full_path, '/', z_disp_varname_fruit, '_ToWs'], ...
                      'VariableName', z_disp_varname_fruit, 'SaveFormat', 'Timeseries', 'SampleTime', '-1', ...
                      'Position', [ws_block_layout_x, segment_layout_y_pos_inside_branch + ws_block_layout_y_offset, ...
                                   ws_block_layout_x + ws_width, segment_layout_y_pos_inside_branch + ws_block_layout_y_offset + ws_height]);
            add_line(current_branch_subsystem_actual_full_path, [fruit_mass_rel_path_in_branch, '/3'], [z_disp_varname_fruit, '_ToWs/1']); 
            ws_block_layout_y_offset = ws_block_layout_y_offset + ws_v_spacing;
            
            add_block('simulink/Sinks/To Workspace', [current_branch_subsystem_actual_full_path, '/', y_accel_varname_fruit, '_ToWs'], ...
                      'VariableName', y_accel_varname_fruit, 'SaveFormat', 'Timeseries', 'SampleTime', '-1', ...
                      'Position', [ws_block_layout_x, segment_layout_y_pos_inside_branch + ws_block_layout_y_offset, ...
                                   ws_block_layout_x + ws_width, segment_layout_y_pos_inside_branch + ws_block_layout_y_offset + ws_height]);
            add_line(current_branch_subsystem_actual_full_path, [fruit_mass_rel_path_in_branch, '/5'], [y_accel_varname_fruit, '_ToWs/1']); 
            ws_block_layout_y_offset = ws_block_layout_y_offset + ws_v_spacing;

            add_block('simulink/Sinks/To Workspace', [current_branch_subsystem_actual_full_path, '/', z_accel_varname_fruit, '_ToWs'], ...
                      'VariableName', z_accel_varname_fruit, 'SaveFormat', 'Timeseries', 'SampleTime', '-1', ...
                      'Position', [ws_block_layout_x, segment_layout_y_pos_inside_branch + ws_block_layout_y_offset, ...
                                   ws_block_layout_x + ws_width, segment_layout_y_pos_inside_branch + ws_block_layout_y_offset + ws_height]);
            add_line(current_branch_subsystem_actual_full_path, [fruit_mass_rel_path_in_branch, '/6'], [z_accel_varname_fruit, '_ToWs/1']); 
            
            % --- 新增：为父级分枝尖端创建To Workspace模块 ---
            ws_block_layout_y_offset = ws_block_layout_y_offset + 20; % 增加一些间距
            add_block('simulink/Sinks/To Workspace', [current_branch_subsystem_actual_full_path, '/', parent_tip_y_disp_varname, '_ToWs'], ...
                'VariableName', parent_tip_y_disp_varname, 'SaveFormat', 'Timeseries', ...
                'Position', [ws_block_layout_x, ws_block_layout_y_offset, ws_block_layout_x+ws_width, ws_block_layout_y_offset+ws_height]); 
            add_line(current_branch_subsystem_actual_full_path, [tip_mass_rel_path_in_branch, '/1'], [parent_tip_y_disp_varname, '_ToWs/1']); 
            ws_block_layout_y_offset=ws_block_layout_y_offset+ws_v_spacing;
            
            add_block('simulink/Sinks/To Workspace', [current_branch_subsystem_actual_full_path, '/', parent_tip_z_disp_varname, '_ToWs'], ...
                'VariableName', parent_tip_z_disp_varname, 'SaveFormat', 'Timeseries', ...
                'Position', [ws_block_layout_x, ws_block_layout_y_offset, ws_block_layout_x+ws_width, ws_block_layout_y_offset+ws_height]); 
            add_line(current_branch_subsystem_actual_full_path, [tip_mass_rel_path_in_branch, '/3'], [parent_tip_z_disp_varname, '_ToWs/1']); 
            ws_block_layout_y_offset=ws_block_layout_y_offset+ws_v_spacing;
            
            add_block('simulink/Sinks/To Workspace', [current_branch_subsystem_actual_full_path, '/', parent_tip_y_accel_varname, '_ToWs'], ...
                'VariableName', parent_tip_y_accel_varname, 'SaveFormat', 'Timeseries', ...
                'Position', [ws_block_layout_x, ws_block_layout_y_offset, ws_block_layout_x+ws_width, ws_block_layout_y_offset+ws_height]); 
            add_line(current_branch_subsystem_actual_full_path, [tip_mass_rel_path_in_branch, '/5'], [parent_tip_y_accel_varname, '_ToWs/1']); 
            ws_block_layout_y_offset=ws_block_layout_y_offset+ws_v_spacing;
            
            add_block('simulink/Sinks/To Workspace', [current_branch_subsystem_actual_full_path, '/', parent_tip_z_accel_varname, '_ToWs'], ...
                'VariableName', parent_tip_z_accel_varname, 'SaveFormat', 'Timeseries', ...
                'Position', [ws_block_layout_x, ws_block_layout_y_offset, ws_block_layout_x+ws_width, ws_block_layout_y_offset+ws_height]); 
            add_line(current_branch_subsystem_actual_full_path, [tip_mass_rel_path_in_branch, '/6'], [parent_tip_z_accel_varname, '_ToWs/1']);
            
            % --- 修正的核心：将父级分枝尖端的信号变量名更新到全局信号管理器中 ---
            fruit_signal_manager_global{idx_fruit_signal_in_manager}.parent_tip_y_disp_varname  = parent_tip_y_disp_varname;
            fruit_signal_manager_global{idx_fruit_signal_in_manager}.parent_tip_z_disp_varname  = parent_tip_z_disp_varname;
            fruit_signal_manager_global{idx_fruit_signal_in_manager}.parent_tip_y_accel_varname = parent_tip_y_accel_varname;
            fruit_signal_manager_global{idx_fruit_signal_in_manager}.parent_tip_z_accel_varname = parent_tip_z_accel_varname;

        else
            warning('build_branch_recursively: 未能在 fruit_signal_manager_global 中找到果实ID "%s" 的条目。无法为其创建位移/加速度记录模块。', fruit_unique_id_for_this_tip);
        end
    end

    % --- 创建Tip状态总线输出 (TipState_ExternalOut) ---
    tip_state_bus_creator_name_inside_branch_base = [path_id_str_for_names, '_TipState_BusCreator_Internal'];
    add_block('simulink/Signal Routing/Bus Creator', ...
              [current_branch_subsystem_actual_full_path, '/', tip_state_bus_creator_name_inside_branch_base], ...
              'Inputs', '4', 'DisplayOption', 'bar', 'MakeNameUnique', 'on', ...
              'Position', [tip_segment_layout_x + layout_params_struct.segment_width + 20, ...
                           segment_layout_y_pos_inside_branch + 20, ...
                           tip_segment_layout_x + layout_params_struct.segment_width + 20 + layout_params_struct.bus_creator_width, ...
                           segment_layout_y_pos_inside_branch + 20 + layout_params_struct.bus_creator_height]);
    tip_state_bus_creator_actual_name_inside_branch = get_param([current_branch_subsystem_actual_full_path, '/', tip_state_bus_creator_name_inside_branch_base], 'Name');
    
    tip_mass_rel_path_in_branch = strrep(segment_mass_paths{3}, [current_branch_subsystem_actual_full_path, '/'], '');
    add_line(current_branch_subsystem_actual_full_path, [tip_mass_rel_path_in_branch, '/1'], [tip_state_bus_creator_actual_name_inside_branch, '/1']); 
    add_line(current_branch_subsystem_actual_full_path, [tip_mass_rel_path_in_branch, '/2'], [tip_state_bus_creator_actual_name_inside_branch, '/2']); 
    add_line(current_branch_subsystem_actual_full_path, [tip_mass_rel_path_in_branch, '/3'], [tip_state_bus_creator_actual_name_inside_branch, '/3']); 
    add_line(current_branch_subsystem_actual_full_path, [tip_mass_rel_path_in_branch, '/4'], [tip_state_bus_creator_actual_name_inside_branch, '/4']); 

    branch_external_tip_state_outport_name = 'TipState_ExternalOut'; 
    outport_tip_state_x_pos = tip_segment_layout_x + layout_params_struct.segment_width + 50 + layout_params_struct.bus_creator_width + 20;
    add_block('simulink/Sinks/Out1', [current_branch_subsystem_actual_full_path, '/', branch_external_tip_state_outport_name], ...
              'Port', '1', ... % TipState_ExternalOut 固定为端口1
              'Position', [outport_tip_state_x_pos, segment_layout_y_pos_inside_branch + 50, ...
                           outport_tip_state_x_pos + layout_params_struct.general_outport_width, ...
                           segment_layout_y_pos_inside_branch + 50 + layout_params_struct.general_outport_height]);
    add_line(current_branch_subsystem_actual_full_path, [tip_state_bus_creator_actual_name_inside_branch, '/1'], [branch_external_tip_state_outport_name, '/1']);

    % --- 递归调用自身以构建下一级子分枝 (如果存在) ---
    if branch_level < 3 && num_sub_branches_from_this_tip > 0 
        next_branch_level_to_build = branch_level + 1;
        child_branch_base_x_pos = base_pos_xy(1) + layout_params_struct.subsystem_width + 100; 
        
        child_branch_initial_y_offset_from_parent = 0;
        if branch_level > 0 
            child_branch_initial_y_offset_from_parent = (layout_params_struct.subsystem_height_spacing / max(1,num_sub_branches_from_this_tip+1)) - current_branch_subsystem_height/2;
        end

        for i_child_branch = 1:num_sub_branches_from_this_tip
            child_branch_indices_for_next_level = [branch_indices, i_child_branch]; 
            
            child_path_id_parts_preview_temp = {};
            if next_branch_level_to_build == 1
                child_path_id_parts_preview_temp = {['P', num2str(i_child_branch)]};
            elseif next_branch_level_to_build == 2
                % path_id_str_for_names 在这里应该是 "P<idx_p>"
                child_path_id_parts_preview_temp = {path_id_str_for_names, ['S', num2str(i_child_branch)]};
            elseif next_branch_level_to_build == 3
                % path_id_str_for_names 在这里应该是 "P<idx_p>_S<idx_s>"
                child_path_id_parts_preview_temp = {path_id_str_for_names, ['T', num2str(i_child_branch)]};
            end
            child_prefix_preview_str = strjoin(child_path_id_parts_preview_temp, '_');
            child_branch_subsystem_path_tentative_for_next_call = [model_base_path, '/', matlab.lang.makeValidName([child_prefix_preview_str, '_Branch'])];
            
            reaction_port_idx_on_parent_tip_for_this_child = tip_next_available_fconn_idx;
            tip_next_available_fconn_idx = tip_next_available_fconn_idx + 1; 
            
            child_actual_y_pos_for_call = actual_subsystem_base_pos_y; 
            if branch_level > 0 
                 child_actual_y_pos_for_call = actual_subsystem_base_pos_y + ...
                     child_branch_initial_y_offset_from_parent + ...
                     (i_child_branch - 1) * (current_branch_subsystem_height * 1.1 / max(1, num_sub_branches_from_this_tip));
            end
            child_actual_y_pos_for_call = max(layout_params_struct.y_start, child_actual_y_pos_for_call);

            build_branch_recursively(model_base_path, ...
                                     current_branch_subsystem_actual_full_path, ... 
                                     segment_mass_paths{3}, ...                     
                                     reaction_port_idx_on_parent_tip_for_this_child, ... 
                                     reaction_port_idx_on_parent_tip_for_this_child, ... 
                                     next_branch_level_to_build, ...
                                     child_branch_indices_for_next_level, ...
                                     model_build_params_struct, layout_params_struct, ...
                                     [child_branch_base_x_pos, child_actual_y_pos_for_call], ...
                                     child_branch_subsystem_path_tentative_for_next_call);
        end
    end
    fprintf('  build_branch_recursively: 分枝 "%s" 构建完成。\n', path_id_str_for_names);
end

%% branch_params
% --- 辅助函数，用于从单自由度值生成多段参数 ---
% m_total: 该分枝的总质量
% k_base: 该分枝的基础刚度 (用于Y方向)
% c_base: 该分枝的基础阻尼 (用于Y方向)
% z_factor: Z方向参数与Y方向参数的比例
% taper_factors_k: 刚度在root/mid/tip连接处的衰减因子 (相对于k_base)
% taper_factors_c: 阻尼在root/mid/tip连接处的衰减因子 (相对于c_base)
% mass_dist_factors: 质量在root/mid/tip的分配因子    
function branch_params = generate_branch_segment_params(m_total, k_base, c_base)
    z_factor_local = 1; % Z方向参数相对于Y方向的比例因子 (局部，可调整)
    mass_dist = [0.5, 0.3, 0.2]; % 质量分配: root, mid, tip
    k_taper   = [0.5, 0.3, 0.2]; % 内部连接刚度衰减因子 for root_conn, mid_conn, tip_conn
    c_taper   = [1.0, 0.8, 0.6]; % 内部连接阻尼衰减因子
    
    branch_params = struct();
    branch_params.root = struct('m', m_total * mass_dist(1), ...
                                'k_y_conn', k_base * k_taper(1), 'c_y_conn', c_base * c_taper(1), ...
                                'k_z_conn', k_base * k_taper(1) * z_factor_local, 'c_z_conn', c_base * c_taper(1) * z_factor_local);
    branch_params.mid  = struct('m', m_total * mass_dist(2), ...
                                'k_y_conn', k_base * k_taper(2), 'c_y_conn', c_base * c_taper(2), ...
                                'k_z_conn', k_base * k_taper(2) * z_factor_local, 'c_z_conn', c_base * c_taper(2) * z_factor_local);
    branch_params.tip  = struct('m', m_total * mass_dist(3), ...
                                'k_y_conn', k_base * k_taper(3), 'c_y_conn', c_base * c_taper(3), ...
                                'k_z_conn', k_base * k_taper(3) * z_factor_local, 'c_z_conn', c_base * c_taper(3) * z_factor_local);
end

%% plot_tree_topology (主调用函数) - 最终正确版
% 功能: 初始化并调用递归函数来绘制果树拓扑结构。
function plot_tree_topology(config)
    cla; 
    hold on;
    
    trunk_base_coord = [0, 0];
    trunk_length = config.trunk.length; 
    branch_length_decay = 0.7; 
    trunk_angle = 90; 
    primary_spread_angle = 60; 
    
    trunk_tip_coord = trunk_base_coord + [trunk_length * cosd(trunk_angle), trunk_length * sind(trunk_angle)];
    plot([trunk_base_coord(1), trunk_tip_coord(1)], [trunk_base_coord(2), trunk_tip_coord(2)], 'k-', 'LineWidth', 5, 'DisplayName', '主干');
    
    legend_map = containers.Map({'P', 'S', 'T', 'Fruit'}, {false, false, false, false});

    recursive_plot_branch(config, trunk_tip_coord, trunk_angle, 1, [], trunk_length * branch_length_decay, primary_spread_angle, branch_length_decay, legend_map);
    
    hold off;
    legend('show','Location','best');
end

%% recursive_plot_branch (递归绘图函数) - 最终正确版
function recursive_plot_branch(config, parent_coord, parent_angle, level, indices, branch_len, spread_angle, decay, legend_map)
    
    if level > 3, return; end
    
    num_branches_at_this_level = 0;
    if level == 1
        num_branches_at_this_level = config.num_primary_branches;
    elseif level == 2
        p_idx = indices(1);
        if p_idx > length(config.secondary_branches_count), return; end
        num_branches_at_this_level = config.secondary_branches_count(p_idx);
    elseif level == 3
        p_idx = indices(1); s_idx = indices(2);
        if p_idx > length(config.tertiary_branches_count) || s_idx > length(config.tertiary_branches_count{p_idx}), return; end
        num_branches_at_this_level = config.tertiary_branches_count{p_idx}(s_idx);
    end

    if num_branches_at_this_level == 0, return; end
    
    angles = linspace(parent_angle - spread_angle / 2, parent_angle + spread_angle / 2, num_branches_at_this_level);
    
    level_info = {{'P', 'b', '一级分枝'}, {'S', 'g', '二级分枝'}, {'T', 'm', '三级分枝'}};
    
    level_tag_key = level_info{level}{1};
    level_color = level_info{level}{2};
    level_label = level_info{level}{3};
    
    for i = 1:num_branches_at_this_level
        current_angle = angles(i);
        child_coord = parent_coord + [branch_len * cosd(current_angle), branch_len * sind(current_angle)];
        
        plot_args = {'-', 'Color', level_color, 'LineWidth', max(1, 5 - level*1.5)};
        
        if ~legend_map(level_tag_key)
            plot_args{end+1} = 'DisplayName';
            plot_args{end+1} = level_label;
            legend_map(level_tag_key) = true;
        else
            plot_args{end+1} = 'HandleVisibility';
            plot_args{end+1} = 'off';
        end
        
        plot([parent_coord(1), child_coord(1)], [parent_coord(2), child_coord(2)], plot_args{:});
        
        new_indices = [indices, i];
        
        is_terminal = false;
        if level == 3
            is_terminal = true;
        elseif level < 3
            % 检查下一级的分枝数是否为0
            num_sub_branches = 0;
            if (level + 1) == 2 % 当前是P，检查S数量
                num_sub_branches = config.secondary_branches_count(new_indices(1));
            elseif (level + 1) == 3 % 当前是S，检查T数量
                num_sub_branches = config.tertiary_branches_count{new_indices(1)}(new_indices(2));
            end
            if num_sub_branches == 0
                is_terminal = true;
            end
        end
        
        if is_terminal
            % 如果是末梢，就在其末端画果实
            if ~legend_map('Fruit')
                plot(child_coord(1), child_coord(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6, 'DisplayName', '果实');
                legend_map('Fruit') = true;
            else
                plot(child_coord(1), child_coord(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6, 'HandleVisibility', 'off');
            end
        else
            % 如果不是末梢，就继续递归
            recursive_plot_branch(config, child_coord, current_angle, level + 1, new_indices, branch_len * decay, spread_angle * 0.8, decay, legend_map);
        end
    end
end

%% map_dofs_recursively_static
% 功能: (静态分析辅助函数) 递归遍历参数结构体，为所有质量点建立一个自由度(DOF)映射表。
%       返回一个元胞数组，每个元素包含 {质量点唯一ID, 质量值}。
%% map_dofs_recursively_static (黄金标准版)
% 功能: (静态分析辅助函数) 递归遍历参数结构体，为所有质量点建立一个自由度(DOF)映射表。
%       此版本作为所有ID生成的“单一事实来源”。
function dof_map_out = map_dofs_recursively_static(parameters)
    
    dof_map = {}; % 初始化空的DOF映射表

    % 嵌套的递归函数
    function traverse(params, path_prefix)
        % 添加当前分枝的 root, mid, tip 质量点
        dof_map{end+1} = {[path_prefix '_root_Mass'], params.root.m};
        dof_map{end+1} = {[path_prefix '_mid_Mass'],  params.mid.m};
        dof_map{end+1} = {[path_prefix '_tip_Mass'],  params.tip.m};

        % 检查 fruit_at_mid 位置的果实
        if isfield(params, 'fruit_at_mid') && isstruct(params.fruit_at_mid) && isfield(params.fruit_at_mid, 'm')
            dof_map{end+1} = {[path_prefix, '_mid_Fruit_Mass'], params.fruit_at_mid.m};
        end
        
        % 检查 fruit_at_tip 位置的果实
        if isfield(params, 'fruit_at_tip') && isstruct(params.fruit_at_tip) && isfield(params.fruit_at_tip, 'm')
            dof_map{end+1} = {[path_prefix, '_Fruit_Mass'], params.fruit_at_tip.m};
        end
        
        % 向后兼容：检查旧字段名 'fruit'（仅当 fruit_at_tip 不存在时）
        if ~isfield(params, 'fruit_at_tip') && isfield(params, 'fruit') && ...
           isstruct(params.fruit) && isfield(params.fruit, 'm')
            dof_map{end+1} = {[path_prefix, '_Fruit_Mass'], params.fruit.m};
        end
        
        % 递归处理二级子分枝 (注意: path_prefix 的构建方式)
        if isfield(params, 'secondary_branches')
            for s_idx = 1:length(params.secondary_branches)
                % 新的path_prefix是当前prefix加上 "_S<idx>"
                new_prefix = [path_prefix, '_S', num2str(s_idx)];
                traverse(params.secondary_branches{s_idx}, new_prefix);
            end
        end
        % 递归处理三级子分枝
        if isfield(params, 'tertiary_branches')
            for t_idx = 1:length(params.tertiary_branches)
                 % 新的path_prefix是当前prefix加上 "_T<idx>"
                new_prefix = [path_prefix, '_T', num2str(t_idx)];
                traverse(params.tertiary_branches{t_idx}, new_prefix);
            end
        end
    end

    % 从主干开始
    traverse(parameters.trunk, 'Trunk');
    
    % 遍历所有一级分枝
    for p_idx = 1:length(parameters.primary)
        % 一级分枝的prefix是 "P<idx>"
        prefix_p = ['P', num2str(p_idx)];
        traverse(parameters.primary{p_idx}, prefix_p);
    end

    % 使用 matlab.lang.makeValidName 确保所有ID都是合法的变量名
    for i = 1:length(dof_map)
        dof_map{i}{1} = matlab.lang.makeValidName(dof_map{i}{1});
    end

    dof_map_out = dof_map;
end

%% populate_K_matrix_static (同步黄金标准版)
% 功能: (静态分析辅助函数) 根据DOF映射表和系统参数，构建并返回全局刚度矩阵K。
%       此版本严格遵循 map_dofs_recursively_static 的ID生成逻辑。
function K_out = populate_K_matrix_static(dof_map, parameters)
    
    num_masses = length(dof_map);
    num_dofs = num_masses * 2;
    K = sparse(num_dofs, num_dofs);
    
    mass_id_to_idx_map = containers.Map(cellfun(@(c) c{1}, dof_map, 'UniformOutput', false), 1:num_masses);

    % (add_spring_to_K 嵌套函数保持不变，此处为简洁省略)
    function add_spring_to_K(mass_id_1, mass_id_2, k_y, k_z)
        if ~isKey(mass_id_to_idx_map, mass_id_1), error('ID %s not found in map', mass_id_1); end
        idx1 = mass_id_to_idx_map(mass_id_1);
        y_dof1 = 2*idx1 - 1; z_dof1 = 2*idx1;
        if isempty(mass_id_2)
            K(y_dof1, y_dof1) = K(y_dof1, y_dof1) + k_y; K(z_dof1, z_dof1) = K(z_dof1, z_dof1) + k_z;
        else
            if ~isKey(mass_id_to_idx_map, mass_id_2), error('ID %s not found in map', mass_id_2); end
            idx2 = mass_id_to_idx_map(mass_id_2);
            y_dof2 = 2*idx2 - 1; z_dof2 = 2*idx2;
            k_sub_y = [k_y, -k_y; -k_y, k_y]; dof_indices_y = [y_dof1, y_dof2]; K(dof_indices_y, dof_indices_y) = K(dof_indices_y, dof_indices_y) + k_sub_y;
            k_sub_z = [k_z, -k_z; -k_z, k_z]; dof_indices_z = [z_dof1, z_dof2]; K(dof_indices_z, dof_indices_z) = K(dof_indices_z, dof_indices_z) + k_sub_z;
        end
    end

    % 嵌套的递归函数，逻辑与 map_dofs 中的 traverse 完全一致
    function traverse(params, path_prefix, parent_tip_id)
        root_id = matlab.lang.makeValidName([path_prefix '_root_Mass']);
        mid_id  = matlab.lang.makeValidName([path_prefix '_mid_Mass']);
        tip_id  = matlab.lang.makeValidName([path_prefix '_tip_Mass']);
        
        if isempty(parent_tip_id)
             add_spring_to_K(root_id, [], params.root.k_y_conn_to_base, params.root.k_z_conn_to_base);
        else
             add_spring_to_K(root_id, parent_tip_id, params.root.k_y_conn, params.root.k_z_conn);
        end

        add_spring_to_K(root_id, mid_id, params.root.k_y_conn, params.root.k_z_conn);
        add_spring_to_K(mid_id, tip_id, params.mid.k_y_conn, params.mid.k_z_conn);

         % 果实刚度 - fruit_at_tip
        if isfield(params, 'fruit_at_tip') && isstruct(params.fruit_at_tip) && isfield(params.fruit_at_tip, 'k_pedicel_y')
            fruit_id = [path_prefix, '_Fruit_Mass'];
            if isKey(id_to_idx, fruit_id)
                add_spring(tip_id, fruit_id, params.fruit_at_tip.k_pedicel_y, params.fruit_at_tip.k_pedicel_z);
            end
        end
        
       % 果实刚度 - fruit_at_tip
        if isfield(params, 'fruit_at_tip') && isstruct(params.fruit_at_tip) && isfield(params.fruit_at_tip, 'k_pedicel_y')
            fruit_id = matlab.lang.makeValidName([path_prefix, '_Fruit_Mass']);
            if isKey(mass_id_to_idx_map, fruit_id)
                add_spring_to_K(tip_id, fruit_id, params.fruit_at_tip.k_pedicel_y, params.fruit_at_tip.k_pedicel_z);
            end
        end
        
        % 果实刚度 - fruit_at_mid
        if isfield(params, 'fruit_at_mid') && isstruct(params.fruit_at_mid) && isfield(params.fruit_at_mid, 'k_pedicel_y')
            fruit_mid_id = matlab.lang.makeValidName([path_prefix, '_mid_Fruit_Mass']);
            if isKey(mass_id_to_idx_map, fruit_mid_id)
                add_spring_to_K(mid_id, fruit_mid_id, params.fruit_at_mid.k_pedicel_y, params.fruit_at_mid.k_pedicel_z);
            end
        end
        
        % 向后兼容旧字段 'fruit'
        if ~isfield(params, 'fruit_at_tip') && isfield(params, 'fruit') && ...
           isstruct(params.fruit) && isfield(params.fruit, 'k_pedicel_y')
            fruit_id = matlab.lang.makeValidName([path_prefix, '_Fruit_Mass']);
            if isKey(mass_id_to_idx_map, fruit_id)
                add_spring_to_K(tip_id, fruit_id, params.fruit.k_pedicel_y, params.fruit.k_pedicel_z);
            end
        end

        if isfield(params, 'secondary_branches')
            for s_idx = 1:length(params.secondary_branches)
                new_prefix = [path_prefix, '_S', num2str(s_idx)];
                traverse(params.secondary_branches{s_idx}, new_prefix, tip_id);
            end
        end

        if isfield(params, 'tertiary_branches')
            for t_idx = 1:length(params.tertiary_branches)
                new_prefix = [path_prefix, '_T', num2str(t_idx)];
                traverse(params.tertiary_branches{t_idx}, new_prefix, tip_id);
            end
        end
    end

    % 从主干开始
    traverse(parameters.trunk, 'Trunk', []);
    
    % 遍历所有一级分枝
    trunk_tip_id = matlab.lang.makeValidName('Trunk_tip_Mass');
    for p_idx = 1:length(parameters.primary)
        prefix_p = ['P', num2str(p_idx)];
        traverse(parameters.primary{p_idx}, prefix_p, trunk_tip_id);
    end

    K_out = K;
end
%% plot_dynamic_signal (新增的绘图辅助函数 - 推荐使用)
% 功能: 在给定的坐标轴(ax)上，健壮地绘制一条信号曲线。
%       新增功能：可以根据需要减去信号的初始值，用于显示动态响应。
% 输入参数:
%   ax (axes handle):            目标绘图坐标轴。
%   signal_map (containers.Map): 包含所有信号数据的Map对象。
%   signal_name (char/string):   要从Map中提取并绘制的信号的名称。
%   display_name (char/string):  图例中显示的曲线名称。
%   line_style (char/string):    线型 (e.g., '-', '--', ':')。
%   color (char/string or RGB):  颜色 (e.g., 'b', 'r', [0 1 0])。
%   subtract_initial_value (logical): 如果为true，则绘制 (Data - Data(1))。
function plot_dynamic_signal(ax, signal_map, signal_name, display_name, line_style, color, subtract_initial_value)
    % 检查信号是否存在于数据map中
    if isKey(signal_map, signal_name)
        ts = signal_map(signal_name);
        % 确保数据是有效的timeseries对象，并且数据非空、长度一致
        if isa(ts, 'timeseries') && ~isempty(ts.Time) && ~isempty(ts.Data) && (length(ts.Time) == length(ts.Data))
            
            data_to_plot = ts.Data;
            % 如果需要，减去初始值
            if subtract_initial_value
                data_to_plot = data_to_plot - data_to_plot(1);
            end
            
            % 满足所有条件，执行绘图
            plot(ax, ts.Time, data_to_plot, 'LineStyle', line_style, 'Color', color, 'DisplayName', display_name);
        end
    end
end

function sim_engine = initializeAdaptiveSimulationEngine(unified_params, tree_topology)
    % 自适应仿真引擎初始化
    %
    % 根据V3手稿2.4.1节:
    % "基于MATLAB/Simulink平台开发能够自动处理拓扑结构变化与混合动力学特性的自适应仿真引擎"
    %
    % 输入:
    %   unified_params - SAD框架输出的统一参数接口
    %   tree_topology  - 树体拓扑结构定义
    %
    % 输出:
    %   sim_engine - 仿真引擎结构体
    
    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════════════╗\n');
    fprintf('║  初始化自适应仿真引擎 (Adaptive Simulation Engine)              ║\n');
    fprintf('╚══════════════════════════════════════════════════════════════════╝\n\n');
    
    sim_engine = struct();
    
    %% 1. 参数-拓扑映射模块
    fprintf('  [1] 构建参数-拓扑映射模块...\n');
    
    sim_engine.param_topology_map = buildParameterTopologyMap(...
        unified_params, tree_topology);
    
    fprintf('      映射了 %d 个节点的参数\n', ...
        length(sim_engine.param_topology_map.node_ids));
    
    %% 2. 局部自适应组装策略
    fprintf('  [2] 配置局部自适应组装策略...\n');
    
    sim_engine.assembly_config = configureAdaptiveAssembly(unified_params);
    
    fprintf('      线性节点: %d, 非线性节点: %d\n', ...
        sum(~sim_engine.assembly_config.is_nonlinear), ...
        sum(sim_engine.assembly_config.is_nonlinear));
    
    %% 3. 混合求解器配置
    fprintf('  [3] 配置混合求解器...\n');
    
    sim_engine.solver_config = configureMixedSolver();
    
    fprintf('      主求解器: %s, 备用求解器: %s\n', ...
        sim_engine.solver_config.primary_solver, ...
        sim_engine.solver_config.backup_solver);
    
    %% 4. 果实脱落逻辑配置
    fprintf('  [4] 配置状态依赖脱落逻辑...\n');
    
    sim_engine.detachment_logic = configureDetachmentLogic(unified_params);
    
    fprintf('      脱落力模型: %s\n', sim_engine.detachment_logic.model_type);
    
    %% 5. 输出记录配置
    fprintf('  [5] 配置输出记录器...\n');
    
    sim_engine.output_recorder = configureOutputRecorder();
    
    fprintf('\n  [√] 仿真引擎初始化完成\n\n');
end


%% =====================================================================
%% 【新增】参数-拓扑映射模块
%% =====================================================================

function map = buildParameterTopologyMap(unified_params, tree_topology)
    % 构建参数-拓扑映射
    %
    % 根据V3手稿:
    % "参数-拓扑映射模块(Parameter-Topology Mapping Module)充当实验数据与数值模型之间的智能接口"
    
    map = struct();
    
    % 节点ID列表
    map.node_ids = {'Trunk_root', 'Trunk_mid', 'Trunk_tip'};
    
    % 添加一级分枝节点
    if isfield(tree_topology, 'primary_branches')
        for p = 1:length(tree_topology.primary_branches)
            prefix = sprintf('P%d', p);
            map.node_ids{end+1} = [prefix '_root'];
            map.node_ids{end+1} = [prefix '_mid'];
            map.node_ids{end+1} = [prefix '_tip'];
        end
    end
    
    % 为每个节点建立参数索引
    map.n_nodes = length(map.node_ids);
    map.param_index = containers.Map();
    
    for i = 1:map.n_nodes
        node_id = map.node_ids{i};
        
        % 确定节点类型 (root/mid/tip)
        if contains(node_id, 'root')
            node_type_idx = 1;
        elseif contains(node_id, 'mid')
            node_type_idx = 2;
        else
            node_type_idx = 3;
        end
        
        % 创建参数映射
        param_struct = struct();
        param_struct.node_type_idx = node_type_idx;
        param_struct.is_nonlinear = unified_params.nonlinear.is_active(node_type_idx);
        param_struct.NL_index = unified_params.nonlinear.NL_index(node_type_idx);
        
        % 线性参数
        param_struct.k_linear = unified_params.linear.K(node_type_idx, node_type_idx);
        param_struct.c_linear = unified_params.linear.C(node_type_idx, node_type_idx);
        param_struct.m = unified_params.linear.M(node_type_idx, node_type_idx);
        
        % 非线性参数 (如果适用)
        if param_struct.is_nonlinear
            param_struct.k3 = unified_params.nonlinear.k3(node_type_idx);
            param_struct.c2 = unified_params.nonlinear.c2(node_type_idx);
        else
            param_struct.k3 = 0;
            param_struct.c2 = 0;
        end
        
        map.param_index(node_id) = param_struct;
    end
    
    % 存储统一参数引用
    map.unified_params = unified_params;
end


%% =====================================================================
%% 【新增】局部自适应组装策略
%% =====================================================================

function config = configureAdaptiveAssembly(unified_params)
    % 配置局部自适应组装策略
    %
    % 根据V3手稿:
    % "局部自适应组装策略(Local Adaptive Assembly Strategy)在确保捕捉关键非线性动力学行为的同时,
    %  避免全非线性系统带来的计算冗余"
    
    config = struct();
    
    % 非线性阈值
    config.NL_threshold = unified_params.nonlinear.NL_threshold;
    
    % 各节点的非线性状态
    config.is_nonlinear = unified_params.nonlinear.is_active;
    
    % 组装模式
    config.assembly_mode = 'adaptive';  % 'adaptive', 'all_linear', 'all_nonlinear'
    
    % 为线性节点创建标准矩阵组装函数
    config.linear_assembly_fn = @(K_local, C_local, K_global, C_global, dof_idx) ...
        assembleLinearElement(K_local, C_local, K_global, C_global, dof_idx);
    
    % 为非线性节点创建Duffing力元函数
    config.nonlinear_force_fn = @(u, v, k_lin, c_lin, k3, c2) ...
        computeDuffingForce(u, v, k_lin, c_lin, k3, c2);
end


function [K_global, C_global] = assembleLinearElement(K_local, C_local, K_global, C_global, dof_idx)
    % 线性单元组装
    K_global(dof_idx, dof_idx) = K_global(dof_idx, dof_idx) + K_local;
    C_global(dof_idx, dof_idx) = C_global(dof_idx, dof_idx) + C_local;
end


function F_nl = computeDuffingForce(u, v, k_lin, c_lin, k3, c2)
    % 计算Duffing非线性恢复力
    %
    % 根据V3手稿公式:
    % F_nl = k_lin*u + k3*u³ + c_lin*v + c2*|v|*v
    
    % 线性部分
    F_linear = k_lin * u + c_lin * v;
    
    % 非线性部分
    % 三次刚度
    F_k3 = k3 * u^3;
    
    % 平方阻尼 (流体气动阻尼形式)
    F_c2 = c2 * abs(v) * v;
    
    % 总恢复力
    F_nl = F_linear + F_k3 + F_c2;
end


%% =====================================================================
%% 【新增】混合求解器配置
%% =====================================================================

function config = configureMixedSolver()
    % 配置混合求解策略
    %
    % 根据V3手稿:
    % "采用变步长Runge-Kutta(ode45)积分算法与向后差分公式(BDF, ode15s)相结合的混合求解策略"
    
    config = struct();
    
    % 主求解器 (用于非刚性问题)
    config.primary_solver = 'ode45';
    config.primary_options = odeset(...
        'RelTol', 1e-6, ...
        'AbsTol', 1e-8, ...
        'MaxStep', 0.001, ...  % 最大步长 1ms
        'Events', @detachmentEventFcn);  % 脱落事件检测
    
    % 备用求解器 (用于刚性问题)
    config.backup_solver = 'ode15s';
    config.backup_options = odeset(...
        'RelTol', 1e-4, ...
        'AbsTol', 1e-6, ...
        'MaxStep', 0.002, ...
        'Events', @detachmentEventFcn);
    
    % 自动切换阈值
    config.stiffness_threshold = 1e6;  % 刚度比阈值
    
    % 过零检测配置
    config.zero_crossing = struct();
    config.zero_crossing.enabled = true;
    config.zero_crossing.direction = 0;  % 双向检测
    config.zero_crossing.terminal = false;  % 非终止事件
end


function [value, isterminal, direction] = detachmentEventFcn(t, y)
    % 脱落事件检测函数
    % 用于过零检测技术
    
    % 这个函数需要访问当前果实节点的受力状态
    % 在实际实现中,需要通过全局变量或闭包传递参数
    
    value = 1;  % 占位符
    isterminal = 0;
    direction = 0;
end


%% =====================================================================
%% 【新增】状态依赖脱落逻辑配置
%% =====================================================================

function config = configureDetachmentLogic(unified_params)
    % 配置状态依赖的果实脱落逻辑
    %
    % 根据V3手稿2.4.1节:
    % "状态依赖的果实脱落逻辑(State-dependent Detachment Logic)"
    % "在每一积分时间步实时计算作用于果实节点上的瞬时合力"
    
    config = struct();
    
    % 脱落模型类型
    config.model_type = 'multi_factor_regression';
    
    % 获取脱落力预测模型
    config.detachment_model = unified_params.detachment;
    
    % 力合成方式
    config.force_components = struct();
    config.force_components.inertia = true;   % 惯性力
    config.force_components.gravity = true;   % 重力
    config.force_components.aerodynamic = false;  % 气动阻力 (可选)
    
    % 脱落判据
    config.detachment_criterion = @(F_stem, F_break) (abs(F_stem) >= F_break);
    
    % 脱落后处理
    config.post_detachment = struct();
    config.post_detachment.set_k_zero = true;  % 连接刚度置零
    config.post_detachment.set_c_zero = true;  % 连接阻尼置零
    config.post_detachment.free_fall = true;   % 转为自由落体
    
    % 计算瞬时茎杆力的函数
    config.compute_stem_force = @(accel, mass, gravity) ...
        computeStemForce(accel, mass, gravity);
end


function F_stem = computeStemForce(accel, mass, gravity)
    % 计算作用于果实节点的瞬时茎杆力
    %
    % 根据V3手稿:
    % "该合力由惯性力、重力及气动阻力矢量合成"
    
    % 惯性力 (F = m * a)
    F_inertia = mass * accel;
    
    % 重力
    F_gravity = mass * gravity;
    
    % 合力
    F_stem = sqrt(F_inertia^2 + F_gravity^2);
end


%% =====================================================================
%% 【新增】输出记录器配置
%% =====================================================================

function config = configureOutputRecorder()
    % 配置仿真输出记录器
    
    config = struct();
    
    % 记录的变量
    config.record_displacement = true;
    config.record_velocity = true;
    config.record_acceleration = true;
    config.record_force = true;
    config.record_energy = true;
    
    % 记录频率
    config.sample_rate = 1000;  % Hz
    
    % 脱落事件记录
    config.record_detachment_events = true;
    config.detachment_fields = {'time', 'fruit_id', 'node_id', 'force', 'position'};
end


%% =====================================================================
%% 【新增】激振点优化模块
%% =====================================================================

function [optimal_point, all_scores, heatmap_data] = optimizeExcitationPoint(...
    sim_engine, tree_topology, excitation_config)
    % 激振点优化
    %
    % 根据V3手稿2.4.2节:
    % "复合目标函数与激振点寻优策略"
    %
    % 输入:
    %   sim_engine       - 初始化的仿真引擎
    %   tree_topology    - 树体拓扑结构
    %   excitation_config - 激励配置
    %
    % 输出:
    %   optimal_point - 最优激振点信息
    %   all_scores    - 所有候选点的评分
    %   heatmap_data  - 用于绘制热力图的数据
    
    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════════════╗\n');
    fprintf('║  激振点优化 (Excitation Point Optimization)                     ║\n');
    fprintf('╚══════════════════════════════════════════════════════════════════╝\n\n');
    
    %% 1. 定义候选激振点集合 Ω_exc
    fprintf('  [1] 定义候选激振点集合...\n');
    
    candidate_points = defineCandidateExcitationPoints(tree_topology);
    
    fprintf('      候选点数量: %d\n', length(candidate_points));
    
    %% 2. 设置标准化激励信号
    fprintf('  [2] 配置标准化激励信号...\n');
    
    % 根据V3手稿:
    % "所有仿真均采用标准化的输入信号,施加恒定力幅F_amp=200N的线性扫频激励"
    % "扫频范围覆盖树体前五阶主导模态区间(5-30 Hz),仿真时长设定为5s"
    
    excitation = struct();
    excitation.type = 'chirp';  % 线性扫频
    excitation.amplitude = 200;  % N
    excitation.freq_start = 5;   % Hz
    excitation.freq_end = 30;    % Hz
    excitation.duration = 5;     % s
    
    fprintf('      激励类型: 线性扫频, 幅值: %d N, 频率: %d-%d Hz\n', ...
        excitation.amplitude, excitation.freq_start, excitation.freq_end);
    
    %% 3. 遍历所有候选点进行仿真
    fprintf('  [3] 遍历仿真所有候选点...\n\n');
    
    n_points = length(candidate_points);
    all_scores = zeros(n_points, 1);
    results_all = cell(n_points, 1);
    
    % 权重系数 (V3手稿: w1=0.5, w2=0.3, w3=0.2)
    w1 = 0.5;  % 脱落率权重
    w2 = 0.3;  % 时间效率权重
    w3 = 0.2;  % 能量效率权重
    
    for k = 1:n_points
        point = candidate_points{k};
        fprintf('      [%d/%d] 仿真激振点: %s...', k, n_points, point.id);
        
        % 运行仿真
        sim_result = runExcitationSimulation(sim_engine, point, excitation);
        
        % 计算综合采收效能指数 J_k
        J_k = computeHarvestingPerformanceIndex(...
            sim_result, w1, w2, w3, excitation.duration);
        
        all_scores(k) = J_k;
        results_all{k} = sim_result;
        
        fprintf(' J = %.3f\n', J_k);
    end
    
    %% 4. 确定最优激振点
    fprintf('\n  [4] 确定最优激振点...\n');
    
    [J_max, idx_opt] = max(all_scores);
    optimal_point = candidate_points{idx_opt};
    optimal_point.score = J_max;
    optimal_point.sim_result = results_all{idx_opt};
    
    fprintf('      最优激振点: %s, 综合评分: %.3f\n', ...
        optimal_point.id, J_max);
    
    %% 5. 生成热力图数据
    fprintf('  [5] 生成激振效能热力图数据...\n');
    
    heatmap_data = generateExcitationHeatmap(candidate_points, all_scores, tree_topology);
    
    %% 6. 分析最优区域
    fprintf('  [6] 识别最优激振区段...\n');
    
    optimal_zone = identifyOptimalExcitationZone(candidate_points, all_scores);
    optimal_point.optimal_zone = optimal_zone;
    
    fprintf('      最优区段: %s\n', optimal_zone.description);
    
    fprintf('\n  [√] 激振点优化完成\n\n');
end


%% =====================================================================
%% 【新增】定义候选激振点集合
%% =====================================================================

function points = defineCandidateExcitationPoints(tree_topology)
    % 定义候选激振点集合 Ω_exc
    %
    % 根据V3手稿:
    % "考虑到实际机械作业中夹持机构的可达性与操作稳定性,
    %  该集合主要涵盖所有一级分枝节点(包括Root、Mid和Tip节点)"
    
    points = {};
    point_idx = 1;
    
    % 主干节点 (通常作为参考)
    for node_type = {'root', 'mid', 'tip'}
        point = struct();
        point.id = sprintf('Trunk_%s', node_type{1});
        point.branch_level = 0;
        point.node_type = node_type{1};
        point.position = [0, 0, 0];  % 需要根据拓扑填充
        point.is_accessible = true;
        
        points{point_idx} = point;
        point_idx = point_idx + 1;
    end
    
    % 一级分枝节点
    if isfield(tree_topology, 'primary_branches')
        n_primary = length(tree_topology.primary_branches);
    else
        n_primary = 3;  % 默认
    end
    
    for p = 1:n_primary
        for node_type = {'root', 'mid', 'tip'}
            point = struct();
            point.id = sprintf('P%d_%s', p, node_type{1});
            point.branch_level = 1;
            point.branch_idx = p;
            point.node_type = node_type{1};
            point.is_accessible = true;
            
            points{point_idx} = point;
            point_idx = point_idx + 1;
        end
    end
end


%% =====================================================================
%% 【新增】运行激振仿真
%% =====================================================================

function result = runExcitationSimulation(sim_engine, excitation_point, excitation_config)
    % 运行单个激振点的仿真
    
    result = struct();
    result.excitation_point = excitation_point;
    result.excitation_config = excitation_config;
    
    % 仿真参数
    t_end = excitation_config.duration;
    dt = 1 / sim_engine.output_recorder.sample_rate;
    
    % 生成激励信号
    t = 0:dt:t_end;
    excitation_signal = generateExcitationSignal(t, excitation_config);
    
    % 获取系统参数
    params = sim_engine.param_topology_map.unified_params;
    M = params.linear.M;
    K = params.linear.K;
    C = params.linear.C;
    
    n_dof = size(M, 1);
    
    % 初始条件
    y0 = zeros(2 * n_dof, 1);  % [位移; 速度]
    
    % 确定激励点对应的DOF
    excitation_dof = getExcitationDOF(excitation_point, params);
    
    % ODE求解
    % 这里使用简化的线性求解作为示例
    % 完整实现应包含非线性处理
    
    ode_fun = @(t_val, y) simulationODE(t_val, y, M, K, C, params, ...
        excitation_signal, t, excitation_dof, sim_engine);
    
    % 选择求解器
    try
        [t_out, y_out] = ode45(ode_fun, [0, t_end], y0, ...
            sim_engine.solver_config.primary_options);
    catch
        % 如果ode45失败,尝试ode15s
        [t_out, y_out] = ode15s(ode_fun, [0, t_end], y0, ...
            sim_engine.solver_config.backup_options);
    end
    
    % 处理结果
    result.time = t_out;
    result.displacement = y_out(:, 1:n_dof);
    result.velocity = y_out(:, n_dof+1:end);
    
    % 计算加速度
    result.acceleration = zeros(size(result.velocity));
    for i = 1:length(t_out)
        dy = ode_fun(t_out(i), y_out(i, :)');
        result.acceleration(i, :) = dy(n_dof+1:end)';
    end
    
    % 模拟果实脱落 (简化版本)
    result.detachment_events = simulateFruitDetachment(...
        result, sim_engine.detachment_logic, params);
    
    % 计算输入能量
    result.input_energy = computeInputEnergy(...
        excitation_signal, result.velocity(:, excitation_dof), dt);
end


function dydt = simulationODE(t, y, M, K, C, params, excitation_signal, t_vec, exc_dof, sim_engine)
    % 仿真ODE函数
    
    n_dof = size(M, 1);
    
    % 分解状态变量
    u = y(1:n_dof);        % 位移
    v = y(n_dof+1:end);    % 速度
    
    % 插值获取当前激励力
    F_exc = interp1(t_vec, excitation_signal, t, 'linear', 0);
    
    % 构建力向量
    F = zeros(n_dof, 1);
    F(exc_dof) = F_exc;
    
    % 计算恢复力
    F_restore = zeros(n_dof, 1);
    
    % 根据各节点的非线性状态计算恢复力
    assembly_config = sim_engine.assembly_config;
    
    for i = 1:n_dof
        if i <= length(assembly_config.is_nonlinear) && assembly_config.is_nonlinear(i)
            % 非线性节点: 使用Duffing模型
            k_lin = K(i, i);
            c_lin = C(i, i);
            k3 = params.nonlinear.k3(min(i, length(params.nonlinear.k3)));
            c2 = params.nonlinear.c2(min(i, length(params.nonlinear.c2)));
            
            F_restore(i) = assembly_config.nonlinear_force_fn(u(i), v(i), k_lin, c_lin, k3, c2);
        else
            % 线性节点: 标准线性恢复力
            F_restore(i) = K(i, :) * u + C(i, :) * v;
        end
    end
    
    % 运动方程: M * a = F - F_restore
    a = M \ (F - F_restore);
    
    % 状态导数
    dydt = [v; a];
end


function signal = generateExcitationSignal(t, config)
    % 生成激励信号
    
    switch config.type
        case 'chirp'
            % 线性扫频信号
            signal = config.amplitude * chirp(t, config.freq_start, ...
                config.duration, config.freq_end);
            
        case 'sine'
            % 定频正弦
            freq = (config.freq_start + config.freq_end) / 2;
            signal = config.amplitude * sin(2 * pi * freq * t);
            
        case 'impulse'
            % 脉冲
            signal = zeros(size(t));
            impulse_idx = round(length(t) * 0.01);
            signal(1:impulse_idx) = config.amplitude;
            
        otherwise
            signal = zeros(size(t));
    end
end


function dof_idx = getExcitationDOF(excitation_point, params)
    % 获取激振点对应的DOF索引
    
    % 简化处理: 根据节点类型确定DOF
    switch excitation_point.node_type
        case 'root'
            dof_idx = 1;
        case 'mid'
            dof_idx = 2;
        case 'tip'
            dof_idx = 3;
        otherwise
            dof_idx = 1;
    end
end


%% =====================================================================
%% 【新增】果实脱落模拟
%% =====================================================================

function events = simulateFruitDetachment(sim_result, detachment_logic, params)
    % 模拟果实脱落过程
    
    events = struct();
    events.n_detached = 0;
    events.detachment_times = [];
    events.detachment_nodes = {};
    
    % 获取每个Tip节点的果实数量
    n_fruits = params.n_fruits;  % Root, Mid, Tip各代表一个区域
    
    % 为每个果实生成脱落阈值
    F_break = zeros(n_fruits, 1);
    for i = 1:n_fruits
        % 使用多因素回归模型
        H_crown = 1.5 + 0.5 * rand();  % 冠层高度 (m)
        P_rel = (i - 1) / (n_fruits - 1);  % 相对位置
        D_fruit = 2.5 + 0.5 * rand();  % 果实直径 (cm)
        S_crack = rand() > 0.7;  % 是否开裂
        
        F_break(i) = detachment_logic.detachment_model.beta0 + ...
            detachment_logic.detachment_model.beta1 * H_crown + ...
            detachment_logic.detachment_model.beta2 * P_rel + ...
            detachment_logic.detachment_model.beta3 * D_fruit + ...
            detachment_logic.detachment_model.beta4 * S_crack + ...
            detachment_logic.detachment_model.sigma_epsilon * randn();
        
        F_break(i) = max(2, F_break(i));  % 最小脱落力
    end
    
    % 遍历时间步,检测脱落
    fruit_detached = false(n_fruits, 1);
    m_fruit = params.m_fruit;  % 果实质量 (kg)
    gravity = 9.81;
    
    for t_idx = 1:length(sim_result.time)
        for fruit_idx = 1:n_fruits
            if fruit_detached(fruit_idx)
                continue;
            end
            
            % 获取果实节点的加速度
            if fruit_idx <= size(sim_result.acceleration, 2)
                accel = sim_result.acceleration(t_idx, fruit_idx);
            else
                continue;
            end
            
            % 计算茎杆力
            F_stem = detachment_logic.compute_stem_force(accel, m_fruit, gravity);
            
            % 检查是否达到脱落条件
            if detachment_logic.detachment_criterion(F_stem, F_break(fruit_idx))
                fruit_detached(fruit_idx) = true;
                events.n_detached = events.n_detached + 1;
                events.detachment_times(end+1) = sim_result.time(t_idx);
                events.detachment_nodes{end+1} = sprintf('Fruit_%d', fruit_idx);
            end
        end
    end
    
    events.total_fruits = n_fruits;
    events.detachment_rate = events.n_detached / n_fruits;
end


%% =====================================================================
%% 【新增】综合采收效能指数计算
%% =====================================================================

function J = computeHarvestingPerformanceIndex(sim_result, w1, w2, w3, T_sim, E_ref)
    % 计算综合采收效能指数 J_k
    %
    % 根据V3手稿公式:
    % J_k = w1 * (N_drop/N_total) + w2 * (1 - t_drop/T_sim) + w3 * (1 - E_in/E_ref)
    
    % 脱落率
    N_drop = sim_result.detachment_events.n_detached;
    N_total = sim_result.detachment_events.total_fruits;
    
    if N_total > 0
        drop_rate = N_drop / N_total;
    else
        drop_rate = 0;
    end
    
    % 平均脱落时间效率
    if ~isempty(sim_result.detachment_events.detachment_times)
        t_drop_mean = mean(sim_result.detachment_events.detachment_times);
        time_efficiency = 1 - t_drop_mean / T_sim;
    else
        time_efficiency = 0;  % 无脱落则时间效率为0
    end
    
    % 能量效率 (假设参考能量为最大能量)
    E_in = sim_result.input_energy;
    
    energy_efficiency = 1 - min(1, E_in / E_ref);
    
    % 综合评分
    J = w1 * drop_rate + w2 * max(0, time_efficiency) + w3 * max(0, energy_efficiency);
end


function E = computeInputEnergy(force_signal, velocity, dt)
    % 计算输入能量
    % E = ∫ F * v dt
    
    if length(force_signal) ~= length(velocity)
        min_len = min(length(force_signal), length(velocity));
        force_signal = force_signal(1:min_len);
        velocity = velocity(1:min_len);
    end
    
    power = force_signal(:) .* velocity(:);
    E = sum(abs(power)) * dt;
end


%% =====================================================================
%% 【新增】激振效能热力图生成
%% =====================================================================

function heatmap_data = generateExcitationHeatmap(candidate_points, scores, tree_topology)
    % 生成激振效能热力图数据
    %
    % 根据V3手稿:
    % "生成全树激振效能热力图(Excitation Performance Heatmap)"
    
    heatmap_data = struct();
    
    n_points = length(candidate_points);
    
    % 提取坐标和评分
    heatmap_data.point_ids = cell(n_points, 1);
    heatmap_data.scores = scores;
    heatmap_data.positions = zeros(n_points, 3);
    heatmap_data.branch_levels = zeros(n_points, 1);
    heatmap_data.node_types = cell(n_points, 1);
    
    for k = 1:n_points
        point = candidate_points{k};
        heatmap_data.point_ids{k} = point.id;
        heatmap_data.branch_levels(k) = point.branch_level;
        heatmap_data.node_types{k} = point.node_type;
        
        if isfield(point, 'position')
            heatmap_data.positions(k, :) = point.position;
        end
    end
    
    % 归一化评分用于颜色映射
    heatmap_data.scores_normalized = (scores - min(scores)) / (max(scores) - min(scores) + eps);
    
    % 按分枝层级组织数据
    heatmap_data.by_branch_level = struct();
    
    unique_levels = unique(heatmap_data.branch_levels);
    for i = 1:length(unique_levels)
        level = unique_levels(i);
        level_mask = heatmap_data.branch_levels == level;
        
        % 安全检查：确保有数据
        if any(level_mask)
            heatmap_data.by_branch_level.(sprintf('level_%d', level)) = struct(...
                'indices', find(level_mask), ...
                'mean_score', mean(scores(level_mask)), ...
                'max_score', max(scores(level_mask)));
        end
    end
end


%% =====================================================================
%% 【新增】识别最优激振区段
%% =====================================================================

function zone = identifyOptimalExcitationZone(candidate_points, scores)
    % 识别最优激振区段（更健壮的实现）
    % candidate_points: cell array，每个元素为 struct，包含 id, branch_level, node_type
    % scores: 数值向量，与 candidate_points 长度一致

    % ----- 基本检查 -----
    if ~iscell(candidate_points)
        error('candidate_points 必须是 cell 数组。');
    end
    if ~isvector(scores) || numel(scores) ~= numel(candidate_points)
        error('scores 必须是与 candidate_points 等长的向量。');
    end

    zone = struct();

    % 找出高评分点 (评分 > 平均分 + 0.5*标准差)
    mean_score = mean(scores);
    std_score = std(scores);
    threshold = mean_score + 0.5 * std_score;

    high_score_idx = find(scores >= threshold);
    if isempty(high_score_idx)
        [~, high_score_idx] = max(scores);
    end

    % 预分配
    nHigh = numel(high_score_idx);
    zone.high_score_points = cell(nHigh, 1);
    branch_levels = zeros(nHigh, 1);
    node_types = cell(nHigh, 1);

    for i = 1:nHigh
        idx = high_score_idx(i);
        point = candidate_points{idx};
        % 假设 point 有字段 id, branch_level, node_type
        zone.high_score_points{i} = point.id;
        branch_levels(i) = point.branch_level;
        node_types{i} = point.node_type;
    end

    % 确定主要区域（默认 trunk）
    if ~isempty(branch_levels) && mode(branch_levels) == 1
        zone.primary_level = 'primary_branch';
    else
        zone.primary_level = 'trunk';
    end

    % 确定主要节点类型（root/mid/tip）
    types = {'root', 'mid', 'tip'};   % <- 先赋值到变量，避免直接字面量后接 {...} 引发的检查错误
    type_counts.root = sum(strcmp(node_types, 'root'));
    type_counts.mid  = sum(strcmp(node_types, 'mid'));
    type_counts.tip  = sum(strcmp(node_types, 'tip'));

    % 找到最大类型索引（1=root,2=mid,3=tip）
    counts_vec = [type_counts.root, type_counts.mid, type_counts.tip];
    [~, max_type] = max(counts_vec);
    % 额外保护：确保 max_type 在 1..3
    if isempty(max_type) || ~isscalar(max_type) || max_type < 1 || max_type > 3
        max_type = 1;
    end

    zone.primary_node_type = types{max_type};

    % 生成描述：保证索引不越界
    next_idx = min(max_type + 1, numel(types));
    zone.description = sprintf('一级分枝 %s 至 %s 区域', ...
        zone.primary_node_type, ...
        types{next_idx});

    zone.score_threshold = threshold;
    zone.n_optimal_points = nHigh;
end

% 安全读取工作区变量
function value = safeGetWorkspaceVar(varName, defaultValue)
    % 安全地从工作区获取变量
    if evalin('base', sprintf('exist(''%s'', ''var'')', varName))
        value = evalin('base', varName);
    else
        value = defaultValue;
    end
end


% 读取识别参数并分配到分枝
function params_with_identified = applyIdentifiedParams(params_struct, identified_params)
    % 将识别的刚度阻尼应用到各分枝参数
    
    params_with_identified = params_struct;
    
    if isempty(identified_params) || ~isfield(identified_params, 'linear')
        warning('未提供识别参数，将使用估算值');
        return;
    end
    
    % 提取识别的矩阵
    K = identified_params.linear.K;
    C = identified_params.linear.C;
    
    % 更新主干参数
    if isfield(params_with_identified, 'trunk')
        trunk = params_with_identified.trunk;
        
        % 假设K和C的对角元素对应root/mid/tip
        if size(K,1) >= 3
            trunk.root.k_y_conn = K(1,1);
            trunk.mid.k_y_conn = K(2,2);
            trunk.tip.k_y_conn = K(3,3);
            
            trunk.root.c_y_conn = C(1,1);
            trunk.mid.c_y_conn = C(2,2);
            trunk.tip.c_y_conn = C(3,3);
        end
        
        params_with_identified.trunk = trunk;
    end
    
    fprintf('  已将识别的刚度阻尼参数应用到主干\n');
end