%% 果树振动分析与仿真 - 统一集成脚本 v2.0
% =========================================================================
% 正确的工作流程：
%   第一步：GUI预配置 -> 拓扑、几何、质量、仿真参数（不含刚度阻尼）
%   第二步：参数识别 -> 从实验数据识别刚度、阻尼、非线性参数
%   第三步：仿真 -> 读取预配置 + 识别结果
%
% 关键修正：
%   - 刚度(k)和阻尼(c)由参数识别代码从实验数据获取，不再手动输入
%   - 果实配置：二级和三级分枝的mid和tip都可挂果
% =========================================================================

clear; clc; close all;

fprintf('╔═══════════════════════════════════════════════════════╗\n');
fprintf('║    果树振动分析与仿真集成系统                          ║\n');
fprintf('║    (正确工作流程版本)                                  ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n\n');

%% ===================================================================
%% 第一步：GUI预配置
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('第一步：打开预配置界面\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('  配置内容：拓扑结构、几何参数、质量参数、果实配置、仿真参数\n');
fprintf('  注意：刚度和阻尼将由后续的参数识别步骤获取！\n\n');

preConfig = BranchConfigGUI();

if isempty(preConfig)
    fprintf('用户取消配置，程序退出。\n');
    return;
end

fprintf('[√] 预配置完成!\n\n');

% 使用项目名称生成文件名
fname_config = sprintf('%s_PreConfig.mat', preConfig.basic.projectName);
save(fname_config, 'preConfig');
fprintf('  预配置已保存到: %s\n', fname_config);

%% ===================================================================
%% 第二步：参数识别
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('第二步：参数识别\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

identifyChoice = questdlg('请选择参数识别方式 (必须基于真实数据):', '参数识别', ...
                          '运行参数识别 (新实验)', '加载已有识别结果 (MAT文件)', ...
                          '加载已有识别结果 (MAT文件)');

identifiedParams = [];

switch identifyChoice
    case '运行参数识别'
        identifiedParams = runParameterIdentification(preConfig);
        
    case '加载已有识别结果'
        identifiedParams = loadIdentifiedParams();
        
    case '跳过（使用估算值）'
        error('违反原则：严禁使用估算值跳过参数识别。所有仿真必须基于实验数据。');
end

%% ===================================================================
%% 第三步：生成仿真参数
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('第三步：生成仿真参数\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

[analysis_params, sim_params] = ConfigAdapter(preConfig, identifiedParams);

% 保存完整仿真参数
fname_sim = sprintf('%s_SimParams.mat', preConfig.basic.projectName);
save(fname_sim, 'sim_params', 'preConfig', 'identifiedParams');
fprintf('  完整仿真参数已保存到: simulation_params_complete.mat\n\n');

%% ===================================================================
%% 第四步：运行仿真
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('第四步：运行仿真\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

fprintf('  正在启动自动化仿真流程...\n');

% 1. 调用自动化仿真函数
% 注意：修改后的 runSimulation 函数内部已经包含了 exportSimParamsToWorkspace
% 并且会在导出参数后自动调用 Build_Extended_MDOF_model.m
runSimulation(sim_params);

% 2. 确保变量驻留在工作区供调试
% 虽然 runSimulation 已经导出了仿真参数，但这几行可以确保 preConfig 等
% 中间变量也能在工作区找到，方便后续手动检查。
assignin('base', 'sim_params', sim_params);
assignin('base', 'preConfig', preConfig);
if exist('identifiedParams', 'var')
    assignin('base', 'identifiedParams', identifiedParams);
end

fprintf('  [√] 第四步执行完毕。\n\n');

%% ===================================================================
%% 完成
fprintf('╔═══════════════════════════════════════════════════════╗\n');
fprintf('║    流程完成!                                          ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n\n');

fprintf('文件清单:\n');
fprintf('  • tree_preconfig_latest.mat - 预配置参数\n');
fprintf('  • simulation_params_complete.mat - 完整仿真参数\n');
if ~isempty(identifiedParams)
    fprintf('  • IdentifiedParameters_*.mat - 识别参数\n');
end
fprintf('\n');

%% ==================== 子函数 ====================
function identifiedParams = runParameterIdentification(preConfig)
    % runParameterIdentification - 全拓扑参数识别流程 (严格数据驱动 + 自动化闭环版)
    %
    % 功能：
    %   1. 遍历拓扑结构中的每一个物理分枝 (P1, P1_S1...)。
    %   2. 强制用户为每个分枝明确指定实验数据文件 (Root/Mid/Tip/Force)。
    %   3. 自动调用 analyse_chibi_data.m 进行参数识别。
    %   4. 捕获识别结果并组装成统一的 identifiedParams 结构体。
    %
    % 输入：
    %   preConfig - 预配置结构体 (包含 topology 和 basic 信息)
    %
    % 输出：
    %   identifiedParams - 识别完成的参数大结构体
    
    fprintf('\n========================================\n');
    fprintf('   全拓扑参数识别流程 (严格数据驱动)\n');
    fprintf('========================================\n');
    
    % 1. 生成所有分枝的 ID 列表
    all_branch_ids = getAllBranchIDs(preConfig.topology);
    
    fprintf('系统检测到拓扑结构包含 %d 个独立分枝。\n', length(all_branch_ids));
    fprintf('原则：一个分枝对应一组独立实验数据，严禁缺省或使用默认值。\n');
    
    % 强制全选，不给用户跳过的机会
    target_branches = all_branch_ids; 
    
    % 准备基础结构
    identifiedParams = struct();
    identifiedParams.is_multi_branch = true;
    identifiedParams.branches = struct();
    identifiedParams.global_linear = []; 
    
    % 准备分析参数 (传递给识别脚本使用)
    [analysis_params, ~] = ConfigAdapter(preConfig, []);
    assignin('base', 'analysis_params', analysis_params); 
    
    % 获取项目名称用于日志或保存 (如果 GUI 中未配置，则使用默认值)
    if isfield(preConfig.basic, 'projectName')
        proj_name = preConfig.basic.projectName;
    else
        proj_name = 'Project';
    end
    
    % 2. 循环处理每一个分枝
    for i = 1:length(target_branches)
        branch_name = target_branches{i};
        fprintf('\n-----------------------------------------------\n');
        fprintf('  >>> 进度 [%d/%d] | 当前处理分枝: %s <<<\n', i, length(target_branches), branch_name);
        fprintf('-----------------------------------------------\n');
        
        % === 步骤 A: 获取实验数据路径 (严格显式指定) ===
        % 提示：为了减少手动点击，您可以将此处改为根据命名规则自动搜索文件
        % 例如: root_file = fullfile(data_dir, [branch_name, '_Root.csv']);
        
        fprintf('  请选择【%s】的实验数据文件 (共4个文件)...\n', branch_name);
        
        % 1. 根部传感器数据
        [f_root, p_root] = uigetfile('*.csv', sprintf('[%s] 1/4: 选择 根部(Root) 传感器数据', branch_name));
        if isequal(f_root, 0), error('流程终止：必须提供根部数据'); end
        
        % 2. 中部传感器数据
        [f_mid, p_mid] = uigetfile('*.csv', sprintf('[%s] 2/4: 选择 中部(Mid) 传感器数据', branch_name));
        if isequal(f_mid, 0), error('流程终止：必须提供中部数据'); end
        
        % 3. 顶部传感器数据
        [f_tip, p_tip] = uigetfile('*.csv', sprintf('[%s] 3/4: 选择 顶部(Tip) 传感器数据', branch_name));
        if isequal(f_tip, 0), error('流程终止：必须提供顶部数据'); end
        
        % 4. 力锤数据
        [f_force, p_force] = uigetfile('*.csv;*.xls;*.xlsx', sprintf('[%s] 4/4: 选择 力锤(Force) 数据', branch_name));
        if isequal(f_force, 0), error('流程终止：必须提供力锤数据'); end
        
        % === 步骤 B: 构建自动加载配置 ===
        % 将路径打包，传递给 analyse_chibi_data.m
        current_data_config = struct();
        current_data_config.root_file = fullfile(p_root, f_root);
        current_data_config.mid_file = fullfile(p_mid, f_mid);
        current_data_config.tip_file = fullfile(p_tip, f_tip);
        current_data_config.force_file = fullfile(p_force, f_force);
        
        % 关键：推送到 Base Workspace，触发识别脚本的“无头模式”
        assignin('base', 'auto_load_config', current_data_config);
        
        try
            % === 步骤 C: 运行识别脚本 ===
            % 清理当前工作区可能残留的结果变量
            if exist('identified_params', 'var'), clear identified_params; end
            
            fprintf('  正在运行核心识别算法 (analyse_chibi_data.m)...\n');
            
            % 运行脚本。注意：脚本将在当前函数的工作区执行。
            run('analyse_chibi_data.m'); 
            
            % === 步骤 D: 捕获结果 (修正作用域问题) ===
            % 直接检查当前工作区，而不是 Base 工作区
            if exist('identified_params', 'var')
                current_result = identified_params;
                
                % 验证结果有效性
                if isempty(current_result)
                    error('识别脚本返回了空结果，请检查数据质量。');
                end
                
                % === 可视化确认环节 ===
                % 让用户快速确认一下拟合图像，保证质量
                btn = questdlg(sprintf('【%s】参数识别完成。\n请检查生成的图表（如FRF拟合、非线性回归）。\n结果是否合格？', branch_name), ...
                               '质量确认', ...
                               '合格，保存并继续', '不合格，重试', '合格，保存并继续');
                
                if strcmp(btn, '不合格，重试')
                    fprintf('  [!] 用户标记结果不合格，正在重置当前分枝...\n');
                    i = i - 1; % 回退索引，重新处理当前分枝
                    continue;
                elseif isempty(btn) || strcmp(btn, '取消')
                    error('用户取消流程');
                end
                
                % === 步骤 E: 保存数据 ===
                identifiedParams.branches.(branch_name) = current_result;
                fprintf('  [√] 分枝 %s 参数已成功保存。\n', branch_name);
                
                % === 清理工作区，准备下一轮 ===
                close all;  % 关闭所有图表
                clear identified_params; % 清除局部变量
                evalin('base', 'clear auto_load_config'); % 清除 Base 中的配置，防止误读
                
            else
                % 这是一个严重的逻辑错误，说明脚本跑完了但没生成变量
                error('严重错误：脚本运行结束，但在当前工作区未找到 "identified_params" 变量。\n请检查 analyse_chibi_data.m 是否正确执行了赋值操作。');
            end
            
        catch ME
            % 错误处理：提供详细信息
            errordlg(sprintf('处理分枝 %s 时发生错误:\n%s', branch_name, ME.message), '识别中断');
            rethrow(ME); % 在控制台打印完整堆栈，便于调试
        end
    end
    
    % 3. 保存最终总结果
    % 使用项目名称生成文件名，避免覆盖
    file_name = sprintf('%s_IdentifiedParams.mat', proj_name);
    save_path = fullfile(pwd, file_name);
    
    save(save_path, 'identifiedParams');
    assignin('base', 'identifiedParams', identifiedParams); % 同时也留在 Base 工作区供后续步骤使用
    
    fprintf('\n========================================\n');
    fprintf('   全部分枝参数识别任务圆满完成！\n');
    fprintf('   识别结果已保存至: %s\n', save_path);
    fprintf('========================================\n');
end

% --- 辅助函数：递归生成所有分枝ID ---
function ids = getAllBranchIDs(topo)
    ids = {'Trunk'};
    
    % 一级
    for p = 1:topo.num_primary_branches
        p_id = sprintf('P%d', p);
        ids{end+1} = p_id;
        
        % 二级
        if p <= length(topo.secondary_branches_count)
            num_s = topo.secondary_branches_count(p);
            for s = 1:num_s
                s_id = sprintf('%s_S%d', p_id, s);
                ids{end+1} = s_id;
                
                % 三级
                if p <= length(topo.tertiary_branches_count) && ...
                   s <= length(topo.tertiary_branches_count{p})
                    num_t = topo.tertiary_branches_count{p}(s);
                    for t = 1:num_t
                        ids{end+1} = sprintf('%s_T%d', s_id, t);
                    end
                end
            end
        end
    end
end

function identifiedParams = loadIdentifiedParams()
    % 加载已有的识别结果
    
    fprintf('  正在加载识别结果...\n');
    
    % 尝试自动查找最新的识别文件
    matFiles = dir('*dentified*.mat');
    
    if isempty(matFiles)
        % 手动选择
        [filename, pathname] = uigetfile('*.mat', '选择识别结果文件');
        if filename == 0
            fprintf('  未选择文件，将使用估算值。\n');
            identifiedParams = [];
            return;
        end
        filepath = fullfile(pathname, filename);
    else
        % 选择最新的文件
        [~, idx] = max([matFiles.datenum]);
        filepath = matFiles(idx).name;
        
        useLatest = questdlg(sprintf('找到识别文件: %s\n使用此文件?', filepath), ...
                             '确认文件', '是', '选择其他文件', '是');
        if strcmp(useLatest, '选择其他文件')
            [filename, pathname] = uigetfile('*.mat', '选择识别结果文件');
            if filename == 0
                identifiedParams = [];
                return;
            end
            filepath = fullfile(pathname, filename);
        end
    end
    
    try
        S = load(filepath);
        
        % 尝试不同的字段名
        if isfield(S, 'identified_params')
            identifiedParams = S.identified_params;
        elseif isfield(S, 'identifiedParams')
            identifiedParams = S.identifiedParams;
        elseif isfield(S, 'linear_params')
            identifiedParams.linear = S.linear_params;
            if isfield(S, 'nonlinear_params')
                identifiedParams.nonlinear = S.nonlinear_params;
            end
        else
            % 假设整个结构就是识别结果
            identifiedParams = S;
        end
        
        fprintf('  [√] 识别结果加载成功!\n');
        
        % 显示加载的内容
        if isfield(identifiedParams, 'linear')
            if isfield(identifiedParams.linear, 'natural_freqs_x')
                fprintf('    固有频率(X): [%s] Hz\n', ...
                        num2str(identifiedParams.linear.natural_freqs_x', '%.2f '));
            end
            if isfield(identifiedParams.linear, 'damping_ratios_x')
                fprintf('    阻尼比(X): [%s]\n', ...
                        num2str(identifiedParams.linear.damping_ratios_x', '%.3f '));
            end
        end
        fprintf('\n');
        
    catch ME
        fprintf('  加载失败: %s\n', ME.message);
        identifiedParams = [];
    end
end

function runSimulation(sim_params)
    % 运行仿真 - 自动化版本
    
    fprintf('\n========================================\n');
    fprintf('   进入自动化仿真构建阶段\n');
    fprintf('========================================\n');
    
    % 1. 切换工作目录 (保持不变)
    if ~isempty(sim_params.workFolder) && exist(sim_params.workFolder, 'dir')
        cd(sim_params.workFolder);
        fprintf('  工作目录已切换至: %s\n', sim_params.workFolder);
    end
    
    % 2. 导出所有仿真参数到工作区 (核心步骤)
    % 这一步非常关键，它会将 config, params_struct 等变量推送到 Base Workspace
    % Build_Extended_MDOF_model.m 将直接从 Base Workspace 读取这些变量
    exportSimParamsToWorkspace(sim_params);
    
    fprintf('  [√] 仿真参数已成功导出到工作区。\n');
    fprintf('  正在启动模型构建与仿真脚本 (Build_Extended_MDOF_model.m)...\n\n');
    
    % 3. 自动运行构建和仿真脚本 (修改点)
    try
        % 检查脚本是否存在
        if exist('Build_Extended_MDOF_model.m', 'file') ~= 2
            error('MATLAB:FileNotFound', ...
                '未找到 "Build_Extended_MDOF_model.m" 文件，无法自动运行。');
        end
        
        % === 自动执行 ===
        run('Build_Extended_MDOF_model.m');
        
        fprintf('\n========================================\n');
        fprintf('   全流程自动化执行完毕！\n');
        fprintf('========================================\n');
        
    catch ME
        errordlg(sprintf('自动运行仿真失败:\n%s', ME.message), '执行错误');
        fprintf(2, '错误详情:\n%s\n', ME.getReport());
    end
end

function exportSimParamsToWorkspace(params)
    % 将仿真参数导出到工作区 - 严格模式
    % 所有参数必须存在，缺失则报错
    
    fprintf('  正在导出参数到工作区（严格模式）...\n');
    
    % ========== 标记使用GUI配置 ==========
    assignin('base', 'params_from_gui', true);
    
    % ========== 模型名称 ==========
    if ~isfield(params, 'model_name') || isempty(params.model_name)
        error('ExportParams:MissingData', '缺少模型名称(model_name)');
    end
    assignin('base', 'model_name', params.model_name);
    
    % ========== 重力加速度 ==========
    if ~isfield(params, 'gravity_g')
        error('ExportParams:MissingData', '缺少重力加速度(gravity_g)');
    end
    assignin('base', 'gravity_g', params.gravity_g);
    
    % ========== 并行计算设置 ==========
    if ~isfield(params, 'use_parallel')
        error('ExportParams:MissingData', '缺少并行计算设置(use_parallel)');
    end
    assignin('base', 'use_parallel', params.use_parallel);
    
    if ~isfield(params, 'parallel_max_workers')
        error('ExportParams:MissingData', '缺少并行Worker数设置(parallel_max_workers)');
    end
    assignin('base', 'parallel_execution_max_workers', params.parallel_max_workers);
    
    % ========== 工作目录 ==========
    if isfield(params, 'workFolder') && ~isempty(params.workFolder)
        assignin('base', 'workFolder', params.workFolder);
    end
    
    % ========== 拓扑配置 ==========
    if ~isfield(params, 'config') || isempty(params.config)
        error('ExportParams:MissingData', '缺少拓扑配置(config)');
    end
    assignin('base', 'config', params.config);
    
    % ========== 主干参数 ==========
    if ~isfield(params, 'trunk') || isempty(params.trunk)
        error('ExportParams:MissingData', '缺少主干参数(trunk)');
    end
    params_struct_export = struct();
    params_struct_export.trunk = params.trunk;
    assignin('base', 'params_struct', params_struct_export);
    
    % ========== 分枝参数 ==========
    if ~isfield(params, 'predefined_params') || isempty(params.predefined_params)
        error('ExportParams:MissingData', '缺少分枝参数(predefined_params)');
    end
    assignin('base', 'predefined_params', params.predefined_params);
    
    % ========== 果实配置 ==========
    if ~isfield(params, 'fruit_config') || isempty(params.fruit_config)
        error('ExportParams:MissingData', '缺少果实配置(fruit_config)');
    end
    assignin('base', 'fruit_config', params.fruit_config);
    
    if ~isfield(params, 'default_fruit_params') || isempty(params.default_fruit_params)
        error('ExportParams:MissingData', '缺少果实参数(default_fruit_params)');
    end
    assignin('base', 'default_fruit_params', params.default_fruit_params);
    
    % ========== 仿真时间参数 ==========
    if ~isfield(params, 'sim_stop_time')
        error('ExportParams:MissingData', '缺少仿真停止时间(sim_stop_time)');
    end
    assignin('base', 'sim_stop_time', params.sim_stop_time);
    
    if ~isfield(params, 'sim_fixed_step')
        error('ExportParams:MissingData', '缺少仿真步长(sim_fixed_step)');
    end
    assignin('base', 'sim_fixed_step', params.sim_fixed_step);
    
    % ========== 激励参数 ==========
    if ~isfield(params, 'excitation') || isempty(params.excitation)
        error('ExportParams:MissingData', '缺少激励参数(excitation)');
    end
    exc = params.excitation;
    
    required_exc_fields = {'type', 'sine_amplitude_y', 'sine_amplitude_z', ...
                           'frequency_hz', 'phase_y_rad', 'phase_z_rad', ...
                           'impulse_gain_y', 'impulse_gain_z', 'pulse_period_s', ...
                           'pulse_width_percent', 'pulse_delay_y_s', 'pulse_delay_z_s', ...
                           'start_time', 'end_time'};
    for i = 1:length(required_exc_fields)
        if ~isfield(exc, required_exc_fields{i})
            error('ExportParams:MissingData', '激励参数缺少字段: %s', required_exc_fields{i});
        end
    end
    
    assignin('base', 'excitation_type', exc.type);
    assignin('base', 'F_excite_y_amplitude', exc.sine_amplitude_y);
    assignin('base', 'F_excite_z_amplitude', exc.sine_amplitude_z);
    assignin('base', 'excitation_frequency_hz', exc.frequency_hz);
    assignin('base', 'excitation_phase_y_rad', exc.phase_y_rad);
    assignin('base', 'excitation_phase_z_rad', exc.phase_z_rad);
    assignin('base', 'impulse_force_gain_y', exc.impulse_gain_y);
    assignin('base', 'impulse_force_gain_z', exc.impulse_gain_z);
    assignin('base', 'pulse_period_s', exc.pulse_period_s);
    assignin('base', 'pulse_width_percent', exc.pulse_width_percent);
    assignin('base', 'pulse_phase_delay_y_s', exc.pulse_delay_y_s);
    assignin('base', 'pulse_phase_delay_z_s', exc.pulse_delay_z_s);
    assignin('base', 'excitation_start_time', exc.start_time);
    assignin('base', 'excitation_end_time', exc.end_time);
    
    % ========== 递减因子（从识别结果） ==========
    if isfield(params, 'taper_factors') && ~isempty(params.taper_factors)
        assignin('base', 'identified_taper_factors', params.taper_factors);
    end
    
    fprintf('  参数导出完成。\n');
end
