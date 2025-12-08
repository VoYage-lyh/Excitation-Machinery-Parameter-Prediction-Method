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
fprintf('║    果树振动分析与仿真集成系统 v2.0                     ║\n');
fprintf('║    (正确工作流程版本)                                  ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n\n');

%% ===================================================================
%% 第一步：GUI预配置
%% ===================================================================
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

% 保存预配置以备后用
save('tree_preconfig_latest.mat', 'preConfig');
fprintf('  预配置已保存到: tree_preconfig_latest.mat\n\n');

%% ===================================================================
%% 第二步：参数识别
%% ===================================================================
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('第二步：参数识别\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

identifyChoice = questdlg('请选择参数识别方式:', '参数识别', ...
                          '运行参数识别', '加载已有识别结果', '跳过（使用估算值）', ...
                          '加载已有识别结果');

identifiedParams = [];

switch identifyChoice
    case '运行参数识别'
        identifiedParams = runParameterIdentification(preConfig);
        
    case '加载已有识别结果'
        identifiedParams = loadIdentifiedParams();
        
    case '跳过（使用估算值）'
        fprintf('  将使用基于几何的估算值替代识别参数\n');
        fprintf('  警告：估算值可能与实际值有较大差异！\n\n');
end

%% ===================================================================
%% 第三步：生成仿真参数
%% ===================================================================
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('第三步：生成仿真参数\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

[analysis_params, sim_params] = ConfigAdapter(preConfig, identifiedParams);

% 保存完整仿真参数
save('simulation_params_complete.mat', 'sim_params', 'preConfig', 'identifiedParams');
fprintf('  完整仿真参数已保存到: simulation_params_complete.mat\n\n');

%% ===================================================================
%% 第四步：运行仿真（可选）
%% ===================================================================
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('第四步：运行仿真\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

simChoice = questdlg('是否立即运行仿真?', '运行仿真', ...
                     '是，运行仿真', '否，稍后手动运行', '否，稍后手动运行');

if strcmp(simChoice, '是，运行仿真')
    runSimulation(sim_params);
else
    fprintf('  仿真参数已准备就绪。\n');
    fprintf('  运行 Build_Extended_MDOF_model 前请确保参数已正确导出。\n\n');
    
    % 【关键】调用导出函数，确保所有参数都正确导出到工作区
    exportSimParamsToWorkspace(sim_params);
    
    % 额外导出这些变量供调试使用
    assignin('base', 'sim_params', sim_params);
    assignin('base', 'preConfig', preConfig);
    assignin('base', 'identifiedParams', identifiedParams);
end

%% ===================================================================
%% 完成
%% ===================================================================
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
    % 运行参数识别流程
    
    fprintf('\n--- 开始参数识别流程 ---\n');
    fprintf('  将打开参数识别代码...\n\n');
    
    % 生成分析参数
    [analysis_params, ~] = ConfigAdapter_v2(preConfig, []);
    
    % 将参数导出到工作区供识别代码使用
    assignin('base', 'analysis_params', analysis_params);
    assignin('base', 'fs_target', analysis_params.fs_target);
    assignin('base', 'CUTOFF_FREQ', analysis_params.cutoff_freq);
    assignin('base', 'FILTER_ORDER', analysis_params.filter_order);
    assignin('base', 'nfft', analysis_params.nfft);
    assignin('base', 'freq_range', analysis_params.freq_range);
    assignin('base', 'SNR_THRESHOLD', analysis_params.snr_threshold);
    
    fprintf('  分析参数已设置到工作区:\n');
    fprintf('    fs_target = %d Hz\n', analysis_params.fs_target);
    fprintf('    CUTOFF_FREQ = %d Hz\n', analysis_params.cutoff_freq);
    fprintf('    freq_range = [%d, %d] Hz\n', analysis_params.freq_range(1), analysis_params.freq_range(2));
    fprintf('    SNR_THRESHOLD = %d dB\n', analysis_params.snr_threshold);
    fprintf('\n');
    
    % 提示用户
    msgbox(sprintf(['参数已设置到工作区。\n\n' ...
                    '请按以下步骤操作：\n' ...
                    '1. 运行 analyse_chibi_data_v8.m（或其修改版）\n' ...
                    '2. 完成锤击数据选择和标注\n' ...
                    '3. 等待参数识别完成\n' ...
                    '4. 识别结果将保存到 .mat 文件\n\n' ...
                    '完成后点击"加载识别结果"按钮继续。']), ...
           '参数识别提示', 'help');
    
    % 等待用户完成识别
    waitChoice = questdlg('参数识别是否已完成?', '等待识别完成', ...
                          '已完成，加载结果', '取消', '已完成，加载结果');
    
    if strcmp(waitChoice, '已完成，加载结果')
        identifiedParams = loadIdentifiedParams();
    else
        identifiedParams = [];
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
    % 运行仿真
    
    fprintf('\n--- 开始仿真流程 ---\n');
    
    % 切换工作目录
    if ~isempty(sim_params.workFolder) && exist(sim_params.workFolder, 'dir')
        cd(sim_params.workFolder);
        fprintf('  工作目录: %s\n', sim_params.workFolder);
    end
    
    % 导出所有仿真参数到工作区
    exportSimParamsToWorkspace(sim_params);
    
    fprintf('  仿真参数已导出到工作区。\n');
    fprintf('  请运行 Build_Extended_MDOF_model 代码（或其修改版）。\n\n');
    
    % 提示
    msgbox(sprintf(['仿真参数已准备就绪!\n\n' ...
                    '请运行 Build_Extended_MDOF_model_best_version_v2.m\n' ...
                    '（需要先按照修改指南进行修改）\n\n' ...
                    '关键修改点:\n' ...
                    '• 从工作区读取参数而非硬编码\n' ...
                    '• 根据新的果实配置逻辑添加果实']), ...
           '运行仿真', 'help');
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
