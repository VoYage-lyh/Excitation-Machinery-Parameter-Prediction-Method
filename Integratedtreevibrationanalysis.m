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

% 保存预配置以备后用
save('tree_preconfig_latest.mat', 'preConfig');
fprintf('  预配置已保存到: tree_preconfig_latest.mat\n');

%% ===================================================================
%% 第二步：参数识别
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
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('第三步：生成仿真参数\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

[analysis_params, sim_params] = ConfigAdapter(preConfig, identifiedParams);

% 保存完整仿真参数
save('simulation_params_complete.mat', 'sim_params', 'preConfig', 'identifiedParams');
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
    % 运行参数识别流程 (全拓扑覆盖 + 可视化确认版)
    
    fprintf('\n--- 开始全拓扑参数识别流程 ---\n');
    
    % 1. 生成所有分枝的 ID 列表 (Trunk, P1, P1_S1, ...)
    all_branch_ids = getAllBranchIDs(preConfig.topology);
    
    % 2. 让用户选择要处理的分枝
    [indx, tf] = listdlg('ListString', all_branch_ids, ...
                         'SelectionMode', 'multiple', ...
                         'ListSize', [300, 400], ...
                         'Name', '选择识别对象', ...
                         'PromptString', '请选择拥有独立实验数据的分枝 (可多选):');
    
    if ~tf, identifiedParams = []; return; end
    target_branches = all_branch_ids(indx);
    
    % 准备基础结构
    identifiedParams = struct();
    identifiedParams.is_multi_branch = true;
    identifiedParams.branches = struct();
    identifiedParams.global_linear = []; 
    
    % 准备分析参数
    [analysis_params, ~] = ConfigAdapter(preConfig, []);
    assignin('base', 'analysis_params', analysis_params); 
    
    % 用于计算全局平均值的累加器
    temp_accum = struct('K', 0, 'C', 0, 'count', 0);
    
    % 3. 循环处理每一个分枝
    for i = 1:length(target_branches)
        branch_name = target_branches{i};
        fprintf('\n═══════════════════════════════════════════════\n');
        fprintf('  >>> 正在处理: %s (%d/%d) <<<\n', branch_name, i, length(target_branches));
        fprintf('═══════════════════════════════════════════════\n');
        
        % 提示加载数据
        uiwait(msgbox(sprintf('请准备好【%s】的实验数据。\n点击确定选择文件...', branch_name), '数据加载提示'));
        
        try
            % 运行识别脚本 (注意：analyse_chibi_data.m 必须已去除 clear)
            run('analyse_chibi_data.m');
            
            % 捕获结果
            if evalin('base', 'exist(''identified_params'', ''var'')')
                current_result = evalin('base', 'identified_params');
                
                % === 可视化确认环节 ===
                % 此时 analyse_chibi_data 的图表仍然打开着
                % 弹出一个模态对话框，暂停程序，直到用户点击按钮
                btn = questdlg(sprintf('【%s】参数识别完成。\n请检查屏幕上的图表 (FRF, 相干性, 拟合曲线)。\n\n结果是否合格？', branch_name), ...
                               '质量确认', ...
                               '合格，保存并继续', '不合格，重试', '跳过此分枝', '合格，保存并继续');
                
                if strcmp(btn, '不合格，重试')
                    fprintf('  用户选择重试当前分枝...\n');
                    i = i - 1; % 回退索引
                    continue;
                elseif strcmp(btn, '跳过此分枝')
                    fprintf('  已跳过 %s。\n', branch_name);
                    continue;
                elseif isempty(btn)
                    error('用户取消流程');
                end
                
                % === 保存数据 ===
                identifiedParams.branches.(branch_name) = current_result;
                fprintf('  [√] %s 参数已保存。\n', branch_name);
                
                % 累加用于全局平均
                if isfield(current_result, 'linear') && isfield(current_result.linear, 'K')
                    % 仅取对角线元素(Root/Mid/Tip)进行粗略平均，用于无数据分枝的回退
                    k_diag = diag(current_result.linear.K);
                    c_diag = diag(current_result.linear.C);
                    % 如果是3x3矩阵才累加
                    if length(k_diag) == 3
                        temp_accum.K = temp_accum.K + k_diag;
                        temp_accum.C = temp_accum.C + c_diag;
                        temp_accum.count = temp_accum.count + 1;
                    end
                end
                
                % 关闭当前图表，准备下一个
                close all; 
            else
                warning('脚本运行结束但未发现 identified_params 变量。');
            end
            
        catch ME
            errordlg(sprintf('处理 %s 时出错: %s', branch_name, ME.message), '错误');
            return;
        end
    end
    
    % 4. 生成全局回退参数 (Global Fallback)
    if temp_accum.count > 0
        % 构造一个对角矩阵作为平均值
        avg_K_diag = temp_accum.K / temp_accum.count;
        avg_C_diag = temp_accum.C / temp_accum.count;
        
        identifiedParams.linear = struct();
        identifiedParams.linear.K = diag(avg_K_diag);
        identifiedParams.linear.C = diag(avg_C_diag);
        % 这里还可以计算 taper_factors 的平均值
        fprintf('\n  [√] 全部分枝处理完毕。已基于 %d 组数据生成全局基准参数。\n', temp_accum.count);
    else
        % 如果一个都没识别成功，必须报错
        error('未获得任何有效的识别参数，无法继续。');
    end
    
    % 保存总结果
    save('IdentifiedParams_FullTopology.mat', 'identifiedParams');
    assignin('base', 'identifiedParams', identifiedParams); % 确保留在工作区
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
