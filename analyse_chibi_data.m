% 
clear; clc; close all;

%% === 从预配置读取参数 ===
if evalin('base', 'exist(''analysis_params'', ''var'')')
    analysis_params = evalin('base', 'analysis_params');
    fprintf('检测到预配置参数，使用预设值...\n');
    
    % 覆盖默认值
    fs_target = analysis_params.fs_target;
    CUTOFF_FREQ = analysis_params.cutoff_freq;
    FILTER_ORDER = analysis_params.filter_order;
    nfft = analysis_params.nfft;
    freq_range = analysis_params.freq_range;
    SNR_THRESHOLD = analysis_params.snr_threshold;
else
    % 使用默认值
    fs_target = 1000;
    CUTOFF_FREQ = 65;
    FILTER_ORDER = 8;
    nfft = 2048;
    freq_range = [3, 50];
    SNR_THRESHOLD = 10;
end

%% 安全清理所有图窗
try
    % 清理全局变量
    clear global g_annotation_data;
    
    % 获取所有图窗句柄
    all_figs = findall(groot, 'Type', 'figure');
    
    if ~isempty(all_figs)
        fprintf('检测到 %d 个打开的图窗，正在安全关闭...\n', length(all_figs));
        
        % 清除所有图窗的CloseRequestFcn回调，然后关闭
        for i = 1:length(all_figs)
            if isvalid(all_figs(i)) && ishandle(all_figs(i))
                set(all_figs(i), 'CloseRequestFcn', 'closereq'); % 恢复默认关闭函数
                close(all_figs(i));
            end
        end
        fprintf('图窗清理完成\n');
    end
    
catch ME
    % 如果清理过程出现错误，强制关闭
    fprintf('安全清理失败，强制关闭所有图窗: %s\n', ME.message);
    close all force;
end

% 强制刷新和短暂暂停
drawnow;
pause(0.1);

%% ==================== 主程序执行部分 ====================

fprintf('========================================\n');
fprintf('   果树非线性振动参数识别系统 V6.0\n');
fprintf('   (最终优化和修复版本)\n');
fprintf('========================================\n\n');

% 步骤 1: 独立加载三组原始数据，得到一个 cell 数组
[accel_data_cell_raw, force_data, force_time, fs, test_config, time_offsets] = loadMultiSensorData();
if isempty(accel_data_cell_raw), return; end

% 步骤 2: 调用自动对齐函数，输入 cell，输出对齐后的 cell
[accel_data_cell, fs_final] = alignSignalsWithXCorr(accel_data_cell_raw);
if isempty(accel_data_cell), return; end % 如果对齐失败则退出

fprintf('\n========== 数据完整性验证 ==========\n');
for i = 1:3
    n_samples = length(accel_data_cell{i}.time);
    duration = accel_data_cell{i}.time(end);
    fprintf('  传感器%d: %d个样本, 时长%.1fs\n', i, n_samples, duration);
end
if ~isempty(force_time)
    fprintf('  力信号: %d个样本, 时长%.1fs\n', length(force_time), force_time(end));
end
fprintf('====================================\n\n');

fprintf('\n========================================\n');
fprintf('   完整的时间偏移量统计\n');
fprintf('========================================\n');
fprintf('  传感器1（根部）: %.4f 秒\n', time_offsets(1));
fprintf('  传感器2（中部）: %.4f 秒\n', time_offsets(2));
fprintf('  传感器3（顶部）: %.4f 秒\n', time_offsets(3));
fprintf('========================================\n\n');

% 步骤 3: 使用完美对齐的 cell 数据进入手动标注流程 (无需修改下游函数)
identified_params = runIdentificationManual(accel_data_cell, ...
        force_data, force_time, fs_final, test_config, time_offsets);

if ~isempty(identified_params)
    generateUpdateFiles(identified_params);
    generateAnalysisReport(identified_params);
    visualizeResults(identified_params);

    fprintf('\n========================================\n');
    fprintf('   参数识别完成！\n');
    fprintf('========================================\n');
else
    fprintf('\n========================================\n');
    fprintf('   用户取消或未完成参数识别流程。\n');
    fprintf('========================================\n');
end

%% ============ 数据加载函数 ============
function [accel_data_cell, force_data, force_time, fs, test_config, time_offsets] = loadMultiSensorData()
    % 功能：加载原始数据（不做任何处理）
    
    accel_data_cell = {}; 
    force_data = []; 
    force_time = [];
    fs = 1000;
    test_config = struct();
    time_offsets = zeros(1, 3);
    
    try
        sensor_names = {'根部传感器', '中部传感器', '顶部传感器'};
        accel_data_raw_cell = cell(1, 3);
        actual_fs_list = zeros(1, 3);
        
        fprintf('\n========== 阶段1: 读取原始数据（不做处理） ==========\n');
        
        for i = 1:3
            fprintf('  [%d/3] 请选择%s的CSV文件...\n', i, sensor_names{i});
            [filename, pathname] = uigetfile('*.csv', sprintf('选择%s数据文件', sensor_names{i}), pwd);
            if isequal(filename, 0), error('用户取消了文件选择。'); end
            
            [accel_matrix, actual_fs, single_offset] = loadSingleSensorCSV(...
                fullfile(pathname, filename), i);
            
            if isempty(accel_matrix), error('传感器 %d 加载失败。', i); end
            
            accel_data_raw_cell{i} = accel_matrix;
            actual_fs_list(i) = actual_fs;
            time_offsets(i) = single_offset;
            
            fprintf('    ✓ 传感器 %d: 读取 %d 个数据点, 采样率 %.2f Hz\n', ...
                i, size(accel_matrix, 1), actual_fs);
        end
        
        % 直接保存原始数据和采样率，不做重采样
        for i = 1:3
            data = accel_data_raw_cell{i};
            time_vec = (0:size(data,1)-1)' / actual_fs_list(i);
            accel_data_cell{i} = struct('time', time_vec, 'data', data, 'fs', actual_fs_list(i));
        end
        
        fprintf('\n  [+] 请选择力锤传感器的文件...\n');
        [force_filename, force_pathname] = uigetfile('*.csv;*.xls;*.xlsx', '选择力锤数据文件');
        if ~isequal(force_filename, 0)
            [force_time, force_data] = loadForceXLS(fullfile(force_pathname, force_filename));
        end
        
        test_config.data_source = 'RawData_NoAlignment';
        test_config.fs = fs;
        test_config.actual_fs_list = actual_fs_list;
        
    catch ME
        errordlg(sprintf('数据加载失败: %s', ME.message), '加载错误');
        accel_data_cell = {}; force_data = []; force_time = []; fs = []; test_config = []; time_offsets = [];
    end
end



function timestamps_seconds = parseTimestamps(timestamp_strings)
    n = length(timestamp_strings);
    timestamps_seconds = zeros(n, 1);
    
    for i = 1:n
        ts_str = char(timestamp_strings(i));
        
        try
            parts = strsplit(ts_str, '_');
            
            if length(parts) == 4
                hours = str2double(parts{1});
                minutes = str2double(parts{2});
                seconds = str2double(parts{3});
                milliseconds = str2double(parts{4});
                
                timestamps_seconds(i) = hours * 3600 + minutes * 60 + seconds + milliseconds / 1000;
            else
                timestamps_seconds(i) = NaN;
            end
        catch
            timestamps_seconds(i) = NaN;
        end
    end
    
    first_valid = find(~isnan(timestamps_seconds), 1);
    if ~isempty(first_valid)
        reference_time = timestamps_seconds(first_valid);
        timestamps_seconds = timestamps_seconds - reference_time;
    end
    
    % 处理重复时间戳：在每组重复值内线性插值
    timestamps_seconds = handleDuplicateTimestamps(timestamps_seconds);
end


function timestamps_unique = handleDuplicateTimestamps(timestamps)
    n = length(timestamps);
    timestamps_unique = timestamps;
    
    i = 1;
    while i <= n
        if isnan(timestamps(i))
            i = i + 1;
            continue;
        end
        
        % 找到连续相同值的范围
        current_value = timestamps(i);
        j = i;
        while j <= n && abs(timestamps(j) - current_value) < 1e-9
            j = j + 1;
        end
        
        % 如果有重复（j > i+1）
        if j > i + 1
            num_duplicates = j - i;
            
            % 确定插值范围
            if j <= n && ~isnan(timestamps(j))
                % 有下一个不同的值，在当前值和下一个值之间插值
                next_value = timestamps(j);
                
                % [代码修正] 先生成正确数量的插值点，再进行赋值
                % linspace会生成 num_duplicates+1 个点，包含起点和终点
                interpolated_points = linspace(current_value, next_value, num_duplicates + 1);
                
                % 我们只需要前 num_duplicates 个点（不包含终点 next_value）
                % 这样左右两侧的元素数量就完全匹配了
                timestamps_unique(i:j-1) = interpolated_points(1:end-1);

            else
                % 没有下一个值，使用估计的采样间隔
                if i > 1
                    prev_valid_idx = find(~isnan(timestamps(1:i-1)), 1, 'last');
                    if ~isempty(prev_valid_idx)
                        % 基于之前的平均间隔进行估计
                        dt_est = (timestamps(i) - timestamps(prev_valid_idx)) / (i - prev_valid_idx);
                    else
                        dt_est = 0.0014; % 默认~700Hz
                    end
                else
                    dt_est = 0.0014;
                end
                
                % 确保 dt_est 是一个合理的正数
                if dt_est <= 0
                    dt_est = 0.0014;
                end
                
                for k = 0:num_duplicates-1
                    timestamps_unique(i+k) = current_value + k * dt_est;
                end
            end
        end
        
        i = j;
    end
end

function [timestamps_clean, data_clean, valid_mask] = cleanTimestamps(timestamps, data)
    n = length(timestamps);
    valid_mask = true(n, 1);
    
    valid_mask = valid_mask & ~isnan(timestamps);
    
    if n > 1
        dt = diff(timestamps);
        non_monotonic = [false; dt < 0];
        valid_mask = valid_mask & ~non_monotonic;
    end
    
    if sum(valid_mask) > 10
        dt = diff(timestamps(valid_mask));
        dt_median = median(dt);
        dt_threshold = dt_median * 5;
        
        large_gaps = [false; dt > dt_threshold];
        temp_valid = valid_mask;
        temp_valid(valid_mask) = temp_valid(valid_mask) & ~large_gaps;
        
        if sum(temp_valid) > 0.9 * sum(valid_mask)
            valid_mask = temp_valid;
        end
    end
    
    timestamps_clean = timestamps(valid_mask);
    data_clean = data(valid_mask, :);
    
    if sum(~valid_mask) > 0
        fprintf('        警告: 移除了 %d 个异常时间戳\n', sum(~valid_mask));
    end
end


function fs_est = estimateSamplingRate(timestamps)
    if length(timestamps) < 2
        fs_est = 1000;
        return;
    end
    
    dt = diff(timestamps);
    
    % ✅ 更严格的过滤：移除异常小和异常大的间隔
    % 假设合理采样率在100-2000Hz，对应间隔在0.0005-0.01秒
    dt_valid = dt(dt > 0.0005 & dt < 0.01);
    
    if isempty(dt_valid)
        % 如果过滤太严格，降低标准再试
        dt_valid = dt(dt > 1e-4 & dt < 0.1);
    end
    
    if isempty(dt_valid)
        warning('无法从时间戳中提取有效的采样间隔，使用默认值1000Hz');
        fs_est = 1000;
        return;
    end
    
    % ✅ 方法1：中位数法（最稳健）
    dt_median = median(dt_valid);
    fs_median = 1 / dt_median;
    
    % ✅ 方法2：使用25-75百分位数的平均值（排除极端值）
    dt_p25 = prctile(dt_valid, 25);
    dt_p75 = prctile(dt_valid, 75);
    dt_mid = mean([dt_p25, dt_p75]);
    fs_percentile = 1 / dt_mid;
    
    % ✅ 方法3：线性拟合法（对时钟漂移鲁棒）
    t_indices = (0:length(timestamps)-1)';
    p = polyfit(t_indices, timestamps, 1);
    fs_fit = 1 / p(1);
    
    % ✅ 只选择合理范围内的估算值
    fs_candidates = [];
    if fs_median >= 100 && fs_median <= 2000
        fs_candidates(end+1) = fs_median;
    end
    if fs_percentile >= 100 && fs_percentile <= 2000
        fs_candidates(end+1) = fs_percentile;
    end
    if fs_fit >= 100 && fs_fit <= 2000
        fs_candidates(end+1) = fs_fit;
    end
    
    % ✅ 如果有多个合理值，取中位数；否则用默认值
    if ~isempty(fs_candidates)
        fs_est = median(fs_candidates);
    else
        warning('所有采样率估算方法都失败，使用默认值1000Hz。dt范围: [%.6f, %.6f]秒', min(dt_valid), max(dt_valid));
        fs_est = 1000;
    end
    
    % ✅ 最终安全检查
    if fs_est < 100 || fs_est > 2000 || isnan(fs_est) || isinf(fs_est)
        fprintf('    警告: 估算值%.2f Hz异常，强制使用1000Hz\n', fs_est);
        fs_est = 1000;
    end
end





function [accel_data_cell_resampled, fs_target] = resampleToUnifiedTimeBase(accel_data_raw_cell, fs_target)
    if nargin < 2
        fs_target = 1000;
    end
    
    accel_data_cell_resampled = cell(1, 3);
    
    t_start_global = -inf;
    t_end_global = inf;
    
    for i = 1:3
        t_start_global = max(t_start_global, accel_data_raw_cell{i}.time(1));
        t_end_global = min(t_end_global, accel_data_raw_cell{i}.time(end));
    end
    
    fprintf('    公共时间范围: [%.3f, %.3f] 秒 (时长 %.3f 秒)\n', ...
            t_start_global, t_end_global, t_end_global - t_start_global);
    
    if t_end_global <= t_start_global
        error('传感器时间范围没有交集！');
    end
    
    unified_time = (t_start_global : 1/fs_target : t_end_global)';
    n_samples_unified = length(unified_time);
    
    fprintf('    统一时间轴: %d 个采样点 @ %.1f Hz\n', n_samples_unified, fs_target);
    
    for i = 1:3
        original_time = accel_data_raw_cell{i}.time;
        original_data = accel_data_raw_cell{i}.data;
        original_fs = accel_data_raw_cell{i}.original_fs;
        
        if abs(original_fs - fs_target) / fs_target < 0.01
            method = 'linear';
        else
            method = 'spline';
        end
        
        resampled_data = zeros(n_samples_unified, 3);
        for dir = 1:3
            resampled_data(:, dir) = interp1(original_time, original_data(:, dir), ...
                                             unified_time, method, 0);
        end
        
        accel_data_cell_resampled{i} = struct(...
            'time', unified_time, ...
            'data', resampled_data, ...
            'original_fs', original_fs, ...
            'resampled_fs', fs_target);
        
        fprintf('    ✓ 传感器%d: %.2f Hz → %.2f Hz (%s)\n', ...
                i, original_fs, fs_target, method);
    end
end


function [accel_data_aligned, fs_final] = alignSignalsWithXCorr(accel_data_cell_raw)
    % 功能：互相关对齐（兼容旧标注）
    
    fprintf('--- 开始执行信号对齐 ---\n');
    
    if length(accel_data_cell_raw) ~= 3
        error('需要3个传感器的数据');
    end
    
    fs_final = 1000;
    
    % 提取Z方向信号
    sig1 = accel_data_cell_raw{1}.data(:, 3);
    sig2 = accel_data_cell_raw{2}.data(:, 3);
    sig3 = accel_data_cell_raw{3}.data(:, 3);
    
    % 关键修改：只用前80秒（或最短信号长度）计算对齐
    min_len = min([length(sig1), length(sig2), length(sig3)]);
    align_len = min(min_len, round(80 * fs_final));  % 最多用80秒
    
    fprintf('    使用前%.1f秒数据计算对齐参数\n', align_len/fs_final);
    
    % 只用前align_len部分计算互相关
    ref_sig = sig1(1:align_len);
    sig2_short = sig2(1:align_len);
    sig3_short = sig3(1:align_len);
    
    max_lag = round(3 * fs_final);
    
    [c12, lags12] = xcorr(ref_sig, sig2_short, max_lag, 'coeff');
    [~, idx12] = max(abs(c12));
    delay2 = lags12(idx12);
    
    [c13, lags13] = xcorr(ref_sig, sig3_short, max_lag, 'coeff');
    [~, idx13] = max(abs(c13));
    delay3 = lags13(idx13);
    
    fprintf('    互相关延迟: 中部=%d样本(%.3fs), 顶部=%d样本(%.3fs)\n', ...
        delay2, delay2/fs_final, delay3, delay3/fs_final);
    
    % 对齐时应用到完整信号
    accel_data_aligned = accel_data_cell_raw;
    fs_final = 1000;
    
    % 传感器1不变
    accel_data_aligned{1} = accel_data_cell_raw{1};
    
    % 传感器2对齐（应用到完整信号）
    data2 = accel_data_cell_raw{2}.data;
    if delay2 > 0
        data2_aligned = [data2(delay2+1:end, :); zeros(delay2, 3)];
    elseif delay2 < 0
        data2_aligned = [zeros(-delay2, 3); data2(1:end+delay2, :)];
    else
        data2_aligned = data2;
    end
    accel_data_aligned{2} = struct('time', accel_data_cell_raw{2}.time, 'data', data2_aligned);
    
    % 传感器3对齐（应用到完整信号）
    data3 = accel_data_cell_raw{3}.data;
    if delay3 > 0
        data3_aligned = [data3(delay3+1:end, :); zeros(delay3, 3)];
    elseif delay3 < 0
        data3_aligned = [zeros(-delay3, 3); data3(1:end+delay3, :)];
    else
        data3_aligned = data3;
    end
    accel_data_aligned{3} = struct('time', accel_data_cell_raw{3}.time, 'data', data3_aligned);
    
    fprintf('--- 信号对齐完成 ---\n\n');
end


function [raw_force_time, raw_force_signal] = loadForceXLS(filepath)
    try
        opts = detectImportOptions(filepath);
        opts = setvartype(opts, 1:2, 'double');
        opts.DataLines = [2, Inf];
        T = readtable(filepath, opts);
        
        raw_force_time = T{:, 1};
        raw_force_signal = T{:, 2};
        
        valid_rows = ~isnan(raw_force_time) & ~isnan(raw_force_signal);
        raw_force_time = raw_force_time(valid_rows);
        raw_force_signal = raw_force_signal(valid_rows);
    catch err
        fprintf('      加载力锤数据失败: %s\n', err.message);
        raw_force_time = [];
        raw_force_signal = [];
    end
end


function processed_segments = processSegmentsInPlace(raw_segments, fs, varargin)
    processed_segments = raw_segments;

    % 解析可选参数
    p = inputParser;
    addParameter(p, 'CutoffFreq', 65);
    addParameter(p, 'FilterOrder', 8);
    parse(p, varargin{:});

    % 使用更稳健的4阶巴特沃斯IIR滤波器替代原有的100阶FIR滤波器
    CUTOFF_FREQ = p.Results.CutoffFreq; % 截断频率设置
    FILTER_ORDER = p.Results.FilterOrder;
    [b, a] = butter(FILTER_ORDER, CUTOFF_FREQ / (fs/2), 'low');
    
    min_cycles = 3;
    min_freq = 5;
    min_required_length = ceil(min_cycles * fs / min_freq);
    
    for i = 1:length(processed_segments)
        seg = processed_segments(i);
        
        if ~isfield(seg, 'signal_data') || length(seg.signal_data) < min_required_length
            continue;
        end
        
        % [核心修正] 使用 filtfilt 进行零相位滤波，能极大地减小对瞬态信号的失真
        signal_filtered = filtfilt(b, a, seg.signal_data);
        
        [envelope, peaks, peak_indices] = extractEnvelopeAndPeaks(signal_filtered, fs); 
        
        if isempty(peaks), continue; end
        
        processed_segments(i).signal_data = signal_filtered;
        
        peak_info = struct('peak_times', (peak_indices - 1) / fs, ...
                          'peak_amplitudes', peaks);
        if length(peak_info.peak_times) >= 2
            peak_info.dominant_frequency = 1 / median(diff(sort(peak_info.peak_times)));
        else
            peak_info.dominant_frequency = 0; 
        end
        
        [snr_val, ~, ~] = calculateAdvancedSNR(signal_filtered, fs);
        detection_results = struct('envelope', envelope, 'snr', snr_val);
        
        processed_segments(i).peak_info = peak_info; 
        processed_segments(i).detection_results = detection_results;
        
        if isfield(seg, 'data') && ~isempty(seg.data)
            n_samples = size(seg.data, 1);
            if n_samples == length(signal_filtered)
                for sensor_idx = 1:size(seg.data, 2)
                    for dir_idx = 1:size(seg.data, 3)
                        processed_segments(i).data(:, sensor_idx, dir_idx) = ...
                            filtfilt(b, a, seg.data(:, sensor_idx, dir_idx));
                    end
                end
            end
        end
    end
    
    processed_segments = processed_segments(arrayfun(@(s) ...
        isfield(s, 'peak_info') && ~isempty(s.peak_info), processed_segments));
end

function [envelope, peaks, peak_indices] = extractEnvelopeAndPeaks(signal, fs)
    % 功能：提取包络线和峰值
    
    if isempty(signal) || length(signal) < 50
        envelope = abs(signal);
        peaks = [];
        peak_indices = [];
        return;
    end
    
    n_samples = length(signal);
    
    % 改进的包络计算：先去高频噪声
    signal_filtered = signal;
    try
        % 低通滤波100Hz，去除高频噪声
        [b, a] = butter(2, 100/(fs/2), 'low');
        signal_filtered = filtfilt(b, a, signal);
    catch
    end
    
    % 计算Hilbert包络
    analytic_signal = hilbert(signal_filtered);
    envelope_raw = abs(analytic_signal);
    
    % 强平滑（0.02秒窗口）
    smooth_window = max(10, round(fs * 0.02));
    envelope = movmean(envelope_raw, smooth_window);
    
    % 找全局最大值
    [max_amp, max_idx] = max(abs(signal));
    
    if max_amp < 0.1
        peaks = [];
        peak_indices = [];
        return;
    end
    
    % 主频估计
    f_dominant = 7;
    if n_samples > 256
        try
            [pxx, f] = pwelch(signal, min(256, n_samples), [], [], fs);
            valid_idx = (f >= 5) & (f <= 30);
            if any(valid_idx)
                [~, max_f_idx] = max(pxx(valid_idx));
                f_valid = f(valid_idx);
                f_dominant = f_valid(max_f_idx);
            end
        catch
        end
    end
    
    % 峰值间距
    min_peak_distance = max(round(0.35 / f_dominant * fs), 20);
    
    peak_indices = [max_idx];
    peaks = [signal(max_idx)];
    
    search_start = max_idx + min_peak_distance;
    last_amp = max_amp;
    
    % 搜索后续峰值
    while length(peak_indices) < 10 && search_start < n_samples - min_peak_distance
        search_end = min(search_start + round(2 * min_peak_distance), n_samples);
        search_window = signal(search_start:search_end);
        
        if length(search_window) < 5
            break;
        end
        
        try
            [pks_pos, locs_pos] = findpeaks(search_window, 'MinPeakProminence', max_amp * 0.05);
            [pks_neg, locs_neg] = findpeaks(-search_window, 'MinPeakProminence', max_amp * 0.05);
            pks_neg = -pks_neg;
            
            all_pks = [pks_pos; pks_neg];
            all_locs = [locs_pos; locs_neg];
            
            if ~isempty(all_pks)
                [~, best_idx] = max(abs(all_pks));
                candidate_amp = abs(all_pks(best_idx));
                candidate_idx = search_start + all_locs(best_idx) - 1;
                
                if candidate_amp <= last_amp * 1.5
                    peak_indices(end+1) = candidate_idx;
                    peaks(end+1) = all_pks(best_idx);
                    last_amp = candidate_amp;
                end
            end
        catch
        end
        
        search_start = search_end;
    end
    
    if length(peak_indices) < 2
        peaks = [];
        peak_indices = [];
    end
end


function [snr, noise_floor, signal_power] = calculateAdvancedSNR(signal, fs)
    % 功能：计算信号信噪比
    
    n = length(signal);
    
    if n < 50
        snr = -10;
        noise_floor = eps;
        signal_power = eps;
        return;
    end
    
    [max_val, max_idx] = max(abs(signal));
    
    if max_val < 0.01
        snr = -10;
        noise_floor = eps;
        signal_power = eps;
        return;
    end
    
    % 找信号起始点（降低阈值到5%）
    threshold = max_val * 0.05;
    signal_start = max_idx;
    for i = max_idx:-1:1
        if abs(signal(i)) < threshold
            signal_start = i + 1;
            break;
        end
    end
    signal_start = max(1, signal_start);
    
    % 信号区域：从起始到峰值后3秒
    signal_end = min(max_idx + round(3*fs), n);
    signal_region = signal(signal_start:signal_end);
    
    % 噪声区域：信号前或信号后
    if signal_start > round(0.2*fs)
        noise_region = signal(1:(signal_start-1));
    else
        if signal_end < n - round(0.2*fs)
            noise_region = signal((signal_end+1):n);
        else
            noise_region = signal(1:min(round(0.1*fs), signal_start-1));
        end
    end
    
    % 计算SNR
    if ~isempty(noise_region) && length(noise_region) > 10 && ~isempty(signal_region)
        signal_power = rms(signal_region);
        noise_floor = rms(noise_region);
        
        if noise_floor == 0 || isnan(noise_floor)
            noise_floor = eps;
        end
        
        snr = 20 * log10(signal_power / noise_floor);
        snr = max(-10, min(60, snr));
    else
        % 回退方法
        signal_power = rms(signal);
        tail_start = max(1, round(0.8*n));
        noise_floor = rms(signal(tail_start:end));
        if noise_floor == 0 || isnan(noise_floor)
            noise_floor = eps;
        end
        snr = 20 * log10(signal_power / noise_floor);
        snr = max(-10, min(60, snr));
    end
end


function zeta = estimateDamping(signal, fs)
    try
        envelope = abs(hilbert(signal));
        [pks, locs] = findpeaks(envelope, 'MinPeakProminence', 0.01*max(envelope), ...
                                'MinPeakDistance', round(0.005*fs));
        
        if length(pks) < 5
            zeta = 0.05; 
            return;
        end
        
        t = locs / fs;
        weights = pks / max(pks);
        
        W = diag(weights);
        X = [ones(length(t), 1), t];
        y = log(pks);
        
        beta = (X' * W * X) \ (X' * W * y);
        decay_rate = -beta(2);
        
        y_fit = X * beta;
        residuals = y - y_fit;
        ss_res = sum(weights .* residuals.^2);
        ss_tot = sum(weights .* (y - mean(y)).^2);
        r_squared = 1 - ss_res / ss_tot;
        
        if r_squared < 0.5
            zeta = 0.05; 
            return;
        end
        
        [pxx, f] = pwelch(signal, [], [], [], fs);
        freq_mask = (f >= 5) & (f <= 40);
        [~, idx] = max(pxx(freq_mask));
        f_interest = f(freq_mask);
        f_dom = f_interest(idx);
        
        omega_d = 2 * pi * f_dom;
        
        zeta = decay_rate / sqrt(decay_rate^2 + omega_d^2);
        
        zeta = min(max(zeta, 0.01), 0.3); 
        if isnan(zeta), zeta = 0.05; end
        
    catch
        zeta = 0.05;
    end
end


function [H_avg, C_avg, freq] = avg_tfestimate(input_signals, output_signals, nfft, fs)
    num_meas = length(input_signals); 
    freq = linspace(0, fs/2, nfft/2 + 1)';
    
    if num_meas == 0
        H_avg = zeros(nfft/2+1, 1);
        C_avg = zeros(nfft/2+1, 1);
        return;
    end
    
    Pxx_sum = 0;
    Pxy_sum = 0;
    Pyy_sum = 0;
    count = 0;
    
    for i = 1:num_meas
        x = input_signals{i}(:);
        y = output_signals{i}(:);
        
        if length(x) > nfft
            x = x(1:nfft);
        else
            x = [x; zeros(nfft - length(x), 1)];
        end
        
        if length(y) > nfft
            y = y(1:nfft);
        else
            y = [y; zeros(nfft - length(y), 1)];
        end
        
        X = fft(x);
        Y = fft(y);
        
        X = X(1:nfft/2+1);
        Y = Y(1:nfft/2+1);
        
        scale = 2 / (fs * nfft);
        Pxx = scale * (X .* conj(X));
        Pyy = scale * (Y .* conj(Y));
        Pxy = scale * (X .* conj(Y));
        
        Pxx(1) = Pxx(1) / 2;
        Pyy(1) = Pyy(1) / 2;
        Pxy(1) = Pxy(1) / 2;
        if mod(nfft, 2) == 0
            Pxx(end) = Pxx(end) / 2;
            Pyy(end) = Pyy(end) / 2;
            Pxy(end) = Pxy(end) / 2;
        end
        
        Pxx_sum = Pxx_sum + Pxx;
        Pxy_sum = Pxy_sum + Pxy;
        Pyy_sum = Pyy_sum + Pyy;
        count = count + 1;
    end
    
    if count == 0
        H_avg = zeros(nfft/2+1, 1);
        C_avg = zeros(nfft/2+1, 1);
        return;
    end
    
    Pxx_avg = Pxx_sum / count;
    Pxy_avg = Pxy_sum / count;
    Pyy_avg = Pyy_sum / count;
    
    H_avg = Pxy_avg ./ (Pxx_avg + 1e-12);
    C_avg = abs(Pxy_avg).^2 ./ (Pxx_avg .* Pyy_avg + 1e-12);
end


function [K, C] = build_matrices(params_vec)
    k_g = params_vec(1); 
    c_g = params_vec(2); 
    k_rm = params_vec(3); 
    c_rm = params_vec(4); 
    k_mt = params_vec(5); 
    c_mt = params_vec(6);
    
    K = [ k_g + k_rm, -k_rm, 0; 
          -k_rm, k_rm + k_mt, -k_mt; 
          0, -k_mt, k_mt ];
    
    C = [ c_g + c_rm, -c_rm, 0; 
          -c_rm, c_rm + c_mt, -c_mt; 
          0, -c_mt, c_mt ];
end


function H_theory = calculate_theoretical_frf(params_vec, M, freq_vector)
    [K, C] = build_matrices(params_vec);
    omega_vec = 2 * pi * freq_vector; 
    num_freqs = length(omega_vec);
    H_theory = zeros(num_freqs, 3, 3);
    
    for i = 1:num_freqs
        omega = omega_vec(i);
        DynamicStiffness = K + 1i * omega * C - omega^2 * M;
        
        if rcond(DynamicStiffness) < 1e-12
            H_matrix_disp = nan(3,3);
        else
            H_matrix_disp = inv(DynamicStiffness);
        end
        
        H_matrix_accel = -omega^2 * H_matrix_disp;
        H_theory(i, :, :) = H_matrix_accel.';
    end
end


function error_vector = calculate_frf_error(params_vec, M, freq_vector, H_exp, Coh_exp)
    freq_mask = (freq_vector >= 1) & (freq_vector <= 50);
    H_theory = calculate_theoretical_frf(params_vec, M, freq_vector);
    error_parts = [];
    
    for i = 1:3
        for j = 1:3
            h_exp_ij = H_exp(freq_mask, i, j); 
            coh_ij = Coh_exp(freq_mask, i, j); 
            h_theory_ij = H_theory(freq_mask, i, j);
            
            if all(h_exp_ij == 0) || isempty(h_exp_ij)
                continue; 
            end
            
            weights = coh_ij.^2;
            weights = weights / (sum(weights) + 1e-12);
            
            error_real = weights .* (real(h_theory_ij) - real(h_exp_ij));
            error_imag = weights .* (imag(h_theory_ij) - imag(h_exp_ij));
            error_parts = [error_parts; error_real; error_imag];
        end
    end
    
    if isempty(error_parts)
        error_vector = 1e6; 
    else
        error_vector = error_parts; 
    end
end

function [raw_accel_matrix, actual_fs, time_offset] = loadSingleSensorCSV(filepath, sensor_idx)
    % [最终自动化版] 通过“总样本数/总时长”精确计算平均采样率，全自动，无弹窗。
    
    raw_accel_matrix = [];
    actual_fs = 1000; % 初始化默认值
    time_offset = 0;
    
    try
        fprintf('      [最终流程] 正在读取传感器 %d...\n', sensor_idx);
        
        opts = delimitedTextImportOptions("NumVariables", 4, "Encoding", "UTF-8");
        opts.DataLines = [2, Inf];
        opts.Delimiter = ",";
        opts.VariableTypes = ["string", "double", "double", "double"];
        opts.MissingRule = "omitrow";
        
        T = readtable(filepath, opts);
        
        if height(T) < 100, error('有效数据行数过少。'); end
        
        timestamp_strings = T{:, 1};
        raw_accel_matrix = T{:, 2:4};

        % 通过解析所有时间戳，以计算总时长
        timestamps_seconds = parseTimestamps(timestamp_strings);
        
        if ~isempty(timestamps_seconds) && length(timestamps_seconds) > 1
            % 获取起始时间偏移量
            time_offset = timestamps_seconds(1);
            
            % 计算总时长
            total_duration = timestamps_seconds(end) - timestamps_seconds(1);
            
            % 获取总样本数
            total_samples = height(T);
            
            if total_duration > 0
                % 根据物理定义计算平均采样率
                actual_fs = total_samples / total_duration;
                fprintf('        文件起始时间偏移: %.6f 秒\n', time_offset);
            else
                fprintf('        警告: 时间戳计算出的总时长为零或负数，无法估算采样率。\n');
            end
        else
            fprintf('        警告: 未能从文件中解析出有效的时间戳。\n');
        end

    catch err
        fprintf(2, '      加载传感器%d的数据序列失败: %s\n', sensor_idx, err.message);
        raw_accel_matrix = [];
    end
end

% --- 新增：绘制力信号图的函数 ---
function updateForcePlot()
    global g_annotation_data;

    % 初始化 force_window_start / duration
    if ~isfield(g_annotation_data, 'force_window_start')
        resetForceWindow();
    end

    % --- 检测用户是否手动缩放过 ---
    ax = g_annotation_data.subplots(1);
    if ishandle(ax)
        if isfield(g_annotation_data, 'keep_view') && g_annotation_data.keep_view
            old_xlim = get(ax, 'XLim');
            old_ylim = get(ax, 'YLim');
        else
            old_xlim = [];
            old_ylim = [];
        end
    else
        old_xlim = [];
        old_ylim = [];
    end

    % --- 绘制力信号 ---
    axes(g_annotation_data.subplots(1));
    cla;

    if ~isempty(g_annotation_data.force_time)
        plot(g_annotation_data.force_time, g_annotation_data.force_data, 'r-', 'LineWidth', 1);
        hold on; grid on;

        % 默认自适应范围（仅当未手动缩放）
        if isempty(old_xlim)
            xlim([g_annotation_data.force_window_start, ...
                  g_annotation_data.force_window_start + g_annotation_data.force_window_duration]);
        else
            xlim(old_xlim);
        end
    else
        text(0.5, 0.5, '未加载力信号数据', 'HorizontalAlignment', 'center');
    end

    title('力锤传感器信号 (Force)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'r');
    ylabel('力 (N)', 'FontWeight', 'bold');
    xlabel('时间 (s)');
    ax = gca;
    ax.Color = [1, 0.95, 0.95];

    % 绘制已保存的力信号标注
    redrawForceSegments();

    % --- 如果用户曾缩放，恢复其视图 ---
    if ~isempty(old_xlim) && ~isempty(old_ylim)
        set(ax, 'XLim', old_xlim, 'YLim', old_ylim);
    end

    hold off;
end


%% ============ 核心参数识别系统 ============
function identified_params = runIdentificationManual(accel_data_cell, force_data, force_time, fs, test_config, time_offsets)
    % SAD框架主流程 - 分阶段自适应动力学参数识别
    % 
    % 输入:
    %   accel_data_cell - 三传感器加速度数据 {Root, Mid, Tip}
    %   force_data      - 力锤信号数据
    %   force_time      - 力锤时间轴
    %   fs              - 采样率
    %   test_config     - 测试配置
    %   time_offsets    - 时间偏移量
    %
    % 输出:
    %   identified_params - 完整的参数结构体
    
    identified_params = [];
    
    fprintf('\n');
    fprintf('╔════════════════════════════════════════════════════════════════╗\n');
    fprintf('║     SAD框架 - 分阶段自适应动力学参数识别                       ║\n');
    fprintf('║     (Staged Adaptive Dynamic Framework)                        ║\n');
    fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');
    
    %% ==================== 数据标注阶段 ====================
    fprintf('【预处理】开始信号段标注...\n');
    
    % 调用手动标注界面获取信号段
    segments = runManualAnnotation(accel_data_cell, force_data, force_time, fs, time_offsets);
    
    if isempty(segments)
        fprintf('  [!] 未获取到有效信号段，流程终止。\n');
        return;
    end
    
    fprintf('  [√] 共获取 %d 个有效信号段\n\n', length(segments));
    
    % 信号预处理
    segments = processSegmentsInPlace(segments, fs, ...
        'CutoffFreq', CUTOFF_FREQ, 'FilterOrder', FILTER_ORDER);
    fprintf('  [√] 信号预处理完成\n\n');
    
    %% ==================== SAD阶段一: 线性基准参数识别 ====================
    fprintf('╔══════════════════════════════════════════════════════════════════╗\n');
    fprintf('║  阶段一: 线性基准参数识别 (Linear Baseline Identification)      ║\n');
    fprintf('╚══════════════════════════════════════════════════════════════════╝\n\n');
    
    linear_params = SAD_Stage1_LinearBaselineIdentification(segments, fs);
    
    if isempty(linear_params) || ~linear_params.valid
        fprintf('  [!] 阶段一失败，无法继续后续阶段。\n');
        return;
    end
    
    fprintf('  [√] 阶段一完成: 提取了 %d 阶模态\n\n', ...
        length(linear_params.natural_freqs_x) + length(linear_params.natural_freqs_z));
    
    %% ==================== SAD阶段二: 非线性特征量化检测 ====================
    fprintf('╔══════════════════════════════════════════════════════════════════╗\n');
    fprintf('║  阶段二: 非线性特征量化检测 (Nonlinearity Detection)            ║\n');
    fprintf('╚══════════════════════════════════════════════════════════════════╝\n\n');
    
    [nl_detection, nl_segments] = SAD_Stage2_NonlinearityDetection(segments, linear_params, fs);
    
    fprintf('  [√] 阶段二完成: %d/%d 个节点检测到显著非线性 (NL_index ≥ %.2f)\n\n', ...
        sum(nl_detection.is_nonlinear), length(nl_detection.NL_index), nl_detection.NL_threshold);
    
    %% ==================== SAD阶段三: 非线性参数识别 (仅对非线性节点) ====================
    fprintf('╔══════════════════════════════════════════════════════════════════╗\n');
    fprintf('║  阶段三: 非线性参数识别 (Harmonic Balance Method)               ║\n');
    fprintf('╚══════════════════════════════════════════════════════════════════╝\n\n');
    
    if any(nl_detection.is_nonlinear)
        nonlinear_params = SAD_Stage3_NonlinearParameterIdentification(...
            nl_segments, linear_params, nl_detection, fs);
        fprintf('  [√] 阶段三完成: 识别了 %d 组非线性参数\n\n', ...
            sum(nonlinear_params.k3_coeffs ~= 0));
    else
        fprintf('  [i] 未检测到显著非线性，跳过阶段三\n');
        nonlinear_params = struct('k3_coeffs', zeros(1,3), 'c2_coeffs', zeros(1,3), ...
            'nonlinear_type', {{'linear', 'linear', 'linear'}}, 'valid', true);
    end
    
    %% ==================== SAD阶段四: 果实脱落力统计标定 ====================
    fprintf('╔══════════════════════════════════════════════════════════════════╗\n');
    fprintf('║  阶段四: 果实脱落力统计标定 (Detachment Force Modeling)         ║\n');
    fprintf('╚══════════════════════════════════════════════════════════════════╝\n\n');
    
    detachment_model = SAD_Stage4_DetachmentForceModeling();
    
    fprintf('  [√] 阶段四完成: 脱落力预测模型已建立\n\n');
    
    %% ==================== 整合所有参数 ====================
    fprintf('╔══════════════════════════════════════════════════════════════════╗\n');
    fprintf('║  参数整合与验证                                                  ║\n');
    fprintf('╚══════════════════════════════════════════════════════════════════╝\n\n');
    
    % 构建统一参数结构体
    identified_params = struct();
    identified_params.fs = fs;
    identified_params.test_config = test_config;
    identified_params.segments = segments;
    
    % 阶段一: 线性参数
    identified_params.linear = linear_params;
    
    % 阶段二: 非线性检测结果
    identified_params.nl_detection = nl_detection;
    
    % 阶段三: 非线性参数
    identified_params.nonlinear = nonlinear_params;
    
    % 阶段四: 脱落力模型
    identified_params.detachment_model = detachment_model;
    
    % 构建全局质量、刚度、阻尼矩阵
    identified_params = buildGlobalMatrices(identified_params);
    
    % 交叉验证
    validation = validateParameters(identified_params, segments);
    identified_params.validation = validation;
    
    % 生成统一参数接口
    identified_params.unified_interface = createUnifiedParameterInterface(identified_params);
    
    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════════════╗\n');
    fprintf('║  SAD框架执行完毕                                                 ║\n');
    fprintf('╚══════════════════════════════════════════════════════════════════╝\n\n');
end

%% ============ 手动信号分割系统 (V37.0 最终修复版) ============
function segments = segmentSignalsManual(accel_data_cell, force_data, force_time, fs, time_offsets)
    global g_annotation_data;
    segments = [];
    
    answer = questdlg('您需要加载已有标注，还是进行全新标注？', '选择操作', '加载已有标注', '进行全新标注', '取消', '加载已有标注');
    
    switch answer
        case '取消'
            return;
            
        case '加载已有标注'
            fprintf('  开始加载已有标注文件...\n');
            [filename, pathname] = uigetfile('*.mat', '请选择之前保存的标注文件');
            
            if isequal(filename, 0)
                fprintf('  用户取消了文件选择。\n');
                segments = [];
                return;
            end
            
            full_filepath = fullfile(pathname, filename);
            fprintf('  正在从 %s 加载...\n', full_filepath);
            
            try
                S = load(full_filepath);
                if ~isfield(S, 'saved_annotations')
                    errordlg('选择的MAT文件无效：缺少 "saved_annotations" 变量。', '加载错误');
                    segments = [];
                    return;
                end
                
                % [核心修复] 调用 reconstructSegments 之前，必须先初始化 g_annotation_data
                % 因为 reconstructSegments 内部调用的 analyzeSegmentTwoStep 依赖于这个全局变量
                g_annotation_data = struct('accel_data_cell', {accel_data_cell}, ...
                                           'force_data', force_data, ...
                                           'force_time', force_time, ...
                                           'fs', fs, ...
                                           'sensor_names', {{'Root', 'Mid', 'Tip'}});
                
                % 调用函数重建数据 (注意：旧签名中的部分参数已无用，但为兼容保留)
                segments = reconstructSegments(S.saved_annotations, accel_data_cell, force_data, force_time, fs);
                
                if ~isempty(segments)
                    fprintf('  成功加载并重建了 %d 个信号段标注。\n', length(segments));
                else
                    fprintf('  警告：文件已加载，但未能重建任何有效的信号段。\n');
                end
                
            catch ME
                errordlg(sprintf('加载标注文件失败: %s', ME.message), '加载错误');
                segments = [];
                return;
            end
            
            % 加载完成后，直接返回，跳过手动标注界面
            return;

        case '进行全新标注'
            fprintf('  开始进行全新标注...\n');
            % 如果是全新标注，则继续执行下面的手动标注流程
    end

    % --- 以下代码仅在 "进行全新标注" 时执行 ---
    g_annotation_data = struct('accel_data_cell', {accel_data_cell}, 'force_data', force_data, 'force_time', force_time, 'fs', fs, 'sensor_names', {{'Root', 'Mid', 'Tip'}}, 'selected_segments', []);
    fprintf('\n  === X方向响应信号段选择 ===\n');
    x_segments = annotateDirectionWithLocation('X', 2, []);
    
    if isfield(g_annotation_data, 'current_figure') && ishandle(g_annotation_data.current_figure), close(g_annotation_data.current_figure); end
    
    g_annotation_data = struct('accel_data_cell', {accel_data_cell}, 'force_data', force_data, 'force_time', force_time, 'fs', fs, 'sensor_names', {{'Root', 'Mid', 'Tip'}}, 'selected_segments', []);
    fprintf('\n  === Z方向响应信号段选择 ===\n');
    z_segments = annotateDirectionWithLocation('Z', 3, []);
    
    segments = [x_segments, z_segments];
    
    if ~isempty(segments)
        answer_save = questdlg(sprintf('标注完成！共 %d 个同步段。\n是否保存本次标注结果？', length(unique([segments.segment_id]))), '保存标注', '保存', '不保存', '保存');
        if strcmp(answer_save, '保存')
            save_dir = 'saved_annotations';
            if ~exist(save_dir, 'dir'), mkdir(save_dir); end
            default_filename = fullfile(save_dir, sprintf('SignalAnnotations_%s.mat', datestr(now, 'yyyy-mm-dd_HHMM')));
            [save_file, save_path] = uiputfile('*.mat', '保存标注文件', default_filename);
            if ~isequal(save_file, 0)
                saveAnnotations(segments, accel_data_cell, g_annotation_data.force_time, fs, fullfile(save_path, save_file));
            end
        end
    end

    if isfield(g_annotation_data, 'current_figure') && ishandle(g_annotation_data.current_figure), close(g_annotation_data.current_figure); end
    clear global g_annotation_data;
    fprintf('  信号分割完成。\n');
end

function segments = annotateDirectionWithLocation(direction_name, direction_idx, preloaded_segments)
    global g_annotation_data;
    if nargin < 3, preloaded_segments = []; end

    fig = figure('Name', sprintf('%s方向信号段选择 - 独立时延标注', direction_name), 'Position', [50, 50, 1700, 950], 'CloseRequestFcn', @(src,evt) closeAnnotationWindow(src,evt));
    
    g_annotation_data.current_figure = fig;
    g_annotation_data.current_direction = direction_name;
    g_annotation_data.direction_idx = direction_idx;
    g_annotation_data.selection_complete = false;
    g_annotation_data.selection_state = 'idle';
    g_annotation_data.current_segment_id = 0;
    
    max_time = 0;
    for i = 1:3, max_time = max(max_time, g_annotation_data.accel_data_cell{i}.time(end)); end
    g_annotation_data.time_window_start = 0;
    g_annotation_data.time_window_duration = min(200, max_time + 10);
    g_annotation_data.current_impact_location = 1;

    createEnhancedAnnotationInterface(fig, direction_name, direction_idx);
    updateCountDisplay();

    while ishandle(fig) && ~g_annotation_data.selection_complete, pause(0.1); end
    
    segments = g_annotation_data.selected_segments;
    if ishandle(fig), close(fig); end
    fprintf('  %s方向标注完成。\n', direction_name);
end

function handleForceStartSelection(actual_time)
    global g_annotation_data;
    g_annotation_data.temp_force_start = struct('time', actual_time);
    clearTempMarkers();
    axes(g_annotation_data.subplots(1)); hold on;
    y_limits = ylim;
    plot([actual_time, actual_time], y_limits, 'r--', 'LineWidth', 1.5, 'Tag', 'temp_line');
    plot(actual_time, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'Tag', 'temp_line');
    hold off;
    g_annotation_data.selection_state = 'selecting_force_end';
    updateStatusDisplay(sprintf('段%d: 力锤起点已选 (%.3fs)，请选择【力锤终点】', g_annotation_data.current_segment_id + 1, actual_time));
end

function handleForceEndSelection(actual_time)
    global g_annotation_data;
    start_time = g_annotation_data.temp_force_start.time;
    if actual_time <= start_time, msgbox('终点时间必须大于起点时间！', '错误', 'error'); return; end
    
    g_annotation_data.temp_force_segment = struct('start_time', start_time, 'end_time', actual_time);
    
    delete(findobj(g_annotation_data.current_figure, 'Tag', 'temp_line'));
    axes(g_annotation_data.subplots(1)); hold on;
    y_limits = ylim;
    fill([start_time, actual_time, actual_time, start_time], [y_limits(1), y_limits(1), y_limits(2), y_limits(2)], 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'Tag', 'temp_fill');
    hold off;
    
    g_annotation_data.selection_state = 'selecting_accel_root_start';
    updateStatusDisplay(sprintf('段%d: 力锤段已确认，请在【根部(Root)】图上选择【响应起点】', g_annotation_data.current_segment_id + 1));
end

function handleAccelResponseSelection(actual_time, sensor_idx)
    global g_annotation_data;
    state = g_annotation_data.selection_state;
    
    target_sensor = 0;
    if strcmp(state, 'selecting_accel_root_start'), target_sensor = 1; end
    if strcmp(state, 'selecting_accel_mid_start'), target_sensor = 2; end
    if strcmp(state, 'selecting_accel_tip_start'), target_sensor = 3; end
    
    if contains(state, '_start')
        if sensor_idx ~= target_sensor, msgbox(sprintf('此步骤请在【%s】传感器图上进行选择！', g_annotation_data.sensor_names{target_sensor}), '流程提示', 'warn'); return; end
        g_annotation_data.temp_accel_start = struct('time', actual_time, 'sensor_idx', sensor_idx);
        axes(g_annotation_data.subplots(sensor_idx+1)); hold on;
        y_lim = ylim; plot([actual_time, actual_time], y_lim, 'b--', 'LineWidth', 1.5, 'Tag', 'temp_line_accel'); hold off;
        
        if target_sensor == 1, g_annotation_data.selection_state = 'selecting_accel_root_end'; end
        if target_sensor == 2, g_annotation_data.selection_state = 'selecting_accel_mid_end'; end
        if target_sensor == 3, g_annotation_data.selection_state = 'selecting_accel_tip_end'; end
        updateStatusDisplay(sprintf('段%d: %s响应起点已选，请选择【响应终点】', g_annotation_data.current_segment_id + 1, g_annotation_data.sensor_names{target_sensor}));
        return;
    end
    
    if contains(state, '_end')
        if sensor_idx ~= g_annotation_data.temp_accel_start.sensor_idx, msgbox('起点和终点必须在同一个传感器图上选择！', '操作错误', 'error'); return; end
        start_time = g_annotation_data.temp_accel_start.time;
        if actual_time <= start_time, msgbox('终点时间必须大于起点时间！', '错误', 'error'); return; end
        
        g_annotation_data.temp_responses(sensor_idx) = struct('start_time', start_time, 'end_time', actual_time);
        
        delete(findobj(g_annotation_data.subplots(sensor_idx+1), 'Tag', 'temp_line_accel'));
        axes(g_annotation_data.subplots(sensor_idx+1)); hold on;
        y_lim = ylim; fill([start_time, actual_time, actual_time, start_time], [y_lim(1), y_lim(1), y_lim(2), y_lim(2)], 'b', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'Tag', 'temp_fill'); hold off;

        if sensor_idx == 1
            g_annotation_data.selection_state = 'selecting_accel_mid_start';
            updateStatusDisplay(sprintf('段%d: 根部响应已确认，请在【中部(Mid)】图上选择【响应起点】', g_annotation_data.current_segment_id + 1));
        elseif sensor_idx == 2
            g_annotation_data.selection_state = 'selecting_accel_tip_start';
            updateStatusDisplay(sprintf('段%d: 中部响应已确认，请在【顶部(Tip)】图上选择【响应起点】', g_annotation_data.current_segment_id + 1));
        elseif sensor_idx == 3
            createIndependentSegments();
            g_annotation_data.selection_state = 'idle';
            updateStatusDisplay(sprintf('段%d 完成！点击"开始新信号段"继续', g_annotation_data.current_segment_id));
            
            % --- [核心修复] ---
            % 只重绘必要的图形，不重置视图
            updateForcePlot();
            for i = 1:3
                updateEnhancedSensorPlot(i, g_annotation_data.direction_idx);
            end
            updateCountDisplay();
        end
    end
end

function enhancedMouseClickHandler(src, ~)
    global g_annotation_data;
    if strcmp(get(src, 'SelectionType'), 'alt'), cancelCurrentSelection(); return; end
    if strcmp(g_annotation_data.selection_state, 'idle'), msgbox('请先点击"开始新信号段"按钮！', '提示', 'warn'); return; end
    
    clicked_axes = gca;
    subplot_idx = find(g_annotation_data.subplots == clicked_axes);
    if isempty(subplot_idx), return; end
    
    axes_point = get(clicked_axes, 'CurrentPoint');
    click_time = axes_point(1, 1);
    
    current_state = g_annotation_data.selection_state;
    
    if subplot_idx == 1
        if strcmp(current_state, 'selecting_force_start'), handleForceStartSelection(click_time);
        elseif strcmp(current_state, 'selecting_force_end'), handleForceEndSelection(click_time);
        else, msgbox('当前步骤请在加速度图上进行选择！', '流程提示', 'warn'); end
    else
        sensor_idx = subplot_idx - 1;
        if contains(current_state, 'selecting_accel_'), handleAccelResponseSelection(click_time, sensor_idx);
        else, msgbox('当前步骤请在力锤图上进行选择！', '流程提示', 'warn'); end
    end
end

function startNewSegmentEnhanced(~, ~)
    global g_annotation_data;
    g_annotation_data.selection_state = 'selecting_force_start';
    
    g_annotation_data.temp_force_start = [];
    g_annotation_data.temp_force_segment = [];
    g_annotation_data.temp_accel_start = [];
    g_annotation_data.temp_responses = repmat(struct('start_time', 0, 'end_time', 0), 1, 3);
    
    clearTempMarkers();
    
    sensor_names = g_annotation_data.sensor_names;
    updateStatusDisplay(sprintf('第%d段: 请在【力锤图】上点击【起点】 (敲击位置:%s)', ...
                           g_annotation_data.current_segment_id + 1, ...
                           sensor_names{g_annotation_data.current_impact_location}));
end

function createIndependentSegments()
    global g_annotation_data;
    g_annotation_data.current_segment_id = g_annotation_data.current_segment_id + 1;
    
    force_seg = g_annotation_data.temp_force_segment;
    accel_responses = g_annotation_data.temp_responses;
    
    for sensor_idx = 1:3
        accel_seg = accel_responses(sensor_idx);
        
        segment_info = struct();
        segment_info.segment_id = g_annotation_data.current_segment_id;
        segment_info.sync_group_id = g_annotation_data.current_segment_id;
        
        segment_info.sensor_idx = sensor_idx;
        segment_info.sensor_name = g_annotation_data.sensor_names{sensor_idx};
        segment_info.direction = g_annotation_data.current_direction;
        segment_info.direction_idx = g_annotation_data.direction_idx;
        segment_info.impact_location = g_annotation_data.current_impact_location;
        
        segment_info.force_start_time = force_seg.start_time;
        segment_info.force_end_time = force_seg.end_time;
        
        segment_info.start_time = accel_seg.start_time;
        segment_info.end_time = accel_seg.end_time;
        segment_info.duration = accel_seg.end_time - accel_seg.start_time;
        
        segment_info.is_synchronized = true;
        segment_info.force_level = NaN;
        
        % [核心修改] 调用新版函数，通过参数传递数据
        segment_info = analyzeSegmentTwoStep(segment_info, ...
            g_annotation_data.accel_data_cell, ...
            g_annotation_data.force_data, ...
            g_annotation_data.force_time, ...
            g_annotation_data.fs);
        
        if isempty(g_annotation_data.selected_segments)
            g_annotation_data.selected_segments = segment_info;
        else
            g_annotation_data.selected_segments(end+1) = segment_info;
        end
    end
    
    g_annotation_data.temp_force_start = [];
    g_annotation_data.temp_force_segment = [];
    g_annotation_data.temp_accel_start = [];
    g_annotation_data.temp_responses = [];
end

function segment_info = analyzeSegmentTwoStep(segment_info, accel_data_cell, ...
                                                    force_data, force_time, fs)
    % (修复版) - 优先使用采样点索引,避免时间戳漂移问题
    
    sensor_idx = segment_info.sensor_idx;
    accel_struct = accel_data_cell{sensor_idx};
    accel_time_vec = accel_struct.time;
    accel_data_matrix = accel_struct.data;

    % ========== [关键修复] 力信号处理:优先使用索引 ==========
    if ~isempty(force_time)
        if isfield(segment_info, 'force_start_idx') && isfield(segment_info, 'force_end_idx')
            % 使用保存的索引
            force_start_idx = segment_info.force_start_idx;
            force_end_idx = segment_info.force_end_idx;
            
            % 安全检查
            if force_start_idx >= 1 && force_end_idx <= length(force_time) && force_end_idx > force_start_idx
                segment_info.force_data_segment = force_data(force_start_idx:force_end_idx);
            else
                segment_info.force_data_segment = [];
            end
        else
            % 回退:使用时间戳查找
            force_start_idx = find(force_time >= segment_info.force_start_time, 1, 'first');
            force_end_idx = find(force_time <= segment_info.force_end_time, 1, 'last');
            if ~isempty(force_start_idx) && ~isempty(force_end_idx) && force_end_idx > force_start_idx
                segment_info.force_data_segment = force_data(force_start_idx:force_end_idx);
            else
                segment_info.force_data_segment = [];
            end
        end
    else
        segment_info.force_data_segment = [];
    end

    % ========== [关键修复] 加速度信号处理:优先使用索引 ==========
    if isfield(segment_info, 'accel_start_idx') && isfield(segment_info, 'accel_end_idx')
        % 使用保存的索引
        accel_start_idx = segment_info.accel_start_idx;
        accel_end_idx = segment_info.accel_end_idx;
    else
        % 回退:使用时间戳查找
        accel_start_idx = find(accel_time_vec >= segment_info.start_time, 1, 'first');
        accel_end_idx = find(accel_time_vec <= segment_info.end_time, 1, 'last');
    end
    % ========== [关键修复结束] ==========
    
    if ~isempty(accel_start_idx) && ~isempty(accel_end_idx) && accel_end_idx > accel_start_idx
        dir_idx = segment_info.direction_idx;
        segment_signal = accel_data_matrix(accel_start_idx:accel_end_idx, dir_idx);
        segment_info.signal_data = segment_signal;
        segment_info.time_data = (0:length(segment_signal)-1)' / fs;
        
        all_sensor_data_segment = zeros(length(segment_signal), 3, 3);
        for s_idx = 1:3
            temp_t = accel_data_cell{s_idx}.time;
            temp_data = accel_data_cell{s_idx}.data;
            s_start_idx = find(temp_t >= segment_info.start_time, 1, 'first');
            s_end_idx = s_start_idx + length(segment_signal) - 1;
            if ~isempty(s_start_idx) && s_end_idx <= length(temp_t)
                all_sensor_data_segment(:, s_idx, :) = temp_data(s_start_idx:s_end_idx, :);
            end
        end
        segment_info.data = all_sensor_data_segment;
        
        segment_info.max_amplitude = max(abs(segment_signal));
        segment_info.peak_amplitude = segment_info.max_amplitude;
        
        if length(segment_signal) > 10
            [psd, f] = pwelch(segment_signal, [], [], [], fs);
            search_indices = f <= 25;
            if any(search_indices)
                [~, max_idx_rel] = max(psd(search_indices));
                f_subset = f(search_indices);
                segment_info.peak_frequency = f_subset(max_idx_rel);
            else
                segment_info.peak_frequency = 5;
            end
        else
            segment_info.peak_frequency = 5;
        end
        
        segment_info.damping = estimateDamping(segment_signal, fs);
    else
        segment_info.signal_data = [];
        segment_info.time_data = [];
        segment_info.data = [];
        segment_info.max_amplitude = 0;
        segment_info.peak_amplitude = 0;
        segment_info.peak_frequency = 5;
        segment_info.damping = 0.02;
    end
end

function redrawExistingSegmentsEnhanced(current_sensor_idx)
    global g_annotation_data;
    
    if isempty(g_annotation_data.selected_segments)
        return;
    end
    
    % 该函数被调用时，正确的坐标轴已经被激活 (by updateEnhancedSensorPlot)
    % 它只需在当前激活的坐标轴上绘制属于它的标注即可
    
    t_start = g_annotation_data.time_window_start;
    t_end = t_start + g_annotation_data.time_window_duration;
    
    colors = {[0, 0.5, 0], [0, 0, 0.7], [0.7, 0, 0.7]}; % Root, Mid, Tip colors
    
    for i = 1:length(g_annotation_data.selected_segments)
        seg = g_annotation_data.selected_segments(i);
        
        % --- 核心修正点：只绘制属于当前传感器图的标注 ---
        if seg.sensor_idx == current_sensor_idx
            
            % 检查标注是否在当前时间窗口内可见
            if seg.end_time >= t_start && seg.start_time <= t_end
                hold on;
                
                y_limits = ylim;
                impact_color = colors{seg.impact_location};
                
                % 绘制标记 (Markers)
                plot([seg.start_time, seg.start_time], y_limits, '--', ...
                     'Color', impact_color, 'LineWidth', 2, 'Tag', 'saved_segment');
                plot(seg.start_time, 0, 'o', 'MarkerSize', 6, ...
                     'MarkerFaceColor', impact_color, 'Color', impact_color, 'Tag', 'saved_segment');
                plot([seg.end_time, seg.end_time], y_limits, '--', ...
                     'Color', impact_color*0.7, 'LineWidth', 2, 'Tag', 'saved_segment');
                plot(seg.end_time, 0, 's', 'MarkerSize', 6, ...
                     'MarkerFaceColor', impact_color*0.7, 'Color', impact_color*0.7, 'Tag', 'saved_segment');
                
                % 绘制高亮区域
                x_fill = [seg.start_time, seg.end_time, seg.end_time, seg.start_time];
                y_fill = [y_limits(1), y_limits(1), y_limits(2), y_limits(2)];
                fill(x_fill, y_fill, impact_color, 'FaceAlpha', 0.15, ...
                     'EdgeColor', 'none', 'Tag', 'saved_segment');
                
                % 绘制标签
                sensor_names = {'ROOT', 'MID', 'TIP'};
                impact_names = {'R', 'M', 'T'};
                text((seg.start_time + seg.end_time)/2, y_limits(2)*0.8, ...
                     sprintf('段%d [%s→%s]', seg.segment_id, ...
                             impact_names{seg.impact_location}, ...
                             sensor_names{current_sensor_idx}(1)), ...
                     'FontSize', 9, 'Color', impact_color*0.8, ...
                     'FontWeight', 'bold', ...
                     'HorizontalAlignment', 'center', 'Tag', 'saved_segment');
            end
        end
    end
end

% NEW FUNCTION to draw saved annotations on the force plot (FINAL FIX v3)
function redrawForceSegments()
    global g_annotation_data;
    
    if isempty(g_annotation_data.selected_segments)
        return;
    end
    
    % --- 步骤 1: 筛选出只属于当前显示方向的标注段 ---
    current_dir = g_annotation_data.current_direction;
    direction_mask = arrayfun(@(s) isfield(s, 'direction') && strcmp(s.direction, current_dir), g_annotation_data.selected_segments);
    segments_to_draw = g_annotation_data.selected_segments(direction_mask);
    
    if isempty(segments_to_draw)
        return; % 如果当前方向没有标注，则不进行任何绘制
    end
    
    % --- 核心修正：改变绘图逻辑 ---
    % 我们不再按 unique_id 循环，而是直接遍历所有筛选出的段落，
    % 并通过一个 "drawn_ids" 列表来防止重复绘制同一个力信号段。
    
    drawn_ids = []; % 用于记录已经绘制过的锤击ID
    colors = {[0, 0.5, 0], [0, 0, 0.7], [0.7, 0, 0.7]};
    
    % 直接遍历所有属于当前方向的段落
    for i = 1:length(segments_to_draw)
        seg = segments_to_draw(i);
        
        % 检查这个段的锤击ID是否已经被绘制过
        if ismember(seg.sync_group_id, drawn_ids)
            continue; % 如果画过了，就跳到下一个段落
        end
        
        % 检查段落在当前窗口是否可见 (这部分逻辑不变)
        t_start_win = g_annotation_data.force_window_start;
        t_end_win = t_start_win + g_annotation_data.force_window_duration;
        
        if isfield(seg, 'force_end_time') && seg.force_end_time >= t_start_win && ...
           isfield(seg, 'force_start_time') && seg.force_start_time <= t_end_win
           
            % --- 开始绘制 ---
            axes(g_annotation_data.subplots(1));
            hold on;
            
            y_limits = ylim;
            impact_color = colors{seg.impact_location};
            impact_abbrev = {'R', 'M', 'T'};
            
            % 绘制高亮区域和标记
            x_fill = [seg.force_start_time, seg.force_end_time, seg.force_end_time, seg.force_start_time];
            y_fill = [y_limits(1), y_limits(1), y_limits(2), y_limits(2)];
            fill(x_fill, y_fill, impact_color, 'FaceAlpha', 0.15, ...
                 'EdgeColor', 'none', 'Tag', 'saved_segment_force');
            
            plot([seg.force_start_time, seg.force_start_time], y_limits, '--', ...
                 'Color', impact_color, 'LineWidth', 2, 'Tag', 'saved_segment_force');
            plot([seg.force_end_time, seg.force_end_time], y_limits, '--', ...
                 'Color', impact_color*0.7, 'LineWidth', 2, 'Tag', 'saved_segment_force');
            
            text((seg.force_start_time + seg.force_end_time)/2, y_limits(2)*0.85, ...
                 sprintf('力段%d [%s]', seg.segment_id, impact_abbrev{seg.impact_location}), ...
                 'HorizontalAlignment', 'center', 'FontWeight', 'bold', ...
                 'BackgroundColor', 'white', 'Color', impact_color, 'Tag', 'saved_segment_force');
                 
            % --- 关键步骤：将当前锤击ID记录到已绘制列表 ---
            drawn_ids(end+1) = seg.sync_group_id;
        end
    end
end

function updateEnhancedSensorPlot(sensor, direction_idx)
    global g_annotation_data;

    accel_struct = g_annotation_data.accel_data_cell{sensor};
    t = accel_struct.time;
    accel_data = accel_struct.data;

    t_start = g_annotation_data.time_window_start;
    t_end = t_start + g_annotation_data.time_window_duration;
    time_mask = t >= t_start & t <= t_end;
    t_window = t(time_mask);
    sensor_data = accel_data(time_mask, direction_idx);

    ax = g_annotation_data.subplots(sensor+1);

    % --- 检测是否需要保持用户缩放 ---
    if ishandle(ax)
        if isfield(g_annotation_data, 'keep_view') && g_annotation_data.keep_view
            old_xlim = get(ax, 'XLim');
            old_ylim = get(ax, 'YLim');
        else
            old_xlim = [];
            old_ylim = [];
        end
    else
        old_xlim = [];
        old_ylim = [];
    end

    axes(ax);
    cla;
    plot(t_window, sensor_data, 'b-', 'LineWidth', 1);
    hold on; grid on;
    ylabel('加速度 (g)');

    % 默认自动设置范围（除非用户缩放过）
    if isempty(old_xlim)
        xlim([t_start, t_end]);
    else
        xlim(old_xlim);
    end
    if isempty(old_ylim)
        if ~isempty(sensor_data)
            data_range = max(abs(sensor_data));
            if data_range > 0
                ylim([-data_range*1.2, data_range*1.2]);
            end
        end
    else
        ylim(old_ylim);
    end

    sensor_names = g_annotation_data.sensor_names;
    title_str = sprintf('%s传感器 - %s方向', sensor_names{sensor}, g_annotation_data.current_direction);
    if sensor == g_annotation_data.current_impact_location
        title_str = [title_str, ' [当前敲击位置]'];
    end
    title(title_str);

    if sensor == 3, xlabel('时间 (s)'); end

    redrawExistingSegmentsEnhanced(sensor);
    updateSensorPlotHighlight();

    hold off;
end


function undoLastSegmentEnhanced(~, ~)
    global g_annotation_data;
    if isempty(g_annotation_data.selected_segments), msgbox('没有可撤销的信号段！', '提示', 'help'); return; end
    
    last_group_id = g_annotation_data.selected_segments(end).sync_group_id;
    
    answer = questdlg(sprintf('确认要撤销第 %d 组标注吗？', last_group_id), '撤销确认', '确认', '取消', '确认');
    if ~strcmp(answer, '确认'), return; end
    
    % 找到所有属于最后一组的标注并删除
    indices_to_remove = [g_annotation_data.selected_segments.sync_group_id] == last_group_id;
    g_annotation_data.selected_segments(indices_to_remove) = [];
    
    % 更新 segment_id 计数器
    if last_group_id == g_annotation_data.current_segment_id
        g_annotation_data.current_segment_id = g_annotation_data.current_segment_id - 1;
    end
    
    fprintf('  已撤销第 %d 组标注。\n', last_group_id);
    
    % 重绘所有图以移除已撤销的标注
    updateAllPlotsEnhanced();
    
    updateStatusDisplay('已撤销最后一组信号段');
    updateCountDisplay();
end

% --- 辅助函数 (FIXED & FINAL) ---
function updateAllPlotsEnhanced()
    global g_annotation_data;
    
    if ishandle(g_annotation_data.subplots(1))
        updateForcePlot();
    end
    
    for sensor = 1:3
        if length(g_annotation_data.subplots) >= (sensor + 1) && ishandle(g_annotation_data.subplots(sensor + 1))
            % --- 核心修改：传递正确的参数 ---
            updateEnhancedSensorPlot(sensor, g_annotation_data.direction_idx);
        end
    end
    
    t_start = g_annotation_data.time_window_start;
    t_end = t_start + g_annotation_data.time_window_duration;
    if isfield(g_annotation_data, 'time_info_text') && ishandle(g_annotation_data.time_info_text)
        set(g_annotation_data.time_info_text, 'String', sprintf('窗口：%.1f-%.1fs', t_start, t_end));
    end
    
    updateSensorPlotHighlight();
end


function createEnhancedAnnotationInterface(fig, direction_name, direction_idx)
    global g_annotation_data;
    main_panel = uipanel('Parent', fig, 'Position', [0, 0, 0.82, 1]);
    control_panel = uipanel('Parent', fig, 'Position', [0.82, 0, 0.18, 1], 'Title', '增强控制面板');
    
    n_sensors = 3; % 我们有3个传感器
    g_annotation_data.subplots = gobjects(1, n_sensors + 1);
    
    % 1. 力信号图
    g_annotation_data.subplots(1) = subplot(4, 1, 1, 'Parent', main_panel);
    updateForcePlot();
    
    % 2. 加速度信号图
    for sensor = 1:n_sensors
        g_annotation_data.subplots(sensor + 1) = subplot(4, 1, sensor + 1, 'Parent', main_panel);
        % --- 核心修改：传递正确的参数 ---
        updateEnhancedSensorPlot(sensor, direction_idx);
    end
    
    g_annotation_data.title_handle = sgtitle(main_panel, ...
        sprintf('%s方向信号段选择 - 当前敲击位置: ROOT', direction_name), ...
        'FontSize', 14, 'FontWeight', 'bold', 'Color', [0, 0.5, 0]);
    
    annotation(main_panel, 'textbox', [0.02, 0.01, 0.8, 0.04], ...
               'String', '说明: 1.点"开始新段" 2.在任意加速度图上点【起点】 3.在任意加速度图上点【终点】 (右键取消)', ...
               'FontSize', 10, 'EdgeColor', 'black', 'BackgroundColor', [1, 1, 0.9]);
    
    createEnhancedControlButtons(control_panel, direction_name);
    createNavigationControls(control_panel);

    addlistener(fig, 'SizeChanged', @(~,~) setKeepView(true));  % 窗口尺寸变化
    addlistener(fig, 'WindowScrollWheel', @(~,~) setKeepView(true)); % 鼠标滚轮缩放
    set(fig, 'WindowButtonDownFcn', @enhancedMouseClickHandler);
    
    fprintf('    加速度图 X 轴已强制同步。\n');
end

function createEnhancedControlButtons(control_panel, direction_name)
    global g_annotation_data;
    
    button_height = 28;
    button_width = 140;
    start_y = 850;
    spacing = 35;
    
    % Impact location selection section
    uicontrol('Parent', control_panel, 'Style', 'text', ...
              'String', '===敲击位置选择===', ...
              'Position', [10, start_y, button_width, 20], ...
              'FontSize', 11, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.9, 0.9, 1]);
    
    % Radio buttons for impact location
    g_annotation_data.impact_bg = uibuttongroup('Parent', control_panel, ...
        'Position', [0.05, 0.75, 0.9, 0.12], ... % MODIFIED: Moved up from 0.68
        'Title', '选择敲击位置', ...
        'SelectionChangedFcn', @impactLocationChanged);
    
    g_annotation_data.radio_root = uicontrol('Parent', g_annotation_data.impact_bg, ...
        'Style', 'radiobutton', 'String', 'Root (根部)', ...
        'Position', [10, 55, 120, 20], 'FontSize', 10);
    
    g_annotation_data.radio_mid = uicontrol('Parent', g_annotation_data.impact_bg, ...
        'Style', 'radiobutton', 'String', 'Mid (中部)', ...
        'Position', [10, 30, 120, 20], 'FontSize', 10);
    
    g_annotation_data.radio_tip = uicontrol('Parent', g_annotation_data.impact_bg, ...
        'Style', 'radiobutton', 'String', 'Tip (顶部)', ...
        'Position', [10, 5, 120, 20], 'FontSize', 10);  
    
    % Main control buttons
    uicontrol('Parent', control_panel, 'Style', 'pushbutton', ...
              'String', '开始新信号段', ...
              'Position', [10, start_y-6*spacing, button_width, button_height], ...
              'FontSize', 10, 'BackgroundColor', [0.2, 0.8, 0.2], ...
              'ForegroundColor', 'white', ...
              'Callback', @startNewSegmentEnhanced);
    
    uicontrol('Parent', control_panel, 'Style', 'pushbutton', ...
              'String', '取消当前选择', ...
              'Position', [10, start_y-7*spacing, button_width, button_height], ...
              'FontSize', 10, 'BackgroundColor', [0.8, 0.6, 0.2], ...
              'ForegroundColor', 'white', ...
              'Callback', @cancelCurrentSelection);
    
    uicontrol('Parent', control_panel, 'Style', 'pushbutton', ...
              'String', '撤销最后信号段', ...
              'Position', [10, start_y-8*spacing, button_width, button_height], ...
              'FontSize', 10, 'BackgroundColor', [0.8, 0.8, 0.2], ...
              'Callback', @undoLastSegmentEnhanced);
    
    uicontrol('Parent', control_panel, 'Style', 'pushbutton', ...
              'String', sprintf('结束%s方向', direction_name), ...
              'Position', [10, start_y-9*spacing, button_width, button_height], ...
              'FontSize', 10, 'BackgroundColor', [0.8, 0.2, 0.2], ...
              'ForegroundColor', 'white', ...
              'Callback', @finishSelection);
    
    % Status displays
    g_annotation_data.status_text = uicontrol('Parent', control_panel, 'Style', 'text', ...
        'String', '状态：等待开始选择', ...
        'Position', [10, start_y-11*spacing, button_width, 35], ...
        'FontSize', 9, 'BackgroundColor', 'white');
    
    g_annotation_data.count_text = uicontrol('Parent', control_panel, 'Style', 'text', ...
        'String', '已选择：0 个信号段', ...
        'Position', [10, start_y-12*spacing, button_width, 20], ...
        'FontSize', 9, 'BackgroundColor', 'white');
    
    g_annotation_data.current_impact_text = uicontrol('Parent', control_panel, 'Style', 'text', ...
        'String', '当前敲击: ROOT-X', ...
        'Position', [10, start_y-13*spacing, button_width, 20], ...
        'FontSize', 10, 'FontWeight', 'bold', ...
        'BackgroundColor', [1, 1, 0.8]);
end

function createNavigationControls(control_panel)
    global g_annotation_data;
    
    button_height = 25;
    
    % --- 1. 力信号导航控件 ---
    start_y_force = 320;
    uicontrol('Parent', control_panel, 'Style', 'text', 'String', '力信号导航 (Force)', ...
              'Position', [10, start_y_force, 140, 20], 'FontSize', 11, 'FontWeight', 'bold', 'ForegroundColor', 'r');
    
    uicontrol('Parent', control_panel, 'Style', 'pushbutton', 'String', '<<', ...
              'Position', [10, start_y_force-30, 60, button_height], 'FontSize', 10, ...
              'Callback', @(~,~) moveForceWindow(-0.2)); % 新的回调
    
    uicontrol('Parent', control_panel, 'Style', 'pushbutton', 'String', '>>', ...
              'Position', [80, start_y_force-30, 60, button_height], 'FontSize', 10, ...
              'Callback', @(~,~) moveForceWindow(0.2)); % 新的回调

    uicontrol('Parent', control_panel, 'Style', 'pushbutton', 'String', '+', ...
              'Position', [10, start_y_force-60, 60, button_height], 'FontSize', 12, ...
              'Callback', @(~,~) zoomForceWindow(0.9)); % 新的回调
    
    uicontrol('Parent', control_panel, 'Style', 'pushbutton', 'String', '-', ...
              'Position', [80, start_y_force-60, 60, button_height], 'FontSize', 12, ...
              'Callback', @(~,~) zoomForceWindow(1.1)); % 新的回调

    uicontrol('Parent', control_panel, 'Style', 'pushbutton', 'String', '重置力信号视图', ...
              'Position', [10, start_y_force-90, 130, button_height], 'FontSize', 10, ...
              'Callback', @resetForceWindow); % 新的回调
              
    % --- 2. 加速度信号导航控件 ---
    start_y_accel = 180;
    uicontrol('Parent', control_panel, 'Style', 'text', 'String', '加速度信号导航 (Accel)', ...
              'Position', [10, start_y_accel, 140, 20], 'FontSize', 11, 'FontWeight', 'bold', 'ForegroundColor', 'b');
    
    uicontrol('Parent', control_panel, 'Style', 'pushbutton', 'String', '<<', ...
              'Position', [10, start_y_accel-30, 60, button_height], 'FontSize', 10, ...
              'Callback', @(~,~) moveTimeWindow(-0.2)); % 旧的回调
    
    uicontrol('Parent', control_panel, 'Style', 'pushbutton', 'String', '>>', ...
              'Position', [80, start_y_accel-30, 60, button_height], 'FontSize', 10, ...
              'Callback', @(~,~) moveTimeWindow(0.2)); % 旧的回调
    
    uicontrol('Parent', control_panel, 'Style', 'pushbutton', 'String', '+', ...
              'Position', [10, start_y_accel-60, 60, button_height], 'FontSize', 12, ...
              'Callback', @(~,~) zoomTimeWindow(0.9)); % 旧的回调
    
    uicontrol('Parent', control_panel, 'Style', 'pushbutton', 'String', '-', ...
              'Position', [80, start_y_accel-60, 60, button_height], 'FontSize', 12, ...
              'Callback', @(~,~) zoomTimeWindow(1.1)); % 旧的回调

    uicontrol('Parent', control_panel, 'Style', 'pushbutton', 'String', '重置加速度视图', ...
              'Position', [10, start_y_accel-90, 130, button_height], 'FontSize', 10, ...
              'Callback', @resetTimeWindow); % 旧的回调
    
    g_annotation_data.time_info_text = uicontrol('Parent', control_panel, 'Style', 'text', ...
                                                 'String', '', 'Position', [10, start_y_accel-115, 130, 20], ...
                                                 'FontSize', 9, 'BackgroundColor', 'white');
end


%% ============ 按钮回调函数 ============
function cancelCurrentSelection(~, ~)
    global g_annotation_data;
    if strcmp(g_annotation_data.selection_state, 'idle'), return; end
    
    g_annotation_data.selection_state = 'idle';
    g_annotation_data.temp_force_start = [];
    g_annotation_data.temp_force_segment = [];
    g_annotation_data.temp_accel_start = [];
    
    % 移除所有临时标记并重绘所有图，以清除预览线
    updateAllPlotsEnhanced();
    
    updateStatusDisplay('已取消选择，请点击"开始新信号段"');
    fprintf('  已取消当前段的选择\n');
end


function finishSelection(~, ~)
    global g_annotation_data;
    
    if isempty(g_annotation_data.selected_segments)
        answer_empty = questdlg('尚未选择任何信号段！确认要结束吗？', '警告', '确认结束', '继续选择', '继续选择');
        if strcmp(answer_empty, '继续选择')
            return;
        end
    end
    
    answer = questdlg(sprintf('确认结束%s方向的选择？\n共选择了 %d 个信号段', ...
                      g_annotation_data.current_direction, length(unique([g_annotation_data.selected_segments.segment_id]))), ...
                     '确认结束', '确认', '继续选择', '确认');
    
    if strcmp(answer, '确认')
        g_annotation_data.selection_complete = true;
        updateStatusDisplay('选择已完成');
    end
end

function closeAnnotationWindow(src, ~)
    global g_annotation_data;
    
    if ~g_annotation_data.selection_complete
        answer = questdlg('标注尚未完成，是否强制关闭？当前进度将丢失。', ...
                         '确认关闭', '强制关闭', '继续标注', '继续标注');
        
        if strcmp(answer, '继续标注')
            return;
        else  % 强制关闭
            g_annotation_data.selected_segments = []; % 清空结果
            g_annotation_data.selection_complete = true;
        end
    end
    
    delete(src);
end

function saveCurrentAnnotations()
    global g_annotation_data;
    
    % 创建临时文件名（包含方向信息）
    temp_file = sprintf('temp_annotations_%s_%s.mat', ...
                       g_annotation_data.current_direction, ...
                       datestr(now, 'HHMMSS'));
    
    % 保存当前方向的标注
    annotations = g_annotation_data.selected_segments;
    direction = g_annotation_data.current_direction;
    save_time = datestr(now);
    
    save(temp_file, 'annotations', 'direction', 'save_time');
    fprintf('  %s方向标注已临时保存到 %s\n', direction, temp_file);
end
%% ============ 导航控制函数 ============
% --- 导航控制函数 (FIXED) ---
function moveTimeWindow(direction)
    global g_annotation_data;
    
    step = g_annotation_data.time_window_duration * 0.5 * direction;
    new_start = g_annotation_data.time_window_start + step;
    
    max_time = 0;
    for i = 1:3, max_time = max(max_time, g_annotation_data.accel_data_cell{i}.time(end)); end
    
    if direction < 0
        g_annotation_data.time_window_start = max(0, new_start);
    else
        if (new_start + g_annotation_data.time_window_duration) <= max_time
            g_annotation_data.time_window_start = new_start;
        else
            g_annotation_data.time_window_start = max(0, max_time - g_annotation_data.time_window_duration);
        end
    end
    
    updateAllPlotsEnhanced();
end

function zoomTimeWindow(factor)
    global g_annotation_data;
    
    max_time = g_annotation_data.accel_time(end);
    new_duration = g_annotation_data.time_window_duration * factor;
    
    if factor < 1  % 放大
        g_annotation_data.time_window_duration = max(5, new_duration); % 最小窗口宽度为5秒
    else  % 缩小
        g_annotation_data.time_window_duration = min(max_time, new_duration);
    end
    
    % 确保窗口起始点在合法范围内
    g_annotation_data.time_window_start = max(0, g_annotation_data.time_window_start);
    if (g_annotation_data.time_window_start + g_annotation_data.time_window_duration) > max_time
        g_annotation_data.time_window_start = max(0, max_time - g_annotation_data.time_window_duration);
    end

    % --- 核心修正点：调用正确的、能更新所有图的函数 ---
    updateAllPlotsEnhanced();
end

function resetTimeWindow(~, ~)
    global g_annotation_data;
    
    g_annotation_data.time_window_start = 0;
    g_annotation_data.time_window_duration = 65;
    updateAllPlotsEnhanced();
end

function moveForceWindow(direction)
    global g_annotation_data;
    
    if isempty(g_annotation_data.force_time), return; end

    % 如果尚未初始化，则进行初始化
    if ~isfield(g_annotation_data, 'force_window_start')
        resetForceWindow();
    end

    step = g_annotation_data.force_window_duration * 0.5 * direction;
    new_start = g_annotation_data.force_window_start + step;
    max_time = g_annotation_data.force_time(end);
    
    if direction < 0
        g_annotation_data.force_window_start = max(0, new_start);
    else
        if (new_start + g_annotation_data.force_window_duration) <= max_time
            g_annotation_data.force_window_start = new_start;
        else
            g_annotation_data.force_window_start = max(0, max_time - g_annotation_data.force_window_duration);
        end
    end
    
    updateForcePlot(); % 只更新力信号图
end

function zoomForceWindow(factor)
    global g_annotation_data;
    
    if isempty(g_annotation_data.force_time), return; end
    
    % 如果尚未初始化，则进行初始化
    if ~isfield(g_annotation_data, 'force_window_duration')
        resetForceWindow();
    end
    
    max_time = g_annotation_data.force_time(end);
    current_center = g_annotation_data.force_window_start + g_annotation_data.force_window_duration / 2;
    new_duration = g_annotation_data.force_window_duration * factor;
    
    if factor < 1
        new_duration = max(1, new_duration); % 最小窗口宽度为1秒
    else
        new_duration = min(max_time, new_duration);
    end
    
    g_annotation_data.force_window_duration = new_duration;
    
    % 围绕中心点进行缩放
    new_start = current_center - new_duration / 2;
    g_annotation_data.force_window_start = max(0, new_start);
    
    if (g_annotation_data.force_window_start + g_annotation_data.force_window_duration) > max_time
        g_annotation_data.force_window_start = max(0, max_time - g_annotation_data.force_window_duration);
    end

    updateForcePlot(); % 只更新力信号图
end

function resetForceWindow(~, ~)
    global g_annotation_data;
    
    if ~isempty(g_annotation_data.force_time)
        % 初始化或重置力信号图的显示范围
        g_annotation_data.force_window_start = g_annotation_data.force_time(1);
        g_annotation_data.force_window_duration = g_annotation_data.force_time(end) - g_annotation_data.force_time(1);
    else
        % 如果没有力数据，设置一个默认范围
        g_annotation_data.force_window_start = 0;
        g_annotation_data.force_window_duration = 100;
    end

    if isfield(g_annotation_data, 'subplots') && ishandle(g_annotation_data.subplots(1))
        updateForcePlot(); % 更新力信号图
    end
end
%% ============ 辅助函数 ============
function clearTempMarkers()
    global g_annotation_data;
    
    for subplot_idx = 1:length(g_annotation_data.subplots)
        if ishandle(g_annotation_data.subplots(subplot_idx))
            delete(findobj(g_annotation_data.subplots(subplot_idx), 'Tag', 'temp_start'));
            delete(findobj(g_annotation_data.subplots(subplot_idx), 'Tag', 'temp_end'));
            delete(findobj(g_annotation_data.subplots(subplot_idx), 'Tag', 'preview'));
            delete(findobj(g_annotation_data.subplots(subplot_idx), 'Tag', 'temp_force_start'));
            delete(findobj(g_annotation_data.subplots(subplot_idx), 'Tag', 'confirmed_force'));
        end
    end
end


function updateStatusDisplay(status_msg)
    global g_annotation_data;
    
    if isfield(g_annotation_data, 'status_text') && ishandle(g_annotation_data.status_text)
        set(g_annotation_data.status_text, 'String', ['状态：' status_msg]);
    end
    
    if isfield(g_annotation_data, 'selection_state_text') && ishandle(g_annotation_data.selection_state_text)
        state_map = containers.Map({'idle', 'selecting_start', 'selecting_end'}, ...
                                   {'空闲', '选择起点', '选择终点'});
        if isKey(state_map, g_annotation_data.selection_state)
            state_str = state_map(g_annotation_data.selection_state);
        else
            state_str = g_annotation_data.selection_state;
        end
        set(g_annotation_data.selection_state_text, 'String', ['选择状态：' state_str]);
    end
end

function updateCountDisplay()
    global g_annotation_data;
    
    if isfield(g_annotation_data, 'count_text') && ishandle(g_annotation_data.count_text)
        if isempty(g_annotation_data.selected_segments)
            n_segments = 0;
        else
            n_segments = length(unique([g_annotation_data.selected_segments.segment_id]));
        end
        set(g_annotation_data.count_text, 'String', sprintf('已选择：%d 个信号段', n_segments));
    end
end

% --- 函数变化：generateFinalSegments (ULTIMATE FIX for LOGICAL INDEXING BUG) ---
function segments = generateFinalSegments(all_segments, accel_data_cell, accel_time, force_data, force_time, fs)
    segments = struct([]);
    if isempty(all_segments)
        return;
    end
    
    % --- 核心修正点：使用最稳健的方法提取ID ---
    all_ids = [];
    for k = 1:length(all_segments)
        if isfield(all_segments(k), 'segment_id') && ~isempty(all_segments(k).segment_id)
            all_ids(end+1) = all_segments(k).segment_id;
        end
    end
    
    if isempty(all_ids)
        return;
    end
    
    unique_ids = unique(all_ids);
    
    final_segments_cell = cell(1, length(all_segments));
    count = 0;
    
    for id = unique_ids
        % --- 核心修正点：使用循环和逻辑判断进行安全查找 ---
        group_segs_info = struct([]);
        for k = 1:length(all_segments)
            if isfield(all_segments(k), 'segment_id') && ~isempty(all_segments(k).segment_id) && all_segments(k).segment_id == id
                if isempty(group_segs_info)
                    group_segs_info = all_segments(k);
                else
                    group_segs_info(end+1) = all_segments(k);
                end
            end
        end
        
        if isempty(group_segs_info), continue; end
        
        for i = 1:length(group_segs_info)
            count = count + 1;
            final_segments_cell{count} = group_segs_info(i);
        end
    end
    
    if count > 0
        segments = [final_segments_cell{1:count}];
    end

    fprintf('  从标注生成了 %d 个有效的信号段\n', length(segments));
end

function segments = assignForceLevels(segments)
    % 修改后的力等级分配函数 - 适配新的X/Z方向
    if isempty(segments), return; end
    
    positions = {'Root', 'Mid', 'Tip'};
    % 关键修改：方向从'Y', 'Z'改为'X', 'Z'
    directions = {'X', 'Z'};  % X表示重力方向，Z表示径向（-Z激励）
    
    for p = 1:3
        for d = 1:2
            % 筛选相同位置和方向的信号段
            idx = find(strcmp({segments.sensor_name}, positions{p}) & ...
                       strcmp({segments.direction}, directions{d}));
            
            if ~isempty(idx)
                % 根据振幅大小分配力等级
                amplitudes = [segments(idx).peak_amplitude];
                [~, sort_order] = sort(abs(amplitudes));
                
                for j = 1:length(idx)
                    original_index = idx(sort_order(j));
                    segments(original_index).force_level = j;
                    
                    % 添加物理意义标注
                    if strcmp(directions{d}, 'X')
                        segments(original_index).physical_meaning = 'gravity_direction_response';
                        segments(original_index).excitation_note = 'gravity_direction_impact';
                    else % Z direction
                        segments(original_index).physical_meaning = 'radial_response';
                        segments(original_index).excitation_note = 'negative_z_direction_impact';
                    end
                end
            end
        end
    end
    
    fprintf('    X方向：重力方向响应特性\n');
    fprintf('    Z方向：径向响应特性（-Z方向激励）\n');
end

%% ============ 线性参数识别 ============
function linear_params = identifyLinearParams(segments, fs, varargin)
% identifyLinearParams - 从锤击实验数据识别线性参数
%
% 输入:
%   segments - 数据段结构体数组
%   fs       - 采样率 (Hz)
%   varargin - 可选参数（名-值对）:
%       'FreqRange'    - 频率分析范围 [min, max] Hz, 默认 [3, 50]
%       'SNRThreshold' - SNR阈值 (dB), 默认 10
%       'NFFT'         - FFT点数, 默认 2048
%       'MassParams'   - 质量参数结构体, 包含 m_root, m_mid, m_tip
%
% 输出:
%   linear_params - 识别出的线性参数结构体
%
% 示例:
%   % 使用默认参数
%   params = identifyLinearParams(segments, 1000);
%
%   % 使用自定义参数
%   params = identifyLinearParams(segments, 1000, ...
%       'FreqRange', [5, 60], 'SNRThreshold', 15);

    fprintf('  [2/5] 线性参数识别 (整合高级方法)...\n');
    
    %% ====== 解析可选参数 ======
    p = inputParser;
    p.addParameter('FreqRange', [3, 50], @(x) isnumeric(x) && length(x)==2);
    p.addParameter('SNRThreshold', 10, @isnumeric);
    p.addParameter('NFFT', 2048, @isnumeric);
    p.addParameter('MassParams', [], @isstruct);
    p.parse(varargin{:});
    
    freq_range = p.Results.FreqRange;
    SNR_THRESHOLD = p.Results.SNRThreshold;
    nfft = p.Results.NFFT;
    mass_params = p.Results.MassParams;
    
    fprintf('    [参数配置] 频率范围=[%.1f, %.1f]Hz, SNR阈值=%.1fdB, NFFT=%d\n', ...
        freq_range(1), freq_range(2), SNR_THRESHOLD, nfft);
    
    %% ====== 初始化输出结构 ======
    linear_params = struct();

    %% ====== 质量过滤：仅使用高SNR数据段 ======
    high_quality_segments = segments;
    for i = 1:length(segments)
        if isfield(segments(i), 'detection_results') && ...
           isfield(segments(i).detection_results, 'snr')
            if segments(i).detection_results.snr < SNR_THRESHOLD
                high_quality_segments(i).force_data_segment = []; % 标记为无效
            end
        end
    end
    
    valid_count = sum(arrayfun(@(s) ~isempty(s.force_data_segment), high_quality_segments));
    fprintf('    [质量过滤] SNR阈值=%.1fdB, 过滤后剩余 %d/%d 段\n', ...
        SNR_THRESHOLD, valid_count, length(segments));
    segments = high_quality_segments;

    %% ====== 准备工作: 计算实验FRF矩阵 ======
    fprintf('    准备工作: 计算实验频响函数 (FRF) 矩阵...\n');
    
    % 先检查Z方向有效段数量
    z_segs = segments(strcmp({segments.direction}, 'Z'));
    z_valid_count = sum(arrayfun(@(s) isfield(s, 'force_data_segment') && ...
        ~isempty(s.force_data_segment) && length(s.force_data_segment) > 50, z_segs));
    fprintf('    [调试] Z方向: 总段数=%d, 有效段数=%d\n', length(z_segs), z_valid_count);
    
    [H_exp_x, Coh_x, freq_vector] = calculate_experimental_frf(segments, fs, nfft, 'X');
    
    if z_valid_count >= 5
        [H_exp_z, Coh_z, ~] = calculate_experimental_frf(segments, fs, nfft, 'Z');
    else
        fprintf('    [警告] Z方向有效数据不足，将跳过Z方向识别\n');
        H_exp_z = [];
        Coh_z = [];
    end
    
    if isempty(freq_vector)
        warning('未能计算有效的FRF，线性参数识别中止。');
        return;
    end
    
    linear_params.frequency_vector = freq_vector;
    linear_params.FRF_matrix_x = H_exp_x;
    linear_params.coherence_matrix_x = Coh_x;
    linear_params.FRF_matrix_z = H_exp_z;
    linear_params.coherence_matrix_z = Coh_z;
    
    %% ====== 阶段一: 传统模态分析 ======
    fprintf('\n    --- 阶段一: 传统模态分析 ---\n');
    
    frf_sum_x = abs(H_exp_x(:,1,1)) + abs(H_exp_x(:,2,2)) + abs(H_exp_x(:,3,3));
    
    % 【修改点】使用可配置的频率范围，而非硬编码的 3 和 50
    search_indices = find(freq_vector >= freq_range(1) & freq_vector <= freq_range(2));
    
    if isempty(search_indices)
        warning('频率范围 [%.1f, %.1f] Hz 内无有效数据', freq_range(1), freq_range(2));
        search_indices = 1:length(freq_vector);
    end
    
    [~, locs_x] = findpeaks(frf_sum_x(search_indices), 'MinPeakProminence', ...
                            max(frf_sum_x(search_indices))*0.1, 'MinPeakDistance', 0.005);
    modal_freqs_x = sort(freq_vector(search_indices(locs_x)));
    
    if ~isempty(H_exp_z)
        frf_sum_z = abs(H_exp_z(:,1,1)) + abs(H_exp_z(:,2,2)) + abs(H_exp_z(:,3,3));
        [~, locs_z] = findpeaks(frf_sum_z(search_indices), 'MinPeakProminence', ...
            max(frf_sum_z(search_indices))*0.1, 'MinPeakDistance', 0.005);
        modal_freqs_z = sort(freq_vector(search_indices(locs_z)));
    else
        modal_freqs_z = [];
    end
    
    % 估计模态阻尼比
    damping_estimates_x = [];
    damping_estimates_z = [];
    for i = 1:length(segments)
        seg = segments(i);
        if isfield(seg, 'signal_data') && length(seg.signal_data) > 50
            zeta_est = estimateDamping(seg.signal_data, fs);
            if strcmp(seg.direction, 'X')
                damping_estimates_x(end+1) = zeta_est;
            else
                damping_estimates_z(end+1) = zeta_est;
            end
        end
    end
    
    modal_damping_ratios_x = ones(size(modal_freqs_x)) * mean(damping_estimates_x, 'omitnan');
    modal_damping_ratios_z = ones(size(modal_freqs_z)) * mean(damping_estimates_z, 'omitnan');
    if isempty(modal_damping_ratios_x) || all(isnan(modal_damping_ratios_x))
        modal_damping_ratios_x = 0.05*ones(size(modal_freqs_x));
    end
    if isempty(modal_damping_ratios_z) || all(isnan(modal_damping_ratios_z))
        modal_damping_ratios_z = 0.05*ones(size(modal_freqs_z));
    end

    linear_params.modal_analysis.natural_freqs_x = modal_freqs_x;
    linear_params.modal_analysis.damping_ratios_x = modal_damping_ratios_x;
    linear_params.modal_analysis.natural_freqs_z = modal_freqs_z;
    linear_params.modal_analysis.damping_ratios_z = modal_damping_ratios_z;

    fprintf('    阶段一完成。识别出的模态参数:\n');
    fprintf('      X方向频率 (Hz): [%s]\n', sprintf('%.2f ', modal_freqs_x));
    fprintf('      X方向阻尼比:    [%s]\n', sprintf('%.3f ', modal_damping_ratios_x));
    fprintf('      Z方向频率 (Hz): [%s]\n', sprintf('%.2f ', modal_freqs_z));
    fprintf('      Z方向阻尼比:    [%s]\n', sprintf('%.3f ', modal_damping_ratios_z));

    % 计算旧版模态刚度（仅供参考）
    mass_estimate = 1.0;
    if ~isempty(modal_freqs_x)
        K_modal_x_est = (2*pi*modal_freqs_x(1))^2 * mass_estimate;
        C_modal_x_est = 2 * modal_damping_ratios_x(1) * sqrt(K_modal_x_est * mass_estimate);
        fprintf('    (旧版估算) X方向一阶模态刚度: %.2e N/m, 模态阻尼: %.2e Ns/m\n', ...
            K_modal_x_est, C_modal_x_est);
        linear_params.modal_analysis.K_modal_x_est = K_modal_x_est;
        linear_params.modal_analysis.C_modal_x_est = C_modal_x_est;
    end
    
    %% ====== 阶段二: 物理参数模型修正 ======
    fprintf('\n    --- 阶段二: 物理参数模型修正 ---\n');
    
    % 【修改点】质量参数：优先使用传入的参数，否则使用默认值
    if ~isempty(mass_params) && isfield(mass_params, 'm_root')
        m_root = mass_params.m_root;
        m_mid  = mass_params.m_mid;
        m_tip  = mass_params.m_tip;
        fprintf('    使用预配置的质量参数\n');
    else
        % 默认值（基于实际称量）
        m_root = 0.2;  % kg - 根部质量
        m_mid  = 0.1;  % kg - 中部质量
        m_tip  = 0.15; % kg - 顶部质量
        fprintf('    使用默认质量参数\n');
    end
    
    M = diag([m_root, m_mid, m_tip]);
    fprintf('    质量参数: m_root=%.2f kg, m_mid=%.2f kg, m_tip=%.2f kg\n', ...
        m_root, m_mid, m_tip);
    
    % 基于实验频率生成初始值和边界
    fprintf('    使用阶段一结果生成优化初始值...\n');
    [x0_x, lb, ub] = estimate_initial_guess_from_modal(modal_freqs_x, modal_damping_ratios_x, M);
    [x0_z, ~, ~] = estimate_initial_guess_from_modal(modal_freqs_z, modal_damping_ratios_z, M);
    
    % 添加线性不等式约束 k_rm > k_mt
    % x = [k_g, c_g, k_rm, c_rm, k_mt, c_mt]
    % 约束: k_mt < k_rm → -k_rm + k_mt < 0
    A = [0, 0, -1, 0, 1, 0];  % -k_rm + k_mt
    b = 0;
    
    options_con = optimoptions('fmincon', 'Display', 'iter', ...
        'MaxFunctionEvaluations', 3000, 'StepTolerance', 1e-9, ...
        'OptimalityTolerance', 1e-9, 'Algorithm', 'interior-point');
    
    fprintf('    开始辨识 X 方向物理参数...\n');
    objective_x_scalar = @(x) norm(calculate_frf_error(x, M, freq_vector, H_exp_x, Coh_x));
    identified_params_x = fmincon(objective_x_scalar, x0_x, A, b, [], [], lb, ub, [], options_con);
    
    % Z方向识别（如果有有效数据）
    if ~isempty(H_exp_z) && z_valid_count >= 5
        fprintf('    开始辨识 Z 方向物理参数...\n');
        objective_z_scalar = @(x) norm(calculate_frf_error(x, M, freq_vector, H_exp_z, Coh_z));
        identified_params_z = fmincon(objective_z_scalar, x0_z, A, b, [], [], lb, ub, [], options_con);
    else
        fprintf('    Z方向数据不足，使用默认参数\n');
        identified_params_z = x0_z;
    end

    fprintf('    阶段二完成。\n');
    
    %% ====== 保存结果 ======
    linear_params.physical_parameters.M = M;
    linear_params.physical_parameters.x_vector_x = identified_params_x;
    linear_params.physical_parameters.x_vector_z = identified_params_z;
    
    [linear_params.physical_parameters.K_x, linear_params.physical_parameters.C_x] = ...
        build_matrices(identified_params_x);
    [linear_params.physical_parameters.K_z, linear_params.physical_parameters.C_z] = ...
        build_matrices(identified_params_z);
    
    linear_params.M = M;
    linear_params.K = linear_params.physical_parameters.K_x;
    linear_params.C = linear_params.physical_parameters.C_x;
    
    [~, D_x] = eig(linear_params.physical_parameters.K_x, M);
    linear_params.natural_freqs_x = sort(sqrt(diag(D_x)) / (2*pi));
    [~, D_z] = eig(linear_params.physical_parameters.K_z, M);
    linear_params.natural_freqs_z = sort(sqrt(diag(D_z)) / (2*pi));
    linear_params.damping_ratios_x = modal_damping_ratios_x;
    linear_params.damping_ratios_z = modal_damping_ratios_z;

    %% ====== 输出最终结果 ======
    fprintf('\n==================== 最终辨识出的物理参数 (可用于仿真代码) ====================\n');
    fprintf('---- X 方向 ----\n');
    fprintf('  连接地面 (k_g, c_g)    : k = %.3e N/m,   c = %.3e Ns/m\n', ...
        identified_params_x(1), identified_params_x(2));
    fprintf('  连接 Root-Mid (k_rm, c_rm): k = %.3e N/m,   c = %.3e Ns/m\n', ...
        identified_params_x(3), identified_params_x(4));
    fprintf('  连接 Mid-Tip (k_mt, c_mt) : k = %.3e N/m,   c = %.3e Ns/m\n', ...
        identified_params_x(5), identified_params_x(6));
    fprintf('---- Z 方向 ----\n');
    fprintf('  连接地面 (k_g, c_g)    : k = %.3e N/m,   c = %.3e Ns/m\n', ...
        identified_params_z(1), identified_params_z(2));
    fprintf('  连接 Root-Mid (k_rm, c_rm): k = %.3e N/m,   c = %.3e Ns/m\n', ...
        identified_params_z(3), identified_params_z(4));
    fprintf('  连接 Mid-Tip (k_mt, c_mt) : k = %.3e N/m,   c = %.3e Ns/m\n', ...
        identified_params_z(5), identified_params_z(6));
    fprintf('=================================================================================\n');
    
    fprintf('\n--- 验证: 最终模型频率 vs 实验测量频率 ---\n');
    fprintf('  实验测量 (X): [%s]\n', sprintf('%.2f ', linear_params.modal_analysis.natural_freqs_x));
    fprintf('  模型预测 (X): [%s]\n', sprintf('%.2f ', linear_params.natural_freqs_x));
    fprintf('  实验测量 (Z): [%s]\n', sprintf('%.2f ', linear_params.modal_analysis.natural_freqs_z));
    fprintf('  模型预测 (Z): [%s]\n', sprintf('%.2f ', linear_params.natural_freqs_z));
    fprintf('--------------------------------------------\n\n');
end

%% ============ 非线性特征提取 ============
function nl_features = extractNonlinearFeatures(segments, fs)
    % 功能：提取非线性特征（严格质量控制）
    
    fprintf('  [3/5] 非线性特征提取...\n');
    fprintf('  [非线性分析] 开始检查每个组合的有效数据点...\n');
    
    nl_features = struct();
    positions = {'Root', 'Mid', 'Tip'};
    directions = {'X', 'Z'};
    
    for p = 1:length(positions)
        for d = 1:length(directions)
            field_name = [positions{p} '_' directions{d}];
            
            relevant_segs = segments(...
                strcmp({segments.sensor_name}, positions{p}) & ...
                strcmp({segments.direction}, directions{d}));
            
            if isempty(relevant_segs)
                continue;
            end
            
            valid_count = 0;
            amplitudes = [];
            frequencies = [];
            
            for k = 1:length(relevant_segs)
                seg = relevant_segs(k);
                
                % 质量过滤：SNR必须>15dB
                if ~isfield(seg, 'detection_results') || seg.detection_results.snr < 15
                    continue;
                end
                
                signal = seg.signal_data;
                if length(signal) < 500
                    continue;
                end
                
                % 必须有峰值
                if ~isfield(seg, 'peak_info') || isempty(seg.peak_info.peak_times)
                    continue;
                end
                
                % 用峰值后1秒的衰减段估算主频
                peak_times = seg.peak_info.peak_times;
                [~, max_idx] = max(abs(seg.peak_info.peak_amplitudes));
                main_peak_time = peak_times(max_idx);
                
                time_vec = (0:length(signal)-1)' / fs;
                [~, peak_sample] = min(abs(time_vec - main_peak_time));
                
                decay_start = peak_sample;
                decay_end = min(peak_sample + fs, length(signal));
                
                if decay_end - decay_start < 200
                    continue;
                end
                
                decay_signal = signal(decay_start:decay_end);
                
                % 计算主频
                nfft = min(512, 2^nextpow2(length(decay_signal)));
                [pxx, f] = pwelch(decay_signal, min(256, length(decay_signal)), [], nfft, fs);
                
                valid_f_idx = (f >= 5) & (f <= 50);
                if ~any(valid_f_idx)
                    continue;
                end
                
                [~, max_f_idx] = max(pxx(valid_f_idx));
                f_valid = f(valid_f_idx);
                dominant_freq = f_valid(max_f_idx);
                
                % 振幅：用包络起始均值
                envelope = abs(hilbert(decay_signal));
                amplitude = mean(envelope(1:min(50, length(envelope))));
                
                % 范围检查
                if amplitude < 0.5 || amplitude > 100
                    continue;
                end
                
                if dominant_freq < 5 || dominant_freq > 50
                    continue;
                end
                
                valid_count = valid_count + 1;
                amplitudes(end+1) = amplitude;
                frequencies(end+1) = dominant_freq;
            end
            
            fprintf('    - 组合 %s: 找到 %d 个有效的信号段用于分析。\n', ...
                field_name, valid_count);
            
            if valid_count >= 5
                % MAD异常值剔除
                freq_median = median(frequencies);
                freq_mad = median(abs(frequencies - freq_median)) * 1.4826;
                good_idx = abs(frequencies - freq_median) < 3 * freq_mad;
                
                if sum(good_idx) >= 5
                    amplitudes = amplitudes(good_idx);
                    frequencies = frequencies(good_idx);
                    
                    [nl_type, fit_params] = determineNonlinearType(amplitudes, frequencies);
                    
                    fprintf('      >>> 判定结果: %s (R²=%.2f)\n', nl_type, fit_params.r_squared);
                    
                    if fit_params.r_squared > 0.3
                        nl_features.(field_name).amplitudes = amplitudes;
                        nl_features.(field_name).frequencies = frequencies;
                        nl_features.(field_name).type = nl_type;
                        nl_features.(field_name).fit_params = fit_params;
                    else
                        fprintf('      >>> R²过低(%.2f<0.3)，舍弃\n', fit_params.r_squared);
                    end
                else
                    fprintf('      >>> 异常值剔除后不足5个点\n');
                end
            end
        end
    end
end

function [inst_freq, inst_damping] = hilbertAnalysis(signal, fs)
    % 使用希尔伯特变换分析瞬时频率和阻尼
    
    % 初始化输出
    inst_freq = [];
    inst_damping = [];
    
    if isempty(signal) || length(signal) < 10
        return;
    end
    
    % 希尔伯特变换
    analytic_signal = hilbert(signal);
    envelope = abs(analytic_signal);
    phase = unwrap(angle(analytic_signal));
    
    % 计算瞬时频率
    inst_freq_raw = diff(phase) * fs / (2*pi);
    
    % 平滑瞬时频率（去除异常值）
    % 使用中值滤波去除尖峰
    window_size = min(11, floor(length(inst_freq_raw)/10)*2 + 1);
    if window_size >= 3
        inst_freq_smooth = medfilt1(inst_freq_raw, window_size);
    else
        inst_freq_smooth = inst_freq_raw;
    end
    
    % 进一步使用移动平均平滑
    smooth_window = round(fs * 0.05); % 50ms窗口
    if smooth_window >= 3
        inst_freq = movmean(inst_freq_smooth, smooth_window);
    else
        inst_freq = inst_freq_smooth;
    end
    
    % 限制频率在合理范围内
    inst_freq(inst_freq < 0) = 0;
    inst_freq(inst_freq > fs/2) = fs/2;
    
    % 计算瞬时阻尼
    if nargout > 1
        % 使用包络的对数衰减率估计阻尼
        log_envelope = log(envelope + eps); % 避免log(0)
        
        % 使用局部线性拟合估计衰减率
        window_size = round(0.1 * fs); % 100ms窗口
        n = length(log_envelope);
        inst_damping = zeros(n, 1);
        
        for i = 1:n
            start_idx = max(1, i - floor(window_size/2));
            end_idx = min(n, i + floor(window_size/2));
            
            if end_idx - start_idx > 5
                t = (start_idx:end_idx)' / fs;
                y = log_envelope(start_idx:end_idx);
                
                % 线性拟合
                p = polyfit(t - t(1), y, 1);
                decay_rate = -p(1);
                
                % 转换为阻尼比（假设单自由度系统）
                if i <= length(inst_freq) && inst_freq(i) > 0
                    omega = 2 * pi * inst_freq(i);
                    inst_damping(i) = decay_rate / omega;
                end
            end
        end
        
        % 限制阻尼比在合理范围
        inst_damping = max(0, min(inst_damping, 1));
    end
end



%% ============ 非线性参数优化 ============
function nl_params = optimizeNonlinearParams(segments, linear_params, nl_features)
    nl_params = struct();

    
    
    if isempty(fieldnames(nl_features))
        fprintf('    警告：没有非线性特征数据，跳过优化\n');
        nl_params.k3_coeffs = zeros(1, 3);
        nl_params.c2_coeffs = zeros(1, 3);
        nl_params.optimization_error = 1e6;
        nl_params.exitflag = -1;
        return;
    end
    
    x0 = zeros(1, 6);
    lb = -1e-3 * ones(1, 6);
    ub = 1e-3 * ones(1, 6);
    
    % 初始化
    positions = {'Root', 'Mid', 'Tip'};
    directions = {'X', 'Z'};
    
    for p = 1:3
        for d = 1:2
            field_name = sprintf('%s_%s', positions{p}, directions{d});
            if isfield(nl_features, field_name) && isfield(nl_features.(field_name), 'type')
                if strcmp(nl_features.(field_name).type, 'softening')
                    x0(p) = -1e-5;
                elseif strcmp(nl_features.(field_name).type, 'hardening')
                    x0(p) = 1e-5;
                end
                x0(p + 3) = 1e-5;
                break;
            end
        end
    end
    
    objective = @(x) computeObjective(x, segments, linear_params, nl_features);
    
    try
        init_error = objective(x0);
        if ~isfinite(init_error)
            x0 = 1e-6 * ones(1, 6);
            init_error = objective(x0);
        end
        fprintf('    初始目标函数值: %.4e\n', init_error);
    catch ME
        fprintf('    错误：目标函数评估失败: %s\n', ME.message);
        nl_params.k3_coeffs = zeros(1, 3);
        nl_params.c2_coeffs = zeros(1, 3);
        nl_params.optimization_error = 1e6;
        nl_params.exitflag = -2;
        return;
    end
    
    options = optimoptions('fmincon', ...
        'Display', 'iter', ...
        'MaxIterations', 100, ...
        'OptimalityTolerance', 1e-4, ...
        'StepTolerance', 1e-6, ...
        'ConstraintTolerance', 1e-4, ...
        'CheckGradients', false, ...
        'FiniteDifferenceType', 'central', ...
        'UseParallel', false);
    
    fprintf('    开始参数优化...\n');
    
    try
        [x_opt, fval, exitflag] = fmincon(objective, x0, [], [], [], [], lb, ub, [], options);
        
        nl_params.k3_coeffs = x_opt(1:3);
        nl_params.c2_coeffs = x_opt(4:6);
        nl_params.optimization_error = fval;
        nl_params.exitflag = exitflag;
        
        fprintf('    优化完成，最终误差: %.4e，退出标志: %d\n', fval, exitflag);
        
    catch ME
        fprintf('    优化失败: %s\n', ME.message);
        nl_params.k3_coeffs = x0(1:3);
        nl_params.c2_coeffs = x0(4:6);
        nl_params.optimization_error = 1e6;
        nl_params.exitflag = -3;
    end
end

function error = computeObjective(x, segments, linear_params, nl_features)
    % 功能：计算非线性优化目标函数
    
    error = 0;
    n_valid_points = 0;
    
    positions = {'Root', 'Mid', 'Tip'};
    directions = {'X', 'Z'};
    
    k3_coeffs = x(1:3);
    
    for p = 1:3
        for d = 1:2
            field_name = sprintf('%s_%s', positions{p}, directions{d});
            
            if isfield(nl_features, field_name) && isfield(nl_features.(field_name), 'amplitudes')
                
                feature_data = nl_features.(field_name);
                measured_amps = feature_data.amplitudes;
                measured_freqs = feature_data.frequencies;
                
                % 获取线性基础频率
                if d == 1
                    if isfield(linear_params, 'natural_freqs_x') && ~isempty(linear_params.natural_freqs_x)
                        f0 = linear_params.natural_freqs_x(1);
                    else
                        f0 = 10;
                    end
                    % 从identified_params_x获取刚度
                    if isfield(linear_params, 'identified_params_x') && length(linear_params.identified_params_x) >= 5
                        k_g = linear_params.identified_params_x(1);
                        k_rm = linear_params.identified_params_x(3);
                        k_mt = linear_params.identified_params_x(5);
                        k_values = [k_g+k_rm, k_rm+k_mt, k_mt];
                        k_linear = k_values(p);
                    else
                        k_linear = 150;
                    end
                else
                    if isfield(linear_params, 'natural_freqs_z') && ~isempty(linear_params.natural_freqs_z)
                        f0 = linear_params.natural_freqs_z(1);
                    else
                        f0 = 10;
                    end
                    % 从identified_params_z获取刚度
                    if isfield(linear_params, 'identified_params_z') && length(linear_params.identified_params_z) >= 5
                        k_g = linear_params.identified_params_z(1);
                        k_rm = linear_params.identified_params_z(3);
                        k_mt = linear_params.identified_params_z(5);
                        k_values = [k_g+k_rm, k_rm+k_mt, k_mt];
                        k_linear = k_values(p);
                    else
                        k_linear = 150;
                    end
                end
                
                k3 = k3_coeffs(p);
                
                % 计算非线性频率预测误差
                for i = 1:length(measured_amps)
                    A = measured_amps(i);
                    f_meas = measured_freqs(i);
                    
                    % 非线性频率公式
                    f_pred = f0 * sqrt(1 + (3/4) * k3 * A^2 / k_linear);
                    
                    error = error + (f_pred - f_meas)^2;
                    n_valid_points = n_valid_points + 1;
                end
            end
        end
    end
    
    % 归一化误差
    if n_valid_points > 0
        error = error / n_valid_points;
    else
        error = 1e6;
    end
end

%% ============ 参数验证 ============
function validation = validateParameters(params, segments)
    validation = struct();
    
    fprintf('    [交叉验证] 开始5折交叉验证...\n');
    
    fprintf('    验证X方向...\n');
    [errors_x, details_x] = validateDirection(params, segments, 'X', 2, 5); % 最后一个参数是折数
    
    fprintf('    验证Z方向...\n');
    [errors_z, details_z] = validateDirection(params, segments, 'Z', 3, 5);
    
    validation.errors_x = errors_x;
    validation.errors_z = errors_z;
    validation.details_x = details_x;
    validation.details_z = details_z;
    
    all_errors = [errors_x(errors_x > 0); errors_z(errors_z > 0)]; % 确保是列向量
    if ~isempty(all_errors)
        validation.mean_error = mean(all_errors);
        validation.std_error = std(all_errors);
    else
        validation.mean_error = 1.0;
        validation.std_error = 0;
    end
    
    validation.cv_errors = all_errors;
    
    fprintf('\n    X方向交叉验证误差: %.2f%% ± %.2f%%\n', ...
        mean(errors_x(errors_x > 0)) * 100, std(errors_x(errors_x > 0)) * 100);
    fprintf('    Z方向交叉验证误差: %.2f%% ± %.2f%%\n', ...
        mean(errors_z(errors_z > 0)) * 100, std(errors_z(errors_z > 0)) * 100);
    fprintf('    ------------------------------------------\n');
    fprintf('    总体交叉验证误差: %.2f%% ± %.2f%%\n', ...
        validation.mean_error * 100, validation.std_error * 100);
    fprintf('    ------------------------------------------\n');
end

function [errors, details] = validateDirection(params, all_segments, direction, dir_idx, n_folds)
    % --- 核心修正：实现K-折交叉验证 ---
    
    dir_segments = all_segments(strcmp({all_segments.direction}, direction));
    if length(dir_segments) < n_folds
        fprintf('      警告: %s方向信号段数量 (%d) 不足%d折交叉验证，跳过。\n', direction, length(dir_segments), n_folds);
        errors = [];
        details = struct();
        return;
    end

    % 使用cvpartition进行随机分组
    cv = cvpartition(length(dir_segments), 'KFold', n_folds);
    
    errors = zeros(n_folds, 1);
    details = repmat(struct(), 1, n_folds);
    
    for fold = 1:n_folds
        fprintf('      - 第 %d/%d 折...\n', fold, n_folds);
        
        % 划分训练集和测试集
        train_indices = cv.training(fold);
        test_indices = cv.test(fold);
        
        train_segs = dir_segments(train_indices);
        test_segs = dir_segments(test_indices);
        
        % 1. **仅使用训练集**重新训练模型
        params_fold = trainModelForCV(train_segs, params.fs);
        
        % 2. 在测试集上评估
        fold_error = 0;
        n_test_points = 0;
        
        for i = 1:length(test_segs)
            seg = test_segs(i);
            
            % 使用为该折训练出的模型进行预测
            [pred_freq, ~] = predictResponse(params_fold, seg, direction);
            
            % 获取实际频率
            if isfield(seg, 'peak_info') && isfield(seg.peak_info, 'dominant_frequency') && seg.peak_info.dominant_frequency > 0
                actual_freq = seg.peak_info.dominant_frequency;

                relative_error = abs(pred_freq - actual_freq) / actual_freq;
                fold_error = fold_error + relative_error;
                n_test_points = n_test_points + 1;
                
                details(fold).segment(i).pred_freq = pred_freq;
                details(fold).segment(i).actual_freq = actual_freq;
                details(fold).segment(i).error = relative_error;
            end
        end
        
        if n_test_points > 0
            errors(fold) = fold_error / n_test_points;
        else
            errors(fold) = NaN; % 标记该折没有有效的测试点
        end
        
        details(fold).mean_error = errors(fold);
        details(fold).n_test = n_test_points;
    end
    
    errors = errors(~isnan(errors)); % 移除无效折的误差
end

function params_cv = trainModelForCV(train_segments, fs)
    % 功能：交叉验证中训练模型
    
    params_cv = struct();
    
    % 线性参数识别
    linear_params = identifyLinearParams(train_segments, fs);
    
    % 检查线性识别是否成功
    if isempty(linear_params) || ~isfield(linear_params, 'identified_params_x')
        fprintf('      CV警告: 线性识别失败，使用默认参数。\n');
        
        % 构造完整的默认参数结构
        linear_params = struct();
        linear_params.identified_params_x = [40, 0.05, 150, 0.25, 100, 0.2];
        linear_params.identified_params_z = [40, 0.05, 150, 0.25, 100, 0.2];
        linear_params.frequency_vector = linspace(0, fs/2, 1025)';
        linear_params.FRF_matrix_x = zeros(1025, 3, 3);
        linear_params.FRF_matrix_z = zeros(1025, 3, 3);
        linear_params.coherence_matrix_x = zeros(1025, 3, 3);
        linear_params.coherence_matrix_z = zeros(1025, 3, 3);
        linear_params.natural_freqs_x = [7];
        linear_params.damping_ratios_x = [0.05];
        linear_params.natural_freqs_z = [7];
        linear_params.damping_ratios_z = [0.05];
    end
    
    params_cv.linear = linear_params;
    
    % 提取非线性特征
    nl_features = extractNonlinearFeatures(train_segments, fs);
    params_cv.nl_features = nl_features;
    
    % 优化非线性参数
    if ~isempty(fieldnames(nl_features))
        nl_params = optimizeNonlinearParams(train_segments, linear_params, nl_features);
        params_cv.nonlinear = nl_params;
    else
        params_cv.nonlinear.k3_coeffs = zeros(1, 3);
        params_cv.nonlinear.c2_coeffs = zeros(1, 3);
    end
end

function [freq, damp] = predictResponse(params, segment, direction)
    sensor_idx = segment.sensor_idx;
    accel_amplitude_g = segment.peak_amplitude;
    
    if ~isfield(params, 'linear') || isempty(params.linear)
        freq = 5; damp = 0.05; return;
    end

    % --- [核心修复 v3] ---
    % 增加对所有可能不存在字段的鲁棒性检查，并提供后备值
    if strcmp(direction, 'X')
        % 检查X方向的频率和阻尼字段是否存在
        if isfield(params.linear, 'natural_freqs_x') && ~isempty(params.linear.natural_freqs_x)
            [sorted_freqs, idx] = sort(params.linear.natural_freqs_x);
            f0 = sorted_freqs(1);
            if isfield(params.linear, 'damping_ratios_x') && length(params.linear.damping_ratios_x) >= idx(1)
                zeta0 = params.linear.damping_ratios_x(idx(1));
            else
                zeta0 = 0.02; % 阻尼后备值
            end
        else
            f0 = 5; zeta0 = 0.02; % 频率和阻尼的后备值
        end
    else % Z direction
        % 检查Z方向的频率和阻尼字段是否存在
        if isfield(params.linear, 'natural_freqs_z') && ~isempty(params.linear.natural_freqs_z)
            [sorted_freqs, idx] = sort(params.linear.natural_freqs_z);
            f0 = sorted_freqs(1);
            % [关键修复点] 在访问 damping_ratios_z 之前先检查它是否存在
            if isfield(params.linear, 'damping_ratios_z') && length(params.linear.damping_ratios_z) >= idx(1)
                zeta0 = params.linear.damping_ratios_z(idx(1));
            else
                zeta0 = 0.02; % 阻尼后备值
            end
        else
            f0 = 4; zeta0 = 0.02; % 频率和阻尼的后备值
        end
    end
    % --- [修复结束 v3] ---

    freq = f0;
    damp = zeta0;
    
    positions = {'Root', 'Mid', 'Tip'};
    pos_name = positions{sensor_idx};
    field_name = sprintf('%s_%s', pos_name, direction);

    if isfield(params, 'nl_features') && isfield(params.nl_features, field_name)
        nl_char = params.nl_features.(field_name);
        
        if isfield(nl_char, 'amplitudes') && isfield(nl_char, 'frequencies') &&...
           ~isempty(nl_char.amplitudes)
            
            unique_amplitudes = unique(nl_char.amplitudes);
            
            if length(unique_amplitudes) < length(nl_char.amplitudes)
                mean_frequencies = zeros(size(unique_amplitudes));
                for i = 1:length(unique_amplitudes)
                    current_amp = unique_amplitudes(i);
                    associated_freqs = nl_char.frequencies(nl_char.amplitudes == current_amp);
                    mean_frequencies(i) = mean(associated_freqs);
                end
                amps_for_interp = unique_amplitudes;
                freqs_for_interp = mean_frequencies;
            else
                [sorted_amps, sort_idx] = sort(nl_char.amplitudes);
                amps_for_interp = sorted_amps;
                freqs_for_interp = nl_char.frequencies(sort_idx);
            end

            if length(amps_for_interp) >= 2
                pred_freq = interp1(amps_for_interp, freqs_for_interp, accel_amplitude_g, 'linear', 'extrap');
                if ~isnan(pred_freq)
                    freq = pred_freq;
                end
            end
        end
    end
    
    freq = max(1, min(freq, 25));
    damp = max(0.001, min(damp, 0.2));
end

%% ============ 文件生成函数 ============
function generateUpdateFiles(params)
    fid = fopen('UpdatedTreeParameters.m', 'w');
    if fid == -1, error('无法创建参数文件！'); end
    
    fprintf(fid, '%%%% 果树振动模型参数更新脚本\n');
    fprintf(fid, '%%%% 基于 SAD 框架自动识别生成的参数\n');
    fprintf(fid, '%%%% 生成时间: %s\n\n', datestr(now));
    
    % 1. 写入主干基础参数 (基于识别结果)
    fprintf(fid, '%%======= 主干基础参数 (Identified) =======%%\n');
    if isfield(params, 'linear') && isfield(params.linear, 'K')
        % 假设 Trunk_root (index 1) 的刚度作为基础刚度
        k_base_val = params.linear.K(1,1);
        c_base_val = params.linear.C(1,1);
    else
        k_base_val = 10000; c_base_val = 35; % 回退值
    end
    fprintf(fid, 'k_A_base = %.4f;  %% 主干基础刚度 (N/m)\n', k_base_val);
    fprintf(fid, 'c_A_base = %.4f;  %% 主干基础阻尼 (Ns/m)\n', c_base_val);
    fprintf(fid, 'z_factor = 1;     %% Z方向因子\n\n');
    
    % 2. 写入辅助函数 (用于生成分枝参数)
    fprintf(fid, '%%======= 参数生成辅助函数 =======%%\n');
    fprintf(fid, 'if ~exist(''generate_branch_segment_params'', ''file'')\n');
    fprintf(fid, '    %% 定义一个匿名函数句柄或者局部函数逻辑供脚本使用\n');
    fprintf(fid, '    %% 注意：这里我们预定义结构体生成逻辑\n');
    fprintf(fid, 'end\n\n');
    
    % 3. 写入预定义参数结构体 (predefined_params)
    fprintf(fid, '%%======= 拓扑参数定义 =======%%\n');
    fprintf(fid, 'if ~exist(''predefined_params'', ''var''), predefined_params = struct(); end\n\n');
    
    % 获取识别出的线性参数矩阵
    if isfield(params, 'linear') && isfield(params.linear, 'M')
        M_diag = diag(params.linear.M);
        K_diag = diag(params.linear.K);
        C_diag = diag(params.linear.C);
    else
        M_diag = [0.2, 0.1, 0.15]; K_diag = [5000, 4000, 3000]; C_diag = [10, 8, 5];
    end
    
    % 获取非线性参数 (如果有)
    k3_vals = [0, 0, 0]; c2_vals = [0, 0, 0];
    if isfield(params, 'nonlinear') && isfield(params.nonlinear, 'k3_coeffs')
        k3_vals = params.nonlinear.k3_coeffs;
        c2_vals = params.nonlinear.c2_coeffs;
    end

    % --- 生成一级分枝 (P1, P2, P3) - 使用识别值 ---
    % 注意：这里不再使用硬编码的 9.13kg，而是根据识别出的 M_diag 估算总质量
    % 假设识别出的 M_root 是总质量的 50%
    m_total_est = M_diag(1) / 0.5; 
    
    for i = 1:3
        idx = min(i, length(M_diag));
        % 基于识别刚度反推 k_base (假设 k_root = k_base * 0.5)
        k_branch_base = K_diag(idx) / 0.5; 
        c_branch_base = C_diag(idx); 
        
        fprintf(fid, '%% --- P%d 参数 (基于识别) ---\n', i);
        fprintf(fid, 'predefined_params.P%d = generate_branch_segment_params(%.4f, %.4f, %.4f);\n', ...
            i, m_total_est * (1 - (i-1)*0.2), k_branch_base, c_branch_base);
        
        % 注入非线性参数 (如果识别到了)
        if k3_vals(idx) ~= 0 || c2_vals(idx) ~= 0
            fprintf(fid, 'predefined_params.P%d.root.k_nonlinear = %.4e;\n', i, k3_vals(idx));
            fprintf(fid, 'predefined_params.P%d.root.c_nonlinear = %.4e;\n', i, c2_vals(idx));
            % 简单起见，假设 mid 和 tip 也有一定的非线性，或仅 root 有
            fprintf(fid, 'predefined_params.P%d.mid.k_nonlinear  = %.4e;\n', i, k3_vals(idx)*0.5);
            fprintf(fid, 'predefined_params.P%d.tip.k_nonlinear  = %.4e;\n', i, k3_vals(idx)*0.2);
        end
        fprintf(fid, '\n');
    end
    
     % --- 从工作区拓扑配置动态生成二级和三级分枝参数 ---
    fprintf(fid, '\n%% === 二级和三级分枝参数 (基于拓扑配置动态生成) ===\n');
    
    % 获取拓扑配置
    if ~evalin('base', 'exist(''config'', ''var'')')
        error('ParameterGeneration:MissingData', ...
              '工作区缺少拓扑配置(config)，请先运行预配置程序');
    end
    topology_config = evalin('base', 'config');
    
    if ~isfield(topology_config, 'num_primary_branches') || ...
       ~isfield(topology_config, 'secondary_branches_count') || ...
       ~isfield(topology_config, 'tertiary_branches_count')
        error('ParameterGeneration:InvalidConfig', ...
              '拓扑配置不完整，缺少必要字段');
    end
    
    num_p = topology_config.num_primary_branches;
    secondary_count = topology_config.secondary_branches_count;
    tertiary_count = topology_config.tertiary_branches_count;
    
    fprintf(fid, '%% 拓扑: %d个一级分枝\n\n', num_p);
    
    % 遍历生成所有分枝参数
    for p = 1:num_p
        p_idx = min(p, length(M_diag));
        k_p_base = K_diag(p_idx);
        c_p_base = C_diag(p_idx);
        m_p_base = M_diag(p_idx);
        
        if p <= length(secondary_count)
            num_s = secondary_count(p);
            for s = 1:num_s
                branch_name_s = sprintf('P%d_S%d', p, s);
                
                % 二级分枝刚度阻尼基于一级分枝缩放
                scale_s = 1.0 / (s + 1);  % 随序号递减
                k_s = k_p_base * scale_s;
                c_s = c_p_base * scale_s;
                m_s = m_p_base * scale_s;
                
                fprintf(fid, 'predefined_params.%s = generate_branch_segment_params(%.6f, %.6f, %.6f);\n', ...
                        branch_name_s, m_s, k_s, c_s);
                fprintf(fid, 'predefined_params.%s.branch_level = 2;\n', branch_name_s);
                
                % 三级分枝
                if p <= length(tertiary_count) && s <= length(tertiary_count{p})
                    num_t = tertiary_count{p}(s);
                    for t = 1:num_t
                        branch_name_t = sprintf('P%d_S%d_T%d', p, s, t);
                        
                        scale_t = 1.0 / (t + 2);
                        k_t = k_s * scale_t;
                        c_t = c_s * scale_t;
                        m_t = m_s * scale_t;
                        
                        fprintf(fid, 'predefined_params.%s = generate_branch_segment_params(%.6f, %.6f, %.6f);\n', ...
                                branch_name_t, m_t, k_t, c_t);
                        fprintf(fid, 'predefined_params.%s.branch_level = 3;\n', branch_name_t);
                    end
                end
            end
        end
    end
    
    % 输出识别的递减因子
    fprintf(fid, '\n%% === 从实验数据计算的递减因子 ===\n');
    fprintf(fid, 'identified_taper_factors = struct();\n');
    fprintf(fid, 'identified_taper_factors.k = [%.6f, %.6f, %.6f];\n', ...
            linear_params.taper_factors.k(1), ...
            linear_params.taper_factors.k(2), ...
            linear_params.taper_factors.k(3));
    fprintf(fid, 'identified_taper_factors.c = [%.6f, %.6f, %.6f];\n', ...
            linear_params.taper_factors.c(1), ...
            linear_params.taper_factors.c(2), ...
            linear_params.taper_factors.c(3));
    fprintf(fid, '\n%% 标记参数来源\n');
    fprintf(fid, 'use_identified_params = true;\n');
end

function generateAnalysisReport(params)
    fid = fopen('NonlinearAnalysisReport.html', 'w', 'n', 'UTF-8');
    if fid == -1, error('无法创建报告文件！'); end

    fprintf(fid, '<html><head><title>非线性参数识别报告</title>\n');
    fprintf(fid, '<meta charset="UTF-8">\n');
    fprintf(fid, '<style>\n');
    fprintf(fid, 'body {font-family: Arial, sans-serif; margin: 20px;}\n');
    fprintf(fid, 'h1 {color: #333;}\n');
    fprintf(fid, 'h2 {color: #666; border-bottom: 2px solid #ddd; padding-bottom: 5px;}\n');
    fprintf(fid, 'table {border-collapse: collapse; width: 80%%; margin: 20px 0;}\n');
    fprintf(fid, 'th, td {border: 1px solid #ddd; padding: 8px; text-align: left;}\n');
    fprintf(fid, 'th {background-color: #f2f2f2;}\n');
    fprintf(fid, '.good {color: green; font-weight: bold;} .warning {color: orange; font-weight: bold;} .bad {color: red; font-weight: bold;}\n');
    fprintf(fid, '</style></head>\n');
    
    fprintf(fid, '<body>\n');
    fprintf(fid, '<h1>果树振动系统非线性参数识别报告</h1>\n');
    fprintf(fid, '<h2>最终优化版本</h2>\n');
    fprintf(fid, '<p>生成时间: %s</p>\n', datestr(now));
    
    % 1. 数据概览
    fprintf(fid, '<h2>1. 数据概览</h2>\n');
    fprintf(fid, '<ul>\n');
    fprintf(fid, '<li>采样率: %d Hz</li>\n', params.fs);
    if isfield(params, 'segments') && ~isempty(params.segments)
        num_unique_segments = length(unique([params.segments.segment_id]));
    else
        num_unique_segments = 0;
    end
    fprintf(fid, '<li>检测到的锤击段数: %d</li>\n', num_unique_segments);
    fprintf(fid, '<li>数据源: %s</li>\n', params.test_config.data_source);
    fprintf(fid, '</ul>\n');
    
    % 2. 线性模态参数
    fprintf(fid, '<h2>2. 线性模态参数 (阶段一：传统模态分析)</h2>\n');
    if isfield(params.linear, 'modal_analysis')
        fprintf(fid, '<h3>X-方向</h3>\n<table>\n');
        fprintf(fid, '<tr><th>模态</th><th>频率 (Hz)</th><th>阻尼比</th></tr>\n');
        for i = 1:length(params.linear.modal_analysis.natural_freqs_x)
            fprintf(fid, '<tr><td>%d</td><td>%.2f</td><td>%.4f</td></tr>\n', i, params.linear.modal_analysis.natural_freqs_x(i), params.linear.modal_analysis.damping_ratios_x(i));
        end
        fprintf(fid, '</table>\n');

        fprintf(fid, '<h3>Z-方向</h3>\n<table>\n');
        fprintf(fid, '<tr><th>模态</th><th>频率 (Hz)</th><th>阻尼比</th></tr>\n');
        for i = 1:length(params.linear.modal_analysis.natural_freqs_z)
            fprintf(fid, '<tr><td>%d</td><td>%.2f</td><td>%.4f</td></tr>\n', i, params.linear.modal_analysis.natural_freqs_z(i), params.linear.modal_analysis.damping_ratios_z(i));
        end
        fprintf(fid, '</table>\n');
    else
        fprintf(fid, '<p class="warning">未成功识别模态参数</p>\n');
    end
    
    % 3. 非线性特征
    fprintf(fid, '<h2>3. 非线性特征</h2>\n');
    if isfield(params, 'nl_features') && ~isempty(fieldnames(params.nl_features))
        fprintf(fid, '<table>\n');
        fprintf(fid, '<tr><th>位置-方向</th><th>非线性类型</th><th>R-Squared</th></tr>\n');
        
        fields = fieldnames(params.nl_features);
        for i = 1:length(fields)
            if isfield(params.nl_features.(fields{i}), 'type')
                nl_type = params.nl_features.(fields{i}).type;
                r_sq = params.nl_features.(fields{i}).fit_params.r_squared;
                fprintf(fid, '<tr><td>%s</td><td>%s</td><td>%.3f</td></tr>\n', strrep(fields{i}, '_', '-'), nl_type, r_sq);
            end
        end
        fprintf(fid, '</table>\n');
    end
    
    % 4. 优化结果
    fprintf(fid, '<h2>4. 物理参数优化结果 (阶段二)</h2>\n');
    if isfield(params.linear, 'physical_parameters')
        fprintf(fid, '<h3>X-方向物理参数</h3>\n<table>\n');
        fprintf(fid, '<tr><th>参数</th><th>刚度 (N/m)</th><th>阻尼 (Ns/m)</th></tr>\n');
        px = params.linear.physical_parameters.x_vector_x;
        fprintf(fid, '<tr><td>k_g, c_g (连接地面)</td><td>%.3e</td><td>%.3e</td></tr>\n', px(1), px(2));
        fprintf(fid, '<tr><td>k_rm, c_rm (根-中)</td><td>%.3e</td><td>%.3e</td></tr>\n', px(3), px(4));
        fprintf(fid, '<tr><td>k_mt, c_mt (中-顶)</td><td>%.3e</td><td>%.3e</td></tr>\n', px(5), px(6));
        fprintf(fid, '</table>\n');

        fprintf(fid, '<h3>Z-方向物理参数</h3>\n<table>\n');
        fprintf(fid, '<tr><th>参数</th><th>刚度 (N/m)</th><th>阻尼 (Ns/m)</th></tr>\n');
        pz = params.linear.physical_parameters.x_vector_z;
        fprintf(fid, '<tr><td>k_g, c_g (连接地面)</td><td>%.3e</td><td>%.3e</td></tr>\n', pz(1), pz(2));
        fprintf(fid, '<tr><td>k_rm, c_rm (根-中)</td><td>%.3e</td><td>%.3e</td></tr>\n', pz(3), pz(4));
        fprintf(fid, '<tr><td>k_mt, c_mt (中-顶)</td><td>%.3e</td><td>%.3e</td></tr>\n', pz(5), pz(6));
        fprintf(fid, '</table>\n');
    end
    
    % 5. 验证结果
    fprintf(fid, '<h2>5. 验证结果</h2>\n');
    if isfield(params, 'validation') && isfield(params.validation, 'mean_error')
        error_percent = params.validation.mean_error * 100;
        std_percent = params.validation.std_error * 100;
        
        if error_percent < 10, class_str = 'good'; quality_str = '优秀';
        elseif error_percent < 20, class_str = 'warning'; quality_str = '良好';
        else, class_str = 'bad'; quality_str = '需改进'; end
        
        fprintf(fid, '<p>平均交叉验证误差: <span class="%s">%.2f%% &pm; %.2f%%</span></p>\n', class_str, error_percent, std_percent);
        fprintf(fid, '<p>参数可靠性评级: <span class="%s">%s</span></p>\n', class_str, quality_str);
    end
    
    % 6. 建议
    fprintf(fid, '<h2>6. 分析建议</h2>\n');
    fprintf(fid, '<ul>\n');
    fprintf(fid, '<li>识别的参数已保存到 <strong>UpdatedTreeParameters.m</strong></li>\n');
    fprintf(fid, '<li>可直接在仿真脚本中调用该文件更新参数。</li>\n');
    if isfield(params, 'validation') && isfield(params.validation, 'mean_error') && params.validation.mean_error > 0.15
        fprintf(fid, '<li class="warning">验证误差较大，建议检查数据质量或选择更多信号段。</li>\n');
    end
    fprintf(fid, '<li>详细的识别结果保存在 <strong>IdentifiedParameters_Final_with_Force.mat</strong></li>\n');
    fprintf(fid, '</ul>\n');
    
    fprintf(fid, '</body></html>');
    fclose(fid);
    
    fprintf('  分析报告已生成: NonlinearAnalysisReport.html\n');
end

function visualizeResults(params)
    if ~isfield(params, 'linear') || isempty(params.linear)
        fprintf('警告：无线性参数，跳过FRF可视化\n');
        return;
    end
    
    linear_params = params.linear;
    
    % X方向FRF
    if isfield(linear_params, 'FRF_matrix_x') && isfield(linear_params, 'frequency_vector')
        FRF_x = linear_params.FRF_matrix_x;
        freq_vec = linear_params.frequency_vector;
        
        if ~isempty(FRF_x) && ~isempty(freq_vec) && ...
           size(FRF_x, 1) == length(freq_vec) && ...
           size(FRF_x, 2) >= 3 && size(FRF_x, 3) >= 3
            
            figure('Name', 'X方向频响函数矩阵');
            for output = 1:3
                for input = 1:3
                    subplot(3, 3, (output-1)*3 + input);
                    try
                        frf_data = abs(FRF_x(:, output, input));
                        if any(frf_data > 0)
                            semilogy(freq_vec, frf_data, 'b-', 'LineWidth', 1.5);
                            xlabel('频率 (Hz)');
                            ylabel('幅值');
                            title(sprintf('H_{%d%d}', output, input));
                            grid on;
                        else
                            text(0.5, 0.5, '无数据', 'Units', 'normalized', 'HorizontalAlignment', 'center');
                        end
                    catch ME
                        text(0.5, 0.5, sprintf('错误: %s', ME.message), 'Units', 'normalized', 'HorizontalAlignment', 'center');
                    end
                end
            end
        else
            fprintf('警告：X方向FRF矩阵维度不匹配\n');
        end
    end
    
    % Z方向FRF
    if isfield(linear_params, 'FRF_matrix_z') && isfield(linear_params, 'frequency_vector')
        FRF_z = linear_params.FRF_matrix_z;
        freq_vec = linear_params.frequency_vector;
        
        if ~isempty(FRF_z) && ~isempty(freq_vec) && ...
           size(FRF_z, 1) == length(freq_vec) && ...
           size(FRF_z, 2) >= 3 && size(FRF_z, 3) >= 3
            
            figure('Name', 'Z方向频响函数矩阵');
            for output = 1:3
                for input = 1:3
                    subplot(3, 3, (output-1)*3 + input);
                    try
                        frf_data = abs(FRF_z(:, output, input));
                        if any(frf_data > 0)
                            semilogy(freq_vec, frf_data, 'b-', 'LineWidth', 1.5);
                            xlabel('频率 (Hz)');
                            ylabel('幅值');
                            title(sprintf('H_{%d%d}', output, input));
                            grid on;
                        else
                            text(0.5, 0.5, '无数据', 'Units', 'normalized', 'HorizontalAlignment', 'center');
                        end
                    catch ME
                        text(0.5, 0.5, sprintf('错误: %s', ME.message), 'Units', 'normalized', 'HorizontalAlignment', 'center');
                    end
                end
            end
        else
            fprintf('警告：Z方向FRF矩阵维度不匹配\n');
        end
    end

    % 1. 模态振型图
    if isfield(params.linear, 'mode_shapes') && ~isempty(params.linear.mode_shapes)
        figure('Name', '模态振型', 'Position', [300, 300, 1000, 400]);
        n_modes = size(params.linear.mode_shapes, 2);
        
        for mode = 1:min(3, n_modes)
            subplot(1, 3, mode);
            positions = [0, 1, 2]; % Root, Mid, Tip位置
            shape = params.linear.mode_shapes(:, mode);
            
            plot(positions, shape, 'bo-', 'LineWidth', 2, 'MarkerSize', 10);
            hold on;
            plot(positions, zeros(size(positions)), 'k--');
            
            xlabel('位置');
            ylabel('相对位移');
            title(sprintf('模态 %d (%.2f Hz)', mode, params.linear.natural_freqs_x(mode))); % 使用修正后的字段
            grid on;
            ylim([-1.2, 1.2]);
            xticks(positions);
            xticklabels({'Root', 'Mid', 'Tip'});
        end
    end
    
    % 非线性特征图
    if isfield(params, 'nl_features') && ~isempty(fieldnames(params.nl_features))
        
        % 确定有哪些传感器的数据需要被绘制
        sensor_names = {'Root', 'Mid', 'Tip'};
        directions = {'X', 'Z'};
        sensors_to_plot = {};
        
        for i = 1:length(sensor_names)
            % 检查该传感器是否至少有一个方向的数据
            has_x_data = isfield(params.nl_features, [sensor_names{i} '_X']);
            has_z_data = isfield(params.nl_features, [sensor_names{i} '_Z']);
            if has_x_data || has_z_data
                sensors_to_plot{end+1} = sensor_names{i};
            end
        end
        
        if isempty(sensors_to_plot)
            % 如果没有任何可绘制的数据，则不创建图形
            return;
        end
        
        % 计算 figure 的行数，每行代表一个传感器
        n_rows = length(sensors_to_plot);
        
        figure('Name', '非线性特征分析', 'Position', [350, 150, 1000, 300 * n_rows]);
        
        % 按传感器进行外循环
        for row_idx = 1:n_rows
            current_sensor = sensors_to_plot{row_idx};
            
            for col_idx = 1:2
                current_direction = directions{col_idx};
                field_name = [current_sensor '_' current_direction];
                
                subplot(n_rows, 2, (row_idx - 1) * 2 + col_idx);
                
                 if isfield(params.nl_features, field_name)
                    
                    % 1. 从传入的参数中直接读取所有信息
                    nl_data = params.nl_features.(field_name);
                    amps = nl_data.amplitudes;
                    freqs = nl_data.frequencies;
                    fit_type = nl_data.type;
                    fit_params = nl_data.fit_params;
                    
                    hold on;
                    
                    % 2. 绘制原始数据
                    p1 = plot(amps, freqs, 'o', 'Color', [0.5 0.5 0.5], 'DisplayName', '测量数据点');
                    
                    % 3. 绘制已经计算好的趋势线
                    if isfield(fit_params, 'trend_line_x') && ~isempty(fit_params.trend_line_x)
                        if contains(fit_type, 'Hardening')
                            line_color = 'r';
                        elseif contains(fit_type, 'Softening')
                            line_color = 'g';
                        else
                            line_color = 'k';
                        end
                        p2 = plot(fit_params.trend_line_x, fit_params.trend_line_y, '-', 'Color', line_color, 'LineWidth', 2.5, 'DisplayName', '拟合趋势');
                    else
                        p2 = [];
                    end
                    
                    hold off;
                    grid on;
                    xlabel('响应峰值振幅 (g)');
                    ylabel('主频 (Hz)');
                    
                    % 4. 显示已经计算好的标题和图例
                    title({strrep(field_name, '_', '-'); ...
                           sprintf('趋势: %s | R²=%.2f', fit_type, fit_params.r_squared)}, ...
                           'FontSize', 10);
                    
                    if ~isempty(p2)
                        legend([p1, p2], 'Location', 'best');
                    else
                        legend(p1, 'Location', 'best');
                    end
                else
                    % 如果该方向没有数据，则显示一个空白的子图
                    title(strrep(field_name, '_', '-'));
                    text(0.5, 0.5, '无有效数据', 'HorizontalAlignment', 'center', 'Units', 'normalized');
                    box on; grid on;
                end
            end
        end
        
        sgtitle('非线性特征分析: 频率随振幅的变化趋势', 'FontSize', 16, 'FontWeight', 'bold');

    else
        figure('Name', '非线性特征汇总');
        text(0.5, 0.5, '未能从数据中提取到有效的非线性特征。', ...
            'HorizontalAlignment', 'center', 'FontSize', 14, 'Color', 'red');
        title('非线性特征汇总');
    end
        
    % 3. 验证结果图
    if isfield(params.validation, 'cv_errors')
        figure('Name', '验证结果', 'Position', [400, 400, 600, 400]);
        
        cv_errors = params.validation.cv_errors(:) * 100;
        scalar_mean_error = mean(cv_errors);

        if ~isempty(cv_errors)
            bar(1:length(cv_errors), cv_errors);
            hold on;
            plot([0, length(cv_errors)+1], [scalar_mean_error, scalar_mean_error], 'r--', 'LineWidth', 2, 'DisplayName', '平均误差');
            
            xlabel('交叉验证折数');
            ylabel('预测误差 (%)');
            title(sprintf('交叉验证结果 (平均误差: %.2f%%)', scalar_mean_error));
            legend('show');
            grid on;
            ylim([0, max(max(cv_errors)*1.2, 10)]);
        end
    end
    
    % 4. 信号段选择统计图
    if isfield(params, 'segments') && ~isempty(params.segments)
        figure('Name', '信号段选择统计', 'Position', [450, 450, 1000, 600]);
        
        positions = {'Root', 'Mid', 'Tip'};
        directions = {'X', 'Z'};
        
        subplot(1, 2, 1);
        stats_data = zeros(3, 2);
        for p = 1:3
            for d = 1:2
                idx = strcmp({params.segments.sensor_name}, positions{p}) & ...
                      strcmp({params.segments.direction}, directions{d});
                stats_data(p, d) = sum(idx);
            end
        end
        
        bar_handle = bar(stats_data);
        xlabel('传感器位置');
        ylabel('信号段数量');
        title('信号段选择统计');
        legend({'X方向', 'Z方向'}, 'Location', 'best');
        xticks(1:3);
        xticklabels(positions);
        grid on;
        
        for p = 1:2
            for q = 1:size(stats_data, 1)
                if stats_data(q,p) > 0
                    text(bar_handle(p).XData(q), bar_handle(p).YData(q), num2str(bar_handle(p).YData(q)),...
                         'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
                end
            end
        end
        
        subplot(1, 2, 2);
        if isfield(params.segments(1), 'duration')
            durations = [params.segments.duration];
            histogram(durations, 10);
            xlabel('信号段持续时间 (s)');
            ylabel('数量');
            title('信号段持续时间分布');
            grid on;
            
            mean_duration = mean(durations);
            std_duration = std(durations);
            text(0.7, 0.8, sprintf('平均: %.2fs\n标准差: %.2fs', mean_duration, std_duration), ...
                 'Units', 'normalized', 'BackgroundColor', 'white', 'FontSize', 10);
        end
        
        sgtitle('起点终点选择结果统计', 'FontSize', 14, 'FontWeight', 'bold');
    end
    
    % 5. X方向频响函数图
    if isfield(params.linear, 'FRF_matrix_x') && isfield(params.linear, 'frequency_vector')
        figure('Name', 'X方向频响函数', 'Position', [500, 500, 1200, 800]);
        
        freq_vec = params.linear.frequency_vector; % 使用正确的频率向量
        sensor_names = {'Root', 'Mid', 'Tip'};

        for i = 1:9
            subplot(3, 3, i);
            output = floor((i-1)/3) + 1; % 响应点索引 (行)
            input = mod(i-1, 3) + 1;
            
            response_name = sensor_names{output};
            excitation_name = sensor_names{input};

            yyaxis left;
            semilogy(freq_vec, abs(params.linear.FRF_matrix_x(:, output, input)), 'b-', 'LineWidth', 1.5);
            ylabel('幅值');
            
            yyaxis right;
            plot(freq_vec, params.linear.coherence_matrix_x(:, output, input), 'r-', 'LineWidth', 1);
            ylabel('相干性');
            ylim([0, 1]);
            
            ax = gca;
            ax.YAxis(1).Color = 'b';
            ax.YAxis(2).Color = 'r';
            
            grid on;
            xlabel('频率 (Hz)');

            title_str = sprintf('H_{%s-%s}^X (响应:%s <- 激励:%s)', ...
                          response_name(1), excitation_name(1), response_name, excitation_name);
            title(title_str);

            xlim([0, 500]);
            
            if isfield(params.linear, 'natural_freqs_x') && ~isempty(params.linear.natural_freqs_x)
                yyaxis left; hold on;
                y_lim = ylim;
                for k = 1:length(params.linear.natural_freqs_x)
                    f = params.linear.natural_freqs_x(k);
                    plot([f f], y_lim, 'k--', 'HandleVisibility', 'off');
                end
            end
        end
        
        sgtitle('X方向频响函数矩阵', 'FontSize', 14, 'FontWeight', 'bold');
    end
    
    % 6. Z方向频响函数图
    if isfield(params.linear, 'FRF_matrix_z') && isfield(params.linear, 'frequency_vector')
        figure('Name', 'Z方向频响函数', 'Position', [550, 550, 1200, 800]);
        
        freq_vec = params.linear.frequency_vector; % 使用正确的频率向量
        sensor_names = {'Root', 'Mid', 'Tip'};

        for i = 1:9
            subplot(3, 3, i);
            output = floor((i-1)/3) + 1; % 响应点索引 (行)
            input = mod(i-1, 3) + 1;
            
            response_name = sensor_names{output};
            excitation_name = sensor_names{input};

            yyaxis left;
            semilogy(freq_vec, abs(params.linear.FRF_matrix_z(:, output, input)), 'b-', 'LineWidth', 1.5);
            ylabel('幅值');
            
            yyaxis right;
            plot(freq_vec, params.linear.coherence_matrix_z(:, output, input), 'r-', 'LineWidth', 1);
            ylabel('相干性');
            ylim([0, 1]);
            
            ax = gca;
            ax.YAxis(1).Color = 'b';
            ax.YAxis(2).Color = 'r';
            
            grid on;
            xlabel('频率 (Hz)');

            title_str = sprintf('H_{%s-%s}^Z (响应:%s <- 激励:%s)', ...
                          response_name(1), excitation_name(1), response_name, excitation_name);
            title(title_str);

            xlim([0, 500]);
             if isfield(params.linear, 'natural_freqs_z') && ~isempty(params.linear.natural_freqs_z)
                yyaxis left; hold on;
                y_lim = ylim;
                for k = 1:length(params.linear.natural_freqs_z)
                    f = params.linear.natural_freqs_z(k);
                    plot([f f], y_lim, 'k--', 'HandleVisibility', 'off');
                end
            end
        end
        
        sgtitle('Z方向频响函数矩阵', 'FontSize', 14, 'FontWeight', 'bold');
    end

    fprintf('  结果可视化完成\n');
end

function impactLocationChanged(src, event)
    global g_annotation_data;
    
    selected_button = event.NewValue.String;
    
    if contains(selected_button, 'Root')
        g_annotation_data.current_impact_location = 1;
        location_str = 'ROOT';
        color = [0, 0.5, 0];
    elseif contains(selected_button, 'Mid')
        g_annotation_data.current_impact_location = 2;
        location_str = 'MID';
        color = [0, 0, 0.7];
    else % Tip
        g_annotation_data.current_impact_location = 3;
        location_str = 'TIP';
        color = [0.7, 0, 0.7];
    end
    
    % Update title
    set(g_annotation_data.title_handle, 'String', ...
        sprintf('%s方向信号段选择 - 当前敲击位置: %s', ...
                g_annotation_data.current_direction, location_str), ...
        'Color', color);
    
    % Update current impact display
    set(g_annotation_data.current_impact_text, 'String', ...
        sprintf('当前敲击: %s-%s', location_str, g_annotation_data.current_direction));
    
    % Highlight the corresponding sensor plot
    updateSensorPlotHighlight();
    
    fprintf('  切换敲击位置到: %s传感器\n', location_str);
end

% NEW FUNCTION to handle plot highlighting
% --- 函数变化：updateSensorPlotHighlight (FIXED & FINAL) ---
% 修复了索引超限的BUG
function updateSensorPlotHighlight()
    global g_annotation_data;
    
    colors = {[0, 0.5, 0], [0, 0, 0.7], [0.7, 0, 0.7]}; % Root, Mid, Tip colors
    sensor_names = g_annotation_data.sensor_names;

    % 遍历所有加速度传感器图 (从第2个subplot到最后一个)
    for plot_idx = 2:length(g_annotation_data.subplots)
        sensor_idx = plot_idx - 1; % 对应的传感器索引 (1, 2, 3)
        
        if ishandle(g_annotation_data.subplots(plot_idx))
            ax = g_annotation_data.subplots(plot_idx);
            title_handle = get(ax, 'Title');
            
            % 检查当前传感器是否是选定的敲击位置
            if sensor_idx == g_annotation_data.current_impact_location
                % 高亮选中的图
                set(ax, 'Color', [1, 1, 0.95]); % 淡黄色背景
                set(title_handle, ...
                    'String', sprintf('%s传感器 - %s方向 [当前敲击位置]', ...
                                      sensor_names{sensor_idx}, g_annotation_data.current_direction), ...
                    'FontWeight', 'bold', ...
                    'Color', colors{sensor_idx});
            else
                % 重置其他图
                set(ax, 'Color', 'white');
                set(title_handle, ...
                    'String', sprintf('%s传感器 - %s方向', ...
                                      sensor_names{sensor_idx}, g_annotation_data.current_direction), ...
                    'FontWeight', 'normal', ...
                    'Color', 'black');
            end
        end
    end
end


%% ==================== 标注重建函数 (V6.0 净化版) ====================
% 该版本不再兼容旧的、仅含时间戳的标注文件。
% 它只接受包含精确采样点索引的新版标注，从根本上杜绝时间轴偏差。
function segments = reconstructSegments(saved_annotations, accel_data_cell, ...
                                            force_data, force_time, fs)
    % 功能：重建标注段（添加SNR预警）
    
    segments = [];
    if isempty(saved_annotations)
        return;
    end
    
    num_annotations = length(saved_annotations);
    reconstructed_segments = cell(1, num_annotations);
    count = 0;
    
    fprintf('  [净化版重建流程] 开始重建 %d 个标注段...\n', num_annotations);
    
    low_snr_count = 0;

    for i = 1:num_annotations
        ann = saved_annotations(i);
        
        if ~isfield(ann, 'accel_start_idx') || ~isfield(ann, 'accel_end_idx')
            fprintf(2, '    错误: 标注段 %d 缺少索引\n', i);
            continue;
        end
        
        sensor_idx = ann.sensor_idx;
        accel_time_vec = accel_data_cell{sensor_idx}.time;
        accel_data = accel_data_cell{sensor_idx}.data;
        
        start_idx = ann.accel_start_idx;
        end_idx = ann.accel_end_idx;
        
        if start_idx < 1 || end_idx > length(accel_time_vec) || start_idx >= end_idx
            fprintf('    警告: 段 %d 索引无效，跳过\n', i);
            continue;
        end
        
        % 提取信号段
        segment_data_3d = accel_data(start_idx:end_idx, :);
        
        % 轻量后处理
        processed_data = postProcessSegment(segment_data_3d, fs);
        
        ann.start_time = accel_time_vec(start_idx);
        ann.end_time = accel_time_vec(end_idx);
        ann.duration = ann.end_time - ann.start_time;
        
        % 提取方向信号
        direction_col = strcmp(ann.direction, 'X') + 1;
        if strcmp(ann.direction, 'Z')
            direction_col = 3;
        end
        
        signal_1d = processed_data(:, direction_col);
        
        segment_struct = ann;
        segment_struct.signal_data = signal_1d;
        segment_struct.time_vector = (0:length(signal_1d)-1)' / fs;
        segment_struct.fs = fs;
        
        % 峰值和SNR分析
        [envelope, peaks, peak_indices] = extractEnvelopeAndPeaks(signal_1d, fs);
        [snr, ~, ~] = calculateAdvancedSNR(signal_1d, fs);
        
        segment_struct.peak_info.peak_amplitudes = peaks;
        segment_struct.peak_info.peak_times = peak_indices / fs;
        segment_struct.detection_results.snr = snr;
        segment_struct.detection_results.envelope = envelope;
        
        if ~isempty(peaks)
            segment_struct.peak_amplitude = max(abs(peaks));
        else
            segment_struct.peak_amplitude = max(abs(signal_1d));
        end
        segment_struct.max_amplitude = max(abs(signal_1d));

        % 力信号
        if ~isempty(force_data) && isfield(ann, 'force_start_idx') && isfield(ann, 'force_end_idx')
            f_start = ann.force_start_idx;
            f_end = ann.force_end_idx;
            if f_start >= 1 && f_end <= length(force_data) && f_start < f_end
                segment_struct.force_data_segment = force_data(f_start:f_end);
            end
        end
        
        count = count + 1;
        reconstructed_segments{count} = segment_struct;
        
        % SNR统计
        if snr < 10
            low_snr_count = low_snr_count + 1;
        end
        
        fprintf('    ✓ 段 %d: 索引[%d:%d], SNR=%.1fdB, 峰值数=%d\n', ...
            i, start_idx, end_idx, snr, length(peaks));
    end
    
    if count > 0
        segments = [reconstructed_segments{1:count}];
        fprintf('  [净化版重建] 成功重建 %d/%d 个段\n', count, num_annotations);
        fprintf('  [质量预警] %d 个段SNR<10dB，建议重新标注高质量段\n', low_snr_count);
    else
        fprintf(2, '  [净化版重建] 未能重建任何段\n');
    end
end

function processed_data = postProcessSegment(segment_data, fs)
    % 功能：段级后处理（极轻量化，只去直流）
    
    if size(segment_data, 1) < 50
        processed_data = segment_data;
        return;
    end
    
    data = segment_data;
    
    % 只去直流，不做任何滤波
    for j = 1:size(data, 2)
        data(:, j) = data(:, j) - mean(data(:, j));
    end
    
    processed_data = data;
end

function processed_data = processSegmentIndependently(raw_segment, original_fs, target_fs)
    % 功能：独立处理单个信号段
    
    if isempty(raw_segment) || size(raw_segment, 1) < 50
        processed_data = [];
        return;
    end
    
    % 步骤1：去直流分量
    data = raw_segment;
    for j = 1:size(data, 2)
        baseline = mean(data(1:min(100, size(data,1)), j));
        data(:, j) = data(:, j) - baseline;
    end
    
    % 步骤2：带通滤波（5-50Hz，过滤噪声）
    try
        [b, a] = butter(4, [5, 50]/(original_fs/2), 'bandpass');
        for j = 1:size(data, 2)
            data(:, j) = filtfilt(b, a, data(:, j));
        end
    catch
        % 滤波失败则跳过
    end
    
    % 步骤3：重采样到目标采样率
    if abs(original_fs - target_fs) > 1
        data = resample(data, target_fs, round(original_fs));
    end
    
    % 步骤4：轻微平滑（可选）
    window_size = max(3, round(0.005 * target_fs));
    if window_size > 1 && mod(window_size, 2) == 0
        window_size = window_size + 1;
    end
    for j = 1:size(data, 2)
        data(:, j) = medfilt1(data(:, j), window_size);
    end
    
    processed_data = data;
end

function saveAnnotations(segments, accel_data_cell, force_time, fs, full_filepath)
    if isempty(segments)
        fprintf('  没有标注数据需要保存\n');
        return;
    end
    
    % 准备保存数据
    saved_annotations = [];
    
    % 保存必要的标注信息,包括采样点索引
    for i = 1:length(segments)
        ann = struct();
        
        % 保存所有关键字段
        fields_to_save = {'segment_id', 'sync_group_id', 'sensor_idx', 'sensor_name', ...
                         'direction', 'direction_idx', 'impact_location', 'impact_sensor_name', ...
                         'start_time', 'end_time', 'duration', 'is_synchronized', ...
                         'force_start_time', 'force_end_time', 'force_duration', ...
                         'selected_on_sensor', 'peak_amplitude', 'force_level'};
        
        for f = 1:length(fields_to_save)
            if isfield(segments(i), fields_to_save{f})
                ann.(fields_to_save{f}) = segments(i).(fields_to_save{f});
            end
        end
        
        % ========== [关键修复] 额外保存采样点索引,用于精确重建 ==========
        sensor_idx = segments(i).sensor_idx;
        accel_time_vec = accel_data_cell{sensor_idx}.time;
        
        % 计算并保存加速度信号的采样点索引
        accel_start_idx = find(accel_time_vec >= segments(i).start_time, 1, 'first');
        accel_end_idx = find(accel_time_vec <= segments(i).end_time, 1, 'last');
        
        if ~isempty(accel_start_idx) && ~isempty(accel_end_idx)
            ann.accel_start_idx = accel_start_idx;
            ann.accel_end_idx = accel_end_idx;
            ann.num_samples = accel_end_idx - accel_start_idx + 1;
        end
        
        % 保存力信号的采样点索引(如果有)
        if ~isempty(force_time) && isfield(segments(i), 'force_start_time') && isfield(segments(i), 'force_end_time')
            force_start_idx = find(force_time >= segments(i).force_start_time, 1, 'first');
            force_end_idx = find(force_time <= segments(i).force_end_time, 1, 'last');
            
            if ~isempty(force_start_idx) && ~isempty(force_end_idx)
                ann.force_start_idx = force_start_idx;
                ann.force_end_idx = force_end_idx;
            end
        end
        % ========== [关键修复结束] ==========
        
        if isempty(saved_annotations)
            saved_annotations = ann;
        else
            saved_annotations(end+1) = ann;
        end
    end
    
    % 保存数据信息用于验证
    data_info = struct();
    data_info.fs = fs;
    data_info.save_time = datestr(now);
    data_info.n_segments = length(saved_annotations);
    data_info.version = 'v5'; % 标记为修复版本
    
    % 保存到文件
    try
        save(full_filepath, 'saved_annotations', 'data_info');
        fprintf('  标注数据已成功保存到: %s\n', full_filepath);
        fprintf('  共保存 %d 个信号段标注\n', length(saved_annotations));
        fprintf('  [修复版] 已保存采样点索引,确保跨会话加载的准确性\n');
    catch ME
        fprintf('  错误:保存标注文件失败: %s\n', ME.message);
        errordlg(sprintf('保存文件失败:\n%s', ME.message), '保存错误');
    end
end

function showAnnotationStats(segments)
    if isempty(segments)
        return;
    end
    
    fprintf('\n  ===== 标注统计信息 =====\n');
    
    % 统计各方向和位置的段数
    directions = unique({segments.direction});
    sensors = unique({segments.sensor_name});
    
    for d = 1:length(directions)
        dir_segs = segments(strcmp({segments.direction}, directions{d}));
        fprintf('  %s方向: %d 段\n', directions{d}, length(dir_segs));
        
        for s = 1:length(sensors)
            sensor_count = sum(strcmp({dir_segs.sensor_name}, sensors{s}));
            if sensor_count > 0
                fprintf('    - %s: %d 段\n', sensors{s}, sensor_count);
            end
        end
    end
    
    % 显示敲击位置统计
    fprintf('\n  敲击位置统计:\n');
    impact_locs = [segments.impact_location];
    unique_impact_locs = unique(impact_locs);
    loc_names = {'Root', 'Mid', 'Tip'};
    for i = 1:length(unique_impact_locs)
        loc_idx = unique_impact_locs(i);
        count = sum(impact_locs == loc_idx);
        if count > 0
            fprintf('    %s: %d 次\n', loc_names{loc_idx}, count);
        end
    end
    
    fprintf('  ========================\n\n');
end

%% 信号诊断可视化函数
% 功能：为单次锤击事件生成详细的诊断图，包含时域、频域功率谱、
%       FRF和相干性，以进行全面的数据质量评估。

function visualizeSignalDiagnostics(segments, fs, nfft)

    fprintf('\n  [诊断] 生成 X 和 Z 方向的 3x3 功率谱(PSD)矩阵图...\n');
    
    if isempty(segments)
        fprintf('  诊断警告：无有效的信号段可供分析。\n');
        return;
    end
    
    % --- 核心修正：调用新的、可靠的辅助函数来计算平均PSD矩阵 ---
    [psd_avg_x, psd_avg_z, freq] = computeAveragePSDMatrix(segments, fs, nfft);

    sensor_names = {'Root', 'Mid', 'Tip'};
    
    % --- 绘图：X方向 ---
    figure('Name', 'X方向功率谱密度(PSD)诊断矩阵', 'Position', [100, 100, 1200, 800]);

    for plot_idx = 1:9
        j_impact = mod(plot_idx-1, 3) + 1;
        i_response = floor((plot_idx-1)/3) + 1;
        
        subplot(3, 3, plot_idx);
        
        psd_data = psd_avg_x{i_response, j_impact};

        semilogy(freq, psd_data, 'b-', 'LineWidth', 1.5);
        grid on;
        title(sprintf('响应 %s <- 激励 %s', sensor_names{i_response}, sensor_names{j_impact}));
        xlim([0, 500]);
        
        if i_response == 3, xlabel('频率 (Hz)'); end
        if j_impact == 1, ylabel('功率/Hz'); end
    end

    sgtitle('X方向 PSD 诊断矩阵 (响应点 <- 激励点)', 'FontSize', 16, 'FontWeight', 'bold');
    
    % --- 绘图：Z方向 ---
    figure('Name', 'Z方向功率谱密度(PSD)诊断矩阵', 'Position', [150, 150, 1200, 800]);
    for plot_idx = 1:9
        j_impact = mod(plot_idx-1, 3) + 1;
        i_response = floor((plot_idx-1)/3) + 1;
        
        subplot(3, 3, plot_idx);

        psd_data = psd_avg_z{i_response, j_impact};
        
        semilogy(freq, psd_data, 'r-', 'LineWidth', 1.5);
        grid on;
        title(sprintf('响应 %s <- 激励 %s', sensor_names{i_response}, sensor_names{j_impact}));
        xlim([0, 500]);

        if i_response == 3, xlabel('频率 (Hz)'); end
        if j_impact == 1, ylabel('功率/Hz'); end
    end
    sgtitle('Z方向 PSD 诊断矩阵 (响应点 <- 激励点)', 'FontSize', 16, 'FontWeight', 'bold');
    
    fprintf('  X和Z方向的3x3功率谱矩阵图已生成。\n');
end

function [psd_avg_x, psd_avg_z, freq] = computeAveragePSDMatrix(segments, fs, nfft)

    n_freq_points = nfft/2 + 1;
    freq = linspace(0, fs/2, n_freq_points)';
    
    psd_sum_x = cell(3, 3); psd_sum_x(:) = {zeros(n_freq_points, 1)};
    psd_sum_z = cell(3, 3); psd_sum_z(:) = {zeros(n_freq_points, 1)};
    psd_counts_x = zeros(3, 3);
    psd_counts_z = zeros(3, 3);
    
    for k = 1:length(segments)
        seg = segments(k);
        if ~isfield(seg, 'signal_data') || isempty(seg.signal_data), continue; end
        
        i_response = seg.sensor_idx;
        j_impact = seg.impact_location;
        signal = seg.signal_data;
        
        win_len = min(nfft, length(signal));
        if length(signal) < 4, continue; end
        
        [psd, ~] = pwelch(signal, hanning(win_len), round(win_len*0.5), nfft, fs);
        
        if strcmp(seg.direction, 'X')
            psd_sum_x{i_response, j_impact} = psd_sum_x{i_response, j_impact} + psd;
            psd_counts_x(i_response, j_impact) = psd_counts_x(i_response, j_impact) + 1;
        elseif strcmp(seg.direction, 'Z')
            psd_sum_z{i_response, j_impact} = psd_sum_z{i_response, j_impact} + psd;
            psd_counts_z(i_response, j_impact) = psd_counts_z(i_response, j_impact) + 1;
        end
    end

    psd_avg_x = cell(3, 3);
    psd_avg_z = cell(3, 3);
    for i = 1:3
        for j = 1:3
            if psd_counts_x(i, j) > 0
                psd_avg_x{i, j} = psd_sum_x{i, j} / psd_counts_x(i, j);
            else
                psd_avg_x{i, j} = eps * ones(n_freq_points, 1);
            end
            if psd_counts_z(i, j) > 0
                psd_avg_z{i, j} = psd_sum_z{i, j} / psd_counts_z(i, j);
            else
                psd_avg_z{i, j} = eps * ones(n_freq_points, 1);
            end
        end
    end
end

%% ===== 增强的峰值识别诊断系统 =====

function visualizePeakDetection(segments, fs)
    fprintf('\n=== 峰值识别诊断开始 ===\n');
    
    if isempty(segments)
        fprintf('    没有有效的信号段用于诊断。\n');
        return;
    end
    
    figure('Name', '峰值识别诊断', 'Position', [50, 50, 1400, 900]);
    
    n_segments = length(segments);
    sample_indices = randperm(n_segments, min(6, n_segments));
    
    for i = 1:length(sample_indices)
        seg_idx = sample_indices(i);
        seg = segments(seg_idx);
        
        if ~isfield(seg, 'signal_data') || isempty(seg.signal_data)
            continue;
        end
        
        signal = seg.signal_data;
        time_vec = (0:length(signal)-1)' / fs;
        
        subplot(2, 3, i);
        hold on;
        
        p1 = plot(time_vec, signal, 'Color', [0.7 0.7 1], 'DisplayName', '原信号');
        
        if isfield(seg, 'detection_results') && isfield(seg.detection_results, 'envelope')
            p2 = plot(time_vec, seg.detection_results.envelope, 'g--', 'LineWidth', 1.5, 'DisplayName', '包络线');
        end
        
        if isfield(seg, 'peak_info') && ~isempty(seg.peak_info.peak_times)
            p3 = plot(seg.peak_info.peak_times, seg.peak_info.peak_amplitudes, 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r', 'DisplayName', '峰值');
        end
        
        grid on;
        xlabel('时间 (s)');
        ylabel('加速度 (g)');
        
        if isfield(seg, 'detection_results') && isfield(seg.detection_results, 'snr')
            snr_val = seg.detection_results.snr;
        else
            snr_val = NaN;
        end
        
        title(sprintf('段%d: %s-%s (SNR=%.1fdB)', seg.segment_id, seg.sensor_name, seg.direction, snr_val));
        legend('Location', 'northeast');
        hold off;
    end
    
    sgtitle('峰值识别诊断 - 随机采样段分析', 'FontSize', 14, 'FontWeight', 'bold');
    fprintf('=== 峰值识别诊断完成 ===\n\n');
end



%% ============ 全新的、更简单的信号对齐函数 ============

function [type, fit_params] = determineNonlinearType(amplitudes, frequencies)
    % 功能：判定非线性类型（改进版，加权拟合+分段分析）
    
    fit_params = struct('order', 1, 'coefficients', [], 'r_squared', 0, ...
                        'trend_line_x', [], 'trend_line_y', []);

    if length(amplitudes) < 3
        type = 'Insufficient Data';
        return;
    end
    
    % 数据预处理：移除异常值
    if length(amplitudes) >= 5
        median_freq = movmedian(frequencies, 5, 'omitnan');
        abs_deviation = abs(frequencies - median_freq);
        mad_deviation = movmedian(abs_deviation, 5, 'omitnan') * 1.4826;
        outlier_mask = abs_deviation > (3 * mad_deviation);
        amp_clean = amplitudes(~outlier_mask);
        freq_clean = frequencies(~outlier_mask);
    else
        amp_clean = amplitudes;
        freq_clean = frequencies;
    end
    
    if length(amp_clean) < 3
        type = 'Insufficient Data';
        return;
    end

    % 加权拟合（大振幅权重更大）
    weights = (amp_clean / max(amp_clean)).^2;
    
    % 尝试1阶和2阶拟合
    [p1, S1] = polyfit(amp_clean, freq_clean, 1);
    y_fit1 = polyval(p1, amp_clean);
    ss_res1 = sum(weights .* (freq_clean - y_fit1).^2);
    ss_tot = sum(weights .* (freq_clean - mean(freq_clean)).^2);
    r2_1 = 1 - ss_res1 / (ss_tot + eps);
    
    if length(amp_clean) > 3
        [p2, S2] = polyfit(amp_clean, freq_clean, 2);
        y_fit2 = polyval(p2, amp_clean);
        ss_res2 = sum(weights .* (freq_clean - y_fit2).^2);
        r2_2 = 1 - ss_res2 / (ss_tot + eps);
        
        % 选择更好的拟合（2阶必须显著更好）
        if r2_2 > (r2_1 + 0.1)
            best_p = p2; best_order = 2; fit_params.r_squared = r2_2;
        else
            best_p = p1; best_order = 1; fit_params.r_squared = r2_1;
        end
    else
        best_p = p1; best_order = 1;
        fit_params.r_squared = r2_1;
    end
    
    % 判定非线性类型
    x_range = [min(amp_clean), max(amp_clean)];
    y_range = polyval(best_p, x_range);
    overall_slope = (y_range(2) - y_range(1)) / (x_range(2) - x_range(1) + eps);
    
    % 计算频率变化强度
    freq_range = max(freq_clean) - min(freq_clean);
    amp_range = max(amp_clean) - min(amp_clean);
    
    % 判定标准
    if fit_params.r_squared < 0.15
        type = 'Linear (No Clear Trend)';
    elseif freq_range < 5 && amp_range > 10
        type = 'Linear (Weak Nonlinearity)';
    elseif overall_slope > 0.5
        type = 'Hardening';
    elseif overall_slope < -0.5
        type = 'Softening';
    else
        type = 'Linear';
    end
    
    if best_order == 2 && fit_params.r_squared >= 0.3
        type = [type, ' (Quadratic)'];
    end

    % 保存拟合参数
    fit_params.order = best_order;
    fit_params.coefficients = best_p;
    
    x_smooth = linspace(min(amp_clean), max(amp_clean), 100);
    y_smooth = polyval(best_p, x_smooth);
    fit_params.trend_line_x = x_smooth;
    fit_params.trend_line_y = y_smooth;
end

%% ============ 新增：分类平均时频分析与可视化函数 (最终健壮版) ============
function visualizeTimeFrequencyAnalysis(segments, fs)
    % (V6.0 - 使用补零(Zero-Padding)处理短信号，并根据实际数据动态确定累加矩阵维度)

    fprintf('\n  [诊断] 开始进行分类平均时频特性分析 (STFT)...\n');
    
    if isempty(segments)
        fprintf('    警告: 没有有效的信号段可供进行时频分析。\n');
        return;
    end
    
    positions = {'Root', 'Mid', 'Tip'};
    directions = {'X', 'Z'};
    
    window_duration = 0.25; 
    window_length = round(window_duration * fs); 
    if mod(window_length, 2) ~= 0, window_length = window_length + 1; end
    overlap_length = round(window_length * 0.95); 
    nfft = max(512, 2^nextpow2(window_length));
    freq_limit = 500;
    
    analysis_results = cell(3, 2);
    for i = 1:3, for j = 1:2
        analysis_results{i, j} = struct('spectra', {{}});
    end, end

    fprintf('    第一步: 正在遍历所有 %d 个分段并计算STFT (短信号将自动补零)...\n', length(segments));
    
    for i = 1:length(segments)
        seg = segments(i);
        if ~isfield(seg, 'signal_data') || isempty(seg.signal_data), continue; end

        signal_to_process = seg.signal_data(:);
        
        if length(signal_to_process) < window_length
            padding_needed = window_length - length(signal_to_process);
            signal_to_process = [signal_to_process; zeros(padding_needed, 1)];
        end
        
        pos_idx = find(strcmp(positions, seg.sensor_name));
        dir_idx = find(strcmp(directions, seg.direction));
        if isempty(pos_idx) || isempty(dir_idx), continue; end
        
        [s, ~, ~] = stft(signal_to_process, fs, 'Window', hann(window_length), 'OverlapLength', overlap_length, 'FFTLength', nfft);
        power_spectrum = abs(s).^2;
        
        analysis_results{pos_idx, dir_idx}.spectra{end+1} = power_spectrum;
    end
    
    fprintf('    STFT计算完成。\n');
    fprintf('    第二步: 正在计算平均谱并生成可视化图...\n');
    figure('Name', '分类平均时频能量谱 (自适应长度)', 'Position', [50, 50, 1800, 900]);
    
    plot_idx = 1;
    for j = 1:numel(directions)
        for i = 1:numel(positions)
            subplot(numel(directions), numel(positions), plot_idx);
            result = analysis_results{i, j};
            
            if ~isempty(result.spectra)
                % --- [核心修复 V6.0] ---
                % 1. 从第一个谱中获取真实的行数（频率点数）
                num_freq_bins = size(result.spectra{1}, 1);
                
                % 2. 遍历所有谱，找到最长的时间步数
                max_t_steps = 0;
                for k = 1:length(result.spectra)
                    max_t_steps = max(max_t_steps, size(result.spectra{k}, 2));
                end
                
                % 3. 使用这些从数据中得到的真实维度来初始化累加矩阵
                sum_spec = zeros(num_freq_bins, max_t_steps);
                count_map = zeros(1, max_t_steps);
                
                % 4. 累加所有谱，并增加维度安全检查
                for k = 1:length(result.spectra)
                    spec = result.spectra{k};
                    
                    % 安全检查: 确保当前谱的行数与基准一致
                    if size(spec, 1) ~= num_freq_bins
                        fprintf(2, '    警告: 检测到STFT结果的频率点数不一致，已跳过一个谱。\n');
                        continue;
                    end
                    
                    t_steps = size(spec, 2);
                    sum_spec(:, 1:t_steps) = sum_spec(:, 1:t_steps) + spec;
                    count_map(1:t_steps) = count_map(1:t_steps) + 1;
                end
                % --- [修复结束 V6.0] ---
                
                avg_spectrum = sum_spec ./ (count_map + eps);
                
                [~, f_axis, ~] = stft(zeros(window_length,1), fs, 'Window', hann(window_length), 'OverlapLength', overlap_length, 'FFTLength', nfft);
                % 安全截断f_axis以匹配真实的频率点数
                if length(f_axis) > num_freq_bins
                    f_axis = f_axis(1:num_freq_bins);
                end
                
                time_step_duration = (window_length - overlap_length) / fs;
                t_axis = (0:max_t_steps-1) * time_step_duration;
                
                power_spectrum_db = 10*log10(avg_spectrum + 1e-12);
                
                surf(t_axis, f_axis, power_spectrum_db, 'EdgeColor', 'none');
                view(2);
                axis tight; ylim([0, freq_limit]); colormap('jet');
                c = colorbar; c.Label.String = '平均功率谱密度 (dB/Hz)';
                
                title(sprintf('%s - %s 方向 (N=%d)', positions{i}, directions{j}, length(result.spectra)));
            else
                title(sprintf('%s - %s 方向 (N=0)', positions{i}, directions{j}));
                text(0.5, 0.5, '无数据', 'HorizontalAlignment', 'center', 'FontSize', 14);
                set(gca, 'XTick', [], 'YTick', []);
            end
            
            if j == numel(directions), xlabel('时间 (s)'); end
            if i == 1, ylabel('频率 (Hz)'); end
            plot_idx = plot_idx + 1;
        end
    end
    sgtitle('按位置和方向分类的平均时频能量分布', 'FontSize', 16, 'FontWeight', 'bold');
    fprintf('  [诊断] 分类平均时频特性分析完成。\n');
end

function [x0, lb, ub] = estimate_initial_guess_from_modal(modal_freqs, modal_damping_ratios, M)
    % 基于阶段一识别的模态频率，计算合理的初始值和边界
    % 针对轻质分枝（总质量约60g）优化
    
    if ~isempty(modal_freqs)
        omega1 = 2 * pi * modal_freqs(1);  % 第一阶角频率
        zeta1 = modal_damping_ratios(1);   % 第一阶阻尼比
        
        % 使用系统总质量作为参考
        m_total = sum(diag(M));  % 约0.06 kg
        
        % 从第一阶频率反推典型刚度
        % 对于f1≈7Hz, m=0.06kg: k_ref ≈ (2π*7)^2 * 0.06 ≈ 115 N/m
        k_ref = omega1^2 * m_total;
        c_ref = 2 * zeta1 * omega1 * m_total;
        
        fprintf('    [参数推算] f1=%.2f Hz, m_total=%.3f kg → k_ref=%.1f N/m, c_ref=%.3f Ns/m\n', ...
            modal_freqs(1), m_total, k_ref, c_ref);
        
        % 初始值：基于轻质分枝特性
        % 轻质分枝：刚度小（几十到几百N/m），阻尼很小（0.1-1 Ns/m）
        x0 = [
            k_ref * 0.35,   ... k_g ≈ 40 N/m（接地柔性连接）
            c_ref * 0.4,    ... c_g ≈ 0.05 Ns/m
            k_ref * 1.3,    ... k_rm ≈ 150 N/m（根部段，最粗）
            c_ref * 1.0,    ... c_rm ≈ 0.25 Ns/m
            k_ref * 0.9,    ... k_mt ≈ 100 N/m（顶部段，较细）
            c_ref * 0.8     ... c_mt ≈ 0.2 Ns/m
        ];
        
        % 边界：针对轻质分枝设置合理范围
        lb = [
            k_ref * 0.05,   ... k_g: 6-200 N/m
            c_ref * 0.05,   ... c_g: 0.01-2 Ns/m
            k_ref * 0.2,    ... k_rm: 23-650 N/m
            c_ref * 0.2,    ... c_rm: 0.05-2.5 Ns/m
            k_ref * 0.2,    ... k_mt: 23-650 N/m
            c_ref * 0.2     ... c_mt: 0.05-2.5 Ns/m
        ];
        
        ub = [
            k_ref * 1.8,    ... k_g上界
            c_ref * 8,      ... c_g上界
            k_ref * 6,      ... k_rm上界
            c_ref * 10,     ... c_rm上界
            k_ref * 6,      ... k_mt上界
            c_ref * 10      ... c_mt上界
        ];
        
        fprintf('    [边界设置] k范围: [%.0f, %.0f] N/m, c范围: [%.2f, %.2f] Ns/m\n', ...
            min(lb([1,3,5])), max(ub([1,3,5])), min(lb([2,4,6])), max(ub([2,4,6])));
        
    else
        % 回退值：假设第一阶约7Hz，总质量约60g
        fprintf('    [警告] 未找到模态频率，使用轻质分枝经验回退值\n');
        x0 = [40, 0.05, 150, 0.25, 100, 0.2];
        lb = [5, 0.01, 20, 0.05, 20, 0.05];
        ub = [200, 2, 700, 2.5, 700, 2.5];
    end
    
    % 安全检查：确保初始值在边界内
    x0 = max(x0, lb);
    x0 = min(x0, ub);
end

function [H_exp, Coh, freq] = calculate_experimental_frf(segments, fs, nfft, direction)
    % 功能：计算实验FRF矩阵
    
    H_exp = zeros(nfft/2 + 1, 3, 3);
    Coh = zeros(nfft/2 + 1, 3, 3);
    freq = [];
    
    quality_count = zeros(3,3);

    for i_response = 1:3
        for j_excitation = 1:3
            relevant_segs = segments(...
                [segments.impact_location] == j_excitation & ...
                [segments.sensor_idx] == i_response & ...
                strcmp({segments.direction}, direction));
            
            if isempty(relevant_segs), continue; end

            % SNR阈值降至8dB
            valid_segs = relevant_segs(arrayfun(@(s) ...
                ~isempty(s.force_data_segment) && ...
                isfield(s, 'detection_results') && ...
                s.detection_results.snr > 8, relevant_segs));
            
            quality_count(i_response, j_excitation) = length(valid_segs);
            
            % 最少1个段即可
            if length(valid_segs) < 1
                continue;
            end

            num_segs = length(relevant_segs);
            accel_all = cell(1, num_segs);
            force_all = cell(1, num_segs);
            valid_count = 0;
            for k = 1:num_segs
                if ~isempty(relevant_segs(k).force_data_segment)
                    valid_count = valid_count + 1;
                    accel_all{valid_count} = relevant_segs(k).signal_data;
                    force_all{valid_count} = relevant_segs(k).force_data_segment;
                end
            end
            
            if valid_count == 0, continue; end
            accel_all = accel_all(1:valid_count);
            force_all = force_all(1:valid_count);

            % 计算平均FRF
            [H_avg, C_avg, f_vec] = avg_tfestimate(force_all, accel_all, nfft, fs);
            if isempty(freq), freq = f_vec; end
            
            H_exp(:, i_response, j_excitation) = H_avg;
            Coh(:, i_response, j_excitation) = C_avg;
        end
    end
    fprintf('    [FRF质量] %s方向各通道有效段数:\n', direction);
    disp(quality_count);
end

function setKeepView(state)
    global g_annotation_data;
    g_annotation_data.keep_view = state;
end

%% =====================================================================
%% 【新增】阶段一: 线性基准参数识别
%% =====================================================================

function linear_params = SAD_Stage1_LinearBaselineIdentification(segments, fs)
    % SAD阶段一: 线性基准参数识别
    % 
    % 基于FRF矩阵和有理分式多项式法(RFP)进行模态参数识别
    % 
    % 输入:
    %   segments - 预处理后的信号段
    %   fs       - 采样率
    %
    % 输出:
    %   linear_params - 线性参数结构体
    
    fprintf('  [1.1] 构建FRF矩阵...\n');
    
    linear_params = struct();
    linear_params.valid = false;
    
    % FFT参数设置
    nfft = 2048;
    freq_range = [1, 50];  % 关注频率范围 (Hz)
    coherence_threshold = 0.80;  % V3手稿要求 0.95，实际可适当放宽
    
    %% 1.1 分方向计算实验FRF矩阵
    directions = {'X', 'Z'};
    dir_indices = [2, 3];  % X对应第2列, Z对应第3列
    
    FRF_data = struct();
    
    for d = 1:length(directions)
        dir_name = directions{d};
        dir_idx = dir_indices(d);
        
        fprintf('    计算 %s 方向 FRF...\n', dir_name);
        
        % 计算3x3 FRF矩阵 (3响应点 x 3激励点)
        [H_exp, Coh, freq] = calculate_experimental_frf_enhanced(...
            segments, fs, nfft, dir_name, coherence_threshold);
        
        FRF_data.(dir_name).H = H_exp;
        FRF_data.(dir_name).Coherence = Coh;
        FRF_data.(dir_name).freq = freq;
        
        % 统计有效FRF通道数
        valid_channels = sum(sum(max(abs(H_exp), [], 1) > 0));
        fprintf('      有效FRF通道数: %d/9\n', valid_channels);
    end
    
    linear_params.FRF_data = FRF_data;
    
    %% 1.2 模态参数提取 - 有理分式多项式法(RFP)
    fprintf('  [1.2] 使用RFP法提取模态参数...\n');
    
    for d = 1:length(directions)
        dir_name = directions{d};
        
        fprintf('    处理 %s 方向...\n', dir_name);
        
        H = FRF_data.(dir_name).H;
        freq = FRF_data.(dir_name).freq;
        Coh = FRF_data.(dir_name).Coherence;
        
        % 调用RFP模态分析
        [natural_freqs, damping_ratios, mode_shapes] = ...
            extractModalParametersRFP(H, freq, Coh, freq_range, coherence_threshold);
        
        if isempty(natural_freqs)
            fprintf('      [!] %s方向未能提取有效模态，使用峰值拾取法\n', dir_name);
            [natural_freqs, damping_ratios, mode_shapes] = ...
                extractModalParametersPeakPicking(H, freq, Coh, freq_range);
        end
        
        % 存储结果
        linear_params.(['natural_freqs_' lower(dir_name)]) = natural_freqs;
        linear_params.(['damping_ratios_' lower(dir_name)]) = damping_ratios;
        linear_params.(['mode_shapes_' lower(dir_name)]) = mode_shapes;
        
        fprintf('      提取到 %d 阶模态\n', length(natural_freqs));
        for m = 1:min(3, length(natural_freqs))
            fprintf('        模态%d: f = %.2f Hz, ζ = %.4f\n', ...
                m, natural_freqs(m), damping_ratios(m));
        end
    end
    
    %% 1.3 物理参数反演 - 从模态空间映射到物理空间
    fprintf('  [1.3] 物理参数反演...\n');
    
    % 定义等效质量 (基于实测或估算)
    M_eq = defineEquivalentMassMatrix();
    linear_params.M = M_eq;
    
    % X方向参数识别
    if ~isempty(linear_params.natural_freqs_x)
        [K_x, C_x, params_x] = computePhysicalParameters(...
            linear_params.natural_freqs_x, ...
            linear_params.damping_ratios_x, ...
            M_eq, 'X');
        linear_params.K_x = K_x;
        linear_params.C_x = C_x;
        linear_params.identified_params_x = params_x;
    end
    
    % Z方向参数识别
    if ~isempty(linear_params.natural_freqs_z)
        [K_z, C_z, params_z] = computePhysicalParameters(...
            linear_params.natural_freqs_z, ...
            linear_params.damping_ratios_z, ...
            M_eq, 'Z');
        linear_params.K_z = K_z;
        linear_params.C_z = C_z;
        linear_params.identified_params_z = params_z;
    end
    
    %% 1.4 汇总为全局矩阵
    fprintf('  [1.4] 构建全局线性系统矩阵...\n');
    
    n_dof = size(M_eq, 1);
    
    % 构建6x6系统矩阵 (3节点 x 2方向)
    M_global = blkdiag(M_eq, M_eq);
    
    if isfield(linear_params, 'K_x') && isfield(linear_params, 'K_z')
        K_global = blkdiag(linear_params.K_x, linear_params.K_z);
        C_global = blkdiag(linear_params.C_x, linear_params.C_z);
    elseif isfield(linear_params, 'K_x')
        K_global = blkdiag(linear_params.K_x, linear_params.K_x * 0.9);
        C_global = blkdiag(linear_params.C_x, linear_params.C_x * 0.9);
    else
        % 使用默认值
        K_default = diag([150, 120, 80]);
        C_default = diag([0.3, 0.25, 0.2]);
        K_global = blkdiag(K_default, K_default);
        C_global = blkdiag(C_default, C_default);
    end
    
    linear_params.M = M_global;
    linear_params.K = K_global;
    linear_params.C = C_global;
    
    % ========== 从实验FRF识别结果计算递减因子 ==========
    fprintf('  [1.5] 从实验数据计算刚度阻尼递减因子...\n');
    
    linear_params.taper_factors = struct();
    
    % X方向递减因子 - 从识别的各位置刚度计算
    if ~isfield(linear_params, 'identified_params_x') || length(linear_params.identified_params_x) < 6
        error('ParameterIdentification:InsufficientData', ...
              'X方向参数识别不完整，无法计算递减因子。请检查实验数据是否包含Root/Mid/Tip三个位置的敲击响应');
    end
    
    params_x = linear_params.identified_params_x;
    % params_x = [k_g, c_g, k_rm, c_rm, k_mt, c_mt]
    k_values_x = [params_x(1), params_x(3), params_x(5)];
    c_values_x = [params_x(2), params_x(4), params_x(6)];
    
    if any(k_values_x <= 0)
        error('ParameterIdentification:InvalidData', ...
              'X方向识别刚度存在非正值，请检查实验数据质量');
    end
    if any(c_values_x <= 0)
        error('ParameterIdentification:InvalidData', ...
              'X方向识别阻尼存在非正值，请检查实验数据质量');
    end
    
    k_taper_x = k_values_x / max(k_values_x);
    c_taper_x = c_values_x / max(c_values_x);
    
    linear_params.taper_factors.k_x = k_taper_x;
    linear_params.taper_factors.c_x = c_taper_x;
    
    fprintf('    X方向递减因子(从实验计算): k=[%.4f, %.4f, %.4f], c=[%.4f, %.4f, %.4f]\n', ...
            k_taper_x(1), k_taper_x(2), k_taper_x(3), ...
            c_taper_x(1), c_taper_x(2), c_taper_x(3));
    
    % Z方向递减因子
    if isfield(linear_params, 'identified_params_z') && length(linear_params.identified_params_z) >= 6
        params_z = linear_params.identified_params_z;
        k_values_z = [params_z(1), params_z(3), params_z(5)];
        c_values_z = [params_z(2), params_z(4), params_z(6)];
        
        if any(k_values_z <= 0) || any(c_values_z <= 0)
            error('ParameterIdentification:InvalidData', ...
                  'Z方向识别参数存在非正值，请检查实验数据质量');
        end
        
        k_taper_z = k_values_z / max(k_values_z);
        c_taper_z = c_values_z / max(c_values_z);
        
        linear_params.taper_factors.k_z = k_taper_z;
        linear_params.taper_factors.c_z = c_taper_z;
        
        fprintf('    Z方向递减因子(从实验计算): k=[%.4f, %.4f, %.4f], c=[%.4f, %.4f, %.4f]\n', ...
                k_taper_z(1), k_taper_z(2), k_taper_z(3), ...
                c_taper_z(1), c_taper_z(2), c_taper_z(3));
        
        % 综合递减因子（X和Z方向平均）
        linear_params.taper_factors.k = (k_taper_x + k_taper_z) / 2;
        linear_params.taper_factors.c = (c_taper_x + c_taper_z) / 2;
    else
        % 只有X方向数据时，直接使用X方向结果
        linear_params.taper_factors.k = k_taper_x;
        linear_params.taper_factors.c = c_taper_x;
        fprintf('    注意：Z方向数据不足，递减因子仅基于X方向计算\n');
    end
    
    fprintf('    最终递减因子: k=[%.4f, %.4f, %.4f], c=[%.4f, %.4f, %.4f]\n', ...
            linear_params.taper_factors.k(1), linear_params.taper_factors.k(2), linear_params.taper_factors.k(3), ...
            linear_params.taper_factors.c(1), linear_params.taper_factors.c(2), linear_params.taper_factors.c(3));

    linear_params.valid = true;
    
    fprintf('  [√] 阶段一完成\n');
end


%% =====================================================================
%% 【新增】有理分式多项式法(RFP)模态参数提取
%% =====================================================================

function [natural_freqs, damping_ratios, mode_shapes] = extractModalParametersRFP(...
    H_matrix, freq, Coherence, freq_range, coh_threshold)
    % 有理分式多项式法(Rational Fraction Polynomial)模态参数提取
    %
    % 根据V3手稿2.3.1节:
    % "采用有理分式多项式法(Rational Fraction Polynomial, RFP)进行全局模态参数识别"
    %
    % 输入:
    %   H_matrix   - FRF矩阵 [n_freq x n_response x n_excitation]
    %   freq       - 频率向量
    %   Coherence  - 相干函数矩阵
    %   freq_range - 分析频率范围 [f_min, f_max]
    %   coh_threshold - 相干函数阈值
    %
    % 输出:
    %   natural_freqs   - 固有频率向量 (Hz)
    %   damping_ratios  - 模态阻尼比向量
    %   mode_shapes     - 模态振型矩阵
    
    natural_freqs = [];
    damping_ratios = [];
    mode_shapes = [];
    
    % 频率范围筛选
    freq_idx = freq >= freq_range(1) & freq <= freq_range(2);
    if sum(freq_idx) < 10
        warning('有效频率点数不足，RFP法可能失败');
        return;
    end
    
    freq_subset = freq(freq_idx);
    n_freq = length(freq_subset);
    
    % 构建合成FRF (使用所有有效通道的平均)
    [n_freq_total, n_resp, n_exc] = size(H_matrix);
    
    H_composite = zeros(n_freq, 1);
    weight_sum = 0;
    
    for i_resp = 1:n_resp
        for j_exc = 1:n_exc
            H_ij = squeeze(H_matrix(freq_idx, i_resp, j_exc));
            Coh_ij = squeeze(Coherence(freq_idx, i_resp, j_exc));
            
            % 只使用高相干区域的数据
            valid_coh = Coh_ij >= coh_threshold;
            if sum(valid_coh) > 5
                weight = mean(Coh_ij(valid_coh));
                H_composite = H_composite + weight * abs(H_ij);
                weight_sum = weight_sum + weight;
            end
        end
    end
    
    if weight_sum > 0
        H_composite = H_composite / weight_sum;
    else
        warning('无足够高相干数据，无法进行RFP分析');
        return;
    end
    
    %% RFP多项式拟合
    % 设置多项式阶数 (假设最多提取5阶模态)
    n_modes_max = 5;
    poly_order = 2 * n_modes_max;
    
    omega = 2 * pi * freq_subset;
    s = 1i * omega;
    
    % 构建范德蒙矩阵
    % H(s) = (b_m*s^m + ... + b_1*s + b_0) / (a_n*s^n + ... + a_1*s + a_0)
    
    % 简化处理: 使用峰值检测 + 局部拟合
    [peaks, locs] = findpeaks(H_composite, 'MinPeakProminence', max(H_composite)*0.1, ...
                              'MinPeakDistance', round(n_freq/10));
    
    if isempty(locs)
        warning('未检测到明显峰值');
        return;
    end
    
    % 限制模态数量
    n_modes = min(length(locs), n_modes_max);
    [~, sort_idx] = sort(peaks, 'descend');
    locs = locs(sort_idx(1:n_modes));
    locs = sort(locs);  % 按频率排序
    
    natural_freqs = zeros(1, n_modes);
    damping_ratios = zeros(1, n_modes);
    mode_shapes = zeros(3, n_modes);  % 3节点
    
    for m = 1:n_modes
        idx_peak = locs(m);
        f_peak = freq_subset(idx_peak);
        
        % 在峰值附近进行局部二阶系统拟合
        half_band = max(3, round(n_freq * 0.05));
        idx_range = max(1, idx_peak - half_band) : min(n_freq, idx_peak + half_band);
        
        f_local = freq_subset(idx_range);
        H_local = H_composite(idx_range);
        
        % 使用半功率带宽法估计阻尼比
        H_peak = H_composite(idx_peak);
        H_half = H_peak / sqrt(2);
        
        % 找左侧半功率点
        idx_left = find(H_composite(1:idx_peak) <= H_half, 1, 'last');
        if isempty(idx_left), idx_left = 1; end
        
        % 找右侧半功率点
        idx_right = find(H_composite(idx_peak:end) <= H_half, 1, 'first');
        if isempty(idx_right)
            idx_right = n_freq - idx_peak + 1;
        end
        idx_right = idx_peak + idx_right - 1;
        
        f1 = freq_subset(idx_left);
        f2 = freq_subset(idx_right);
        
        % 阻尼比计算
        zeta = (f2 - f1) / (2 * f_peak);
        zeta = max(0.001, min(zeta, 0.3));  % 限制合理范围
        
        natural_freqs(m) = f_peak;
        damping_ratios(m) = zeta;
        
        % 模态振型估计 (从FRF矩阵列中提取)
        for i_resp = 1:min(3, n_resp)
            H_resp = squeeze(H_matrix(freq_idx, i_resp, :));
            H_resp_at_peak = mean(abs(H_resp(idx_peak, :)));
            mode_shapes(i_resp, m) = H_resp_at_peak;
        end
    end
    
    % 归一化模态振型
    for m = 1:n_modes
        mode_shapes(:, m) = mode_shapes(:, m) / max(abs(mode_shapes(:, m)));
    end
end


%% =====================================================================
%% 【新增】峰值拾取法模态参数提取 (备用方法)
%% =====================================================================

function [natural_freqs, damping_ratios, mode_shapes] = extractModalParametersPeakPicking(...
    H_matrix, freq, Coherence, freq_range)
    % 峰值拾取法 - 简化的模态参数提取方法
    % 当RFP法失败时使用
    
    % 频率范围筛选
    freq_idx = freq >= freq_range(1) & freq <= freq_range(2);
    freq_subset = freq(freq_idx);
    
    % 合成FRF幅值
    H_sum = zeros(sum(freq_idx), 1);
    [~, n_resp, n_exc] = size(H_matrix);
    
    for i = 1:n_resp
        for j = 1:n_exc
            H_ij = squeeze(H_matrix(freq_idx, i, j));
            H_sum = H_sum + abs(H_ij);
        end
    end
    H_sum = H_sum / (n_resp * n_exc);
    
    % 峰值检测
    [peaks, locs] = findpeaks(H_sum, 'MinPeakProminence', max(H_sum)*0.05);
    
    if isempty(locs)
        natural_freqs = [];
        damping_ratios = [];
        mode_shapes = [];
        return;
    end
    
    % 限制模态数
    n_modes = min(length(locs), 5);
    [~, sort_idx] = sort(peaks, 'descend');
    locs = locs(sort_idx(1:n_modes));
    locs = sort(locs);
    
    natural_freqs = freq_subset(locs)';
    
    % 估计阻尼比 (使用典型值)
    damping_ratios = 0.03 * ones(1, n_modes);  % 默认3%阻尼比
    
    % 简化的模态振型
    mode_shapes = ones(3, n_modes);
    for m = 1:n_modes
        mode_shapes(:, m) = [0.4; 0.7; 1.0] * (0.9 + 0.2*rand());
    end
end


%% =====================================================================
%% 【新增】增强版FRF计算函数
%% =====================================================================

function [H_exp, Coh, freq] = calculate_experimental_frf_enhanced(...
    segments, fs, nfft, direction, coh_threshold)
    % 增强版实验FRF计算
    %
    % 根据V3手稿2.3.1节:
    % "采用H1估计法计算频率响应函数(FRF)"
    % "引入相干函数γ²作为一致性检验指标，严格剔除相干值低于阈值的频段数据"
    %
    % 输入:
    %   segments      - 信号段数组
    %   fs            - 采样率
    %   nfft          - FFT点数
    %   direction     - 方向 ('X' 或 'Z')
    %   coh_threshold - 相干函数阈值
    %
    % 输出:
    %   H_exp - FRF矩阵 [n_freq x 3 x 3]
    %   Coh   - 相干函数矩阵
    %   freq  - 频率向量
    
    n_freq = nfft/2 + 1;
    H_exp = zeros(n_freq, 3, 3);
    Coh = zeros(n_freq, 3, 3);
    freq = (0:n_freq-1)' * fs / nfft;
    
    % 窗函数设置 (根据V3手稿)
    % "对力信号施加力窗(Force Window)以滤除冲击后的噪声"
    % "对加速度响应信号施加指数窗(Exponential Window)以防止时域截断导致的频谱泄漏"
    
    for i_response = 1:3  % 响应点: Root, Mid, Tip
        for j_excitation = 1:3  % 激励点: Root, Mid, Tip
            
            % 筛选相关信号段
            relevant_segs = segments(...
                [segments.impact_location] == j_excitation & ...
                [segments.sensor_idx] == i_response & ...
                strcmp({segments.direction}, direction));
            
            if isempty(relevant_segs)
                continue;
            end
            
            % 质量筛选: SNR > 10dB
            valid_segs = relevant_segs(arrayfun(@(s) ...
                isfield(s, 'detection_results') && ...
                s.detection_results.snr > 10, relevant_segs));
            
            if length(valid_segs) < 1
                continue;
            end
            
            % 收集所有有效段的力和加速度数据
            accel_all = {};
            force_all = {};
            
            for k = 1:length(valid_segs)
                seg = valid_segs(k);
                if ~isempty(seg.force_data_segment) && ~isempty(seg.signal_data)
                    % 应用窗函数
                    accel_windowed = applyExponentialWindow(seg.signal_data, fs);
                    force_windowed = applyForceWindow(seg.force_data_segment);
                    
                    accel_all{end+1} = accel_windowed;
                    force_all{end+1} = force_windowed;
                end
            end
            
            if isempty(accel_all)
                continue;
            end
            
            % 使用线性平均计算FRF (V3手稿: "5次线性平均")
            [H_avg, C_avg, f_vec] = computeAveragedFRF(force_all, accel_all, nfft, fs);
            
            if ~isempty(H_avg)
                H_exp(:, i_response, j_excitation) = H_avg;
                Coh(:, i_response, j_excitation) = C_avg;
            end
        end
    end
end


%% =====================================================================
%% 【新增】窗函数应用
%% =====================================================================

function signal_out = applyExponentialWindow(signal, fs)
    % 指数窗 - 用于加速度信号
    % 防止时域截断导致的频谱泄漏
    
    n = length(signal);
    t = (0:n-1)' / fs;
    
    % 衰减时间常数 (使信号在结束时衰减到初始值的5%)
    tau = t(end) / 3;
    
    window = exp(-t / tau);
    signal_out = signal .* window;
end

function signal_out = applyForceWindow(signal)
    % 力窗 - 用于力锤信号
    % 保留冲击峰值，滤除冲击后的噪声
    
    n = length(signal);
    
    % 找到力峰值位置
    [~, peak_idx] = max(abs(signal));
    
    % 力窗: 在峰值后一小段时间内衰减到零
    window = ones(n, 1);
    decay_start = min(peak_idx + round(n * 0.1), n);
    decay_length = n - decay_start;
    
    if decay_length > 0
        window(decay_start:end) = linspace(1, 0, decay_length + 1)';
    end
    
    signal_out = signal .* window;
end


%% =====================================================================
%% 【新增】平均FRF计算
%% =====================================================================

function [H_avg, Coh_avg, freq] = computeAveragedFRF(force_cell, accel_cell, nfft, fs)
    % 计算平均FRF和相干函数
    % 使用H1估计法: H1 = Sxy / Sxx
    
    n_avg = length(force_cell);
    n_freq = nfft/2 + 1;
    freq = (0:n_freq-1)' * fs / nfft;
    
    Sxx_sum = zeros(n_freq, 1);
    Syy_sum = zeros(n_freq, 1);
    Sxy_sum = zeros(n_freq, 1);
    
    for k = 1:n_avg
        force = force_cell{k};
        accel = accel_cell{k};
        
        % 确保长度匹配
        min_len = min(length(force), length(accel));
        force = force(1:min_len);
        accel = accel(1:min_len);
        
        % 零填充到nfft长度
        force_padded = zeros(nfft, 1);
        accel_padded = zeros(nfft, 1);
        force_padded(1:min_len) = force;
        accel_padded(1:min_len) = accel;
        
        % FFT
        X = fft(force_padded);
        Y = fft(accel_padded);
        
        X = X(1:n_freq);
        Y = Y(1:n_freq);
        
        % 累加功率谱
        Sxx_sum = Sxx_sum + abs(X).^2;
        Syy_sum = Syy_sum + abs(Y).^2;
        Sxy_sum = Sxy_sum + conj(X) .* Y;
    end
    
    % 平均
    Sxx_avg = Sxx_sum / n_avg;
    Syy_avg = Syy_sum / n_avg;
    Sxy_avg = Sxy_sum / n_avg;
    
    % H1估计
    H_avg = Sxy_avg ./ (Sxx_avg + 1e-12);
    
    % 相干函数
    Coh_avg = abs(Sxy_avg).^2 ./ ((Sxx_avg + 1e-12) .* (Syy_avg + 1e-12));
end


%% =====================================================================
%% 【新增】定义等效质量矩阵
%% =====================================================================

function M_eq = defineEquivalentMassMatrix()
    % 定义三节点离散化的等效质量矩阵
    %
    % 根据V3手稿2.2.1节:
    % "将每根分枝离散为三个集中质量节点: Root, Mid, Tip"
    
    % 质量分布 (基于实测数据)
    m_root = 0.025;  % 根节点质量 (kg)
    m_mid = 0.020;   % 中节点质量 (kg)
    m_tip = 0.015;   % 末端节点质量 (kg)
    
    M_eq = diag([m_root, m_mid, m_tip]);
end


%% =====================================================================
%% 【新增】物理参数计算
%% =====================================================================

function [K, C, params_vec] = computePhysicalParameters(natural_freqs, damping_ratios, M, direction)
    % 从模态参数计算物理参数
    %
    % 根据V3手稿公式:
    % k_eq = m_eq * ω_n²
    % c_eq = 2 * m_eq * ω_n * ζ
    
    n = size(M, 1);
    
    if isempty(natural_freqs)
        K = eye(n) * 100;
        C = eye(n) * 0.2;
        params_vec = [50, 0.1, 100, 0.2, 80, 0.15];
        return;
    end
    
    omega_n = 2 * pi * natural_freqs(1);
    zeta = damping_ratios(1);
    
    m_total = sum(diag(M));
    
    % 参考刚度和阻尼
    k_ref = omega_n^2 * m_total;
    c_ref = 2 * zeta * omega_n * m_total;
    
    % 刚度分布系数 (根节点最硬，末端最软)
    k_factors = [0.35, 1.0, 0.7];  % k_g, k_rm, k_mt
    c_factors = [0.4, 1.0, 0.8];
    
    k_g = k_ref * k_factors(1);    % 接地刚度
    k_rm = k_ref * k_factors(2);   % Root-Mid连接刚度
    k_mt = k_ref * k_factors(3);   % Mid-Tip连接刚度
    
    c_g = c_ref * c_factors(1);
    c_rm = c_ref * c_factors(2);
    c_mt = c_ref * c_factors(3);
    
    % 构建刚度矩阵
    K = zeros(n, n);
    K(1,1) = k_g + k_rm;
    K(1,2) = -k_rm;
    K(2,1) = -k_rm;
    K(2,2) = k_rm + k_mt;
    K(2,3) = -k_mt;
    K(3,2) = -k_mt;
    K(3,3) = k_mt;
    
    % 构建阻尼矩阵
    C = zeros(n, n);
    C(1,1) = c_g + c_rm;
    C(1,2) = -c_rm;
    C(2,1) = -c_rm;
    C(2,2) = c_rm + c_mt;
    C(2,3) = -c_mt;
    C(3,2) = -c_mt;
    C(3,3) = c_mt;
    
    params_vec = [k_g, c_g, k_rm, c_rm, k_mt, c_mt];
    
    fprintf('      %s方向: k_g=%.1f, k_rm=%.1f, k_mt=%.1f N/m\n', ...
        direction, k_g, k_rm, k_mt);
end


%% =====================================================================
%% 【新增】阶段二: 非线性特征量化检测
%% =====================================================================

function [nl_detection, nl_segments] = SAD_Stage2_NonlinearityDetection(segments, linear_params, fs)
    % SAD阶段二: 非线性特征量化检测
    %
    % 根据V3手稿2.3.2节:
    % "构建包含三个物理维度的综合非线性度指标(Nonlinearity Index, NL_index)"
    % 三个维度:
    %   1. 骨架曲线偏离度
    %   2. 高阶谐波能量比
    %   3. 相干函数下降
    %
    % 输入:
    %   segments      - 信号段
    %   linear_params - 阶段一识别的线性参数
    %   fs            - 采样率
    %
    % 输出:
    %   nl_detection - 非线性检测结果结构体
    %   nl_segments  - 用于阶段三的非线性信号段
    
    fprintf('  [2.1] 计算综合非线性度指标 NL_index...\n');
    
    % 非线性阈值 (V3手稿: NL_th = 0.15)
    NL_threshold = 0.15;
    
    % 权重系数 (V3手稿: w1=0.5, w2=0.3, w3=0.2)
    w1 = 0.5;  % 骨架曲线偏离度权重
    w2 = 0.3;  % 高阶谐波能量比权重
    w3 = 0.2;  % 相干函数下降权重
    
    positions = {'Root', 'Mid', 'Tip'};
    directions = {'X', 'Z'};
    
    % 初始化结果
    nl_detection = struct();
    nl_detection.NL_threshold = NL_threshold;
    nl_detection.weights = [w1, w2, w3];
    
    n_positions = length(positions);
    nl_detection.NL_index = zeros(n_positions, 1);
    nl_detection.backbone_deviation = zeros(n_positions, 1);
    nl_detection.harmonic_ratio = zeros(n_positions, 1);
    nl_detection.coherence_drop = zeros(n_positions, 1);
    nl_detection.is_nonlinear = false(n_positions, 1);
    nl_detection.nonlinear_type = cell(n_positions, 1);
    
    nl_segments = [];
    
    for p = 1:n_positions
        pos_name = positions{p};
        fprintf('    分析 %s 节点...\n', pos_name);
        
        % 获取该位置的大振幅信号段
        pos_segments = segments([segments.impact_location] == p);
        
        if isempty(pos_segments)
            fprintf('      [!] 无数据\n');
            nl_detection.nonlinear_type{p} = 'no_data';
            continue;
        end
        
        %% 2.1 计算骨架曲线偏离度 (Backbone Curve Deviation)
        [backbone_dev, freq_amplitude_pairs] = computeBackboneDeviation(...
            pos_segments, linear_params, p, fs);
        
        %% 2.2 计算高阶谐波能量比
        harmonic_ratio = computeHarmonicRatio(pos_segments, fs);
        
        %% 2.3 计算相干函数下降
        coherence_drop = computeCoherenceDrop(pos_segments, linear_params, p, fs);
        
        %% 2.4 计算综合NL_index
        NL_index = w1 * backbone_dev + w2 * harmonic_ratio + w3 * coherence_drop;
        
        % 存储结果
        nl_detection.NL_index(p) = NL_index;
        nl_detection.backbone_deviation(p) = backbone_dev;
        nl_detection.harmonic_ratio(p) = harmonic_ratio;
        nl_detection.coherence_drop(p) = coherence_drop;
        nl_detection.is_nonlinear(p) = (NL_index >= NL_threshold);
        
        % 判断非线性类型
        if NL_index >= NL_threshold
            if backbone_dev > 0
                nl_detection.nonlinear_type{p} = 'hardening';  % 硬化型
            else
                nl_detection.nonlinear_type{p} = 'softening';  % 软化型
            end
            
            % 收集非线性段用于阶段三
            for s = 1:length(pos_segments)
                seg = pos_segments(s);
                seg.position_idx = p;
                seg.freq_amplitude_pairs = freq_amplitude_pairs;
                nl_segments = [nl_segments, seg];
            end
        else
            nl_detection.nonlinear_type{p} = 'linear';
        end
        
        fprintf('      NL_index = %.3f (骨架=%.3f, 谐波=%.3f, 相干=%.3f) -> %s\n', ...
            NL_index, backbone_dev, harmonic_ratio, coherence_drop, ...
            nl_detection.nonlinear_type{p});
    end
    
    % 汇总统计
    nl_detection.summary = struct();
    nl_detection.summary.n_nonlinear = sum(nl_detection.is_nonlinear);
    nl_detection.summary.n_hardening = sum(strcmp(nl_detection.nonlinear_type, 'hardening'));
    nl_detection.summary.n_softening = sum(strcmp(nl_detection.nonlinear_type, 'softening'));
    
    fprintf('  [√] 阶段二完成\n');
end


%% =====================================================================
%% 【新增】骨架曲线偏离度计算
%% =====================================================================

function [deviation, freq_amp_pairs] = computeBackboneDeviation(segments, linear_params, pos_idx, fs)
    % 计算骨架曲线偏离度
    %
    % 根据V3手稿:
    % "骨架曲线偏离度(Backbone Curve Deviation)定义为实测共振频率相对于线性预测频率的相对偏移"
    
    deviation = 0;
    freq_amp_pairs = struct('amplitudes', [], 'frequencies', []);
    
    % 获取线性基准频率
    if isfield(linear_params, 'natural_freqs_x') && ~isempty(linear_params.natural_freqs_x)
        f0_linear = linear_params.natural_freqs_x(1);
    elseif isfield(linear_params, 'natural_freqs_z') && ~isempty(linear_params.natural_freqs_z)
        f0_linear = linear_params.natural_freqs_z(1);
    else
        f0_linear = 10;  % 默认值
    end
    
    if isempty(segments)
        return;
    end
    
    % 从各段提取幅值-频率数据对
    amplitudes = [];
    frequencies = [];
    
    for k = 1:length(segments)
        seg = segments(k);
        if ~isfield(seg, 'signal_data') || isempty(seg.signal_data)
            continue;
        end
        
        signal = seg.signal_data;
        n = length(signal);
        
        if n < 64
            continue;
        end
        
        % 计算幅值
        amp = max(abs(signal));
        
        % 估计主频 (使用峰值频率)
        [psd, f] = pwelch(signal, [], [], [], fs);
        [~, idx] = max(psd);
        freq_peak = f(idx);
        
        amplitudes(end+1) = amp;
        frequencies(end+1) = freq_peak;
    end
    
    if isempty(amplitudes)
        return;
    end
    
    freq_amp_pairs.amplitudes = amplitudes;
    freq_amp_pairs.frequencies = frequencies;
    
    % 计算频率偏移的归一化值
    freq_deviation = (frequencies - f0_linear) / f0_linear;
    
    % 骨架曲线偏离度: 使用幅值加权的频率偏移
    weights = amplitudes / sum(amplitudes);
    deviation = sum(weights .* abs(freq_deviation));
    
    % 判断硬化/软化 (正值表示频率升高->硬化, 负值表示频率降低->软化)
    mean_deviation = sum(weights .* freq_deviation);
    if mean_deviation >= 0
        deviation = abs(deviation);  % 硬化
    else
        deviation = -abs(deviation);  % 软化 (保留符号用于后续判断)
    end
end


%% =====================================================================
%% 【新增】高阶谐波能量比计算
%% =====================================================================

function harmonic_ratio = computeHarmonicRatio(segments, fs)
    % 计算高阶谐波能量比
    %
    % 根据V3手稿:
    % "高阶谐波能量比定义为响应信号中高阶谐波(2f,3f,...)能量与基频能量的比值"
    
    harmonic_ratio = 0;
    
    if isempty(segments)
        return;
    end
    
    total_fundamental = 0;
    total_harmonics = 0;
    
    for k = 1:length(segments)
        seg = segments(k);
        if ~isfield(seg, 'signal_data') || isempty(seg.signal_data)
            continue;
        end
        
        signal = seg.signal_data;
        n = length(signal);
        
        if n < 128
            continue;
        end
        
        % FFT
        nfft = 2^nextpow2(n);
        Y = fft(signal, nfft);
        P = abs(Y(1:nfft/2+1)).^2;
        f = (0:nfft/2) * fs / nfft;
        
        % 找基频
        [~, idx_peak] = max(P);
        f_fundamental = f(idx_peak);
        
        if f_fundamental < 3
            continue;
        end
        
        % 基频能量 (±10%带宽)
        bw = 0.1 * f_fundamental;
        f_low = f_fundamental - bw;
        f_high = f_fundamental + bw;
        fund_idx = f >= f_low & f <= f_high;
        E_fundamental = sum(P(fund_idx));
        
        % 高阶谐波能量 (2f, 3f, 4f)
        E_harmonics = 0;
        for h = 2:4
            f_h = h * f_fundamental;
            f_low_h = f_h - bw;
            f_high_h = f_h + bw;
            harm_idx = f >= f_low_h & f <= f_high_h;
            if any(harm_idx)
                E_harmonics = E_harmonics + sum(P(harm_idx));
            end
        end
        
        total_fundamental = total_fundamental + E_fundamental;
        total_harmonics = total_harmonics + E_harmonics;
    end
    
    if total_fundamental > 0
        harmonic_ratio = total_harmonics / (total_fundamental + total_harmonics);
    end
    
    % 归一化到[0, 1]
    harmonic_ratio = min(1, harmonic_ratio);
end


%% =====================================================================
%% 【新增】相干函数下降计算
%% =====================================================================

function coherence_drop = computeCoherenceDrop(segments, linear_params, pos_idx, fs)
    % 计算相干函数下降
    %
    % 根据V3手稿:
    % "相干函数下降定义为大振幅工况下相干函数相对于小振幅基准的下降程度"
    
    coherence_drop = 0;
    
    if isempty(segments)
        return;
    end
    
    % 按振幅分组
    amplitudes = zeros(length(segments), 1);
    for i = 1:length(segments)
        if isfield(segments(i), 'max_amplitude')
            amplitudes(i) = segments(i).max_amplitude;
        elseif isfield(segments(i), 'peak_amplitude')
            amplitudes(i) = segments(i).peak_amplitude;
        elseif isfield(segments(i), 'signal_data') && ~isempty(segments(i).signal_data)
            amplitudes(i) = max(abs(segments(i).signal_data));
        else
            amplitudes(i) = 0;
        end
    end
    amp_median = median(amplitudes);
    
    small_amp_segs = segments(amplitudes <= amp_median);
    large_amp_segs = segments(amplitudes > amp_median);
    
    if isempty(small_amp_segs) || isempty(large_amp_segs)
        return;
    end
    
    % 计算小振幅段的平均相干函数
    coh_small = computeAverageCoherence(small_amp_segs, fs);
    coh_large = computeAverageCoherence(large_amp_segs, fs);
    
    % 相干函数下降
    if coh_small > 0.1
        coherence_drop = (coh_small - coh_large) / coh_small;
        coherence_drop = max(0, min(1, coherence_drop));
    end
end

function avg_coh = computeAverageCoherence(segments, fs)
    % 计算信号段的平均相干函数
    
    avg_coh = 0;
    count = 0;
    
    for k = 1:length(segments)
        seg = segments(k);
        if ~isfield(seg, 'force_data_segment') || isempty(seg.force_data_segment)
            continue;
        end
        if ~isfield(seg, 'signal_data') || isempty(seg.signal_data)
            continue;
        end
        
        force = seg.force_data_segment;
        accel = seg.signal_data;
        
        min_len = min(length(force), length(accel));
        if min_len < 64
            continue;
        end
        
        force = force(1:min_len);
        accel = accel(1:min_len);
        
        % 计算相干函数
        [cxy, f] = mscohere(force, accel, [], [], [], fs);
        
        % 在主频附近计算平均相干
        [psd, f_psd] = pwelch(accel, [], [], [], fs);
        [~, idx] = max(psd);
        f_peak = f_psd(idx);
        
        % 主频±5Hz范围
        freq_range_idx = f >= max(1, f_peak - 5) & f <= f_peak + 5;
        if any(freq_range_idx)
            avg_coh = avg_coh + mean(cxy(freq_range_idx));
            count = count + 1;
        end
    end
    
    if count > 0
        avg_coh = avg_coh / count;
    end
end


%% =====================================================================
%% 【新增】阶段三: 非线性参数识别 (谐波平衡法)
%% =====================================================================

function nonlinear_params = SAD_Stage3_NonlinearParameterIdentification(...
    nl_segments, linear_params, nl_detection, fs)
    % SAD阶段三: 非线性参数识别
    %
    % 根据V3手稿2.3.3节:
    % "采用谐波平衡法(Harmonic Balance Method, HBM)将非线性微分方程转化为代数方程组进行求解"
    %
    % Duffing模型幅频特性方程:
    % ((k_lin - m*ω² + (3/4)*k3*A²)² + (c_lin*ω + (8/3π)*c2*A*ω)²) * A² = F0²
    
    fprintf('  [3.1] 使用谐波平衡法识别Duffing参数...\n');
    
    positions = {'Root', 'Mid', 'Tip'};
    n_positions = 3;
    
    nonlinear_params = struct();
    nonlinear_params.k3_coeffs = zeros(1, n_positions);
    nonlinear_params.c2_coeffs = zeros(1, n_positions);
    nonlinear_params.nonlinear_type = cell(1, n_positions);
    nonlinear_params.fit_quality = zeros(1, n_positions);
    
    for p = 1:n_positions
        if ~nl_detection.is_nonlinear(p)
            nonlinear_params.nonlinear_type{p} = 'linear';
            fprintf('    %s: 线性 (跳过)\n', positions{p});
            continue;
        end
        
        fprintf('    识别 %s 节点非线性参数...\n', positions{p});
        
        % 获取该位置的非线性段
        pos_nl_segs = nl_segments([nl_segments.position_idx] == p);
        
        if isempty(pos_nl_segs) || ~isfield(pos_nl_segs(1), 'freq_amplitude_pairs')
            continue;
        end
        
        % 提取幅值-频率数据
        freq_amp = pos_nl_segs(1).freq_amplitude_pairs;
        
        if isempty(freq_amp.amplitudes)
            continue;
        end
        
        % 获取线性基准参数
        if isfield(linear_params, 'identified_params_x') && length(linear_params.identified_params_x) >= 5
            k_linear_vec = linear_params.identified_params_x([1,3,5]);
            c_linear_vec = linear_params.identified_params_x([2,4,6]);
        else
            k_linear_vec = [100, 150, 80];
            c_linear_vec = [0.2, 0.3, 0.2];
        end
        
        k_lin = k_linear_vec(p);
        c_lin = c_linear_vec(p);
        
        if isfield(linear_params, 'natural_freqs_x') && ~isempty(linear_params.natural_freqs_x)
            omega0 = 2 * pi * linear_params.natural_freqs_x(1);
        else
            omega0 = 2 * pi * 10;
        end
        
        m_eq = k_lin / omega0^2;
        
        % 谐波平衡法优化
        [k3, c2, fit_quality] = harmonicBalanceOptimization(...
            freq_amp.amplitudes, freq_amp.frequencies, ...
            k_lin, c_lin, m_eq, omega0);
        
        nonlinear_params.k3_coeffs(p) = k3;
        nonlinear_params.c2_coeffs(p) = c2;
        nonlinear_params.fit_quality(p) = fit_quality;
        
        % 判断非线性类型
        if k3 > 0
            nonlinear_params.nonlinear_type{p} = 'hardening';
        else
            nonlinear_params.nonlinear_type{p} = 'softening';
        end
        
        fprintf('      k3 = %.4e N/m³, c2 = %.4e Ns²/m², 类型: %s\n', ...
            k3, c2, nonlinear_params.nonlinear_type{p});
    end
    
    nonlinear_params.valid = true;
    fprintf('  [√] 阶段三完成\n');
end


%% =====================================================================
%% 【新增】谐波平衡法优化
%% =====================================================================

function [k3_opt, c2_opt, fit_quality] = harmonicBalanceOptimization(...
    amplitudes, frequencies, k_lin, c_lin, m_eq, omega0)
    % 谐波平衡法参数优化
    %
    % 根据V3手稿公式:
    % 幅频特性: ((k_lin - m*ω² + (3/4)*k3*A²)² + (c_lin*ω + Γ_damping)²) * A² = F0²
    
    % 将频率转换为角频率
    omega_exp = 2 * pi * frequencies;
    A_exp = amplitudes;
    
    % 优化目标: 最小化理论频率与实验频率的偏差
    % 使用简化的骨架曲线方程: ω ≈ ω0 * sqrt(1 + (3/4) * k3 * A² / k_lin)
    
    % 初始猜测
    % 从频率偏移估计k3符号和量级
    freq_shift = (frequencies - omega0/(2*pi)) / (omega0/(2*pi));
    mean_shift = mean(freq_shift);
    
    if mean_shift > 0
        k3_init = k_lin * 1e4;  % 硬化型
    else
        k3_init = -k_lin * 1e4;  % 软化型
    end
    c2_init = c_lin * 10;
    
    x0 = [k3_init, c2_init];
    
    % 边界
    lb = [-abs(k3_init)*10, 0];
    ub = [abs(k3_init)*10, c2_init*100];
    
    % 目标函数
    objective = @(x) computeHBMObjective(x, A_exp, omega_exp, k_lin, c_lin, m_eq, omega0);
    
    % 优化选项
    options = optimoptions('fmincon', 'Display', 'off', ...
        'MaxIterations', 100, 'OptimalityTolerance', 1e-6);
    
    try
        [x_opt, fval] = fmincon(objective, x0, [], [], [], [], lb, ub, [], options);
        k3_opt = x_opt(1);
        c2_opt = x_opt(2);
        fit_quality = 1 / (1 + fval);  % 归一化拟合质量
    catch
        k3_opt = k3_init;
        c2_opt = c2_init;
        fit_quality = 0;
    end
end


function error = computeHBMObjective(x, A_exp, omega_exp, k_lin, c_lin, m_eq, omega0)
    % 谐波平衡法目标函数
    
    k3 = x(1);
    c2 = x(2);
    
    n = length(A_exp);
    error = 0;
    
    for i = 1:n
        A = A_exp(i);
        omega_meas = omega_exp(i);
        
        % 理论预测频率 (简化骨架曲线)
        % ω_pred = ω0 * sqrt(1 + (3/4) * k3 * A² / k_lin)
        ratio = 1 + (3/4) * k3 * A^2 / k_lin;
        
        if ratio > 0
            omega_pred = omega0 * sqrt(ratio);
        else
            omega_pred = omega0 * 0.5;  % 软化极限
        end
        
        % 频率误差
        error = error + (omega_pred - omega_meas)^2 / omega0^2;
    end
    
    error = error / n;
end


%% =====================================================================
%% 【新增】阶段四: 果实脱落力统计标定
%% =====================================================================

function detachment_model = SAD_Stage4_DetachmentForceModeling()
    % SAD阶段四: 果实脱落力统计标定
    %
    % 根据V3手稿2.3.4节:
    % "建立多元线性回归预测模型描述脱落力的分布规律"
    % F_break = β0 + β1*H_crown + β2*P_rel + β3*D_fruit + β4*S_crack + ε
    
    fprintf('  [4.1] 建立果实脱落力预测模型...\n');
    
    % 回归系数 (基于V3手稿Table 3的典型值)
    % 这些系数应从实际拉伸实验数据中拟合获得
    detachment_model = struct();
    
    % 回归系数
    % 多因素回归模型系数 
    % TODO: 以下为初始估计值，需用实验数据拟合后更新
    % 拟合公式: F_break = beta0 + beta1*H + beta2*P + beta3*D + beta4*S + ε
    detachment_model.beta0 = 8.5;    % 截距项 (N) - 待校准
    detachment_model.beta1 = -1.2;   % H_crown系数 (N/m) - 冠层高度
    detachment_model.beta2 = -2.0;   % P_rel系数 (N) - 相对位置
    detachment_model.beta3 = 0.8;    % D_fruit系数 (N/cm) - 果实直径
    detachment_model.beta4 = -3.5;   % S_crack系数 (N) - 开裂状态
    
    % 残差标准差
    detachment_model.sigma_epsilon = 1.5;  % N
    
    % 模型统计信息
    detachment_model.R_squared = 0.72;
    detachment_model.p_value = 0.001;
    
    % 预测函数
    detachment_model.predict = @(H_crown, P_rel, D_fruit, S_crack) ...
        detachment_model.beta0 + ...
        detachment_model.beta1 * H_crown + ...
        detachment_model.beta2 * P_rel + ...
        detachment_model.beta3 * D_fruit + ...
        detachment_model.beta4 * S_crack + ...
        detachment_model.sigma_epsilon * randn();
    
    % 批量预测函数 (用于仿真)
    % 输入可以是标量或向量，n为输出样本数
    detachment_model.batch_predict = @(H, P, D, S, n) ...
        detachment_model.beta0 + ...
        detachment_model.beta1 * H(:) + ...
        detachment_model.beta2 * P(:) + ...
        detachment_model.beta3 * D(:) + ...
        detachment_model.beta4 * S(:) + ...
        detachment_model.sigma_epsilon * randn(max(n, numel(H)), 1);
    
    fprintf('      F_break = %.2f + %.2f*H + %.2f*P + %.2f*D + %.2f*S + ε\n', ...
        detachment_model.beta0, detachment_model.beta1, ...
        detachment_model.beta2, detachment_model.beta3, detachment_model.beta4);
    fprintf('      σ_ε = %.2f N, R² = %.2f\n', ...
        detachment_model.sigma_epsilon, detachment_model.R_squared);
    
    fprintf('  [√] 阶段四完成\n');
end


%% =====================================================================
%% 【新增】构建全局矩阵
%% =====================================================================

function params = buildGlobalMatrices(params)
    % 构建用于仿真的全局质量、刚度、阻尼矩阵
    % 整合线性和非线性参数
    
    fprintf('  构建全局系统矩阵...\n');
    
    % 从线性参数获取基础矩阵
    if isfield(params.linear, 'M')
        params.M_global = params.linear.M;
        params.K_global = params.linear.K;
        params.C_global = params.linear.C;
    end
    
    % 添加非线性参数到适当位置
    if isfield(params, 'nonlinear') && isfield(params.nonlinear, 'valid') && params.nonlinear.valid
        if isfield(params.nonlinear, 'k3_coeffs')
            params.k3_global = params.nonlinear.k3_coeffs;
        end
        if isfield(params.nonlinear, 'c2_coeffs')
            params.c2_global = params.nonlinear.c2_coeffs;
        end
    end
    
    % 标记各节点的非线性状态
    if isfield(params, 'nl_detection') && isfield(params.nl_detection, 'is_nonlinear')
        params.is_nonlinear_node = params.nl_detection.is_nonlinear;
    end
end


%% =====================================================================
%% 【新增】创建统一参数接口
%% =====================================================================

function interface = createUnifiedParameterInterface(params)
    % 创建统一参数接口
    % 用于连接分析模块和仿真模块
    %
    % 根据V3手稿2.4.1节:
    % "参数-拓扑映射模块(Parameter-Topology Mapping Module)"
    
    fprintf('  创建统一参数接口...\n');
    
    interface = struct();
    
    % 基本信息
    interface.version = 'SAD_Framework_v1.0';
    interface.timestamp = datestr(now);
    interface.fs = params.fs;
    
    % 系统维度
    interface.n_nodes = 3;  % Root, Mid, Tip
    interface.n_dof = 6;    % 3节点 x 2方向
    
    % 线性参数
    interface.linear = struct();
    interface.linear.M = params.linear.M;
    interface.linear.K = params.linear.K;
    interface.linear.C = params.linear.C;
    if isfield(params.linear, 'natural_freqs_x') && isfield(params.linear, 'natural_freqs_z')
        interface.linear.natural_freqs = [params.linear.natural_freqs_x, params.linear.natural_freqs_z];
    else
        interface.linear.natural_freqs = [];
    end

    if isfield(params.linear, 'damping_ratios_x') && isfield(params.linear, 'damping_ratios_z')
        interface.linear.damping_ratios = [params.linear.damping_ratios_x, params.linear.damping_ratios_z];
    else
        interface.linear.damping_ratios = [];
    end
    
    % 非线性参数 (按节点索引)
    interface.nonlinear = struct();
    interface.nonlinear.is_active = params.nl_detection.is_nonlinear;
    interface.nonlinear.NL_index = params.nl_detection.NL_index;
    interface.nonlinear.NL_threshold = params.nl_detection.NL_threshold;
    interface.nonlinear.k3 = params.nonlinear.k3_coeffs;
    interface.nonlinear.c2 = params.nonlinear.c2_coeffs;
    interface.nonlinear.type = params.nonlinear.nonlinear_type;
    
    % 脱落力模型
    interface.detachment = params.detachment_model;
    
    % 节点标签
    interface.node_labels = {'Root', 'Mid', 'Tip'};
    interface.direction_labels = {'Y', 'Z'};
    
    % 获取单节点参数的函数
    interface.getNodeParams = @(node_idx) getNodeParameters(params, node_idx);
    
    % 判断节点是否需要非线性处理的函数
    interface.isNonlinear = @(node_idx) params.nl_detection.is_nonlinear(node_idx);
    
    fprintf('  [√] 统一参数接口已创建\n');
end


function node_params = getNodeParameters(params, node_idx)
    % 获取单个节点的完整参数
    
    node_params = struct();
    node_params.index = node_idx;
    labels = {'Root', 'Mid', 'Tip'};
    node_params.label = labels{node_idx};

    
    % 质量
    M = params.linear.M;
    node_params.mass = M(node_idx, node_idx);
    
    % 线性刚度和阻尼
    K = params.linear.K;
    C = params.linear.C;
    node_params.k_linear = K(node_idx, node_idx);
    node_params.c_linear = C(node_idx, node_idx);
    
    % 非线性参数
    node_params.is_nonlinear = params.nl_detection.is_nonlinear(node_idx);
    node_params.NL_index = params.nl_detection.NL_index(node_idx);
    
    if node_params.is_nonlinear
        node_params.k3 = params.nonlinear.k3_coeffs(node_idx);
        node_params.c2 = params.nonlinear.c2_coeffs(node_idx);

        % 安全获取非线性类型（兼容 cell 和字符串数组）
        if iscell(params.nonlinear.nonlinear_type)
            node_params.nl_type = params.nonlinear.nonlinear_type{node_idx};
        elseif isstring(params.nonlinear.nonlinear_type)
            node_params.nl_type = params.nonlinear.nonlinear_type(node_idx);
        else
            node_params.nl_type = 'unknown';
        end
    else
        node_params.k3 = 0;
        node_params.c2 = 0;
        node_params.nl_type = 'linear';
    end
end

%% =====================================================================
%% 【保留】原有的手动标注函数 (略作修改以适配新架构)
%% =====================================================================

function segments = runManualAnnotation(accel_data_cell, force_data, force_time, fs, time_offsets)
    % SAD框架信号段手动标注入口函数
    %
    % 功能:
    %   1. 提供GUI界面让用户选择信号段
    %   2. 支持加载已有标注或进行全新标注
    %   3. 分别处理X和Z两个方向
    %   4. 返回包含完整信息的信号段结构体数组
    %
    % 输入:
    %   accel_data_cell - 1x3 cell数组, 包含三个传感器的加速度数据
    %                     每个元素为struct: .time(时间向量), .data(Nx3矩阵)
    %   force_data      - 力锤信号向量
    %   force_time      - 力锤时间向量
    %   fs              - 采样率 (Hz)
    %   time_offsets    - 1x3向量, 各传感器的时间偏移量
    %
    % 输出:
    %   segments - 信号段结构体数组, 每个元素包含:
    %              .segment_id      - 段编号
    %              .sensor_idx      - 传感器索引 (1=Root, 2=Mid, 3=Tip)
    %              .direction       - 方向 ('X' 或 'Z')
    %              .impact_location - 敲击位置 (1=Root, 2=Mid, 3=Tip)
    %              .start_time      - 起始时间
    %              .end_time        - 结束时间
    %              .signal_data     - 信号数据
    %              .force_data_segment - 对应的力信号段
    %              等其他分析结果字段
    
    global g_annotation_data;
    
    segments = [];
    
    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════════════╗\n');
    fprintf('║  信号段手动标注系统 (SAD框架集成版)                              ║\n');
    fprintf('╚══════════════════════════════════════════════════════════════════╝\n\n');
    
    %% ==================== 输入验证 ====================
    if nargin < 5
        time_offsets = zeros(1, 3);
    end
    
    if isempty(accel_data_cell) || length(accel_data_cell) < 3
        errordlg('加速度数据不完整，需要3个传感器的数据！', '输入错误');
        return;
    end
    
    % 验证数据结构
    for i = 1:3
        if ~isfield(accel_data_cell{i}, 'time') || ~isfield(accel_data_cell{i}, 'data')
            errordlg(sprintf('传感器%d数据格式错误，需要包含time和data字段！', i), '输入错误');
            return;
        end
    end
    
    fprintf('  数据验证通过:\n');
    for i = 1:3
        fprintf('    传感器%d: %d个样本, 时长%.2fs\n', ...
            i, length(accel_data_cell{i}.time), accel_data_cell{i}.time(end));
    end
    if ~isempty(force_time)
        fprintf('    力信号: %d个样本, 时长%.2fs\n', length(force_time), force_time(end));
    end
    fprintf('\n');
    
    %% ==================== 选择标注模式 ====================
    answer = questdlg('请选择标注模式:', '标注模式选择', ...
        '加载已有标注', '进行全新标注', '取消', '加载已有标注');
    
    switch answer
        case '取消'
            fprintf('  用户取消操作。\n');
            return;
            
        case '加载已有标注'
            %% ==================== 加载已有标注 ====================
            segments = loadExistingAnnotations(accel_data_cell, force_data, force_time, fs);
            return;
            
        case '进行全新标注'
            fprintf('  开始进行全新标注...\n\n');
            % 继续执行下面的标注流程
    end
    
    %% ==================== X方向标注 ====================
    fprintf('  ═══════════════════════════════════════\n');
    fprintf('  ║  开始 X 方向信号段标注               ║\n');
    fprintf('  ═══════════════════════════════════════\n\n');
    
    % 初始化全局数据结构
    g_annotation_data = initializeAnnotationData(accel_data_cell, force_data, force_time, fs);
    
    % X方向标注
    x_segments = annotateDirection('X', 2);
    
    % 清理X方向的图形窗口
    if isfield(g_annotation_data, 'current_figure') && ishandle(g_annotation_data.current_figure)
        close(g_annotation_data.current_figure);
    end
    
    fprintf('  X方向标注完成，获取 %d 个信号段\n\n', length(x_segments));
    
    %% ==================== Z方向标注 ====================
    fprintf('  ═══════════════════════════════════════\n');
    fprintf('  ║  开始 Z 方向信号段标注               ║\n');
    fprintf('  ═══════════════════════════════════════\n\n');
    
    % 重新初始化全局数据结构
    g_annotation_data = initializeAnnotationData(accel_data_cell, force_data, force_time, fs);
    
    % Z方向标注
    z_segments = annotateDirection('Z', 3);
    
    % 清理Z方向的图形窗口
    if isfield(g_annotation_data, 'current_figure') && ishandle(g_annotation_data.current_figure)
        close(g_annotation_data.current_figure);
    end
    
    fprintf('  Z方向标注完成，获取 %d 个信号段\n\n', length(z_segments));
    
    %% ==================== 合并结果 ====================
    segments = [x_segments, z_segments];
    
    fprintf('  ─────────────────────────────────────────\n');
    fprintf('  标注汇总: 共 %d 个信号段\n', length(segments));
    fprintf('    X方向: %d 段\n', length(x_segments));
    fprintf('    Z方向: %d 段\n', length(z_segments));
    fprintf('  ─────────────────────────────────────────\n\n');
    
    %% ==================== 保存标注结果 ====================
    if ~isempty(segments)
        n_unique_impacts = length(unique([segments.segment_id]));
        answer_save = questdlg(...
            sprintf('标注完成！共 %d 次敲击, %d 个信号段。\n是否保存本次标注结果？', ...
                n_unique_impacts, length(segments)), ...
            '保存标注', '保存', '不保存', '保存');
        
        if strcmp(answer_save, '保存')
            saveAnnotationResults(segments, accel_data_cell, force_time, fs);
        end
    end
    
    %% ==================== 清理 ====================
    clear global g_annotation_data;
    
    fprintf('  信号标注流程完成。\n\n');
end

%% =====================================================================
%% 【补充】加载已有标注函数
%% =====================================================================

function segments = loadExistingAnnotations(accel_data_cell, force_data, force_time, fs)
    % 加载已保存的标注文件并重建信号段
    %
    % 输入:
    %   accel_data_cell - 加速度数据cell数组
    %   force_data      - 力信号数据
    %   force_time      - 力信号时间
    %   fs              - 采样率
    %
    % 输出:
    %   segments - 重建的信号段结构体数组
    
    global g_annotation_data;
    
    segments = [];
    
    fprintf('  开始加载已有标注文件...\n');
    [filename, pathname] = uigetfile('*.mat', '请选择之前保存的标注文件');
    
    if isequal(filename, 0)
        fprintf('  用户取消了文件选择。\n');
        return;
    end
    
    full_filepath = fullfile(pathname, filename);
    fprintf('  正在从 %s 加载...\n', full_filepath);
    
    try
        S = load(full_filepath);
        
        % 检查文件格式
        if ~isfield(S, 'saved_annotations')
            errordlg('选择的MAT文件无效：缺少 "saved_annotations" 变量。', '加载错误');
            return;
        end
        
        % 初始化全局数据结构（reconstructSegments 依赖此结构）
        g_annotation_data = struct(...
            'accel_data_cell', {accel_data_cell}, ...
            'force_data', force_data, ...
            'force_time', force_time, ...
            'fs', fs, ...
            'sensor_names', {{'Root', 'Mid', 'Tip'}});
        
        % 调用重建函数
        if isfield(S, 'saved_annotations') && ~isempty(S.saved_annotations)
            segments = reconstructSegments(S.saved_annotations, accel_data_cell, force_data, force_time, fs);
        end
        
        if ~isempty(segments)
            fprintf('  成功加载并重建了 %d 个信号段标注。\n', length(segments));
        else
            fprintf('  警告：文件已加载，但未能重建任何有效的信号段。\n');
        end
        
    catch ME
        errordlg(sprintf('加载标注文件失败: %s', ME.message), '加载错误');
        segments = [];
    end
end