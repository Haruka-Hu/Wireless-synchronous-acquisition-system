% =========================================================================
% IMU与肌电(EMG)多通道潜伏期自动化分析脚本 (V9 独立窗口展示版)
% 核心逻辑：
% 1. 峰值回溯与角度积分计算潜伏期。
% 2. 【展示更新】为每一次敲击单独生成一个Figure窗口，聚焦局部特征。
% =========================================================================

clear; clc; close all;

% --- 用户配置区 ---
filename = 'data/imu_capture_20260511_201534.csv';
gyro_axis = 'x';                              
notch_freq = 50;                              
min_tap_interval = 1.0; % 两次叩击最小间隔(秒)
% --------------------

fprintf('正在读取并清洗数据...\n');
data = readtable(filename);

% 1. 数据分离与严格清洗
idx_acc = strcmp(data.source, 'Accel');
idx_gyro = strcmp(data.source, 'Gyro');
idx_emg = strcmp(data.source, 'EMG');

[~, u_idx_acc]  = unique(data.timestamp_us(idx_acc));
[~, u_idx_gyro] = unique(data.timestamp_us(idx_gyro));
[~, u_idx_emg]  = unique(data.timestamp_us(idx_emg));

acc_data  = data(find(idx_acc), :);  acc_data  = acc_data(u_idx_acc, :);
gyro_data = data(find(idx_gyro), :); gyro_data = gyro_data(u_idx_gyro, :);
emg_data  = data(find(idx_emg), :);  emg_data  = emg_data(u_idx_emg, :);

% 2. 统一时间节点
min_time_us = min([acc_data.timestamp_us; gyro_data.timestamp_us; emg_data.timestamp_us]);
t_acc = (acc_data.timestamp_us - min_time_us) / 1e6;
t_gyro = (gyro_data.timestamp_us - min_time_us) / 1e6;
t_emg = (emg_data.timestamp_us - min_time_us) / 1e6;

Fs_acc = 1 / mean(diff(t_acc)); Fs_gyro = 1 / mean(diff(t_gyro)); Fs_emg = 1 / mean(diff(t_emg));

% 3. 零相移滤波处理
Q = 35; w0 = notch_freq / (Fs_emg / 2); bw = w0 / Q;                     
[b_notch, a_notch] = iirnotch(w0, bw);
high_cutoff = min(400, (Fs_emg/2) * 0.95); 
[b_emg, a_emg] = butter(4, [20, high_cutoff] / (Fs_emg / 2), 'bandpass');

emg_x_filt = filtfilt(b_emg, a_emg, filtfilt(b_notch, a_notch, emg_data.x));
emg_y_filt = filtfilt(b_emg, a_emg, filtfilt(b_notch, a_notch, emg_data.y));
emg_z_filt = filtfilt(b_emg, a_emg, filtfilt(b_notch, a_notch, emg_data.z));

[b_acc, a_acc] = butter(4, 20 / (Fs_acc / 2), 'low');
acc_norm_filt = filtfilt(b_acc, a_acc, acc_data.vector_norm);

switch lower(gyro_axis)
    case 'x', gyro_sig_raw = gyro_data.x; case 'y', gyro_sig_raw = gyro_data.y; case 'z', gyro_sig_raw = gyro_data.z;
end
[b_gyro, a_gyro] = butter(4, 20 / (Fs_gyro / 2), 'low');
gyro_sig_filt = filtfilt(b_gyro, a_gyro, gyro_sig_raw);

% 4. 信号降维与处理
emg_combined = sqrt(emg_x_filt.^2 + emg_y_filt.^2 + emg_z_filt.^2);
emg_env = movmean(emg_combined, round(Fs_emg * 0.01)); 

dt_gyro = 1 / Fs_gyro;
gyro_angle_raw = cumtrapz(t_gyro, gyro_sig_filt); 
[b_hp, a_hp] = butter(2, 0.5 / (Fs_gyro / 2), 'high');
gyro_angle = filtfilt(b_hp, a_hp, gyro_angle_raw);
angle_env = movmean(abs(gyro_angle), round(Fs_gyro * 0.02));

% =========================================================================
% 5. 峰值回溯法寻找叩击时刻 (T_tap)
% =========================================================================
baseline_acc = mean(acc_norm_filt(1:round(Fs_acc*1.0)));
peak_thresh = baseline_acc + 6 * std(acc_norm_filt(1:round(Fs_acc*1.0)));
[pks, locs] = findpeaks(acc_norm_filt, 'MinPeakHeight', peak_thresh, 'MinPeakDistance', round(min_tap_interval * Fs_acc));

tap_times = [];
tap_peaks = [];

for i = 1:length(locs)
    peak_idx = locs(i);
    tap_peaks(end+1) = t_acc(peak_idx);
    
    base_start = max(1, peak_idx - round(Fs_acc * 0.2));
    base_end = max(1, peak_idx - round(Fs_acc * 0.1));
    local_base = acc_norm_filt(base_start:base_end);
    local_thresh = mean(local_base) + 4 * std(local_base);
    
    search_start = max(1, peak_idx - round(Fs_acc * 0.1));
    search_window = acc_norm_filt(search_start : peak_idx);
    
    idx_below = find(search_window < local_thresh, 1, 'last');
    if isempty(idx_below)
        tap_idx = peak_idx; 
    else
        tap_idx = search_start + idx_below; 
    end
    tap_times(end+1) = t_acc(tap_idx);
end

% =========================================================================
% 6. 计算潜伏期与逐个弹窗绘图
% =========================================================================
fprintf('\n================ 腱反射独立视图分析报告 ================\n');
fprintf('共检测到 %d 次叩击。\n\n', length(tap_times));

results_emg_onset = NaN(1, length(tap_times)); 
results_angle_onset = NaN(1, length(tap_times));

for i = 1:length(tap_times)
    t_tap = tap_times(i);
    fprintf('【第 %d 次】 物理接触时刻: %.4f 秒\n', i, t_tap);
    
    % --- 肌电潜伏期 ---
    base_idx_emg = t_emg > (t_tap - 0.5) & t_emg < (t_tap - 0.05);
    if ~any(base_idx_emg), base_idx_emg = 1:round(Fs_emg*0.1); end
    thresh_emg = mean(emg_env(base_idx_emg)) + 5 * std(emg_env(base_idx_emg)); 
    
    search_idx_emg = find(t_emg >= (t_tap + 0.005) & t_emg <= (t_tap + 0.15));
    exceeds_emg = emg_env(search_idx_emg) > thresh_emg;
    edges_emg = diff([0; exceeds_emg; 0]);
    starts_emg = find(edges_emg == 1); ends_emg = find(edges_emg == -1);
    
    valid_emg = find((ends_emg - starts_emg) >= round(Fs_emg * 0.01), 1, 'first');
    if ~isempty(valid_emg)
        results_emg_onset(i) = t_emg(search_idx_emg(starts_emg(valid_emg)));
        fprintf('  ▶ 肌电潜伏期: \t %.2f ms\n', (results_emg_onset(i) - t_tap) * 1000);
    else
        fprintf('  ▶ 肌电潜伏期: \t 未检测到有效爆发\n');
    end
    
    % --- 摆动角度潜伏期 ---
    base_idx_angle = t_gyro > (t_tap - 0.5) & t_gyro < (t_tap - 0.05);
    if ~any(base_idx_angle), base_idx_angle = 1:round(Fs_gyro*0.1); end
    thresh_angle = mean(angle_env(base_idx_angle)) + 5 * std(angle_env(base_idx_angle));
    
    search_idx_angle = find(t_gyro >= (t_tap + 0.015) & t_gyro <= (t_tap + 0.40)); 
    exceeds_angle = angle_env(search_idx_angle) > thresh_angle;
    edges_angle = diff([0; exceeds_angle; 0]);
    starts_angle = find(edges_angle == 1); ends_angle = find(edges_angle == -1);
    
    valid_angle = find((ends_angle - starts_angle) >= round(Fs_gyro * 0.025), 1, 'first');
    if ~isempty(valid_angle)
        results_angle_onset(i) = t_gyro(search_idx_angle(starts_angle(valid_angle)));
        fprintf('  ▶ 摆动潜伏期: \t %.2f ms\n', (results_angle_onset(i) - t_tap) * 1000);
    else
        fprintf('  ▶ 摆动潜伏期: \t 未检测到有效角度变化\n');
    end
    fprintf('---------------------------------------------------\n');
    
    % =====================================================================
    % 为当前这次叩击生成专属的独立视图
    % =====================================================================
    fig_name = sprintf('第 %d 次叩击微观视图 (时间点: %.2f秒)', i, t_tap);
    figure('Name', fig_name, 'Position', [50 + i*30, 50 + i*30, 1000, 800]);
    
    plot_vars = {emg_x_filt, emg_y_filt, emg_z_filt}; 
    titles = {sprintf('第%d次: 肌电 EMG X轴', i), '肌电 EMG Y轴', '肌电 EMG Z轴'};
    axs = gobjects(5, 1);

    % 画肌电三通道
    for j = 1:3
        axs(j) = subplot(5, 1, j); 
        plot(t_emg, plot_vars{j}, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5); hold on;
        if j == 1, plot(t_emg, emg_env, 'r', 'LineWidth', 1.5, 'DisplayName','总包络线'); legend('Location', 'northeast'); end
        title(titles{j}, 'FontSize', 10, 'FontWeight', 'bold'); ylabel('Amplitude'); grid on;
        
        % 只画当前敲击的辅助线
        xline(t_tap, 'r--', 'LineWidth', 1.5, 'Label', 'Tap');
        if ~isnan(results_emg_onset(i))
            xline(results_emg_onset(i), 'g-', 'LineWidth', 1.5, 'Label', 'EMG Onset'); 
        end
    end

    % 画加速度
    axs(4) = subplot(5, 1, 4); 
    plot(t_acc, acc_norm_filt, 'k', 'LineWidth', 1.0); hold on; 
    plot(tap_peaks(i), acc_norm_filt(t_acc == tap_peaks(i)), 'bo', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName','波峰');
    xline(t_tap, 'r--', 'LineWidth', 1.5, 'Label', '起跳点');
    title(sprintf('第%d次: 加速度模长 (峰值回溯)', i), 'FontSize', 10, 'FontWeight', 'bold'); 
    ylabel('Norm'); grid on; legend('Location', 'northeast');

    % 画关节角度
    axs(5) = subplot(5, 1, 5); 
    plot(t_gyro, gyro_angle, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5); hold on; 
    xline(t_tap, 'r--', 'LineWidth', 1.5);
    if ~isnan(results_angle_onset(i))
        xline(results_angle_onset(i), 'b-', 'LineWidth', 1.5, 'Label', 'Angle Onset'); 
    end
    title(sprintf('第%d次: 关节运动角度 (由 Gyro %s 轴积分)', i, upper(gyro_axis)), 'FontSize', 10, 'FontWeight', 'bold'); 
    xlabel('时间 (秒)', 'FontSize', 11); ylabel('Angle (Deg)'); grid on;

    % 关联子图X轴，并锁定观测窗口 (敲击前0.5秒 ~ 敲击后0.8秒)
    linkaxes(axs, 'x'); 
    xlim([t_tap - 0.5, t_tap + 0.8]);
end