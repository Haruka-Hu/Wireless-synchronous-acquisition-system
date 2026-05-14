% =========================================================================
% IMU与肌电(EMG)多通道潜伏期自动化分析脚本 (V5 连续多次反射版)
% 包含：清洗、零相移滤波、多重叩击识别、抗伪影微调、五图联动展示
% =========================================================================

clear; clc; close all;

% --- 1. 用户配置区 ---
filename = 'data/imu_capture_20260513_144911.csv'; % 数据文件名
gyro_axis = 'z';                              % 展示与计算的角速度轴
notch_freq = 50;                              % 工频干扰频率 
min_tap_interval = 1.5;                       % 两次叩击之间的最短间隔(秒)
% --------------------

fprintf('正在读取并清洗数据...\n');
data = readtable(filename);

% 2. 数据分离与清洗 (排序 + 去重)
idx_acc = strcmp(data.source, 'Accel');
idx_gyro = strcmp(data.source, 'Gyro');
idx_emg = strcmp(data.source, 'EMG');

[~, u_idx_acc]  = unique(data.timestamp_us(idx_acc));
[~, u_idx_gyro] = unique(data.timestamp_us(idx_gyro));
[~, u_idx_emg]  = unique(data.timestamp_us(idx_emg));

acc_data  = data(find(idx_acc), :);  acc_data  = acc_data(u_idx_acc, :);
gyro_data = data(find(idx_gyro), :); gyro_data = gyro_data(u_idx_gyro, :);
emg_data  = data(find(idx_emg), :);  emg_data  = emg_data(u_idx_emg, :);

% 3. 时间轴对齐
min_time_us = min([acc_data.timestamp_us; gyro_data.timestamp_us; emg_data.timestamp_us]);
t_acc = (acc_data.timestamp_us - min_time_us) / 1e6;
t_gyro = (gyro_data.timestamp_us - min_time_us) / 1e6;
t_emg = (emg_data.timestamp_us - min_time_us) / 1e6;

Fs_acc = 1 / mean(diff(t_acc));
Fs_gyro = 1 / mean(diff(t_gyro));
Fs_emg = 1 / mean(diff(t_emg));

% 4. 零相移滤波设计与处理
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
    case 'x', gyro_sig_raw = gyro_data.x;
    case 'y', gyro_sig_raw = gyro_data.y;
    case 'z', gyro_sig_raw = gyro_data.z;
    otherwise, gyro_sig_raw = gyro_data.y;
end
[b_gyro, a_gyro] = butter(4, 20 / (Fs_gyro / 2), 'low');
gyro_sig_filt = filtfilt(b_gyro, a_gyro, gyro_sig_raw);

% =========================================================================
% 5. 多次叩击识别与潜伏期循环计算
% =========================================================================
fprintf('\n================ 腱反射潜伏期分析报告 ================\n');

% 提取包络线 (用于后续查找)
emg_combined = sqrt(emg_x_filt.^2 + emg_y_filt.^2 + emg_z_filt.^2);
emg_env = movmean(emg_combined, round(Fs_emg * 0.02)); 
gyro_env = movmean(abs(gyro_sig_filt), round(Fs_gyro * 0.02));

% 使用突变率(Jerk)寻找所有叩击点
acc_jerk = [0; diff(acc_data.vector_norm)]; 
jerk_thresh = mean(abs(acc_jerk)) + 5 * std(acc_jerk);
min_dist_samples = round(min_tap_interval * Fs_acc);

% 寻找所有满足高度和时间间隔的敲击峰值
[pks, locs] = findpeaks(abs(acc_jerk), 'MinPeakHeight', jerk_thresh, 'MinPeakDistance', min_dist_samples);
tap_times = t_acc(locs);

if isempty(tap_times)
    fprintf('警告：未能自动识别到叩击点。请检查数据或降低 jerk_thresh 阈值。\n');
    return;
end

fprintf('共检测到 %d 次有效叩击。\n\n', length(tap_times));

% 预分配存储结果的数组
results_emg_onset = NaN(1, length(tap_times));
results_gyro_onset = NaN(1, length(tap_times));

for i = 1:length(tap_times)
    t_tap = tap_times(i);
    fprintf('【第 %d 次叩击】 发生时刻: %.4f 秒\n', i, t_tap);
    
    % --- 寻找该次叩击的肌电Onset ---
    % 1. 基线取该次叩击前 0.5秒 到 0.05秒 (避开敲击瞬间的震动)
    base_idx_emg = t_emg > (t_tap - 0.5) & t_emg < (t_tap - 0.05);
    if ~any(base_idx_emg), base_idx_emg = 1:round(Fs_emg*0.1); end
    thresh_emg = mean(emg_env(base_idx_emg)) + 5 * std(emg_env(base_idx_emg)); % 阈值：均值+5倍标准差
    
    % 2. 搜索区间：该次叩击之后 0.15秒内
    search_idx_emg = find(t_emg >= t_tap & t_emg <= (t_tap + 0.15));
    exceeds_emg = emg_env(search_idx_emg) > thresh_emg;
    edges_emg = diff([0; exceeds_emg; 0]);
    starts_emg = find(edges_emg == 1);
    ends_emg = find(edges_emg == -1);
    
    % 3. 防伪影微调：必须连续 15ms 以上超过阈值才算数
    valid_emg = find((ends_emg - starts_emg) >= round(Fs_emg * 0.015), 1, 'first');
    
    if ~isempty(valid_emg)
        results_emg_onset(i) = t_emg(search_idx_emg(starts_emg(valid_emg)));
        fprintf('  ▶ 肌电潜伏期: \t %.2f ms\n', (results_emg_onset(i) - t_tap) * 1000);
    else
        fprintf('  ▶ 肌电潜伏期: \t 未检测到有效爆发\n');
    end
    
    % --- 寻找该次叩击的摆动Onset ---
    base_idx_gyro = t_gyro > (t_tap - 0.5) & t_gyro < (t_tap - 0.05);
    if ~any(base_idx_gyro), base_idx_gyro = 1:round(Fs_gyro*0.1); end
    thresh_gyro = mean(gyro_env(base_idx_gyro)) + 5 * std(gyro_env(base_idx_gyro));
    
    search_idx_gyro = find(t_gyro >= t_tap & t_gyro <= (t_tap + 0.3)); % 动作稍慢，搜索窗设为0.3秒
    exceeds_gyro = gyro_env(search_idx_gyro) > thresh_gyro;
    edges_gyro = diff([0; exceeds_gyro; 0]);
    starts_gyro = find(edges_gyro == 1);
    ends_gyro = find(edges_gyro == -1);
    
    % 摆动动作的低频特性，要求连续 20ms
    valid_gyro = find((ends_gyro - starts_gyro) >= round(Fs_gyro * 0.02), 1, 'first');
    
    if ~isempty(valid_gyro)
        results_gyro_onset(i) = t_gyro(search_idx_gyro(starts_gyro(valid_gyro)));
        fprintf('  ▶ 摆动潜伏期: \t %.2f ms\n', (results_gyro_onset(i) - t_tap) * 1000);
    else
        fprintf('  ▶ 摆动潜伏期: \t 未检测到有效运动\n');
    end
    fprintf('---------------------------------------------------\n');
end

% =========================================================================
% 6. 全局波形图与标注卡尺线
% =========================================================================
figure('Name', '多次腱反射潜伏期自动化分析', 'Position', [50, 50, 1200, 900]);

% 定义绘图通用逻辑，简化代码
plot_vars = {emg_x_filt, emg_y_filt, emg_z_filt};
titles = {'肌电 EMG X轴', '肌电 EMG Y轴', '肌电 EMG Z轴'};
axs = gobjects(5, 1);

for j = 1:3
    axs(j) = subplot(5, 1, j);
    plot(t_emg, plot_vars{j}, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5); hold on;
    if j == 1, plot(t_emg, emg_env, 'r', 'LineWidth', 1.5, 'DisplayName','整体包络'); end
    title(titles{j}, 'FontSize', 10, 'FontWeight', 'bold');
    ylabel('Amplitude'); grid on;
    % 标记所有的 Tap 和 EMG Onset
    for i = 1:length(tap_times)
        xline(tap_times(i), 'r--', 'LineWidth', 1.5);
        if ~isnan(results_emg_onset(i)), xline(results_emg_onset(i), 'g-', 'LineWidth', 1.5); end
    end
end

axs(4) = subplot(5, 1, 4);
plot(t_acc, acc_norm_filt, 'k', 'LineWidth', 1.5); hold on;
title('加速度模长 (红虚线: 叩击标定)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('Norm Value'); grid on;
for i = 1:length(tap_times)
    xline(tap_times(i), 'r--', 'LineWidth', 1.5);
end

axs(5) = subplot(5, 1, 5);
plot(t_gyro, gyro_sig_filt, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5); hold on;
title(sprintf('角速度 Gyro %s 轴 (蓝实线: 摆动启动)', upper(gyro_axis)), 'FontSize', 10, 'FontWeight', 'bold');
xlabel('时间 (秒)', 'FontSize', 11); ylabel('Angular Vel'); grid on;
for i = 1:length(tap_times)
    xline(tap_times(i), 'r--', 'LineWidth', 1.5);
    if ~isnan(results_gyro_onset(i)), xline(results_gyro_onset(i), 'b-', 'LineWidth', 1.5); end
end

linkaxes(axs, 'x');
% 自动调整视图到包含所有叩击点的合适范围
xlim([max(0, tap_times(1)-1), tap_times(end)+1]);