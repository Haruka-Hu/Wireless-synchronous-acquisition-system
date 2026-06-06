% =========================================================================
% 功能：sEMG 表面肌电信号读取、滤波预处理与时域可视化
% 滤波器配置：20-450Hz 4阶巴特沃斯带通滤波 + 50Hz IIR窄带陷波
% 特色：自动计算采样率、均值中心化、双向零相位滤波、自动提取100ms RMS包络
% =========================================================================

clear; clc; close all;

%% 1. 读取数据文件
filename = 'data/imu_capture_20260525_173130.csv';
fprintf('正在读取数据文件: %s ...\n', filename);

% 自动检测文件的导入选项以确保最佳兼容性
opts = detectImportOptions(filename);
data = readtable(filename, opts);

%% 2. 动态计算采样频率 (fs)
% 利用数据中的微秒级时间戳 (timestamp_us) 的差值中位数精确推导实际采样率
timestamps = data.timestamp_us; % 单位：微秒
dt_seconds = median(diff(timestamps)) / 1e6; % 转换为秒
fs = round(1 / dt_seconds);                  % 四舍五入得到标称采样率（此处精确识别为 2000 Hz）

fprintf('数据读取成功！总采样点数: %d\n', height(data));
fprintf('自动识别的系统采样率 (fs): %d Hz\n', fs);

%% 3. 滤波系统设计
% 3.1 设计 20 - 450 Hz 的 4 阶巴特沃斯带通滤波器
f_low = 20;
f_high = 450;
[b_bp, a_bp] = butter(4, [f_low, f_high] / (fs/2), 'bandpass');

% 3.2 设计 50 Hz 工频陷波器 (Notch Filter)
f_notch = 50;
q_factor = 35; % 品质因数 Q = f_notch / 带宽，值越大陷波越精准窄小
wo = f_notch / (fs/2);
bw = wo / q_factor;
[b_notch, a_notch] = iirnotch(wo, bw);

%% 4. 数据提取、去直流项与串联滤波
% 提取 x, y, z 三个通道的原始数据
raw_signals = [data.x, data.y, data.z];
channel_names = {'通道 X', '通道 Y', '通道 Z'};
num_channels = size(raw_signals, 2);

% 初始化矩阵，用于存储滤波后的信号和滑窗 RMS 包络线
filtered_signals = zeros(size(raw_signals));
rms_envelopes = zeros(size(raw_signals));

% 设定肌肉激活包络线的移动 RMS 窗口大小（推荐 100 毫秒）
window_ms = 100;
window_size = round(window_ms / 1000 * fs); 

for i = 1:num_channels
    % 步骤 A: 消除基线偏置 / 去除直流分量（Mean Removal）
    sig_detrend = raw_signals(:, i) - mean(raw_signals(:, i));
    
    % 步骤 B: 20-450Hz 带通滤波（使用 filtfilt 实现无相位延迟）
    sig_bp = filtfilt(b_bp, a_bp, sig_detrend);
    
    % 步骤 C: 50Hz 陷波滤波（消除空间环境的电网工频干扰）
    filtered_signals(:, i) = filtfilt(b_notch, a_notch, sig_bp);
    
    % 步骤 D: 计算时域移动 RMS 包络线，更清晰地勾勒肌肉收缩特征
    rms_envelopes(:, i) = sqrt(movmean(filtered_signals(:, i).^2, window_size));
end

%% 5. 跨通道多子图可视化展示
time_seconds = (0:length(timestamps)-1) / fs; % 构建精准的时间轴（单位：秒）

figure('Name', '表面肌电信号 (sEMG) 滤波与包络提取结果', 'Color', 'w', 'Position', [100, 100, 1100, 800]);

for i = 1:num_channels
    subplot(num_channels, 1, i);
    
    % 绘制预处理后纯净的 sEMG 波形
    plot(time_seconds, filtered_signals(:, i), 'Color', [0.12, 0.47, 0.71, 0.7], 'LineWidth', 0.5);
    hold on;
    
    % 绘制红色粗线的 100ms RMS 激活包络线
    plot(time_seconds, rms_envelopes(:, i), 'Color', [0.89, 0.10, 0.11], 'LineWidth', 1.8);
    hold off;
    
    % 轴属性优化与标签美化
    title(sprintf('%s 处理结果 (%d-%d Hz 带通 + %d Hz 陷波)', channel_names{i}, f_low, f_high, f_notch), 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('时间 (秒)', 'FontSize', 10);
    ylabel('幅值 (ADC 原始单位)', 'FontSize', 10);
    grid on;
    xlim([time_seconds(1), time_seconds(end)]);
    
    % 智能图例放置
    legend('滤波后 sEMG 信号', '100ms 移动 RMS 包络', 'Location', 'northeast');
end

% 添加全局大标题
sgtitle('表面肌电信号 (sEMG) 数字信号处理工作流演示', 'FontSize', 14, 'FontWeight', 'bold');