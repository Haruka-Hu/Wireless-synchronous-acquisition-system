%% Patellar Reflex Latency Analysis
%  Analyze tap (hammer acceleration), muscle response (EMG), and foot motion (gyro)
%  to compute reflex latency: knock → EMG response and knock → mechanical response
%
%  Data format: CSV with columns
%    host_rx_time, source, source_id, timestamp_us, sample_seq, batch_seq, rx_flags, x, y, z, vector_norm
%  where source is one of: "Accel" (hammer), "EMG" (muscle), "Gyro" (shank)

clear; clc; close all;

%% Config
scriptDir = fileparts(mfilename("fullpath"));
projectRoot = fileparts(scriptDir);

params.dataDir = fullfile(projectRoot, "data");
params.filePattern = "imu_capture_*.csv";

% Knock detection (from hammer acceleration)
params.accBaselineWindowSec = 0.25;
params.knockThresholdMad = 8.0;        % Median absolute deviation threshold
params.knockMinPeakDistanceSec = 0.03; % Minimum gap between peaks
params.knockPeakGroupGapSec = 1.00;    % Group peaks into single knock event
params.knockPeaksPerEvent = 3;         % Typical knock has 3 peaks

% EMG response detection
params.emgBaselineWindowSec = 0.50;
params.emgThresholdMad = 5.0;          % EMG magnitude threshold
params.emgMinActivationLengthSec = 0.020; % Activation must persist this long
params.emgIgnoreAfterKnockSec = 0.005;    % Ignore initial 5ms (sensor artifact)
params.emgSearchWindowSec = 0.500;     % Search up to 500ms after knock

% Gyro (mechanical) response detection
params.gyroPreWindowSec = 0.30;
params.gyroIgnoreAfterKnockSec = 0.015;
params.gyroSearchWindowSec = 1.00;
params.gyroThresholdMad = 6.0;
params.gyroMinAbsThreshold = 40;
params.gyroHoldSec = 0.020;

% Output
params.saveSummaryTable = true;
params.outputDir = fullfile(params.dataDir, "analysis_reflex");
params.makePlots = true;

if ~exist(params.outputDir, 'dir')
    mkdir(params.outputDir);
end

%% Find and process all CSV files
files = dir(fullfile(params.dataDir, params.filePattern));
if isempty(files)
    error("No CSV files matching '%s' found in %s", params.filePattern, params.dataDir);
end

allResults = table();

for fileIdx = 1:numel(files)
    csvPath = fullfile(files(fileIdx).folder, files(fileIdx).name);
    fprintf("\n========== Processing: %s ==========\n", files(fileIdx).name);
    
    % Load CSV
    T = readtable(csvPath);
    requiredVars = ["source", "timestamp_us", "x", "y", "z"];
    missingVars = setdiff(requiredVars, string(T.Properties.VariableNames));
    if ~isempty(missingVars)
        warning("Skip %s: missing columns: %s\n", files(fileIdx).name, strjoin(missingVars, ", "));
        continue;
    end
    
    source = string(T.source);
    isAccel = strcmpi(source, "Accel");
    isEMG = strcmpi(source, "EMG");
    isGyro = strcmpi(source, "Gyro");
    
    % Check for required data
    if ~any(isAccel)
        warning("Skip %s: no Accel data (hammer)\n", files(fileIdx).name);
        continue;
    end
    if ~any(isEMG)
        warning("Skip %s: no EMG data (muscle)\n", files(fileIdx).name);
        continue;
    end
    
    % Normalize time
    t0Us = min(double(T.timestamp_us));
    
    % Extract sensor data
    acc = sortrows(T(isAccel, :), "timestamp_us");
    emg = sortrows(T(isEMG, :), "timestamp_us");
    if any(isGyro)
        gyro = sortrows(T(isGyro, :), "timestamp_us");
    else
        gyro = table();
    end
    
    tAcc = (double(acc.timestamp_us) - t0Us) / 1e6;
    ax = double(acc.x);
    ay = double(acc.y);
    az = double(acc.z);
    accMag = sqrt(ax.^2 + ay.^2 + az.^2);
    
    tEMG = (double(emg.timestamp_us) - t0Us) / 1e6;
    emgX = double(emg.x);
    emgY = double(emg.y);
    emgZ = double(emg.z);
    emgMag = sqrt(emgX.^2 + emgY.^2 + emgZ.^2);
    
    if ~isempty(gyro)
        tGyro = (double(gyro.timestamp_us) - t0Us) / 1e6;
        gz = double(gyro.z);
    else
        tGyro = [];
        gz = [];
    end
    
    % Detect knocks from acceleration
    [knockTimes, knockAmps, accDynamic, accThreshold] = detectKnocks(tAcc, accMag, params);
    nKnocks = numel(knockTimes);
    
    if nKnocks == 0
        fprintf("  ⚠ No knocks detected.\n");
        continue;
    end
    
    fprintf("  ✓ Found %d knock(s)\n", nKnocks);
    
    % For each knock, detect EMG and Gyro responses
    emgLatenciesMs = nan(nKnocks, 1);
    emgAmplitudes = nan(nKnocks, 1);
    gyroLatenciesMs = nan(nKnocks, 1);
    gyroAmplitudes = nan(nKnocks, 1);
    
    for k = 1:nKnocks
        % Detect EMG response
        [emgTime, emgAmp] = detectEMGResponse(tEMG, emgMag, knockTimes(k), params);
        if isfinite(emgTime)
            emgLatenciesMs(k) = (emgTime - knockTimes(k)) * 1000;
            emgAmplitudes(k) = emgAmp;
        end
        
        % Detect Gyro response (mechanical)
        if ~isempty(tGyro)
            [gyroTime, gyroAmp] = detectGyroResponse(tGyro, gz, knockTimes(k), params);
            if isfinite(gyroTime)
                gyroLatenciesMs(k) = (gyroTime - knockTimes(k)) * 1000;
                gyroAmplitudes(k) = gyroAmp;
            end
        end
    end
    
    % Build results table
    nValidEMG = sum(isfinite(emgLatenciesMs));
    nValidGyro = sum(isfinite(gyroLatenciesMs));
    
    fprintf("  ✓ EMG responses detected: %d/%d\n", nValidEMG, nKnocks);
    if ~isempty(tGyro)
        fprintf("  ✓ Gyro responses detected: %d/%d\n", nValidGyro, nKnocks);
    end
    
    % Summary statistics
    if nValidEMG > 0
        fprintf("  → EMG latency: %.1f ± %.1f ms (range: %.1f–%.1f ms)\n", ...
            mean(emgLatenciesMs, 'omitnan'), std(emgLatenciesMs, 'omitnan'), ...
            min(emgLatenciesMs, [], 'omitnan'), max(emgLatenciesMs, [], 'omitnan'));
    end
    if nValidGyro > 0
        fprintf("  → Gyro latency: %.1f ± %.1f ms (range: %.1f–%.1f ms)\n", ...
            mean(gyroLatenciesMs, 'omitnan'), std(gyroLatenciesMs, 'omitnan'), ...
            min(gyroLatenciesMs, [], 'omitnan'), max(gyroLatenciesMs, [], 'omitnan'));
    end
    
    % Append to results table
    fileResults = table( ...
        repmat(string(files(fileIdx).name), nKnocks, 1), ...
        (1:nKnocks)', ...
        knockTimes(:), ...
        knockAmps(:), ...
        emgLatenciesMs(:), ...
        emgAmplitudes(:), ...
        gyroLatenciesMs(:), ...
        gyroAmplitudes(:), ...
        'VariableNames', {'Filename', 'KnockIndex', 'KnockTimeS', 'KnockAmplitude', ...
                          'EMGLatencyMs', 'EMGAmplitude', 'GyroLatencyMs', 'GyroAmplitude'});
    
    allResults = [allResults; fileResults];
    
    % Create plots
    if params.makePlots
        plotReflexResponse(tAcc, accMag, accDynamic, accThreshold, knockTimes, ...
                          tEMG, emgMag, emgLatenciesMs, ...
                          tGyro, gz, gyroLatenciesMs, ...
                          files(fileIdx).name);
        
        pngPath = fullfile(params.outputDir, [files(fileIdx).name(1:end-4), '_reflex.png']);
        saveas(gcf, pngPath);
        fprintf("  ✓ Saved plot to: %s\n", pngPath);
        close(gcf);
    end
end

%% Save results summary
if params.saveSummaryTable && ~isempty(allResults)
    csvPath = fullfile(params.outputDir, "reflex_latency_summary.csv");
    writetable(allResults, csvPath);
    fprintf("\n✓ Summary saved to: %s\n", csvPath);
    
    % Print overall statistics
    fprintf("\n========== OVERALL SUMMARY ==========\n");
    validEMG = allResults.EMGLatencyMs(isfinite(allResults.EMGLatencyMs));
    if ~isempty(validEMG)
        fprintf("EMG Latency (all knocks, n=%d):\n", numel(validEMG));
        fprintf("  Mean: %.1f ms\n", mean(validEMG));
        fprintf("  Std:  %.1f ms\n", std(validEMG));
        fprintf("  Min:  %.1f ms\n", min(validEMG));
        fprintf("  Max:  %.1f ms\n", max(validEMG));
    end
    
    validGyro = allResults.GyroLatencyMs(isfinite(allResults.GyroLatencyMs));
    if ~isempty(validGyro)
        fprintf("\nMechanical Latency (all knocks, n=%d):\n", numel(validGyro));
        fprintf("  Mean: %.1f ms\n", mean(validGyro));
        fprintf("  Std:  %.1f ms\n", std(validGyro));
        fprintf("  Min:  %.1f ms\n", min(validGyro));
        fprintf("  Max:  %.1f ms\n", max(validGyro));
    end
end

%% Helper functions

function [knockTimes, knockAmps, accDynamic, accThreshold] = detectKnocks(tAcc, accMag, params)
    % Detect knock impulses from acceleration magnitude
    % Strategy: smooth baseline, threshold relative to noise (MAD), then find peaks
    
    % Estimate baseline with wide moving median
    nBase = round(params.accBaselineWindowSec * (length(tAcc) / (tAcc(end) - tAcc(1))));
    nBase = max(nBase, 1);
    try
        accBaseline = movmedian(accMag, nBase);
    catch
        accBaseline = repmat(median(accMag), size(accMag));
    end
    
    % Dynamic component
    accDynamic = accMag - accBaseline;
    accDynamic = max(accDynamic, 0); % Only positive deviations
    
    % Noise estimation (MAD of low-amplitude samples)
    lowAmp = accDynamic <= prctile(accDynamic, 50);
    medianDev = median(abs(accDynamic(lowAmp) - median(accDynamic(lowAmp))));
    accThreshold = median(accDynamic(lowAmp)) + params.knockThresholdMad * medianDev;
    
    % Find peaks above threshold
    minDist = round(params.knockMinPeakDistanceSec * (length(tAcc) / (tAcc(end) - tAcc(1))));
    minDist = max(minDist, 1);
    
    [peakAmps, peakIdx] = findpeaks(accDynamic, 'MinPeakHeight', accThreshold, 'MinPeakDistance', minDist);
    
    if isempty(peakIdx)
        knockTimes = [];
        knockAmps = [];
        return;
    end
    
    % Group nearby peaks into knock events
    peakTimes = tAcc(peakIdx);
    knockGroupDist = params.knockPeakGroupGapSec;
    
    [groups, ~] = splitapply(@(x) {x}, (1:numel(peakTimes))', ...
        discretize(peakTimes, [0; (peakTimes(2:end) + peakTimes(1:end-1))/2; inf]));
    
    knockTimes = [];
    knockAmps = [];
    
    for g = 1:numel(groups)
        groupIdx = groups{g};
        groupPeaks = peakAmps(groupIdx);
        groupTimes = peakTimes(groupIdx);
        
        % Use the last peak of each group (strongest/clearest)
        [~, lastIdx] = max(groupPeaks);
        knockTimes = [knockTimes; groupTimes(lastIdx)];
        knockAmps = [knockAmps; groupPeaks(lastIdx)];
    end
    
    % Remove duplicates and sort
    [knockTimes, order] = sort(knockTimes);
    knockAmps = knockAmps(order);
    
    % Remove knock events that are too close (< knockPeakGroupGapSec)
    keep = [true; diff(knockTimes) > knockGroupDist];
    knockTimes = knockTimes(keep);
    knockAmps = knockAmps(keep);
end

function [emgTime, emgAmp] = detectEMGResponse(tEMG, emgMag, knockTime, params)
    % Detect EMG response onset after knock
    % Returns time of first significant EMG activation
    
    emgTime = nan;
    emgAmp = nan;
    
    % Search window
    tStart = knockTime + params.emgIgnoreAfterKnockSec;
    tEnd = knockTime + params.emgSearchWindowSec;
    
    searchIdx = (tEMG >= tStart) & (tEMG <= tEnd);
    if ~any(searchIdx)
        return;
    end
    
    tSub = tEMG(searchIdx);
    emgSub = emgMag(searchIdx);
    
    % Baseline and threshold
    nBase = round(params.emgBaselineWindowSec * (length(emgSub) / (tSub(end) - tSub(1))));
    nBase = max(nBase, 1);
    try
        emgBaseline = movmedian(emgSub, nBase);
    catch
        emgBaseline = repmat(median(emgSub), size(emgSub));
    end
    
    emgDynamic = emgSub - emgBaseline;
    lowAmp = emgDynamic <= prctile(emgDynamic, 50);
    medianDev = median(abs(emgDynamic(lowAmp) - median(emgDynamic(lowAmp))));
    threshold = median(emgDynamic(lowAmp)) + params.emgThresholdMad * medianDev;
    
    % Find first crossing above threshold
    above = emgDynamic >= threshold;
    
    % Require sustained activation (hold for minActivationLength)
    nHold = round(params.emgMinActivationLengthSec * (length(emgSub) / (tSub(end) - tSub(1))));
    nHold = max(nHold, 1);
    
    convAbove = conv(double(above), ones(nHold, 1), 'same');
    sustained = convAbove >= nHold;
    
    activationIdx = find(sustained, 1, 'first');
    if ~isempty(activationIdx)
        emgTime = tSub(activationIdx);
        emgAmp = mean(emgSub(max(1, activationIdx - nHold):min(numel(emgSub), activationIdx + nHold)));
    end
end

function [gyroTime, gyroAmp] = detectGyroResponse(tGyro, gz, knockTime, params)
    % Detect shank rotation (angular velocity) onset after knock
    
    gyroTime = nan;
    gyroAmp = nan;
    
    % Search window
    tStart = knockTime + params.gyroIgnoreAfterKnockSec;
    tEnd = knockTime + params.gyroSearchWindowSec;
    
    searchIdx = (tGyro >= tStart) & (tGyro <= tEnd);
    if ~any(searchIdx)
        return;
    end
    
    tSub = tGyro(searchIdx);
    gzSub = gz(searchIdx);
    
    % Baseline and threshold
    nBase = round(params.gyroPreWindowSec * (length(gzSub) / (tSub(end) - tSub(1))));
    nBase = max(nBase, 1);
    try
        gyroBaseline = movmedian(gzSub, nBase);
    catch
        gyroBaseline = repmat(median(gzSub), size(gzSub));
    end
    
    gyroDynamic = abs(gzSub - gyroBaseline);
    lowAmp = gyroDynamic <= prctile(gyroDynamic, 50);
    medianDev = median(abs(gyroDynamic(lowAmp) - median(gyroDynamic(lowAmp))));
    threshold = max(median(gyroDynamic(lowAmp)) + params.gyroThresholdMad * medianDev, ...
                     params.gyroMinAbsThreshold);
    
    % Find first sustained crossing
    above = gyroDynamic >= threshold;
    nHold = round(params.gyroHoldSec * (length(gzSub) / (tSub(end) - tSub(1))));
    nHold = max(nHold, 1);
    
    convAbove = conv(double(above), ones(nHold, 1), 'same');
    sustained = convAbove >= nHold;
    
    activationIdx = find(sustained, 1, 'first');
    if ~isempty(activationIdx)
        gyroTime = tSub(activationIdx);
        gyroAmp = mean(abs(gzSub(max(1, activationIdx - nHold):min(numel(gzSub), activationIdx + nHold))));
    end
end

function plotReflexResponse(tAcc, accMag, accDynamic, accThreshold, knockTimes, ...
                            tEMG, emgMag, emgLatencies, ...
                            tGyro, gz, gyroLatencies, ...
                            filename)
    % Create 3-panel plot showing acceleration, EMG, and gyro with detected events
    
    fig = figure('Position', [100, 100, 1200, 800], 'Visible', 'off');
    
    % Panel 1: Acceleration
    ax1 = subplot(3, 1, 1);
    hold on;
    plot(tAcc, accMag, 'b-', 'LineWidth', 1, 'DisplayName', 'Acceleration Magnitude');
    plot(tAcc, accDynamic, 'g-', 'LineWidth', 1, 'DisplayName', 'Dynamic Component');
    yline(accThreshold, 'r--', 'DisplayName', 'Knock Threshold');
    for k = 1:numel(knockTimes)
        xline(knockTimes(k), 'k--', 'Alpha', 0.5);
        text(knockTimes(k), max(accMag)*0.95, sprintf('K%d', k), 'HorizontalAlignment', 'center');
    end
    ylabel('Acceleration (m/s²)');
    legend('Location', 'northeast');
    title(sprintf('Patellar Reflex Analysis - %s', filename));
    grid on;
    
    % Panel 2: EMG
    ax2 = subplot(3, 1, 2);
    hold on;
    plot(tEMG, emgMag, 'b-', 'LineWidth', 1, 'DisplayName', 'EMG Magnitude');
    for k = 1:numel(knockTimes)
        xline(knockTimes(k), 'k--', 'Alpha', 0.5);
        if isfinite(emgLatencies(k))
            responseTime = knockTimes(k) + emgLatencies(k)/1000;
            xline(responseTime, 'g--', 'LineWidth', 2);
            text(responseTime, max(emgMag)*0.95, sprintf('EMG\n%.0f ms', emgLatencies(k)), ...
                'HorizontalAlignment', 'center', 'FontSize', 9);
        end
    end
    ylabel('EMG (magnitude)');
    legend('Location', 'northeast');
    grid on;
    
    % Panel 3: Gyro
    if ~isempty(tGyro)
        ax3 = subplot(3, 1, 3);
        hold on;
        plot(tGyro, gz, 'b-', 'LineWidth', 1, 'DisplayName', 'Gyro Z (angular velocity)');
        for k = 1:numel(knockTimes)
            xline(knockTimes(k), 'k--', 'Alpha', 0.5);
            if isfinite(gyroLatencies(k))
                responseTime = knockTimes(k) + gyroLatencies(k)/1000;
                xline(responseTime, 'r--', 'LineWidth', 2);
                text(responseTime, max(gz)*0.95, sprintf('Gyro\n%.0f ms', gyroLatencies(k)), ...
                    'HorizontalAlignment', 'center', 'FontSize', 9);
            end
        end
        ylabel('Angular Velocity (deg/s)');
        xlabel('Time (s)');
        legend('Location', 'northeast');
        grid on;
    else
        ax3 = subplot(3, 1, 3);
        text(0.5, 0.5, 'No Gyro data available', 'Units', 'normalized', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
        axis off;
    end
    
    linkaxes([ax1, ax2], 'x');
end
