% Analyze IMU CSV files under data/.
%
% The script displays the acceleration resultant waveform, detects knock
% instants from the 3-axis acceleration magnitude, then estimates the latency
% from each knock to the first shank swing onset detected on gyroscope gz.

clear; clc; close all;

scriptDir = fileparts(mfilename("fullpath"));
projectRoot = fileparts(scriptDir);

params.dataDir = fullfile(projectRoot, "data");
params.filePattern = "*.csv";

% Detection parameters. Tune these first if a recording is very noisy or weak.
params.accBaselineWindowSec = 0.25;
params.knockThresholdMad = 8.0;
params.knockMinPeakDistanceSec = 0.03; % Merge tiny peak jitter within one acceleration peak.
params.knockPeakGroupGapSec = 1.00;    % Candidate peaks closer than this belong to one knock burst.
params.knockPeaksPerEvent = 3;         % A knock usually has 3 accel peaks; use the last one.
params.maxKnocksPerFile = inf;

params.gyroPreWindowSec = 0.30;
params.gyroIgnoreAfterKnockSec = 0.015;
params.gyroSearchWindowSec = 1.00;
params.gyroThresholdMad = 6.0;
params.gyroMinAbsThreshold = 40;       % Raw gz units.
params.gyroHoldSec = 0.020;            % Threshold must hold this long.

params.saveSummaryCsv = true;
params.saveFigures = false;
params.outputDir = fullfile(params.dataDir, "analysis");

files = dir(fullfile(params.dataDir, params.filePattern));
if isempty(files)
    error("No CSV files found in %s", params.dataDir);
end

allResults = table();

for fileIdx = 1:numel(files)
    csvPath = fullfile(files(fileIdx).folder, files(fileIdx).name);
    fprintf("\n=== %s ===\n", files(fileIdx).name);

    T = readtable(csvPath);
    requiredVars = ["source", "timestamp_us", "x", "y", "z"];
    missingVars = setdiff(requiredVars, string(T.Properties.VariableNames));
    if ~isempty(missingVars)
        warning("Skip %s: missing columns: %s", files(fileIdx).name, strjoin(missingVars, ", "));
        continue;
    end

    source = string(T.source);
    isAccel = strcmpi(source, "Accel");
    isGyro = strcmpi(source, "Gyro");

    if ~any(isAccel) || ~any(isGyro)
        warning("Skip %s: both Accel and Gyro rows are required.", files(fileIdx).name);
        continue;
    end

    t0Us = min(double(T.timestamp_us(isAccel | isGyro)));

    acc = sortrows(T(isAccel, :), "timestamp_us");
    gyro = sortrows(T(isGyro, :), "timestamp_us");

    tAcc = (double(acc.timestamp_us) - t0Us) / 1e6;
    ax = double(acc.x);
    ay = double(acc.y);
    az = double(acc.z);
    accMag = sqrt(ax.^2 + ay.^2 + az.^2);

    tGyro = (double(gyro.timestamp_us) - t0Us) / 1e6;
    gz = double(gyro.z);

    [knockTimes, knockAmps, accDynamic, accScore, accThreshold] = detectKnocks(tAcc, accMag, params);
    if isfinite(params.maxKnocksPerFile) && numel(knockTimes) > params.maxKnocksPerFile
        knockTimes = knockTimes(1:params.maxKnocksPerFile);
        knockAmps = knockAmps(1:params.maxKnocksPerFile);
    end

    nKnocks = numel(knockTimes);
    swingTimes = nan(nKnocks, 1);
    latenciesMs = nan(nKnocks, 1);
    gyroThresholds = nan(nKnocks, 1);

    for k = 1:nKnocks
        [swingTimes(k), gyroThresholds(k)] = detectSwingOnset(tGyro, gz, knockTimes(k), params);
        if isfinite(swingTimes(k))
            latenciesMs(k) = (swingTimes(k) - knockTimes(k)) * 1000;
        end
    end

    result = table( ...
        repmat(string(files(fileIdx).name), nKnocks, 1), ...
        (1:nKnocks)', ...
        knockTimes(:), ...
        knockAmps(:), ...
        swingTimes(:), ...
        latenciesMs(:), ...
        gyroThresholds(:), ...
        'VariableNames', ["file", "knock_index", "knock_time_s", "acc_peak_raw", ...
                          "swing_onset_time_s", "latency_ms", "gyro_threshold_raw"]);

    if nKnocks == 0
        fprintf("No knock detected. Try lowering params.knockThresholdMad.\n");
    else
        disp(result(:, ["knock_index", "knock_time_s", "swing_onset_time_s", "latency_ms"]));
        allResults = [allResults; result]; %#ok<AGROW>
    end

    plotAnalysis(files(fileIdx).name, tAcc, accMag, accDynamic, accScore, accThreshold, ...
                 tGyro, gz, knockTimes, swingTimes, latenciesMs, params);

    if params.saveFigures
        if ~exist(params.outputDir, "dir")
            mkdir(params.outputDir);
        end
        [~, baseName] = fileparts(files(fileIdx).name);
        saveas(gcf, fullfile(params.outputDir, baseName + "_knock_latency.png"));
    end
end

if params.saveSummaryCsv && ~isempty(allResults)
    if ~exist(params.outputDir, "dir")
        mkdir(params.outputDir);
    end
    outPath = fullfile(params.outputDir, "knock_latency_summary.csv");
    writetable(allResults, outPath);
    fprintf("\nSummary written to %s\n", outPath);
end

function [knockTimes, knockAmps, accDynamic, score, threshold] = detectKnocks(t, accMag, params)
    dt = estimateSampleInterval(t);
    baselineWin = secondsToOddSamples(params.accBaselineWindowSec, dt);
    accBaseline = movmedian(accMag, baselineWin);
    accDynamic = accMag - accBaseline;
    score = abs(accDynamic);

    threshold = medianFinite(score) + params.knockThresholdMad * robustMad(score);
    minPeakDistSamples = max(1, round(params.knockMinPeakDistanceSec / dt));

    candidateIdx = localPeaksAboveThreshold(score, threshold, minPeakDistSamples);
    peakIdx = selectKnockStartPeaks(candidateIdx, t, params.knockPeakGroupGapSec, params.knockPeaksPerEvent);

    knockTimes = t(peakIdx);
    knockAmps = accMag(peakIdx);
end

function [swingTime, threshold] = detectSwingOnset(tGyro, gz, knockTime, params)
    swingTime = nan;

    preMask = tGyro >= knockTime - params.gyroPreWindowSec & tGyro < knockTime - params.gyroIgnoreAfterKnockSec;
    if nnz(preMask) >= 5
        baseline = medianFinite(gz(preMask));
        noise = abs(gz(preMask) - baseline);
        threshold = max(params.gyroMinAbsThreshold, medianFinite(noise) + params.gyroThresholdMad * robustMad(noise));
    else
        baseline = medianFinite(gz);
        noise = abs(gz - baseline);
        threshold = max(params.gyroMinAbsThreshold, medianFinite(noise) + params.gyroThresholdMad * robustMad(noise));
    end

    searchMask = tGyro >= knockTime + params.gyroIgnoreAfterKnockSec & ...
                 tGyro <= knockTime + params.gyroSearchWindowSec;
    searchIdx = find(searchMask);
    if isempty(searchIdx)
        return;
    end

    dt = estimateSampleInterval(tGyro);
    holdSamples = max(1, round(params.gyroHoldSec / dt));
    above = abs(gz(searchIdx) - baseline) >= threshold;

    firstLocal = firstSustainedTrue(above, holdSamples);
    if ~isnan(firstLocal)
        swingTime = tGyro(searchIdx(firstLocal));
    end
end

function plotAnalysis(fileName, tAcc, accMag, accDynamic, accScore, accThreshold, ...
                      tGyro, gz, knockTimes, swingTimes, latenciesMs, params)
    figure("Name", "Knock latency - " + fileName, "Color", "w");
    tiledlayout(3, 1, "TileSpacing", "compact", "Padding", "compact");

    nexttile;
    hAccRaw = plot(tAcc, accMag, "k-", "LineWidth", 0.8); hold on;
    hAccDyn = plot(tAcc, accDynamic + medianFinite(accMag), "Color", [0.20 0.55 0.90], "LineWidth", 0.7);
    legendHandles = [hAccRaw, hAccDyn];
    legendLabels = ["|a| raw", "|a| dynamic + offset"];
    if ~isempty(knockTimes)
        knockAcc = sampleAtTimes(tAcc, accMag, knockTimes);
        hKnock = plot(knockTimes, knockAcc, "rv", "MarkerFaceColor", "r", "MarkerSize", 7);
        legendHandles = [legendHandles, hKnock];
        legendLabels = [legendLabels, "knock"];
    end
    addEventLines(knockTimes, [0.85 0.10 0.10], "--");
    grid on;
    ylabel("Accel norm");
    title("Acceleration resultant: knock detection");
    legend(legendHandles, legendLabels, "Location", "best");

    nexttile;
    hGz = plot(tGyro, gz, "Color", [0.10 0.10 0.10], "LineWidth", 0.8); hold on;
    addEventLines(knockTimes, [0.85 0.10 0.10], "--");
    addEventLines(swingTimes(isfinite(swingTimes)), [0.00 0.45 0.20], "-");
    hKnockLine = plot(nan, nan, "--", "Color", [0.85 0.10 0.10], "LineWidth", 1.1);
    hSwingLine = plot(nan, nan, "-", "Color", [0.00 0.45 0.20], "LineWidth", 1.1);
    grid on;
    ylabel("gz raw");
    title("Gyroscope gz: shank swing onset");
    legend([hGz, hKnockLine, hSwingLine], ["gz", "knock", "swing onset"], "Location", "best");

    nexttile;
    if any(isfinite(latenciesMs))
        stem(find(isfinite(latenciesMs)), latenciesMs(isfinite(latenciesMs)), "filled", ...
             "Color", [0.00 0.35 0.75], "LineWidth", 1.2);
    else
        text(0.5, 0.5, "No valid latency found", "HorizontalAlignment", "center");
    end
    grid on;
    xlabel("Knock index");
    ylabel("Latency (ms)");
    title(sprintf("Latency = swing onset(gz) - knock(acc), gyro threshold >= %.0f raw units", ...
          max(params.gyroMinAbsThreshold, 0)));

    % Show the acceleration score threshold without adding another subplot.
    fprintf("Acceleration score threshold: %.3f raw units\n", accThreshold);
    if ~isempty(accScore)
        fprintf("Acceleration score max: %.3f raw units\n", max(accScore));
    end
end

function peakIdx = localPeaksAboveThreshold(y, threshold, minDistSamples)
    y = y(:);
    if numel(y) < 3
        peakIdx = [];
        return;
    end

    candidateIdx = find(y(2:end-1) >= y(1:end-2) & y(2:end-1) > y(3:end) & y(2:end-1) >= threshold) + 1;
    if isempty(candidateIdx)
        peakIdx = [];
        return;
    end

    [~, order] = sort(y(candidateIdx), "descend");
    candidateIdx = candidateIdx(order);
    accepted = false(size(candidateIdx));

    for i = 1:numel(candidateIdx)
        idx = candidateIdx(i);
        if ~any(abs(idx - candidateIdx(accepted)) < minDistSamples)
            accepted(i) = true;
        end
    end

    peakIdx = sort(candidateIdx(accepted));
end

function selectedIdx = selectKnockStartPeaks(candidateIdx, t, groupGapSec, peaksPerEvent)
    candidateIdx = sort(candidateIdx(:));
    if isempty(candidateIdx)
        selectedIdx = [];
        return;
    end

    candidateTimes = t(candidateIdx);
    selectedIdx = [];
    groupStart = 1;

    for i = 2:numel(candidateIdx)
        if candidateTimes(i) - candidateTimes(i - 1) > groupGapSec
            groupIdx = candidateIdx(groupStart:i - 1);
            pickLocal = min(peaksPerEvent, numel(groupIdx));
            selectedIdx(end + 1, 1) = groupIdx(pickLocal); %#ok<AGROW>
            groupStart = i;
        end
    end

    groupIdx = candidateIdx(groupStart:end);
    pickLocal = min(peaksPerEvent, numel(groupIdx));
    selectedIdx(end + 1, 1) = groupIdx(pickLocal);
end

function yq = sampleAtTimes(t, y, tq)
    t = t(:);
    y = y(:);
    tq = tq(:);

    [tUnique, uniqueIdx] = unique(t, "stable");
    yUnique = y(uniqueIdx);

    if numel(tUnique) == 1
        yq = repmat(yUnique(1), size(tq));
    else
        yq = interp1(tUnique, yUnique, tq, "linear", "extrap");
    end
end

function firstIdx = firstSustainedTrue(mask, holdSamples)
    firstIdx = nan;
    if isempty(mask)
        return;
    end
    runLength = 0;
    for i = 1:numel(mask)
        if mask(i)
            runLength = runLength + 1;
            if runLength >= holdSamples
                firstIdx = i - holdSamples + 1;
                return;
            end
        else
            runLength = 0;
        end
    end
end

function dt = estimateSampleInterval(t)
    diffs = diff(t(:));
    diffs = diffs(isfinite(diffs) & diffs > 0);
    if isempty(diffs)
        dt = 1e-3;
    else
        dt = median(diffs);
    end
end

function n = secondsToOddSamples(seconds, dt)
    n = max(3, round(seconds / dt));
    if mod(n, 2) == 0
        n = n + 1;
    end
end

function m = medianFinite(x)
    x = x(isfinite(x));
    if isempty(x)
        m = nan;
    else
        m = median(x);
    end
end

function s = robustMad(x)
    x = x(isfinite(x));
    if isempty(x)
        s = 0;
        return;
    end
    med = median(x);
    s = 1.4826 * median(abs(x - med));
    if ~isfinite(s) || s == 0
        s = std(x);
    end
    if ~isfinite(s) || s == 0
        s = eps;
    end
end

function addEventLines(times, color, lineStyle)
    times = times(:);
    times = times(isfinite(times));
    for i = 1:numel(times)
        xline(times(i), lineStyle, "Color", color, "LineWidth", 1.1);
    end
end
