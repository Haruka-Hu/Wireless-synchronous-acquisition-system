% Plot IMU waveforms from CSV files under data/.
%
% Shows:
% 1. Acceleration resultant: sqrt(ax^2 + ay^2 + az^2)
% 2. Gyroscope Z axis: gz

clear; clc; close all;

scriptDir = fileparts(mfilename("fullpath"));
projectRoot = fileparts(scriptDir);
dataDir = fullfile(projectRoot, "data");

files = dir(fullfile(dataDir, "*.csv"));
if isempty(files)
    error("No CSV files found in %s", dataDir);
end

for fileIdx = 1:numel(files)
    csvPath = fullfile(files(fileIdx).folder, files(fileIdx).name);
    fprintf("Plotting %s\n", files(fileIdx).name);

    T = readtable(csvPath);
    source = string(T.source);

    acc = sortrows(T(strcmpi(source, "Accel"), :), "timestamp_us");
    gyro = sortrows(T(strcmpi(source, "Gyro"), :), "timestamp_us");

    if isempty(acc) || isempty(gyro)
        warning("Skip %s: missing Accel or Gyro data.", files(fileIdx).name);
        continue;
    end

    t0Us = min(double([acc.timestamp_us; gyro.timestamp_us]));

    tAcc = (double(acc.timestamp_us) - t0Us) / 1e6;
    accNorm = sqrt(double(acc.x).^2 + double(acc.y).^2 + double(acc.z).^2);

    tGyro = (double(gyro.timestamp_us) - t0Us) / 1e6;
    gz = double(gyro.z);

    figure("Name", files(fileIdx).name, "Color", "w");
    tiledlayout(2, 1, "TileSpacing", "compact", "Padding", "compact");

    nexttile;
    plot(tAcc, accNorm, "k-", "LineWidth", 0.8);
    grid on;
    ylabel("|a| raw");
    title("Acceleration resultant");

    nexttile;
    plot(tGyro, gz, "b-", "LineWidth", 0.8);
    grid on;
    xlabel("Time (s)");
    ylabel("gz raw");
    title("Gyroscope Z axis");
end

dtAcc = diff(double(acc.timestamp_us)) / 1000;   % ms
dtGyro = diff(double(gyro.timestamp_us)) / 1000; % ms

figure("Name", files(fileIdx).name + " timestamp dt", "Color", "w");
tiledlayout(2,1);

nexttile;
plot(dtAcc, "k-");
grid on;
ylabel("Accel dt (ms)");
title("Accel timestamp interval");

nexttile;
plot(dtGyro, "b-");
grid on;
ylabel("Gyro dt (ms)");
xlabel("Sample index");
title("Gyro timestamp interval");

