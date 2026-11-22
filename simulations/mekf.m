%% ================ Initialization ================

clear; close all; clc;

addpath(genpath('../src'));

plots = true;

fprintf('>>> Initializing MEKF algorithm...\n');

%% ================ Load Sensor and Trajectory Data  ================

file_name = 'truth_sensor_data_2026_22_11-21_37_33.mat';
try
    imu_trajectory_data = load(file_name);
    sensors = imu_trajectory_data.sensors;
    truth = imu_trajectory_data.truth;
    config = imu_trajectory_data.config;
catch ME
    error('Failed to load truth and sensor data: %s', ME.message);
end

est = initialize_estimation_structure(sensors);

% Initialize filter state
[x, dx, P] = initialize_filter_state(truth);

% Discrete process noise covariance matrix
Qk = zeros(6);
Qk(1:3,1:3) = config.R_gyro * (1/config.fs_gyro)^2 + 1/3 * config.Q_bg * (1/config.fs_gyro)^3;
Qk(1:3,4:6) = -1/2 * config.Q_bg * (1/config.fs_gyro)^2;
Qk(4:6,1:3) = Qk(1:3,4:6);
Qk(4:6,4:6) = config.Q_bg / config.fs_gyro;

% Sensor time indices
idx_acc = 1;
idx_mag = 1;

%% ================ MEKF Main Loop ================

fprintf('>>> Running MEKF estimation...\n');

for k = 1:length(sensors.gyro.time)
%for k = 1:10
    
    % Calculate time step
    tk = sensors.gyro.time(k);
    dt = calculate_time_step(sensors.gyro.time, k);
    
    % --- PROPAGATION STEP ---
    [x, dx, P] = propagation_step(x, dx, P, sensors.gyro.data(:, k), dt, Qk);
    P
    % --- UPDATE STEP ---
    [update_available, measurement_data] = check_sensor_availability(tk, sensors, idx_acc, idx_mag);
    if update_available
        [dx, P, idx_acc, idx_mag] = update_step(x, P, measurement_data, sensors, config, idx_acc, idx_mag);
    end
    P
    % Apply correction to nominal states
    qs = x(1);
    qv = x(2:4);
    x(1:4) = x(1:4) + 0.5 * [-qv'; qs * eye(3) + skew(qv)] * dx(1:3);
    x(5:7) = x(5:7) + dx(4:6);

    % Reset error state
    dx = zeros(6,1);
    
    % Store results
    est.q(:, k)  = x(1:4);
    est.bg(:, k) = x(5:7);
    est.P{k} = P;
end

fprintf('>>> MEKF estimation completed successfully.\n');

%% ================ Generate Results ================

if plots
    generate_estimation_plots(est, truth, sensors, config);
end

calculate_estimation_performance(est.t, est.q, truth.time, truth.quat);





