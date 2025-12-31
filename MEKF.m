%% ================ Initialization ================

clear; close all; clc;

addpath('./MEKF functions/')
addpath('./Attitude kinematics functions/')
addpath('./Sensor and real data/')

fprintf('>>> Initializing MEKF algorithm...\n');

%% ================ Load Sensor and Trajectory Data  ================

file_name = 'truth_sensor_data_2025_12_31-13_40_46.mat';
try
    imu_trajectory_data = load(file_name);
    sensors = imu_trajectory_data.sensors;
    truth = imu_trajectory_data.truth;
    config = imu_trajectory_data.config;
catch ME
    error('Failed to load truth and sensor data: %s', ME.message);
end

% Preallocate estimation results
est_MEKF = initialize_estimation_structure_MEKF(sensors);

% Initialize filter state
[state_MEKF, error_state_MEKF, P_MEKF] = initialize_filter_state_MEKF(truth);

% Process covariance matrix
Qc_MEKF = blkdiag(config.R_gyro*config.fs_gyro, config.Q_bg);

% Sensor time indices
idx_acc = 1;
idx_mag = 1;

%% ================ MEKF Main Loop ================

fprintf('>>> Running MEKF estimation...\n');

for k = 1:length(sensors.gyro.time)
    % Calculate time step
    current_time = sensors.gyro.time(k);
    dt = calculate_time_step(sensors.gyro.time, k);
    
    % --- PROPAGATION STEP ---
    [state_MEKF, error_state_MEKF, P_MEKF] = propagation_step_MEKF(state_MEKF, error_state_MEKF, P_MEKF, sensors.gyro.data(:, k), dt, Qc_MEKF);
    
    % --- UPDATE STEP ---
    [update_available_MEKF, measurement_data_MEKF] = check_sensor_availability_MEKF(current_time, sensors, idx_acc, idx_mag);
    if update_available_MEKF
        [state_MEKF, P_MEKF, idx_acc, idx_mag] = update_step_MEKF(state_MEKF, error_state_MEKF, P_MEKF, measurement_data_MEKF, sensors, config, idx_acc, idx_mag);
    end

    % Reset error state
    error_state_MEKF.dq = zeros(3,1);
    error_state_MEKF.dbg = zeros(3,1);
    
    % Store results
    est_MEKF.quat(:, k) = state_MEKF.q_nom;
    est_MEKF.bg(:, k) = state_MEKF.bg;
end

fprintf('>>> MEKF estimation completed successfully.\n');

%% ================ Generate Results ================

generate_estimation_plots_MEKF(est_MEKF, truth, sensors);

% Calculate estimation performance if truth data is available
calculate_estimation_performance_MEKF(est_MEKF, truth);

rmpath('./MEKF functions/')
rmpath('./Attitude kinematics functions/')
rmpath('./Sensor and real data/')

%% ================ Helper Functions ================

function dt = calculate_time_step(time_vector, current_index)
% CALCULATE_TIME_STEP Calculate time step between measurements
    if current_index == 1
        if length(time_vector) > 1
            dt = time_vector(2) - time_vector(1);
        else
            dt = 0.01;  % Default time step
        end
    else
        dt = time_vector(current_index) - time_vector(current_index - 1);
    end
    
    % Validate time step
    if dt <= 0
        warning('Non-positive time step detected at index %d. Using default.', current_index);
        dt = 0.01;
    end
end

function generate_estimation_plots_MEKF(est, truth, sensors)
% GENERATE_ESTIMATION_PLOTS Generate plots for estimation results
    
    % Estimated quaternions
    figure('Name', 'Estimated vs Real Attitude ', 'Position', [100 100 800 600]);
    stairs(est.time, est.quat', 'LineWidth', 1.5);
    hold on;
    grid on;
    plot(truth.time, truth.quat', 'LineWidth', 1.5, 'LineStyle', '--');
    xlabel('Time [s]'); ylabel('Quaternion Components');
    title('Estimated vs Real Attitude Quaternions');
    legend('$$\hat{q}_0$$', '$$\hat{q}_1$$', '$$\hat{q}_2$$', '$$\hat{q}_3$$', '$$q_0$$', '$$q_1$$', '$$q_2$$', '$$q_3$$', 'Location', 'best', 'Interpreter', 'Latex');
    
    % Estimated gyro bias
    figure('Name', 'Estimated Gyro Bias', 'Position', [200 200 800 600]);
    stairs(est.time, est.bg', 'LineWidth', 1.5);
    grid on; hold on
    stairs(sensors.gyro.time, truth.gyro_bias', 'LineStyle', '--');
    xlabel('Time [s]'); ylabel('Bias [rad/s]');
    title('Estimated Gyro Bias');
    legend('$$\hat{b}_{gx}$$', '$$\hat{b}_{gy}$$', '$$\hat{b}_{gz}$$', '$$b_{gx}$$', '$$b_{gy}$$', '$$b_{gz}$$', 'Location', 'best', 'Interpreter', 'Latex');

    % Quaternion norm 
    figure('Name', 'Quaternion Norm', 'Position', [300 300 600 400]);
    q_norm = sqrt(sum(est.quat.^2, 1));
    plot(est.time, q_norm, 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]'); ylabel('Norm');
    title('Quaternion Norm');
    %ylim([0.99, 1.01]);
end

