%% ================ Initialization ================

clear; close all; clc;

fprintf('>>> Initializing MEKF algorithm...\n');

%% ================ Load IMU and Trajectory Data  ================

file_name = 'imu_trajectory_data_2025_11_04-21_33_26.mat';
try
    imu_trajectory_data = load(file_name);
    sensors = imu_trajectory_data.sensors;
    truth = imu_trajectory_data.truth;
    config = imu_trajectory_data.config;
catch ME
    error('Failed to load trajectory data: %s', ME.message);
end

% Preallocate estimation results
est = initialize_estimation_structure(sensors);

% Initialize filter state
[state, error_state, P] = initialize_filter_state();

% Process covariance matrix
Q = blkdiag(config.R_gyro, config.Q_bg);

% Sensor time indices
idx_acc = 1;
idx_mag = 1;

%% ================ MEKF Main Loop ================

fprintf('>>> Running MEKF estimation...\n');

for k = 1:length(sensors.gyro.time)
    % Calculate time step
    dt = calculate_time_step(sensors.gyro.time, k);
    
    % --- PROPAGATION STEP ---
    [state, error_state, P] = propagation_step(state, error_state, P, sensors.gyro.data(:, k), dt, Q);
    
    % --- UPDATE STEP ---
    [update_available, measurement_data] = check_sensor_availability(sensors, k, idx_acc, idx_mag);
    
    if update_available
        [state, P, idx_acc, idx_mag] = update_step(state, error_state, P, measurement_data, sensors, config, idx_acc, idx_mag);
    end

    % Reset error state
    error_state.dq = zeros(3,1);
    error_state.dbg = zeros(3,1);
    
    % Store results
    est.q(:, k) = state.q_nom;
    est.bg(:, k) = state.bg;
end

fprintf('>>> MEKF estimation completed successfully.\n');

%% ================ Generate Results ================

generate_estimation_plots(est, truth);

% Calculate estimation performance if truth data is available
calculate_estimation_performance(est, truth);

%% ================ Helper Functions ================

function est = initialize_estimation_structure(sensors)
% INITIALIZE_ESTIMATION_STRUCTURE Preallocate memory for estimation results
    n_points = length(sensors.gyro.time);
    
    est.q = zeros(4, n_points);
    est.bg = zeros(3, n_points);
    est.time = sensors.gyro.time;
    est.P_history = cell(1, n_points);  % Optional: store covariance if needed
end

function [state, error_state, P] = initialize_filter_state()
% INITIALIZE_FILTER_STATE Initialize filter state and covariance
    % Initial quaternion (nominal part)
    state.q_nom = [1; 0; 0; 0];  % Identity quaternion
    
    % Initial gyro bias estimate
    state.bg = [0; 0; 0];
    
    % Initial error state [orientation error; bias error]
    dq0 = [8.5e-3; 8.5e-3; 8.5e-3];   % Quaternion orientation error
    dbg0 = [1e-3; 1e-3; 1e-3];        % Gyro bias error

    % Initial error state
    error_state.dq = dq0;
    error_state.dbg = dbg0;
    
    % Initial error state covariance
    P = diag([...
        (8.5e-3)^2, (8.5e-3)^2, (8.5e-3)^2, ...  % Orientation error variance
        (1e-3)^2, (1e-3)^2, (1e-3)^2 ...         % Bias error variance
    ]);
end

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

function Phi = build_error_state_transition_matrix(omega, dt)
% BUILD_ERROR_STATE_TRANSITION_MATRIX Build error state transition matrix
    Phi = eye(6);
    Phi(1:3, 1:3) = Phi(1:3, 1:3) - skew(omega) * dt;
    Phi(1:3, 4:6) = -0.5 * eye(3) * dt;
end

function [update_available, measurement_data] = check_sensor_availability(sensors, k, idx_acc, idx_mag)
% CHECK_SENSOR_AVAILABILITY Check if sensor measurements are available
    
    tolerance = 1e-4;
    current_time = sensors.gyro.time(k);
    
    % Check accelerometer availability
    acc_available = (idx_acc <= length(sensors.accel.time)) && (abs(current_time - sensors.accel.time(idx_acc)) < tolerance);
    
    % Check magnetometer availability
    mag_available = (idx_mag <= length(sensors.mag.time)) && (abs(current_time - sensors.mag.time(idx_mag)) < tolerance);
    
    update_available = acc_available || mag_available;
    
    % Package measurement data
    measurement_data.acc_available = acc_available;
    measurement_data.mag_available = mag_available;
    measurement_data.current_time = current_time;
end

function [H_acc, z_acc] = build_accelerometer_update(state, acc_measurement)
% BUILD_ACCELEROMETER_UPDATE Build accelerometer measurement model
    
    % Normalize measurement
    y_acc = acc_measurement / norm(acc_measurement);
    
    % Expected gravity direction in body frame
    R_be = quat_to_dcm(state.q_nom);
    est_gravity_body = R_be * [0; 0; 1];  % Gravity in NED frame
    
    % Measurement innovation
    z_acc = y_acc - est_gravity_body;
    
    % Measurement Jacobian
    H_acc = [2 * skew(est_gravity_body), zeros(3, 3)];
end

function [H_mag, z_mag] = build_magnetometer_update(state, mag_measurement, config)
% BUILD_MAGNETOMETER_UPDATE Build magnetometer measurement model
    
    % Normalize measurement
    y_mag = mag_measurement / norm(mag_measurement);
    
    % Expected magnetic field direction in body frame
    R_be = quat_to_dcm(state.q_nom);
    est_mag_body = R_be * config.m_ned;
    
    % Measurement innovation
    z_mag = y_mag - est_mag_body;
    
    % Measurement Jacobian
    H_mag = [2 * skew(est_mag_body), zeros(3, 3)];
end

function [state, P] = perform_kalman_update(state, error_state, P, H, z, R)
% PERFORM_KALMAN_UPDATE Perform Kalman filter update step
    
    % Innovation covariance
    S = H * P * H' + R;
    
    % Kalman gain
    K = P * H' / S;
    
    % Error state update (dx = [dtheta; dbg])
    dx = [error_state.dq;error_state.dbg] + K * z;
    
    % Covariance update (Joseph form)
    I = eye(size(P));
    P = (I - K * H) * P * (I - K * H)' + K * R * K';
    
    % Apply correction to nominal states
    dq = [1; dx(1:3)];  % Small angle approximation for quaternion
    dq = dq / norm(dq);
    
    state.q_nom = quat_prod(dq, state.q_nom);
    state.bg = state.bg + dx(4:6);
    
    % Reset error state
    % Note: In MEKF, the error state is reset to zero after update
end

function [state, error_state, P] = propagation_step(state, error_state, P, gyro_measurement, dt, Q)
% PROPAGATION_STEP Perform MEKF propagation step
    
    % Estimated angular rate (gyro measurement minus bias)
    est_omega_B = gyro_measurement - state.bg;
    
    % Quaternion propagation using first-order integration
    Omega = omega_matrix(est_omega_B);
    Theta = eye(4) + 0.5 * Omega * dt;
    state.q_nom = Theta * state.q_nom;
    state.q_nom = state.q_nom / norm(state.q_nom);  % Normalize
    
    % State transition matrix
    Phi = build_error_state_transition_matrix(est_omega_B, dt);

    % Error state propagation
    error_state_propagation = Phi * [error_state.dq; error_state.dbg];
    error_state.dq = error_state_propagation(1:3);
    error_state.dbg = error_state_propagation(4:6);
    
    % Process noise transformation matrix
    Lambda = blkdiag(-0.5 * eye(3), eye(3)) * sqrt(dt);

    % Covariance propagation (Joseph form for numerical stability)
    P = Phi * P * Phi' + Lambda * Q * Lambda';
end

function [state, P, new_idx_acc, new_idx_mag] = update_step(state, error_state, P, measurement_data, sensors, config, idx_acc, idx_mag)
% UPDATE_STEP Perform MEKF update step with available measurements
    
    H_all = [];
    z_all = [];
    R_blocks = {};
    
    new_idx_acc = idx_acc;
    new_idx_mag = idx_mag;
    
    % Accelerometer update (gravity reference)
    if measurement_data.acc_available
        [H_acc, z_acc] = build_accelerometer_update(state, sensors.accel.data(:, new_idx_acc));
        
        H_all = [H_all; H_acc];
        z_all = [z_all; z_acc];
        R_blocks{end+1} = config.R_acc;
        
        new_idx_acc = new_idx_acc + 1;
    end
    
    % Magnetometer update (magnetic north reference)
    if measurement_data.mag_available
        [H_mag, z_mag] = build_magnetometer_update(state, sensors.mag.data(:, new_idx_mag), config);
        
        H_all = [H_all; H_mag];
        z_all = [z_all; z_mag];
        R_blocks{end+1} = config.R_mag;
        
        new_idx_mag = new_idx_mag + 1;
    end
    
    % Combine measurement noise covariance
    R_all = blkdiag(R_blocks{:});
    
    % Kalman update
    [state, P] = perform_kalman_update(state, error_state, P, H_all, z_all, R_all);
end

function generate_estimation_plots(est, truth)
% GENERATE_ESTIMATION_PLOTS Generate plots for estimation results
    
    % Estimated quaternions
    figure('Name', 'Estimated vs Real Attitude ', 'Position', [100 100 800 600]);
    stairs(est.time, est.q', 'LineWidth', 1.5);
    hold on;
    grid on;
    plot(truth.time, truth.quat', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Quaternion Components');
    title('Estimated vs Real Attitude Quaternions');
    legend('$$\hat{q}_0$$', '$$\hat{q}_1$$', '$$\hat{q}_2$$', '$$\hat{q}_3$$', '$$q_0$$', '$$q_1$$', '$$q_2$$', '$$q_3$$', 'Location', 'best', 'Interpreter', 'Latex');
    
    % Estimated gyro bias
    figure('Name', 'Estimated Gyro Bias', 'Position', [200 200 800 600]);
    stairs(est.time, est.bg', 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]'); ylabel('Bias [rad/s]');
    title('Estimated Gyro Bias');
    legend('$$\hat{b}_{gx}$$', '$$\hat{b}_{gy}$$', '$$\hat{b}_{gz}$$', 'Location', 'best', 'Interpreter', 'Latex');
    
    % Quaternion norm (should be close to 1)
    figure('Name', 'Quaternion Norm', 'Position', [300 300 600 400]);
    q_norm = sqrt(sum(est.q.^2, 1));
    plot(est.time, q_norm, 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]'); ylabel('Norm');
    title('Quaternion Norm (should be 1)');
    ylim([0.99, 1.01]);
end

function calculate_estimation_performance(est, truth)
% CALCULATE_ESTIMATION_PERFORMANCE Calculate estimation performance metrics
    if isempty(truth) || ~isfield(truth, 'quat')
        fprintf('>>> No truth data available for performance evaluation.\n');
        return;
    end
    
    % Interpolate truth data to estimation times
    truth_q_interp = zeros(4, length(est.time));
    for i = 1:4
        truth_q_interp(i, :) = interp1(truth.time, truth.quat(i, :), est.time, 'linear', 'extrap');
    end
    
    % Calculate quaternion error
    q_error = zeros(1, length(est.time));
    for k = 1:length(est.time)
        q_err = quat_prod(quat_conj(est.q(:, k)), truth_q_interp(:, k));
        q_error(k) = 2 * acos(abs(q_err(1)));  % Angular error in radians
    end
    
    % Plot estimation error
    figure('Name', 'Estimation Error', 'Position', [400 400 800 400]);
    plot(est.time, rad2deg(q_error), 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]'); ylabel('Attitude Error [deg]');
    title('Attitude Estimation Error');
    
    fprintf('>>> Estimation Performance:\n');
    fprintf('    - Mean attitude error: %.4f deg\n', mean(rad2deg(q_error)));
    fprintf('    - Max attitude error: %.4f deg\n', max(rad2deg(q_error)));
    fprintf('    - RMS attitude error: %.4f deg\n', rms(rad2deg(q_error)));
end