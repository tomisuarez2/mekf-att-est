%% ================ Initialization ================

clear; close all; clc;

addpath('./Example trajectory/')

%% ================ Configuration Parameters ================

% Sensor noise characteristics 
CONFIG.R_gyro  = diag([2.830380375642008306e-06, 4.934009191301758550e-06, 3.320219599119017912e-06]);
CONFIG.R_acc   = diag([1.090681860817302234e-03, 9.279916753344519612e-04, 2.146372869350902587e-03]);
CONFIG.R_mag   = 0.1 * diag([2.890238578269250028e-03, 1.901404294437207543e-02, 7.715742865748796697e-03]);
CONFIG.Q_bg    = (1e-10) * eye(3);

% Sampling frequencies [Hz]
CONFIG.fs_gyro = 100;
CONFIG.fs_acc  = 50;
CONFIG.fs_mag  = 25;

% Earth constants
CONFIG.g_ned   = [0; 0; 9.81];  % gravity vector in NED [m/s^2]
CONFIG.m_ned   = [18823.7; -584.3; -13244.4];  % [nT]
CONFIG.m_ned   = CONFIG.m_ned / norm(CONFIG.m_ned);  % normalized magnetic field

%% ================ Load and Process Trajectory Data from Simulink Model ================

try
    trajectory_data = load('state_trajectory.mat');
    X = trajectory_data.ans;
catch ME
    error('Failed to load trajectory data: %s', ME.message);
end

% Time
time = X.q.Time;

% Extract position data (NED frame: x=N, y=E, z=D)
pos = extract_trajectory_data(X.posE.data);

% Extract velocity in Earth frame
vel = extract_trajectory_data(X.velE.data);

% Extract acceleration in Earth frame
accel_e = extract_trajectory_data(X.acelE.data);

% Extract acceleration in Body frame
acceL_b = extract_trajectory_data(X.acelB.data);

% Extract angular velocity in Body frame
omega_b = extract_trajectory_data(X.omegaB.data);

% Extract attitude quaternions
quat = squeeze(X.q.data);
if size(quat, 1) ~= 4
    quat = quat';  % Ensure quaternions are 4×N
end

% Extract rotation matrices
R_be = X.DCMbe.data;

%% ================ Compute Sensor Measurements ================

[mag_truth, sp_force_truth] = compute_sensor_measurements(accel_e, R_be, CONFIG);

%% ================ Generate Noisy IMU Data ================

[sensors, gyro_bias] = generate_imu_measurements(time, omega_b, sp_force_truth, mag_truth, CONFIG);

%% ================ Package Truth Data ================

truth = package_truth_data(time, pos, vel, accel_e, quat, gyro_bias);

%% ================ Save Data ================

save_generated_data(truth, sensors, CONFIG);

%% ================ Generate Plots ================

generate_validation_plots(truth, sensors);

fprintf('>>> Trajectory and simulated IMU measurements have been generated.\n');

rmpath('./Example trajectory/');

clear;

%% ================ Helper Functions ================

function data = extract_trajectory_data(raw_data)
% EXTRACT_TRAJECTORY_DATA Extract and reshape trajectory data
% Converts from 3×1×N to 3×N format
    data = squeeze(raw_data);
    if size(data, 1) ~= 3
        data = data';
    end
end

function [mag_truth, sp_force_truth] = compute_sensor_measurements(accel_e, R_be, config)
% COMPUTE_SENSOR_MEASUREMENTS Compute ideal sensor measurements
    n_points = size(accel_e, 2);
    mag_truth = zeros(3, n_points);
    sp_force_truth = zeros(3, n_points);
    
    for k = 1:n_points
        % Specific force = acceleration + gravity (in Earth frame)
        f_e = accel_e(:, k) + config.g_ned;
        sp_force_truth(:, k) = R_be(:, :, k) * f_e;
        
        % Magnetic field in body frame
        mag_truth(:, k) = R_be(:, :, k) * config.m_ned;
    end
end

function [sensors, gyro_bias] = generate_imu_measurements(time, omega_b, sp_force, mag, config)
% GENERATE_IMU_MEASUREMENTS Generate noisy IMU measurements at different rates
    
    % Create time vectors for each sensor
    t_gyro = 0:(1/config.fs_gyro):time(end);
    t_acc  = 0:(1/config.fs_acc):time(end);
    t_mag  = 0:(1/config.fs_mag):time(end);
    
    % Initialize sensor structures
    sensors = struct();
    
    % Generate gyroscope measurements
    gyro_clean = interpolate_data(time, omega_b, t_gyro);
    gyro_bias = cumsum(sqrt(config.Q_bg) * randn(3, length(t_gyro)), 2);
    sensors.gyro.data = gyro_clean + sqrt(config.R_gyro) * randn(size(gyro_clean)) + gyro_bias;
    sensors.gyro.time = t_gyro;
    sensors.gyro.fs   = config.fs_gyro;
    
    % Generate accelerometer measurements
    accel_clean = interpolate_data(time, sp_force, t_acc);
    sensors.accel.data = accel_clean + sqrt(config.R_acc) * randn(size(accel_clean));
    sensors.accel.time = t_acc;
    sensors.accel.fs   = config.fs_acc;
    
    % Generate magnetometer measurements
    mag_clean = interpolate_data(time, mag, t_mag);
    sensors.mag.data = mag_clean + sqrt(config.R_mag) * randn(size(mag_clean));
    sensors.mag.time = t_mag;
    sensors.mag.fs   = config.fs_mag;
end

function interpolated = interpolate_data(original_time, original_data, new_time)
% INTERPOLATE_DATA Interpolate data to new time vector
    interpolated = zeros(size(original_data, 1), length(new_time));
    for i = 1:size(original_data, 1)
        interpolated(i, :) = interp1(original_time, original_data(i, :), new_time, 'linear', 'extrap');
    end
end

function truth = package_truth_data(time, pos, vel, accel, quat, gyro_bias)
% PACKAGE_TRUTH_DATA Package ground truth data into structure
    truth.time = time;
    truth.pos  = pos;
    truth.vel  = vel;
    truth.acc  = accel;
    truth.quat = quat;
    truth.gyro_bias = gyro_bias;
end

function save_generated_data(truth, sensors, config)
% SAVE_GENERATED_DATA Save generated data to files
    dt = datetime('now');
    timestamp = datestr(dt, 'yyyy_mm_dd-HH_MM_SS');
    
    % Save main data file
    filename = sprintf('Sensor and real data/imu_trajectory_data_%s.mat', timestamp);
    save(filename, 'truth', 'sensors', 'config', '-v7.3');
    fprintf('>>> Data saved to: %s\n', filename);
end

function generate_validation_plots(truth, sensors)
% GENERATE_VALIDATION_PLOTS Generate validation plots
    
    % Position plot
    figure('Name', 'Trajectory', 'Position', [100 100 800 600]);
    plot3(truth.pos(1, :), truth.pos(2, :), -truth.pos(3, :)); 
    grid on; axis equal;
    xlabel('North [m]'); ylabel('East [m]'); zlabel('Altitude [m]');
    title('Drone Ground-Truth Trajectory');
    
    % Quaternions plot
    figure('Name', 'Attitude', 'Position', [200 200 800 400]);
    plot(truth.time, truth.quat); grid on;
    xlabel('Time [s]'); ylabel('q_i'); 
    title('Attitude Quaternions');
    legend('q_0', 'q_1', 'q_2', 'q_3');
    
    % Gyro bias plot
    figure('Name', 'Gyro Bias', 'Position', [300 300 800 400]);
    plot(sensors.gyro.time, truth.gyro_bias'); grid on;
    xlabel('Time [s]'); ylabel('b_g [rad/s]'); 
    title('Gyro Drift Bias');
    legend('X', 'Y', 'Z');
    
    % Gyro measurements plot
    figure('Name', 'Gyro Measurements', 'Position', [300 300 800 400]);
    plot(sensors.gyro.time, sensors.gyro.data'); grid on;
    xlabel('Time [s]'); ylabel('[rad/s]');
    title('Gyro Measurements');
    legend('\omega_x', '\omega_y', '\omega_z');
    
    % Accelerometer measurements plot
    figure('Name', 'Accelerometer Measurements', 'Position', [300 300 800 400]);
    plot(sensors.accel.time, sensors.accel.data'); grid on;
    xlabel('Time [s]'); ylabel('[m/s^2]');
    legend('a_x', 'a_y', 'a_z');
    title('Accelerometer Measurements');
    
    % Magnetometer measurements plot
    figure('Name', 'Magnetometer Measurements', 'Position', [300 300 800 400]);
    plot(sensors.mag.time, sensors.mag.data'); grid on;
    xlabel('Time [s]'); ylabel('Normalized Units [-]');
    title('Magnetometer Measurements');
    legend('m_x', 'm_y', 'm_z');
end