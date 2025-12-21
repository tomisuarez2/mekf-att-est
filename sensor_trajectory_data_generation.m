%% ================ Initialization ================

clear; close all; clc;

% Data selection
euro_mav = true;

addpath('./Attitude kinematics function/') 

%% ================ Configuration Parameters ================

% Sensor noise characteristics, based on real cheap sensors. EuroMav is designed to operate with more precise sensors.
CONFIG.R_gyro  = 0.1*diag([2.830380375642008306e-06, 4.934009191301758550e-06, 3.320219599119017912e-06]);
CONFIG.R_acc   = 0.1*diag([1.090681860817302234e-03, 9.279916753344519612e-04, 2.146372869350902587e-03]);
CONFIG.R_mag   = 0.1*diag([2.890238578269250028e-03, 1.901404294437207543e-02, 7.715742865748796697e-03]);
CONFIG.Q_bg    = 1*(1e-8) * eye(3);

% Sampling frequencies [Hz]
CONFIG.fs_gyro = 100;
CONFIG.fs_acc  = 100;
CONFIG.fs_mag  = 50;

% Earth constants
CONFIG.g_ned   = [0; 0; 9.81];  % gravity vector in NED [m/s^2]
CONFIG.m_ned   = [18823.7; -584.3; -13244.4];  % [nT]
CONFIG.m_ned   = CONFIG.m_ned / norm(CONFIG.m_ned);  % normalized magnetic field

%% ================ Load and Process Trajectory Data from Simulink Model ================

if euro_mav % Real precise ground truth data

    addpath('./mav0/state_groundtruth_estimate0/');
    gt_dataset_name = "data.csv";

    % Read data
    ground_truth_data = readmatrix(gt_dataset_name);

    % Time data [s]
    date_time = datetime(uint64(ground_truth_data(:,1)),'ConvertFrom','epochtime','TicksPerSecond',1e9,'Format','dd-MMM-yyyy HH:mm:ss.SSSSSSSSS');
    start = date_time(1);
    time = seconds(date_time - start);
    n_truth_samples = length(time);

    % Position data [m] (ENU frame: x=E, y=N, z=U)
    pos = ground_truth_data(:,2:4)';

    % Quaternion data (Scalar first), world to body
    quat = zeros(4, n_truth_samples);
    for i=1:n_truth_samples
        quat(:,i) = quat_conj(ground_truth_data(i,5:8)');
    end

    % Ground truth velocity data [m/s]
    vel = ground_truth_data(:, 9:11)';

    % Body frame velocity data [m/s]
    vel_b = zeros(3, n_truth_samples);
    for i=1:n_truth_samples
        vel_b(:,i) = quat_to_dcm(quat(:,i)) * vel(:,i);
    end
    
    % Ground truth acceleration data [m/s^2]
    acc(1,:) = gradient(vel(1,:), time);
    acc(2,:) = gradient(vel(2,:), time);
    acc(3,:) = gradient(vel(3,:), time);

    rmpath('./mav0/state_groundtruth_estimate0/');

    % Change world reference from ENU to NED
    R_enu_to_ned = [0 1 0;1 0 0;0 0 -1];
    q_n_e = dcm_to_quat(R_enu_to_ned);   
    for i = 1:n_truth_samples
        pos(:,i) = R_enu_to_ned * pos(:,i);
        vel(:,i) = R_enu_to_ned * vel(:,i);
        acc(:, i) = R_enu_to_ned * acc(:,i);
        quat(:,i) = quat_prod(q_n_e, quat(:,i));
        quat(:,i) = quat(:,i) / norm(quat(:,i));
    end  

    % Change body frame to make first orientation coincidence with NED frame.
    quat_body_change = quat_conj(quat(:,1));
    for i = 1:n_truth_samples
        quat(:,i) = quat_prod(quat(:,i), quat_body_change);
        quat(:,i) = quat(:,i) / norm(quat(:,i));
    end 

    % Quaternion rate of change data
    quat_roc(1,:) = gradient(quat(1,:), time);
    quat_roc(2,:) = gradient(quat(2,:), time);
    quat_roc(3,:) = gradient(quat(3,:), time);
    quat_roc(4,:) = gradient(quat(4,:), time);

    % Body frame angular velocity data [rad/s] 
    omega_b = zeros(3, n_truth_samples);
    for i=1:n_truth_samples
        bar_omega_b = 2 * quat_prod(quat_roc(:,i), quat_conj(quat(:,i)));
        omega_b(:,i) = bar_omega_b(2:4, 1);
    end

    % Body frame linear acceleration data [m/s^2]
    acc_b = zeros(3, n_truth_samples);
    for i=1:n_truth_samples
        acc_b(:,i) = quat_to_dcm(quat(:,i)) * acc(:,i); % Body COM and reference frame B center coincide
    end

else

    addpath('./Example trajectory/');

    try
        trajectory_data = load('state_trajectory.mat');
        X = trajectory_data.ans;
    catch ME
        error('Failed to load trajectory data: %s', ME.message);
    end

    % Time data [s]
    time = X.q.Time;
    n_truth_samples = length(time);

    % Position data [m] (NED frame: x=N, y=E, z=D)
    pos = extract_trajectory_data(X.posE.data);

    % Velocity truth velocity data [m/s]
    vel = extract_trajectory_data(X.velE.data);

    % Quaternion data (Scalar first)
    quat = squeeze(X.q.data);
    if size(quat, 1) ~= 4
        quat = quat';  % Ensure quaternions are 4×N
    end

    % Ground truth acceleration data [m/s^2]
    acc_be = extract_trajectory_data(X.acelE.data);
    acc = zeros(3, n_truth_samples);
    for i=1:n_truth_samples
        acc(:,i) = quat_to_dcm(quat(:,i))' * acc_be(:,i);
    end

    % Body frame angular velocity data [rad/s] 
    omega_b = extract_trajectory_data(X.omegaB.data);
    
    % Body frame linear acceleration data [m/s^2]
    acc_b = acc_be;

    rmpath('./Example trajectory/');
end

%% ================ Compute Sensor Measurements ================

[mag_truth, sp_force_truth] = compute_sensor_measurements(acc_b, quat, CONFIG);

%% ================ Generate Noisy Sensor Data ================

[sensors, gyro_bias] = generate_sensor_measurements(time, omega_b, sp_force_truth, mag_truth, CONFIG);

%% ================ Package Truth Data ================

truth = package_truth_data(time, quat, pos, vel, acc, omega_b, acc_b, gyro_bias);

%% ================ Save Data ================

save_generated_data(truth, sensors, CONFIG);

%% ================ Generate Plots ================

generate_validation_plots(truth, sensors);

fprintf('>>> Trajectory and simulated sensor measurements have been generated.\n');

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

function [mag_truth, sp_force_truth] = compute_sensor_measurements(accel_b, quat, config)
% COMPUTE_SENSOR_MEASUREMENTS Compute ideal sensor measurements
    n_points = size(accel_b, 2);
    mag_truth = zeros(3, n_points);
    sp_force_truth = zeros(3, n_points);
    
    for k = 1:n_points
        % Specific force = acceleration - gravity 
        R_be = quat_to_dcm(quat(:, k));
        sp_force_truth(:, k) = accel_b(:, k) - R_be * config.g_ned;
        
        % Magnetic field in body frame
        mag_truth(:, k) = R_be * config.m_ned;
    end
end

function [sensors, gyro_bias] = generate_sensor_measurements(time, omega_b, sp_force, mag, config)
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

function truth = package_truth_data(time, quat, pos, vel, acc, ang_vel, body_acc, gyro_bias)
% PACKAGE_TRUTH_DATA Package ground truth data into structure
    truth.time = time;
    truth.quat = quat;
    truth.pos  = pos;
    truth.vel  = vel;
    truth.acc  = acc;
    truth.ang_vel = ang_vel;
    truth.acc_b = body_acc;
    truth.gyro_bias = gyro_bias;
end

function save_generated_data(truth, sensors, config)
% SAVE_GENERATED_DATA Save generated data to files
    dt = datetime('now');
    timestamp = datestr(dt, 'yyyy_mm_dd-HH_MM_SS');
    
    % Save main data file
    filename = sprintf('Sensor and real data/truth_sensor_data_%s.mat', timestamp);
    save(filename, 'truth', 'sensors', 'config', '-v7.3');
    fprintf('>>> Data saved to: %s\n', filename);
end

function generate_validation_plots(truth, sensors)
% GENERATE_VALIDATION_PLOTS Generate validation plots
    
    % Quaternion plot
    figure('Name', 'Quaternion', 'Position', [300 300 800 400]);
    plot(truth.time, truth.quat'); grid on;
    xlabel('Time [s]'); ylabel('[]');
    title('Drone Quaternion Evolution');
    legend('q_w', 'q_x', 'q_y', 'q_z');

    % Position plot
    figure('Name', 'Trajectory', 'Position', [100 100 800 600]);
    plot3(truth.pos(1, :), truth.pos(2, :), -truth.pos(3, :)); 
    grid on; axis equal;
    xlabel('North [m]'); ylabel('East [m]'); zlabel('-Down [m]');
    title('Drone Ground-Truth Trajectory');

    % Velocity plot
    figure('Name', 'Velocity', 'Position', [300 300 800 400]);
    plot(truth.time, truth.vel'); grid on;
    xlabel('Time [s]'); ylabel('[m/s]');
    title('Drone Ground-Truth Velocity');
    legend('North', 'East', 'Down');

    % Acceleration plot
    figure('Name', 'Acceleration', 'Position', [300 300 800 400]);
    plot(truth.time, truth.acc'); grid on;
    xlabel('Time [s]'); ylabel('[m/s^2]');
    title('Drone Ground-Truth Acceleration');
    legend('North', 'East', 'Down');

    % Body frame angular velocity plot
    figure('Name', 'Angular Velocity', 'Position', [300 300 800 400]);
    plot(truth.time, truth.ang_vel'); grid on;
    xlabel('Time [s]'); ylabel('[rad/s]');
    title('Drone Body Frame Angular Velocity');
    legend('wx', 'wy', 'wz');
    
    % Gyro bias plot
    figure('Name', 'Gyro Bias', 'Position', [300 300 800 400]);
    stairs(sensors.gyro.time, truth.gyro_bias'); grid on;
    xlabel('Time [s]'); ylabel('b_g [rad/s]'); 
    title('Gyro Drift Bias');
    legend('X', 'Y', 'Z');
    
    % Gyro measurements plot
    figure('Name', 'Gyro Measurements', 'Position', [300 300 800 400]);
    stairs(sensors.gyro.time, sensors.gyro.data'); grid on;
    xlabel('Time [s]'); ylabel('[rad/s]');
    title('Gyro Measurements');
    legend('\omega_x', '\omega_y', '\omega_z');
    
    % Accelerometer measurements plot
    figure('Name', 'Accelerometer Measurements', 'Position', [300 300 800 400]);
    stairs(sensors.accel.time, sensors.accel.data'); grid on;
    xlabel('Time [s]'); ylabel('[m/s^2]');
    title('Accelerometer Measurements');
    legend('a_x', 'a_y', 'a_z');
    
    % Magnetometer measurements plot
    figure('Name', 'Magnetometer Measurements', 'Position', [300 300 800 400]);
    stairs(sensors.mag.time, sensors.mag.data'); grid on;
    xlabel('Time [s]'); ylabel('Normalized Units [-]');
    title('Magnetometer Measurements');
    legend('m_x', 'm_y', 'm_z');
end