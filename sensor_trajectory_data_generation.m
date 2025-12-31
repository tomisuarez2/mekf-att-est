% ================ Initialization ================

clear; close all; clc;

% Data selection
euro_mav = true;

addpath('./Attitude kinematics functions/') 

%% ================ Configuration Parameters ================

% Sensor noise characteristics, based on real cheap sensors. EuroMav is designed to operate with more precise sensors.
CONFIG.R_acc  = 2.15e-03*eye(3); % [m^2/s^4]
CONFIG.Q_ba   = 1.00e-10*eye(3); % [m^2/s^5]
CONFIG.R_gyro = 4.93e-06*eye(3); % [rad^2/s^2]
CONFIG.Q_bg   = 3.00e-10*eye(3); % [rad^2/s^3]
CONFIG.R_mag  = 1.90e-02*eye(3); % [-]
CONFIG.R_baro = 1.85e-02; % [m^2]
CONFIG.Q_bb   = 1.29e-03; % [m^2/s]
CONFIG.Q_bgps = diag([0.00095, 0.00107, 0.00060]); % [m^2/s]
CONFIG.R_gps  = [1.071 0  0; 0  1.902 0; 0 0  3.404];  % [m^2]
CONFIG.R_tof  = 4.84e-6; % [m^2]
if euro_mav
    CONFIG.R_acc  = 0.1*CONFIG.R_acc;
    CONFIG.R_gyro = 0.01*CONFIG.R_gyro;
    CONFIG.R_mag  = 0.005*CONFIG.R_mag;
    CONFIG.R_baro = 0.1*CONFIG.R_baro;
    CONFIG.Q_bb   = 0.1*CONFIG.Q_bb;
    CONFIG.Q_bgps = 1.25e-03*CONFIG.Q_bgps;
    CONFIG.R_gps  = 0.05*CONFIG.R_gps;
end

% Sampling frequencies [Hz]
CONFIG.fs_acc  = 100;
CONFIG.fs_gyro = 100;
CONFIG.fs_mag  = 50;
CONFIG.fs_baro = 50;
CONFIG.fs_gps  = 20;
CONFIG.fs_tof  = 50;

% Measurement covariances to different sampling rates of the characterized ones
CONFIG.R_acc  = CONFIG.fs_acc/100*CONFIG.R_acc;
CONFIG.R_gyro = CONFIG.fs_gyro/100*CONFIG.R_gyro;
CONFIG.R_mag  = CONFIG.fs_mag/50*CONFIG.R_mag;
CONFIG.R_baro = CONFIG.fs_baro/80*CONFIG.R_baro;
CONFIG.R_tof  = CONFIG.fs_tof/50*CONFIG.R_tof;

% Earth constants
CONFIG.g_ned  = [0; 0; 9.81];  % gravity vector in NED [m/s^2]
CONFIG.m_ned  = [18823.7; -584.3; -13244.4];  % [nT]
CONFIG.m_ned  = CONFIG.m_ned / norm(CONFIG.m_ned);  % normalized magnetic field

%% ================ Load and Process Trajectory Data from Dataset or Simulink Model ================

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
    z_min = min(pos(3,:));
    pos = pos - [0;0;z_min];

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
    q_e_n = dcm_to_quat(R_enu_to_ned);   
    for i = 1:n_truth_samples
        pos(:,i) = R_enu_to_ned * pos(:,i);
        vel(:,i) = R_enu_to_ned * vel(:,i);
        acc(:, i) = R_enu_to_ned * acc(:,i);
        quat(:,i) = quat_prod(quat(:,i), quat_conj(q_e_n));
        quat(:,i) = quat(:,i) / norm(quat(:,i));
    end  

    % Change body frame to make first orientation coincidence with NED frame.
    quat_body_change = quat_conj(quat(:,1));
    for i = 1:n_truth_samples
        quat(:,i) = quat_prod(quat_body_change, quat(:,i));
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

[mag_truth, sp_force_truth, tof_truth] = compute_sensor_measurements(acc_b, quat, pos, CONFIG);

%% ================ Generate Noisy Sensor Data ================
                                                       
[sensors, accel_bias, gyro_bias, baro_bias, gps_bias] = generate_sensor_measurements(time, omega_b, sp_force_truth, mag_truth, tof_truth, pos, CONFIG);

%% ================ Package Truth Data ================

truth = package_truth_data(time, quat, pos, vel, acc, omega_b, acc_b, accel_bias, gyro_bias, baro_bias, gps_bias);

%% ================ Save Data ================

save_generated_data(truth, sensors, CONFIG);

%% ================ Generate Plots ================

generate_validation_plots(truth, sensors);

fprintf('>>> Trajectory and simulated sensor measurements have been generated.\n');

rmpath('./Attitude kinematics functions/') 

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

function [mag_truth, sp_force_truth, tof_truth] = compute_sensor_measurements(accel_b, quat, pos, config)
% COMPUTE_SENSOR_MEASUREMENTS Compute ideal sensor measurements
    n_points = size(accel_b, 2);
    mag_truth = zeros(3, n_points);
    sp_force_truth = zeros(3, n_points);
    tof_truth = zeros(1, n_points);

    for k = 1:n_points
        % Specific force = acceleration - gravity 
        R_be = quat_to_dcm(quat(:, k));
        sp_force_truth(:, k) = accel_b(:, k) - R_be * config.g_ned;
        % Magnetic field in body frame
        mag_truth(:, k) = R_be * config.m_ned;
        % Lidar measured distance
        tof_truth(k) = -R_be(3,3) * pos(3,k); % We are assuming we know at any time the position of the floor.
    end
end

function [sensors, accel_bias, gyro_bias, baro_bias, gps_bias] = generate_sensor_measurements(time, omega_b, sp_force, mag, tof_dist, pos, config)
% GENERATE_SENSOR_MEASUREMENTS Generate noisy sensor measurements at different rates
    
    % Create time vectors for each sensor
    t_acc  = 0:(1/config.fs_acc):time(end);
    t_gyro = 0:(1/config.fs_gyro):time(end);
    t_mag  = 0:(1/config.fs_mag):time(end);
    t_baro = 0:(1/config.fs_baro):time(end);
    t_gps  = 0:(1/config.fs_gps):time(end);
    t_tof  = 0:(1/config.fs_tof):time(end);
    
    % Initialize sensor structures
    sensors = struct();
      
    % Generate accelerometer measurements
    accel_clean = interpolate_data(time, sp_force, t_acc);
    accel_bias = cumsum(sqrt(config.Q_ba/config.fs_acc) * randn(3, length(t_acc)), 2);
    sensors.accel.data = accel_clean + sqrt(config.R_acc) * randn(size(accel_clean)) + accel_bias;
    sensors.accel.time = t_acc;
    sensors.accel.fs   = config.fs_acc;

    % Generate gyroscope measurements
    gyro_clean = interpolate_data(time, omega_b, t_gyro);
    gyro_bias = cumsum(sqrt(config.Q_bg/config.fs_gyro) * randn(3, length(t_gyro)), 2);
    sensors.gyro.data = gyro_clean + sqrt(config.R_gyro) * randn(size(gyro_clean)) + gyro_bias;
    sensors.gyro.time = t_gyro;
    sensors.gyro.fs   = config.fs_gyro;
    
    % Generate magnetometer measurements
    mag_clean = interpolate_data(time, mag, t_mag);
    sensors.mag.data = mag_clean + sqrt(config.R_mag) * randn(size(mag_clean));
    sensors.mag.time = t_mag;
    sensors.mag.fs   = config.fs_mag;

    % Generate barometer measurements
    baro_clean = interpolate_data(time, -pos(3,:), t_baro);
    baro_bias = cumsum(sqrt(config.Q_bb/config.fs_baro) * randn(1, length(t_baro)), 2);
    sensors.baro.data = baro_clean + sqrt(config.R_baro) * randn(size(baro_clean)) + baro_bias;
    sensors.baro.time = t_baro;
    sensors.baro.fs   = config.fs_baro;
    
    % Generate gps measurements
    gps_clean = interpolate_data(time, pos, t_gps);
    gps_bias = cumsum(sqrt(config.Q_bgps/config.fs_gps) * randn(3, length(t_gps)), 2);
    coeff= ones(1, 15)/15;
    gps_bias(1,:) = filter(coeff, 1, gps_bias(1,:));
    gps_bias(2,:) = filter(coeff, 1, gps_bias(2,:));
    gps_bias(3,:) = filter(coeff, 1, gps_bias(3,:));
    sensors.gps.data = gps_clean + gps_bias;
    sensors.gps.time = t_gps;
    sensors.gps.fs   = config.fs_gps;

    % Generate tof measurements
    tof_clean = interpolate_data(time, tof_dist, t_tof);
    sensors.tof.data = tof_clean + sqrt(config.R_tof) * randn(size(tof_clean));
    sensors.tof.time = t_tof;
    sensors.tof.fs   = config.fs_tof;
end

function interpolated = interpolate_data(original_time, original_data, new_time)
% INTERPOLATE_DATA Interpolate data to new time vector
    interpolated = zeros(size(original_data, 1), length(new_time));
    for i = 1:size(original_data, 1)
        interpolated(i, :) = interp1(original_time, original_data(i, :), new_time, 'linear', 'extrap');
    end
end

function truth = package_truth_data(time, quat, pos, vel, acc, ang_vel, body_acc, accel_bias, gyro_bias, baro_bias, gps_bias)
% PACKAGE_TRUTH_DATA Package ground truth data into structure
    truth.time = time;
    truth.quat = quat;
    truth.pos  = pos;
    truth.vel  = vel;
    truth.acc = acc;
    truth.ang_vel = ang_vel;
    truth.acc_b = body_acc;
    truth.accel_bias = accel_bias;
    truth.gyro_bias = gyro_bias;
    truth.baro_bias = baro_bias;
    truth.gps_bias = gps_bias;
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
    norma = zeros(1, length(truth.time));
    for i=1:length(truth.time)
        norma(1,i) = norm(truth.acc(:,i));
    end
    plot(truth.time, [truth.acc;norma]'); grid on;
    xlabel('Time [s]'); ylabel('[m/s^2]');
    title('Drone Ground-Truth Acceleration');
    legend('North', 'East', 'Down');

    % Body frame angular velocity plot
    figure('Name', 'Angular Velocity', 'Position', [300 300 800 400]);
    plot(truth.time, truth.ang_vel'); grid on;
    xlabel('Time [s]'); ylabel('[rad/s]');
    title('Drone Body Frame Angular Velocity');
    legend('wx', 'wy', 'wz');
    
    % Accelerometer bias plot
    figure('Name', 'Accel Bias', 'Position', [300 300 800 400]);
    stairs(sensors.accel.time, truth.accel_bias'); grid on;
    xlabel('Time [s]'); ylabel('b_a [m/s^2]'); 
    title('Accel Drift Bias');
    legend('X', 'Y', 'Z');
    
    % Accelerometer measurements plot
    figure('Name', 'Accelerometer Measurements', 'Position', [300 300 800 400]);
    stairs(sensors.accel.time, sensors.accel.data'); grid on;
    xlabel('Time [s]'); ylabel('[m/s^2]');
    title('Accelerometer Measurements');
    legend('a_x', 'a_y', 'a_z');

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

    % Magnetometer measurements plot
    figure('Name', 'Magnetometer Measurements', 'Position', [300 300 800 400]);
    stairs(sensors.mag.time, sensors.mag.data'); grid on;
    xlabel('Time [s]'); ylabel('Normalized Units [-]');
    title('Magnetometer Measurements');
    legend('m_x', 'm_y', 'm_z');

    % Baro bias plot
    figure('Name', 'Baro Bias', 'Position', [300 300 800 400]);
    stairs(sensors.baro.time, truth.baro_bias); grid on;
    xlabel('Time [s]'); ylabel('b_b [m]'); 
    title('Baro Drift Bias');
    
    % Baro measurements plot
    figure('Name', 'Barometer Measurements', 'Position', [300 300 800 400]);
    stairs(sensors.baro.time, sensors.baro.data); grid on;
    xlabel('Time [s]'); ylabel('Down [m]');
    title('Barometer Measurements');
   
    % GPS bias plot
    figure('Name', 'GPS Bias', 'Position', [300 300 800 400]);
    stairs(sensors.gps.time, truth.gps_bias'); grid on;
    xlabel('Time [s]'); ylabel('b_{gps} [m]'); 
    title('GPS Drift Bias');
    legend('North', 'East', 'Down');
    
    % GPS measurements plot
    figure('Name', 'GPS Measurements', 'Position', [300 300 800 400]);
    stairs(sensors.gps.time, sensors.gps.data'); grid on;
    xlabel('Time [s]'); ylabel('[m]');
    title('GPS Measurements');
    legend('North', 'East', 'Down');

    % TOF measurements plot
    figure('Name', 'TOF Measurements', 'Position', [300 300 800 400]);
    stairs(sensors.tof.time, sensors.tof.data); grid on;
    xlabel('Time [s]'); ylabel('Distance [m]');
    title('TOF Measurements');
end