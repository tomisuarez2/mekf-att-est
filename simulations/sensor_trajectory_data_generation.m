% ================ Initialization ================

clear; close all; clc;

addpath(genpath('../src'));

%% ================ Configuration Parameters ================

% Sampling frequencies [Hz]
CONFIG.fs_acc  = 100;
CONFIG.fs_gyro = 500;
CONFIG.fs_mag  = 50;
CONFIG.fs_baro = 50;
CONFIG.fs_gnss  = 2;
CONFIG.fs_tof  = 50;

% Sensor noise characteristics, based on real cheap sensors.
CONFIG.R_acc   = 2.1e-03*eye(3)*CONFIG.fs_acc/100; % [m^2/s^4] (Due to White Gaussian Noise assumption, we assume a constant PSD matrix, the value was characterized for 100 Hz)
CONFIG.Q_ba    = 1.00e-10*eye(3); % [m^2/s^5]
CONFIG.R_gyro  = 4.93e-06*eye(3)*CONFIG.fs_gyro/100; % [rad^2/s^2] (Due to White Gaussian Noise assumption, we assume a constant PSD matrix, the value was characterized for 100 Hz)
CONFIG.Q_bg    = 2.28e-11*eye(3); % [rad^2/s^3] 
CONFIG.R_mag   = 1.90e-02*eye(3)*CONFIG.fs_mag/50; % [-] (Due to White Gaussian Noise assumption, we assume a constant PSD matrix, the value was characterized for 100 Hz)
CONFIG.R_baro  = 2.3e-04*CONFIG.fs_baro/80; % [m^2] (Due to White Gaussian Noise assumption, we assume a constant PSD matrix, the value was characterized for 100 Hz)
CONFIG.Q_bb    = 1.29e-03; % [m^2/s]
CONFIG.Q_bgnss = diag([1.74, 1.03, 1.99]); % [m^2/s]
CONFIG.R_gnss  = [1.071 0  0; 0  1.902 0; 0 0  3.404];  % [m^2]
CONFIG.R_tof   = 4.84e-6*CONFIG.fs_tof/50; % [m^2] (Due to White Gaussian Noise assumption, we assume a constant PSD matrix, the value was characterized for 100 Hz)

% Earth constants
CONFIG.g_ned  = [0; 0; 9.80665];  % gravity vector in NED [m/s^2]
CONFIG.m_ned  = [18400; -551; -13300];  % [nT]
CONFIG.m_ned  = CONFIG.m_ned / norm(CONFIG.m_ned);  % normalized magnetic field

%% ================ Load and Process State Data from Simulink Model ================

try
    load('simulation_state_data.mat');
catch ME
    error('Failed to load state data data: %s', ME.message);
end

% Time data [s]
time = state_data.q.time;
n_truth_samples = length(time);

% Position data [m] (NED frame: x=N, y=E, z=D)
pos = extract_trajectory_data(state_data.posE.data);

% Velocity truth velocity data [m/s]
vel = extract_trajectory_data(state_data.velE.data);

% Quaternion data (Scalar first)
quat = squeeze(state_data.q.data);
if size(quat, 1) ~= 4
    quat = quat';  % Ensure quaternions are 4×N
end

% Ground truth acceleration data [m/s^2]
acc = extract_trajectory_data(state_data.acelE.data);

% Body frame angular velocity data [rad/s] 
omega_b = extract_trajectory_data(state_data.omegaB.data);

%% ================ Compute Sensor Measurements ================

[mag_truth, sp_force_truth, tof_truth] = compute_sensor_measurements(acc, quat, pos, CONFIG);

%% ================ Generate Noisy Sensor Data ================
                                                       
[sensors, accel_bias, gyro_bias, baro_bias] = generate_sensor_measurements(time, omega_b, sp_force_truth, mag_truth, tof_truth, pos, CONFIG);

%% ================ Package Truth Data ================

truth = package_truth_data(time, quat, pos, vel, acc, omega_b, accel_bias, gyro_bias, baro_bias);

%% ================ Save Data ================

save_generated_data(truth, sensors, CONFIG);

%% ================ Generate Plots ================

generate_validation_plots(truth, sensors);

fprintf('>>> Trajectory and simulated sensor measurements have been generated.\n');

rmpath(genpath('../src'));

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

function [mag_truth, sp_force_truth, tof_truth] = compute_sensor_measurements(acc, quat, pos, config)
% COMPUTE_SENSOR_MEASUREMENTS Compute ideal sensor measurements
    n_points = size(quat, 2);
    mag_truth = zeros(3, n_points);
    sp_force_truth = zeros(3, n_points);
    tof_truth = zeros(1, n_points);

    for k = 1:n_points
        % Specific force = acceleration - gravity 
        R_be = quat_to_dcm(quat(:, k));
        sp_force_truth(:, k) = R_be*(acc(:, k) - config.g_ned);
        % Magnetic field in body frame
        mag_truth(:, k) = R_be * config.m_ned;
        % Lidar measured distance
        tof_truth(k) = -R_be(3,3) * pos(3,k); % We are assuming we know at any time the position of the floor.
    end
end

function [sensors, accel_bias, gyro_bias, baro_bias] = generate_sensor_measurements(time, omega_b, sp_force, mag, tof_dist, pos, config)
% GENERATE_SENSOR_MEASUREMENTS Generate noisy sensor measurements at different rates
    
    % Create time vectors for each sensor
    t_acc  = 0:(1/config.fs_acc):time(end);
    t_gyro = 0:(1/config.fs_gyro):time(end);
    t_mag  = 0:(1/config.fs_mag):time(end);
    t_baro = 0:(1/config.fs_baro):time(end);
    t_gnss  = 0:(1/config.fs_gnss):time(end);
    t_tof  = 0:(1/config.fs_tof):time(end);
    
    % Initialize sensor structures
    sensors = struct();
      
    % Generate accelerometer measurements
    accel_clean = interpolate_data(time, sp_force, t_acc);
    accel_bias = cumsum(chol(config.Q_ba/config.fs_acc) * randn(3, length(t_acc)), 2);
    sensors.accel.data = accel_clean + chol(config.R_acc) * randn(size(accel_clean)) + accel_bias;
    sensors.accel.time = t_acc;
    sensors.accel.fs   = config.fs_acc;

    % Generate gyroscope measurements
    gyro_clean = interpolate_data(time, omega_b, t_gyro);
    gyro_bias = cumsum(chol(config.Q_bg/config.fs_gyro) * randn(3, length(t_gyro)), 2);
    sensors.gyro.data = gyro_clean + chol(config.R_gyro) * randn(size(gyro_clean)) + gyro_bias;
    sensors.gyro.time = t_gyro;
    sensors.gyro.fs   = config.fs_gyro;
    
    % Generate magnetometer measurements
    mag_clean = interpolate_data(time, mag, t_mag);
    sensors.mag.data = mag_clean + chol(config.R_mag) * randn(size(mag_clean));
    sensors.mag.time = t_mag;
    sensors.mag.fs   = config.fs_mag;

    % Generate barometer measurements
    baro_clean = interpolate_data(time, -pos(3,:), t_baro);
    baro_bias = cumsum(sqrt(config.Q_bb/config.fs_baro) * randn(1, length(t_baro)), 2);
    sensors.baro.data = baro_clean + sqrt(config.R_baro) * randn(size(baro_clean)) + baro_bias;
    sensors.baro.time = t_baro;
    sensors.baro.fs   = config.fs_baro;
    
    % Generate gnss measurements
    gnss_clean = interpolate_data(time, pos, t_gnss);
    gnss_bias = cumsum(sqrt(config.Q_bgnss/config.fs_gnss) * randn(3, length(t_gnss)), 2);
    coeff= ones(1, 15)/15;
    gnss_bias(1,:) = filter(coeff, 1, gnss_bias(1,:));
    gnss_bias(2,:) = filter(coeff, 1, gnss_bias(2,:));
    gnss_bias(3,:) = filter(coeff, 1, gnss_bias(3,:));
    sensors.gnss.data = gnss_clean + gnss_bias;
    sensors.gnss.time = t_gnss;
    sensors.gnss.fs   = config.fs_gnss;

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

function truth = package_truth_data(time, quat, pos, vel, acc, ang_vel, accel_bias, gyro_bias, baro_bias)
% PACKAGE_TRUTH_DATA Package ground truth data into structure
    truth.time = time;
    truth.quat = quat;
    truth.pos  = pos;
    truth.vel  = vel;
    truth.acc = acc;
    truth.ang_vel = ang_vel;
    truth.accel_bias = accel_bias;
    truth.gyro_bias = gyro_bias;
    truth.baro_bias = baro_bias;
end

function save_generated_data(truth, sensors, config)
% SAVE_GENERATED_DATA Save generated data to files
    dt = datetime('now');
    timestamp = datestr(dt, 'yyyy_mm_dd-HH_MM_SS');
    
    % Save main data file
    filename = sprintf('../src/sensor and real data/truth_sensor_data_%s.mat', timestamp);
    save(filename, 'truth', 'sensors', 'config', '-v7.3');
    fprintf('>>> Data saved to: %s\n', filename);
end

function generate_validation_plots(truth, sensors)
% GENERATE_VALIDATION_PLOTS Generate validation plots

    set(groot, 'defaultAxesFontName', 'Times New Roman');
    set(groot, 'defaultAxesFontSize', 13);
    set(groot, 'defaultAxesLineWidth', 1.0);
    set(groot, 'defaultAxesGridAlpha', 0.3);
    set(groot, 'defaultLineLineWidth', 1.6);
    set(groot, 'defaultFigureColor', 'w');
    set(groot, 'defaultLegendBox', 'off');
    set(groot, 'defaultLegendFontSize', 12);
    set(groot, 'defaultTextInterpreter', 'latex');
    set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
    set(groot, 'defaultLegendInterpreter', 'latex');
    set(groot, 'defaultAxesTitleFontWeight', 'normal');
    set(groot, 'defaultAxesTitleFontSizeMultiplier', 1.1);
    set(groot, 'defaultFigureUnits', 'normalized', ...
               'defaultFigurePosition', [0.2, 0.2, 0.6, 0.6]);

    % Plot parameters
    lw_default = 2.5; 
    lw_auxiliary = 1.5; 
    label_fontsize = 20; 
    title_fontsize = 34;
    ticks_fontsize = 16; % FontSize: Label Ticks

    % Quaternion plot
    q_components = {'$q_0$', '$q_1$', '$q_2$', '$q_3$'};
    
    for i=1:4
        figure;
        ax_main = axes;
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        plot(truth.time, truth.quat(i,:), '-', 'Color', '#77B5FE', 'LineWidth', 3, 'DisplayName', 'Simulation');

        %------------------ Principal axes format ------------------%

        xlabel('Time [s]')
        ylabel(q_components{i})
        title(sprintf('%s component of the orientation quaternion', q_components{i}), ...
            'FontSize', title_fontsize, ...
            'FontName', 'Times New Roman', ...
            'FontWeight', 'bold', ...
            'Interpreter', 'latex', ...
            'Color', 'k');
        ax_main.FontSize = ticks_fontsize;
        ax_main.LineWidth = lw_auxiliary;
    
        %------------------ Legend ------------------%
        lgd = legend('show', 'Location', 'northeast');
        lgd.Box = 'on';
        lgd.Color = 'white';
        lgd.EdgeColor = 'black';
        lgd.LineWidth = 1;
    end

    % Position plot
    pos_components = {'$X^E$ [m]', '$Y^E$ [m]', '$Z^E$ [m]'};
    pos_components_tit = {'$X^E$', '$Y^E$', '$Z^E$'};
    
    for i=1:3
        figure;
        ax_main = axes;
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        plot(truth.time, truth.pos(i,:), '-', 'Color', '#77B5FE', 'LineWidth', 3, 'DisplayName', 'Simulation');

        %------------------ Principal axes format ------------------%

        xlabel('Time [s]')
        ylabel(pos_components{i})
        title(sprintf('%s component of the position vector', pos_components_tit{i}), ...
            'FontSize', title_fontsize, ...
            'FontName', 'Times New Roman', ...
            'FontWeight', 'bold', ...
            'Interpreter', 'latex', ...
            'Color', 'k');
        ax_main.FontSize = ticks_fontsize;
        ax_main.LineWidth = lw_auxiliary;
    
        %------------------ Legend ------------------%
        lgd = legend('show', 'Location', 'northeast');
        lgd.Box = 'on';
        lgd.Color = 'white';
        lgd.EdgeColor = 'black';
        lgd.LineWidth = 1;
    end

    % Velocity plot
    vel_components = {'$\dot{X}^E [m/s]$','$\dot{Y}^E [m/s]$','$\dot{Z}^E [m/s]$'};
    vel_components_tit = {'$\dot{X}^E$','$\dot{Y}^E$','$\dot{Z}^E$'};
    
    for i=1:3
        figure;
        ax_main = axes;
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        plot(truth.time, truth.vel(i,:), '-', 'Color', '#77B5FE', 'LineWidth', 3, 'DisplayName', 'Simulation');

        %------------------ Principal axes format ------------------%

        xlabel('Time [s]')
        ylabel(vel_components{i})
        title(sprintf('%s component of the velocity vector', vel_components_tit{i}), ...
            'FontSize', title_fontsize, ...
            'FontName', 'Times New Roman', ...
            'FontWeight', 'bold', ...
            'Interpreter', 'latex', ...
            'Color', 'k');
        ax_main.FontSize = ticks_fontsize;
        ax_main.LineWidth = lw_auxiliary;
    
        %------------------ Legend ------------------%
        lgd = legend('show', 'Location', 'northeast');
        lgd.Box = 'on';
        lgd.Color = 'white';
        lgd.EdgeColor = 'black';
        lgd.LineWidth = 1;
    end

    % Acceleration plot
    acc_components = {'$\ddot{X}^E [m/s^2]$','$\ddot{Y}^E [m/s^2]$','$\ddot{Z}^E [m/s^2]$'};
    acc_components_tit = {'$\ddot{X}^E$','$\ddot{Y}^E$','$\ddot{Z}^E$'};

    for i=1:3
        figure;
        ax_main = axes;
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        plot(truth.time, truth.acc(i,:), '-', 'Color', '#77B5FE', 'LineWidth', 3, 'DisplayName', 'Simulation');

        %------------------ Principal axes format ------------------%

        xlabel('Time [s]')
        ylabel(acc_components{i})
        title(sprintf('%s component of the acceleration vector', acc_components_tit{i}), ...
            'FontSize', title_fontsize, ...
            'FontName', 'Times New Roman', ...
            'FontWeight', 'bold', ...
            'Interpreter', 'latex', ...
            'Color', 'k');
        ax_main.FontSize = ticks_fontsize;
        ax_main.LineWidth = lw_auxiliary;
    
        %------------------ Legend ------------------%
        lgd = legend('show', 'Location', 'northeast');
        lgd.Box = 'on';
        lgd.Color = 'white';
        lgd.EdgeColor = 'black';
        lgd.LineWidth = 1;
    end

    % Body frame angular velocity plot
    wB_components = {'${\omega^B}_1 [rad/s]$', '${\omega^B}_2 [rad/s]$', '${\omega^B}_3 [rad/s]$'};
    wB_components_tit = {'${\omega^B}_1$', '${\omega^B}_2$', '${\omega^B}_3$'};
    
    for i=1:3
        figure;
        ax_main = axes;
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        plot(truth.time, truth.ang_vel(i,:), '-', 'Color', '#77B5FE', 'LineWidth', 3, 'DisplayName', 'Simulation');

        %------------------ Principal axes format ------------------%

        xlabel('Time [s]')
        ylabel(wB_components{i})
        title(sprintf('%s component of the angular velocity vector', wB_components_tit{i}), ...
            'FontSize', title_fontsize, ...
            'FontName', 'Times New Roman', ...
            'FontWeight', 'bold', ...
            'Interpreter', 'latex', ...
            'Color', 'k');
        ax_main.FontSize = ticks_fontsize;
        ax_main.LineWidth = lw_auxiliary;
    
        %------------------ Legend ------------------%
        lgd = legend('show', 'Location', 'northeast');
        lgd.Box = 'on';
        lgd.Color = 'white';
        lgd.EdgeColor = 'black';
        lgd.LineWidth = 1;
    end
    
    % Accelerometer bias plot
    accBb_components = {'$b_{a1} [m/s^2]$','$b_{a2} [m/s^2]$','$b_{a3} [m/s^2]$'};
    accBb_components_tit = {'$b_{a1}$','$b_{a2}$','$b_{a3}$'};

    for i=1:3
        figure;
        ax_main = axes;
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        plot(sensors.accel.time, truth.accel_bias(i,:), '-', 'Color', '#77B5FE', 'LineWidth', 3, 'DisplayName', 'Simulation');

        %------------------ Principal axes format ------------------%

        xlabel('Time [s]')
        ylabel(accBb_components{i})
        title(sprintf('%s component of the accelerometer bias', accBb_components_tit{i}), ...
            'FontSize', title_fontsize, ...
            'FontName', 'Times New Roman', ...
            'FontWeight', 'bold', ...
            'Interpreter', 'latex', ...
            'Color', 'k');
        ax_main.FontSize = ticks_fontsize;
        ax_main.LineWidth = lw_auxiliary;
    
        %------------------ Legend ------------------%
        lgd = legend('show', 'Location', 'northeast');
        lgd.Box = 'on';
        lgd.Color = 'white';
        lgd.EdgeColor = 'black';
        lgd.LineWidth = 1;
    end
    
    % Accelerometer measurements plot
    accB_components = {'$a^B_{m1} [m/s^2]$','$a^B_{m2} [m/s^2]$','$a^B_{m3} [m/s^2]$'};
    accB_components_tit  = {'$a^B_{m1}$','$a^B_{m2}$','$a^B_{m3}$'};

    for i=1:3
        figure;
        ax_main = axes;
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        stairs(sensors.accel.time, sensors.accel.data(i,:), '-', 'Color', '#77B5FE', 'LineWidth', 3, 'DisplayName', 'Simulated sensor');

        %------------------ Principal axes format ------------------%

        xlabel('Time [s]')
        ylabel(accB_components{i})
        title(sprintf('%s component of the accelerometer measurement', accB_components_tit{i}), ...
            'FontSize', title_fontsize, ...
            'FontName', 'Times New Roman', ...
            'FontWeight', 'bold', ...
            'Interpreter', 'latex', ...
            'Color', 'k');
        ax_main.FontSize = ticks_fontsize;
        ax_main.LineWidth = lw_auxiliary;
    
        %------------------ Legend ------------------%
        lgd = legend('show', 'Location', 'northeast');
        lgd.Box = 'on';
        lgd.Color = 'white';
        lgd.EdgeColor = 'black';
        lgd.LineWidth = 1;
    end

    % Gyro bias plot
    gyrob_components = {'$b_{g1} [rad/s]$','$b_{g2} [rad/s]$','$b_{g3} [rad/s]$'};
    gyrob_components_tit = {'$b_{g1}$','$b_{g2}$','$b_{g3}$'};

    for i=1:3
        figure;
        ax_main = axes;
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        plot(sensors.gyro.time, truth.gyro_bias(i,:), '-', 'Color', '#77B5FE', 'LineWidth', 3, 'DisplayName', 'Simulation');

        %------------------ Principal axes format ------------------%

        xlabel('Time [s]')
        ylabel(gyrob_components{i})
        title(sprintf('%s component of the gyroscope bias', gyrob_components_tit{i}), ...
            'FontSize', title_fontsize, ...
            'FontName', 'Times New Roman', ...
            'FontWeight', 'bold', ...
            'Interpreter', 'latex', ...
            'Color', 'k');
        ax_main.FontSize = ticks_fontsize;
        ax_main.LineWidth = lw_auxiliary;
    
        %------------------ Legend ------------------%
        lgd = legend('show', 'Location', 'northeast');
        lgd.Box = 'on';
        lgd.Color = 'white';
        lgd.EdgeColor = 'black';
        lgd.LineWidth = 1;
    end
    
    % Gyroscope measurements plot
    gyro_components = {'$\omega^B_{m1} [rad/s]$','$\omega^B_{m2} [rad/s]$','$\omega^B_{m3} [rad/s]$'};
    gyro_components_tit  = {'$\omega^B_{m1}$','$\omega^B_{m2}$','$\omega^B_{m3}$'};

    for i=1:3
        figure;
        ax_main = axes;
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        stairs(sensors.gyro.time, sensors.gyro.data(i,:), '-', 'Color', '#77B5FE', 'LineWidth', 3, 'DisplayName', 'Simulated sensor');

        %------------------ Principal axes format ------------------%

        xlabel('Time [s]')
        ylabel(gyro_components{i})
        title(sprintf('%s component of the gyroscope measurement', gyro_components_tit{i}), ...
            'FontSize', title_fontsize, ...
            'FontName', 'Times New Roman', ...
            'FontWeight', 'bold', ...
            'Interpreter', 'latex', ...
            'Color', 'k');
        ax_main.FontSize = ticks_fontsize;
        ax_main.LineWidth = lw_auxiliary;
    
        %------------------ Legend ------------------%
        lgd = legend('show', 'Location', 'northeast');
        lgd.Box = 'on';
        lgd.Color = 'white';
        lgd.EdgeColor = 'black';
        lgd.LineWidth = 1;
    end

    % Magnetometer measurements plot
    mag_components = {'$m^B_{m1} [-]$','$m^B_{m2} [-]$','$m^B_{m3} [-]$'};
    mag_components_tit  = {'$m^B_{m1}$','$m^B_{m2}$','$m^B_{m3}$'};

    for i=1:3
        figure;
        ax_main = axes;
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        stairs(sensors.mag.time, sensors.mag.data(i,:), '-', 'Color', '#77B5FE', 'LineWidth', 3, 'DisplayName', 'Simulated sensor');

        %------------------ Principal axes format ------------------%

        xlabel('Time [s]')
        ylabel(mag_components{i})
        title(sprintf('%s component of the magnetometer measurement', mag_components_tit{i}), ...
            'FontSize', title_fontsize, ...
            'FontName', 'Times New Roman', ...
            'FontWeight', 'bold', ...
            'Interpreter', 'latex', ...
            'Color', 'k');
        ax_main.FontSize = ticks_fontsize;
        ax_main.LineWidth = lw_auxiliary;
    
        %------------------ Legend ------------------%
        lgd = legend('show', 'Location', 'northeast');
        lgd.Box = 'on';
        lgd.Color = 'white';
        lgd.EdgeColor = 'black';
        lgd.LineWidth = 1;
    end

    % Barometer bias
    bb_components = {'$b_{b} [m]$'};

    for i=1:1
        figure;
        ax_main = axes;
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        plot(sensors.baro.time, truth.baro_bias(i,:), '-', 'Color', '#77B5FE', 'LineWidth', 3, 'DisplayName', 'Simulation');

        %------------------ Principal axes format ------------------%

        xlabel('Time [s]')
        ylabel(bb_components{i})
        title("Barometer bias", ...
            'FontSize', title_fontsize, ...
            'FontName', 'Times New Roman', ...
            'FontWeight', 'bold', ...
            'Interpreter', 'latex', ...
            'Color', 'k');
        ax_main.FontSize = ticks_fontsize;
        ax_main.LineWidth = lw_auxiliary;
    
        %------------------ Legend ------------------%
        lgd = legend('show', 'Location', 'northeast');
        lgd.Box = 'on';
        lgd.Color = 'white';
        lgd.EdgeColor = 'black';
        lgd.LineWidth = 1;
    end
    
    % Barometer measurements plot
    baro_components = {'$b^B_{m} [m]$'};

    for i=1:1
        figure;
        ax_main = axes;
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        stairs(sensors.baro.time, sensors.baro.data(i,:), '-', 'Color', '#77B5FE', 'LineWidth', 3, 'DisplayName', 'Simulated sensor');

        %------------------ Principal axes format ------------------%

        xlabel('Time [s]')
        ylabel(baro_components{i})
        title("Barometer measurement", ...
            'FontSize', title_fontsize, ...
            'FontName', 'Times New Roman', ...
            'FontWeight', 'bold', ...
            'Interpreter', 'latex', ...
            'Color', 'k');
        ax_main.FontSize = ticks_fontsize;
        ax_main.LineWidth = lw_auxiliary;
    
        %------------------ Legend ------------------%
        lgd = legend('show', 'Location', 'northeast');
        lgd.Box = 'on';
        lgd.Color = 'white';
        lgd.EdgeColor = 'black';
        lgd.LineWidth = 1;
    end
    
    % GNSS measurements plot
    gnss_components = {'$GNSS_{X} [m]$','$GNSS_{Y} [m]$','$GNSS_{Z} [m]$'};
    gnss_components_tit = {'$GNSS_{X}$','$GNSS_{Y}$','$GNSS_{Z}$'};

    for i=1:3
        figure;
        ax_main = axes;
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        stairs(sensors.gnss.time, sensors.gnss.data(i,:), '-', 'Color', '#77B5FE', 'LineWidth', 3, 'DisplayName', 'Simulated sensor');

        %------------------ Principal axes format ------------------%

        xlabel('Time [s]')
        ylabel(gnss_components{i})
        title(sprintf('%s component of the GNSS measurement', gnss_components_tit{i}), ...
            'FontSize', title_fontsize, ...
            'FontName', 'Times New Roman', ...
            'FontWeight', 'bold', ...
            'Interpreter', 'latex', ...
            'Color', 'k');
        ax_main.FontSize = ticks_fontsize;
        ax_main.LineWidth = lw_auxiliary;
    
        %------------------ Legend ------------------%
        lgd = legend('show', 'Location', 'northeast');
        lgd.Box = 'on';
        lgd.Color = 'white';
        lgd.EdgeColor = 'black';
        lgd.LineWidth = 1;
    end

    % TOF measurement
    tof_components = {'$d^B_{m} [m]$'};

    for i=1:1
        figure;
        ax_main = axes;
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        stairs(sensors.tof.time, sensors.tof.data(i,:), '-', 'Color', '#77B5FE', 'LineWidth', 3, 'DisplayName', 'Simulated sensor');

        %------------------ Principal axes format ------------------%

        xlabel('Time [s]')
        ylabel(tof_components{i})
        title("TOF measurement", ...
            'FontSize', title_fontsize, ...
            'FontName', 'Times New Roman', ...
            'FontWeight', 'bold', ...
            'Interpreter', 'latex', ...
            'Color', 'k');
        ax_main.FontSize = ticks_fontsize;
        ax_main.LineWidth = lw_auxiliary;
    
        %------------------ Legend ------------------%
        lgd = legend('show', 'Location', 'northeast');
        lgd.Box = 'on';
        lgd.Color = 'white';
        lgd.EdgeColor = 'black';
        lgd.LineWidth = 1;
    end
end