%% ================ Initialization ================

clear; close all; clc;

addpath(genpath('../src'));

fprintf('>>> Initializing MEKF algorithm...\n');

%% ================ Load Sensor and Trajectory Data  ================

file_name = 'truth_sensor_data_2026_03_24-18_42_03.mat';
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
Qc_MEKF = blkdiag(config.R_gyro/config.fs_gyro, config.Q_bg);

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

generate_estimation_plots_MEKF(est_MEKF, truth, sensors, config);

% Calculate estimation performance if truth data is available
calculate_estimation_performance_MEKF(est_MEKF.time, est_MEKF.quat, truth.time, truth.quat);

rmpath(genpath('../src'));

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

function generate_estimation_plots_MEKF(est, truth, sensors, config)
% GENERATE_ESTIMATION_PLOTS Generate plots for estimation results

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

    mag_interp_1 = interp1(sensors.mag.time, sensors.mag.data(1,:), sensors.accel.time);
    mag_interp_2 = interp1(sensors.mag.time, sensors.mag.data(2,:), sensors.accel.time);
    mag_interp_3 = interp1(sensors.mag.time, sensors.mag.data(3,:), sensors.accel.time);
    mag_interp = [mag_interp_1;mag_interp_2;mag_interp_3];

    q_TRIAD = TRIAD(sensors.accel.time', sensors.accel.data', mag_interp', config.m_ned, [0 0 1]')';

    for i=1:4
        figure;
        ax_main = axes;
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        stairs(sensors.accel.time, q_TRIAD(i,:), '*', 'Color','#F39C12', 'LineWidth', 1, 'DisplayName', 'TRIAD measurement');
        plot(truth.time, truth.quat(i,:), '-', 'Color', '#77B5FE', 'LineWidth', 3, 'DisplayName', 'Simulation');
        stairs(est.time, est.quat(i,:), 'g--', 'Color','#55CC55', 'LineWidth', 3, 'DisplayName', 'MEKF estimation');

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
    
        %------------------ Leyenda ------------------%
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
        stairs(est.time, est.bg(i,:), 'g--', 'Color','#55CC55', 'LineWidth', 3, 'DisplayName', 'MEKF estimation');

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
    
        %------------------ Leyenda ------------------%
        lgd = legend('show', 'Location', 'northeast');
        lgd.Box = 'on';
        lgd.Color = 'white';
        lgd.EdgeColor = 'black';
        lgd.LineWidth = 1;
    end

end

