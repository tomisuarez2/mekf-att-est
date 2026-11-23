% ================ Initialization ================

clear; close all; clc;

addpath(genpath('../src'));

plots = false;
save = true;

%% ================ Configuration Parameters ================

% Sampling frequencies [Hz]
CONFIG.fs_acc  = 100;
CONFIG.fs_gyro = 100;
CONFIG.fs_mag  = 100;

% Sensor noise characteristics, based on real cheap sensors.
CONFIG.R_acc         = 2.10e-03*eye(3); % [m^2/s^4]   (The value was characterized for 100 Hz)
CONFIG.R_gyro        = 4.93e-06*eye(3); % [rad^2/s^2] (The value was characterized for 100 Hz)
CONFIG.Q_bg          = 2.28e-11*eye(3); % [rad^2/s^3] (The value was characterized for 100 Hz) 
CONFIG.gyro_sys_bias = [0.01; -0.02; 0.03]; % [rad/s] (Example value)
CONFIG.R_mag         = 1.90e-02*eye(3); % [-]         (The value was characterized for 100 Hz)

% Earth constants
CONFIG.g_ned  = [0; 0; 9.80665];                    % gravity vector in NED [m/s^2]
CONFIG.m_ned  = [18400; -551; -13300];              % [nT]
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

% Quaternion data (Scalar first)
quat = squeeze(state_data.q.data);
if size(quat, 1) ~= 4
    quat = quat';  % Ensure quaternions are 4×N
end

% Ground truth acceleration data [m/s^2]
acc = extract_trajectory_data(state_data.acelE.data);

% Body frame angular velocity data [rad/s] 
omega_b = extract_trajectory_data(state_data.omegaB.data);

%% ======================== Generate Sensor Data ========================

[mag_truth, sp_force_truth] = compute_sensor_measurements(acc, quat, CONFIG);
                                                       
[sensors, gyro_bias] = generate_sensor_measurements(time, omega_b, sp_force_truth, mag_truth, CONFIG);

truth = package_truth_data(time, quat, acc, omega_b, gyro_bias);

if save
    save_generated_data(truth, sensors, CONFIG);
end

if plots
    generate_validation_plots(truth, sensors);
end

fprintf('>>> Trajectory and simulated sensor measurements have been generated.\n');