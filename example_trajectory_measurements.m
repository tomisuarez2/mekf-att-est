%% ================ Initialization ================
clear; close all; clc;

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
t = X.q.Time;

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

%% ================ Earth constants ================

magB  = zeros(3,length(t));          % Measured magnetic field in body frame
sp_force = zeros(3,length(t));       % Measured specific force in body frame
for k = 1:length(t)
    f_e = accel_e(:,k) + CONFIG.g_ned;                     % acceleration + gravity
    sp_force(:,k) = R_be(:,:,k) * f_e;   
    magB(:,k) = R_be(:,:,k) * CONFIG.m_ned;
end

%% ================ Noisy IMU data generation ================

% Time vectors
t_gyro = 0:(1/CONFIG.fs_gyro):t(end);
t_acc  = 0:(1/CONFIG.fs_acc):t(end);
t_mag  = 0:(1/CONFIG.fs_mag):t(end);

% Pre-allocate IMU data
gyro_meas = zeros(3, length(t_gyro));
accel_meas = zeros(3, length(t_acc));
mag_meas = zeros(3, length(t_mag));

% IMU data generation
for i=1:3
    gyro_meas(i,:) = interp1(t',omega_b(i,:),t_gyro);
    accel_meas(i,:) = interp1(t',sp_force(i,:),t_acc);
    mag_meas(i,:) = interp1(t',magB(i,:),t_mag);
end

% Add measurement noise
gyro_meas  = gyro_meas + sqrt(CONFIG.R_gyro)*randn(3,length(t_gyro));  % [rad/s]
accel_meas = accel_meas + sqrt(CONFIG.R_acc)*randn(3,length(t_acc)); % [m/s^2]
mag_meas   = mag_meas + sqrt(CONFIG.R_mag)*randn(3,length(t_mag));     % [normalized units]

% Add random walk bias noise
bg = cumsum(sqrt(CONFIG.Q_bg)*randn(3,length(t_gyro)),2);
gyro_meas = gyro_meas + bg;

%% ================ Save trajectory and measurements ================

truth.time = t;
truth.pos  = pos;
truth.vel  = vel;
truth.acc  = accel_e;
truth.q    = quat;
truth.bg   = bg;

sensors.gyro.time  = t_gyro;
sensors.gyro.data  = gyro_meas;
sensors.accel.time = t_acc;
sensors.accel.data = accel_meas;
sensors.mag.time   = t_mag;
sensors.mag.data   = mag_meas;

disp('>>> Trajectory and simulated IMU measurements have been generated.')

%% ================ Position plot ================
figure;
plot3(pos(1,:),pos(2,:),-pos(3,:)); grid on; axis equal;
xlabel('North [m]'); ylabel('East [m]'); zlabel('Altitude [m]');
title('Drone Ground-Truth Trajectory');
%% ================ Quaternions plot ================
figure;
plot(t,quat); grid on;
xlabel('Time [s]'); ylabel('q_i'); 
title('Attitude Quaternions');
%% ================ Gyro bias plot ================
figure;
plot(t_gyro,bg); grid on;
xlabel('Time [s]'); ylabel('bg_i'); 
title('Gyro drift bias');
%% ================ Gyro measurements plot ================ 
figure;
plot(t_gyro,gyro_meas); grid on;
xlabel('Time [s]'); ylabel('[rad/s]');
title('Gyro measurements');
%% ================ Accel measurements plot ================
figure;
plot(t_acc,accel_meas); grid on;
xlabel('Time [s]'); ylabel('[m/s^2]');
legend('ax','ay','az');
title('Accelerometer measurements');
%% ================ Mag measurements plot ================
figure;
plot(t_mag,mag_meas); grid on;
xlabel('Time [s]'); ylabel('[-]');
title('Magnetometer measurements');

%%
function data = extract_trajectory_data(raw_data)
% EXTRACT_TRAJECTORY_DATA Extract and reshape trajectory data
% Converts from 3×1×N to 3×N format
    data = squeeze(raw_data);
    if size(data, 1) ~= 3
        data = data';
    end
end