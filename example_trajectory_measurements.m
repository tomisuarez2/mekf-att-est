clear; close all; clc;

%=====================================================
% ------ Example trajectory from simulink model ------
%=====================================================

X = load('state_trajectory.mat'); % State vector (Complete)

% Time
t = X.ans.q.Time;

% Position (NED frame: x=N, y=E, z=D)
x(:) = X.ans.posE.data(1,:,:);
y(:) = X.ans.posE.data(2,:,:);
z(:) = X.ans.posE.data(3,:,:);
p = [x; y; z];

% Velocity in Earth frame (derivative of p)
vx(:) = X.ans.velE.data(1,:,:);
vy(:) = X.ans.velE.data(2,:,:);
vz(:) = X.ans.velE.data(3,:,:);
v = [vx; vy; vz];

% Acceleration in Earth frame (derivative of v)
ax(:) = X.ans.acelE.data(1,:,:);
ay(:) = X.ans.acelE.data(2,:,:);
az(:) = X.ans.acelE.data(3,:,:);
a = [ax; ay; az];

% Acceleration in Boody frame 
aBx(:) = X.ans.acelB.data(1,:,:);
aBy(:) = X.ans.acelB.data(2,:,:);
aBz(:) = X.ans.acelB.data(3,:,:);
aB = [aBx; aBy; aBz];

% Angular velocity in Boody frame 
omegaBx(:) = X.ans.omegaB.data(1,:,:);
omegaBy(:) = X.ans.omegaB.data(2,:,:);
omegaBz(:) = X.ans.omegaB.data(3,:,:);
omegaB = [omegaBx; omegaBy; omegaBz];

% Attitude quaternions
q(1:4,:) = X.ans.q.data(:,:,:);

% Rotation matrices Body to Earth (R_be)
R_be = X.ans.DCMbe.data(:,:,:);

%==============================
% ------ Earth constants ------
%==============================

% Earth constants
g = [0;0;9.81];                      % gravity vector in NED (Down positive) [m/s^2]
m_ned = [18823.7; -584.3; -13244.4]; % [nT]
m_ned = m_ned / norm(m_ned);         % example normalized magnetic field according to LLA
magB     = zeros(3,length(t));
sp_force = zeros(3,length(t));
for k = 1:length(t)
    % Specific force in body
    f_e = a(:,k) + g;                     % acceleration + gravity
    sp_force(:,k) = R_be(:,:,k)' * f_e;    % transform to body frame
    % Magnetometer in body frame
    magB(:,k) = R_be(:,:,k)' * m_ned;
end

%========================================
% ------ Noisy IMU data generation ------
%========================================

% Noise measurement covariances
R_gyro  = diag([2.830380375642008306e-06, 4.934009191301758550e-06, 3.320219599119017912e-06]);
R_accel = diag([1.090681860817302234e-03, 9.279916753344519612e-04, 2.146372869350902587e-03]);
R_mag   = 0.1*diag([2.890238578269250028e-03, 1.901404294437207543e-02, 7.715742865748796697e-03]);

% Downsample to sensor-specific rates
fs_gyro = 70;   % Hz 100
fs_acc  = 50;    % Hz 70
fs_mag  = 50;    % Hz 30

% Time vectors
t_gyro = 0:(1/fs_gyro):t(end);
t_acc  = 0:(1/fs_acc):t(end);
t_mag  = 0:(1/fs_mag):t(end);

% Pre-allocate IMU data
gyro = zeros(3, length(t_gyro));
accel = zeros(3, length(t_acc));
mag = zeros(3, length(t_mag));

% IMU data generation
for i=1:3
    gyro(i,:) = interp1(t',omegaB(i,:),t_gyro);
    accel(i,:) = interp1(t',sp_force(i,:),t_acc);
    mag(i,:) = interp1(t',magB(i,:),t_mag);
end

% Add measurement noise
gyro_meas  = gyro + sqrt(R_gyro)*randn(3,length(t_gyro));  % [rad/s]
accel_meas = accel + sqrt(R_accel)*randn(3,length(t_acc)); % [m/s^2]
mag_meas   = mag + sqrt(R_mag)*randn(3,length(t_mag));     % [normalized units]

% Add random walk bias noise
gyro_meas = gyro_meas + cumsum(sqrt((1e-7^2)*eye(3))*randn(3,length(t_gyro)),2);

%===============================================
% ------ Save trajectory and measurements ------
%===============================================

% Save results
truth.time = t;
truth.pos  = p;
truth.vel  = v;
truth.acc  = a;
truth.q    = q';

sensors.gyro.time  = t_gyro;
sensors.gyro.data  = gyro_meas;
sensors.accel.time = t_acc;
sensors.accel.data = accel_meas;
sensors.mag.time   = t_mag;
sensors.mag.data   = mag_meas;

disp('>>> Trajectory and simulated IMU measurements have been generated.')

%% Plot
figure;
plot3(p(1,:),p(2,:),-p(3,:)); grid on; axis equal;
xlabel('North [m]'); ylabel('East [m]'); zlabel('Altitude [m]');
title('Drone Ground-Truth Trajectory');

figure;
plot(t,q); grid on;
xlabel('Time [s]'); ylabel('q_i'); 
title('Attitude Quaternions');

figure;
plot(t_gyro,gyro_meas); grid on;
xlabel('Time [s]'); ylabel('[rad/s]');
title('Gyro measurements');

figure;
plot(t_acc,accel_meas); grid on;
xlabel('Time [s]'); ylabel('[m/s^2]');
legend('ax','ay','az');
title('Accelerometer measurements');

figure;
plot(t_mag,mag_meas); grid on;
xlabel('Time [s]'); ylabel('[-]');
title('Magnetometer measurements');