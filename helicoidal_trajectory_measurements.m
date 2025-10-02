clear; close all; clc;

%==========================================
% ------ Ideal trajectory generation ------
%==========================================

% ------ Helicoidal trajectory ------

% Master time buse (ground truth trajectory)
dt = 0.001;               % high-rate step [s] (1 kHz)
T = 60;                   % total duration [s]
t = 0:dt:T;

% Trajectory parameters
R = 5;                    % radius [m]
omega = 0.1;              % angular speed [rad/s]
vz = -0.05;               % climb rate [m/s]

% Position (NED frame: x=N, y=E, z=D)
x = R*cos(omega*t);
y = R*sin(omega*t);
z = vz*t;
p = [x; y; z];

% Velocity in Earth frame (derivative of p)
vx = -R*omega*sin(omega*t);
vy =  R*omega*cos(omega*t);
vz = vz*ones(size(t));
v = [vx; vy; vz];

% Acceleration in Earth frame (derivative of v)
ax = -R*omega^2*cos(omega*t);
ay = -R*omega^2*sin(omega*t);
az = zeros(size(t));
a = [ax; ay; az];

% Desired yaw (heading follows tangent of circle)
yaw = atan2(vy, vx);
pitch = zeros(size(t));
roll  = zeros(size(t));

% Convert Euler to quaternions (Body to Earth)
q = eul2quat([yaw' pitch' roll']); 

% Rotation matrices Earth to Body (R_be)
R_be = zeros(3,3,length(t));
for k = 1:length(t)
    R_be(:,:,k) = quat2rotm(q(k,:));
end

%==============================
% ------ Earth constants ------
%==============================

% Earth constants
g = [0;0;9.81];           % gravity vector in NED (Down positive) [m/s^2]
m_ned = [0.2; 0; -0.5];   % example normalized magnetic field

%========================================
% ------ Noisy IMU data generation ------
%========================================

% Noise measurement covariances
R_gyro  = diag([2.830380375642008306e-06, 4.934009191301758550e-06, 3.320219599119017912e-06]);
R_accel = diag([1.090681860817302234e-03, 9.279916753344519612e-04, 2.146372869350902587e-03]);
R_mag   = diag([2.890238578269250028e-03, 1.901404294437207543e-02, 7.715742865748796697e-03]);

% Preallocate IMU data
gyro  = zeros(3,length(t));
accel = zeros(3,length(t));
mag   = zeros(3,length(t));

% IMU data generation
for k = 1:length(t)
    % Specific force in body
    f_e = a(:,k) + g;                  % acceleration + gravity
    accel(:,k) = R_be(:,:,k) * f_e;    % transform to body frame

    % Gyroscope: angular velocity in body
    % Quaternion kinematics: q_dot = 0.5 * Omega(ω) * q
    if k < length(t)
        dq = (q(k+1,:) - q(k,:))/dt;
        % approximate angular velocity from quaternion derivative
        qk = q(k,:)';
        Omega = [ -qk(2:4)'; ...
                   qk(1)*eye(3) + skew(qk(2:4)) ];
        omega_b = 2 * Omega \ dq';
        gyro(:,k) = omega_b;
    else
        gyro(:,k) = gyro(:,k-1);
    end

    % Magnetometer in body frame
    mag(:,k) = R_be(:,:,k) * m_ned;
end

% Downsample to sensor-specific rates
fs_gyro = 100;   % Hz
fs_acc  = 50;    % Hz
fs_mag  = 10;    % Hz

idx_gyro = 1:(1/(dt*fs_gyro)):length(t);
idx_acc  = 1:(1/(dt*fs_acc)):length(t);
idx_mag  = 1:(1/(dt*fs_mag)):length(t);

% Time vectors
t_gyro = t(round(idx_gyro));
t_acc  = t(round(idx_acc));
t_mag  = t(round(idx_mag));

% Add measurement noise
gyro_meas  = gyro(:,round(idx_gyro)) + sqrt(R_gyro)*randn(3,length(t_gyro));  % [rad/s]
accel_meas = accel(:,round(idx_acc)) + sqrt(R_accel)*randn(3,length(t_acc));  % [m/s^2]
mag_meas   = mag(:,round(idx_mag)) + sqrt(R_mag)*randn(3,length(t_mag));      % [normalized units]

% Add random walk bias noise
gyro_meas = gyro_meas + cumsum(sqrt((1e-5^2)*eye(3))*randn(3,length(t_gyro)),2);

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

%% Example plot
figure;
plot3(p(1,:),p(2,:),p(3,:)); grid on; axis equal;
xlabel('North [m]'); ylabel('East [m]'); zlabel('Altitude [m]');
title('Drone Ground-Truth Trajectory');
