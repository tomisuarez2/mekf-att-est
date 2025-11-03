%=============================
% ------ MEKF algorithm ------
%=============================

% Preallocate estimatation results
est.q = zeros(4,length(sensors.gyro.time));
est.bg = zeros(3,length(sensors.gyro.time));
est.time = sensors.gyro.time;

% Initialize variables
q_nom = [1 0 0 0]';        % Quaternion current estimation
dq0   = [0 0 0]';          % Quaternion orientation error current estimation
bg    = [0 0 0]';          % Gyro bias current estimation
dbg0  = zeros(3,1);        % Gyro bias error current estimation
dx    = [dq0; dbg0];       % Error state current estimation
P = diag([(1e-2)^2,(1e-2)^2,(1e-2)^2,(1e-3)^2,(1e-3)^2,(1e-3)^2]);   % Error-state covariance

% Process covariance matrix
Q = blkdiag(R_gyro, n_bg); 

% Sensor time indices
i_acc = 1; 
i_mag = 1;

for k = 1:length(sensors.gyro.time) % Simulates new gyro available calibrated measurement
    % --- PROPAGATION ---
    if k == 1
        dt = 0.01;
    else
        dt = sensors.gyro.time(k) - sensors.gyro.time(k-1);
    end
    est_omega_B = sensors.gyro.data(:,k) - bg;

    % q_nom propagation
    Omega = [0 -est_omega_B'; est_omega_B -skew(est_omega_B)];
    Theta = eye(4) + 0.5*Omega*dt;
    q_nom = Theta * q_nom;
    
    % Transition matrix and Lambda matrix
    Phi = eye(6);
    Phi(1:3,1:3) = Phi(1:3,1:3) - skew(est_omega_B)*dt;
    Phi(1:3,4:6) = -0.5*eye(3)*dt;
    Lambda = blkdiag(-0.5*eye(3), eye(3))*dt; 

    % Error state propagation
    dx = Phi*dx;
 
    % Covariance matrix propagation
    P = Phi * P * Phi' + Lambda * Q * Lambda';
    
    % --- UPDATE --- 
    acc_available = (i_acc <= length(sensors.accel.time)) && ...
                    (abs(sensors.gyro.time(k) - sensors.accel.time(i_acc)) < 1e-4);

    mag_available = (i_mag <= length(sensors.mag.time)) && ...
                    (abs(sensors.gyro.time(k) - sensors.mag.time(i_mag)) < 1e-4);
    
    if acc_available || mag_available
        H_all = [];
        z_all = [];
        R_all = [];
    
        % Accelerometer update (gravity reference)
        if acc_available
            y = sensors.accel.data(:,i_acc); % Measured specific force
            y = y/norm(y);
            est_vB = ep_to_dcm(q_nom) * [0;0;1]; % Expected gravity direction
            h = est_vB;
            z = y - h; 
    
            H_acc = [2*skew(est_vB) zeros(3,3)];
            
            H_all = [H_all; H_acc];
            z_all = [z_all; z];
            R_all = blkdiag(R_all, R_acc);
    
            i_acc = i_acc + 1;
        end
    
        % Magnetometer update (magnetic north reference)
        if mag_available
            y = sensors.mag.data(:,i_mag); % Measured magnetic field
            y = y/norm(y);
            est_vB = ep_to_dcm(q_nom) * m_ned; % Expected magnetic field direction
            h = est_vB;
            z = y - h;
    
            H_mag = [2*skew(est_vB) zeros(3,3)];
            
            H_all = [H_all; H_mag];
            z_all = [z_all; z];
            R_all = blkdiag(R_all, R_mag);
    
            i_mag = i_mag + 1;
        end
    
        % Error state update
        S = H_all*P*H_all' + R_all;
        K = P*H_all'/S;
        dx = dx + K*z_all;

        % Error state covarianza update
        P = (eye(6)-K*H_all)*P;
    
        % Nominal quaternion propagation
        q_nom = quat_prod([1;dx(1:3)],q_nom);
        
        % Estimated bias gyro update
        bg = bg + dx(4:6);  
        dx = zeros(6,1);

    end
    est.q(:,k) = q_nom;
    est.bg(:,k) = bg;
end
%% ================ Estimated quaternions plot ================
figure;
stairs(est.time,est.q'); grid on;
xlabel('Time [s]'); ylabel('q_i'); 
title('Estimated Attitude Quaternions');

%% ================ Estimateed gyro bias plot ================
figure;
stairs(est.time,est.bg'); grid on;
xlabel('Time [s]'); ylabel('bg_i');
title('Estimated Gyro Bias');
