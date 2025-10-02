%=============================
% ------ MEKF algorithm ------
%=============================

% Preallocate estimatation results
est.q = zeros(length(sensors.gyro.time),4);
est.time = sensors.gyro.time;

% Initialize variables
q_nom = [1 0 0 0]; % Initial estimation
bg    = zeros(3,1);          % gyro bias
P     = 1e-3*eye(6);         % error-state covariance

% Noise covariances
R_gyro  = diag([2.830380375642008306e-06, 4.934009191301758550e-06, 3.320219599119017912e-06]);
R_acc = diag([1.090681860817302234e-03, 9.279916753344519612e-04, 2.146372869350902587e-03]);
R_mag   = 0.1*diag([2.890238578269250028e-03, 1.901404294437207543e-02, 7.715742865748796697e-03]);

Q = blkdiag(R_gyro, (1e-7^2)*eye(3)); % gyro noise, bias noise

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
    omega_m = sensors.gyro.data(:,k) - bg;

    % Nominal quaternion propagation
    q_nom = quat_propagate(q_nom, omega_m, dt);
    % Short rotation only
    if q_nom(1) < 0
        q_nom = -q_nom;
    end

    % Linearized propagation of error covariance
    F = eye(6);
    F(1:3,1:3) = F(1:3,1:3) - skew(omega_m)*dt;
    F(1:3,4:6) = -eye(3)*dt;
    G = blkdiag(-eye(3), eye(3));
    P = F*P*F' + G*Q*G'*dt;
    
    % --- CORRECTIONS ---
    acc_available = (i_acc <= length(sensors.accel.time)) && ...
                    (abs(sensors.gyro.time(k) - sensors.accel.time(i_acc)) < 1e-2);
    
    mag_available = (i_mag <= length(sensors.mag.time)) && ...
                    (abs(sensors.gyro.time(k) - sensors.mag.time(i_mag)) < 1e-2);

    if acc_available || mag_available
        disp("si");
        H_all = [];
        y_all = [];
        R_all = [];
    
        % Accelerometer update (gravity reference)
        if acc_available
            z = sensors.accel.data(:,i_acc);
            z = z/norm(z);
            h = quat2rotm(q_nom) * [0;0;1]; % expected gravity dir
            y = z - h;  % innovation
    
            H_acc = [-skew(quat2rotm(q_nom)*[0;0;1]), zeros(3,3)];
            
            H_all = [H_all; H_acc];
            y_all = [y_all; y];
            R_all = blkdiag(R_all, R_acc);
    
            i_acc = i_acc + 1;
        end
    
        % Magnetometer update (magnetic north reference)
        if mag_available
            z = sensors.mag.data(:,i_mag);
            z = z/norm(z);
            h = quat2rotm(q_nom) * m_ned; % expected mag dir
            y = z - h;
    
            H_mag = [-skew(quat2rotm(q_nom)*m_ned), zeros(3,3)];
            
            H_all = [H_all; H_mag];
            y_all = [y_all; y];
            R_all = blkdiag(R_all, R_mag);
    
            i_mag = i_mag + 1;
        end
    
        % Single joint update
        S = H_all*P*H_all' + R_all;
        K = P*H_all'/S;
    
        dx = K*y_all;
        P = (eye(6)-K*H_all)*P;
    
        % Inject error into quaternion + gyro bias
        dq = [1; 0.5*dx(1:3)];
        q_nom = quat_prod(dq, q_nom')';
        q_nom = q_nom / norm(q_nom);
        if q_nom(1) < 0
            q_nom = -q_nom;
        end
    
        bg = bg + dx(4:6);  
    end
    est.q(k,:) = q_nom;
end
%% Some plots
figure;
plot(est.time,est.q); grid on;
xlabel('Time [s]'); ylabel('\hat{q}_i'); 
title('Estimated Attitude Quaternions');


%%
% --- align estimator timestamps to truth (nearest) ---
% truth.time (1xN), truth.q (Nx4)
% est.time (1xM), est.q (Mx4)
M = size(est.q,1);
angle_err = zeros(M,1);
eul_err = zeros(M,3); % yaw,pitch,roll differences (rad)

for k = 1:M
    % find nearest truth index (simple nearest-neighbor)
    [~, idx] = min(abs(truth.time - est.time(k)));
    q_true = truth.q(idx,:);   % row [w x y z]
    q_est  = est.q(k,:);
    
    % quaternion error: q_err = q_true * conj(q_est)
    q_err = quat_prod(q_true', quat_conj(q_est'))';
    q_err = q_err / norm(q_err);
    
    % rotation angle error (rad)
    qw = q_err(1);
    qw = max(min(abs(qw),1.0), -1.0);
    angle_err(k) = 2 * acos(qw);    % in radians
    
    % Euler error (convert both to euler, unwrap and subtract)
    % use same eul order used previously: [yaw pitch roll]
    eul_true = quat2eul(q_true); % returns [yaw pitch roll]
    eul_est  = quat2eul(q_est);
    eul_err(k,:) = angle_wrap(eul_true - eul_est); % wrap to [-pi, pi]
end

% --- summary statistics ---
deg = 180/pi;
stats.angle.RMSE_deg = sqrt(mean(angle_err.^2)) * deg;
stats.angle.mean_deg = mean(angle_err) * deg;
stats.angle.std_deg = std(angle_err) * deg;
stats.angle.max_deg = max(angle_err) * deg;
stats.angle.median_deg = median(angle_err) * deg;

stats.euler.RMSE_deg = sqrt(mean(eul_err.^2))' * deg; % 3x1 (yaw,pitch,roll)
stats.euler.mean_deg = mean(eul_err)' * deg;
stats.euler.std_deg = std(eul_err)' * deg;
stats.euler.max_deg = max(abs(eul_err))' * deg;

% print
fprintf('Attitude rotation error (deg) — RMSE: %.3f, mean: %.3f, std: %.3f, max: %.3f\n', ...
    stats.angle.RMSE_deg, stats.angle.mean_deg, stats.angle.std_deg, stats.angle.max_deg);
fprintf('Euler RMSE (yaw,pitch,roll) [deg]: %.3f, %.3f, %.3f\n', stats.euler.RMSE_deg);

% --- plots ---
figure;
subplot(2,1,1);
plot(est.time, angle_err*deg); grid on;
ylabel('Rotation error [deg]'); title('Attitude rotation error vs time');

subplot(2,1,2);
plot(est.time, eul_err*deg); grid on;
legend('yaw','pitch','roll'); ylabel('Euler error [deg]'); xlabel('time [s]');

% --- histogram of angle error ---
figure; histogram(angle_err*deg,50); grid on;
xlabel('Rotation error [deg]'); ylabel('counts'); title('Angle error distribution');

%% --- helper functions ---
function v = angle_wrap(v)
    % wrap angles to [-pi, pi]
    v = mod(v + pi, 2*pi) - pi;
end
