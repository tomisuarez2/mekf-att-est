function [x, dx, P] = propagation_step(x, dx, P, gyro_meas, dt, Qk)
% PROPAGATION_STEP Perform MEKF propagation step.
    
    qk  = x(1:4);
    bgk = x(5:7);

    % Estimated angular rate (gyro measurement minus bias)
    omega_hat = gyro_meas - bgk;

    % Scalar last convention
    qk = [qk(2:4); qk(1)];
    
    % Quaternion propagation
    Omega_bar = compute_Omega_bar(dt, omega_hat);
    qk_1 = Omega_bar * qk;

    qk_1 = qk_1 / norm(qk_1);

    % Scalar first convention
    qk_1 = [qk_1(4); qk_1(1:3)];

    x(1:4) = qk_1;
    
    % State transition matrix
    Phi = compute_Phi(dt, omega_hat);

    % Error state propagation
    dx = Phi * dx;
    
    % Discrete process noise covariance matrix
    Gk = blkdiag(-eye(3), eye(3));

    % Covariance propagation
    P = Phi * P * Phi' + Gk * Qk * Gk';

end

