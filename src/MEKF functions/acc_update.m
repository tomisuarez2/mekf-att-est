function [H_acc, z_acc, R_accw, epsilon] = acc_update(state, acc_measurement, config)
% ACC_UPDATE build accelerometer measurement model.

    % Parameters
    alpha_max = 500;

    % Measured specific force (normalized)
    acc_norm = norm(acc_measurement);
    y_acc = acc_measurement / acc_norm;

    % Gravity magnitude mismatch (for gating / weighting)
    g = norm(config.g_ned);
    epsilon = abs(acc_norm - g) / g;

    % Adaptive measurement noise
    alpha = 1 + 250 * epsilon^2;
    alpha = min(alpha, alpha_max);
    R_accw = config.R_acc * alpha;

    %% Expected gravity direction in body frame
    % Nominal rotation: body <- inertial (NED)
    R_be = quatToDCM(state(1:4));

    % Gravity direction in NED (unit vector, down)
    g_n = [0; 0; 1];

    % Expected unit gravity vector in body frame
    est_g_b = -R_be * g_n;

    % Innovation (measurement residual)
    z_acc = y_acc - est_g_b;

    % Measurement Jacobian (MEKF form)
    H_acc = [skew(est_g_b), zeros(3,3)];

end
