
function [H_acc, z_acc, R_accw, epsilon] = build_accelerometer_update_MEKF(state, acc_measurement, config)
% BUILD_ACCELEROMETER_UPDATE_MEKF
% Accelerometer-based attitude correction for MEKF

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
    R_bn = quat_to_dcm(state.q_nom);

    % Gravity direction in NED (unit vector, down)
    g_n = config.g_ned / g;

    % Expected unit gravity vector in body frame
    y_hat = -R_bn * g_n;

    %% Innovation (measurement residual)
    z_acc = y_acc - y_hat;

    %% Measurement Jacobian (MEKF form)
    H_acc = [ 2*skew(y_hat), zeros(3,3) ];

end
%}
