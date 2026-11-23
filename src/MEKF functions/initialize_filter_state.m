function [x, dx, P] = initialize_filter_state(truth)
% INITIALIZE_FILTER_STATE Initialize filter state and covariance.

    % Initial quaternion (nominal part)
    q_nom_0 = truth.quat(:,1) + [0.0001; -0.0001; 0.0002; 0.0001];  % Uncertainty
    q_nom_0 = q_nom_0 / norm(q_nom_0);
    
    % Initial gyro bias estimate
    bg_0 = [0; 0; 0];

    % State
    x = [q_nom_0; bg_0];

    % Initial error state
    dalpha_0 = [0; 0; 0]; % Small angle orientation error
    dbg_0 = [0; 0; 0];    % Gyro bias error

    % Error state
    dx = [dalpha_0; dbg_0];
    
    % Initial error state covariance
    P = blkdiag((8.5e-2)^2 * eye(3), (1e-1)^2 * eye(3));
end
