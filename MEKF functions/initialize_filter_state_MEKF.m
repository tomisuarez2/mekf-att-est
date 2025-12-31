function [state, error_state, P] = initialize_filter_state_MEKF(truth)
% INITIALIZE_FILTER_STATE Initialize filter state and covariance
    % Initial quaternion (nominal part)
    state.q_nom = truth.quat(:,1) + [0.0001; -0.0001; 0.0002; 0.0001];  % Uncertainty
    state.q_nom = state.q_nom / norm(state.q_nom);
    
    % Initial gyro bias estimate
    state.bg = [0; 0; 0];

    % Initial error state
    error_state.dq = [0; 0; 0];   % Quaternion orientation error
    error_state.dbg = [0; 0; 0];        % Gyro bias error
    
    % Initial error state covariance
    P = diag([...
        (8.5e-3)^2, (8.5e-3)^2, (8.5e-3)^2, ...  % Orientation error variance
        (1e-4)^2, (1e-4)^2, (1e-4)^2 ...         % Bias error variance
    ]);
end
