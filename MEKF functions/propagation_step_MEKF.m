function [state, error_state, P] = propagation_step_MEKF(state, error_state, P, gyro_measurement, dt, Qc)
% PROPAGATION_STEP Perform MEKF propagation step
    
    % Estimated angular rate (gyro measurement minus bias)
    est_omega_B = gyro_measurement - state.bg;
    
    % Quaternion propagation using first-order integration
    Omega = omega_matrix(est_omega_B);
    Theta = eye(4) + 0.5 * Omega * dt;
    state.q_nom = Theta * state.q_nom;
    state.q_nom = state.q_nom / norm(state.q_nom);  % Normalize
    
    % State transition matrix
    Phi = build_error_state_transition_matrix_MEKF(est_omega_B, dt);

    % Error state propagation
    error_state_propagation = Phi * [error_state.dq; error_state.dbg];
    error_state.dq = error_state_propagation(1:3);
    error_state.dbg = error_state_propagation(4:6);
    
    % Discrete process noise covariance matrix
    Qd = build_discrete_process_covariance_matrix_MEKF(est_omega_B, dt, Qc, 1);

    % Covariance propagation (Joseph form for numerical stability)
    P = Phi * P * Phi' + Qd;
end

