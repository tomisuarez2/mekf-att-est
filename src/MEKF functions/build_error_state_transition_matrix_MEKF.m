function Phi = build_error_state_transition_matrix_MEKF(omega, dt)
% BUILD_ERROR_STATE_TRANSITION_MATRIX Build error state transition matrix
    Phi = eye(6);
    Phi(1:3, 1:3) = Phi(1:3, 1:3) - skew(omega) * dt;
    Phi(1:3, 4:6) = -0.5 * eye(3) * dt;
end

