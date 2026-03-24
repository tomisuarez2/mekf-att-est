function [state, P] = perform_kalman_update_MEKF(state, error_state, P, H, z, R)
% PERFORM_KALMAN_UPDATE Perform Kalman filter update step
    
    % Innovation covariance
    S = H * P * H' + R;
    
    % Kalman gain
    K = P * H' / S;
    
    % Error state update (dx = [dtheta; dbg])
    dx = [error_state.dq;error_state.dbg] + K * z;
    
    % Covariance update (Joseph form)
    I = eye(size(P));
    P = (I - K * H) * P * (I - K * H)' + K * R * K';
    
    % Apply correction to nominal states
    dq = [1; dx(1:3)];  % Small angle approximation for quaternion
    dq = dq / norm(dq);
    
    state.q_nom = quat_prod(dq, state.q_nom);
    state.bg = state.bg + dx(4:6);
    
    % Reset error state
    % Note: In MEKF, the error state is reset to zero after update
end

