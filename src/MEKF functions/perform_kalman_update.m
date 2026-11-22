function [dx, P] = perform_kalman_update(P, H, z, R)
% PERFORM_KALMAN_UPDATE Perform Kalman filter update step.
    
    % Innovation covariance
    S = H * P * H' + R;
    
    % Kalman gain
    K = P * H' / S;
    
    % Error state update
    dx = K * z;
    
    % Covariance update (Joseph's form)
    IKH = I - K*H;
    P = IKH*P*IKH' + K*R*K';
    P = (P + P')/2;

end

