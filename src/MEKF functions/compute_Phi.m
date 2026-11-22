function Phi = compute_Phi(dt, omega_hat)
%COMPUTE_PHI  Phi error state transition matrix [6x6].
%
%   Phi = [Phi11  Phi12;
%          Phi21  Phi22]
%
%   Inputs
%   ------
%       dt         Time step [1x1] [s]
%       omega_hat  Estimated angular velocity expressed in the body frame [3x1] [rad/s]
%
%   Outputs
%   -------
%       Phi        Error state transition matrix [6x6]
    
    omega_hat = omega_hat(:);          
    w  = norm(omega_hat);              
    wx = skew(omega_hat);              % [omega_hat x]
    wx2 = wx * wx;                     % [omega_hat x]^2
    I3 = eye(3);
    
    if w < 1e-8
        % Limit as ||omega_hat|| -> 0 (avoids division by zero):
        % sin(w dt)/w            -> dt
        % (1 - cos(w dt))/w^2    -> dt^2/2
        % (w dt - sin(w dt))/w^3 -> dt^3/6
        s_w  = dt;
        c_w2 = dt^2 / 2;
        r_w3 = dt^3 / 6;
    else
        s_w  = sin(w * dt) / w;
        c_w2 = (1 - cos(w * dt)) / w^2;
        r_w3 = (w * dt - sin(w * dt)) / w^3;
    end
    
    Phi11 = I3 - wx * s_w + wx2 * c_w2;
    Phi12 = wx * c_w2 - I3 * dt - wx2 * r_w3;
    Phi21 = zeros(3);
    Phi22 = I3;
    
    Phi = [Phi11, Phi12; Phi21, Phi22];
end