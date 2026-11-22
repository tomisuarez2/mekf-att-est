function Omega = compute_Omega_bar(dt, omega_hat)
%COMPUTE_OMEGA  Quaternion propagation matrix Omega_bar [4x4].
%
%   Omega = [ cos(0.5*||w||*dt)*I3 - [psi x]    psi ;
%             -psi'                             cos(0.5*||w||*dt) ]
%
%   psi = sin(0.5*||w||*dt) * w / ||w||
%
%   Inputs
%   ------
%       dt         Time step [1x1] [s]
%       omega_hat  Estimated angular velocity expressed in the body frame [3x1] [rad/s]
%
%   Outputs
%   -------
%       Omega      Quaternion state transition matrix [6x6]
%
%   Notes
%   -----
%       Scalar last convention is assumed [q1 q2 q3 q4].

    omega_hat = omega_hat(:);              
    w = norm(omega_hat);                   % ||omega_hat||
    half_angle = 0.5 * w * dt;             % 0.5*||omega_hat||*dt

    c = cos(half_angle);

    if w < 1e-8
        % Limit as ||omega_hat|| -> 0 (avoids division by zero):
        % sin(0.5*w*dt)/w -> 0.5*dt
        psi = 0.5 * dt * omega_hat;
    else
        psi = sin(half_angle) * omega_hat / w;
    end

    Omega = [c * eye(3) - skew(psi),  psi;
             -psi',                   c];
end