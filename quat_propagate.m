% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function q_next = quat_propagate(q, omega, dt)
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    Propagates quaternion assuming small angle variation due to
    angular velocity.
    
    Inputs:
        q - 1x4 past unitary quaternion set
        omega - 3x1 current body frame angular velocity vector [rad/s]
        dt - Time interval [s]
    Outputs:
        q_next - Estimated 1x4 next quaternion set
%}
% --------------------------------------------

    theta = norm(omega)*dt;
    if theta < 1e-8
        dq = [1; 0.5*omega*dt];
    else
        dq = [cos(theta/2); sin(theta/2)*(omega*dt/theta)]; 
    end
    q_next = quat_prod(dq, q')';
    q_next = q_next/norm(q_next);
end
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
