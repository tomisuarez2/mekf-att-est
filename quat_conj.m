% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function quat_conj = quat_conj(quat)
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    Computes the conjugate of the input quaternion. Scalar part first.
    
    Inputs:
        quat      - 4x1 vector that represents a set of quaternions
    Outputs:
        quat_conj - 4x1 vector that represents conjugate set of quaternions, 

%}
%-----------------------------------

    quat_conj = [quat(1);-quat(2:4)];

end
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

