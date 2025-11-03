% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function ep_final = quat_prod(quat_2, quat_1)
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    Computes the quaternion product between ep_1 and ep_2.
    
    Inputs:
        quat_1     - 4x1 vector that represents first set of quaternions
        quat_2     - 4x1 vector that represents second set of quaternions
    Outputs:
        quat_final - 4x1 vector that represents final set of quaternions, 
                     the product between quat_1 and quat_2 
%}
%--------------------------------------------

    GB = [quat_1(1) -quat_1(2) -quat_1(3) -quat_1(4);
          quat_1(2)  quat_1(1) -quat_1(4)  quat_1(3);
          quat_1(3)  quat_1(4)  quat_1(1) -quat_1(2);
          quat_1(4) -quat_1(3)  quat_1(2)  quat_1(1)];
    ep_final = GB*quat_2;

end
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~