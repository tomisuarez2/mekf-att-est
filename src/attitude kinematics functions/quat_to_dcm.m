% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function DCM = ep_to_dcm(quat)
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    Computes the corresponding DCM from a quaternion set.
    
    Inputs:
        quat - 4x1 array that quaternion set
    Outputs
        DCM  - 3x3 Direction Cosine Matrix
    
%}
%------------------------------

DCM = [quat(1)^2 + quat(2)^2 - quat(3)^2 - quat(4)^2         2*(quat(2)*quat(3) + quat(1)*quat(4))            2*(quat(2)*quat(4) - quat(1)*quat(3));
            2*(quat(2)*quat(3) - quat(1)*quat(4))       quat(1)^2 - quat(2)^2 + quat(3)^2 - quat(4)^2         2*(quat(3)*quat(4) + quat(1)*quat(2));
            2*(quat(2)*quat(4) + quat(1)*quat(3))            2*(quat(3)*quat(4) - quat(1)*quat(2))       quat(1)^2 - quat(2)^2 - quat(3)^2 + quat(4)^2];

end
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~

