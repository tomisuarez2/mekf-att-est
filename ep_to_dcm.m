% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function DCM = ep_to_dcm(beta)
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    Computes the corresponding DCM from an Euler parameter set.
    
    Inputs:
        beta - 1x4 vector that contains Euler parameter set
    Outputs
        DCM  - 3x3 Direction Cosine Matrix
    
%}
%------------------------------

DCM = [beta(1)^2 + beta(2)^2 - beta(3)^2 - beta(4)^2         2*(beta(2)*beta(3) + beta(1)*beta(4))            2*(beta(2)*beta(4) - beta(1)*beta(3));
            2*(beta(2)*beta(3) - beta(1)*beta(4))       beta(1)^2 - beta(2)^2 + beta(3)^2 - beta(4)^2         2*(beta(3)*beta(4) + beta(1)*beta(2));
            2*(beta(2)*beta(4) + beta(1)*beta(3))            2*(beta(3)*beta(4) - beta(1)*beta(2))       beta(1)^2 - beta(2)^2 - beta(3)^2 + beta(4)^2];

end
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~

