% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function [ep] = dcm_to_ep(DCM, solutions)
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    Converts a Direction Cosine Matrix (DCM) to Euler parameters (quaternions)
    by means of the Shepperd algorithm.

    Inputs:
        DCM       - 3x3 Direction Cosine Matrix
        solutions - Optional, 'all' to return 2 possible sign variations.
                    Short rotation and Long rotation
    Output:
        ep        - 4x1 or 4x2 matrix of quaternions [beta0 beta1 beta2 beta3]^T in
                    each row.
%}
%-----------------------------

    % Precompute trace of DCM and beta^2 values
    tr          = trace(DCM);
    betaSquared = [1 + tr; ...
                   1 + 2*DCM(1,1) - tr; ...
                   1 + 2*DCM(2,2) - tr; ...
                   1 + 2*DCM(3,3) - tr] / 4;
    
    % Find index of maximum beta^2
    beta     = zeros(1,4);
    greatest = 0;
    idx      = 0;
    for i=1:4
        if betaSquared(i) > greatest
            greatest = betaSquared(i);
            idx      = i;
        end
    end
    
    beta(idx) = sqrt(greatest);
    
    % Fill in remaining components based on Shepperd algorithm
    switch idx
        case 1
            beta(2) = (DCM(2,3) - DCM(3,2)) / (4*beta(1));
            beta(3) = (DCM(3,1) - DCM(1,3)) / (4*beta(1));
            beta(4) = (DCM(1,2) - DCM(2,1)) / (4*beta(1));
        case 2
            beta(1) = (DCM(2,3) - DCM(3,2)) / (4*beta(2));
            beta(3) = (DCM(1,2) + DCM(2,1)) / (4*beta(2));
            beta(4) = (DCM(3,1) + DCM(1,3)) / (4*beta(2));
        case 3
            beta(1) = (DCM(3,1) - DCM(1,3)) / (4*beta(3));
            beta(2) = (DCM(1,2) + DCM(2,1)) / (4*beta(3));
            beta(4) = (DCM(2,3) + DCM(3,2)) / (4*beta(3));
        case 4
            beta(1) = (DCM(1,2) - DCM(2,1)) / (4*beta(4));
            beta(2) = (DCM(3,1) + DCM(1,3)) / (4*beta(4));
            beta(3) = (DCM(2,3) + DCM(3,2)) / (4*beta(4));
    end
    
    % Return all quaternion sign combinations if requested
    if nargin > 1 && isequal(solutions,'all')
        ep = [ beta;
              -beta]';
    else
        ep = beta';
    end

end
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~