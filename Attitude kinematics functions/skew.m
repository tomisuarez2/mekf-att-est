% ~~~~~~~~~~~~~~~~~~
function S = skew(v)
% ~~~~~~~~~~~~~~~~~~
%{
    Computes skew-symmetric matrix from a vector v.

    Inputs:
        v - 1x3 vector
    Outputs:
        S - Skew-symmetric matrix from vector v
%}
% -----------------

    S = [  0  -v(3) v(2);
          v(3)  0  -v(1);
         -v(2) v(1)  0  ];

end
% ~~~~~~~~~~~~~~~~~~