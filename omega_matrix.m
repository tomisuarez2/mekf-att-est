% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function Omega = omega_matrix(omega)
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    Computes the quaternion update matrix.

    Inputs:
        omega - Array of dimension 3, representing the body angular
                velocity [rad/s]
    Outputs:
        Omega - Quaternion update matrix
%}
% ----------------------------------
    Omega = [...
        0,          -omega(1),  -omega(2),  -omega(3);
        omega(1),   0,           omega(3),  -omega(2);
        omega(2),   -omega(3),   0,          omega(1);
        omega(3),   omega(2),   -omega(1),   0 ...
    ];
end
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~