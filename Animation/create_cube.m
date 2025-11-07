% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function [v, f] = create_cube(s)
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    Creates a cube for rigid body representation,
    
    Input:
        s - scale of the cube
    Outputs:
        v - vertices of the cube
        f - faces of the cube
%}
% ------------------------------

    v = s * [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1;
             -1 -1  1; 1 -1  1; 1 1  1; -1 1  1];
    f = [1 2 3 4; 5 6 7 8; 1 2 6 5;
         2 3 7 6; 3 4 8 7; 4 1 5 8];
    
end
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~