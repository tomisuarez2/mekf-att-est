function drawFixedReferenceAxes(scale, color, letter)
%DRAWFIXEDREFERENCEAXES Draws fixed reference frame unit vectors
%   scale  - length of each axis vector (scalar)
%   color  - color of the reference frame unit vector axes
%   letter - letter of the reference frame unit vector axes

    [~,~] = drawArrow([1,0,0], scale, color, strcat(letter,'_1'));
    [~,~] = drawArrow([0,1,0], scale, color, strcat(letter,'_2'));
    [~,~] = drawArrow([0,0,1], scale, color, strcat(letter,'_3'));

end