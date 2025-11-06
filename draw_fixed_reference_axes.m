function draw_fixed_reference_axes(scale, color, letter)
%DRAW_FIXED_REFERENCE_AXES Draws fixed reference frame unit vectors
%   scale  - length of each axis vector (scalar)
%   color  - color of the reference frame unit vector axes
%   letter - letter of the reference frame unit vector axes
    if ~ischar(letter) && ~isempty(letter) 
        error("Axis reference identification must be a letter or word...")
    end
    if strcmp(letter,'NED') % NED reference system
        [~,~] = draw_arrow([1 0 0], scale, color, 'Y_e');
        [~,~] = draw_arrow([0 1 0], scale, color, 'X_e');
        [~,~] = draw_arrow([0 0 1], scale, color, '-Z_e');
    else
        [~,~] = draw_arrow([1 0 0], scale, color, strcat('X_',letter));
        [~,~] = draw_arrow([0 1 0], scale, color, strcat('Y_',letter));
        [~,~] = draw_arrow([0 0 1], scale, color, strcat('Z_',letter));
    end

end