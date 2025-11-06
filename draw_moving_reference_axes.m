function [xArrow, yArrow, zArrow, xText, yText, zText] = draw_moving_reference_axes(R, scale, color, letter)
%DRAW_MOVING_REFERENCE_AXES Draws body axes in inertial frame
%   R      - DCM (columns are body axes)
%   scale  - arrow length scaling factor
%   color  - color of the reference frame unit vector axes
%   letter - letter of the reference frame unit vector axes
    if ~ischar(letter) && ~isempty(letter) 
        error("Axis reference identification must be a letter or word...")
    end
    
    xVec = R(:,1) * scale;
    yVec = R(:,2) * scale;
    zVec = R(:,3) * scale;

    if strcmp(letter,'NED') % NED reference system
        [xArrow,xText] = draw_arrow(xVec, scale, color, 'Y_e');
        [yArrow,yText] = draw_arrow(yVec, scale, color, 'X_e');
        [zArrow,zText] = draw_arrow(zVec, scale, color, '-Z_e');
    else
        [xArrow,xText] = draw_arrow(xVec, scale, color, strcat('X_',letter));
        [yArrow,yText] = draw_arrow(yVec, scale, color, strcat('Y_',letter));
        [zArrow,zText] = draw_arrow(zVec, scale, color, strcat('Z_',letter));
    end
end