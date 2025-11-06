function update_reference_axes(rf, R, scale)
%UPDATE_REFERNCE_AXES Updates orientation and labels of reference axes
%   rf    - struct with reference frame arrows handles and text handles
%   R     - DCM at current time
%   scale - scaling factor

    % Axes vectors
    xVec = R(:,1);
    yVec = R(:,2);
    zVec = R(:,3);

    % Update arrows
    update_arrow(rf.bX, rf.iTxt, xVec, scale);
    update_arrow(rf.bY, rf.jTxt, yVec, scale);
    update_arrow(rf.bZ, rf.kTxt, zVec, scale);

end