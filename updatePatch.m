function updatePatch(patchHandle, R, vertices)
%UPDATEPATCH Rotates and updates spacecraft patch
%   patchHandle - handle to existing patch
%   R           - DCM at current time
%   vertices    - [Vx3] body-frame vertices

    rotatedVertices = (R * vertices')';
    set(patchHandle, 'Vertices', rotatedVertices);
end