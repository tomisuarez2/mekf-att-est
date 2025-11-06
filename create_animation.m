%{
% Example 1: Single object from STL file
animate_attitude_history(truth.time, truth.quat, {}, 'drone2.stl', 'truth_attitude_history');
%}

% Interpolate truth data to estimation times
truth_q_interp = zeros(4, length(est.time));
for i = 1:4
    truth_q_interp(i, :) = interp1(truth.time, truth.quat(i, :), est.time, 'linear', 'extrap');
end

% Example 2: Multiple objects
obj1 = struct('filename', 'drone2.stl', 'color', [0 0.5 0.01], 'name', 'Truth');
obj2 = struct('filename', 'drone2.stl', 'color', [0.5 0 0.5], 'name', 'Est.');

% Each object has its own attitude history
animate_attitude_history(est.time, {truth_q_interp, est.q}, {[], []}, {obj1, obj2}, 'truth_vs_estimated_attitude_history');
