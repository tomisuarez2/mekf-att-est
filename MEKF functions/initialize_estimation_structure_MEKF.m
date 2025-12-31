function est = initialize_estimation_structure_MEKF(sensors)
% INITIALIZE_ESTIMATION_STRUCTURE Preallocate memory for estimation results
    n_points = length(sensors.gyro.time);
    est.quat = zeros(4, n_points);
    est.bg = zeros(3, n_points);
    est.time = sensors.gyro.time;
    est.P_history = cell(1, n_points);  % Optional: store covariance if needed
end

