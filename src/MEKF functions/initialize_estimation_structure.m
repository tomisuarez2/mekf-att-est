function est = initialize_estimation_structure(sensors)
% INITIALIZE_ESTIMATION_STRUCTURE Preallocate memory for estimation
% results.

    n_points = length(sensors.gyro.time);
    est.q    = zeros(4, n_points);
    est.bg   = zeros(3, n_points);
    est.t    = sensors.gyro.time;
    est.P    = cell(1, n_points);  % Optional: store covariance if needed
    
end

