function [mag_truth, sp_force_truth] = compute_sensor_measurements(acc, quat, config)
% COMPUTE_SENSOR_MEASUREMENTS Compute ideal sensor measurements

    n_points = size(quat, 2);
    mag_truth = zeros(3, n_points);
    sp_force_truth = zeros(3, n_points);
    
    for k = 1:n_points
        % Specific force = acceleration - gravity 
        R_be = quatToDCM(quat(:, k));
        sp_force_truth(:, k) = R_be*(acc(:,k) - config.g_ned);
        % Magnetic field in body frame
        mag_truth(:, k) = R_be * config.m_ned;
    end
    
end