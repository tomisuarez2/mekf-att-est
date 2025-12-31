function [H_mag, z_mag] = build_magnetometer_update_MEKF(state, mag_measurement, config)
% BUILD_MAGNETOMETER_UPDATE Build magnetometer measurement model
    
    % Normalize measurement
    y_mag = mag_measurement / norm(mag_measurement);
    
    % Expected magnetic field direction in body frame
    R_be = quat_to_dcm(state.q_nom);
    est_mag_body = R_be * config.m_ned;
    
    % Measurement innovation
    z_mag = y_mag - est_mag_body;
    
    % Measurement Jacobian
    H_mag = [2 * skew(est_mag_body), zeros(3, 3)];
end
