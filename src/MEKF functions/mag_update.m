function [H_mag, z_mag] = mag_update(state, mag_measurement, config)
% MAG_UPDATE Build magnetometer measurement model.
    
    % Normalize measurement
    y_mag = mag_measurement / norm(mag_measurement);
    
    % Expected magnetic field direction in body frame
    R_be = quatToDCM(state(1:4));
    est_mag_b = R_be * config.m_ned;
    
    % Measurement innovation
    z_mag = y_mag - est_mag_b;
    
    % Measurement Jacobian
    H_mag = [skew(est_mag_b) zeros(3, 3)];
end
