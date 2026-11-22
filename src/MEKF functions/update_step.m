function [dx, P, new_idx_acc, new_idx_mag] = update_step(x, P, measurement_data, sensors, config, idx_acc, idx_mag)
% UPDATE_STEP Perform MEKF update step with available measurements
    
    H_all = [];
    z_all = [];
    R_blocks = {};
    
    new_idx_acc = idx_acc;
    new_idx_mag = idx_mag;
    
    % Accelerometer update (gravity reference)
    if measurement_data.acc_available
        [H_acc, z_acc, R_accw, epsilon] = acc_update(x, sensors.accel.data(:, new_idx_acc), config);
        
        epsilon_max = 0.25;

        if epsilon <= epsilon_max % Else reject measurement
            H_all = [H_all; H_acc];
            z_all = [z_all; z_acc];
            R_blocks{end+1} = R_accw;
        end
            
        new_idx_acc = new_idx_acc + 1;
    end
    
    % Magnetometer update (magnetic north reference)
    if measurement_data.mag_available
        [H_mag, z_mag] = mag_update(x, sensors.mag.data(:, new_idx_mag), config);
        
        H_all = [H_all; H_mag];
        z_all = [z_all; z_mag];
        R_blocks{end+1} = config.R_mag;
        
        new_idx_mag = new_idx_mag + 1;
    end
    
    % Combine measurement noise covariance
    R_all = blkdiag(R_blocks{:});
    
    % Kalman update
    if ~isempty(R_all)
        [dx, P] = perform_kalman_update(P, H_all, z_all, R_all);
    end
end

