function [update_available, measurement_data] = check_sensor_availability(tk, sensors, idx_acc, idx_mag)
% CHECK_SENSOR_AVAILABILITY Check if sensor measurements are available
    
    tolerance = 1e-4;
    
    % Check accelerometer availability
    acc_available = (idx_acc <= length(sensors.accel.time)) && (abs(tk - sensors.accel.time(idx_acc)) < tolerance);
    
    % Check magnetometer availability
    mag_available = (idx_mag <= length(sensors.mag.time)) && (abs(tk - sensors.mag.time(idx_mag)) < tolerance);
    
    update_available = acc_available || mag_available;
    
    % Package measurement data
    measurement_data.acc_available = acc_available;
    measurement_data.mag_available = mag_available;
    measurement_data.current_time = tk;
    
end
