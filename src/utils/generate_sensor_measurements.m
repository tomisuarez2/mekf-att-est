function [sensors, gyro_bias] = generate_sensor_measurements(time, omega_b, sp_force, mag, config)
% GENERATE_SENSOR_MEASUREMENTS Generate noisy sensor measurements at
% different rates.

    % Create time vectors for each sensor
    t_acc  = 0:(1/config.fs_acc) :time(end);
    t_gyro = 0:(1/config.fs_gyro):time(end);
    t_mag  = 0:(1/config.fs_mag) :time(end);
    
    % Initialize sensor structures
    sensors = struct();
    
    % Generate accelerometer measurements
    accel_clean = interpolate_data(time, sp_force, t_acc);
    sensors.accel.data = accel_clean + chol(config.R_acc) * randn(size(accel_clean));
    sensors.accel.time = t_acc;
    sensors.accel.fs   = config.fs_acc;
    
    % Generate gyroscope measurements
    gyro_clean = interpolate_data(time, omega_b, t_gyro);
    gyro_rnd_bias  = cumsum(chol(config.Q_bg/config.fs_gyro) * randn(3, length(t_gyro)), 2);
    gyro_bias = config.gyro_sys_bias + gyro_rnd_bias;
    sensors.gyro.data = gyro_clean + chol(config.R_gyro) * randn(size(gyro_clean)) + gyro_bias;
    sensors.gyro.time = t_gyro;
    sensors.gyro.fs   = config.fs_gyro;
    
    % Generate magnetometer measurements
    mag_clean = interpolate_data(time, mag, t_mag);
    sensors.mag.data = mag_clean + chol(config.R_mag) * randn(size(mag_clean));
    sensors.mag.time = t_mag;
    sensors.mag.fs   = config.fs_mag;
    
end