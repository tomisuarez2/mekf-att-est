function generate_validation_plots(truth, sensors)
% GENERATE_VALIDATION_PLOTS Generate validation plots

    % Each row: {time, data, legend, symbol, unit, description}
    cfg = {
        sensors.accel.time, sensors.accel.data, 'Simulated sensor', 'a^B_{m%d}',      'm/s^2', 'the accelerometer measurement'
        sensors.gyro.time,  truth.gyro_bias,    'Simulation',       'b_{g%d}',        'rad/s', 'the gyroscope bias'
        sensors.gyro.time,  sensors.gyro.data,  'Simulated sensor', 'w^B_{m%d}', 'rad/s', 'the gyroscope measurement'
        sensors.mag.time,   sensors.mag.data,   'Simulated sensor', 'm^B_{m%d}',      '-',     'the magnetometer measurement'
        };
    
    for k = 1:size(cfg, 1)
        for i = 1:3
            plot_component(cfg{k,:}, i);
        end
    end
end

