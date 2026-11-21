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

function plot_component(t, data, legendName, symbolFmt, unit, descr, i)
    
    % Plot parameters
    lw_default     = 2.5;
    lw_auxiliary   = 1.5;
    label_fontsize = 20;
    title_fontsize = 34;
    ticks_fontsize = 16;
    
    symbol = sprintf(symbolFmt, i);
    
    figure;

    ax = axes; 
    hold(ax, 'on'); 
    grid(ax, 'on');
    stairs(t, data(i,:), '-', 'Color', '#77B5FE', 'LineWidth', lw_default, ...
        'DisplayName', legendName);
    
    xlabel('Time [s]', 'FontSize', label_fontsize, 'Interpreter', 'latex')
    ylabel(sprintf('$%s [%s]$', symbol, unit), 'FontSize', label_fontsize, 'Interpreter', 'latex')
    title(sprintf('$%s$ component of %s', symbol, descr), ...
        'FontSize', title_fontsize, 'FontName', 'Times New Roman', ...
        'FontWeight', 'bold', 'Interpreter', 'latex', 'Color', 'k');
    ax.FontSize  = ticks_fontsize;
    ax.LineWidth = lw_auxiliary;
    
    legend('show', 'Location', 'northeast', 'Box', 'on', 'Color', 'white', ...
        'EdgeColor', 'black', 'LineWidth', 1);

end