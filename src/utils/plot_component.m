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