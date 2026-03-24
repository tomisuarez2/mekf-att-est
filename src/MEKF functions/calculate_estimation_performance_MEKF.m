function calculate_estimation_performance_MEKF(est_time, est_quat, truth_time, truth_quat)
% CALCULATE_ESTIMATION_PERFORMANCE Calculate estimation performance metrics
    
    lw_default = 2.5; % LineWidth por defecto
    lw_auxiliary = 1.5; % LineWidth auxiliar
    
    label_fontsize = 20; % FontSize: Label
    title_fontsize = 34; % FontSize: Title
    ticks_fontsize = 16; % FontSize: Label Ticks

    % Interpolate truth data to estimation times
    n = length(est_time);
    truth_q_interp = zeros(4, n);

    for i = 1:n
        idx = find(truth_time <= est_time(i), 1, 'last');
        
        if idx >= length(truth_time)
            truth_q_interp(:, i) = truth_quat(:, end);
            continue;
        end
        
        t1 = truth_time(idx);
        t2 = truth_time(idx+1);
        alpha = (est_time(i) - t1) / (t2 - t1);
        
        % SLERP
        q1 = truth_quat(:, idx) / norm(truth_quat(:, idx));
        q2 = truth_quat(:, idx+1) / norm(truth_quat(:, idx+1));
        truth_q_interp(:, i) = quatinterp(q1', q2', alpha, 'slerp')';
    end
    
    % quaternion error
    q_error = zeros(1, length(est_time));
    for k = 1:length(est_time)
        q_err = quat_prod(truth_q_interp(:, k), quat_conj(est_quat(:, k)));
        q_error(k) = 2 * acos(abs(q_err(1)));  % Angular error in radians
    end
    
    
    fprintf('>>> Estimation Performance:\n');
    fprintf('    - Mean attitude error: %.4f deg\n', mean(rad2deg(q_error)));
    fprintf('    - Max attitude error: %.4f deg\n', max(rad2deg(q_error)));
    fprintf('    - RMS attitude error: %.4f deg\n', rms(rad2deg(q_error)));

    % Plot estimation error
    figure;
    ax_main = axes;
    hold(ax_main, 'on');
    grid(ax_main, 'on');
    plot(est_time, rad2deg(q_error), 'Color', '#77B5FE', 'LineWidth', 2, 'DisplayName', 'Error');
    
    %------------------ Principal axis format ------------------%
    xlabel('Time [s]'); 
    ylabel('Attiude error [deg]');
    title(sprintf('Attitude error represented as angle'), 'FontSize', 22, ...
        'FontName', 'Times New Roman', ...
        'FontWeight', 'bold', ...
        'Interpreter', 'latex', ...
        'Color', 'k');
    ax_main.FontSize = ticks_fontsize;
    ax_main.LineWidth = lw_auxiliary;
    
    %------------------ Legend ------------------%
    lgd = legend('show', 'Location', 'northeast');
    lgd.Box = 'on';
    lgd.Color = 'white';
    lgd.EdgeColor = 'black';
    lgd.LineWidth = 1;
end
