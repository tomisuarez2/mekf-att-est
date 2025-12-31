function calculate_estimation_performance_MEKF(est, truth)
% CALCULATE_ESTIMATION_PERFORMANCE Calculate estimation performance metrics
    if isempty(truth) || ~isfield(truth, 'quat')
        fprintf('>>> No truth data available for performance evaluation.\n');
        return;
    end

    % Interpolate truth data to estimation times
    n = length(est.time);
    truth_q_interp = zeros(4, n);

    for i = 1:n
        idx = find(truth.time <= est.time(i), 1, 'last');
        
        if idx >= length(truth.time)
            truth_q_interp(:, i) = truth.quat(:, end);
            continue;
        end
        
        t1 = truth.time(idx);
        t2 = truth.time(idx+1);
        alpha = (est.time(i) - t1) / (t2 - t1);
        
        % SLERP
        q1 = truth.quat(:, idx) / norm(truth.quat(:, idx));
        q2 = truth.quat(:, idx+1) / norm(truth.quat(:, idx+1));
        truth_q_interp(:, i) = quatinterp(q1', q2', alpha, 'slerp')';
    end
    
    % Calculate quaternion error
    q_error = zeros(1, length(est.time));
    for k = 1:length(est.time)
        q_err = quat_prod(truth_q_interp(:, k), quat_conj(est.quat(:, k)));
        q_error(k) = 2 * acos(abs(q_err(1)));  % Angular error in radians
    end
    
    % Plot estimation error
    figure('Name', 'Estimation Error', 'Position', [400 400 800 400]);
    plot(est.time, rad2deg(q_error), 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]'); ylabel('Attitude Error [deg]');
    title('Attitude Estimation Error');
    
    fprintf('>>> Estimation Performance:\n');
    fprintf('    - Mean attitude error: %.4f deg\n', mean(rad2deg(q_error)));
    fprintf('    - Max attitude error: %.4f deg\n', max(rad2deg(q_error)));
    fprintf('    - RMS attitude error: %.4f deg\n', rms(rad2deg(q_error)));
end
