function generate_estimation_plots(est, truth, sensors, config)
    % GENERATE_ESTIMATION_PLOTS Generate plots for estimation results
    
    % Plot parameters
    lw_default = 2.5; 
    lw_auxiliary = 1.5; 
    label_fontsize = 20; 
    title_fontsize = 34;
    ticks_fontsize = 16;
    
    % Quaternion plot
    q_components = {'$q_0$', '$q_1$', '$q_2$', '$q_3$'};
    
    % If magnetometer has different sampling rate from accelerometer
    mag_interp_1 = interp1(sensors.mag.time, sensors.mag.data(1,:), sensors.accel.time);
    mag_interp_2 = interp1(sensors.mag.time, sensors.mag.data(2,:), sensors.accel.time);
    mag_interp_3 = interp1(sensors.mag.time, sensors.mag.data(3,:), sensors.accel.time);
    mag_interp = [mag_interp_1; mag_interp_2; mag_interp_3];
    
    % Compute TRIAD attitude measurements from synchronized accelerometer and magnetometer data
    n_samples = length(sensors.accel.data(1,:));
    q_TRIAD = zeros(4,n_samples);
    for k=1:n_samples
        acc_meas = sensors.accel.data(:, k);
        mag_meas = mag_interp(:, k);
        est_dcm = TRIAD([acc_meas mag_meas], [-config.g_ned, config.m_ned]);
        q_TRIAD(:,k) = dcmToQuat(est_dcm);
        q_TRIAD(:,k) = q_TRIAD(:,k) / norm(q_TRIAD(:,k));
    end
    
    for i=1:4
        figure;
        ax_main = axes;
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        stairs(sensors.accel.time, q_TRIAD(i,:), '*', 'Color','#F39C12', 'LineWidth', 1, 'DisplayName', 'TRIAD measurement');
        plot(truth.time, truth.quat(i,:), '-', 'Color', '#77B5FE', 'LineWidth', lw_default, 'DisplayName', 'Simulation');
        stairs(est.t, est.q(i,:), '-', 'Color','#55CC55', 'LineWidth', lw_default, 'DisplayName', 'MEKF estimation');
    
        %------------------ Principal axes format ------------------%
    
        xlabel('Time [s]', 'FontSize', label_fontsize, 'Interpreter', 'latex')
        ylabel(q_components{i}, 'FontSize', label_fontsize, 'Interpreter', 'latex')
        title(sprintf('%s component of the orientation quaternion', q_components{i}), ...
            'FontSize', title_fontsize, ...
            'FontName', 'Times New Roman', ...
            'FontWeight', 'bold', ...
            'Interpreter', 'latex');
        ax_main.FontSize = ticks_fontsize;
        ax_main.LineWidth = lw_auxiliary;
    
        lgd = legend('show', 'Location', 'northeast');
        lgd.Box = 'on';
    end
    
    % Gyro bias plot
    gyrob_components = {'$b_{g1} [rad/s]$','$b_{g2} [rad/s]$','$b_{g3} [rad/s]$'};
    gyrob_components_tit = {'$b_{g1}$','$b_{g2}$','$b_{g3}$'};
    
    for i=1:3
        figure;
        ax_main = axes;
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        plot(sensors.gyro.time, truth.gyro_bias(i,:), '-', 'Color', '#77B5FE', 'LineWidth', lw_default, 'DisplayName', 'Simulation');
        stairs(est.t, est.bg(i,:), '-', 'Color','#55CC55', 'LineWidth', lw_default, 'DisplayName', 'MEKF estimation');
    
        %------------------ Principal axes format ------------------%
    
        xlabel('Time [s]', 'FontSize', label_fontsize, 'Interpreter', 'latex')
        ylabel(gyrob_components{i}, 'FontSize', label_fontsize, 'Interpreter', 'latex')
        title(sprintf('%s component of the gyroscope bias', gyrob_components_tit{i}), ...
            'FontSize', title_fontsize, ...
            'FontName', 'Times New Roman', ...
            'FontWeight', 'bold', ...
            'Interpreter', 'latex');
        ax_main.FontSize = ticks_fontsize;
        ax_main.LineWidth = lw_auxiliary;
    
        lgd = legend('show', 'Location', 'northeast');
        lgd.Box = 'on';
    end

    % Covariance matrix diagonal plot
    n_P = length(est.P);
    n_states = size(est.P{1}, 1);
    P_diag = zeros(n_states, n_P);
    for k = 1:n_P
        P_diag(:, k) = diag(est.P{k});
    end

    % Labels (assumes error state = [attitude error (3); gyro bias error (3)])
    if n_states == 6
        P_labels = {'$P_{1,1}$ ($\delta\theta_1$)', '$P_{2,2}$ ($\delta\theta_2$)', '$P_{3,3}$ ($\delta\theta_3$)', ...
            '$P_{4,4}$ ($\delta b_{g1}$)', '$P_{5,5}$ ($\delta b_{g2}$)', '$P_{6,6}$ ($\delta b_{g3}$)'};
        P_titles = {'$P_{1,1}$', '$P_{2,2}$', '$P_{3,3}$', '$P_{4,4}$', '$P_{5,5}$', '$P_{6,6}$'};
    else
        P_labels = arrayfun(@(i) sprintf('$P_{%d,%d}$', i, i), 1:n_states, 'UniformOutput', false);
        P_titles = P_labels;
    end

    for i = 1:n_states
        figure;
        ax_main = axes;
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        stairs(est.t, P_diag(i,:), '-', 'Color', '#55CC55', 'LineWidth', lw_default, ...
            'DisplayName', 'MEKF covariance');
        set(ax_main, 'YScale', 'log');   % covariance typically spans orders of magnitude
        %------------------ Principal axes format ------------------%
        xlabel('Time [s]', 'FontSize', label_fontsize, 'Interpreter', 'latex')
        ylabel(P_labels{i}, 'FontSize', label_fontsize, 'Interpreter', 'latex')
        title(sprintf('Evolution of %s (covariance diagonal element)', P_titles{i}), ...
            'FontSize', title_fontsize, ...
            'FontName', 'Times New Roman', ...
            'FontWeight', 'bold', ...
            'Interpreter', 'latex');
        ax_main.FontSize = ticks_fontsize;
        ax_main.LineWidth = lw_auxiliary;
        
        lgd = legend('show', 'Location', 'northeast');
        lgd.Box = 'on';
    end

end