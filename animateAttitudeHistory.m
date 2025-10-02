function animateAttitudeHistory(timeVector, quat_history, ang_vel_history, vertices, faces, animationFileName)
    % Animates object orientation and angular velocities over time
    % It creates an animation file
    % Plots are not display in real time

    maxL = computeMaxEdgeLength(vertices, faces);
    figure3D = figure('Visible','off','Position',[100 100 1920 1080]);
    subplot(2,4,[1 2 5 6]);
    axis equal;
    grid on;
    xlabel('Axis 1'); ylabel('Axis 2'); zlabel('Axis 3');
    view([1 1 1]);
    xlim([-1 1]*1.75*maxL); ylim([-1 1]*1.75*maxL); zlim([-1 1]*1.75*maxL); 
    hold on;
    tittle3D = title(sprintf('Attitude History Animation\n t = %.2f s', timeVector(1)));

    % Draw inertial reference frame
    drawFixedReferenceAxes(2*maxL, 'k','n');

    % Initialize patch and body axes
    NB = ep_to_dcm(quat_history(1,:))'; 

    satellitePatch = patch('Faces', faces, 'Vertices', (NB * vertices')', 'FaceColor', [0.8 0.8 1], 'FaceAlpha', 0.5, 'EdgeColor', 'k');
    [bX, bY, bZ, iTxt, jTxt, kTxt] = drawMovingReferenceAxes(NB, 2*maxL, 'b', 'b');
    if ~isempty(ang_vel_history)
        [wArrow, wTxt] = drawArrow(NB * ang_vel_history(:,1), maxL, 'c', 'w');
    end

    % Set plots
    colors = lines(4);

    % MRPs
    subplot(2,4,[3 4]); hold on; title('Attitude Quaternions');
    xlim([0 timeVector(end)])
    ylim(abs(max(quat_history,[],'all'))*[-1 1])
    for i = 1:4
        q(i) = plot(NaN, NaN, '-', 'Color', colors(i,:));
    end
    legend('q_1','q_2','q_3','q_4');
    
    % Angular velocity
    if ~isempty(ang_vel_history)
        subplot(2,4,[7 8]); hold on; title('Angular Velocity (rad/s)');
        xlim([0 timeVector(end)])
        ylim(abs(max(ang_vel_history,[],'all'))*[-1 1])
        for i = 1:3
            w(i) = plot(NaN, NaN, '-', 'Color', colors(i,:));
        end
        legend('\omega_1','\omega_2','\omega_3');
    end
    % Animation File
    video = VideoWriter(strcat(animationFileName,'.mp4'), 'MPEG-4');
    % Tiempo real
    video.FrameRate = round(length(timeVector)/timeVector(end));
    open(video);
    
    % Animation loop
    disp(length(timeVector))
    for k = 2:length(timeVector)
        disp(k)
        % Update 3D plot
        NB = ep_to_dcm(quat_history(k,:))';
        updatePatch(satellitePatch, NB, vertices);
        updateReferenceAxes(bX, bY, bZ, iTxt, jTxt, kTxt, NB, 2*maxL);
        if ~isempty(ang_vel_history)
            updateArrow(wArrow, wTxt, NB * ang_vel_history(:,k), 20*maxL);
        end
        tittle3D.String = sprintf('Attitude History Animation\n t = %.2f s', timeVector(k));
        % Update 2D plots
        for i = 1:4
            set(q(i),  'XData', timeVector(1:k), 'YData', quat_history(1:k,i));
        end
        if ~isempty(ang_vel_history)
            for i=1:3
                set(w(i),  'XData', timeVector(1:k), 'YData', ang_vel_history(i,1:k));
            end
        end
        drawnow;

        % Video write
        frame = getframe(figure3D);
        writeVideo(video, frame);
    end

    % Close video
    close(video);
end
