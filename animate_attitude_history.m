% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function animate_attitude_history(time_vector, quat_history, ang_vel_history, vertices, faces, animation_filename)
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    Animates object orientation and angular velocities over time, it
    creates an animation file.
    
    Inputs:
        timeVector         - 1D array of N elements containing time evolution [s]
        quat_history       - 4xN array containing quaternion evolution [-]
        ang_vel_history    - 3xN array containing angular velocity evolution [rad/s]
        vertices           - Object vertices for plotting
        faces              - Object faces for plotting
        animation_filename - Animation filename

%}
% ------------------------------------------------------------------------------------------------------------

    maxL = computeMaxEdgeLength(vertices, faces);
    figure3D = figure('Visible','off','Position',[100 100 1920 1080]);
    subplot(2,4,[1 2 5 6]);
    axis equal;
    grid on;
    xlabel('Axis 1'); ylabel('Axis 2'); zlabel('Axis 3');
    view([1 1 1]);
    xlim([-1 1]*1.75*maxL); ylim([-1 1]*1.75*maxL); zlim([-1 1]*1.75*maxL); 
    hold on;
    tittle3D = title(sprintf('Attitude History Animation\n t = %.2f s', time_vector(1)));

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
    xlim([0 time_vector(end)])
    ylim(abs(max(quat_history,[],'all'))*[-1 1])
    for i = 1:4
        q(i) = plot(NaN, NaN, '-', 'Color', colors(i,:));
    end
    legend('q_1','q_2','q_3','q_4');
    
    % Angular velocity
    if ~isempty(ang_vel_history)
        subplot(2,4,[7 8]); hold on; title('Angular Velocity (rad/s)');
        xlim([0 time_vector(end)])
        ylim(abs(max(ang_vel_history,[],'all'))*[-1 1])
        for i = 1:3
            w(i) = plot(NaN, NaN, '-', 'Color', colors(i,:));
        end
        legend('\omega_1','\omega_2','\omega_3');
    end
    % Animation File
    video = VideoWriter(strcat(animation_filename,'.mp4'), 'MPEG-4');
    % Tiempo real
    video.FrameRate = round(length(time_vector)/time_vector(end));
    open(video);
    
    % Animation loop
    disp(length(time_vector))
    for k = 2:length(time_vector)
        disp(k)
        % Update 3D plot
        NB = ep_to_dcm(quat_history(k,:))';
        updatePatch(satellitePatch, NB, vertices);
        updateReferenceAxes(bX, bY, bZ, iTxt, jTxt, kTxt, NB, 2*maxL);
        if ~isempty(ang_vel_history)
            updateArrow(wArrow, wTxt, NB * ang_vel_history(:,k), 20*maxL);
        end
        tittle3D.String = sprintf('Attitude History Animation\n t = %.2f s', time_vector(k));
        % Update 2D plots
        for i = 1:4
            set(q(i),  'XData', time_vector(1:k), 'YData', quat_history(1:k,i));
        end
        if ~isempty(ang_vel_history)
            for i=1:3
                set(w(i),  'XData', time_vector(1:k), 'YData', ang_vel_history(i,1:k));
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
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

%% ============ Helper functions ============

function animation_loop(figure3D, video, time_vector, quat_cell, ang_vel_cell,  object_handles, plot_handles, n_objects)
% ANIMATION_LOOP Main animation loop
    for k = 1:length(time_vector)
        % Update 3D visualization for each object
        for i = 1:n_objects
            % Update orientation
            R = quaternion_to_dcm(quat_cell{i}(:, k));
            
            % Update object patch
            if iscell(object_handles.objects)
                obj = object_handles.objects{i};
            else
                obj = object_handles.objects;
            end
            vertices_transformed = (R * obj.vertices')';
            set(object_handles.patches{i}, 'Vertices', vertices_transformed);
            
            % Update reference frame
            update_reference_axes(object_handles.reference_frames{i}, R, 2 * object_handles.max_dim);
            
            % Update angular velocity arrow
            if ~isempty(ang_vel_cell{i}) && ~isempty(object_handles.angular_velocity{i})
                update_arrow(object_handles.angular_velocity{i}, R * ang_vel_cell{i}(:, k), 2 * object_handles.max_dim);
            end
        end
        
        % Update title
        set(object_handles.title, 'String', ...
            sprintf('Attitude History Animation\n t = %.2f s', time_vector(k)));
        
        % Update 2D plots
        update_time_plots(plot_handles, time_vector, quat_cell, ang_vel_cell, k, n_objects);
        
        % Capture frame
        drawnow;
        frame = getframe(figure3D);
        writeVideo(video, frame);
        
        % Progress indicator
        if mod(k, 100) == 0
            fprintf('  Processed %d/%d frames\n', k, length(time_vector));
        end
    end
end

function max_dim = compute_max_dimension(vertices)
% COMPUTE_MAX_DIMENSION Calculate maximum dimension of vertices
    ranges = max(vertices) - min(vertices);
    max_dim = max(ranges);
end