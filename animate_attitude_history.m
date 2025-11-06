% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function animate_attitude_history(time_vector, quat_history, ang_vel_history, object_data, animation_filename, varargin)
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
    Animates object orientation and angular velocities over time

    Inputs:
        time_vector      - 1×N array of time points [s]
        quat_history     - 4×N array of quaternion history (or cell array for multiple objects)
        ang_vel_history  - 3×N array of angular velocity history (or cell array for multiple objects)
        object_data      - Structure, cell array, or char array specifying object(s):
                            - Char array: single STL filename
                            - Structure with fields 'filename', 'color', 'scale', etc.
                            - Cell array of structures for multiple objects
        animation_filename - Output filename (without extension)
        Optional parameters (name-value pairs):
            'FrameRate'    - Video frame rate (default: real-time based on data)
            'ViewAngle'    - Camera view angle [az, el] (default: [45, 30])
            'AxisScale'    - Scaling factor for axes (default: 1.5)
            'ShowTrajectory'- Show attitude trajectory (default: false)
            'BackgroundColor' - Figure background color (default: 'white')

    Example usage:
        % Single object from STL file
        animate_attitude_history(t, q, w, 'satellite.stl', 'animation');
        
        % Multiple objects with different properties
        obj1 = struct('filename', 'body.stl', 'color', [0 0.5 0], 'scale', 1.0);
        obj2 = struct('filename', 'solar_panel.stl', 'color', [0.9 0.7 0.3], 'scale', 0.5);
        animate_attitude_history(t, {q1, q2}, {w1, w2}, {obj1, obj2}, 'animation');

%}
% ----------------------------------------------------------------------------------------------------------------------

    % Parse optional parameters
    p = inputParser;
    addParameter(p, 'FrameRate', [], @isnumeric);
    addParameter(p, 'ViewAngle', [45, 30], @(x) isnumeric(x) && numel(x)==2);
    addParameter(p, 'AxisScale', 1.5, @isnumeric);
    addParameter(p, 'ShowTrajectory', false, @islogical);
    addParameter(p, 'BackgroundColor', 'white', @(x) ischar(x) || isnumeric(x));
    addParameter(p, 'Resolution', [1920, 1080], @(x) isnumeric(x) && numel(x)==2);
    
    parse(p, varargin{:});
    config = p.Results;
    
    % Process object data
    [objects, n_objects] = process_object_data(object_data);
    
    % Process attitude data
    [quat_cell, ang_vel_cell] = process_attitude_data(quat_history, ang_vel_history, n_objects);
    
    % Validate data consistency
    validate_data_consistency(time_vector, quat_cell, ang_vel_cell, n_objects);
    
    % Initialize figure and video
    [figure3D, video] = initialize_animation(config, animation_filename);
    
    % Load and prepare 3D objects
    object_handles = prepare_3d_objects(objects, n_objects, config);
    
    % Initialize plots
    plot_handles = initialize_plots(objects, time_vector, ang_vel_cell, n_objects);

    % Animation loop
    fprintf('>>> Generating animation with %d frames...\n', length(time_vector));
    animation_loop(figure3D, video, time_vector, quat_cell, ang_vel_cell, ...
                   object_handles, plot_handles, n_objects);

    % Cleanup
    close(video);
    fprintf('>>> Animation saved as: %s.mp4\n', animation_filename);
    %}
end
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

%% ============ Helper functions ============

function [objects, n_objects] = process_object_data(object_data)
% PROCESS_OBJECT_DATA Process and validate object input data
    if ischar(object_data)
        % Single STL filename
        objects = struct('filename', object_data, 'color', [0 0.5 0], 'scale', 1.0, 'alpha', 0.7, 'edgecolor', 'k');
        n_objects = 1;
    elseif isstruct(object_data) && isfield(object_data, 'filename')
        % Single object structure
        objects = validate_object_structure(object_data);
        n_objects = 1;
    elseif iscell(object_data)
        % Multiple objects
        n_objects = length(object_data);
        objects = cell(1, n_objects);
        for i = 1:n_objects
            if ischar(object_data{i})
                objects{i} = struct('filename', object_data{i}, 'color', [0 0.5 0], 'scale', 1.0, 'alpha', 0.7, 'edgecolor', 'k');
            else
                objects{i} = validate_object_structure(object_data{i});
            end
        end
    else
        error('Invalid object_data format. See function help for usage examples.');
    end
    
    % Load and validate STL files
    for i = 1:n_objects
        if iscell(objects)
            obj = objects{i};
        else
            obj = objects;
        end
        
        try
            [obj.vertices, obj.faces] = read_stl_file(obj.filename);
            obj.vertices = obj.vertices * obj.scale;
            obj.max_dim = compute_max_dimension(obj.vertices);
            
            if iscell(objects)
                objects{i} = obj;
            else
                objects = obj;
            end
        catch ME
            error('Failed to load STL file %s: %s', obj.filename, ME.message);
        end
    end
end

function obj_struct = validate_object_structure(obj_struct)
% VALIDATE_OBJECT_STRUCTURE Ensure object structure has all required fields
    default_fields = struct('color', [0 0.5 0], 'scale', 1.0, 'alpha', 0.7, 'edgecolor', 'k', 'name', '');
    field_names = fieldnames(default_fields);
    for i = 1:length(field_names)
        if ~isfield(obj_struct, field_names{i})
            obj_struct.(field_names{i}) = default_fields.(field_names{i});
        end
    end
    
    if ~isfield(obj_struct, 'filename')
        error('Object structure must contain ''filename'' field');
    end
end

function [vertices, faces] = read_stl_file(filename)
% READ_STL_FILE Read vertices and faces from STL file
    if ~exist(filename, 'file')
        error('STL file not found: %s', filename);
    end
    % Try using MATLAB's built-in stlread if available
    if exist('stlread', 'file')
        model = stlread(filename);
        vertices = model.Points;
        faces = model.ConnectivityList;
    else
        error('stlread built-in not found');
    end
end

function [quat_cell, ang_vel_cell] = process_attitude_data(quat_history, ang_vel_history, n_objects)
% PROCESS_ATTITUDE_DATA Process quaternion and angular velocity data
    % Process quaternions
    if iscell(quat_history)
        if length(quat_history) ~= n_objects
            error('Number of quaternion histories (%d) must match number of objects (%d)', length(quat_history), n_objects);
        end
        quat_cell = quat_history;
    else
        quat_cell = repmat({quat_history}, 1, n_objects); % Assume the same quaternion history for all objects.
    end
    
    % Process angular velocities
    if isempty(ang_vel_history)
        ang_vel_cell = repmat({[]}, 1, n_objects);
    elseif iscell(ang_vel_history)
        if length(ang_vel_history) ~= n_objects
            error('Number of angular velocity histories (%d) must match number of objects (%d)', length(ang_vel_history), n_objects);
        end
        ang_vel_cell = ang_vel_history;
    else
        ang_vel_cell = repmat({ang_vel_history}, 1, n_objects);
    end
end

function validate_data_consistency(time_vector, quat_cell, ang_vel_cell, n_objects)
% VALIDATE_DATA_CONSISTENCY Check data dimensions and consistency
    n_time = length(time_vector);
    
    for i = 1:n_objects
        if size(quat_cell{i}, 2) ~= n_time
            error('Quaternion history %d has %d time points, expected %d', i, size(quat_cell{i}, 2), n_time);
        end
        
        if ~isempty(ang_vel_cell{i}) && size(ang_vel_cell{i}, 2) ~= n_time
            error('Angular velocity history %d has %d time points, expected %d', i, size(ang_vel_cell{i}, 2), n_time);
        end
    end
end

function [figure3D, video] = initialize_animation(config, animation_filename)
% INITIALIZE_ANIMATION Initialize figure and video writer
    figure3D = figure('Visible', 'off', 'Position', [100, 100, config.Resolution]);
    set(figure3D, 'Color', config.BackgroundColor);
    
    % Initialize video
    video = VideoWriter([animation_filename, '.mp4'], 'MPEG-4');
    video.Quality = 95;
    
    if isempty(config.FrameRate)
        video.FrameRate = 10;
    else
        video.FrameRate = config.FrameRate;
    end
    
    open(video);
end

function object_handles = prepare_3d_objects(objects, n_objects, config)
% PREPARE_3D_OBJECTS Load and prepare 3D objects for animation
    object_handles = struct();
    object_handles.patches = cell(1, n_objects);
    object_handles.reference_frames = cell(1, n_objects);
    object_handles.angular_velocity = cell(1, n_objects);
    object_handles.objects = objects;
    
    % Calculate overall bounding box
    all_vertices = [];
    for i = 1:n_objects
        if iscell(objects)
            obj = objects{i};
        else
            obj = objects;
        end
        all_vertices = [all_vertices; obj.vertices];
    end
    
    max_dim = compute_max_dimension(all_vertices);
    axis_limits = config.AxisScale * max_dim * [-0.65, 0.65];
    
    % Create 3D subplot
    subplot(2, 4, [1, 2, 5, 6]);
    axis equal;
    grid on;
    xlabel('E'); ylabel('N'); zlabel('-D'); % NED reference frame
    view(config.ViewAngle);
    xlim(axis_limits); ylim(axis_limits); zlim(axis_limits);
    hold on;
    
    % Draw inertial reference frame
    draw_fixed_reference_axes(1 * max_dim, 'k', 'NED');
    
    % Initialize objects
    colors = lines(n_objects);
    for i = 1:n_objects
        if iscell(objects)
            obj = objects{i};
        else
            obj = objects;
        end
        
        % Use object color or assign from colormap
        if isequal(obj.color, [0 0.5 0]) && n_objects > 1
            obj.color = colors(i, :);
        end
        
        % Initial orientation
        R0 = eye(3);
        vertices_transformed = (R0 * obj.vertices')';
        
        % Create patch
        object_handles.patches{i} = patch('Faces', obj.faces, ...
                                         'Vertices', vertices_transformed, ...
                                         'FaceColor', obj.color, ...
                                         'FaceAlpha', obj.alpha, ...
                                         'EdgeColor', obj.edgecolor);
        
        % Create reference frame
        [bX, bY, bZ, iTxt, jTxt, kTxt] = draw_moving_reference_axes(R0, 1 * max_dim, obj.color, 'B');
        object_handles.reference_frames{i} = struct('bX', bX, 'bY', bY, 'bZ', bZ, ...
                                                   'iTxt', iTxt, 'jTxt', jTxt, 'kTxt', kTxt);
        
        % Create angular velocity arrow (if data available)
        object_handles.angular_velocity{i} = [];
    end
    
    object_handles.title = title(sprintf('Attitude History Animation\n t = %.2f s', 0));
    object_handles.max_dim = max_dim;
end

function plot_handles = initialize_plots(objects, time_vector, ang_vel_cell, n_objects)
% INITIALIZE_PLOTS Initialize 2D time history plots
    plot_handles = struct();
    colors = lines(4*n_objects);
    
    % Quaternion plot
    subplot(2, 4, [3, 4]);
    hold on; grid on;
    title('Attitude Quaternions');
    xlabel('Time [s]');
    ylabel('Quaternion Components');
    xlim([0, time_vector(end)]);
    
    plot_handles.quaternions = cell(1, n_objects);
    for i = 1:n_objects
        for j = 1:4
            if iscell(objects)
                if isfield(objects{i},'name') && ischar(objects{i}.name)
                    name = objects{i}.name;
                else
                    name = 'Obj';
                end
            else
                name = 'Obj';
            end
            plot_handles.quaternions{i}(j) = plot(NaN, NaN, '-', ...
                'Color', colors((4*i-4)+j, :), 'LineWidth', 1.5, ...
                'DisplayName', sprintf('%s %d: q_%d', name, i, j));
        end
    end
    legend('show', 'NumColumns', 2);
    
    % Angular velocity plot
    plot_handles.angular_velocities = cell(1, n_objects);
    if any(~cellfun(@isempty, ang_vel_cell))
        subplot(2, 4, [7, 8]);
        hold on; grid on;
        title('Angular Velocity (rad/s)');
        xlabel('Time [s]');
        ylabel('\omega [rad/s]');
        xlim([0, time_vector(end)]);
        
        for i = 1:n_objects
            if ~isempty(ang_vel_cell{i})
                for j = 1:3
                    plot_handles.angular_velocities{i}(j) = plot(NaN, NaN, '-', ...
                        'Color', colors((3*i-3)+j, :), 'LineWidth', 1.5, ...
                        'DisplayName', sprintf('Obj %d: \\omega_%d', i, j));
                end
            end
        end
        legend('show', 'NumColumns', 2);
    end
end

function auto_scale_plots(plot_handles, quat_cell, ang_vel_cell, current_index, n_objects)
% AUTO_SCALE_PLOTS Automatically adjust y-axis limits based on visible data

    % Auto-scale quaternion plot
    if current_index > 0
        % Get all quaternion data up to current time
        all_quat_data = [];
        for i = 1:n_objects
            if current_index == 1
                all_quat_data = [all_quat_data, quat_cell{i}(:, 1)];
            else
                all_quat_data = [all_quat_data, quat_cell{i}(:, 1:current_index)];
            end
        end
        
        if ~isempty(all_quat_data)
            y_min = min(all_quat_data, [], 'all');
            y_max = max(all_quat_data, [], 'all');
            y_range = y_max - y_min;
            
            % Add some padding (10% of range)
            padding = y_range * 0.1;
            y_lim = [y_min - padding, y_max + padding];
            
            % Ensure we have valid limits
            if ~isempty(y_lim) && all(isfinite(y_lim)) && y_lim(1) < y_lim(2)
                subplot(2, 4, [3, 4]);
                ylim(y_lim);
            end
        end
    end
    
    % Auto-scale angular velocity plot
    if current_index > 0 && ~isempty(plot_handles.angular_velocities{1})
        % Get all angular velocity data up to current time
        all_omega_data = [];
        for i = 1:n_objects
            if ~isempty(ang_vel_cell{i})
                if current_index == 1
                    all_omega_data = [all_omega_data, ang_vel_cell{i}(:, 1)];
                else
                    all_omega_data = [all_omega_data, ang_vel_cell{i}(:, 1:current_index)];
                end
            end
        end
        
        if ~isempty(all_omega_data)
            y_min = min(all_omega_data, [], 'all');
            y_max = max(all_omega_data, [], 'all');
            y_range = y_max - y_min;
            
            % Add some padding (10% of range)
            padding = y_range * 0.1;
            y_lim = [y_min - padding, y_max + padding];
            
            % Ensure we have valid limits and some minimum range
            if ~isempty(y_lim) && all(isfinite(y_lim)) && y_lim(1) < y_lim(2)
                if diff(y_lim) < 0.1  % Minimum range of 0.1
                    center = mean(y_lim);
                    y_lim = [center - 0.05, center + 0.05];
                end
                subplot(2, 4, [7, 8]);
                ylim(y_lim);
            end
        end
    end
end

function update_time_plots(plot_handles, time_vector, quat_cell, ang_vel_cell, current_index, n_objects)
% UPDATE_TIME_PLOTS Update the 2D time history plots during animation
    % Update quaternion plots for all objects
    for i = 1:n_objects
        if ~isempty(plot_handles.quaternions{i})
            % Update each quaternion component (q0, q1, q2, q3)
            for j = 1:4
                if current_index == 1
                    % Initialize with first point
                    set(plot_handles.quaternions{i}(j), ...
                        'XData', time_vector(1), ...
                        'YData', quat_cell{i}(j, 1));
                else
                    % Update with all points up to current time
                    set(plot_handles.quaternions{i}(j), ...
                        'XData', time_vector(1:current_index), ...
                        'YData', quat_cell{i}(j, 1:current_index));
                end
            end
        end
    end
    
    % Update angular velocity plots for objects that have the data
    for i = 1:n_objects
        if ~isempty(ang_vel_cell{i}) && ~isempty(plot_handles.angular_velocities{i})
            % Update each angular velocity component (ωx, ωy, ωz)
            for j = 1:3
                if current_index == 1
                    % Initialize with first point
                    set(plot_handles.angular_velocities{i}(j), ...
                        'XData', time_vector(1), ...
                        'YData', ang_vel_cell{i}(j, 1));
                else
                    % Update with all points up to current time
                    set(plot_handles.angular_velocities{i}(j), ...
                        'XData', time_vector(1:current_index), ...
                        'YData', ang_vel_cell{i}(j, 1:current_index));
                end
            end
        end
    end
    
    % Auto-scale the y-axis limits based on current data
    auto_scale_plots(plot_handles, quat_cell, ang_vel_cell, current_index, n_objects);
end

function animation_loop(figure3D, video, time_vector, quat_cell, ang_vel_cell,  object_handles, plot_handles, n_objects)
% ANIMATION_LOOP Main animation loop
    for k = 1:length(time_vector)
        % Update 3D visualization for each object
        for i = 1:n_objects
            % Update orientation
            R = quat_to_dcm(quat_cell{i}(:, k));
            
            % Update object patch
            if iscell(object_handles.objects)
                obj = object_handles.objects{i};
            else
                obj = object_handles.objects;
            end
            vertices_transformed = (R * obj.vertices')';
            set(object_handles.patches{i}, 'Vertices', vertices_transformed);
            
            % Update reference frame
            update_reference_axes(object_handles.reference_frames{i}, R, 1 * object_handles.max_dim);
            % Update angular velocity arrow
            if ~isempty(ang_vel_cell{i}) && ~isempty(object_handles.angular_velocity{i})
                update_arrow(object_handles.angular_velocity{i}, R * ang_vel_cell{i}(:, k), 1 * object_handles.max_dim);
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
        fprintf('Processed %d/%d frames\n', k, length(time_vector));
    end
end

function max_dim = compute_max_dimension(vertices)
% COMPUTE_MAX_DIMENSION Calculate maximum dimension of vertices
    ranges = max(vertices) - min(vertices);
    max_dim = max(ranges);
end