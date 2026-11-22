function dt = calculate_time_step(time_vector, current_index)
% CALCULATE_TIME_STEP Calculate time step between measurements.

    if current_index == 1
        if length(time_vector) > 1
            dt = time_vector(2) - time_vector(1);
        else
            dt = 0.01;  % Default time step
        end
    else
        dt = time_vector(current_index) - time_vector(current_index - 1);
    end
    
    % Validate time step
    if dt <= 0
        warning('Non-positive time step detected at index %d. Using default.', current_index);
        dt = 0.01;
    end
    
end