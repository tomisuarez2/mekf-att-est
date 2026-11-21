function interpolated = interpolate_data(original_time, original_data, new_time)
    % INTERPOLATE_DATA Interpolate data to new time vector.
    interpolated = zeros(size(original_data, 1), length(new_time));
    for i = 1:size(original_data, 1)
        interpolated(i, :) = interp1(original_time, original_data(i, :), new_time, 'linear', 'extrap');
    end
end