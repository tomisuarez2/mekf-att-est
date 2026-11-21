function data = extract_trajectory_data(raw_data)
% EXTRACT_TRAJECTORY_DATA Extract and reshape trajectory data.
% Converts from 3×1×N to 3×N format.
    data = squeeze(raw_data);
    if size(data, 1) ~= 3
        data = data';
    end
end