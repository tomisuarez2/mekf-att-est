function save_generated_data(truth, sensors, config)
% SAVE_GENERATED_DATA Save generated data to files

    timestamp = string(datetime('now', 'Format','uuuu_dd_MM-HH_mm_ss'));
    
    % Save main data file
    filename = sprintf('../src/sensor and real data/truth_sensor_data_%s.mat', timestamp);
    save(filename, 'truth', 'sensors', 'config', '-v7.3');
    fprintf('>>> Data saved to: %s\n', filename);
    
end