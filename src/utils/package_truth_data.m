function truth = package_truth_data(time, quat, acc, ang_vel, gyro_bias)
% PACKAGE_TRUTH_DATA Package ground truth data into structure
    truth.time = time;
    truth.quat = quat;
    truth.acc = acc;
    truth.ang_vel = ang_vel;
    truth.gyro_bias = gyro_bias;
end