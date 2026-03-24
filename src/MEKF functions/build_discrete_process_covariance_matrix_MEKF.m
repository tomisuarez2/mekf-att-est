function Qd = build_discrete_process_covariance_matrix_MEKF(omega, dt, Qc, complete)
% BUILD_DISCRETE_PROCESS_TRANSFORMATION_MATRIX Build Qd matrix
% This implementation assumes equal noise and random bias walk noise on each
% axis of the gyroscope.
% We can use simplified or complete version.
    Rgyro = Qc(1:3,1:3);
    Qbg  = Qc(4:6,4:6);
    Qd = zeros(6);
    Wb = skew(omega);
    dt2 = dt^2;
    dt3 = dt2*dt;
    if complete 
        Qd(1:3,1:3) = 1/4*Rgyro*dt - 5/8*Rgyro*Wb*dt2 + 1/3*Rgyro*Wb^2*dt3;
        Qd(1:3,4:6) = -1/4*Qbg*dt2;
        Qd(4:6,1:3) = Qd(1:3,4:6);
        Qd(4:6,4:6) = Qbg*dt;
    else
        Qd(1:3,1:3) = 1/4*Rgyro*dt;
        Qd(4:6,4:6) = Qbg*dt;
    end
end
