function [X_Est, P_Est, GT] = EKF(out)
% Extract sensor data
tIMU = out.tIMU.time;
accelRaw = out.Sensor_ACCEL.signals.values;
gyroRaw  = out.Sensor_GYRO.signals.values;
ToF1     = out.Sensor_ToF1.signals.values;
ToF2     = out.Sensor_ToF2.signals.values;
ToF3     = out.Sensor_ToF3.signals.values;
gt_time  = out.GT_time.time;
gt_pos   = out.GT_position.signals.values;
gt_quat  = out.GT_rotation.signals.values;

% Allocate arrays
N = length(tIMU);
X_Est_out = zeros(5, N);
P_Est_out = cell(N, 1);

% Initial state (assume first GT position is accurate)
X_k = [gt_pos(1,1:2), 0, 0, 0]';  % [x, y, theta, vx, vy]
P_k = eye(5) * 0.5;

% Process noise covariance
Q = diag([1e-3, 1e-3, 1e-4, 1e-2, 1e-2]);
R = diag([0.01, 0.01, 0.1]); % Measurement noise for ToF

prevTime = tIMU(1);

for k = 2:N
    currentTime = tIMU(k);
    dt = currentTime - prevTime;
    prevTime = currentTime;
    
    % Get sensor measurements
    omega_z = gyroRaw(k, 3); % Yaw rate from gyroscope
    a_x = accelRaw(k, 1);
    a_y = accelRaw(k, 2);
    
    % ToF readings (distances)
    z_tof = [ToF1(k,1); ToF2(k,1); ToF3(k,1)];
    
    % EKF Prediction and Update
    [X_k, P_k] = ekf_update(X_k, P_k, omega_z, a_x, a_y, z_tof, Q, R, dt);
    
    % Store results
    X_Est_out(:, k) = X_k;
    P_Est_out{k} = P_k;
end

GT = [gt_time, gt_pos, gt_quat];
X_Est = X_Est_out.';
P_Est = P_Est_out;
end
