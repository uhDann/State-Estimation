function [X_Est, P_Est, GT] = EKF(out)
% [X_Est, P_Est, GT] = myEKF(out)
% Parameters:
%   out
%     struct with fields:
%     .Sensor_Time
%     .Sensor_ACCEL,
%     .Sensor_GYRO,
%     .Sensor_MAG
%     .Sensor_ToF1, .Sensor_ToF2, .Sensor_ToF3  (distance, status, etc.)
%     .GT_time, .GT_position, .GT_rotation
% Returns:
%   X_Est
%     Array of state estiamtes
%   P_Est
%     Array of covariance matrices
%   GT
%     ground-truth state

% N x 1
% Updates @ 200Hz
tIMU = out.Sensor_Time.time;

% 3 x N
% Updates @ 104Hz
accelRaw = squeeze(out.Sensor_ACCEL.signals.values)';
accelLPRaw = squeeze(out.Sensor_LP_ACCEL.signals.values)';

% 3 x N
% Updates @ 104Hz
gyroRaw  = squeeze(out.Sensor_GYRO.signals.values)';

% 3 x N
% Updates @ 50
magRaw   = squeeze(out.Sensor_MAG.signals.values)';

% N x 4
% Updates @ 10Hz
% [dist, ambient, signal, status]
% Status: 0=Probably too close but OK, 2=Long OK, 4=Short OK
ToF1     = out.Sensor_ToF1.signals.values;
ToF2     = out.Sensor_ToF2.signals.values;
ToF3     = out.Sensor_ToF3.signals.values;

% N x 1
gt_time = out.GT_time.time;
% N x 3
% Updates @ 200Hz
gt_pos   = out.GT_position.signals.values;
% N x 4
% Updates @ 200Hz
gt_quat  = out.GT_rotation.signals.values;

% Allocate arrays
N = length(tIMU);
X_Est_out = zeros(5, N);
P_Est_out = cell(N, 1);

% Initial state (assume first GT position is accurate)
% [x, y, theta, vx, vy]
X_k = [gt_pos(1,1:2), 0, 0, 0]';
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
    % Z in global frame
    omega_z = gyroRaw(k, 1);
    a_x = accelRaw(k, 2);
    a_y = accelRaw(k, 3);
    
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
