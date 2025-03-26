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
tIMU = out.tIMU.time;

% 1 x 3 x N
% Updates @ 104Hz
accelRaw = out.Sensor_ACCEL.signals.values;
accelLPRaw = out.Sensor_LP_ACCEL.signals.values;

% 1 x 3 x N
% Updates @ 104Hz
gyroRaw  = out.Sensor_GYRO;

% 1 x 3 x N
% Updates @ 50
magRaw   = out.Sensor_MAG;

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
X_Est_out = zeros(5,N);
P_Est_out = cell(N,1);

% Initialise current position and orientation (quaternion)
% X = [X, Y, Z, W, Wx, Wy, Wz]'
X_k = zeros(5,1);

% Covariance matrix
P_k = eye(5)*0.5;

Q = diag([1e-3, 1e-3, 1e-4, 1e-2, 1e-2]);

prevTime = tIMU(1);

for k = 2:N
    currentTime = tIMU(k);
    dt = currentTime - prevTime;
    % ???
end

GT = [gt_time, gt_pos, gt_quat];

X_Est = X_Est_out.';
P_Est = P_Est_out;
end

