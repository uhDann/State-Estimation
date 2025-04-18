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

%% Load data

% N x 1
% Updates @ 200Hz
tIMU = out.Sensor_Time.time;

% N x 3
% Updates @ 104Hz
accel_raw = squeeze(out.Sensor_ACCEL.signals.values)';
accel_raw = interpolate_sensor(accel_raw, 200, 104);

accel_LP_raw = squeeze(out.Sensor_LP_ACCEL.signals.values)';
accel_LP_raw = interpolate_sensor(accel_LP_raw, 200, 100);

accel_calibrated = calibrate_accel(accel_raw, accel_LP_raw);

% N x 4
% Updates @ 200Hz
GT_rotation = squeeze(out.GT_rotation.signals.values);
GT_heading = quat2eul(GT_rotation, 'ZYX');
GT_heading = unwrap(GT_heading(:,1));

% N x 3
% Updates @ 104Hz
gyro_raw = squeeze(out.Sensor_GYRO.signals.values)';
gyro_raw = interpolate_sensor(gyro_raw, 200, 104);

gyro_calibrated = calibrate_gyro(gyro_raw);

gyro_yaw = cumtrapz(tIMU, gyro_calibrated(:, 1));
gyro_yaw = unwrap(gyro_yaw) + (GT_heading(1) - gyro_yaw(1));

% N x 3
% Updates @ 50
mag_raw = squeeze(out.Sensor_MAG.signals.values)';
% N x 2
% WARN: magnetometer xy is columns 2 and 3
mag_xy_raw = mag_raw(:, 2:3);
mag_xy_raw = interpolate_sensor(mag_xy_raw, 200, 50);

% N x 1
% Heading in radians for each timestep
% calParams = load("MAG_calParams.mat").calParams;
modelPath = "NNMagCal_2D.mat";
[mag_yaw, ~] = applyNNMagnetometerCalibration(mag_xy_raw, modelPath);

% modelPath = "MAG_calParams.mat";
% mag_yaw = applyMagnetometerCalibration(mag_xy_raw, modelPath);

% mag_yaw = unwrap(atan2(mag_xy_raw(:, 2), mag_xy_raw(:, 1)));

mag_yaw = zero_phase_smooth(mag_yaw, 4, 0.5, 50);

% yaw = mag_yaw;
yaw = gyro_yaw;

% N x 4
% Updates @ 10Hz
% [dist, ambient, signal, status]
% Status: 0=Probably too close but OK, 2=Long OK, 4=Short OK
% Left, centre (backwards), right
ToF1 = out.Sensor_ToF1.signals.values;
ToF2 = out.Sensor_ToF2.signals.values;
ToF3 = out.Sensor_ToF3.signals.values;

ToF1 = interpolate_sensor(ToF1, 200, 10);
ToF2 = interpolate_sensor(ToF2, 200, 10);
ToF3 = interpolate_sensor(ToF3, 200, 10);

all_ToF = remove_outliers(calibrate_ToF([ToF1(:, 1), ToF2(:, 1), ToF3(:, 1)]), 500);

% Extract ground truth data
% N x 1
gt_time = out.GT_time.time;
% N x 3
% Updates @ 200Hz
gt_pos   = out.GT_position.signals.values;
% N x 4
% Updates @ 200Hz
gt_quat  = out.GT_rotation.signals.values;

% Convert quaternion to Euler angles
gt_eul = quat2eul(gt_quat, "XYZ");
gt_heading = gt_eul(:, 3);

% Allocate arrays
N = length(tIMU);
X_Est_out = zeros(5, N);
P_Est_out = cell(N, 1);

dt = 1/200;

z_meas_tof = [remove_outliers(ToF_mag_to_meas(all_ToF, yaw), 650), yaw];

% Initial state (assume first GT position is accurate)
% [x, y, theta, vx, vy]
X_k = [z_meas_tof(1, 1), z_meas_tof(1, 2), yaw(1), 0, 0]';
% Evaluation showed that RMSE of magnetometer heading is 0.28
P_k = diag([0.1, 0.1, 0.28, 0.01, 0.01]);

% Process noise covariance
Q = diag([0.1, 0.1, 0.17, 0.1, 0.1]*0.05);
% Standard deviation of
R = diag([0.17, 0.18, 0.1].^2);

X_Est_out(:, 1) = X_k;

for k = 2:N
    % Get sensor measurements
    % Z in global frame
    omega_z = gyro_calibrated(k, 1);
    a_x = accel_calibrated(k, 1);
    a_y = accel_calibrated(k, 2);
    
    % EKF Prediction and Update
    [X_k, P_k] = EKF_update(X_k, P_k, omega_z, a_x, a_y, z_meas_tof(k, :)', Q, R, true, dt);
    
    % Store results
    X_Est_out(:, k) = X_k;
    P_Est_out{k} = P_k;
end

GT = [gt_time, gt_pos, gt_quat];
X_Est = X_Est_out.';
P_Est = P_Est_out;
end
