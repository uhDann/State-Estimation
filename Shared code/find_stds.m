clear; clc;
load("../trainingData/calib1_rotate.mat");
% load("../trainingData/calib2_straight.mat")
% load("../trainingData/task2_1.mat")
% load("trainingData/task1_2.mat")
% load("../trainingData/task1_3.mat")
% load("trainingData/task1_4.mat")
% load("trainingData/task1_5.mat")

% N x 1
% Updates @ 200Hz
tIMU = out.Sensor_Time.time;

% N x 3
% Updates @ 104Hz
accel_raw = squeeze(out.Sensor_ACCEL.signals.values)';
accel_LP_raw = squeeze(out.Sensor_LP_ACCEL.signals.values)';
accel_calibrated = calibrate_accel(accel_raw, accel_LP_raw);

% N x 3
% Updates @ 104Hz
gyro_raw = squeeze(out.Sensor_GYRO.signals.values)';
gyro_calibrated = calibrate_gyro(gyro_raw);

% N x 3
% Updates @ 50
mag_raw = squeeze(out.Sensor_MAG.signals.values)';
% N x 2
% WARN: magnetometer xy is columns 2 and 3
mag_xy_raw = mag_raw(:, 2:3);

% N x 1
% Heading in radians for each timestep
% calParams = load("MAG_calParams.mat").calParams;
modelPath = "NNMagCal_2D.mat";
[mag_yaw, ~] = applyNNMagnetometerCalibration(mag_xy_raw, modelPath);

% N x 4
% Updates @ 10Hz
% [dist, ambient, signal, status]
% Status: 0=Probably too close but OK, 2=Long OK, 4=Short OK
% Left, centre (backwards), right
ToF1 = out.Sensor_ToF1.signals.values;
ToF2 = out.Sensor_ToF2.signals.values;
ToF3 = out.Sensor_ToF3.signals.values;

all_ToF = calibrate_ToF([ToF1(:, 1), ToF2(:, 1), ToF3(:, 1)]);

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

dt = 200;

z_meas_tof = [ToF_mag_to_meas(all_ToF, mag_yaw), mag_yaw];

sigma_error_x = std(z_meas_tof(:, 1) - gt_pos(:, 1));
sigma_error_y = std(z_meas_tof(:, 2) - gt_pos(:, 2));
disp("Standard deviation of error in x: " + sigma_error_x);
disp("Standard deviation of error in y: " + sigma_error_y);

