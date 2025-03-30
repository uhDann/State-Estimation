clear; clc;
% out = load("../trainingData/calib1_rotate.mat").out;
load("../trainingData/task1_1.mat")
% load("trainingData/task1_1.mat")
% load("trainingData/task1_2.mat")
% load("trainingData/task1_3.mat")
% load("trainingData/task1_4.mat")
% load("trainingData/task1_5.mat")

% N x 3
% Updates @ 104Hz
accelRaw = squeeze(out.Sensor_ACCEL.signals.values)';
accelLPRaw = squeeze(out.Sensor_LP_ACCEL.signals.values)';

% N x 3
% Updates @ 104Hz
gyroRaw  = squeeze(out.Sensor_GYRO.signals.values)';

% N x 3
% Updates @ 50
magRaw   = squeeze(out.Sensor_MAG.signals.values)';

% N x 1
% Heading in radians for each timestep
magCal = calibrate_magnetometer(magRaw);


% N x 4
% Updates @ 10Hz
% [dist, ambient, signal, status]
% Status: 0=Probably too close but OK, 2=Long OK, 4=Short OK
ToF1     = out.Sensor_ToF1.signals.values;
ToF2     = out.Sensor_ToF2.signals.values;
ToF3     = out.Sensor_ToF3.signals.values;

all_ToF = [ToF1(:, 1), -ToF2(:, 1), ToF3(:, 1)];

% N x 1
gt_time = out.GT_time.time;
% N x 3
% Updates @ 200Hz
gt_pos   = out.GT_position.signals.values;
% N x 4
% Updates @ 200Hz
gt_quat  = out.GT_rotation.signals.values;

gt_eul = quat2eul(gt_quat, "XYZ");
heading = gt_eul(:, 3);

plot_trajectory(gt_time, gt_pos, heading, all_ToF, true)
% EKF(out)
