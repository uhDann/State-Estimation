clear; clc;
% load("../trainingData/calib1_rotate.mat");
% load("../trainingData/calib2_straight.mat")
load("../trainingData/task2_2.mat")
% load("trainingData/task1_2.mat")
% load("../trainingData/task1_3.mat")
% load("trainingData/task1_4.mat")
% load("trainingData/task1_5.mat")

ToF1 = out.Sensor_ToF1.signals.values;
ToF2 = out.Sensor_ToF2.signals.values;
ToF3 = out.Sensor_ToF3.signals.values;

all_ToF = [ToF1(:, 1), ToF2(:, 1), ToF3(:, 1)];

% plot_trajectory(gt_time, gt_pos, heading, all_ToF, true)
[X_est, P_Est, GT] = EKF(out);
gt_time = GT(:, 1);
plot_trajectory(gt_time, X_est(:, 1:2), X_est(:, 3), all_ToF, true);

% pos_est = ToF_mag_to_meas(smoothed_ToF, heading);
% plot_trajectory(gt_time, pos_est, heading, smoothed_ToF, true);
