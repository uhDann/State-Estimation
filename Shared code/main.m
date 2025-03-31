clear; clc;
% load("../trainingData/calib1_rotate.mat");
% load("../trainingData/calib2_straight.mat")
load("../trainingData/task2_3.mat")
% load("trainingData/task1_2.mat")
% load("../trainingData/task1_3.mat")
% load("trainingData/task1_4.mat")
% load("trainingData/task1_5.mat")

GT_Time = out.Sensor_Time.time;

GT_position = squeeze(out.GT_position.signals.values);
GT_rotation = squeeze(out.GT_rotation.signals.values);

GT_heading = quat2eul(GT_rotation, 'ZYX'); 
GT_heading = unwrap(GT_heading(:,1));

ToF1 = out.Sensor_ToF1.signals.values;
ToF2 = out.Sensor_ToF2.signals.values;
ToF3 = out.Sensor_ToF3.signals.values;

all_ToF = calibrate_ToF([ToF1(:, 1), ToF2(:, 1), ToF3(:, 1)]);

% plot_trajectory(GT_Time, GT_position, GT_heading, all_ToF, true)

[X_est, P_Est, GT] = EKF(out);
% X_est = zero_phase_smooth(X_est, 4, 0.5, 200);
plot_trajectory(GT_Time, GT_position, X_est(:, 1:2), X_est(:, 3), all_ToF, true);

pos_est = ToF_mag_to_meas(all_ToF, GT_heading);
% plot_trajectory(GT_Time, pos_est, GT_heading, all_ToF, true);

[RMSE, metrics] = evaluateTrajectory(X_est, out);

figure;
plot(metrics.AbsoluteError)