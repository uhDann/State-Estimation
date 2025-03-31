%% Loading data
clear; clc;
% load("../trainingData/calib1_rotate.mat");
% load("../trainingData/calib2_straight.mat")
load("../trainingData/task2_3.mat")
% load("../trainingData/task1_2.mat")
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

%% EKF
[X_est, P_Est, GT] = EKF(out);
X_est(:, 1:2) = zero_phase_smooth(X_est(:, 1:2), 4, 0.6, 200);

plot_trajectory(GT_Time, GT_position, X_est(:, 1:2), X_est(:, 3), all_ToF, false);

[RMSE, metrics] = evaluateTrajectory(X_est, out);

%% Plotting
plot_var_xy = 2;

figure;
hold on;

% Plot GT_position and X_est on the left y-axis
yyaxis left;
plot(GT_position(:, plot_var_xy), 'b--', 'LineWidth', 2); % Ground Truth
plot(X_est(:, plot_var_xy), 'r', 'LineWidth', 2);         % Estimated Position
ylabel('Position');
legend({'GT Position', 'X Est'}, 'Location', 'best');

% Compute the RMSE
error_x = (GT_position(:, 1) - X_est(:, 1)).^2;
error_y = (GT_position(:, 2) - X_est(:, 2)).^2;
error_theta = (GT_heading - X_est(:, 3)).^2;

error = [error_x, error_y, error_theta];

% Plot the RMSE on the right y-axis
yyaxis right;
plot(error(:, plot_var_xy), 'k-', 'LineWidth', 2); % RMSE in black
ylabel('RMSE');

% Labels and title
xlabel('Time Step');
title('GT Position, Estimated Position, and RMSE');
grid on;
