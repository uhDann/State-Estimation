clear; clc; clear figure
load("../trainingData/task1_3.mat")
GT_Time = out.Sensor_Time.time;

accel = zero_phase_smooth(squeeze(out.Sensor_ACCEL.signals.values)');
gyro = zero_phase_smooth(squeeze(out.Sensor_GYRO.signals.values)');


figure;
subplot(2,1,1)
plot(GT_Time, accel(:,2:3))
title('Accelerometer Data')
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
legend('X', 'Y', 'Z')

% Plot gyroscope data
subplot(2,1,2)
plot(GT_Time, gyro(:,3))
title('Gyroscope Data')
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
legend('X', 'Y', 'Z')