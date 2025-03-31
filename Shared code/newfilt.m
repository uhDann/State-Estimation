clear; clc; clear figure
load("../trainingData/calib1_rotate.mat")
GT_Time = out.Sensor_Time.time;

gyro = squeeze(out.Sensor_GYRO.signals.values)';

accel1 = squeeze(out.Sensor_ACCEL.signals.values)';
accel2 = squeeze(out.Sensor_LP_ACCEL.signals.values)';

% Subtracting the mean to ensure centering
accel1 = (accel1(:,2:3))-mean(accel1(10:200,2:3));
accel2 = accel2(:,[1,3])-mean(accel2(10:200,[1,3]));

% figure;
% plot(GT_Time, accel2); hold on;
% plot(GT_Time, (accel1));
% title('Filtered Accelerometer Data')
% xlabel('Time (s)')
% ylabel('Acceleration (m/s^2)')
% legend("X", 'Y', "X", 'Y')
var1 = var(accel1);
var2 = var(accel2);

% Calculate weights (element-wise, axis by axis)
w1 = var2 ./ (var1 + var2);
w2 = var1 ./ (var1 + var2);

% Fuse each axis individually
accel_fused = w1 .* accel1 + w2 .* accel2;

accel_filtered = zero_phase_smooth(accel_fused, 4, 2, 104);

gyro_filtered = zero_phase_smooth(gyro, 4, 2, 104);

% Plot filtered signals
figure;
subplot(2,1,1)
plot(GT_Time, accel_filtered)
title('Filtered Accelerometer Data')
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
legend("X", 'Y', 'Z')

subplot(2,1,2)
plot(GT_Time, gyro_filtered(:,1))
title('Filtered Gyroscope Data')
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
legend('Z')

time = GT_Time;

% Integrate Z-axis angular velocity to get rotation angle (in radians)
rotation_z = cumtrapz(time, gyro_filtered(:,1)); % Z-axis (yaw)

% Optional: convert to degrees
rotation_z_deg = rad2deg(unwrap(rotation_z));

% Plot cumulative rotation
figure;
plot(time, rotation_z_deg)
title('Cumulative Rotation Around Z-Axis')
xlabel('Time (s)')
ylabel('Rotation (°)')
grid on

velocity = cumtrapz(GT_Time, accel_filtered); 
velocity = velocity - mean(velocity(10:100,:)); 
% displacement = cumtrapz(GT_Time, velocity);

[bh, ah] = butter(4, 0.05/(104/2), 'high');

% Apply zero-phase high-pass filtering
velocity_hp = filtfilt(bh, ah, velocity);
displacement = cumtrapz(GT_Time, velocity_hp);

GT_position = squeeze(out.GT_position.signals.values);

figure;
plot(GT_Time, displacement); hold on;
plot(GT_Time, GT_position(:, 1:2), 'LineWidth', 3, 'Marker','.')
legend('X', 'Y', 'X-GT', 'Y-GT')
xlabel('Time (s)')
ylabel('Displacement (m)')
title('Estimated Displacement Over Time')
grid on
