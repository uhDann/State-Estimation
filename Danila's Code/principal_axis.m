% Principal Axis (Whitened) Calibration
rotation = load("trainingData/calib1_rotate.mat");
GT_rotation = squeeze(rotation.out.GT_rotation.signals.values);
time = squeeze(rotation.out.GT_time.time);
Mraw = squeeze(rotation.out.Sensor_MAG.signals.values)';

Mx   = Mraw(:,2);
My   = Mraw(:,3);
scaleFactor = 1e5;

% Combine for 2D
XY_raw = [Mx, My];
XY_raw_scaled = XY_raw * scaleFactor;
cx = mean(XY_raw(:,1));
cy = mean(XY_raw(:,2));

% Subtract offset
XY_centered = XY_raw - [cx, cy];

%--------------------------------------------------
% 3) COMPUTE 2x2 COVARIANCE & EIGEN-DECOMPOSITION
%--------------------------------------------------
% Covariance of the centered data
C = cov(XY_centered);  % 2x2

% Eigen-decomposition
[V, D] = eig(C);  % C = V * D * V'
% D has the variances along principal axes
lam = diag(D);

% If lam(1) or lam(2) is near zero, there's almost no spread in that axis
% => can cause degeneracy. But usually you'll have positive values.

%--------------------------------------------------
% 4) SOFT-IRON MATRIX = inv( sqrt(C) )
%--------------------------------------------------
% The matrix that "circularizes" the data is:
%    A_2x2 = V * diag(1./sqrt(lam)) * V'
%
% So that XY_centered * A_2x2 has identity covariance ( ~ unit circle ).
A_2x2 = V * diag(1./sqrt(lam)) * V';

%--------------------------------------------------
% 5) APPLY CALIBRATION
%--------------------------------------------------
XY_calibrated = XY_centered * A_2x2;

%--------------------------------------------------
% 6) PRINT & PLOT RESULTS
%--------------------------------------------------
fprintf('Hard-iron offset = (%.6f, %.6f)\n', cx, cy);
fprintf('Soft-iron matrix (2x2):\n');
disp(A_2x2);

figure;
plot(XY_raw(:,1), XY_raw(:,2), 'o');
axis equal; grid on;
xlabel('M_x (raw)'); ylabel('M_y (raw)');
title('Raw Magnetometer Data (XY only)');

figure;
plot(XY_calibrated(:,1), XY_calibrated(:,2), 'o');
axis equal; grid on;
xlabel('M_x (cal)'); ylabel('M_y (cal)');
title('Calibrated (Whitened) Magnetometer Data');

% Example heading
heading_rad = atan2(XY_calibrated(:,2), XY_calibrated(:,1));
heading_deg = rad2deg(heading_rad);
fprintf('First 5 headings [deg]:\n');
disp(heading_deg(1:5));