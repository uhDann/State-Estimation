function [X_k, P_k] = EKF_update(X_k, P_k, omega_z, a_x, a_y, z_tof, Q, R, dt)
% State transition model
F = eye(5) + dt * [
    0, 0, -X_k(5), 1, 0;
    0, 0,  X_k(4), 0, 1;
    0, 0, 0, 0, 0;
    0, 0, 0, 0, 0;
    0, 0, 0, 0, 0
    ];

% Control input (IMU-based motion model)
X_k(1) = X_k(1) + X_k(4) * dt;
X_k(2) = X_k(2) + X_k(5) * dt;
X_k(3) = X_k(3) + omega_z * dt;
X_k(4) = X_k(4) + a_x * dt;
X_k(5) = X_k(5) + a_y * dt;

% Prediction step
P_k = F * P_k * F' + Q;

% Measurement model (ToF simplification)
H = [1, 0, 0, 0, 0;
    0, 1, 0, 0, 0;
    0, 0, 1, 0, 0];
z_pred = H * X_k;
y = z_tof - z_pred;

% Compute Kalman Gain
S = H * P_k * H' + R;
K = P_k * H' / S;

% Update step
X_k = X_k + K * y;
P_k = (eye(5) - K * H) * P_k;
end
