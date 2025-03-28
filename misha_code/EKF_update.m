function [X_k, P_k] = EKF_update(X_k, P_k, omega_z, a_x_body, a_y_body, z_tof, Q, R, dt)
% X_k = [x, y, theta, vx, vy]
% State transition model


theta    = X_k(3);
cos_th   = cos(theta);
sin_th   = sin(theta);

% a_x, a_y in the WORLD frame
a_x_world =  cos_th * a_x_body - sin_th * a_y_body;
a_y_world =  sin_th * a_x_body + cos_th * a_y_body;

dVx_dTheta = dt * ( -sin_th*a_x_body - cos_th*a_y_body );
dVy_dTheta = dt * (  cos_th*a_x_body - sin_th*a_y_body );

% a_x_world and a_y_world depend on cos(theta) and sin(theta)).
% vx_{k+1} - vx_k = dt*( a_x_world ),
%     => d(vx_{k+1})/dtheta = dt * [ -sin(th)*a_x_body - cos(th)*a_y_body ]
% vy_{k+1} - vy_k = dt*( a_y_world )
%     => d(vy_{k+1})/dtheta = dt * [  cos(th)*a_x_body - sin(th)*a_y_body ]
F = eye(5) + [
    0, 0,          0, dt,  0;
    0, 0,          0,  0, dt;
    0, 0,          0,  0,   0;
    0, 0, dVx_dTheta,  0,   0;
    0, 0, dVy_dTheta,  0,   0
    ];


% Prediction step
X_k(1) = X_k(1) + X_k(4) * dt;
X_k(2) = X_k(2) + X_k(5) * dt;
X_k(3) = X_k(3) + omega_z * dt;
X_k(4) = X_k(4) + a_x_world * dt;
X_k(5) = X_k(5) + a_y_world * dt;

% Predict covariance
P_k = F * P_k * F' + Q;

% Measurement update
% Assume z_meas = [x, y, theta].
% H picks out x, y, theta from the 5D state:
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
