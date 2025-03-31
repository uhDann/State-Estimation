function [X_k, P_k] = UKF_update(X_prev, P_prev, omega_z, a_x, a_y, z, Q, R, dt)
% UKF_UPDATE  Unscented Kalman Filter prediction + update
%
% Inputs:
%   X_prev  - state at previous step, [5x1]: [x, y, theta, vx, vy]
%   P_prev  - covariance [5x5]
%   omega_z - gyro Z reading
%   a_x     - calibrated accel in x
%   a_y     - calibrated accel in y
%   z       - measurement vector, e.g. [x_meas; y_meas; heading_meas]
%   Q       - process noise covariance
%   R       - measurement noise covariance
%   dt      - time step
%
% Outputs:
%   X_k     - updated state
%   P_k     - updated covariance

    % Dimensions
    L = numel(X_prev);   % state dimension = 5
    m = numel(z);        % measurement dimension, e.g. = 3

    %--------------------------------------------------------------------------
    % 1) Generate sigma points around (X_prev, P_prev)
    %--------------------------------------------------------------------------

    % Unscented transform parameters
    alpha = 1e-3;
    beta  = 2;
    kappa = 3-L;           % or use 3-L, etc., depends on preference
    lambda = alpha^2 * (L + kappa) - L;

    % Compute square root of P_prev (e.g., via Cholesky)
    S = chol(P_prev, 'lower');

    % Scale factor for sigma points
    c = sqrt(L + lambda);

    % Sigma points: X_sigma will be size [L x (2L+1)]
    % Columns: [X_prev, X_prev +/- c * each column of S]
    X_sigma = zeros(L, 2*L+1);
    X_sigma(:,1) = X_prev;
    for i = 1:L
        X_sigma(:,i+1)     = X_prev + c * S(:,i);
        X_sigma(:,i+1+L)   = X_prev - c * S(:,i);
    end

    % Weights for mean and covariance
    W_m = zeros(2*L+1,1);
    W_c = zeros(2*L+1,1);

    W_m(1) = lambda / (L + lambda);
    W_c(1) = lambda / (L + lambda) + (1 - alpha^2 + beta);
    for i = 2:2*L+1
        W_m(i) = 1/(2*(L + lambda));
        W_c(i) = 1/(2*(L + lambda));
    end

    %--------------------------------------------------------------------------
    % 2) Predict step: pass sigma points through process model f()
    %--------------------------------------------------------------------------
    % X_sigma_pred will hold the propagated sigma points
    X_sigma_pred = zeros(L, 2*L+1);
    for i = 1:(2*L+1)
        X_sigma_pred(:,i) = f_process(X_sigma(:,i), omega_z, a_x, a_y, dt);
    end

    % Predicted mean
    x_pred = zeros(L,1);
    for i = 1:(2*L+1)
        x_pred = x_pred + W_m(i) * X_sigma_pred(:,i);
    end

    % Predicted covariance
    P_pred = zeros(L);
    for i = 1:(2*L+1)
        dx = X_sigma_pred(:,i) - x_pred;
        % Handle angle wrapping if needed (theta ~ wrap to +/- pi)
        dx(3) = wrapToPi(dx(3)); 
        P_pred = P_pred + W_c(i) * (dx * dx');
    end
    P_pred = P_pred + Q;  % Add process noise

    %--------------------------------------------------------------------------
    % 3) Update step: pass predicted sigma points through measurement model h()
    %--------------------------------------------------------------------------
    Z_sigma = zeros(m, 2*L+1);
    for i = 1:(2*L+1)
        Z_sigma(:,i) = h_measurement(X_sigma_pred(:,i));
    end

    % Predicted measurement mean
    z_pred = zeros(m,1);
    for i = 1:(2*L+1)
        z_pred = z_pred + W_m(i) * Z_sigma(:,i);
    end

    % Innovation covariance
    Szz = zeros(m);
    for i = 1:(2*L+1)
        dz = Z_sigma(:,i) - z_pred;
        % If yaw is in measurement, wrap as necessary
        dz(3) = wrapToPi(dz(3));
        Szz = Szz + W_c(i) * (dz * dz');
    end
    Szz = Szz + R;

    % Cross covariance Pxz
    Pxz = zeros(L,m);
    for i = 1:(2*L+1)
        dx = X_sigma_pred(:,i) - x_pred;
        dx(3) = wrapToPi(dx(3));  % handle angle
        dz = Z_sigma(:,i) - z_pred;
        dz(3) = wrapToPi(dz(3));
        Pxz = Pxz + W_c(i) * (dx * dz');
    end

    % Kalman gain
    K = Pxz / Szz;

    % Measurement residual
    z_res = z - z_pred;
    z_res(3) = wrapToPi(z_res(3));  % wrap heading residual

    % Updated state mean and covariance
    X_k = x_pred + K * z_res;
    % If needed, wrap angle
    X_k(3) = wrapToPi(X_k(3));

    P_k = P_pred - K * Szz * K.';
end

function X_next = f_process(X, omega_z, a_x, a_y, dt)
    % X = [x, y, theta, vx, vy]

    x     = X(1);
    y     = X(2);
    theta = X(3);
    vx    = X(4);
    vy    = X(5);

    % You may assume heading changes at rate omega_z
    theta_next = theta + omega_z * dt;

    % Acc in global frame => rotate (a_x, a_y) by theta
    ax_global = a_x*cos(theta) - a_y*sin(theta);
    ay_global = a_x*sin(theta) + a_y*cos(theta);

    % Next velocities
    vx_next = vx + ax_global*dt;
    vy_next = vy + ay_global*dt;

    % Next position
    x_next = x + vx*dt; 
    y_next = y + vy*dt;

    X_next = [ x_next;
               y_next;
               wrapToPi(theta_next);
               vx_next;
               vy_next ];
end

function z_est = h_measurement(X)
    % X = [x, y, theta, vx, vy]
    % z_est = [x; y; theta]
    z_est = [ X(1);
              X(2);
              X(3) ];
end