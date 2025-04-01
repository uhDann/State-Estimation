function [yaw_est, heading_vec] = applyNNMagnetometerCalibration(XY_raw, modelPath)
% Applies trained NN magnetometer calibration to new data
%
% Inputs:
%   XY_raw   - Nx2 matrix of raw magnetometer readings [Mx, My]
%   modelPath - Path to saved .mat model file (e.g. 'NNMagCal_2D.mat')
%
% Outputs:
%   yaw_est     - Nx1 vector of predicted yaw (radians)
%   heading_vec - Nx2 normalized heading vectors [cos(yaw), sin(yaw)]

    % Load the trained model and normalization parameters
    data = load(modelPath, 'NNMagCal');
    net   = data.NNMagCal.net;
    mu    = data.NNMagCal.mu;
    sigma = data.NNMagCal.sigma;

    % Normalize input
    X_norm = (XY_raw - mu) ./ sigma;

    % Predict heading vectors
    heading_vec = net(X_norm')';  % Output Nx2

    % Convert to yaw angle
    yaw_est = unwrap(atan2(heading_vec(:,2), heading_vec(:,1)));
end
