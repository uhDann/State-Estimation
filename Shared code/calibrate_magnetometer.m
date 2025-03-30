function [mag_heading]= applyMagnetometerCalibration2D(mag_xy_raw, calParams)
% Unpack the struct
mu          = calParams.mu;
sigma       = calParams.sigma;
offset_norm = calParams.offset_norm;
scaleMatrix = calParams.scaleMatrix;
R_opt       = calParams.R_opt;

% Normalize
% Important to do so because the transformations were done in the
% normalized space of the clibration data.
XY_norm = (mag_xy_raw - mu) ./ sigma;
% Hard iron correction
XY_centered = XY_norm - offset_norm;
% Scale
calibrated_norm = (scaleMatrix * XY_centered')';
% Heading alignment rotation (align to GT yaw frame)
calibrated_rot = (R_opt * calibrated_norm')';
% Denormalize
calibratedNew = (calibrated_rot .* sigma) + mu;
% Recenter (Not sure if necessary)
calibratedNew = calibratedNew - mean(calibratedNew);

%% Convert to yaw
% Yaw for Mag
mag_heading = atan2(calibratedNew(:,2), calibratedNew(:,1));
mag_heading = unwrap(mag_heading);

end
