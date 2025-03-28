function z_meas_tof = ToF_mag_to_meas(ToF_centre, mag_theta)
% Convert ToF and magnetometer data to measurement [x, y, theta]
% Parameters:
%   ToF_centre: Nx3
%       % TODO: confirm this is the correct format
%       Time-of-flight data from the centre of the robot (left, right, front)

%   mag_theta: Nx1
%       Calibrated magnetometer data, heading in radians
% Returns:
%   z_meas_tof: Nx3
%       Measurement data [x, y, theta]

z_meas_tof = zeros(size(ToF_centre, 1), 3);
% Convert ToF data and magentometer heading to [x, y, theta]

d_c = ToF_centre(1, :);
d_l = ToF_centre(2, :);
d_r = ToF_centre(3, :);

% Compute x and y position based on ToF and heading
x = (L/2 - d_r * cos(mag_theta)) - (L/2 - d_l * cos(mag_theta));
y = L/2 - d_c * cos(mag_theta);

z_meas_tof(1, :) = x;
z_meas_tof(2, :) = y;
z_meas_tof(3, :) = mag_theta;

end
