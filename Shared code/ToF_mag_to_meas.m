function pos_est = ToF_mag_to_meas(tof_distances, yaws)
% Estimates the (x,y) position of robot with ToF sensors and magnetometer using optimisation.
% Parameters:
%   tof_distances : Nx3
%       Columns:
%           1 - left sensor (mounted at yaw+pi/2)
%           2 - centre sensor (mounted at yaw+pi)
%           3 - right sensor (mounted at yaw-pi/2)
%   yaws: Nx1
%       Vector of robot heading angles (in radians). Yaw is positive counter-clockwise and is 0 when the robot is pointing towards the right wall.
% Returns:
%   pos_est: Nx2
%       Array with estimated [x, y] positions.

L = 2.4;

% Number of time steps
N = size(tof_distances,1);

pos_est = zeros(N,2);

options = optimset('Display','off');

for n = 1:N
    theta = yaws(n);
    
    % Sensor effective angles (left, centre, right)
    sensorAngles = [theta + pi/2, theta+pi, theta - pi/2];
    
    tof = tof_distances(n,:);
    
    % Given a candidate position pos = [x, y], compute the sum of squared errors.
    costFun = @(pos) computeCost(pos, sensorAngles, tof, L);
    
    % Guess. Use the previous estimate if available, otherwise the room center.
    if n == 1
        x0 = [0, 0];
    else
        x0 = pos_est(n-1,:);
    end
    
    % Run the optimization for current time step.
    pos_fit = fminsearch(costFun, x0, options);
    
    % Force the solution to lie within the room
    pos_fit(1) = min(max(pos_fit(1), -L/2), L/2);
    pos_fit(2) = min(max(pos_fit(2), -L/2), L/2);
    
    pos_est(n,:) = pos_fit;
end

end

% -------------------------------------------------------------------------
function cost = computeCost(pos, sensorAngles, tof, L)
% computeCost returns the sum of squared differences between the predicted and measured distances for a candidate position pos = [x, y].
%
% Parameters:
%   pos: 1x2
%       candidate position [x, y]
% sensorAngles: 1x3
%   Array of effective sensor angles for [left, centre, right]
% tof: 1x3
%   Array of measured distances for [left, centre, right]
% L: double
%   room side length

x = pos(1);
y = pos(2);

cost = 0;

% For each sensor, compute the predicted distance
for i = 1:length(sensorAngles)
    phi = sensorAngles(i);
    d_pred = predictedDistance(x, y, phi, L);
    cost = cost + (d_pred - tof(i))^2;
end

% Impose a penalty if the candidate is outside the room boundaries
penalty = 0;
if abs(x) > L/2
    penalty = penalty + 1e6*(abs(x)-L/2)^2;
end
if abs(y) > L/2
    penalty = penalty + 1e6*(abs(y)-L/2)^2;
end

cost = cost + penalty;

end

% -------------------------------------------------------------------------
function d = predictedDistance(x, y, phi, L)
% Computes the distance from point (x,y) along the direction phi to the first wall encountered in square room with side lengths L.
% Parameters:
%   x: double
%       x-coordinate of the point
%   y: double
%       y-coordinate of the point
%   phi: double
%       direction angle in radians
%   L: double
%       room side length
% Returns:
%   d: double
%       distance to the wall

% Tolerance for near-zero
tol = 1e-6;

% Vertical wall candidate
if abs(cos(phi)) > tol
    % Determine which vertical wall is hit based on the direction
    wall_x = sign(cos(phi)) * (L/2);
    d_x = (wall_x - x) / cos(phi);
else
    d_x = inf;
end

% Horizontal wall candidate
if abs(sin(phi)) > tol
    % Determine which horizontal wall is hit
    wall_y = sign(sin(phi)) * (L/2);
    d_y = (wall_y - y) / sin(phi);
else
    d_y = inf;
end

% Only consider positive distances
if d_x <= 0
    d_x = inf;
end
if d_y <= 0
    d_y = inf;
end

% Predicted distance is the minimum positive candidate
d = min(d_x, d_y);

end
