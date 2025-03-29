function plot_trajectory(time, pos, heading, animate)
% Plots the 2D trajectory of the robot
%
% Parameters:
%   time: Nx1
%       Vector of time steps
%   pos: Nx3
%       Matrix of positions (columns: x, y, z)
%   heading: Nx1
%       Vector of headings (radians)
%   animate - (optional) boolean
%       Flag to animate the robot

x = pos(:, 1);
y = pos(:, 2);

figure;
hold on;
grid on;
axis equal;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Robot Trajectory');
xlim([min(x) - 0.1, max(x) + 0.1]);
ylim([min(y) - 0.1, max(y) + 0.1]);
plot(x, y, 'b-', 'LineWidth', 1.5);

% Start
plot(x(1), y(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
% End
plot(x(end), y(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

if animate
    robot_marker = plot(x(1), y(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    heading_arrow = quiver(x(1), y(1), 0.1*cos(heading(1)), 0.1*sin(heading(1)), 0.3, 'r', 'LineWidth', 2);
    
    for k = 1:length(time)
        % Update robot marker position
        set(robot_marker, 'XData', x(k), 'YData', y(k));
        
        % Update heading arrow
        set(heading_arrow, 'XData', x(k), 'YData', y(k), ...
            'UData', 0.1*cos(heading(k)), 'VData', 0.1*sin(heading(k)));
        
        pause(0.05);
    end
end

hold off;
end

