function plot_trajectory(time, pos, heading, ToF, animate)
% Plots the 2D trajectory of the robot
%
% Parameters:
%   time: Nx1
%       Vector of time steps
%   pos: Nx3
%       Matrix of positions (columns: x, y, z)
%   heading: Nx1
%       Vector of headings (radians)
%   ToF: Nx3
% TODO: confirm this is the correct format
%       Matrix of ToF readings (columns: left, right, front)

%   animate - (optional) boolean
%       Flag to animate the robot

x = pos(:, 1);
y = pos(:, 2);

% Room size
L = 2.4;

figure;
hold on;
grid on;
axis equal;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Robot Trajectory');
xlim([-L/2 - 0.1, L/2 + 0.1]);
ylim([-L/2 - 0.1, L/2 + 0.1]);
plot(x, y, 'b-', 'LineWidth', 1.5);

% Plot room boundaries
plot([-L/2, -L/2, L/2, L/2, -L/2], [-L/2, L/2, L/2, -L/2, -L/2], 'k-', 'LineWidth', 1.5);

% Start
plot(x(1), y(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
% End
plot(x(end), y(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

ToF_front = ToF(:, 1);
ToF_left = ToF(:, 2);
ToF_right = ToF(:, 3);

if animate
    robot_marker = plot(x(1), y(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    heading_arrow = quiver(x(1), y(1), cos(heading(1)), sin(heading(1)), 'r', 'LineWidth', 2);
    
    % ToF_left = ToF(:, 1);
    % ToF_right = ToF(:, 3);
    % ToF_front = ToF(:, 2);
    
    ToF_left_beam = quiver(x(1), y(1), ToF_left(1) * cos(heading(1) + pi/2), ToF_left(1) * sin(heading(1)+pi/2), 'b', 'LineWidth', 2);
    ToF_right_beam = quiver(x(1), y(1), ToF_right(1) * cos(heading(1) - pi/2), ToF_right(1) * sin(heading(1)-pi/2), 'b', 'LineWidth', 2);
    ToF_front_beam = quiver(x(1), y(1), ToF_front(1) * cos(heading(1)), ToF_front(1) * sin(heading(1)), 'b', 'LineWidth', 2);
    
    for k = 1:length(time)
        % Update robot marker position
        set(robot_marker, 'XData', x(k), 'YData', y(k));
        
        % Update heading arrow
        set(heading_arrow, 'XData', x(k), 'YData', y(k), ...
            'UData', cos(heading(k)), 'VData', sin(heading(k)));
        
        % Update ToF beams
        set(ToF_left_beam, 'XData', x(k), 'YData', y(k), 'UData', ToF_left(k) * cos(heading(k) + pi/2), 'VData', ToF_left(k) * sin(heading(k)+pi/2));
        set(ToF_right_beam, 'XData', x(k), 'YData', y(k), 'UData', ToF_right(k) * cos(heading(k) - pi/2), 'VData', ToF_right(k) * sin(heading(k)-pi/2));
        set(ToF_front_beam, 'XData', x(k), 'YData', y(k), 'UData', ToF_front(k) * cos(heading(k)), 'VData', ToF_front(k) * sin(heading(k)));
        
        pause(0.0001);
    end
end

hold off;
end

