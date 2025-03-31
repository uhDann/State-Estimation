function [RMSE, metrics] = evaluateTrajectory(X_est, out)
    
    % Extracting the GT
    GT_Time = out.Sensor_Time.time;

    GT_position = squeeze(out.GT_position.signals.values);
    GT_position(:,3) = 0;
    
    GT_rotation = squeeze(out.GT_rotation.signals.values);

    GT_heading = quat2eul(GT_rotation, 'ZYX'); 
    GT_heading = unwrap(GT_heading(:,1));

    %Extracting the estimation
    N = length(GT_Time);
    % Basic manual RMSE for debugging
    pos_error = sqrt(sum((GT_position(:,1:2) - X_est(:,1:2)).^2, 2));
    RMSE_manual = sqrt(mean(pos_error.^2));
    fprintf('Manual RMSE (XY): %.4f m\n', RMSE_manual);

    for i = 1:N

        tform_GT(i) = rigidtform3d(eul2rotm([GT_heading(i), 0, 0], 'ZYX'), GT_position(i,:));
        
        est_pos = [X_est(i,1), X_est(i,2), 0];              % [x y 0]
        est_eul = [X_est(i,3), 0, 0];                       % [yaw pitch roll] = [theta 0 0]
        est_rotm = eul2rotm(est_eul, 'ZYX');                % Only yaw applied

        tform_est(i) = rigidtform3d(est_rotm, est_pos);
    end

    metrics = compareTrajectories(tform_est, tform_GT);
    RMSE = metrics.AbsoluteRMSE;

    fprintf('Absolute RMSE (Position): %.4f deg\n', RMSE(1));
    fprintf('Absolute RMSE (Rotation): %.4f m\n', RMSE(2));
end