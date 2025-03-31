function [RMSE, metrics] = evaluateTrajectory(X_est, out)
    
    % Extracting the GT
    GT_Time = out.Sensor_Time.time;

    GT_position = squeeze(out.GT_position.signals.values);
    GT_position(:,3) = 0;
    
    GT_rotation = squeeze(out.GT_rotation.signals.values);

    %Extracting the estimation
    N = length(GT_Time);

    for i = 1:N

        tform_GT(i) = rigidtform3d(quat2rotm(GT_rotation(i,:)), GT_position(i,:));
        
        est_pos = [X_est(i,1), X_est(i,2), 0];              % [x y 0]
        est_eul = [X_est(i,3), 0, 0];                       % [yaw pitch roll] = [theta 0 0]
        est_rotm = eul2rotm(est_eul, 'ZYX');                % Only yaw applied

        tform_est(i) = rigidtform3d(est_rotm, est_pos);
    end

    metrics = compareTrajectories(tform_est, tform_GT);
    RMSE = metrics.AbsoluteRMSE;

    fprintf('Absolute RMSE (Position): %.4f m\n', RMSE(1));
    fprintf('Absolute RMSE (Rotation): %.4f deg\n', RMSE(2));
end