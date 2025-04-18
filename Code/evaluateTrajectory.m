function [metrics] = evaluateTrajectory(X_est, out, dataset_name)
    
    % Extracting the GT
    GT_Time = out.Sensor_Time.time;

    GT_position = squeeze(out.GT_position.signals.values);
    GT_position(:,3) = 0;
    
    GT_rotation = squeeze(out.GT_rotation.signals.values);

    GT_heading = quat2eul(GT_rotation, 'ZYX'); 
    GT_heading = unwrap(GT_heading(:,1));

    %Extracting the estimation
    N = length(GT_Time);

    for i = 1:N

        tform_GT(i) = rigidtform3d(eul2rotm([GT_heading(i), 0, 0], 'ZYX'), GT_position(i,:));
        
        est_pos = [X_est(i,1), X_est(i,2), 0];              % [x y 0]
        est_eul = [X_est(i,3), 0, 0];                       % [yaw pitch roll] = [theta 0 0]
        est_rotm = eul2rotm(est_eul, 'ZYX');                % Only yaw applied

        tform_est(i) = rigidtform3d(est_rotm, est_pos);
    end

    metrics = compareTrajectories(tform_est, tform_GT);
    MaxAbsoluteError = max(metrics.AbsoluteError);
    ARMSE = metrics.AbsoluteRMSE;
    FinalError = metrics.AbsoluteError(end, :);

    
    fprintf("%s & %.4f & %.4f & %.4f & %.4f & %.4f & %.4f \\\\\n", dataset_name, MaxAbsoluteError(2), ARMSE(2), FinalError(2), MaxAbsoluteError(1), ARMSE(1), FinalError(1));
end