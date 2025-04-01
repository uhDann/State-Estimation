load("../trainingData/calib2_straight.mat");

% N x 4
% Updates @ 10Hz
% [dist, ambient, signal, status]
% Status: 0=Probably too close but OK, 2=Long OK, 4=Short OK
% Left, centre (backwards), right
ToF1 = out.Sensor_ToF1.signals.values;
ToF2 = out.Sensor_ToF2.signals.values;
ToF3 = out.Sensor_ToF3.signals.values;

GT_position = squeeze(out.GT_position.signals.values);
GT_rotation = squeeze(out.GT_rotation.signals.values);

heading = quat2eul(GT_rotation, 'ZYX');
heading = unwrap(heading(:,1));

find_ToF_offsets([ToF1(:, 1), ToF2(:, 1), ToF3(:, 1)], GT_position, heading);

function find_ToF_offsets(ToF_raw, GT_position, heading)

ToF1_raw = ToF_raw(:,1);
ToF2_raw = ToF_raw(:,2);
ToF3_raw = ToF_raw(:,3);

GT_y = GT_position(:,2);

L = 2.4;

% ToF2: Forward-Facing
cost_fn2 = @(offset) sum(((L/2 - GT_y) - (sin(heading+pi).*ToF2_raw(:, 1) + sin(heading+pi)*offset)).^2);

offset2_0 = 0;  % Initial guess
[ToF2_offset, ~] = fminsearch(cost_fn2, offset2_0);

% ToF1 and ToF3: Side-Facing
LR_beam_wall_intersect_dist = L./cos(heading+pi/2);
LR_beam_total_lengths = ToF1_raw(:,1) + ToF3_raw(:,1);

LR_ToF_distance = mean(LR_beam_wall_intersect_dist - LR_beam_total_lengths);

ToF13_offset = LR_ToF_distance/2;

calibrationToF.ToF13 = ToF13_offset;
calibrationToF.ToF2 = ToF2_offset;

save("calibrationToF.mat", "calibrationToF")

end
