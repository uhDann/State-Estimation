function accel_filtered = calibrate_accel(accel1, accel2)

% Subtracting the mean to ensure centering
accel1 = (accel1(:,2:3))-mean(accel1(10:200,2:3));
accel2 = accel2(:,[1,3])-mean(accel2(10:200,[1,3]));

var1 = var(accel1);
var2 = var(accel2);

% Calculate weights (element-wise, axis by axis)
w1 = var2 ./ (var1 + var2);
w2 = var1 ./ (var1 + var2);

% Fuse each axis individually
accel_fused = w1 .* accel1 + w2 .* accel2;

accel_filtered = zero_phase_smooth(accel_fused, 4, 2, 200);
[bh, ah] = butter(4, 0.045/(200/2), 'high');

% Apply zero-phase high-pass filtering
accel_filtered = filtfilt(bh, ah, accel_filtered);

end
