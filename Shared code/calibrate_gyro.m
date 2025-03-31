function gyro_calibrated = calibrate_gyro(gyro_raw)
order = 4;
cutoff = 2;
gyro_calibrated = zero_phase_smooth(gyro_raw, order, cutoff, 200);
end
