function ToF_calibrated = calibrate_ToF(ToF_raw)
load("calibrationToF.mat", "calibrationToF")
ToF13_offset = calibrationToF.ToF13;
ToF2_offset = calibrationToF.ToF2;

ToF_calibrated = ToF_raw + [ToF13_offset, ToF2_offset, ToF13_offset];
end
