load("trainingData/calib1_rotate.mat")

figure;
plot(sqeeze(out.Sensor_Time.time)), out.Sensor_GYRO.signals.values(:,1,:))
