function interpolated = interpolate_sensor(sensor, time, time_new)
    interpolated = interp1(time, sensor, time_new, 'linear', 'extrap');
end
