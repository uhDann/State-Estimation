function sensor_clean = remove_outliers(sensor_data, window_size)
% Parameters for outlier detection:
method = 'movmedian';

% Detect outliers
outlierIdx = isoutlier(sensor_data, method, window_size);

% Option 1: Replace outliers with NaN and then interpolate:
sensor_clean = sensor_data;
sensor_clean(outlierIdx) = NaN;
sensor_clean = fillmissing(sensor_clean, 'linear');

end
