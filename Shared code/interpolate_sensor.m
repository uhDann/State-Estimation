function interpData = interpolate_sensor(sensorData, sampleRate, updateFreq)
% interpolateTrueUpdates Interpolates sensor data between true update points.
%
%   interpData = interpolateTrueUpdates(sensorData, sampleRate, updateFreq)
%
% Inputs:
%   sensorData : Vector (or column array) of sensor readings sampled at sampleRate.
%   sampleRate : Sampling rate in Hz (e.g., 200).
%   updateFreq : True update frequency of the sensor in Hz.
%
% Outputs:
%   interpData : The sensor data interpolated between true update points.
%
% The function first determines the indices at which the sensor has true updates.
% It then uses these as anchor points and performs linear interpolation over the full
% time vector.
%

% Calculate the number of samples between true updates.
updateInterval = sampleRate / updateFreq;  % e.g., if 200 Hz and updateFreq=50 Hz, updateInterval=4

% Create a time vector for the sensorData.
N = size(sensorData, 1);
t = (0:N-1) / sampleRate;

% Determine the indices corresponding to true updates.
% Here we assume that the first sample is a true update.
updateIndices = round(1:updateInterval:N);

% Extract the sensor values at the true update times.
trueUpdates = sensorData(updateIndices, :);
tUpdates = t(updateIndices);

if ndims(sensorData) > 1
    interpData = zeros(N, size(sensorData, 2));
    
    for i = 1 : size(sensorData, 2)
        % Perform linear interpolation over the full time vector.
        interpData(:, i) = interp1(tUpdates, trueUpdates(:, i), t, 'linear', 'extrap');
    end
else
    interpData = interp1(tUpdates, trueUpdates, t, 'linear', 'extrap');
end

end

