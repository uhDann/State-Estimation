function smoothed = zero_phase_smooth(data)
% Smooths each column of an NxM array with zero-phase filtering.
%
%   smoothed = smoothColumnsZeroPhase(data)
%
% Inputs:
%   data - An NxM array that may have step-like behavior.
%
% Outputs:
%   smoothed - An NxM array where each column has been smoothed with a 
%              zero-phase Butterworth low-pass filter.
%
% In this implementation, we design a Butterworth filter and use filtfilt
% to avoid any phase delay.

% Filter order
order = 2;
% Normalised cutoff frequency
cutoff = 0.1;

% Design Butterworth low-pass filter.
[b, a] = butter(order, cutoff);

% Preallocate the smoothed result.
smoothed = zeros(size(data));

% Process each column independently.
for col = 1:size(data,2)
    smoothed(:,col) = filtfilt(b, a, data(:,col));
end

end

