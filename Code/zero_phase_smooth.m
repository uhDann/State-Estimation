function smoothed = zero_phase_smooth(data, order, cutoff, sample_frequency)
% Smooths each column of an NxM array with zero-phase filtering.
%
% Rewrite docstring to be like other files
% Parameters:
%   data: NxM
%       Array that may have step-like behavior.
%   order: int
%       Order of the Butterworth filter.
%   cutoff: float
%       Cutoff frequency of the Butterworth filter.
%   sample_frequency: float
%       Sample frequency of the data.
% Inputs:
%   data: NxM
%       Array that may have step-like behavior.
%
% Returns:
%   smoothed: NxM
%       Array where each column has been smoothed with a zero-phase Butterworth low-pass filter.

% Design Butterworth low-pass filter.
[b, a] = butter(order, cutoff/(sample_frequency/2), "low");

smoothed = zeros(size(data));

% Process each column independently.
for col = 1:size(data,2)
    smoothed(:,col) = filtfilt(b, a, data(:,col));
end

end

