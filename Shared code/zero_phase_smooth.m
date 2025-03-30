function smoothed = zero_phase_smooth(data, order, cutoff)
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
% Inputs:
%   data: NxM
%       Array that may have step-like behavior.
%
% Outputs:
%   smoothed: NxM
%       Array where each column has been smoothed with a zero-phase Butterworth low-pass filter.

% Design Butterworth low-pass filter.
[b, a] = butter(order, cutoff);

smoothed = zeros(size(data));

% Process each column independently.
for col = 1:size(data,2)
    smoothed(:,col) = filtfilt(b, a, data(:,col));
end

end

