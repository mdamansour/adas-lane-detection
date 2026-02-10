function params = getLaneDetectionParams()
% GETLANEDETECTIONPARAMS Returns configuration parameters for lane stability
%   Defines constants for exponential moving average (alpha), tracking
%   loss limits, and outlier rejection.

    params = struct(...
        'alpha', 0.85, ...         % Lane smoothing factor (0-1)
        'alphaWidth', 0.92, ...    % Width smoothing factor
        'MAX_LOST', 20, ...        % Max frames to hold lane before resetting
        'MIN_PTS', 20, ...         % Min points for valid polyfit
        'MIN_YSPAN', 0.15, ...     % Min vertical span (ratio of height)
        'MAX_JUMP_PX', 120);       % Max allowed horizontal jump between frames
end
