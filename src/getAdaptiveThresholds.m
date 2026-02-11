function thresholds = getAdaptiveThresholds(frame)
% GETADAPTIVETHRESHOLDS Computes dynamic color/edge thresholds based on luminance
%   Uses HSV color space for illumination-invariant lane detection.
%   Yellow is identified by Hue, White by low Saturation + high Value.

    grayFrame = rgb2gray(frame);
    meanLum = mean(grayFrame(:)) / 255;
    meanLum = max(0.0, min(1.0, meanLum));

    % Yellow Lane (HSV): Hue defines color regardless of lighting
    % Hue range: 30-60 degrees (normalized to 0.08-0.17 in MATLAB)
    yellowHueLo = 0.08;
    yellowHueHi = 0.17;
    yellowSatLo = 0.4;  % Must be vivid enough to be "yellow"
    
    % Adaptive Value threshold for yellow (handles shadows)
    yellowValLo = 0.3 + 0.3 * meanLum;
    yellowValLo = max(0.25, min(0.65, yellowValLo));

    % White Lane (HSV): Low saturation (near grayscale) + high value
    whiteSatHi = 0.25;  % Almost no color
    whiteValLo = 0.65 + 0.2 * meanLum;
    whiteValLo = max(0.60, min(0.85, whiteValLo));

    % Dynamic Edge and ROI parameters
    edgeThresh = 0.18 + 0.10 * (1 - meanLum);
    edgeThresh = max(0.12, min(0.30, edgeThresh));

    % ROI Horizon: Never look above the midline (0.50) to avoid sky noise
    % Previously 0.45, which captured too much horizon/mountains
    roiTopRatio = 0.50 + 0.15 * (1 - meanLum);
    roiTopRatio = max(0.50, min(0.65, roiTopRatio));

    thresholds = struct(...
        'meanLum', meanLum, ...
        'yellowHueLo', yellowHueLo, ...
        'yellowHueHi', yellowHueHi, ...
        'yellowSatLo', yellowSatLo, ...
        'yellowValLo', yellowValLo, ...
        'whiteSatHi', whiteSatHi, ...
        'whiteValLo', whiteValLo, ...
        'edgeThresh', edgeThresh, ...
        'roiTopRatio', roiTopRatio);
end
