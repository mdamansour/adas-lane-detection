function thresholds = getAdaptiveThresholds(frame)
% GETADAPTIVETHRESHOLDS Computes dynamic color/edge thresholds based on luminance
%   analyzes frame brightness to adjust sensitivity for shadows or glare.

    grayFrame = rgb2gray(frame);
    meanLum = mean(grayFrame(:)) / 255;
    meanLum = max(0.0, min(1.0, meanLum));

    % Dynamic White Thresholds
    whiteLo = round(170 + 50 * meanLum);
    whiteLo = max(160, min(220, whiteLo));

    % Dynamic Yellow Thresholds
    yellowRLo = round(110 + 40 * meanLum);
    yellowGLo = round(110 + 40 * meanLum);
    yellowRLo = max(100, min(160, yellowRLo));
    yellowGLo = max(100, min(160, yellowGLo));
    
    % Yellow Blue channel upper limit (yellow has low blue)
    yellowBHi = round(120 + 20 * (1 - meanLum));
    yellowBHi = max(90, min(140, yellowBHi));

    % Dynamic Edge and ROI parameters
    edgeThresh = 0.18 + 0.10 * (1 - meanLum);
    edgeThresh = max(0.12, min(0.30, edgeThresh));

    roiTopRatio = 0.45 + 0.15 * (1 - meanLum);
    roiTopRatio = max(0.45, min(0.65, roiTopRatio));

    thresholds = struct(...
        'meanLum', meanLum, ...
        'whiteLo', whiteLo, ...
        'yellowRLo', yellowRLo, ...
        'yellowGLo', yellowGLo, ...
        'yellowBHi', yellowBHi, ...
        'edgeThresh', edgeThresh, ...
        'roiTopRatio', roiTopRatio);
end
