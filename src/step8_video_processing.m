%% Clear workspace and command window
clear; clc; close all;

%% Input/Output setup
videoPath = fullfile('..', 'data', 'test_drive.mp4');
outputPath = fullfile('..', 'data', 'output_annotated.avi');
videoObj = VideoReader(videoPath);
outputVideo = VideoWriter(outputPath);
outputVideo.FrameRate = videoObj.FrameRate;
open(outputVideo);

%% Console status
fprintf('Step 8: Processing video %s\n', videoPath);
fprintf('Output: %s\n', outputPath);

frameCount = 0;
startTime = tic;

%% Frame loop
while hasFrame(videoObj)
    frameCount = frameCount + 1;
    frame = readFrame(videoObj);
    frame = imgaussfilt3(frame);

    %% Color masking
    yellowMask = (frame(:,:,1) >= 130 & frame(:,:,1) <= 255) & ...
                 (frame(:,:,2) >= 130 & frame(:,:,2) <= 255) & ...
                 (frame(:,:,3) >= 0 & frame(:,:,3) <= 130);

    whiteMask = (frame(:,:,1) >= 200 & frame(:,:,1) <= 255) & ...
                (frame(:,:,2) >= 200 & frame(:,:,2) <= 255) & ...
                (frame(:,:,3) >= 200 & frame(:,:,3) <= 255);

    %% Edge detection
    edgesYellow = edge(yellowMask, 'canny', 0.2);
    edgesWhite = edge(whiteMask, 'canny', 0.2);
    edgesYellow = bwareaopen(edgesYellow, 15);
    edgesWhite = bwareaopen(edgesWhite, 15);

    %% ROI extraction
    [imHeight, imWidth] = size(edgesYellow);
    roiVertices = [200, 315; imWidth-200, 315; imWidth-50, imHeight; 50, imHeight];
    roiMask = poly2mask(roiVertices(:,1), roiVertices(:,2), imHeight, imWidth);
    roiYellow = edgesYellow & roiMask;
    roiWhite = edgesWhite & roiMask;

    %% Hough Transform
    [H_Y, theta_Y, rho_Y] = hough(roiYellow);
    [H_W, theta_W, rho_W] = hough(roiWhite);
    numPeaks = 10;
    P_Y = houghpeaks(H_Y, numPeaks, 'threshold', 2);
    P_W = houghpeaks(H_W, numPeaks, 'threshold', 2);
    linesYellow = houghlines(roiYellow, theta_Y, rho_Y, P_Y, 'FillGap', 3000, 'MinLength', 20);
    linesWhite = houghlines(roiWhite, theta_W, rho_W, P_W, 'FillGap', 3000, 'MinLength', 20);

    %% Collect points for curve fitting
    linesAll = [linesYellow, linesWhite];
    midX = imWidth / 2;
    leftPoints = [];
    rightPoints = [];

    for k = 1:length(linesAll)
        p1 = linesAll(k).point1;
        p2 = linesAll(k).point2;
        if p2(1) == p1(1)
            continue;
        end
        slope = (p2(2) - p1(2)) / (p2(1) - p1(1));
        if abs(slope) < 0.4
            continue;
        end
        xBottom = p1(1) + (imHeight - p1(2)) / slope;
        validLeft = (slope < 0) && (midX > p1(1)) && (xBottom > -200 && xBottom < midX);
        validRight = (slope > 0) && (midX < p1(1)) && (xBottom > midX && xBottom < imWidth + 200);

        if validLeft || validRight
            lineLen = norm(p1 - p2);
            numSamples = max(2, ceil(lineLen / 5));
            xSamp = linspace(p1(1), p2(1), numSamples)';
            ySamp = linspace(p1(2), p2(2), numSamples)';
            pts = [xSamp, ySamp];
            if validLeft
                leftPoints = [leftPoints; pts];
            else
                rightPoints = [rightPoints; pts];
            end
        end
    end

    %% Polynomial fitting
    polyParamsL = [];
    polyParamsR = [];
    hasLeft = false;
    hasRight = false;

    if size(leftPoints, 1) >= 3
        polyParamsL = polyfit(leftPoints(:,2), leftPoints(:,1), 2);
        hasLeft = true;
    elseif size(leftPoints, 1) >= 2
        polyParamsL = polyfit(leftPoints(:,2), leftPoints(:,1), 1);
        hasLeft = true;
    end

    if size(rightPoints, 1) >= 3
        polyParamsR = polyfit(rightPoints(:,2), rightPoints(:,1), 2);
        hasRight = true;
    elseif size(rightPoints, 1) >= 2
        polyParamsR = polyfit(rightPoints(:,2), rightPoints(:,1), 1);
        hasRight = true;
    end

    %% Vanishing point and direction
    vanishingPoint = [];
    direction = 'Unknown';
    if hasLeft && hasRight
        pL = [0 0 0];
        pR = [0 0 0];
        if length(polyParamsL) == 3, pL = polyParamsL; else, pL = [0 polyParamsL]; end
        if length(polyParamsR) == 3, pR = polyParamsR; else, pR = [0 polyParamsR]; end
        coeffs = pL - pR;
        rootsY = roots(coeffs);
        validRoots = rootsY(imag(rootsY) == 0 & rootsY > 0 & rootsY < imHeight);
        if ~isempty(validRoots)
            yHorizon = 320;
            nearHorizon = validRoots(validRoots <= yHorizon);
            if ~isempty(nearHorizon)
                yV = min(nearHorizon);
            else
                yV = min(validRoots);
            end
            xV = polyval(pL, yV);
            vanishingPoint = [xV, yV];
            vanishingRatio = xV / imWidth;
            if vanishingRatio < 0.49
                direction = 'Turn Left';
            elseif vanishingRatio <= 0.51
                direction = 'Go Straight';
            else
                direction = 'Turn Right';
            end
        end
    end

    %% Generate curves for display
    yTop = 320;
    if hasLeft && hasRight && ~isempty(vanishingPoint)
        yTop = max(320, vanishingPoint(2));
    end
    yRange = linspace(imHeight, yTop, 50)';
    xLeftPred = [];
    xRightPred = [];
    if hasLeft
        xLeftPred = polyval(polyParamsL, yRange);
    end
    if hasRight
        xRightPred = polyval(polyParamsR, yRange);
    end

    %% Visualization overlay
    outputFrame = frame;
    if ~isempty(xLeftPred) && ~isempty(xRightPred)
        v1 = [xLeftPred, yRange];
        v2 = [flipud(xRightPred), flipud(yRange)];
        verts = [v1; v2];
        outputFrame = insertShape(outputFrame, 'FilledPolygon', verts(:)', ...
            'Color', 'green', 'Opacity', 0.4);
        outputFrame = insertShape(outputFrame, 'Line', [xLeftPred yRange], ...
            'Color', 'yellow', 'LineWidth', 5);
        outputFrame = insertShape(outputFrame, 'Line', [xRightPred yRange], ...
            'Color', 'yellow', 'LineWidth', 5);
    end

    if ~isempty(vanishingPoint)
        outputFrame = insertShape(outputFrame, 'Circle', [vanishingPoint 6], ...
            'Color', 'red', 'LineWidth', 2);
    end

    outputFrame = insertText(outputFrame, [midX-100, 30], direction, ...
        'FontSize', 24, 'BoxOpacity', 0.6, 'TextColor', 'red');

    %% Write output
    writeVideo(outputVideo, outputFrame);

    %% Console progress update
    if mod(frameCount, 30) == 0
        elapsed = toc(startTime);
        fps = frameCount / elapsed;
        fprintf('Processed %d frames | Avg FPS: %.2f\n', frameCount, fps);
    end
end

close(outputVideo);

elapsed = toc(startTime);
fprintf('Done. Total frames: %d | Avg FPS: %.2f\n', frameCount, frameCount / elapsed);
