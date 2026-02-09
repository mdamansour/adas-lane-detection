%% Step 8: Full video processing with stability refinements
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

%% Stability and smoothing settings
alpha = 0.85;             % EMA for polynomial coefficients (stronger smoothing)
alphaWidth = 0.92;        % EMA for lane width (stronger smoothing)
MAX_LOST = 20;            % Frames to keep last good lanes (increased)
MIN_PTS = 20;             % Minimum points to accept a fit (relaxed)
MIN_YSPAN = 0.15;         % Minimum vertical span (relaxed)
MAX_JUMP_PX = 120;        % Max average jump vs last good curve (relaxed for curves)

avgPolyL = [];
avgPolyR = [];
lastGoodPolyL = [];
lastGoodPolyR = [];
lostL = 0;
lostR = 0;

avgLaneWidth = [];

%% Frame loop
while hasFrame(videoObj)
    frameCount = frameCount + 1;
    frame = readFrame(videoObj);
    frame = imgaussfilt3(frame);

    %% Adaptive brightness for thresholds
    grayFrame = rgb2gray(frame);
    meanLum = mean(grayFrame(:)) / 255;
    meanLum = max(0.0, min(1.0, meanLum));

    whiteLo = round(170 + 50 * meanLum);
    whiteLo = max(160, min(220, whiteLo));
    yellowRLo = round(110 + 40 * meanLum);
    yellowGLo = round(110 + 40 * meanLum);
    yellowRLo = max(100, min(160, yellowRLo));
    yellowGLo = max(100, min(160, yellowGLo));
    yellowBHi = round(120 + 20 * (1 - meanLum));
    yellowBHi = max(90, min(140, yellowBHi));

    %% Color masking (adaptive thresholds)
    yellowMask = (frame(:,:,1) >= yellowRLo & frame(:,:,1) <= 255) & ...
                 (frame(:,:,2) >= yellowGLo & frame(:,:,2) <= 255) & ...
                 (frame(:,:,3) >= 0         & frame(:,:,3) <= yellowBHi);

    whiteMask = (frame(:,:,1) >= whiteLo & frame(:,:,1) <= 255) & ...
                (frame(:,:,2) >= whiteLo & frame(:,:,2) <= 255) & ...
                (frame(:,:,3) >= whiteLo & frame(:,:,3) <= 255);

    %% Edge detection (adaptive threshold)
    edgeThresh = 0.18 + 0.10 * (1 - meanLum);
    edgeThresh = max(0.12, min(0.30, edgeThresh));
    edgesYellow = edge(yellowMask, 'canny', edgeThresh);
    edgesWhite = edge(whiteMask, 'canny', edgeThresh);
    edgesYellow = bwareaopen(edgesYellow, 15);
    edgesWhite = bwareaopen(edgesWhite, 15);

    %% ROI extraction (adaptive top and margins)
    [imHeight, imWidth] = size(edgesYellow);
    roiTopRatio = 0.45 + 0.15 * (1 - meanLum);
    roiTopRatio = max(0.45, min(0.65, roiTopRatio));
    roiTop = round(imHeight * roiTopRatio);
    sideMargin = round(imWidth * 0.08);
    bottomMargin = round(imWidth * 0.04);
    roiVertices = [sideMargin, roiTop; imWidth-sideMargin, roiTop; imWidth-bottomMargin, imHeight; bottomMargin, imHeight];
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

    %% Polynomial fitting with smoothing
    currPolyL = [];
    currPolyR = [];

    leftCount = size(leftPoints, 1);
    rightCount = size(rightPoints, 1);
    leftSpan = 0;
    rightSpan = 0;
    if leftCount > 1
        leftSpan = max(leftPoints(:,2)) - min(leftPoints(:,2));
    end
    if rightCount > 1
        rightSpan = max(rightPoints(:,2)) - min(rightPoints(:,2));
    end
    leftGood = leftCount >= MIN_PTS && leftSpan >= MIN_YSPAN * imHeight;
    rightGood = rightCount >= MIN_PTS && rightSpan >= MIN_YSPAN * imHeight;

    if leftGood && size(leftPoints, 1) >= 3
        currPolyL = polyfit(leftPoints(:,2), leftPoints(:,1), 2);
    elseif leftGood && size(leftPoints, 1) >= 2
        currPolyL = polyfit(leftPoints(:,2), leftPoints(:,1), 1);
    end

    if rightGood && size(rightPoints, 1) >= 3
        currPolyR = polyfit(rightPoints(:,2), rightPoints(:,1), 2);
    elseif rightGood && size(rightPoints, 1) >= 2
        currPolyR = polyfit(rightPoints(:,2), rightPoints(:,1), 1);
    end

    % Normalize polynomial lengths for EMA
    if ~isempty(currPolyL) || ~isempty(currPolyR)
        if ~isempty(currPolyL) && length(currPolyL) == 2
            currPolyL = [0 currPolyL];
        end
        if ~isempty(currPolyR) && length(currPolyR) == 2
            currPolyR = [0 currPolyR];
        end
    end

    if ~isempty(currPolyL)
        if ~isempty(lastGoodPolyL)
            yCheck = [imHeight; round(imHeight * 0.7); round(imHeight * 0.55)];
            xCurr = polyval(currPolyL, yCheck);
            xPrev = polyval(lastGoodPolyL, yCheck);
            if mean(abs(xCurr - xPrev)) > MAX_JUMP_PX
                currPolyL = [];
            end
        end
    end

    if ~isempty(currPolyR)
        if ~isempty(lastGoodPolyR)
            yCheck = [imHeight; round(imHeight * 0.7); round(imHeight * 0.55)];
            xCurr = polyval(currPolyR, yCheck);
            xPrev = polyval(lastGoodPolyR, yCheck);
            if mean(abs(xCurr - xPrev)) > MAX_JUMP_PX
                currPolyR = [];
            end
        end
    end

    if ~isempty(currPolyL)
        if isempty(avgPolyL)
            avgPolyL = currPolyL;
        else
            avgPolyL = alpha * avgPolyL + (1 - alpha) * currPolyL;
        end
        lastGoodPolyL = avgPolyL;
        lostL = 0;
    else
        lostL = lostL + 1;
        if lostL <= MAX_LOST && ~isempty(lastGoodPolyL)
            avgPolyL = lastGoodPolyL;
        else
            avgPolyL = [];
        end
    end

    if ~isempty(currPolyR)
        if isempty(avgPolyR)
            avgPolyR = currPolyR;
        else
            avgPolyR = alpha * avgPolyR + (1 - alpha) * currPolyR;
        end
        lastGoodPolyR = avgPolyR;
        lostR = 0;
    else
        lostR = lostR + 1;
        if lostR <= MAX_LOST && ~isempty(lastGoodPolyR)
            avgPolyR = lastGoodPolyR;
        else
            avgPolyR = [];
        end
    end

    %% Vanishing point and direction (smoothed polys)
    vanishingPoint = [];
    direction = 'Unknown';
    hasLeft = ~isempty(avgPolyL);
    hasRight = ~isempty(avgPolyR);

    if hasLeft && hasRight
        pL = avgPolyL;
        pR = avgPolyR;
        coeffs = pL - pR;
        rootsY = roots(coeffs);
        validRoots = rootsY(imag(rootsY) == 0 & rootsY > 0 & rootsY < imHeight);

        if ~isempty(validRoots)
            yV = min(validRoots);
            xV = polyval(pL, yV);

            if xV > 0 && xV < imWidth
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
    end

    %% Generate curves for display with stability guards
    yTopBase = round(imHeight * 0.55);
    yTopMin = round(imHeight * 0.45);
    yTopMax = round(imHeight * 0.75);

    yTop = yTopBase;
    if hasLeft && hasRight && ~isempty(vanishingPoint)
        yTop = min(max(vanishingPoint(2), yTopMin), yTopMax);
    end

    yRange = linspace(imHeight, yTop, 50)';
    xLeftPred = [];
    xRightPred = [];

    if hasLeft
        xLeftPred = polyval(avgPolyL, yRange);
    end
    if hasRight
        xRightPred = polyval(avgPolyR, yRange);
    end

    % Lane width sanity checks (prevents "mountains" and jumps)
    if ~isempty(xLeftPred) && ~isempty(xRightPred)
        widthBottom = xRightPred(1) - xLeftPred(1);
        minWidth = 0.2 * imWidth;
        maxWidth = 0.9 * imWidth;

        if widthBottom >= minWidth && widthBottom <= maxWidth
            if isempty(avgLaneWidth)
                avgLaneWidth = widthBottom;
            else
                avgLaneWidth = alphaWidth * avgLaneWidth + (1 - alphaWidth) * widthBottom;
            end
        else
            % Reject unstable width and fallback to last good polys
            if ~isempty(lastGoodPolyL) && ~isempty(lastGoodPolyR)
                xLeftPred = polyval(lastGoodPolyL, yRange);
                xRightPred = polyval(lastGoodPolyR, yRange);
            else
                xLeftPred = [];
                xRightPred = [];
            end
        end
    elseif ~isempty(avgLaneWidth)
        % If only one side is visible, synthesize the other side
        if ~isempty(xLeftPred)
            xRightPred = xLeftPred + avgLaneWidth;
        elseif ~isempty(xRightPred)
            xLeftPred = xRightPred - avgLaneWidth;
        end
    end

    %% Visualization overlay
    outputFrame = frame;

    if ~isempty(xLeftPred) && ~isempty(xRightPred)
        v1 = [xLeftPred, yRange];
        v2 = [flipud(xRightPred), flipud(yRange)];
        verts = [v1; v2];

        % Allow up to 10% crossing points for curve stability
        validPoints = sum(xLeftPred < xRightPred);
        if validPoints >= 0.9 * length(xLeftPred)
            xPoly = verts(:,1);
            yPoly = verts(:,2);
            polyInterleaved = [xPoly'; yPoly'];

            outputFrame = insertShape(outputFrame, 'FilledPolygon', polyInterleaved(:)', ...
                'Color', 'green', 'Opacity', 0.4);
        end

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
