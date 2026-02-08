%% Clear workspace and command window
clear; clc; close all;

%% Load and preprocess frame
videoPath = fullfile('..', 'data', 'test_drive.mp4');
videoObj = VideoReader(videoPath);
videoObj.CurrentTime = videoObj.Duration / 2;
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
P_Y = houghpeaks(H_Y, 2, 'threshold', 2);
P_W = houghpeaks(H_W, 2, 'threshold', 2);
linesYellow = houghlines(roiYellow, theta_Y, rho_Y, P_Y, 'FillGap', 3000, 'MinLength', 20);
linesWhite = houghlines(roiWhite, theta_W, rho_W, P_W, 'FillGap', 3000, 'MinLength', 20);

%% Select ego-lane boundaries (color-agnostic)
linesAll = [linesYellow, linesWhite];
midX = imWidth / 2;
leftLine = [];
rightLine = [];
leftBest = -inf;
rightBest = inf;

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
    if xBottom < midX
        if xBottom > leftBest
            leftBest = xBottom;
            leftLine = linesAll(k);
        end
    else
        if xBottom < rightBest
            rightBest = xBottom;
            rightLine = linesAll(k);
        end
    end
end

%% Extrapolate lines to top and bottom of ROI
plotFrame = frame;
figure('Name', 'Step 6: Line Extrapolation', 'Position', [100 100 1200 600]);
imshow(plotFrame);
hold on;

if ~isempty(leftLine)
    p1 = leftLine.point1;
    p2 = leftLine.point2;
    mL = (p2(2) - p1(2)) / (p2(1) - p1(1));
    bL = p1(2) - mL * p1(1);
end

if ~isempty(rightLine)
    p1 = rightLine.point1;
    p2 = rightLine.point2;
    mR = (p2(2) - p1(2)) / (p2(1) - p1(1));
    bR = p1(2) - mR * p1(1);
end

yTop = round(imHeight * 0.60);
yBottom = imHeight;

if ~isempty(leftLine)
    xLeftTop = (yTop - bL) / mL;
    xLeftBottom = (yBottom - bL) / mL;
    plot([xLeftTop, xLeftBottom], [yTop, yBottom], 'LineWidth', 6, 'Color', 'yellow');
end

if ~isempty(rightLine)
    xRightTop = (yTop - bR) / mR;
    xRightBottom = (yBottom - bR) / mR;
    plot([xRightTop, xRightBottom], [yTop, yBottom], 'LineWidth', 6, 'Color', 'yellow');
end

if ~isempty(leftLine) && ~isempty(rightLine)
    lanePolygon = [xLeftTop, yTop; xRightTop, yTop; xRightBottom, yBottom; xLeftBottom, yBottom];
    patch('Faces', [1 2 3 4], 'Vertices', lanePolygon, 'FaceColor', 'green', 'EdgeColor', 'green', 'FaceAlpha', 0.3);
end

hold off;
title('Extrapolated Lane Lines');
