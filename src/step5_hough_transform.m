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

%% Detect Hough Peaks (2 peaks per lane)
P_Y = houghpeaks(H_Y, 2, 'threshold', 2);
P_W = houghpeaks(H_W, 2, 'threshold', 2);

%% Extract Lines from Peaks
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

%% Display results
figure('Name', 'Step 5: Hough Transform', 'Position', [100 100 1600 900]);

% Original frame with detected lines
subplot(2, 2, 1);
imshow(frame);
hold on;
if ~isempty(leftLine)
    xy = [leftLine.point1; leftLine.point2];
    plot(xy(:,1), xy(:,2), 'LineWidth', 2, 'Color', 'yellow');
end
if ~isempty(rightLine)
    xy = [rightLine.point1; rightLine.point2];
    plot(xy(:,1), xy(:,2), 'LineWidth', 2, 'Color', 'green');
end
hold off;
title('Detected Hough Lines');

% Hough space for yellow
subplot(2, 2, 2);
imshow(imadjust(rescale(H_Y)), 'XData', theta_Y, 'YData', rho_Y, 'InitialMagnification', 'fit');
xlabel('\theta (degrees)');
ylabel('\rho');
axis on; axis normal;
hold on;
if ~isempty(P_Y)
    plot(theta_Y(P_Y(:,2)), rho_Y(P_Y(:,1)), 's', 'color', 'red', 'MarkerSize', 10);
end
hold off;
title('Hough Space - Yellow Lane');

% Hough space for white
subplot(2, 2, 3);
imshow(imadjust(rescale(H_W)), 'XData', theta_W, 'YData', rho_W, 'InitialMagnification', 'fit');
xlabel('\theta (degrees)');
ylabel('\rho');
axis on; axis normal;
hold on;
if ~isempty(P_W)
    plot(theta_W(P_W(:,2)), rho_W(P_W(:,1)), 's', 'color', 'red', 'MarkerSize', 10);
end
hold off;
title('Hough Space - White Lane');

% Edge images with detected lines
subplot(2, 2, 4);
imshow(frame);
hold on;
if ~isempty(leftLine)
    xy = [leftLine.point1; leftLine.point2];
    plot(xy(:,1), xy(:,2), 'LineWidth', 3, 'Color', 'red');
    plot(xy(1,1), xy(1,2), 'x', 'LineWidth', 2, 'Color', 'yellow', 'MarkerSize', 10);
    plot(xy(2,1), xy(2,2), 'x', 'LineWidth', 2, 'Color', 'yellow', 'MarkerSize', 10);
end
if ~isempty(rightLine)
    xy = [rightLine.point1; rightLine.point2];
    plot(xy(:,1), xy(:,2), 'LineWidth', 3, 'Color', 'red');
    plot(xy(1,1), xy(1,2), 'x', 'LineWidth', 2, 'Color', 'cyan', 'MarkerSize', 10);
    plot(xy(2,1), xy(2,2), 'x', 'LineWidth', 2, 'Color', 'cyan', 'MarkerSize', 10);
end
hold off;
title('Line Endpoints');
