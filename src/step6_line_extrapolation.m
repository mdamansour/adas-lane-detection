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
% Define ROI: Bottom corners and horizon
roiVertices = [200, 315; imWidth-200, 315; imWidth-50, imHeight; 50, imHeight];
roiMask = poly2mask(roiVertices(:,1), roiVertices(:,2), imHeight, imWidth);
roiYellow = edgesYellow & roiMask;
roiWhite = edgesWhite & roiMask;

%% Hough Transform - Increased peaks for curve fitting
[H_Y, theta_Y, rho_Y] = hough(roiYellow);
[H_W, theta_W, rho_W] = hough(roiWhite);

% Use more peaks to capture curvature
numPeaks = 10; 
P_Y = houghpeaks(H_Y, numPeaks, 'threshold', 2);
P_W = houghpeaks(H_W, numPeaks, 'threshold', 2);

linesYellow = houghlines(roiYellow, theta_Y, rho_Y, P_Y, 'FillGap', 3000, 'MinLength', 20);
linesWhite = houghlines(roiWhite, theta_W, rho_W, P_W, 'FillGap', 3000, 'MinLength', 20);

%% Collect Points for Curve Fitting
linesAll = [linesYellow, linesWhite];
midX = imWidth / 2;

% storage for points [x, y]
leftPoints = [];
rightPoints = [];

for k = 1:length(linesAll)
    p1 = linesAll(k).point1;
    p2 = linesAll(k).point2;
    
    % Avoid vertical lines (infinite slope)
    if p2(1) == p1(1)
        continue;
    end
    
    slope = (p2(2) - p1(2)) / (p2(1) - p1(1));
    
    % Filter gentle slopes (noise)
    if abs(slope) < 0.4
        continue;
    end
    
    % Classify based on location relative to center and meaningful slope direction
    % We use the line midpoint to check side
    midLineX = (p1(1) + p2(1)) / 2;
    
    if slope < -0.2 && midLineX < midX
        % Candidate for Left Lane
        leftPoints = [leftPoints; p1; p2];
    elseif slope > 0.2 && midLineX > midX
        % Candidate for Right Lane
        rightPoints = [rightPoints; p1; p2];
    end
end

%% Polynomial Fitting (Degree 2)
% Define display range (from bottom to horizon)
yRange = linspace(imHeight, 320, 100)'; % 100 points
yRange = round(yRange);

xLeftPred = [];
xRightPred = [];

% Fit Left
if size(leftPoints, 1) >= 3
    % Fit x = f(y) -> x = ay^2 + by + c
    polyParamsL = polyfit(leftPoints(:,2), leftPoints(:,1), 2);
    xLeftPred = polyval(polyParamsL, yRange);
elseif size(leftPoints, 1) >= 2
    % Fallback to linear if not enough points for curve
    polyParamsL = polyfit(leftPoints(:,2), leftPoints(:,1), 1);
    xLeftPred = polyval(polyParamsL, yRange);
end

% Fit Right
if size(rightPoints, 1) >= 3
    polyParamsR = polyfit(rightPoints(:,2), rightPoints(:,1), 2);
    xRightPred = polyval(polyParamsR, yRange);
elseif size(rightPoints, 1) >= 2
    polyParamsR = polyfit(rightPoints(:,2), rightPoints(:,1), 1);
    xRightPred = polyval(polyParamsR, yRange);
end

%% Visualization
figure('Name', 'Step 6: Curved Lane Fitting', 'Position', [100 100 1200 600]);
imshow(frame);
hold on;

% Draw detected hough segments (thin, for reference)
for k = 1:length(linesAll)
   xy = [linesAll(k).point1; linesAll(k).point2];
   plot(xy(:,1), xy(:,2), 'LineWidth', 1, 'Color', 'yellow', 'LineStyle', ':'); 
end

% Draw Polygon Patch if both lanes detected
if ~isempty(xLeftPred) && ~isempty(xRightPred)
    % Construct polygon vertices
    % Order: Left Bottom -> Left Top -> Right Top -> Right Bottom
    
    % Vertices must be sorted to form a closed loop without crossing
    % xLeftPred corresponds to yRange (Bottom to Top)
    % xRightPred corresponds to yRange (Bottom to Top)
    
    % Polygon: Left Lane Bottom-Up -> Right Lane Top-Down
    v1 = [xLeftPred, yRange];                  % Left: Bottom -> Top
    v2 = [flipud(xRightPred), flipud(yRange)]; % Right: Top -> Bottom
    
    verts = [v1; v2];
    
    % Draw boundary lines
    plot(xLeftPred, yRange, 'LineWidth', 5, 'Color', 'red');
    plot(xRightPred, yRange, 'LineWidth', 5, 'Color', 'red');
    
    % Draw green transparent patch
    patch('Faces', 1:size(verts,1), 'Vertices', verts, ...
          'FaceColor', 'green', 'FaceAlpha', 0.4, 'EdgeColor', 'none');
      
    text(midX, 50, 'Curved Lane Detected', 'Color', 'green', 'FontSize', 14, ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold');
elseif ~isempty(xLeftPred)
     plot(xLeftPred, yRange, 'LineWidth', 5, 'Color', 'red');
elseif ~isempty(xRightPred)
     plot(xRightPred, yRange, 'LineWidth', 5, 'Color', 'red');
end

hold off;
title('Curved Lane Extrapolation (Polynomial Fit)');
