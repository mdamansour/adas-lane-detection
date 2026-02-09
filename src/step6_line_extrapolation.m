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

%% Collect Points for Curve Fitting (With Outlier Rejection and Weighting)
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
    
    % Filter gentle slopes (horizon lines)
    if abs(slope) < 0.4
        continue;
    end
    
    % Calculate bottom x-intercept
    % x = (y - b) / m  => xBottom = p1(1) + (imHeight - p1(2)) / slope
    xBottom = p1(1) + (imHeight - p1(2)) / slope;
    
    % Reject lines that project too far sideways (e.g. adjacent lanes)
    validLeft = (slope < 0) && (midX > p1(1)) && (xBottom > -200 && xBottom < midX);
    validRight = (slope > 0) && (midX < p1(1)) && (xBottom > midX && xBottom < imWidth + 200);
    
    if validLeft || validRight
        % WEIGHTING: Generate more points for longer lines to anchor the fit
        lineLen = norm(p1 - p2);
        numSamples = max(2, ceil(lineLen / 5)); % Sampling density: 1 point every 5px
        
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

%% Polynomial Fitting (Degree 2)
% Initialize polys (default to linear verticalish if empty)
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

%% Calculate Dynamic Top Boundary (Intersection of Lanes)
yTop = 320; % Default horizon

if hasLeft && hasRight
    % Solve for intersection: aL*y^2 + bL*y + cL = aR*y^2 + bR*y + cR
    % (aL-aR)*y^2 + (bL-bR)*y + (cL-cR) = 0
    
    pL = [0 0 0]; pR = [0 0 0];
    if length(polyParamsL) == 3, pL = polyParamsL; else, pL = [0 polyParamsL]; end
    if length(polyParamsR) == 3, pR = polyParamsR; else, pR = [0 polyParamsR]; end
    
    coeffs = pL - pR;
    rootsY = roots(coeffs);
    
    % Find valid real intersection above bottom
    validRoots = rootsY(imag(rootsY)==0 & rootsY < imHeight & rootsY > 0);
    
    if ~isempty(validRoots)
        % Pick the lowest Y (highest on screen) that is valid or the max Y (closest to camera)
        % Actually we want the point where they converge in distance (smallest Y > horizon limit)
        intersectionY = max(validRoots); % Usually they cross at the vanishing point
        yTop = max(320, intersectionY); % Don't go above hard limit
    end
end

% Define generation range
yRange = linspace(imHeight, yTop, 50)'; % Bottom to Intersection

%% Generate Curves
xLeftPred = [];
xRightPred = [];

if hasLeft
    xLeftPred = polyval(polyParamsL, yRange);
end

if hasRight
    xRightPred = polyval(polyParamsR, yRange);
end

%% Visualization
figure('Name', 'Step 6: Evaluated & Improved', 'Position', [100 100 1200 600]);
imshow(frame);
hold on;

% Draw Polygon Patch
if ~isempty(xLeftPred) && ~isempty(xRightPred)
    % Vertices: Left(Bottom->Top) -> Right(Top->Bottom)
    v1 = [xLeftPred, yRange];
    v2 = [flipud(xRightPred), flipud(yRange)];
    verts = [v1; v2];
    
    patch('Faces', 1:size(verts,1), 'Vertices', verts, ...
          'FaceColor', 'green', 'FaceAlpha', 0.4, 'EdgeColor', 'none');
          
    plot(xLeftPred, yRange, 'LineWidth', 5, 'Color', 'yellow');
    plot(xRightPred, yRange, 'LineWidth', 5, 'Color', 'yellow');
    
    text(midX, 50, 'Curved Lane Detected', 'Color', 'green', 'FontSize', 14, ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end

hold off;
title('Curved Lane Extrapolation (Weighted Polynomial Fit)');
