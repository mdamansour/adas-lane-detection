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

%% Define ROI (trapezoid covering road area)
[imHeight, imWidth] = size(edgesYellow);
roiVertices = [
    200, 315;          % Top-left
    imWidth-200, 315;  % Top-right
    imWidth-50, imHeight;  % Bottom-right
    50, imHeight       % Bottom-left
];

%% Create ROI mask
roiMask = poly2mask(roiVertices(:,1), roiVertices(:,2), imHeight, imWidth);

%% Apply ROI to edges
roiYellow = edgesYellow & roiMask;
roiWhite = edgesWhite & roiMask;

%% Display results
figure('Name', 'Step 4: ROI Extraction', 'Position', [100 100 1600 900]);

subplot(2, 2, 1);
imshow(frame);
hold on;
plot([roiVertices(:,1); roiVertices(1,1)], [roiVertices(:,2); roiVertices(1,2)], 'r-', 'LineWidth', 2);
hold off;
title('ROI Selection');

subplot(2, 2, 2);
imshow(roiMask);
title('ROI Mask');

subplot(2, 2, 3);
imshow(roiYellow);
title('Yellow Lane (ROI Filtered)');

subplot(2, 2, 4);
imshow(roiWhite);
title('White Lane (ROI Filtered)');
