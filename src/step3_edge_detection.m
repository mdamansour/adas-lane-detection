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

%% Edge detection (Canny)
edgesYellow = edge(yellowMask, 'canny', 0.2);
edgesWhite = edge(whiteMask, 'canny', 0.2);

%% Remove small noise regions
edgesYellow = bwareaopen(edgesYellow, 15);
edgesWhite = bwareaopen(edgesWhite, 15);

%% Display results
figure('Name', 'Step 3: Edge Detection', 'Position', [100 100 1600 500]);

subplot(1, 3, 1);
imshow(frame);
title('Original Frame');

subplot(1, 3, 2);
imshow(edgesYellow);
title('Yellow Lane Edges');

subplot(1, 3, 3);
imshow(edgesWhite);
title('White Lane Edges');
