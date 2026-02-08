%% Clear workspace and command window
clear; clc; close all;

%% Load and preprocess frame
videoPath = fullfile('..', 'data', 'test_drive.mp4');
videoObj = VideoReader(videoPath);
videoObj.CurrentTime = videoObj.Duration / 2;
frame = readFrame(videoObj);
frame = imgaussfilt3(frame);

%% Yellow lane mask (RGB thresholds)
yellowMask = (frame(:,:,1) >= 130 & frame(:,:,1) <= 255) & ...
             (frame(:,:,2) >= 130 & frame(:,:,2) <= 255) & ...
             (frame(:,:,3) >= 0 & frame(:,:,3) <= 130);

%% White lane mask (RGB thresholds)
whiteMask = (frame(:,:,1) >= 200 & frame(:,:,1) <= 255) & ...
            (frame(:,:,2) >= 200 & frame(:,:,2) <= 255) & ...
            (frame(:,:,3) >= 200 & frame(:,:,3) <= 255);

%% Display results
figure('Name', 'Step 2: Color Masking', 'Position', [100 100 1600 500]);

subplot(1, 3, 1);
imshow(frame);
title('Filtered Frame');

subplot(1, 3, 2);
imshow(yellowMask);
title('Yellow Mask');

subplot(1, 3, 3);
imshow(whiteMask);
title('White Mask');
