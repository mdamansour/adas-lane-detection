%% Clear workspace and command window
clear; clc; close all;

%% Load Video
videoPath = fullfile('..', 'data', 'test_drive.mp4');
if ~isfile(videoPath)
    error('Video file not found: %s', videoPath);
end

videoObj = VideoReader(videoPath);
videoObj.CurrentTime = videoObj.Duration / 2;
originalFrame = readFrame(videoObj);

%% Apply Gaussian Filter
filteredFrame = imgaussfilt3(originalFrame);

%% Display Results
figure('Name', 'Step 1: Gaussian Filtering', 'Position', [100 100 1400 500]);
subplot(1, 2, 1);
imshow(originalFrame);
title('Original Frame');

subplot(1, 2, 2);
imshow(filteredFrame);
title('Filtered Frame');

