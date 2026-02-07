%% Bird's-Eye View (BEV) Transform for Lane Detection
% Author: Mohammed Amansour
% Date: February 7, 2026
% Purpose: Apply Inverse Perspective Mapping (IPM) to transform the road
%          image from camera view to a top-down bird's-eye view. This
%          simplifies lane boundary detection by making parallel lane lines
%          appear parallel in the transformed image.
%
% Dependencies:
%   - Computer Vision Toolbox
%   - Image Processing Toolbox
%
% Notes: We use a projective transformation with manually calibrated ROI
%        coordinates optimized for the Udacity dataset (1280x720 resolution).

%% Clear workspace and command window
clear; clc; close all;

%% Step 1: Load Video
fprintf('======================================\n');
fprintf('Bird''s-Eye View Transformation\n');
fprintf('======================================\n\n');

testVideoPath = fullfile('..', 'data', 'test_drive.mp4');

if ~isfile(testVideoPath)
    error('Test video not found. Please place test_drive.mp4 in the data/ folder.');
end

fprintf('Loading video: %s\n', testVideoPath);
videoObj = VideoReader(testVideoPath);

% Read a representative frame (frame 100 for stable road view)
videoObj.CurrentTime = min(100 / videoObj.FrameRate, videoObj.Duration);
frame = readFrame(videoObj);

fprintf('Video loaded successfully.\n');
fprintf('  - Resolution: %d x %d\n', videoObj.Width, videoObj.Height);

%% Step 2: Define Camera Intrinsics (For Documentation)
% These parameters describe the camera geometry. While not directly used
% in the projective transform, they provide context for potential future
% calibration refinements or Simulink integration.

focalLength = [1150, 1150];        % pixels
principalPoint = [640, 360];       % image center for 1280x720
imageSize = [720, 1280];           % [height, width]
mountingHeight = 1.5;              % meters above ground
pitchAngle = 0;                    % degrees

fprintf('\nCamera Configuration:\n');
fprintf('  - Focal Length: [%.0f, %.0f] px\n', focalLength(1), focalLength(2));
fprintf('  - Principal Point: [%.0f, %.0f] px\n', principalPoint(1), principalPoint(2));
fprintf('  - Mounting Height: %.2f m\n', mountingHeight);

%% Step 3: Define Region of Interest (ROI)
% We define a trapezoidal region that isolates the lane markings. The
% coordinates are manually calibrated for the Udacity dataset to capture
% the relevant road area while excluding sky, horizon, and vehicle hood.

% Source points in the original image (trapezoid vertices)
% Order: Top-Left, Top-Right, Bottom-Right, Bottom-Left
sourcePoints = [575, 460;   % Top-left corner
                705, 460;   % Top-right corner
                1100, 720;  % Bottom-right corner
                200, 720];  % Bottom-left corner

% Destination points in bird's-eye view (rectangular region)
% We map the trapezoid to a vertical rectangle to "unwarp" the perspective
destinationPoints = [300, 0;      % Top-left
                     900, 0;      % Top-right
                     900, 720;    % Bottom-right
                     300, 720];   % Bottom-left

fprintf('\nROI Definition:\n');
fprintf('  - Source (Camera View): Trapezoid\n');
fprintf('  - Destination (Bird''s-Eye): Rectangle [300,0] to [900,720]\n');

%% Step 4: Compute Projective Transformation
% We use a homography (projective transformation) to map the source ROI
% to the destination rectangle. This effectively "flattens" the perspective.

tform = fitgeotrans(sourcePoints, destinationPoints, 'projective');

fprintf('\nProjective transformation matrix computed.\n');

%% Step 5: Apply Bird's-Eye View Transform
% We warp the original frame using the computed transformation. The output
% is a top-down view where lane lines appear more parallel.

bevFrame = imwarp(frame, tform, 'OutputView', ...
    imref2d(size(frame), [1 size(frame,2)], [1 size(frame,1)]));

fprintf('Bird''s-eye view transform applied.\n');

%% Step 6: Visualization
% We create a side-by-side comparison showing:
%   (1) Original frame with ROI overlay
%   (2) Transformed bird's-eye view

figure('Name', 'Bird''s-Eye View Transformation', 'Position', [100 100 1400 600]);

% Subplot 1: Original frame with ROI trapezoid
subplot(1, 2, 1);
imshow(frame);
hold on;

% Draw the ROI trapezoid
roiX = [sourcePoints(:,1); sourcePoints(1,1)];  % Close the polygon
roiY = [sourcePoints(:,2); sourcePoints(1,2)];
plot(roiX, roiY, 'r-', 'LineWidth', 3);

% Mark the vertices
plot(sourcePoints(:,1), sourcePoints(:,2), 'ro', 'MarkerSize', 10, ...
    'MarkerFaceColor', 'yellow', 'LineWidth', 2);

title('Original Frame with ROI', 'FontSize', 14, 'FontWeight', 'bold');
hold off;

% Subplot 2: Bird's-eye view
subplot(1, 2, 2);
imshow(bevFrame);
title('Bird''s-Eye View (Top-Down)', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('\n✓ Visualization complete.\n');
fprintf('======================================\n');
fprintf('Next Step: Lane line detection in BEV space.\n');
fprintf('======================================\n');
