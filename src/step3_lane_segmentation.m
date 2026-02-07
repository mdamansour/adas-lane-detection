%% Lane Segmentation using Color and Gradient Thresholding
% Author: Mohammed Amansour
% Date: February 7, 2026
% Purpose: Extract lane marking pixels from bird's-eye view images using
%          a combination of color-space thresholding (HLS) and gradient
%          analysis (Sobel). This multi-modal approach ensures robust
%          detection of both yellow and white lane markings under varying
%          lighting conditions.
%
% Dependencies:
%   - Computer Vision Toolbox
%   - Image Processing Toolbox
%
% Pipeline:
%   1. Load video and apply IPM transform
%   2. Color segmentation in HLS space (yellow + white lanes)
%   3. Gradient-based edge detection (Sobel X-direction)
%   4. Mask fusion and morphological refinement
%   5. Visualization of segmentation results

%% Clear workspace and command window
clear; clc; close all;

%% Step 1: Load Video and Apply Bird's-Eye View Transform
fprintf('======================================\n');
fprintf('Lane Segmentation Pipeline\n');
fprintf('======================================\n\n');

testVideoPath = fullfile('..', 'data', 'test_drive.mp4');

if ~isfile(testVideoPath)
    error('Test video not found. Please place test_drive.mp4 in the data/ folder.');
end

fprintf('Loading video: %s\n', testVideoPath);
videoObj = VideoReader(testVideoPath);

% Select a representative frame with clear lane markings
videoObj.CurrentTime = min(100 / videoObj.FrameRate, videoObj.Duration);
frame = readFrame(videoObj);

fprintf('Video loaded. Processing frame at t = %.2f seconds.\n', videoObj.CurrentTime);

% Define IPM transformation parameters (from step2_bev_transform.m)
sourcePoints = [575, 460;   % Top-left
                705, 460;   % Top-right
                1100, 720;  % Bottom-right
                200, 720];  % Bottom-left

destinationPoints = [300, 0;
                     900, 0;
                     900, 720;
                     300, 720];

% Compute and apply homography transformation
tform = fitgeotrans(sourcePoints, destinationPoints, 'projective');
bevFrame = imwarp(frame, tform, 'OutputView', ...
    imref2d(size(frame), [1 size(frame,2)], [1 size(frame,1)]));

fprintf('Bird''s-eye view transformation applied.\n\n');

%% Step 2: Color Thresholding in HLS Space
% HLS (Hue, Lightness, Saturation) color space provides better separation
% of chromaticity from intensity compared to RGB, making it more robust to
% illumination changes. We target yellow (H~0.08-0.15) and white (L>0.65).

fprintf('Applying color-space segmentation...\n');

% Convert to HLS color space
hlsFrame = rgb2hsv(bevFrame);  % MATLAB uses HSV; we treat V as L for simplicity
H = hlsFrame(:,:,1);  % Hue channel [0, 1]
L = hlsFrame(:,:,3);  % Lightness/Value channel [0, 1]
S = hlsFrame(:,:,2);  % Saturation channel [0, 1]

% Yellow lane detection: Target hue range for yellow (0.08 - 0.18 normalized)
% and require sufficient saturation to avoid desaturated yellows
yellowMask = (H >= 0.08) & (H <= 0.18) & (S >= 0.4);

% White lane detection: High lightness with low saturation
% White appears as high value across all color channels
whiteMask = (L >= 0.70);

fprintf('  - Yellow lane mask: %d pixels detected\n', sum(yellowMask(:)));
fprintf('  - White lane mask: %d pixels detected\n', sum(whiteMask(:)));

%% Step 3: Gradient-Based Segmentation (Sobel Operator)
% Lane markings exhibit strong intensity gradients perpendicular to their
% orientation. For vertical/near-vertical lanes in BEV, the X-gradient
% (horizontal direction) captures the lane edges effectively.

fprintf('\nApplying gradient-based segmentation...\n');

% Convert to grayscale for gradient computation
grayFrame = rgb2gray(bevFrame);

% Apply Sobel filter in X-direction (detects vertical edges)
sobelX = imgradientxy(grayFrame, 'sobel');
sobelX = abs(sobelX);  % Magnitude of gradient

% Adaptive thresholding: Use Otsu's method for automatic threshold selection
% We apply a scaling factor (0.8) to make it more sensitive
threshold = 0.8 * graythresh(sobelX);
gradientMask = imbinarize(sobelX, threshold);

fprintf('  - Gradient mask: %d pixels detected\n', sum(gradientMask(:)));
fprintf('  - Sobel threshold: %.4f\n', threshold);

%% Step 4: Mask Fusion and Morphological Refinement
% We combine the color and gradient masks using logical OR to capture
% all potential lane pixels. Morphological closing fills small gaps and
% connects nearby lane segments.

fprintf('\nFusing segmentation masks...\n');

% Combine all three masks
combinedMask = yellowMask | whiteMask | gradientMask;

% Morphological closing: Remove small holes and connect nearby components
% Use a small structuring element to avoid over-smoothing
se = strel('disk', 3);
finalBinary = imclose(combinedMask, se);

% Optional: Remove small isolated components (noise rejection)
finalBinary = bwareaopen(finalBinary, 50);  % Remove objects < 50 pixels

fprintf('  - Combined mask: %d pixels\n', sum(combinedMask(:)));
fprintf('  - Final binary (after morphology): %d pixels\n', sum(finalBinary(:)));

%% Step 5: Visualization
% We display the color BEV image alongside the binary segmentation mask
% to qualitatively assess the lane extraction quality.

fprintf('\n✓ Segmentation complete. Generating visualization...\n');

figure('Name', 'Lane Segmentation Results', 'Position', [100 100 1400 600]);

% Subplot 1: Bird's-eye view color image
subplot(1, 3, 1);
imshow(bevFrame);
title('Bird''s-Eye View (Color)', 'FontSize', 14, 'FontWeight', 'bold');

% Subplot 2: Individual masks visualization
subplot(1, 3, 2);
% Convert logical masks to uint8 for RGB visualization
maskRGB = cat(3, uint8(yellowMask)*255, uint8(whiteMask)*255, uint8(gradientMask)*255);
imshow(maskRGB);
title('Segmentation Channels (R=Yellow, G=White, B=Gradient)', ...
    'FontSize', 12, 'FontWeight', 'bold');

% Subplot 3: Final binary segmentation mask
subplot(1, 3, 3);
imshow(finalBinary);
title('Final Lane Pixel Mask', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('\n======================================\n');
fprintf('Next Step: Polynomial fitting & curvature estimation.\n');
fprintf('======================================\n');
