%% Polynomial Lane Fitting and Curvature Estimation
% Author: Mohammed Amansour
% Date: February 7, 2026
% Purpose: Extract lane geometry from binary segmentation masks using
%          sliding window search and second-order polynomial fitting.
%          Calculate real-world navigation metrics including radius of
%          curvature and vehicle lateral offset from lane center.
%
% Dependencies:
%   - Computer Vision Toolbox
%   - Image Processing Toolbox
%
% Pipeline:
%   1. Load video and generate binary lane mask (Steps 1-3)
%   2. Histogram-based lane peak detection
%   3. Sliding window pixel extraction
%   4. Polynomial curve fitting (x = Ay^2 + By + C)
%   5. Metric extraction (curvature, offset)
%   6. Visualization with overlay

%% Clear workspace and command window
clear; clc; close all;

%% Step 1: Load Video and Generate Binary Lane Mask
fprintf('======================================\n');
fprintf('Polynomial Lane Fitting Pipeline\n');
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

%% Step 2: Apply Bird's-Eye View Transform
% Define IPM transformation parameters (from step2_bev_transform.m)
sourcePoints = [575, 460;   % Top-left
                705, 460;   % Top-right
                1100, 720;  % Bottom-right
                200, 720];  % Bottom-left

destinationPoints = [300, 0;
                     900, 0;
                     900, 720;
                     300, 720];

tform = fitgeotrans(sourcePoints, destinationPoints, 'projective');
bevFrame = imwarp(frame, tform, 'OutputView', ...
    imref2d(size(frame), [1 size(frame,2)], [1 size(frame,1)]));

fprintf('Bird''s-eye view transformation applied.\n');

%% Step 3: Generate Binary Lane Mask
% Apply the multi-modal segmentation from step3_lane_segmentation.m

% Convert to HLS color space
hlsFrame = rgb2hsv(bevFrame);
H = hlsFrame(:,:,1);
L = hlsFrame(:,:,3);
S = hlsFrame(:,:,2);

% Yellow and white lane detection
yellowMask = (H >= 0.08) & (H <= 0.18) & (S >= 0.4);
whiteMask = (L >= 0.70);

% Gradient-based segmentation
grayFrame = rgb2gray(bevFrame);
sobelX = imgradientxy(grayFrame, 'sobel');
sobelX = abs(sobelX);
threshold = 0.8 * graythresh(sobelX);
gradientMask = imbinarize(sobelX, threshold);

% Mask fusion and morphological refinement
combinedMask = yellowMask | whiteMask | gradientMask;
se = strel('disk', 3);
finalBinary = imclose(combinedMask, se);
finalBinary = bwareaopen(finalBinary, 50);

fprintf('Binary lane mask generated (%d pixels).\n\n', sum(finalBinary(:)));

%% Step 4: Histogram-Based Lane Peak Detection
% We compute a histogram of the bottom half of the binary image to identify
% the x-positions where lane pixels are concentrated. The two highest peaks
% correspond to the left and right lane starting positions.

fprintf('Detecting lane base positions...\n');

% Take histogram of bottom half
histogram = sum(finalBinary(floor(size(finalBinary,1)/2):end, :), 1);

% Find the midpoint
midpoint = floor(size(finalBinary, 2) / 2);

% Identify peaks in left and right halves
[~, leftx_base] = max(histogram(1:midpoint));
[~, rightx_base] = max(histogram(midpoint+1:end));
rightx_base = rightx_base + midpoint;  % Offset to absolute position

fprintf('  - Left lane base: x = %d pixels\n', leftx_base);
fprintf('  - Right lane base: x = %d pixels\n\n', rightx_base);

%% Step 5: Sliding Window Search
% We use 9 vertical windows per lane to track pixel clusters from bottom to top.
% Each window is centered on the mean x-position of pixels in the previous window.

fprintf('Executing sliding window search...\n');

% Define sliding window parameters
nwindows = 9;
window_height = floor(size(finalBinary, 1) / nwindows);
margin = 100;        % Width of search window (±100 pixels from center)
minpix = 50;         % Minimum pixels to recenter window

% Identify all nonzero pixel positions
[nonzeroy, nonzerox] = find(finalBinary);

% Current positions for each lane (will be updated as we step through windows)
leftx_current = leftx_base;
rightx_current = rightx_base;

% Storage for pixel indices belonging to each lane
left_lane_inds = [];
right_lane_inds = [];

% Storage for window boundaries (for visualization)
window_rectangles = [];

% Step through windows from bottom to top
for window_idx = 0:(nwindows-1)
    % Define window boundaries in y
    win_y_low = size(finalBinary, 1) - (window_idx + 1) * window_height;
    win_y_high = size(finalBinary, 1) - window_idx * window_height;
    
    % Define window boundaries in x for left and right lanes
    win_xleft_low = leftx_current - margin;
    win_xleft_high = leftx_current + margin;
    win_xright_low = rightx_current - margin;
    win_xright_high = rightx_current + margin;
    
    % Store rectangles for visualization [x, y, width, height]
    window_rectangles = [window_rectangles; 
                         win_xleft_low, win_y_low, 2*margin, window_height;
                         win_xright_low, win_y_low, 2*margin, window_height];
    
    % Identify pixels within the left window
    good_left_inds = find((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & ...
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high));
    
    % Identify pixels within the right window
    good_right_inds = find((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & ...
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high));
    
    % Append these indices to the lane lists
    left_lane_inds = [left_lane_inds; good_left_inds];
    right_lane_inds = [right_lane_inds; good_right_inds];
    
    % Recenter windows based on mean position if enough pixels found
    if length(good_left_inds) > minpix
        leftx_current = round(mean(nonzerox(good_left_inds)));
    end
    if length(good_right_inds) > minpix
        rightx_current = round(mean(nonzerox(good_right_inds)));
    end
end

% Extract left and right lane pixel positions
leftx = nonzerox(left_lane_inds);
lefty = nonzeroy(left_lane_inds);
rightx = nonzerox(right_lane_inds);
righty = nonzeroy(right_lane_inds);

fprintf('  - Left lane pixels: %d\n', length(leftx));
fprintf('  - Right lane pixels: %d\n\n', length(rightx));

%% Step 6: Polynomial Fitting in Pixel Space
% Fit second-order polynomials: x = Ay^2 + By + C
% We use polyfit with y as independent variable (since lanes are near-vertical)

fprintf('Fitting polynomial curves...\n');

if length(leftx) > 0
    left_fit_px = polyfit(lefty, leftx, 2);  % [A, B, C]
    fprintf('  - Left lane: x = %.6f*y^2 + %.6f*y + %.6f\n', ...
            left_fit_px(1), left_fit_px(2), left_fit_px(3));
else
    warning('No left lane pixels detected!');
    left_fit_px = [0, 0, 0];
end

if length(rightx) > 0
    right_fit_px = polyfit(righty, rightx, 2);
    fprintf('  - Right lane: x = %.6f*y^2 + %.6f*y + %.6f\n\n', ...
            right_fit_px(1), right_fit_px(2), right_fit_px(3));
else
    warning('No right lane pixels detected!');
    right_fit_px = [0, 0, 0];
end

%% Step 7: Metric Extraction - Pixel to Meter Conversion
% Define conversion factors based on US highway lane specifications
% Lane width: 3.7 meters (typical US highway)
% Look-ahead distance: ~30 meters captured in 720 pixels vertical span

fprintf('Converting to real-world coordinates...\n');

ym_per_pix = 30.0 / 720.0;     % meters per pixel in y dimension
xm_per_pix = 3.7 / 700.0;      % meters per pixel in x dimension

fprintf('  - Y conversion: %.4f m/pixel\n', ym_per_pix);
fprintf('  - X conversion: %.4f m/pixel\n\n', xm_per_pix);

% Refit polynomials in world coordinates
if length(leftx) > 0
    left_fit_m = polyfit(lefty * ym_per_pix, leftx * xm_per_pix, 2);
else
    left_fit_m = [0, 0, 0];
end

if length(rightx) > 0
    right_fit_m = polyfit(righty * ym_per_pix, rightx * xm_per_pix, 2);
else
    right_fit_m = [0, 0, 0];
end

%% Step 8: Radius of Curvature Calculation
% Formula: R_curve = [(1 + (2Ay + B)^2)^(3/2)] / |2A|
% Evaluate at y = image bottom (closest to vehicle)

fprintf('Computing navigation metrics...\n');

y_eval = size(finalBinary, 1) * ym_per_pix;  % Bottom of image in meters

% Left lane curvature
if left_fit_m(1) ~= 0
    left_curverad = ((1 + (2*left_fit_m(1)*y_eval + left_fit_m(2))^2)^1.5) / abs(2*left_fit_m(1));
else
    left_curverad = inf;
end

% Right lane curvature
if right_fit_m(1) ~= 0
    right_curverad = ((1 + (2*right_fit_m(1)*y_eval + right_fit_m(2))^2)^1.5) / abs(2*right_fit_m(1));
else
    right_curverad = inf;
end

% Average curvature
avg_curvature = (left_curverad + right_curverad) / 2;

fprintf('  - Left lane radius: %.2f m\n', left_curverad);
fprintf('  - Right lane radius: %.2f m\n', right_curverad);
fprintf('  - Average curvature: %.2f m\n', avg_curvature);

%% Step 9: Vehicle Offset from Lane Center
% Assume camera is at vehicle centerline (image center)
% Lane center = midpoint between left and right lane positions at image bottom

image_center = size(finalBinary, 2) / 2;  % pixels

% Evaluate polynomial at bottom of image
y_bottom = size(finalBinary, 1);
left_lane_pos = polyval(left_fit_px, y_bottom);    % x position of left lane
right_lane_pos = polyval(right_fit_px, y_bottom);  % x position of right lane

% Lane center in pixels
lane_center = (left_lane_pos + right_lane_pos) / 2;

% Offset in pixels, then convert to meters
offset_pixels = image_center - lane_center;
offset_meters = offset_pixels * xm_per_pix;

fprintf('  - Vehicle offset: %.2f m ', offset_meters);
if offset_meters > 0
    fprintf('(right of center)\n\n');
else
    fprintf('(left of center)\n\n');
end

%% Step 10: Visualization
% Create a comprehensive visualization showing:
%   1. Original BEV with sliding windows overlaid
%   2. Polynomial curves fitted to lane pixels
%   3. Filled lane region with curvature/offset metrics

fprintf('✓ Lane fitting complete. Generating visualization...\n');

% Generate points for plotting the fitted polynomials
ploty = linspace(0, size(finalBinary, 1) - 1, size(finalBinary, 1));
left_fitx = polyval(left_fit_px, ploty);
right_fitx = polyval(right_fit_px, ploty);

% Create output image (convert BEV to RGB if grayscale)
out_img = bevFrame;

% Create a blank mask for drawing the lane region
lane_mask = zeros(size(finalBinary), 'uint8');

% Prepare points for filled polygon (lane region)
% MATLAB uses [x,y] format for polygon vertices
left_points = [left_fitx', ploty'];
right_points = [right_fitx', ploty'];

% Create closed polygon: left lane (bottom to top) + right lane (top to bottom)
pts = [left_points; flipud(right_points)];

% Fill the lane region
lane_mask = insertShape(lane_mask, 'FilledPolygon', reshape(pts', 1, []), ...
                        'Color', 'white', 'Opacity', 1);
lane_mask = rgb2gray(lane_mask);

% Color the lane region green with transparency
lane_colored = cat(3, zeros(size(lane_mask)), lane_mask, zeros(size(lane_mask)));
out_img = imadd(out_img, uint8(lane_colored * 0.3));

% Draw the polynomial curves
out_img = insertShape(out_img, 'Line', ...
    [left_fitx(1:end-1)', ploty(1:end-1)', left_fitx(2:end)', ploty(2:end)'], ...
    'Color', 'yellow', 'LineWidth', 5);
out_img = insertShape(out_img, 'Line', ...
    [right_fitx(1:end-1)', ploty(1:end-1)', right_fitx(2:end)', ploty(2:end)'], ...
    'Color', 'yellow', 'LineWidth', 5);

% Draw sliding windows
for i = 1:size(window_rectangles, 1)
    rect = window_rectangles(i, :);
    out_img = insertShape(out_img, 'Rectangle', rect, ...
                         'Color', 'red', 'LineWidth', 2);
end

% Overlay metrics text
if isinf(avg_curvature)
    curv_text = sprintf('Curvature: Straight road');
else
    curv_text = sprintf('Radius of Curvature: %.0f m', avg_curvature);
end
offset_text = sprintf('Vehicle Offset: %.2f m', offset_meters);

% Insert text annotations
out_img = insertText(out_img, [50, 50], curv_text, ...
                    'FontSize', 18, 'BoxColor', 'black', 'TextColor', 'white');
out_img = insertText(out_img, [50, 100], offset_text, ...
                    'FontSize', 18, 'BoxColor', 'black', 'TextColor', 'white');

% Display final result
figure('Name', 'Polynomial Lane Fitting Results', 'Position', [100 100 1200 700]);
imshow(out_img);
title('Lane Detection with Sliding Window Search and Polynomial Fitting', ...
      'FontSize', 14, 'FontWeight', 'bold');

fprintf('======================================\n');
fprintf('Next Step: Real-time processing & Simulink integration.\n');
fprintf('======================================\n');
