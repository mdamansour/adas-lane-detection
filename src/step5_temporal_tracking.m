%% Temporal Lane Tracking with Recursive Filtering
% Author: Mohammed Amansour
% Date: February 7, 2026
% Purpose: Implement frame-to-frame lane tracking using "search from prior"
%          strategy to reduce computational overhead. Apply recursive
%          smoothing (Alpha-Beta filter) to polynomial coefficients to
%          eliminate jitter in curvature and offset estimates.
%
% Dependencies:
%   - Computer Vision Toolbox
%   - Image Processing Toolbox
%
% Architecture:
%   - State: TRACKING (fast search around prior polynomial)
%   - State: RE-ACQUISITION (full sliding window search)
%   - Transition: Confidence score < threshold triggers re-acquisition
%
% Performance Target: > 30 FPS real-time processing

%% Clear workspace and command window
clear; clc; close all;

%% Configuration Parameters
ALPHA = 0.1;                % Smoothing factor for recursive filter
SEARCH_MARGIN = 100;        % Search area around prior polynomial (pixels)
CONFIDENCE_THRESHOLD = 1000; % Minimum pixels for tracking confidence
NWINDOWS = 9;               % Number of sliding windows for re-acquisition
WINDOW_MARGIN = 100;        % Sliding window search margin
MINPIX = 50;                % Minimum pixels to recenter window

% Pixel-to-meter conversion factors
YM_PER_PIX = 30.0 / 720.0;
XM_PER_PIX = 3.7 / 700.0;

%% Step 1: Load Video
fprintf('======================================\n');
fprintf('Temporal Lane Tracking Pipeline\n');
fprintf('======================================\n\n');

testVideoPath = fullfile('..', 'data', 'test_drive.mp4');

if ~isfile(testVideoPath)
    error('Test video not found. Please place test_drive.mp4 in the data/ folder.');
end

fprintf('Loading video: %s\n', testVideoPath);
videoObj = VideoReader(testVideoPath);
fprintf('  - Duration: %.2f seconds\n', videoObj.Duration);
fprintf('  - Frame Rate: %.2f fps\n', videoObj.FrameRate);
fprintf('  - Total Frames: %d\n\n', floor(videoObj.Duration * videoObj.FrameRate));

%% Step 2: Define BEV Transformation (Reusable)
sourcePoints = [575, 460; 705, 460; 1100, 720; 200, 720];
destinationPoints = [300, 0; 900, 0; 900, 720; 300, 720];
tform = fitgeotrans(sourcePoints, destinationPoints, 'projective');

%% Step 3: Initialize Tracking State
% Pre-allocate storage for polynomial coefficients (MATLAB Coder compatible)
left_fit_prev = [0.0, 0.0, 0.0];   % Previous frame left lane polynomial
right_fit_prev = [0.0, 0.0, 0.0];  % Previous frame right lane polynomial
left_fit_smooth = [0.0, 0.0, 0.0]; % Smoothed left lane polynomial
right_fit_smooth = [0.0, 0.0, 0.0];% Smoothed right lane polynomial

tracking_state = 'RE-ACQUISITION';  % Initial state (no prior information)
frame_count = 0;
fps_accumulator = zeros(1, 30);     % Rolling window for FPS calculation

fprintf('Tracking Parameters:\n');
fprintf('  - Smoothing Alpha: %.2f\n', ALPHA);
fprintf('  - Search Margin: %d pixels\n', SEARCH_MARGIN);
fprintf('  - Confidence Threshold: %d pixels\n\n', CONFIDENCE_THRESHOLD);

%% Step 4: Setup Video Writer for Output
outputVideoPath = fullfile('..', 'data', 'tracked_output.avi');
outputVideo = VideoWriter(outputVideoPath, 'Motion JPEG AVI');
outputVideo.FrameRate = videoObj.FrameRate;
open(outputVideo);

fprintf('Output video: %s\n\n', outputVideoPath);
fprintf('Processing frames...\n');
fprintf('Frame | State          | Left Px | Right Px | Curvature (m) | Offset (m) | FPS\n');
fprintf('------|----------------|---------|----------|---------------|------------|-----\n');

%% Step 5: Main Processing Loop
while hasFrame(videoObj)
    tic;  % Start frame timer
    
    % Read and transform frame
    frame = readFrame(videoObj);
    frame_count = frame_count + 1;
    
    % Apply Bird's-Eye View Transform
    bevFrame = imwarp(frame, tform, 'OutputView', ...
        imref2d(size(frame), [1 size(frame,2)], [1 size(frame,1)]));
    
    %% Generate Binary Lane Mask (Segmentation Pipeline from Step 3)
    hlsFrame = rgb2hsv(bevFrame);
    H = hlsFrame(:,:,1);
    L = hlsFrame(:,:,3);
    S = hlsFrame(:,:,2);
    
    yellowMask = (H >= 0.08) & (H <= 0.18) & (S >= 0.4);
    whiteMask = (L >= 0.70);
    
    grayFrame = rgb2gray(bevFrame);
    sobelX = imgradientxy(grayFrame, 'sobel');
    sobelX = abs(sobelX);
    threshold = 0.8 * graythresh(sobelX);
    gradientMask = imbinarize(sobelX, threshold);
    
    combinedMask = yellowMask | whiteMask | gradientMask;
    se = strel('disk', 3);
    finalBinary = imclose(combinedMask, se);
    finalBinary = bwareaopen(finalBinary, 50);
    
    % Get all nonzero pixel positions
    [nonzeroy, nonzerox] = find(finalBinary);
    
    %% Adaptive Lane Pixel Extraction (State-Dependent)
    if strcmp(tracking_state, 'TRACKING')
        % ===== TRACKING MODE: Search from Prior =====
        % Use previous polynomial to define targeted search area
        
        % Generate polynomial curves from previous coefficients
        ploty = (0:size(finalBinary,1)-1)';
        left_fitx = polyval(left_fit_prev, ploty);
        right_fitx = polyval(right_fit_prev, ploty);
        
        % Extract pixels within margin of predicted curves
        left_lane_inds = [];
        right_lane_inds = [];
        
        for i = 1:length(nonzeroy)
            y_pos = nonzeroy(i);
            x_pos = nonzerox(i);
            
            % Check if pixel is within margin of left curve
            left_predicted = polyval(left_fit_prev, y_pos);
            if abs(x_pos - left_predicted) <= SEARCH_MARGIN
                left_lane_inds = [left_lane_inds; i];
            end
            
            % Check if pixel is within margin of right curve
            right_predicted = polyval(right_fit_prev, y_pos);
            if abs(x_pos - right_predicted) <= SEARCH_MARGIN
                right_lane_inds = [right_lane_inds; i];
            end
        end
        
        % Extract pixel coordinates
        leftx = nonzerox(left_lane_inds);
        lefty = nonzeroy(left_lane_inds);
        rightx = nonzerox(right_lane_inds);
        righty = nonzeroy(right_lane_inds);
        
        % Confidence check: Transition to RE-ACQUISITION if insufficient pixels
        if length(leftx) < CONFIDENCE_THRESHOLD || length(rightx) < CONFIDENCE_THRESHOLD
            tracking_state = 'RE-ACQUISITION';
            fprintf('%5d | TRACK->REACQ   | %-7d | %-8d | (confidence lost)\n', ...
                    frame_count, length(leftx), length(rightx));
            continue;  % Re-process this frame in RE-ACQUISITION mode
        end
        
    else
        % ===== RE-ACQUISITION MODE: Full Sliding Window Search =====
        % Compute histogram of bottom half
        histogram = sum(finalBinary(floor(size(finalBinary,1)/2):end, :), 1);
        midpoint = floor(size(finalBinary, 2) / 2);
        
        [~, leftx_base] = max(histogram(1:midpoint));
        [~, rightx_base] = max(histogram(midpoint+1:end));
        rightx_base = rightx_base + midpoint;
        
        % Sliding window search
        window_height = floor(size(finalBinary, 1) / NWINDOWS);
        leftx_current = leftx_base;
        rightx_current = rightx_base;
        left_lane_inds = [];
        right_lane_inds = [];
        
        for window_idx = 0:(NWINDOWS-1)
            win_y_low = size(finalBinary, 1) - (window_idx + 1) * window_height;
            win_y_high = size(finalBinary, 1) - window_idx * window_height;
            
            win_xleft_low = leftx_current - WINDOW_MARGIN;
            win_xleft_high = leftx_current + WINDOW_MARGIN;
            win_xright_low = rightx_current - WINDOW_MARGIN;
            win_xright_high = rightx_current + WINDOW_MARGIN;
            
            good_left_inds = find((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & ...
                                  (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high));
            good_right_inds = find((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & ...
                                   (nonzerox >= win_xright_low) & (nonzerox < win_xright_high));
            
            left_lane_inds = [left_lane_inds; good_left_inds];
            right_lane_inds = [right_lane_inds; good_right_inds];
            
            if length(good_left_inds) > MINPIX
                leftx_current = round(mean(nonzerox(good_left_inds)));
            end
            if length(good_right_inds) > MINPIX
                rightx_current = round(mean(nonzerox(good_right_inds)));
            end
        end
        
        leftx = nonzerox(left_lane_inds);
        lefty = nonzeroy(left_lane_inds);
        rightx = nonzerox(right_lane_inds);
        righty = nonzeroy(right_lane_inds);
        
        % Transition to TRACKING if sufficient pixels found
        if length(leftx) >= CONFIDENCE_THRESHOLD && length(rightx) >= CONFIDENCE_THRESHOLD
            tracking_state = 'TRACKING';
        end
    end
    
    %% Polynomial Fitting
    if length(leftx) > 0
        left_fit_new = polyfit(lefty, leftx, 2);
    else
        left_fit_new = left_fit_prev;  % Retain previous fit if no pixels
    end
    
    if length(rightx) > 0
        right_fit_new = polyfit(righty, rightx, 2);
    else
        right_fit_new = right_fit_prev;
    end
    
    %% Recursive Smoothing (Alpha-Beta Filter)
    % Coef_smooth = alpha * Coef_new + (1 - alpha) * Coef_old
    if frame_count == 1
        % Initialize with first measurement
        left_fit_smooth = left_fit_new;
        right_fit_smooth = right_fit_new;
    else
        % Apply low-pass filter
        left_fit_smooth = ALPHA * left_fit_new + (1 - ALPHA) * left_fit_smooth;
        right_fit_smooth = ALPHA * right_fit_new + (1 - ALPHA) * right_fit_smooth;
    end
    
    % Update previous coefficients for next frame
    left_fit_prev = left_fit_smooth;
    right_fit_prev = right_fit_smooth;
    
    %% Metric Extraction
    % Refit in world coordinates
    left_fit_m = polyfit(lefty * YM_PER_PIX, leftx * XM_PER_PIX, 2);
    right_fit_m = polyfit(righty * YM_PER_PIX, rightx * XM_PER_PIX, 2);
    
    % Radius of curvature
    y_eval = size(finalBinary, 1) * YM_PER_PIX;
    if left_fit_m(1) ~= 0
        left_curverad = ((1 + (2*left_fit_m(1)*y_eval + left_fit_m(2))^2)^1.5) / abs(2*left_fit_m(1));
    else
        left_curverad = inf;
    end
    if right_fit_m(1) ~= 0
        right_curverad = ((1 + (2*right_fit_m(1)*y_eval + right_fit_m(2))^2)^1.5) / abs(2*right_fit_m(1));
    else
        right_curverad = inf;
    end
    avg_curvature = (left_curverad + right_curverad) / 2;
    
    % Vehicle offset
    image_center = size(finalBinary, 2) / 2;
    y_bottom = size(finalBinary, 1);
    left_lane_pos = polyval(left_fit_smooth, y_bottom);
    right_lane_pos = polyval(right_fit_smooth, y_bottom);
    lane_center = (left_lane_pos + right_lane_pos) / 2;
    offset_meters = (image_center - lane_center) * XM_PER_PIX;
    
    %% Visualization
    ploty = linspace(0, size(finalBinary, 1) - 1, size(finalBinary, 1));
    left_fitx = polyval(left_fit_smooth, ploty);
    right_fitx = polyval(right_fit_smooth, ploty);
    
    out_img = bevFrame;
    
    % Draw lane region
    lane_mask = zeros(size(finalBinary), 'uint8');
    left_points = [left_fitx', ploty'];
    right_points = [right_fitx', ploty'];
    pts = [left_points; flipud(right_points)];
    lane_mask = insertShape(lane_mask, 'FilledPolygon', reshape(pts', 1, []), ...
                            'Color', 'white', 'Opacity', 1);
    lane_mask = rgb2gray(lane_mask);
    lane_colored = cat(3, zeros(size(lane_mask)), lane_mask, zeros(size(lane_mask)));
    out_img = imadd(out_img, uint8(lane_colored * 0.3));
    
    % Draw polynomial curves
    out_img = insertShape(out_img, 'Line', ...
        [left_fitx(1:end-1)', ploty(1:end-1)', left_fitx(2:end)', ploty(2:end)'], ...
        'Color', 'yellow', 'LineWidth', 5);
    out_img = insertShape(out_img, 'Line', ...
        [right_fitx(1:end-1)', ploty(1:end-1)', right_fitx(2:end)', ploty(2:end)'], ...
        'Color', 'yellow', 'LineWidth', 5);
    
    % Overlay metrics and state
    if isinf(avg_curvature)
        curv_text = sprintf('Curvature: Straight road');
    else
        curv_text = sprintf('Radius: %.0f m', avg_curvature);
    end
    offset_text = sprintf('Offset: %.2f m', offset_meters);
    state_text = sprintf('State: %s', tracking_state);
    
    % Calculate FPS
    frame_time = toc;
    current_fps = 1 / frame_time;
    fps_accumulator(mod(frame_count-1, 30) + 1) = current_fps;
    avg_fps = mean(fps_accumulator(1:min(frame_count, 30)));
    fps_text = sprintf('FPS: %.1f', avg_fps);
    
    out_img = insertText(out_img, [50, 50], curv_text, ...
                        'FontSize', 18, 'BoxColor', 'black', 'TextColor', 'white');
    out_img = insertText(out_img, [50, 100], offset_text, ...
                        'FontSize', 18, 'BoxColor', 'black', 'TextColor', 'white');
    out_img = insertText(out_img, [50, 150], state_text, ...
                        'FontSize', 18, 'BoxColor', 'black', 'TextColor', 'green');
    out_img = insertText(out_img, [50, 200], fps_text, ...
                        'FontSize', 18, 'BoxColor', 'black', 'TextColor', 'cyan');
    
    % Write to output video
    writeVideo(outputVideo, out_img);
    
    % Console logging
    fprintf('%5d | %-14s | %-7d | %-8d | %-13.0f | %-10.2f | %.1f\n', ...
            frame_count, tracking_state, length(leftx), length(rightx), ...
            avg_curvature, offset_meters, current_fps);
    
    % Optional: Display every Nth frame to avoid overwhelming the display
    if mod(frame_count, 10) == 1
        figure(1);
        imshow(out_img);
        title(sprintf('Frame %d - Temporal Tracking', frame_count), ...
              'FontSize', 14, 'FontWeight', 'bold');
        drawnow;
    end
end

%% Cleanup
close(outputVideo);

fprintf('\n======================================\n');
fprintf('Processing Complete!\n');
fprintf('  - Total Frames: %d\n', frame_count);
fprintf('  - Average FPS: %.1f\n', avg_fps);
fprintf('  - Output saved: %s\n', outputVideoPath);
fprintf('======================================\n');
fprintf('Next Step: Simulink integration for HIL testing.\n');
fprintf('======================================\n');
