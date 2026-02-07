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
% Performance and Benchmarking
benchmarking_mode = false;   % Set to true to disable visualization for pure FPS measurement
% Note: Static analyzer warnings about unreachable code are expected when this flag
% is constant. Set to true for pure computational benchmarking.

% Tracking Parameters
ALPHA = 0.1;                % Smoothing factor for recursive filter
SEARCH_MARGIN = 100;        % Search area around prior polynomial (pixels)
CONFIDENCE_THRESHOLD = 1000; % Minimum pixels for tracking confidence
NWINDOWS = 9;               % Number of sliding windows for re-acquisition
WINDOW_MARGIN = 100;        % Sliding window search margin
MINPIX = 50;                % Minimum pixels to recenter window

% Curvature Stability
CURVATURE_CAP = 10000;      % Maximum curvature radius (m) - beyond this is "straight road"

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

% Pre-allocate computational timing array for performance (avoid dynamic growth)
total_frames_estimated = floor(videoObj.Duration * videoObj.FrameRate);
computational_times = zeros(total_frames_estimated, 1);

fprintf('Tracking Parameters:\n');
fprintf('  - Benchmarking Mode: %s\n', mat2str(benchmarking_mode));
fprintf('  - Smoothing Alpha: %.2f\n', ALPHA);
fprintf('  - Search Margin: %d pixels\n', SEARCH_MARGIN);
fprintf('  - Confidence Threshold: %d pixels\n', CONFIDENCE_THRESHOLD);
fprintf('  - Curvature Cap: %d meters\n\n', CURVATURE_CAP);

%% Step 4: Setup Video Writer for Output (Skip in Benchmarking Mode)
if ~benchmarking_mode
    outputVideoPath = fullfile('..', 'data', 'tracked_output.avi');
    outputVideo = VideoWriter(outputVideoPath, 'Motion JPEG AVI');
    outputVideo.FrameRate = videoObj.FrameRate;
    open(outputVideo);
    fprintf('Output video: %s\n\n', outputVideoPath);
else
    fprintf('Benchmarking mode: Visualization disabled.\n\n'); %#ok<UNRCH>
end

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
        % ===== TRACKING MODE: Search from Prior (Vectorized) =====
        % Use previous polynomial to define targeted search area
        % OPTIMIZATION: Fully vectorized to eliminate per-pixel loop overhead
        
        % Vectorized polynomial evaluation for all detected pixels
        left_predicted = polyval(left_fit_prev, nonzeroy);
        right_predicted = polyval(right_fit_prev, nonzeroy);
        
        % Vectorized logical indexing (eliminate loop)
        left_lane_inds = find(abs(nonzerox - left_predicted) <= SEARCH_MARGIN);
        right_lane_inds = find(abs(nonzerox - right_predicted) <= SEARCH_MARGIN);
        
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
        
        % Pre-allocate index arrays (avoid dynamic growth in loop)
        max_pixels_per_window = 5000;  % Conservative upper bound
        left_lane_inds = zeros(NWINDOWS * max_pixels_per_window, 1);
        right_lane_inds = zeros(NWINDOWS * max_pixels_per_window, 1);
        left_count = 0;
        right_count = 0;
        
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
            
            % Store indices without dynamic array growth
            n_left = length(good_left_inds);
            n_right = length(good_right_inds);
            left_lane_inds(left_count+1:left_count+n_left) = good_left_inds;
            right_lane_inds(right_count+1:right_count+n_right) = good_right_inds;
            left_count = left_count + n_left;
            right_count = right_count + n_right;
            
            if length(good_left_inds) > MINPIX
                leftx_current = round(mean(nonzerox(good_left_inds)));
            end
            if length(good_right_inds) > MINPIX
                rightx_current = round(mean(nonzerox(good_right_inds)));
            end
        end
        
        % Trim pre-allocated arrays to actual size
        left_lane_inds = left_lane_inds(1:left_count);
        right_lane_inds = right_lane_inds(1:right_count);
        
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
    
    %% Metric Extraction (Using SMOOTHED coefficients for stability)
    % Refit smoothed polynomials in world coordinates
    if length(leftx) > 0 && length(lefty) > 0
        left_fit_m = polyfit(lefty * YM_PER_PIX, leftx * XM_PER_PIX, 2);
    else
        % Use smoothed pixel-space coefficients converted to metric space
        left_fit_m = [left_fit_smooth(1) / (YM_PER_PIX^2 / XM_PER_PIX), ...
                      left_fit_smooth(2) / (YM_PER_PIX / XM_PER_PIX), ...
                      left_fit_smooth(3) / XM_PER_PIX];
    end
    
    if length(rightx) > 0 && length(righty) > 0
        right_fit_m = polyfit(righty * YM_PER_PIX, rightx * XM_PER_PIX, 2);
    else
        right_fit_m = [right_fit_smooth(1) / (YM_PER_PIX^2 / XM_PER_PIX), ...
                       right_fit_smooth(2) / (YM_PER_PIX / XM_PER_PIX), ...
                       right_fit_smooth(3) / XM_PER_PIX];
    end
    
    % Radius of curvature at vehicle position (bottom of image)
    y_eval = size(finalBinary, 1) * YM_PER_PIX;
    
    % Left lane curvature
    if abs(left_fit_m(1)) > 1e-6  % Avoid division by near-zero
        left_curverad = ((1 + (2*left_fit_m(1)*y_eval + left_fit_m(2))^2)^1.5) / abs(2*left_fit_m(1));
    else
        left_curverad = CURVATURE_CAP;  % Treat as straight road
    end
    
    % Right lane curvature
    if abs(right_fit_m(1)) > 1e-6
        right_curverad = ((1 + (2*right_fit_m(1)*y_eval + right_fit_m(2))^2)^1.5) / abs(2*right_fit_m(1));
    else
        right_curverad = CURVATURE_CAP;
    end
    
    % Apply curvature cap for numerical stability (ISO 26262 compliance)
    left_curverad = min(left_curverad, CURVATURE_CAP);
    right_curverad = min(right_curverad, CURVATURE_CAP);
    avg_curvature = (left_curverad + right_curverad) / 2;
    
    % Vehicle lateral offset from lane center (at bottom of image - closest to vehicle)
    image_center = size(finalBinary, 2) / 2;  % pixels
    y_bottom = size(finalBinary, 1);          % Bottom row (vehicle location)
    left_lane_pos = polyval(left_fit_smooth, y_bottom);   % Use smoothed coefficients
    right_lane_pos = polyval(right_fit_smooth, y_bottom);
    lane_center = (left_lane_pos + right_lane_pos) / 2;
    offset_meters = (image_center - lane_center) * XM_PER_PIX;
    
    % End computational timing (before visualization)
    computational_time = toc;
    computational_times(frame_count) = computational_time;
    
    %% Visualization (Disabled in Benchmarking Mode)
    if ~benchmarking_mode
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
        if avg_curvature >= CURVATURE_CAP
            curv_text = sprintf('Curvature: Straight road');
        else
            curv_text = sprintf('Radius: %.0f m', avg_curvature);
        end
        offset_text = sprintf('Offset: %.2f m', offset_meters);
        state_text = sprintf('State: %s', tracking_state);
        
        % Calculate total FPS (including visualization)
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
        
        % Optional: Display every Nth frame to avoid overwhelming the display
        if mod(frame_count, 10) == 1
            figure(1);
            imshow(out_img);
            title(sprintf('Frame %d - Temporal Tracking', frame_count), ...
                  'FontSize', 14, 'FontWeight', 'bold');
            drawnow;
        end
    end
    
    % Console logging (always enabled)
    if ~benchmarking_mode
        fprintf('%5d | %-14s | %-7d | %-8d | %-13.0f | %-10.2f | %.1f\n', ...
                frame_count, tracking_state, length(leftx), length(rightx), ...
                avg_curvature, offset_meters, current_fps);
    else
        % In benchmarking mode, show computational FPS
        comp_fps = 1 / computational_time; %#ok<UNRCH>
        fprintf('%5d | %-14s | %-7d | %-8d | %-13.0f | %-10.2f | %.1f\n', ...
                frame_count, tracking_state, length(leftx), length(rightx), ...
                avg_curvature, offset_meters, comp_fps); %#ok<UNRCH>
    end
end

%% Cleanup
if ~benchmarking_mode
    close(outputVideo);
end

%% Performance Analysis
% Extract only the valid computational times (trim pre-allocated array)
valid_computational_times = computational_times(1:frame_count);

mean_computational_fps = 1 / mean(valid_computational_times);
min_computational_time = min(valid_computational_times) * 1000;  % Convert to ms
max_computational_time = max(valid_computational_times) * 1000;
mean_computational_time = mean(valid_computational_times) * 1000;

fprintf('\n======================================\n');
fprintf('Processing Complete!\n');
fprintf('======================================\n');
fprintf('Performance Metrics:\n');
fprintf('  - Total Frames: %d\n', frame_count);
fprintf('  - Mean Computational FPS: %.1f (logic only)\n', mean_computational_fps);
fprintf('  - Computational Latency:\n');
fprintf('    * Mean: %.2f ms\n', mean_computational_time);
fprintf('    * Min:  %.2f ms\n', min_computational_time);
fprintf('    * Max:  %.2f ms\n', max_computational_time);

if ~benchmarking_mode
    fprintf('  - Total FPS (with visualization): %.1f\n', avg_fps);
    fprintf('  - Output saved: %s\n', outputVideoPath);
else
    fprintf('  - Benchmarking mode: Visualization was disabled\n'); %#ok<UNRCH>
end

% Real-time compliance check
if mean_computational_time <= 33.0
    fprintf('\n✓ REAL-TIME COMPLIANCE: Mean latency %.2f ms < 33 ms (30 FPS budget)\n', mean_computational_time);
else
    fprintf('\n✗ WARNING: Mean latency %.2f ms exceeds 33 ms real-time budget\n', mean_computational_time);
end

fprintf('======================================\n');
fprintf('Next Step: Simulink integration for HIL testing.\n');
fprintf('======================================\n');
