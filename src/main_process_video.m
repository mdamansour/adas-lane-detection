%% MAIN_PROCESS_VIDEO: Orchestrates the full Lane Detection Pipeline
%
% This script serves as the main entry point for the ADAS Lane Detection System.
% It integrates modular components for:
% 1. Video I/O
% 2. Preprocessing (Gaussian + Adaptive Thresholds)
% 3. Edge Detection & ROI
% 4. Hough Transform & Line Selection
% 5. Polynomial Fitting & Temporal Stabilization
% 6. Vanishing Point Estimation & Visualization
%
% Author: Mohammed Amansour
% Date: Feb 10, 2026

clear; clc; close all;

%% 1. Setup Input/Output
[videoObj, outputVideo, videoPath, outputPath] = setupVideoIO();

% Console Status
fprintf('------------------------------------------------\n');
fprintf('Main: Processing video %s\n', videoPath);
fprintf('Main: Output destination %s\n', outputPath);
fprintf('------------------------------------------------\n');

%% 2. Initialize State and Parameters
params = getLaneDetectionParams();
state = initLaneState();

frameCount = 0;
startTime = tic;

%% 3. Main Frame Processing Loop
while hasFrame(videoObj)
    frameCount = frameCount + 1;
    
    % A. Read and Preprocess
    % Note: 3D Gaussian filter is expensive but vital for noise reduction
    frame = imgaussfilt3(readFrame(videoObj));
    thresholds = getAdaptiveThresholds(frame);
    
    % B. Edge Detection and ROI
    [roiYellow, roiWhite, imHeight, imWidth, midX] = buildRoiEdges(frame, thresholds);
    
    % C. Hough Transform
    linesAll = detectHoughLines(roiYellow, roiWhite);
    
    % D. Lane Candidate Selection (with ROI Tracking)
    [leftPoints, rightPoints] = collectLanePoints(linesAll, imHeight, imWidth, midX, state.avgPolyL, state.avgPolyR);
    
    % E. State Update (PolyFit + Smoothing)
    state = updateLaneState(state, leftPoints, rightPoints, imHeight, params);
    
    % F. High-Level Logic (Turn Prediction)
    [vanishingPoint, direction] = computeVanishingPoint(state.avgPolyL, state.avgPolyR, imHeight, imWidth);
    
    % G. Generate Visualization Curves
    [xLeftPred, xRightPred, yRange, state] = generateLaneCurves(state, imHeight, imWidth, vanishingPoint, params);
    
    % H. Render Overlay
    outputFrame = drawOverlay(frame, xLeftPred, xRightPred, yRange, vanishingPoint, direction, imWidth);
    writeVideo(outputVideo, outputFrame);

    % Optional: Performance logging
    if mod(frameCount, 30) == 0
        currentElapsed = toc(startTime);
        fps = frameCount / currentElapsed;
        fprintf('Processed %d frames | Avg FPS: %.2f\n', frameCount, fps);
    end
end

%% 4. Cleanup
close(outputVideo);
totalElapsed = toc(startTime);
fprintf('------------------------------------------------\n');
fprintf('Done. Total frames: %d | Total Time: %.2fs | Avg FPS: %.2f\n', ...
    frameCount, totalElapsed, frameCount / totalElapsed);
fprintf('------------------------------------------------\n');
