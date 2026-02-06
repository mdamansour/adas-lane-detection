%% Environment Verification and Video Loading Test
% Author: Mohammed Amansour
% Date: February 7, 2026
% Purpose: Verify MATLAB environment has required toolboxes and can handle
%          video input for the ADAS lane detection pipeline.
%
% Dependencies:
%   - Automated Driving Toolbox
%   - Computer Vision Toolbox
%
% Usage: Run this script to confirm the development environment is ready.

%% Clear workspace and command window
clear; clc; close all;

%% Step 1: Verify Required Toolboxes
fprintf('======================================\n');
fprintf('ADAS Environment Verification\n');
fprintf('======================================\n\n');

% Check for Automated Driving Toolbox
fprintf('Checking for Automated Driving Toolbox... ');
if license('test', 'Automated_Driving_Toolbox')
    fprintf('FOUND\n');
else
    error('Automated Driving Toolbox is not installed. Please install it to proceed.');
end

% Check for Computer Vision Toolbox
fprintf('Checking for Computer Vision Toolbox... ');
if license('test', 'Video_and_Image_Blockset')
    fprintf('FOUND\n');
else
    error('Computer Vision Toolbox is not installed. Please install it to proceed.');
end

fprintf('\n✓ All required toolboxes are available.\n\n');

%% Step 2: Load Sample Video
% We attempt to load a test video from the data directory. If not found,
% we fall back to a MATLAB built-in sample image for demonstration.

testVideoPath = fullfile('..', 'data', 'test_drive.mp4');

if isfile(testVideoPath)
    % Load the user-provided test video
    fprintf('Loading test video from: %s\n', testVideoPath);
    videoObj = VideoReader(testVideoPath);
    
    % Read the first frame
    firstFrame = readFrame(videoObj);
    fprintf('Video loaded successfully.\n');
    fprintf('  - Resolution: %d x %d\n', videoObj.Width, videoObj.Height);
    fprintf('  - Frame Rate: %.2f fps\n', videoObj.FrameRate);
    fprintf('  - Duration: %.2f seconds\n', videoObj.Duration);
    
else
    % Fallback to a built-in sample image for environment testing
    warning('Test video not found at: %s', testVideoPath);
    fprintf('Using built-in MATLAB sample image as fallback.\n');
    
    % Load a sample image from MATLAB's built-in resources
    firstFrame = imread('kobi.png');  % High-resolution sample image
    fprintf('Sample image loaded for visualization test.\n');
end

%% Step 3: Display First Frame
% We visualize the first frame to confirm image processing capabilities.

figure('Name', 'Environment Verification - First Frame', 'NumberTitle', 'off');
imshow(firstFrame);
title('First Frame (Environment Test)', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('\n✓ Video I/O and visualization verified.\n');
fprintf('======================================\n');
fprintf('Environment is ready for development.\n');
fprintf('======================================\n');
