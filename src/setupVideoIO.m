function [videoObj, outputVideo, videoPath, outputPath] = setupVideoIO()
% SETUPVIDEOIO Initializes video reader and writer objects
%   Configures input/output paths for the ADAS lane detection system.
%   Assumes execution from the 'src' directory.

    videoPath = fullfile('..', 'data', 'test_drive.mp4');
    outputPath = fullfile('..', 'data', 'output_annotated.avi');

    % Create reader object
    videoObj = VideoReader(videoPath);

    % Create writer object with matching frame rate
    outputVideo = VideoWriter(outputPath);
    outputVideo.FrameRate = videoObj.FrameRate;
    open(outputVideo);
end
