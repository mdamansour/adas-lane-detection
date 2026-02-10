function linesAll = detectHoughLines(roiYellow, roiWhite)
% DETECTHOUGHLINES Finds line segments in binary edge images using Hough Transform
%   Combines detections from both yellow and white edge masks.

    % Hough Transform for Yellow
    [H_Y, theta_Y, rho_Y] = hough(roiYellow);
    
    % Hough Transform for White
    [H_W, theta_W, rho_W] = hough(roiWhite);
    
    numPeaks = 10;
    
    % Find peaks in Hough accumulator
    P_Y = houghpeaks(H_Y, numPeaks, 'threshold', 2);
    P_W = houghpeaks(H_W, numPeaks, 'threshold', 2);
    
    % Extract line segments
    linesYellow = houghlines(roiYellow, theta_Y, rho_Y, P_Y, 'FillGap', 3000, 'MinLength', 20);
    linesWhite = houghlines(roiWhite, theta_W, rho_W, P_W, 'FillGap', 3000, 'MinLength', 20);
    
    linesAll = [linesYellow, linesWhite];
end
