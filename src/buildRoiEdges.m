function [roiYellow, roiWhite, imHeight, imWidth, midX] = buildRoiEdges(frame, thresholds)
% BUILDROIEDGES Extracts edges within a region of interest
%   Applies HSV-based color masking, Canny edge detection, and ROI masking.
%   HSV provides illumination invariance for robust detection in shadows/tunnels.

    % Convert to HSV color space
    hsvFrame = rgb2hsv(frame);
    H = hsvFrame(:,:,1);
    S = hsvFrame(:,:,2);
    V = hsvFrame(:,:,3);

    % Yellow Mask (HSV): Hue in yellow range + sufficient saturation + adaptive value
    yellowMask = (H >= thresholds.yellowHueLo & H <= thresholds.yellowHueHi) & ...
                 (S >= thresholds.yellowSatLo) & ...
                 (V >= thresholds.yellowValLo);

    % White Mask (HSV): Low saturation (grayscale) + high value
    whiteMask = (S <= thresholds.whiteSatHi) & ...
                (V >= thresholds.whiteValLo);

    % Edge Detection (Canny)
    edgesYellow = edge(yellowMask, 'canny', thresholds.edgeThresh);
    edgesWhite = edge(whiteMask, 'canny', thresholds.edgeThresh);

    % Morphological cleaning (remove small noise)
    edgesYellow = bwareaopen(edgesYellow, 15);
    edgesWhite = bwareaopen(edgesWhite, 15);

    % ROI Masking
    [imHeight, imWidth] = size(edgesYellow);
    roiTop = round(imHeight * thresholds.roiTopRatio);
    sideMargin = round(imWidth * 0.08);
    bottomMargin = round(imWidth * 0.04);
    
    roiVertices = [sideMargin, roiTop; 
                   imWidth-sideMargin, roiTop; 
                   imWidth-bottomMargin, imHeight; 
                   bottomMargin, imHeight];
                   
    roiMask = poly2mask(roiVertices(:,1), roiVertices(:,2), imHeight, imWidth);
    
    roiYellow = edgesYellow & roiMask;
    roiWhite = edgesWhite & roiMask;
    midX = imWidth / 2;
end
