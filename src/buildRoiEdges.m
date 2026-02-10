function [roiYellow, roiWhite, imHeight, imWidth, midX] = buildRoiEdges(frame, thresholds)
% BUILDROIEDGES Extracts edges within a region of interest
%   Applies color masking, Canny edge detection, and ROI masking.

    % Create Yellow HSV/RGB Mask
    yellowMask = (frame(:,:,1) >= thresholds.yellowRLo & frame(:,:,1) <= 255) & ...
                 (frame(:,:,2) >= thresholds.yellowGLo & frame(:,:,2) <= 255) & ...
                 (frame(:,:,3) >= 0 & frame(:,:,3) <= thresholds.yellowBHi);

    % Create White RGB Mask
    whiteMask = (frame(:,:,1) >= thresholds.whiteLo & frame(:,:,1) <= 255) & ...
                (frame(:,:,2) >= thresholds.whiteLo & frame(:,:,2) <= 255) & ...
                (frame(:,:,3) >= thresholds.whiteLo & frame(:,:,3) <= 255);

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
