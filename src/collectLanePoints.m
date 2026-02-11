function [leftPoints, rightPoints] = collectLanePoints(linesAll, imHeight, imWidth, midX, prevPolyL, prevPolyR)
% COLLECTLANEPOINTS Classifies Hough lines into Left/Right lane candidates
%   Filters lines by slope and position, then samples points for fitting.
%   Uses geometry to discern ego-lane from neighbors.
%   Implements ROI Tracking: if previous polynomials exist, restrict search
%   to ±50px corridor around them (eliminates false positives).

    leftPoints = [];
    rightPoints = [];
    
    TRACKING_CORRIDOR = 50;
    trackingActive = ~isempty(prevPolyL) && ~isempty(prevPolyR);

    for k = 1:length(linesAll)
        p1 = linesAll(k).point1;
        p2 = linesAll(k).point2;
        
        % Avoid vertical line division by zero (rare)
        if p2(1) == p1(1)
            continue;
        end
        
        slope = (p2(2) - p1(2)) / (p2(1) - p1(1));
        
        % Reject horizontal-ish lines
        if abs(slope) < 0.4
            continue;
        end
        
        % Project line to bottom of image to check interception
        xBottom = p1(1) + (imHeight - p1(2)) / slope;
        
        % Left Lane Logic: Negative slope, left of center
        validLeft = (slope < 0) && (midX > p1(1)) && (xBottom > -200 && xBottom < midX);
        
        % Right Lane Logic: Positive slope, right of center
        validRight = (slope > 0) && (midX < p1(1)) && (xBottom > midX && xBottom < imWidth + 200);

        if validLeft || validRight
            % Sample points along the line segment
            lineLen = norm(p1 - p2);
            numSamples = max(2, ceil(lineLen / 5));
            xSamp = linspace(p1(1), p2(1), numSamples)';
            ySamp = linspace(p1(2), p2(2), numSamples)';
            pts = [xSamp, ySamp];
            
            % Tracking Mode: Check if points fall within corridor
            if trackingActive
                if validLeft
                    xExpected = polyval(prevPolyL, pts(:,2));
                    withinCorridor = abs(pts(:,1) - xExpected) < TRACKING_CORRIDOR;
                    if sum(withinCorridor) / length(withinCorridor) > 0.6
                        leftPoints = [leftPoints; pts(withinCorridor, :)];
                    end
                else
                    xExpected = polyval(prevPolyR, pts(:,2));
                    withinCorridor = abs(pts(:,1) - xExpected) < TRACKING_CORRIDOR;
                    if sum(withinCorridor) / length(withinCorridor) > 0.6
                        rightPoints = [rightPoints; pts(withinCorridor, :)];
                    end
                end
            else
                % Acquisition Mode: Accept all valid points
                if validLeft
                    leftPoints = [leftPoints; pts];
                else
                    rightPoints = [rightPoints; pts];
                end
            end
        end
    end
end
