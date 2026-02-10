function state = updateLaneState(state, leftPoints, rightPoints, imHeight, params)
% UPDATELANESTATE Fits polynomials and updates tracking state with smoothing
%   Performs robust polyfit, rejects outliers, and applies Exponential Moving Average.

    currPolyL = [];
    currPolyR = [];

    % 1. Quality Check
    leftCount = size(leftPoints, 1);
    rightCount = size(rightPoints, 1);
    leftSpan = 0;
    rightSpan = 0;
    
    if leftCount > 1
        leftSpan = max(leftPoints(:,2)) - min(leftPoints(:,2));
    end
    if rightCount > 1
        rightSpan = max(rightPoints(:,2)) - min(rightPoints(:,2));
    end
    
    leftGood = leftCount >= params.MIN_PTS && leftSpan >= params.MIN_YSPAN * imHeight;
    rightGood = rightCount >= params.MIN_PTS && rightSpan >= params.MIN_YSPAN * imHeight;

    % 2. Polynomial Fitting
    if leftGood && size(leftPoints, 1) >= 3
        currPolyL = polyfit(leftPoints(:,2), leftPoints(:,1), 2);
    elseif leftGood && size(leftPoints, 1) >= 2
        currPolyL = polyfit(leftPoints(:,2), leftPoints(:,1), 1);
    end

    if rightGood && size(rightPoints, 1) >= 3
        currPolyR = polyfit(rightPoints(:,2), rightPoints(:,1), 2);
    elseif rightGood && size(rightPoints, 1) >= 2
        currPolyR = polyfit(rightPoints(:,2), rightPoints(:,1), 1);
    end

    % Normalize to degree 2
    currPolyL = normalizePoly(currPolyL);
    currPolyR = normalizePoly(currPolyR);

    % 3. Outlier Rejection (Jump Check)
    if ~isempty(currPolyL) && ~isempty(state.lastGoodPolyL)
        yCheck = [imHeight; round(imHeight * 0.7); round(imHeight * 0.55)];
        xCurr = polyval(currPolyL, yCheck);
        xPrev = polyval(state.lastGoodPolyL, yCheck);
        if mean(abs(xCurr - xPrev)) > params.MAX_JUMP_PX
            currPolyL = [];
        end
    end

    if ~isempty(currPolyR) && ~isempty(state.lastGoodPolyR)
        yCheck = [imHeight; round(imHeight * 0.7); round(imHeight * 0.55)];
        xCurr = polyval(currPolyR, yCheck);
        xPrev = polyval(state.lastGoodPolyR, yCheck);
        if mean(abs(xCurr - xPrev)) > params.MAX_JUMP_PX
            currPolyR = [];
        end
    end

    % 4. Temporal Smoothing (Left)
    if ~isempty(currPolyL)
        if isempty(state.avgPolyL)
            state.avgPolyL = currPolyL;
        else
            state.avgPolyL = params.alpha * state.avgPolyL + (1 - params.alpha) * currPolyL;
        end
        state.lastGoodPolyL = state.avgPolyL;
        state.lostL = 0;
    else
        state.lostL = state.lostL + 1;
        if state.lostL <= params.MAX_LOST && ~isempty(state.lastGoodPolyL)
            state.avgPolyL = state.lastGoodPolyL; % Hold last known position
        else
            state.avgPolyL = []; % Reset if lost too long
        end
    end

    % 5. Temporal Smoothing (Right)
    if ~isempty(currPolyR)
        if isempty(state.avgPolyR)
            state.avgPolyR = currPolyR;
        else
            state.avgPolyR = params.alpha * state.avgPolyR + (1 - params.alpha) * currPolyR;
        end
        state.lastGoodPolyR = state.avgPolyR;
        state.lostR = 0;
    else
        state.lostR = state.lostR + 1;
        if state.lostR <= params.MAX_LOST && ~isempty(state.lastGoodPolyR)
            state.avgPolyR = state.lastGoodPolyR;
        else
            state.avgPolyR = [];
        end
    end
end

function poly = normalizePoly(poly)
% Ensures polynomial is always degree 2 (padded with 0 if degree 1)
    if ~isempty(poly) && length(poly) == 2
        poly = [0 poly];
    end
end
