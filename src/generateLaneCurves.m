function [xLeftPred, xRightPred, yRange, state] = generateLaneCurves(state, imHeight, imWidth, vanishingPoint, params)
% GENERATELANECURVES Evaluating polynomial models to generate visualization points
%   Synthesizes missing lanes using width constraints if needed.
%   Updates 'state.avgLaneWidth' based on reliable detections.

    % Define vertical range for drawing
    yTopBase = round(imHeight * 0.55);
    yTopMin = round(imHeight * 0.50); % Strict horizon clamp to avoid sky (was 0.45)
    yTopMax = round(imHeight * 0.75);

    % Adjust draw height based on vanishing point
    yTop = yTopBase;
    if ~isempty(state.avgPolyL) && ~isempty(state.avgPolyR) && ~isempty(vanishingPoint)
        % Don't draw all the way to the intersection (pointy tip).
        % Stop 15 pixels short for a natural "road interior" look.
        vpLimit = vanishingPoint(2) + 15;
        yTop = min(max(vpLimit, yTopMin), yTopMax);
    end

    yRange = linspace(imHeight, yTop, 50)';
    xLeftPred = [];
    xRightPred = [];

    % Evaluate current models
    if ~isempty(state.avgPolyL)
        xLeftPred = polyval(state.avgPolyL, yRange);
    end
    if ~isempty(state.avgPolyR)
        xRightPred = polyval(state.avgPolyR, yRange);
    end

    % Logic to handle single-lane or width stabilization
    if ~isempty(xLeftPred) && ~isempty(xRightPred)
        % Both lanes present - check consistency
        widthBottom = xRightPred(1) - xLeftPred(1);
        minWidth = 0.2 * imWidth;
        maxWidth = 0.9 * imWidth;

        if widthBottom >= minWidth && widthBottom <= maxWidth
            % Valid width - update tracked width
            if isempty(state.avgLaneWidth)
                state.avgLaneWidth = widthBottom;
            else
                state.avgLaneWidth = params.alphaWidth * state.avgLaneWidth + (1 - params.alphaWidth) * widthBottom;
            end
        else
            % Invalid width - fallback to last good
            if ~isempty(state.lastGoodPolyL) && ~isempty(state.lastGoodPolyR)
                xLeftPred = polyval(state.lastGoodPolyL, yRange);
                xRightPred = polyval(state.lastGoodPolyR, yRange);
            else
                xLeftPred = [];
                xRightPred = [];
            end
        end
    elseif ~isempty(state.avgLaneWidth)
        % Single lane case - synthesize missing side
        if ~isempty(xLeftPred)
            xRightPred = xLeftPred + state.avgLaneWidth;
        elseif ~isempty(xRightPred)
            xLeftPred = xRightPred - state.avgLaneWidth;
        end
    end
end
