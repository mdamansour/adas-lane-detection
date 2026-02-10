function outputFrame = drawOverlay(frame, xLeftPred, xRightPred, yRange, vanishingPoint, direction, imWidth)
% DRAWOVERLAY Renders lane polygons, lines, and turn info on the frame
%   Uses Computer Vision Toolbox functions for drawing.

    outputFrame = frame;

    if ~isempty(xLeftPred) && ~isempty(xRightPred)
        % Create polygon vertices
        v1 = [xLeftPred, yRange];
        v2 = [flipud(xRightPred), flipud(yRange)];
        verts = [v1; v2];

        % Check for crossover (invalid polygon)
        validPoints = sum(xLeftPred < xRightPred);
        if validPoints >= 0.9 * length(xLeftPred)
            xPoly = verts(:,1);
            yPoly = verts(:,2);
            polyInterleaved = [xPoly'; yPoly'];

            % Draw Green Lane Area
            outputFrame = insertShape(outputFrame, 'FilledPolygon', polyInterleaved(:)', ...
                'Color', 'green', 'Opacity', 0.4);
        end

        % Draw Boundary Lines
        outputFrame = insertShape(outputFrame, 'Line', [xLeftPred yRange], ...
            'Color', 'yellow', 'LineWidth', 5);
        outputFrame = insertShape(outputFrame, 'Line', [xRightPred yRange], ...
            'Color', 'yellow', 'LineWidth', 5);
    end

    % Draw Vanishing Point
    if ~isempty(vanishingPoint)
        outputFrame = insertShape(outputFrame, 'Circle', [vanishingPoint 6], ...
            'Color', 'red', 'LineWidth', 2);
    end

    % Draw Direction Text
    outputFrame = insertText(outputFrame, [imWidth/2-100, 30], direction, ...
        'FontSize', 24, 'BoxOpacity', 0.6, 'TextColor', 'red');
end
