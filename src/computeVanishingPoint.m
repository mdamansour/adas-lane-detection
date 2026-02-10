function [vanishingPoint, direction] = computeVanishingPoint(avgPolyL, avgPolyR, imHeight, imWidth)
% COMPUTEVANISHINGPOINT Estimates the VP from lane intersection
%   Solves for intersection of Left and Right lane polynomials.
%   Uses VP horizontal position to predict turn direction.

    vanishingPoint = [];
    direction = 'Unknown';

    if ~isempty(avgPolyL) && ~isempty(avgPolyR)
        % Find intersection: PolyL(y) = PolyR(y) => (PolyL - PolyR)(y) = 0
        coeffs = avgPolyL - avgPolyR;
        rootsY = roots(coeffs);
        
        % Filter roots for valid image height range
        validRoots = rootsY(imag(rootsY) == 0 & rootsY > 0 & rootsY < imHeight);

        if ~isempty(validRoots)
            yV = min(validRoots); % Visual intersection is usually the "highest" point (min y)
            xV = polyval(avgPolyL, yV);
            
            if xV > 0 && xV < imWidth
                vanishingPoint = [xV, yV];
                
                % Turn Classification
                vanishingRatio = xV / imWidth;
                if vanishingRatio < 0.49
                    direction = 'Turn Left';
                elseif vanishingRatio <= 0.51
                    direction = 'Go Straight';
                else
                    direction = 'Turn Right';
                end
            end
        end
    end
end
