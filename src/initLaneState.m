function state = initLaneState()
% INITLANESTATE Initializes the tracking state structure
%   Creates empty containers for polynomial coefficients and lost counters.

    state = struct(...
        'avgPolyL', [], ...        % Smoothed Left Polynomial
        'avgPolyR', [], ...        % Smoothed Right Polynomial
        'lastGoodPolyL', [], ...   % Last valid Left Polynomial
        'lastGoodPolyR', [], ...   % Last valid Right Polynomial
        'lostL', 0, ...            % Consec frames left lane lost
        'lostR', 0, ...            % Consec frames right lane lost
        'avgLaneWidth', []);       % Smoothed lane width
end
