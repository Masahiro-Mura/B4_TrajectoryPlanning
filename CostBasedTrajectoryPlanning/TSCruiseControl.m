function [terminalStates, times, leadGaps] = TSCruiseControl(refPath, laneWidth, egoState, targetVelocity, dt, AllActorGlobalState)

    % Convert ego to Frenet and predict lane offsets for each horizon
    frenetState = global2frenet(refPath, egoState);
    futureLane = PredictLane(frenetState, laneWidth, dt);

    % lateralOffsets = (2-futureLane+.5)*laneWidth; %4車線用
    lateralOffsets = -futureLane*laneWidth+7.2; % 3車線用

    % Prepare terminal states: (N x 6) where N = numel(dt)
    terminalStates      = zeros(numel(dt),6);
    terminalStates(:,1) = nan;
    terminalStates(:,2) = targetVelocity;
    terminalStates(:,4) = lateralOffsets;
    times = dt(:);

    % Compute predicted lead-vehicle gaps for each prediction horizon.
    % If actor information is not provided, return +inf (no lead).
    if nargin < 6 || isempty(AllActorGlobalState)
        leadGaps = inf(numel(dt),1);
    else
        % Reuse TSObstacleDistances which maps timeHorizons -> allDT
        try
            leadGaps = TSObstacleDistances(refPath, egoState, AllActorGlobalState, laneWidth, dt(:)', dt(:)');
        catch
            % If something goes wrong, fall back to inf to avoid breaking caller
            leadGaps = inf(numel(dt),1);
        end
        leadGaps = leadGaps(:);
    end
end