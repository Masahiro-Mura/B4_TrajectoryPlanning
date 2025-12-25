function [terminalStates, times, leadGaps, behindGaps] = ...
TSCruiseControl(refPath, laneWidth, egoState, targetVelocity, dt, AllActorGlobalState)

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

    % 先行車 / 後方車との距離計算
    if nargin < 6 || isempty(AllActorGlobalState)
        leadGaps = inf(numel(dt),1);
        behindGaps = inf(numel(dt),1);
    else
        % 先行車との距離
        try
            leadGaps = TSForwardObstacleDistances(refPath, egoState, AllActorGlobalState, laneWidth, dt(:)', dt(:)');
        catch
            % If something goes wrong, fall back to inf to avoid breaking caller
            leadGaps = inf(numel(dt),1);
        end
        leadGaps = leadGaps(:);

        % 後方車との距離
        try
            behindGaps = TSBehindObstacleDistances(refPath, egoState, AllActorGlobalState, laneWidth, dt(:)', dt(:)');
        catch
            % If something goes wrong, fall back to inf to avoid breaking caller
            behindGaps = inf(numel(dt),1);
        end
        behindGaps = behindGaps(:);
    end
end