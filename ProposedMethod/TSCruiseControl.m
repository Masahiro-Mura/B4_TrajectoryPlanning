function [terminalStates, times, leadGaps, behindGaps] = ...
TSCruiseControl(refPath, laneWidth, egoState, targetVelocity, dt, AllActorGlobalState)
    
    frenetState = global2frenet(refPath, egoState);
    futureLane = PredictLane(frenetState, laneWidth, dt);
    
    P = parameters();
    lateralOffsets = (P.laneCount/2 - futureLane + 0.5) * laneWidth; % 車線
    
    terminalStates      = zeros(numel(dt),6);
    terminalStates(:,1) = nan;
    terminalStates(:,2) = targetVelocity;
    terminalStates(:,4) = lateralOffsets;
    times = dt(:);

    % 先行車との距離
    try
        leadGaps = TSForwardObstacleDistances(refPath, egoState, AllActorGlobalState, laneWidth, dt(:)', dt(:)');
    catch
        leadGaps = inf(numel(dt),1);
    end
    leadGaps = leadGaps(:);
    
    % 後方車との距離
    try
        behindGaps = TSBehindObstacleDistances(refPath, egoState, AllActorGlobalState, laneWidth, dt(:)', dt(:)');
    catch
        behindGaps = inf(numel(dt),1);
    end
    behindGaps = behindGaps(:);
end