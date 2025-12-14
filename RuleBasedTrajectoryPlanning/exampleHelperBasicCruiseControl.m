function [terminalStates, times] = exampleHelperBasicCruiseControl(refPath, laneWidth, egoState, targetVelocity, dt)

    frenetState = global2frenet(refPath, egoState); 

    futureLane = exampleHelperPredictLane(frenetState, laneWidth, dt); 

    % lateralOffsets = (2-futureLane+.5)*laneWidth; %4車線用
    lateralOffsets = -futureLane*laneWidth+7.2; % 3車線用

    terminalStates      = zeros(numel(dt),6);
    terminalStates(:,1) = nan;                 
    terminalStates(:,2) = targetVelocity;                  
    terminalStates(:,4) = lateralOffsets;      
    times = dt(:);
end


