function [costs, latCost, timeCost, speedCost] = EvaluateTSCost(terminalStates, times, laneWidth, speedLimit, speedWeight, latWeight, timeWeight)
    
    % Find lateral deviation from nearest lane center
    % laneCenters = (1.5:-1:-1.5)*laneWidth; % 4車線用
    laneCenters = (1:-1:-1)*laneWidth; % 3車線用
    
    latDeviation = abs(laneCenters-terminalStates(:,4)); % 各候補の終端位置と各車線中心との距離, 4とはs, ds, dds, l, dl, ddlの4番目ということ
    
    % Calculate lateral deviation cost (column vector)
    latCost = min(latDeviation,[],2) * latWeight;

    % Ensure times is a column vector so all cost terms are column vectors
    timeCost = (1 ./times) * timeWeight;

    % Calculate terminal speed vs desired speed cost (column vector)
    speedCost = abs(terminalStates(:,2)-speedLimit) * speedWeight;

    
    % Return cumulative cost as a column vector (one scalar per terminal)
    costs = latCost + timeCost + speedCost;

end