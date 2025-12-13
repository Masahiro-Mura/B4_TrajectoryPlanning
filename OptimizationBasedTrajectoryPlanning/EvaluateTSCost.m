function [Costs, LatCost, TimeCost, SpeedCost, DistanceCost, AdjForwardCost, AdjBehindCost] = EvaluateTSCost(terminalStates, times, laneWidth, speedLimit, SpeedWeight, LatWeight, TimeWeight, LeadingGapWeight, AdjForwardGapWeight, AdjBehindGapWeight, ForwardGaps)
    
    % Find lateral deviation from nearest lane center
    % laneCenters = (1.5:-1:-1.5)*laneWidth; % 4車線用
    laneCenters = (1:-1:-1)*laneWidth; % 3車線用
    
    latDeviation = abs(laneCenters-terminalStates(:,4)); % 各候補の終端位置と各車線中心との距離, 4とはs, ds, dds, l, dl, ddlの4番目ということ
    
    % Calculate lateral deviation cost (column vector)
    LatCost = min(latDeviation,[],2) * LatWeight;

    % Ensure times is a column vector so all cost terms are column vectors
    TimeCost = (1 ./times) * TimeWeight;

    % Calculate terminal speed vs desired speed cost (column vector)
    SpeedCost = abs(terminalStates(:,2)-speedLimit) * SpeedWeight;

    % ここに先行車両との車間距離の項を追記
    DistanceCost = (1 ./ForwardGaps) * LeadingGapWeight;

    % ここに隣接前方車両との車間距離の項を追記
    AdjForwardCost = (1 ./AdjForwardGap) * AdjForwardGapWeight;

    % ここに隣接後方車両との車間距離の項を追記
    AdjBehindCost = (1 ./AdjBehindGap) * AdjBehindGapWeight;
    
    % Return cumulative cost as a column vector (one scalar per terminal)
    Costs = LatCost + TimeCost + SpeedCost + DistanceCost + AdjForwardCost + AdjBehindCost;

end