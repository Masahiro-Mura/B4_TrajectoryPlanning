function [Costs, LatCost, TimeCost, SpeedCost] = EvaluateTSCost(terminalStates, times, laneWidth, speedLimit, SpeedWeight, LatWeight, TimeWeight) 
% function [Costs, LatCost, TimeCost, SpeedCost, ForwardDistanceCost, BehindDistanceCost] = EvaluateTSCost(terminalStates, times, laneWidth, speedLimit, SpeedWeight, LatWeight, TimeWeight, LeadingGapWeight, BehindGapWeight) 
    % Find lateral deviation from nearest lane center
    % laneCenters = (1.5:-1:-1.5)*laneWidth; % 4車線用
    laneCenters = (1:-1:-1)*laneWidth; % 3車線用
    
    latDeviation = abs(laneCenters-terminalStates(:,4)); % 各候補の終端位置と各車線中心との距離, 4とはs, ds, dds, l, dl, ddlの4番目ということ
    
    % 横偏差コスト
    LatCost = min(latDeviation,[],2) * LatWeight;

    % 時間差コスト
    TimeCost = (1 ./times) * TimeWeight;

    % 速度差コスト
    SpeedCost = abs(terminalStates(:,2)-speedLimit) * Speed

    % ここに先行車両との車間距離の項を追記
    % ForwardDistanceCost = (1 ./( - terminalStates(:,1))) * LeadingGapWeight;

    % ここに後方車両との車間距離の項を追記
    % BehindDistanceCost  = (1 ./(terminalStates(:,1) - ))  * BehindGapWeight;
    
    % 全コスト 合計
    Costs = LatCost + TimeCost + SpeedCost;
    % Costs = LatCost + TimeCost + SpeedCost + ForwardDistanceCost + BehindDistanceCost;

end