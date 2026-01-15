%% 元のコスト関数
function [costs, latCost, timeCost, speedCost] = EvaluateTSCost(terminalStates, times, laneWidth, SpeedLimit, speedWeight, latWeight, timeWeight)
    
    P = parameters();
    laneCenters = ((P.laneCount-1)/2:-1:-(P.laneCount-1)/2)*laneWidth; % 車線用

    latDeviation = abs(laneCenters-terminalStates(:,4)); % 各候補の終端位置と各車線中心との距離, 4とはs, ds, dds, l, dl, ddlの4番目ということ

    % 横偏差コスト
    latCost = min(latDeviation,[],2) * latWeight;

    % 時間差コスト
    timeCost = (1 ./times) * timeWeight;

    % 速度差コスト
    speedCost = abs(terminalStates(:,2)-SpeedLimit) * speedWeight;

    % 全合計コスト
    costs = latCost + timeCost + speedCost;

end


