%% 正規化済み
function [costs, latCost, timeCost, speedCost] = EvaluateTSCostNormalization(terminalStates, times, laneWidth, SpeedLimit, SpeedWeight, LatWeight, TimeWeight)

    laneCenters = (1:-1:-1)*laneWidth;  % 3車線用
    latDeviation = abs(laneCenters - terminalStates(:,4));

    % 横偏差コスト
    LatCost_raw   = (min(latDeviation,[],2));

    % 時間差コスト
    TimeCost_raw  = (1 ./ times(:));

    % 速度差コスト
    SpeedCost_raw = abs(terminalStates(:,2) - SpeedLimit);


    % 正規化
    latCost_n   = minmax01(LatCost_raw);
    timeCost_n  = minmax01(TimeCost_raw);
    speedCost_n = minmax01(SpeedCost_raw);

    % 重みづけ
    latCost   = LatWeight   * latCost_n;
    timeCost  = TimeWeight  * timeCost_n;
    speedCost = SpeedWeight * speedCost_n;

    costs = latCost + timeCost + speedCost;
end