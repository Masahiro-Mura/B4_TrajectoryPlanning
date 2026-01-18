%% 正規化 + コスト関数
function [costs, latCost, timeCost, speedCost] = CostFunction_Conv( ...
    terminalStates, times, SpeedLimit, SpeedWeight, LatWeight, TimeWeight, EgoFrenetState)
    %% １．横偏差コスト（レーン中心からのずれ）
    LatCost_raw = abs(terminalStates(:,4) - EgoFrenetState(4));


    %% ２．時間差コスト（短時間の方がコスト小）
    TimeCost_raw  = 1 ./ times(:);


    %% ３．速度差コスト（SpeedLimit からのずれ）
    SpeedCost_raw = abs(terminalStates(:,2) - SpeedLimit);

    
    %% 各コスト 正規化
    latCost_n         = minmax01(LatCost_raw);
    timeCost_n        = minmax01(TimeCost_raw);
    speedCost_n       = minmax01(SpeedCost_raw);

    
    %% 重み付け
    latCost         = LatWeight           * latCost_n;
    timeCost        = TimeWeight          * timeCost_n;
    speedCost       = SpeedWeight         * speedCost_n;

    
    %% 総コスト
    costs = latCost + timeCost + speedCost;
end