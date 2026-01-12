%% 正規化 + コスト関数
function [costs, latCost, timeCost, speedCost, ForwardSafeCost, BehindSafeCost] = ...
    CostFunction_Normalization(terminalStates, times, ...
                                ~, SpeedLimit, ...
                                SpeedWeight, LatWeight, TimeWeight, ForwardSafetyWeight, BehindSafetyWeight, MEscoreWeight, c1, ...
                                dForwardObs, dBehindObs, forwardsafetyGap, behindsafetyGap, EgoGlobalState, EgoFrenetState, carLen, rearAxleRatio)
    %% １．横偏差コスト（レーン中心からのずれ）
    % laneCenters   = (1:-1:-1) * laneWidth;  % 3車線用
    % latDeviation  = abs(laneCenters - terminalStates(:,4));
    % LatCost_raw   = min(latDeviation, [], 2);
    LatCost_raw = abs(terminalStates(:,4) - EgoFrenetState(4));


    %% ２．時間差コスト（短時間の方がコスト小）
    TimeCost_raw  = 1 ./ times(:);


    %% ３．速度差コスト（SpeedLimit からのずれ）
    SpeedCost_raw = abs(terminalStates(:,2) - SpeedLimit);


    %% ４．前方向安全コスト
    % dObs: 各軌道 i の「前方障害物までの距離」 [m]
    ForwardSafeCost_raw = zeros(size(dForwardObs));

    % 相対距離: 0なら安全距離ちょうど, 負なら安全距離の内側
    dRelForward = dForwardObs - carLen * rearAxleRatio - forwardsafetyGap;

    % 障害物無：軌道はコスト0
    isObsFree = isinf(dForwardObs) | isnan(dForwardObs) ;
    ForwardSafeCost_raw(isObsFree) = 0;

    % 障害物有：距離が近いほどコストが大きくなるロジスティック関数
    idxForward = ~isObsFree;
    ForwardSafeCost_raw(idxForward) = 2 - 2 ./ (1 + exp(-c1 .* dRelForward(idxForward)));


    %% ５．後方安全コスト
    % dObs: 各軌道 i の「後方障害物までの距離」 [m]
    BehindSafeCost_raw = zeros(size(dBehindObs));

    % 相対距離: 0なら安全距離ちょうど, 負なら安全距離の内側
    dRelBehind = dBehindObs - carLen * rearAxleRatio - behindsafetyGap;

    % 障害物無：軌道はコスト0
    isObsFreeBehind = isinf(dBehindObs) | isnan(dBehindObs);
    BehindSafeCost_raw(isObsFreeBehind) = 0;

    % 障害物有：距離が近いほどコストが大きくなるロジスティック関数
    idxBehind = ~isObsFreeBehind;
    BehindSafeCost_raw(idxBehind) = 2 - 2 ./ (1 + exp(-c1 .* dRelBehind(idxBehind)));

    %% ６．快適性コスト
    dt = scenario.SampleTime; % これの引数への入れ方？？？
    [acc, jerk]  = IDM_improve(v_actor, EgoGlobalState(5), gap, v_target, EgoGlobalState(6), a_prev, dt);
    MEsocore_raw = MEscore(acc, jerk);
    
    %% 各コスト 正規化
    latCost_n         = minmax01(LatCost_raw);
    timeCost_n        = minmax01(TimeCost_raw);
    speedCost_n       = minmax01(SpeedCost_raw);
    ForwardSafeCost_n = minmax01(ForwardSafeCost_raw);
    BehindSafeCost_n  = minmax01(BehindSafeCost_raw);
    MEsocore_n        = minmax01(MEsocore_raw);

    %% 重み付け
    latCost         = LatWeight           * latCost_n;
    timeCost        = TimeWeight          * timeCost_n;
    speedCost       = SpeedWeight         * speedCost_n;
    ForwardSafeCost = ForwardSafetyWeight * ForwardSafeCost_n;
    BehindSafeCost  = BehindSafetyWeight  * BehindSafeCost_n;
    MEscoreCost     = MEscoreWeight       * MEsocore_n;

    %% 総コスト
    costs = latCost + timeCost + speedCost + ForwardSafeCost + BehindSafeCost + MEscoreCost;
end