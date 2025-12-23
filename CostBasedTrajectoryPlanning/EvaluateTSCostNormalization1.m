%% 正規化 + 縦方向安全コスト
function [costs, latCost, timeCost, speedCost, longSafeCost] = ...
    EvaluateTSCostNormalization1(terminalStates, times, ...
                                laneWidth, SpeedLimit, ...
                                SpeedWeight, LatWeight, TimeWeight, ...
                                LongSafetyWeight, dObs, c1, forwardsafetyGap)

    % 既存のコスト計算
    laneCenters   = (1:-1:-1) * laneWidth;  % 3車線用
    latDeviation  = abs(laneCenters - terminalStates(:,4));

    % 横偏差コスト（レーン中心からのずれ）
    LatCost_raw   = min(latDeviation, [], 2);

    % 時間差コスト（短時間の方がコスト小）
    TimeCost_raw  = 1 ./ times(:);

    % 速度差コスト（SpeedLimit からのずれ）
    SpeedCost_raw = abs(terminalStates(:,2) - SpeedLimit);

    % 縦方向安全コスト
    % dObs: 各軌道 i の「前方障害物までの距離」 [m]
    LongSafeCost_raw = zeros(size(dObs));
    
    % 相対距離: 0なら安全距離ちょうど, 負なら安全距離の内側
    dRel = dObs - forwardsafetyGap;

    % 障害物無：軌道はコスト0
    isObsFree = isinf(dObs) | isnan(dObs) ;
    LongSafeCost_raw(isObsFree) = 0;

    % 障害物有：距離が近いほどコストが大きくなるロジスティック関数
    idx = ~isObsFree;
    LongSafeCost_raw(idx) = 2 - 2 ./ (1 + exp(-c1 .* dRel(idx)));

    % 各コスト 正規化
    latCost_n       = minmax01(LatCost_raw);
    timeCost_n      = minmax01(TimeCost_raw);
    speedCost_n     = minmax01(SpeedCost_raw);
    longSafeCost_n  = minmax01(LongSafeCost_raw);
    disp('LongSafeCost_raw');
    disp(LongSafeCost_raw);
    disp('longSafeCost_n');
    disp(longSafeCost_n);

    % 重み付け
    latCost      = LatWeight       * latCost_n;
    timeCost     = TimeWeight      * timeCost_n;
    speedCost    = SpeedWeight     * speedCost_n;
    longSafeCost = LongSafetyWeight * longSafeCost_n;

    % 総コスト
    costs = latCost + timeCost + speedCost + longSafeCost;
end
