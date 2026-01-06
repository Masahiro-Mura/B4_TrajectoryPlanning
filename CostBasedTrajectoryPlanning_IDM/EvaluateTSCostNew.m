%% 正規化＋一貫性コスト追加
function [costs, latCost, timeCost, speedCost, consistencyCost] ...
    = EvaluateTSCostNew( ...
    terminalStates, times, laneWidth, speedLimit, ...
    SpeedWeight, LatWeight, TimeWeight, ConsistencyWeight, ...
    prevTS, prevT, modeBinLog)

    laneCenters = (1:-1:-1)*laneWidth;
    latDeviation = abs(laneCenters - terminalStates(:,4));

    % 横偏差コスト
    LatCost_raw   = min(latDeviation,[],2);

    % 時間差コスト
    TimeCost_raw  = 1 ./ times(:);

    % 速度差コスト
    SpeedCost_raw = abs(terminalStates(:,2) - speedLimit);

    % 一貫性コスト（前回情報が無い最初の一回は0）
    if nargin < 5 || isempty(prevTS) || isempty(prevT)
        Consistency_raw = zeros(size(times(:)));  % 最初は0
    else
        % モード間の差分（L2ノルムまたは単純な差分）
        prevMode = modeBinLog(prevT,:);  % 前回選ばれたモード
        currentMode = modeBinLog(end,:); % 現在選ばれたモード
        Consistency_raw = currentMode - prevMode;
    end

    % 正規化
    latCost_n   = minmax01(LatCost_raw);
    timeCost_n  = minmax01(TimeCost_raw);
    speedCost_n = minmax01(SpeedCost_raw);
    consCost_n  = minmax01(Consistency_raw);

    % 重み付きコスト
    latCost         = LatWeight         * latCost_n;
    timeCost        = TimeWeight        * timeCost_n;
    speedCost       = SpeedWeight       * speedCost_n;
    consistencyCost = ConsistencyWeight * consCost_n;

    costs = latCost + timeCost + speedCost + consistencyCost;
end