%% 正規化 + コスト関数
function [costs, latCost, timeCost, speedCost, ForwardSafeCost, BehindSafeCost, MEscoreCost] = ...
    CostFunction_Normalization(terminalStates, times, ...
                                SpeedLimit, ...
                                SpeedWeight, LatWeight, TimeWeight, ForwardSafetyWeight, BehindSafetyWeight, MEscoreWeight, c1, ...
                                dForwardObs, dBehindObs, forwardsafetyGap, behindsafetyGap, EgoFrenetState, carLen, rearAxleRatio, futureTrajectory, actorVTargets, laneWidth, dt)
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
    numCandidates = size(terminalStates,1);
    numActors = numel(futureTrajectory);

    v_actor  = nan(numCandidates,1);
    v_leader = terminalStates(:,2);
    gap      = dBehindObs;
    v_target = nan(numCandidates,1);
    a_leader = terminalStates(:,3);
    a_prev   = nan(numCandidates,1);

    numSteps = size(futureTrajectory(1).Trajectory,1);

    for i = 1:numCandidates
        tpred = times(i);
        stepIdx = min(max(floor(tpred/dt)+1,1), numSteps);

        egoS = terminalStates(i,1); 
        egoL = terminalStates(i,4); 
        EgoFrenetRear = egoS - carLen * rearAxleRatio;

        gaps = nan(numActors,1);
        speeds = nan(numActors,1);
        acc_prevs = nan(numActors,1);
        for a = 1:numActors
            trajA = futureTrajectory(a).Trajectory;
            if size(trajA,1) >= stepIdx
                actorX = trajA(stepIdx,1);
                actorY = trajA(stepIdx,2);
                actorFront = actorX + carLen * (1-rearAxleRatio);

                if abs(actorY - egoL) <= laneWidth/2 && actorFront < EgoFrenetRear
                    gaps(a) = EgoFrenetRear - actorFront;
                    speeds(a) = trajA(stepIdx,5);
                    if stepIdx > 1
                        acc_prevs(a) = trajA(stepIdx-1,6);
                    else
                        acc_prevs(a) = trajA(stepIdx,6);
                    end
                end
            end
        end

        valid = find(~isnan(gaps));
        if ~isempty(valid)
            [~, rel] = min(gaps(valid));
            sel = valid(rel);
            v_actor(i) = speeds(sel);
            a_prev(i) = acc_prevs(sel);
            if exist('actorVTargets','var') && numel(actorVTargets) >= sel
                v_target(i) = actorVTargets(sel);
            else
                v_target(i) = SpeedLimit;
            end
        else
            v_actor(i) = nan;
            a_prev(i)   = nan;
            v_target(i) = SpeedLimit;
        end
    end

    acc = nan(numCandidates,1);
    jerk = nan(numCandidates,1);
    for i = 1:numCandidates
        if ~isnan(v_actor(i))
            [acc(i), jerk(i)] = IDM_improve(v_actor(i), v_leader(i), gap(i), v_target(i), a_leader(i), a_prev(i), dt);
        else
            acc(i) = 0;
            jerk(i) = 0;
        end
    end
    MEsocore_raw = arrayfun(@(a,j) MEscore(a,j), acc, abs(jerk));
    
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