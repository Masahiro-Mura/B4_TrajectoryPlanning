%% 正規化 + コスト関数
function [costs, latCost, timeCost, speedCost, ForwardSafeCost, BehindSafeCost, MEscoreCost, targetSpeedCost, gapRatioCost] = ...
    CostFunction_Prop(terminalStates, times, ...
                                SpeedLimit, ...
                                SpeedWeight, LatWeight, TimeWeight, ForwardSafetyWeight, BehindSafetyWeight, MEscoreWeight, TargetSpeedWeight, GapRatioWeight, c1, ...
                                                            dForwardObs, dBehindObs, forwardsafetyGap, behindsafetyGap, EgoFrenetState, carLen, rearAxleRatio, futureTrajectory, actorVTargets, laneWidth, refPath, dt)
    P = parameters();
    %% １．横偏差コスト（レーン中心からのずれ）
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
    gap = nan(numCandidates, 1);
    v_target = nan(numCandidates,1);
    a_leader = terminalStates(:,3);
    a_prev   = nan(numCandidates,1);

    numSteps = size(futureTrajectory(1).Trajectory,1);

    for i = 1:numCandidates
        tpred = times(i);
        stepIdx = min(max(floor(tpred/dt)+1,1), numSteps);

        % --- NaN の場合の s 座標推定 ---
        egoS = terminalStates(i,1); 
        if isnan(egoS)
            % s が NaN の場合、(現在の位置 + 平均速度 * 時間) で推定
            egoS = EgoFrenetState(1) + (EgoFrenetState(2) + terminalStates(i,2))/2 * tpred;
        end
        
        egoL = terminalStates(i,4); 
        EgoFrenetRear = egoS - carLen * rearAxleRatio;

        gaps = nan(numActors,1);
        speeds = nan(numActors,1);
        acc_prevs = nan(numActors,1);
        
        for a = 1:numActors
            trajA = futureTrajectory(a).Trajectory;
            if size(trajA,1) >= stepIdx
                % --- アクターの座標を Frenet に変換して比較 ---
                % すでに AllActorGlobalState などから Frenet 位置が計算されている場合はそれを使う
                % ここでは global2frenet を使用（計算負荷に注意）
                actorGlobal = trajA(stepIdx, 1:6);
                actorFrenet = global2frenet(refPath, actorGlobal);
                
                actorS_front = actorFrenet(1) + carLen * (1-rearAxleRatio);
                actorL = actorFrenet(4);
                
                % 同じ車線かつ後方にいるか判定
                if abs(actorL - egoL) <= laneWidth/2 && actorS_front < EgoFrenetRear
                    gaps(a) = EgoFrenetRear - actorS_front;
                    speeds(a) = actorGlobal(5);
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
            [minGap, rel] = min(gaps(valid));
            sel = valid(rel);
            gap(i) = minGap;
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

    % 予測速度を格納する配列
    v_predicted = nan(numCandidates,1); 
    acc = nan(numCandidates,1);
    jerk = nan(numCandidates,1);

    for i = 1:numCandidates
        if ~isnan(v_actor(i))
            [acc(i), jerk(i)] = IDM_improve(v_actor(i), v_leader(i), gap(i), v_target(i), a_leader(i), a_prev(i), dt);
            
            % 終端時刻 times(i) における予測速度を計算
            v_predicted(i) = max(0, v_actor(i) + acc(i) * times(i));
        else
            acc(i) = 0;
            jerk(i) = 0;
            v_predicted(i) = nan;
        end
    end
    MEsocore_raw = arrayfun(@(a,j) MEscore(a,j), acc, abs(jerk));
    
    %% 7. 後方車両の目標速度乖離
    targetSpeedCost_raw = zeros(numCandidates,1);
    validIdx = find(~isnan(v_predicted)); % v_actor ではなく v_predicted を使う
    if ~isempty(validIdx)
        % 予測された終端速度と目標速度の差分をとる
        targetSpeedCost_raw(validIdx) = abs(v_target(validIdx) - v_predicted(validIdx));
    end

    %% 8. 後方車両との車間距離比率
    gapRatioCost_raw = zeros(numCandidates,1);

    % v_actor: 後方車両の現在速度, v_target: 後方車両の目標速度
    validActor = find(~isnan(v_predicted));
    if ~isempty(validActor)
        % 車間距離比率コスト (s_star / d)^2
        dlta_v_n = v_leader(validActor) - v_predicted(validActor);
        s_star_full = nan(numCandidates,1);
        s_star_full(validActor) = P.S0 + v_predicted(validActor) .* P.T - (v_predicted(validActor) .* dlta_v_n) ./ (2 * sqrt(P.a * P.b));

        % 計算可能なギャップインデックス
        validGapIdx = validActor(gap(validActor) > 0);
        if ~isempty(validGapIdx)
            gapRatioCost_raw(validGapIdx) = (s_star_full(validGapIdx) ./ gap(validGapIdx)).^2;
        end
    end

    %% 各コスト 正規化
    latCost_n         = minmax01(LatCost_raw);
    timeCost_n        = minmax01(TimeCost_raw);
    speedCost_n       = minmax01(SpeedCost_raw);
    ForwardSafeCost_n = minmax01(ForwardSafeCost_raw);
    BehindSafeCost_n  = minmax01(BehindSafeCost_raw);
    MEsocore_n        = minmax01(MEsocore_raw);
    targetSpeedCost_n = minmax01(targetSpeedCost_raw);
    gapRatioCost_n    = minmax01(gapRatioCost_raw);
    
    %% 重み付け
    latCost         = LatWeight           * latCost_n;
    timeCost        = TimeWeight          * timeCost_n;
    speedCost       = SpeedWeight         * speedCost_n;
    ForwardSafeCost = ForwardSafetyWeight * ForwardSafeCost_n;
    BehindSafeCost  = BehindSafetyWeight  * BehindSafeCost_n;
    MEscoreCost     = MEscoreWeight       * MEsocore_n;
    targetSpeedCost = TargetSpeedWeight   * targetSpeedCost_n;
    gapRatioCost    = GapRatioWeight      * gapRatioCost_n;
    
    %% 総コスト
    costs = latCost + timeCost + speedCost + ForwardSafeCost*0 + BehindSafeCost*0 + MEscoreCost*0 + targetSpeedCost + gapRatioCost;
    % costs = ForwardSafeCost*0 + MEscoreCost*0 + targetSpeedCost + gapRatioCost;
end