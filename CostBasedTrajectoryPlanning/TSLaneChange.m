function [terminalStates, times] = TSLaneChange( ...
    refPath, laneWidth, forwardsafetyGap, behindsafetyGap, ...
    EgoGlobalState, AllActorGlobalState, carLen, rearAxleRatio, dt)
%% --- 自車速度ゼロなら車線変更は行わない ---
    if EgoGlobalState(5) == 0
        terminalStates = [];
        times = [];
        return;
    end

    nDt = numel(dt);

    %% ==== 1. 自車 Frenet 変換 & 現在レーン取得 ====
    egoFrenet = global2frenet(refPath, EgoGlobalState);  % [s, s_dot, ..., d, ...] 想定
    sEgo0     = egoFrenet(1);
    vEgo      = EgoGlobalState(5);          % 終端状態での目標速度として使用
    curLane   = PredictLane(egoFrenet, laneWidth, 0);  % t=0 でのレーン

    %% ==== 2. 隣接レーン（左・右） ====
    % ここでは 3 車線道路を前提に laneID ∈ {1,2,3}
    rawAdjLanes   = curLane + [-1 1];          % 左,右
    validMask     = rawAdjLanes > 0 & rawAdjLanes <= 3;
    adjacentLanes = rawAdjLanes(validMask);
    numAdjLane    = numel(adjacentLanes);

    if numAdjLane == 0
        terminalStates = [];
        times = [];
        return;
    end

    %% ==== 3. 周辺車両 Frenet 変換 & レーン番号 ====
    numActors = size(AllActorGlobalState, 1);

    frenetActors = [];
    actorLanes   = [];

    if numActors > 0
        frenetActors = zeros(numActors, 6);
        actorLanes   = zeros(numActors, 1);

        % ついでに「自車と同じ位置っぽいもの」を除外する保険もかけておく
        dxy = vecnorm(AllActorGlobalState(:,1:2) - EgoGlobalState(1:2), 2, 2);
        AllActorGlobalState(dxy < 1e-3, :) = [];  % ほぼ同位置なら除外
        numActors = size(AllActorGlobalState, 1);

        if numActors > 0
            frenetActors = zeros(numActors, 6);
            actorLanes   = zeros(numActors, 1);
            for i = 1:numActors
                frenetActors(i,:) = global2frenet(refPath, AllActorGlobalState(i,:));
                actorLanes(i,1)   = PredictLane(frenetActors(i,:), laneWidth, 0);
            end
        end
    end

    %% ==== 4. 隣接レーンごとの先行車・後方車の「dt秒後 s」を計算 ====
    AdjacentForwardVehicle = NaN(numAdjLane, nDt);  % laneIdx x nDt
    AdjacentBehindVehicle  = NaN(numAdjLane, nDt);

    for laneIdx = 1:numAdjLane
        laneID = adjacentLanes(laneIdx);

        if numActors == 0
            continue;
        end

        % この隣接レーン上の車両だけ抜き出し
        idxLane = find(actorLanes == laneID);
        if isempty(idxLane)
            continue;  % このレーンには車両がいない
        end

        sActors = frenetActors(idxLane, 1);  % s 座標
        % vActors = frenetActors(idxLane, 2);  % s 方向速度 (global2frenet の仕様依存)

        % --- 先行車候補（時刻0で自車より前） ---
        forwardMask  = sActors > sEgo0;
        forwardIdxes = idxLane(forwardMask);
        if ~isempty(forwardIdxes)
            sF_all = frenetActors(forwardIdxes, 1);
            vF_all = frenetActors(forwardIdxes, 2);
            % 自車に最も近い先行車（s 差が最小）を選択
            [~, iMin] = min(sF_all - sEgo0);
            sF0 = sF_all(iMin);
            vF0 = vF_all(iMin);
            AdjacentForwardVehicle(laneIdx, :) = sF0 + vF0 * dt;  % 等速で予測
        end

        % --- 後方車候補（時刻0で自車より後ろ） ---
        behindMask  = sActors < sEgo0;
        behindIdxes = idxLane(behindMask);
        if ~isempty(behindIdxes)
            sB_all = frenetActors(behindIdxes, 1);
            vB_all = frenetActors(behindIdxes, 2);
            % 自車に最も近い後方車（s 差が最大のもの）を選択
            [~, iMax] = max(sB_all);  % sB_all は < sEgo0 なので最大値が最も近い
            sB0 = sB_all(iMax);
            vB0 = vB_all(iMax);
            AdjacentBehindVehicle(laneIdx, :) = sB0 + vB0 * dt;  % 等速で予測
        end
    end

    %% ==== 5. 完全分岐で終端状態を生成 ====
    terminalStates = [];
    times          = [];

    for laneIdx = 1:numAdjLane

        laneID   = adjacentLanes(laneIdx);
        % 3車線: laneID=1,2,3 のとき 中央レーン(2)を d=0 とする
        d_target = (2 - laneID) * laneWidth;

        for k = 1:nDt
            t = dt(k);

            Af = AdjacentForwardVehicle(laneIdx, k);  % 先行車の s(t)
            Ab = AdjacentBehindVehicle(laneIdx, k);   % 後方車の s(t)

            hasForwardVehicle = ~isnan(Af);
            hasBehindVehicle  = ~isnan(Ab);

            % AdjacentForwardGap = (sF0   - sEgo0);
            % AdjacentBehindGap  = (sEgo0 - sB0);
            


            % --- ケース分岐 ---
            if ~hasForwardVehicle && ~hasBehindVehicle
                % ■ 隣接車線に車両がいない場合
                s_term = NaN;

            elseif hasForwardVehicle && ~hasBehindVehicle && (sF0   - sEgo0) > forwardsafetyGap
                % ■ 隣接先行車のみがいる場合
                % s = AdjacentForwardVehicle - carLen*rearAxleRatio - forwardsafetyGap - carLen*(1-rearAxleRatio)
                s_term = Af - carLen * rearAxleRatio - forwardsafetyGap - carLen * (1 - rearAxleRatio);

            elseif ~hasForwardVehicle && hasBehindVehicle && (sEgo0 - sB0) > behindsafetyGap
                % ■ 隣接後方車のみがいる場合
                % s = AdjacentBehindVehicle + carLen*(1-rearAxleRatio) + behindsafetyGap + carLen*rearAxleRatio
                s_term = Ab + carLen * (1 - rearAxleRatio) + behindsafetyGap + carLen * rearAxleRatio;

            else
                % ■ 先行車・後方車どちらもいる場合
                frontBoundary  = Af - carLen * rearAxleRatio - forwardsafetyGap;
                behindBoundary = Ab + carLen * (1 - rearAxleRatio) + behindsafetyGap;

                gap = frontBoundary - behindBoundary;

                % ギャップが carLen 以上のときのみ終端状態生成
                if gap >= carLen && (sF0   - sEgo0) > forwardsafetyGap && (sEgo0 - sB0) > behindsafetyGap
                    % ギャップ中央に自車を置く
                    s_term = frontBoundary - gap / 2;
                else
                    % ギャップ不足 → この dt では終端状態を生成しない
                    continue;
                end
            end

            % --- ここで 1 行分の終端状態を生成 ---
            state = zeros(1,6);
            state(1) = s_term;     % s
            state(2) = vEgo;       % v（目標速度）
            % state(3) = 0;       % 必要なら加速度など
            state(4) = d_target;   % d
            % state(5:6) は未使用のまま

            terminalStates = [terminalStates; state]; %#ok<AGROW>
            times          = [times; t];              %#ok<AGROW>
        end
    end
end
