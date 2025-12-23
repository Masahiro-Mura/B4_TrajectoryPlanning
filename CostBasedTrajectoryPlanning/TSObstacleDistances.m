function dObs = TSObstacleDistances(refPath, EgoGlobalState, AllActorGlobalState, ...
                                           laneWidth, timeHorizons, allDT)
    % === 自車の Frenet & 現在レーン ===
    EgoFrenet0 = global2frenet(refPath, EgoGlobalState);
    sEgo0 = EgoFrenet0(1);      % 自車の現在 s
    vEgo0 = EgoFrenet0(2);      % 自車の現在 v（ここでは一定と仮定）
    curEgoLane0 = PredictLane(EgoFrenet0, laneWidth, 0);  % 現在レーン ID

    % === アクターの Frenet ===
    if isempty(AllActorGlobalState)
        % アクターがいなければ全部 inf のままで OK
        dObs = inf(size(allDT));
        return;
    end

    ActorsFrenet0 = global2frenet(refPath, AllActorGlobalState);
    sActors0 = ActorsFrenet0(:,1);   % 各アクターの現在 s
    vActors0 = ActorsFrenet0(:,2);   % 各アクターの現在 v

    numActors = size(AllActorGlobalState,1);
    nH        = numel(timeHorizons);

    % === 各アクターの将来レーンを予測 (nH x numActors) ===
    predictedActorLanes = zeros(nH, numActors);
    for kAct = 1:numActors
        predictedActorLanes(:,kAct) = PredictLane(ActorsFrenet0(kAct,:), laneWidth, timeHorizons);
    end

    % === 各予測秒数ごとの最短車間距離 dObsPerDt ===
    dObsPerDt = inf(nH,1);  % デフォルトは「先行車なし」

    for k = 1:nH
        t = timeHorizons(k);

        % t 秒後に自車と同一レーンにいるアクターか？
        laneMatch = (predictedActorLanes(k,:)' == curEgoLane0);

        % 現在，自車より前方にいるか？
        leadNow   = sActors0 > sEgo0;

        valid = laneMatch & leadNow;
        if ~any(valid)
            continue;   % この t では先行車なし → inf のまま
        end

        % t 秒後の先行車 s（一定速度で直進と仮定）
        sLeadFuture = sActors0(valid) + vActors0(valid) * t;

        % t 秒後の自車 s（同じく一定速度）
        sEgoFuture  = sEgo0 + vEgo0 * t;

        % 車間距離
        gaps = sLeadFuture - sEgoFuture;

        % もし追い越してしまっていたら 0 にクリップ
        gaps(gaps < 0) = 0;

        % 最も近い先行車との車間距離
        dObsPerDt(k) = min(gaps);
    end

    % === dObsPerDt を allDT に展開 ===
    dObs = nan(size(allDT));

    for k = 1:nH
        t = timeHorizons(k);
        mask = abs(allDT - t) < 1e-6;  % 浮動小数誤差対策
        dObs(mask) = dObsPerDt(k);
    end

    % どの timeHorizons にも属さなかった TS は先行車なしとみなす
    dObs(isnan(dObs)) = inf;
end
