% 終端状態における各予測秒数ごとの後方車間距離を算出
function dObs = TSBehindObstacleDistances(refPath, EgoGlobalState, AllActorGlobalState, ...
                                           laneWidth, timeHorizons, allDT)
    % Ego から見た「後方車両」との車間距離 [m] を返す
    % 返り値 dObs は allDT と同じサイズで、
    % 各要素はその TS の予測時間に対する「最も近い後方車両との s 方向距離」

    % === 自車の Frenet & 現在レーン ===
    EgoFrenet0   = global2frenet(refPath, EgoGlobalState);
    sEgo0        = EgoFrenet0(1);      % 自車の現在 s
    vEgo0        = EgoFrenet0(2);      % 自車の現在 v（ここでは一定と仮定）
    curEgoLane0  = PredictLane(EgoFrenet0, laneWidth, 0);  % 現在レーン ID

    % === アクターの Frenet ===
    if isempty(AllActorGlobalState)
        % アクターがいなければ全部 inf のままで OK
        dObs = inf(size(allDT));
        return;
    end

    ActorsFrenet0 = global2frenet(refPath, AllActorGlobalState);
    sActors0      = ActorsFrenet0(:,1);   % 各アクターの現在 s
    vActors0      = ActorsFrenet0(:,2);   % 各アクターの現在 v

    numActors = size(AllActorGlobalState,1);
    nH        = numel(timeHorizons);

    % === 各アクターの将来レーンを予測 (nH x numActors) ===
    predictedActorLanes = zeros(nH, numActors);
    for kAct = 1:numActors
        predictedActorLanes(:,kAct) = PredictLane(ActorsFrenet0(kAct,:), laneWidth, timeHorizons);
    end

    % === 各予測秒数ごとの最短車間距離 dObsPerDt ===
    dObsPerDt = inf(nH,1);  % デフォルトは「後方車なし」

    for k = 1:nH
        t = timeHorizons(k);

        % t 秒後に自車と同一レーンにいるアクターか？
        laneMatch = (predictedActorLanes(k,:)' == curEgoLane0);

        % 現在，自車より後方にいるか？（※ここが lead 版と対称）
        behindNow = sActors0 < sEgo0;

        valid = laneMatch & behindNow;
        if ~any(valid)
            continue;   % この t では後方車なし → inf のまま
        end

        % t 秒後の後方車 s（一定速度で直進と仮定）
        sBehindFuture = sActors0(valid) + vActors0(valid) * t;

        % t 秒後の自車 s
        sEgoFuture = sEgo0 + vEgo0 * t;

        % 車間距離（ego から見て後方なので sEgo - sBehind）
        gaps = sEgoFuture - sBehindFuture;

        % もし追い越されてしまっていたら 0 にクリップ
        % （その時刻ではもはや「後方車」ではなくなる想定）
        gaps(gaps < 0) = 0;

        % 最も近い後方車との車間距離
        dObsPerDt(k) = min(gaps);
    end

    % === dObsPerDt を allDT に展開 ===
    dObs = nan(size(allDT));

    for k = 1:nH
        t = timeHorizons(k);
        mask = abs(allDT - t) < 1e-6;  % 浮動小数誤差対策
        dObs(mask) = dObsPerDt(k);
    end

    % どの timeHorizons にも属さなかった TS は後方車なしとみなす
    dObs(isnan(dObs)) = inf;
end
