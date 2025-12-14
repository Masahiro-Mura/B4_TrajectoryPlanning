scenario = SDpaper_scenario1;
% scenario = scenario1;


% デフォルトの車両プロパティ
carLen   = 4.7; % [m] 一般的な乗用車の全長
carWidth = 1.8; % [m] 一般的な乗用車の全幅
rearAxleRatio = .25; % 全長 4.7 m なら、後軸は後端から 1.175 m (4.7 × 0.25) の位置にある、という意味。

% Define road dimensions
laneWidth   = carWidth*2; % [m] 車線幅

plot(scenario);

waypoints = [0 0; 800 0]; % in meters
refPath = referencePathFrenet(waypoints); 

connector = trajectoryGeneratorFrenet(refPath);



capList = dynamicCapsuleList;



egoID = 1; % 自車両には「1」を割り振り，dynamicCapsuleList に登録する車両を一意に識別する番号
[egoID, egoGeom] = egoGeometry(capList,egoID); % dynamicCapsuleList（capList）内で、ID=1 の Ego 車両用の ジオメトリ情報を取り出します。

egoGeom.Geometry.Length = carLen; % 4.7 [m]
egoGeom.Geometry.Radius = carWidth/2; % 1.8/s=0.9 [m]
egoGeom.Geometry.FixedTransform(1,end) = -carLen*rearAxleRatio; % [m], -carLen * 0.25 = -1.175 m を指定すると、カプセルが 後輪軸を基準に配置される

updateEgoGeometry(capList,egoID,egoGeom);

actorID = (1:6); 
actorGeom = repelem(egoGeom,6,1); % egoGeom → 先ほど定義した 車両の形状（長さ=4.7m, 幅=1.8m, 後輪基準）, repelem(...,5,1) → これを 6台分コピーして、6×1 の構造体配列にする
updateObstacleGeometry(capList,actorID,actorGeom) % capList に「ID=1〜6 の車両」を 障害物（Obstacle）として登録。それぞれの車両は actorGeom に基づいて、長さ4.7m・幅1.8m のカプセルで表される


% シミュレータの更新レートを軌道生成器の離散化間隔に合わせるように同期
% scenario.SampleTime → シナリオのシミュレーション刻み時間
% connector.TimeResolution → Frenet軌道生成器が使う時間刻み
% これを揃えることで、軌道生成とシナリオ進行を同じタイミングで動かす
scenario.SampleTime = connector.TimeResolution; % in seconds


% 10 Hz = 0.1秒ごとに軌道を再計算。
replanRate = 10; % Hz

% 現在の状態と計画状態の間隔を定義する。
timeHorizons = 1:5; % [sec] 
maxHorizon = max(timeHorizons); % [sec] 

% コストパラメータの定義
latDevWeight    =  10; % 横方向ずれ（車線中央からの逸脱）
timeWeight      =  -10; % 短い時間を好む
speedWeight     =  1; % 目標速度からの逸脱

% 以下の制約に違反する軌道を排除する。
maxAcceleration = 1; % [m/s^2] 車両が出せる最大加速度を超えたら不適
maxCurvature    = 100; % [1/m], or [rad/m] 最小旋回半径 ≈ 1m
minVelocity     = 0; % [m/sec] 負の速度（バック）や停止を避ける。

% クルーズコントロール動作および軌道コスト評価時に使用される目標速度設定値。
speedLimit = 18; % [m/sec] 巡航時の目標速度。コスト評価の基準値

% プランナーが追従動作において目標とする最小距離。
forwardsafetyGap = 20; % [m] 前車との最低距離。これを下回る軌道は不適切として棄却
behindsafetyGap = 12;




[scenarioViewer,futureTrajectory,actorID,actorPoses,egoID,egoPoses,stepPerUpdate,egoState,isRunning,lineHandles] = exampleHelperInitializeSimulator(scenario,capList,refPath,laneWidth,replanRate,carLen);


% --- prepare logging for longitudinal gap to the nearest forward vehicle ---
numActors = numel(actorID);
gapForwardLog = [];        % rows: timesteps, single column: nearest lead gap (m)
gapBehindLog = [];  % rows: timesteps, single column: nearest behind gap (m)
leadIdxLog = [];    % store which actor was the nearest lead (optional)
behindIdxLog = [];  % store which actor was the nearest behind (optional)
timeLog = [];
speedLog = [];      % rows: timesteps, single column: ego speed (m/s)
simStep = 0;

% --- モードログ用初期化ボックス ---
modeTimeLog  = [];              % Nx1 時刻[s]
modeLabelLog = strings(0,1);    % Nx1 選定モード（"Cruise","Follow","LaneChange",または "" ）
modeBinLog   = zeros(0,3);      % Nx3 [Cruise, Follow, LaneChange] を 0/1 で one-hot
labelsLog    = {};              % 各ステップの候補ラベル配列（可変長なので cell）
modeColLog   = {};              % 各ステップの ✓/"" ベクトル（可変長なので cell）
yawLog = [];  % [rad] 自車両のヨー角ログ


%% ---逐次軌道計算ループ--- %%
tic 
while isRunning
    simStep = simStep + 1;

    % --- ① 現在および将来のアクター状態を取得 ---
    [curActorState,futureTrajectory,isRunning] = ...
        exampleHelperRetrieveActorGroundTruth(scenario,futureTrajectory,replanRate,maxHorizon);

    % --- ② 現在のエゴ車両と前方車の距離を算出 ---
    egoF = global2frenet(refPath, egoState);
    actorFs = global2frenet(refPath, curActorState);
    curEgoLane = exampleHelperPredictLane(egoF, laneWidth, 0);

    % 同一レーン内で ego より前にいる車両を探す
    gaps = nan(1,numActors);
    for a = 1:numActors
        aLane = exampleHelperPredictLane(actorFs(a,:), laneWidth, 0);
        if abs(aLane - curEgoLane) < 0.5 && actorFs(a,1) > egoF(1)
            gaps(a) = actorFs(a,1) - egoF(1);
        end
    end
    minGap = min(gaps,[],'omitnan');

    % --- ③ Cruise モード起動条件 ---
    cruiseEnabled = isempty(minGap) || minGap >= forwardsafetyGap;

    % --- ④ 動作モードごとの終端状態を生成 ---
    termStatesCC = [];
    timesCC = [];
    if cruiseEnabled
        % 距離十分な場合のみCruiseを生成
        [termStatesCC,timesCC] = exampleHelperBasicCruiseControl(...
            refPath,laneWidth,egoState,speedLimit,timeHorizons);
    end

    [termStatesF,timesF] = exampleHelperBasicLeadVehicleFollow(...
        refPath,laneWidth,forwardsafetyGap,egoState,curActorState,timeHorizons);

    [termStatesLC,timesLC] = exampleHelperBasicLaneChange(...
        refPath,laneWidth,egoState,timeHorizons);

    % --- ⑤ 終端状態の統合 ---
    allTS = [termStatesCC; termStatesLC; termStatesF];
    allDT = [timesCC; timesLC; timesF];
    labels = [...
        repmat("Cruise",size(termStatesCC,1),1);
        repmat("LaneChange",size(termStatesLC,1),1);
        repmat("Follow",size(termStatesF,1),1)];

    % --- ⑥ コスト計算 ---
    [costTS,latCostTS,timeCostTS,speedCostTS] = exampleHelperEvaluateTSCost(...
        allTS,allDT,laneWidth,speedLimit,speedWeight,latDevWeight,timeWeight);

    % --- ⑦ 軌道生成と有効性チェック ---
    egoFrenetState = global2frenet(refPath,egoState);
    [frenetTraj,globalTraj] = connect(connector,egoFrenetState,allTS,allDT);
    isValid = exampleHelperEvaluateTrajectory(globalTraj,maxAcceleration,maxCurvature,minVelocity);

    % --- ⑧ 衝突・距離違反チェック ---
    [cost,idx] = sort(costTS);
    optimalTrajectory = [];
    trajectoryEvaluation = nan(numel(isValid),1);

    for i = 1:numel(idx)
        thisIdx = idx(i);

        if isValid(thisIdx)
            % --- 衝突チェック ---
            egoPoses.States = globalTraj(thisIdx).Trajectory(:,1:3);
            updateEgoPose(capList,egoID,egoPoses);
            isColliding = checkCollision(capList);

            % --- Cruise時のみ再度安全距離チェック ---
            violatesGap = false;
            if labels(thisIdx) == "Cruise"
                egoTrajF = global2frenet(refPath,globalTraj(thisIdx).Trajectory);
                for a = 1:numActors
                    actorF = global2frenet(refPath,curActorState(a,:));
                    aLane = exampleHelperPredictLane(actorF,laneWidth,0);
                    if abs(aLane - curEgoLane) < 0.5 && actorF(1) > egoTrajF(end,1)
                        gap = actorF(1) - egoTrajF(end,1);
                        if gap < forwardsafetyGap
                            violatesGap = true;
                            break;
                        end
                    end
                end
            end

            % --- 最終採択条件 ---
            if all(~isColliding) && ~violatesGap
                trajectoryEvaluation(thisIdx) = 1;
                optimalTrajectory = globalTraj(thisIdx).Trajectory;
                break;
            else
                trajectoryEvaluation(thisIdx) = 0;
            end
        end
    end

    %% === 衝突なし軌道のコスト表示 === %%
    numCandidates = numel(costTS);
    isCollisionFree = false(numCandidates,1);
    for i = 1:numCandidates
        egoPoses.States = globalTraj(i).Trajectory(:,1:3);
        updateEgoPose(capList,egoID,egoPoses);
        isColliding = checkCollision(capList);
        isCollisionFree(i) = all(~isColliding);
    end

    if ~isempty(timeLog)
        currentSimTime = timeLog(end);
    else
        currentSimTime = (simStep-1)*scenario.SampleTime;
    end
    if ~isempty(gapForwardLog)
        currentGap = gapForwardLog(end);
    else
        currentGap = nan;
    end

    Tdisp = table;
    Tdisp.SimTime_s   = repmat(currentSimTime,numCandidates,1);
    Tdisp.ForwardGap_m = repmat(currentGap,numCandidates,1);
    Tdisp.Mode        = labels;
    Tdisp.Horizon_s   = allDT;
    Tdisp.Collision   = ~isCollisionFree;
    Tdisp.Cost        = costTS;
    Tvalid = Tdisp(~Tdisp.Collision,:);
    Tvalid = sortrows(Tvalid,'Cost','ascend');

    disp('==== 衝突のない軌道の中でコストの小さい順に並べた結果 ====');
    disp(Tvalid);

    if ~isempty(Tvalid)
        bestRow = Tvalid(1,:);
        fprintf('\n最小コストモード: %s, Horizon = %.1f s, Cost = %.2f, Gap = %.2f m, Time = %.1f s\n',...
            bestRow.Mode, bestRow.Horizon_s, bestRow.Cost, bestRow.ForwardGap_m, bestRow.SimTime_s);
    else
        disp('衝突のない軌道は存在しません。');
    end

  

    % === 残りの描画・更新処理 ===
    lineHandles = exampleHelperVisualizeScene(lineHandles,globalTraj,isValid,trajectoryEvaluation);
    hold on;
    show(capList,'TimeStep',1:capList.MaxNumSteps,'FastUpdate',1);
    hold off;

    if isempty(optimalTrajectory)
        error('No valid trajectory has been found.');
    else
        for i = (2+(0:(stepPerUpdate-1)))
            dt = toc;
            if scenario.SampleTime-dt > 0
                pause(scenario.SampleTime-dt);
            end
            egoState = optimalTrajectory(i,:);
            scenarioViewer.Actors(1).Position(1:2) = egoState(1:2);
            scenarioViewer.Actors(1).Velocity(1:2) = [cos(egoState(3)) sin(egoState(3))]*egoState(5);
            scenarioViewer.Actors(1).Yaw = egoState(3)*180/pi;
            yawLog(end+1,1) = egoState(3)*180/pi;
            scenarioViewer.Actors(1).AngularVelocity(3) = egoState(4)*egoState(5);
            hold on;
            show(capList,'TimeStep',i:capList.MaxNumSteps,'FastUpdate',1);
            hold off;
            advance(scenarioViewer);
            tic;
        end
    end
end

