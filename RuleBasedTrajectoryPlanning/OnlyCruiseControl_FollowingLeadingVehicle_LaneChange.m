%% === 巡航 + 追従 + 車線変更 モード対応版（NaN防止 + 滑らか軌道） === %%
scenario = scenario1;

% --- 車両プロパティ ---
carLen   = 4.7;
carWidth = 1.8;
rearAxleRatio = .25;
laneWidth   = carWidth*2;
plot(scenario);

connector = trajectoryGeneratorFrenet(refPath);
capList = dynamicCapsuleList;

egoID = 1;
[egoID, egoGeom] = egoGeometry(capList,egoID);
egoGeom.Geometry.Length = carLen;
egoGeom.Geometry.Radius = carWidth/2;
egoGeom.Geometry.FixedTransform(1,end) = -carLen*rearAxleRatio;
updateEgoGeometry(capList,egoID,egoGeom);
show(capList)

actorID = (1:6)';
actorGeom = repelem(egoGeom,6,1);
updateObstacleGeometry(capList,actorID,actorGeom)

scenario.SampleTime = connector.TimeResolution;
replanRate = 10;

timeHorizons = 1:0.5:3;
maxHorizon = max(timeHorizons);

latDevWeight = 5;
timeWeight   = -1;
speedWeight  = 1;

maxAcceleration = 10;
maxCurvature    = 100;
minVelocity     = 0;

speedLimit = 17;
forwardsafetyGap = 16;
behindsafetyGap  = 12;

[scenarioViewer,futureTrajectory,actorID,actorPoses,egoID,egoPoses,stepPerUpdate,egoState,isRunning,lineHandles] = ...
    exampleHelperInitializeSimulator(scenario,capList,refPath,laneWidth,replanRate,carLen);

numActors = numel(actorID);
gapForwardLog = [];
gapBehindLog = [];
timeLog = [];
speedLog = [];
yawLog = [];
simStep = 0;

modeTimeLog  = [];
modeLabelLog = strings(0,1);
modeBinLog   = zeros(0,3);

%% --- 逐次軌道計算ループ --- %%
tic
while isRunning
    simStep = simStep + 1;
    [curActorState,futureTrajectory,isRunning] = ...
        exampleHelperRetrieveActorGroundTruth(scenario,futureTrajectory,replanRate,maxHorizon);

    % --- 前方車までの距離を計算 ---
    egoF = global2frenet(refPath, egoState);
    if any(isnan(egoF))
        warning('Ego Frenet state is invalid (NaN). Skipping step.');
        continue;
    end

    actorFs = global2frenet(refPath, curActorState);
    curEgoLane = exampleHelperPredictLane(egoF, laneWidth, 0);
    gaps = nan(1,numActors);
    for a = 1:numActors
        aLane = exampleHelperPredictLane(actorFs(a,:), laneWidth, 0);
        if aLane == curEgoLane && actorFs(a,1) > egoF(1)
            gaps(a) = actorFs(a,1) - egoF(1);
        end
    end
    validIdx = find(~isnan(gaps));
    if ~isempty(validIdx)
        [minGap, relIdx] = min(gaps(validIdx));
        actorIdx = validIdx(relIdx);
    else
        minGap = nan;
        actorIdx = nan;
    end

    %% --- モード切替ロジック ---
    if isnan(minGap)
        currentMode = "Cruise";
    elseif minGap > forwardsafetyGap + 5
        currentMode = "Cruise";
    elseif minGap <= forwardsafetyGap && minGap > forwardsafetyGap/2
        currentMode = "Follow";
    else
        currentMode = "LaneChange";
    end

    %% --- 各モードで目標状態生成 ---
    switch currentMode
        case "Cruise"
            [termStates,times] = exampleHelperBasicCruiseControl(refPath,laneWidth,egoState,speedLimit,timeHorizons);

        case "Follow"
            [termStates,times] = exampleHelperBasicLeadVehicleFollow(refPath,laneWidth,forwardsafetyGap,egoState,curActorState,timeHorizons);
            if minGap < forwardsafetyGap
                termStates(:,5) = max(0, termStates(:,5) - 0.5*(forwardsafetyGap - minGap)); % 減速補正
            end

        case "LaneChange"
            % === 滑らかな5次多項式的遷移（NaN防止付き） ===
            egoF = global2frenet(refPath, egoState);
            if any(isnan(egoF))
                warning('Skipping lane change due to invalid ego Frenet state.');
                [termStates,times] = exampleHelperBasicCruiseControl(refPath,laneWidth,egoState,speedLimit,timeHorizons);
                currentMode = "Cruise";
                break;
            end

            curLaneCenter = exampleHelperPredictLane(egoF, laneWidth, 0);

            % レーン変更方向決定（左優先）
            leftLaneFree  = true;
            rightLaneFree = true;
            if leftLaneFree
                targetLaneCenter = (curLaneCenter-1)*laneWidth; % 左へ
            elseif rightLaneFree
                targetLaneCenter = (curLaneCenter+1)*laneWidth; % 右へ
            else
                [termStates,times] = exampleHelperBasicLeadVehicleFollow(refPath,laneWidth,forwardsafetyGap,egoState,curActorState,timeHorizons);
                currentMode = "Follow";
                break;
            end

            % 複数時間地平ごとに終端状態生成
            numH = numel(timeHorizons);
            termStates = zeros(numH,6);
            times = zeros(numH,1);
            for i = 1:numH
                tH = timeHorizons(i);
                S  = egoF(1) + egoF(5)*tH;
                dS = max(egoF(5), 0.1);
                ddS= 0;
                L  = targetLaneCenter; dL = 0; ddL = 0;
                termStates(i,:) = [S dS ddS L dL ddL];
                times(i) = tH;
            end
    end

    % --- NaN防止 ---
    if any(isnan(termStates(:)))
        warning('NaN detected in termStates, skipping this iteration.');
        continue;
    end

    labels = repmat(currentMode, size(termStates,1), 1);
    allTS = termStates;
    allDT = times;

    % --- コスト評価 ---
    [costTS,latCostTS,timeCostTS,speedCostTS] = ...
        exampleHelperEvaluateTSCost(allTS,allDT,laneWidth,speedLimit,speedWeight,latDevWeight,timeWeight);

    % --- 軌道生成 ---
    egoFrenetState = global2frenet(refPath,egoState);
    if any(isnan(egoFrenetState))
        warning('Skipping invalid ego Frenet state.');
        continue;
    end

    [frenetTraj,globalTraj] = connect(connector,egoFrenetState,allTS,allDT);
    if isempty(globalTraj)
        warning('No valid trajectories generated.');
        continue;
    end

    % --- 衝突チェック ---
    for i = 1:numel(actorPoses)
        actorPoses(i).States = futureTrajectory(i).Trajectory(:,1:3);
    end
    updateObstaclePose(capList,actorID,actorPoses);

    [cost, idx] = sort(costTS);
    optimalTrajectory = [];
    trajectoryEvaluation = nan(numel(isValid),1);
    isValid = exampleHelperEvaluateTrajectory(globalTraj,maxAcceleration,maxCurvature,minVelocity);

    % --- 衝突・有効軌道選定 ---
    for i = 1:numel(idx)
        if isValid(idx(i))
            egoPoses.States = globalTraj(idx(i)).Trajectory(:,1:3);
            updateEgoPose(capList,egoID,egoPoses);
            isColliding = checkCollision(capList);
            if all(~isColliding)
                trajectoryEvaluation(idx(i)) = 1;
                optimalTrajectory = globalTraj(idx(i)).Trajectory;
                break;
            else
                trajectoryEvaluation(idx(i)) = 0;
            end
        end
    end

    %% --- ログ更新 ---
    gapForwardLog(end+1,1) = minGap;
    timeLog(end+1,1) = (simStep-1)*scenario.SampleTime;
    if numel(egoState) >= 5
        speedLog(end+1,1) = egoState(5);
    else
        speedLog(end+1,1) = nan;
    end
    yawLog(end+1,1) = egoState(3);

    % --- モード one-hot ---
    switch currentMode
        case "Cruise"
            onehot = [1 0 0];
        case "Follow"
            onehot = [0 1 0];
        case "LaneChange"
            onehot = [0 0 1];
    end
    modeTimeLog(end+1,1)  = (simStep-1)*scenario.SampleTime;
    modeLabelLog(end+1,1) = currentMode;
    modeBinLog(end+1,:)   = onehot;

    % --- 軌道候補の可視化 ---
    lineHandles = exampleHelperVisualizeScene(lineHandles,globalTraj,isValid,trajectoryEvaluation);
    hold on; show(capList,'TimeStep',1:capList.MaxNumSteps,'FastUpdate',1); hold off;

    if isempty(optimalTrajectory)
        warning('No valid trajectory found this cycle.');
        continue;
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
            scenarioViewer.Actors(1).AngularVelocity(3) = egoState(4)*egoState(5);
            advance(scenarioViewer);
            tic;
        end
    end
end

%% --- 車間距離ログ ---
figName = fullfile(pwd, 'gap_plot.png');
if ~isempty(timeLog)
    hFig = figure('Name','Ego to lead gaps','NumberTitle','off'); clf(hFig); hold on; grid on;
    hPlotLead = plot(timeLog, gapForwardLog, '-', 'Color', 'b', 'LineWidth', 3);
    hYlineLead = yline(forwardsafetyGap, 'b--', 'Label', '');
    xlabel('Time (s)'); ylabel('Gap (m)');
    title('Forward vehicle gap');
    legend([hPlotLead hYlineLead], {'Forward gap','SafetyGap'}, 'Location','northeast');
    set(gca,'FontName','Times New Roman','FontSize',14);
    saveas(hFig, figName);
end

%% --- 速度ログ ---
speedFigName = fullfile(pwd, 'speed_plot.png');
if ~isempty(timeLog)
    hFig2 = figure('Name','Ego speed','NumberTitle','off'); clf(hFig2); grid on; hold on;
    hSpeed = plot(timeLog, speedLog, '-', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Speed (m/s)'); title('Ego vehicle speed');
    legend(hSpeed, {'Ego speed'}, 'Location','best');
    set(gca,'FontName','Times New Roman','FontSize',14);
    saveas(hFig2, speedFigName);
end

%% --- ヨー角ログ ---
yawFigName = fullfile(pwd, 'yaw_plot.png');
if ~isempty(timeLog) && ~isempty(yawLog)
    yawLog = unwrap(yawLog);
    hFig3 = figure('Name','Ego yaw angle','NumberTitle','off'); clf(hFig3); grid on; hold on;
    hYaw = plot(timeLog, yawLog, '-', 'Color', [0.3010 0.7450 0.9330], 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Yaw angle (rad)'); title('Ego vehicle yaw angle');
    legend(hYaw, {'Yaw angle'}, 'Location','best');
    set(gca,'FontName','Times New Roman','FontSize',14);
    saveas(hFig3, yawFigName);
end

%% --- モード遷移ログ ---
modeFigName = fullfile(pwd, 'mode_plot.png');
if ~isempty(modeTimeLog)
    hFig4 = figure('Name','Selected Mode','NumberTitle','off'); clf(hFig4); grid on; hold on;
    hCruise     = stairs(modeTimeLog, modeBinLog(:,1), 'r-', 'LineWidth', 2);
    hFollow     = stairs(modeTimeLog, modeBinLog(:,2), 'b-', 'LineWidth', 2);
    hLaneChange = stairs(modeTimeLog, modeBinLog(:,3), 'm-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Mode'); title('Mode Transition');
    legend([hCruise, hFollow, hLaneChange], {'Cruise','Follow','LaneChange'}, 'Location','best');
    ylim([-0.1 1.1]);
    set(gca,'FontName','Times New Roman','FontSize',14);
    saveas(hFig4, modeFigName);
end
