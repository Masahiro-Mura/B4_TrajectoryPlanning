%% === 巡航 + 追従 モード対応版 === %%
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
leadIdxLog = [];
behindIdxLog = [];
timeLog = [];
speedLog = [];
yawLog = [];
simStep = 0;

modeTimeLog  = [];
modeLabelLog = strings(0,1);
modeBinLog   = zeros(0,3);
labelsLog    = {};
modeColLog   = {};

%% --- 逐次軌道計算ループ --- %%
tic
while isRunning
    simStep = simStep + 1;
    [curActorState,futureTrajectory,isRunning] = ...
        exampleHelperRetrieveActorGroundTruth(scenario,futureTrajectory,replanRate,maxHorizon);

    % --- 前方車までの距離を計算 ---
    egoF = global2frenet(refPath, egoState);
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

    % --- モード切替ロジック ---
    % 前方車がいない、または十分に離れていれば Cruise、近ければ Follow
    if ~isnan(minGap) && minGap < forwardsafetyGap + 5
        currentMode = "Follow";
    else
        currentMode = "Cruise";
    end

    %% --- 各モードで目標状態生成 ---
    switch currentMode
        case "Cruise"
            [termStates,times] = exampleHelperBasicCruiseControl(refPath,laneWidth,egoState,speedLimit,timeHorizons);
        case "Follow"
            [termStates,times] = exampleHelperBasicLeadVehicleFollow(refPath,laneWidth,forwardsafetyGap,egoState,curActorState,timeHorizons);
    end

    labels = repmat(currentMode, size(termStates,1), 1);
    allTS = termStates;
    allDT = times;

    % --- コスト評価 ---
    [costTS,latCostTS,timeCostTS,speedCostTS] = ...
        exampleHelperEvaluateTSCost(allTS,allDT,laneWidth,speedLimit,speedWeight,latDevWeight,timeWeight);

    % --- 軌道生成 ---
    egoFrenetState = global2frenet(refPath,egoState);
    [frenetTraj,globalTraj] = connect(connector,egoFrenetState,allTS,allDT);

    isValid = exampleHelperEvaluateTrajectory(globalTraj,maxAcceleration,maxCurvature,minVelocity);

    % --- 衝突チェック ---
    for i = 1:numel(actorPoses)
        actorPoses(i).States = futureTrajectory(i).Trajectory(:,1:3);
    end
    updateObstaclePose(capList,actorID,actorPoses);

    [cost, idx] = sort(costTS);
    optimalTrajectory = [];
    trajectoryEvaluation = nan(numel(isValid),1);
    collisionResult = strings(numel(isValid),1);

    for i = 1:numel(idx)
        if isValid(idx(i))
            egoPoses.States = globalTraj(idx(i)).Trajectory(:,1:3);
            updateEgoPose(capList,egoID,egoPoses);
            isColliding = checkCollision(capList);

            if any(isColliding)
                collisionResult(idx(i)) = "×";
            else
                collisionResult(idx(i)) = "✓";
            end

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

    yawLog(end+1,1) = egoState(3); % rad

    % モード one-hot
    switch currentMode
        case "Cruise"
            onehot = [1 0 0];
        case "Follow"
            onehot = [0 1 0];
    end

    modeTimeLog(end+1,1)  = (simStep-1)*scenario.SampleTime;
    modeLabelLog(end+1,1) = currentMode;
    modeBinLog(end+1,:)   = onehot;

    % --- 描画更新 ---
    lineHandles = exampleHelperVisualizeScene(lineHandles,globalTraj,isValid,trajectoryEvaluation);
    hold on; show(capList,'TimeStep',1:capList.MaxNumSteps,'FastUpdate',1); hold off;

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
            scenarioViewer.Actors(1).AngularVelocity(3) = egoState(4)*egoState(5);
            advance(scenarioViewer);
            tic;
        end
    end
end

%% --- 車間距離のログ --- %%
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

%% --- 速度ログ --- %%
speedFigName = fullfile(pwd, 'speed_plot.png');
if ~isempty(timeLog)
    hFig2 = figure('Name','Ego speed','NumberTitle','off'); clf(hFig2); grid on; hold on;
    hSpeed = plot(timeLog, speedLog, '-', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Speed (m/s)'); title('Ego vehicle speed');
    legend(hSpeed, {'Ego speed'}, 'Location','best');
    set(gca,'FontName','Times New Roman','FontSize',14);
    saveas(hFig2, speedFigName);
end

%% --- ヨー角ログ --- %%
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

%% --- モード遷移ログ --- %%
modeFigName = fullfile(pwd, 'mode_plot.png');
if ~isempty(modeTimeLog)
    hFig4 = figure('Name','Selected Mode','NumberTitle','off'); clf(hFig4); grid on; hold on;
    hCruise = stairs(modeTimeLog, modeBinLog(:,1), 'Color','r', 'LineStyle',':', 'LineWidth',2);
    hFollow = stairs(modeTimeLog, modeBinLog(:,2), 'Color','b', 'LineStyle',':', 'LineWidth',2);
    xlabel('Time (s)'); ylabel('Mode'); title('Mode Transition');
    legend([hCruise, hFollow], {'Cruise','Follow'}, 'Location','best');
    ylim([-0.1 1.1]);
    set(gca,'FontName','Times New Roman','FontSize',14);
    saveas(hFig4, modeFigName);
end
