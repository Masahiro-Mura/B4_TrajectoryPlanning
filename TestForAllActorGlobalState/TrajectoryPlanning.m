%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% ===== メインコード ===== %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc; clear; close all;
addpath('..\Scenario\m_file\');

scenario = scenario_20260108;

%% パラメータの読み込み
P = parameters();

carLen        = P.carLen;
carWidth      = P.carWidth;
rearAxleRatio = P.rearAxleRatio;
laneWidth     = P.laneWidth;

plot(scenario);
waypoints = [0 0; P.roadLen 0]; 
refPath = referencePathFrenet(waypoints); 

connector = trajectoryGeneratorFrenet(refPath);

capList = dynamicCapsuleList;

egoID = 1; 
[egoID, egoGeom] = egoGeometry(capList,egoID); 

egoGeom.Geometry.Length = carLen; 
egoGeom.Geometry.Radius = carWidth/2; 
egoGeom.Geometry.FixedTransform(1,end) = -carLen*rearAxleRatio; 

updateEgoGeometry(capList,egoID,egoGeom);

actorID = (1:P.Number_of_Actor); 
actorGeom = repelem(egoGeom,P.Number_of_Actor,1); 
updateObstacleGeometry(capList,actorID,actorGeom) 

scenario.SampleTime = connector.TimeResolution; 

% パラメータ定義
replanRate          = P.replanRate;
timeHorizons        = P.timeHorizons;
maxHorizon          = P.maxHorizon;

% コスト関数のパラメータ
LatWeight           = P.LatWeight;
TimeWeight          = P.TimeWeight;
SpeedWeight         = P.SpeedWeight;
ForwardSafetyWeight = P.ForwardSafetyWeight;
BehindSafetyWeight  = P.BehindSafetyWeight;

% 距離計算
c1                  = P.c1;  

% 制約
maxAcceleration     = P.maxAcceleration;
maxCurvature        = P.maxCurvature;
minVelocity         = P.minVelocity;

SpeedLimit          = P.SpeedLimit;       
forwardsafetyGap    = P.forwardsafetyGap;  
behindsafetyGap     = P.behindsafetyGap;

simStep = P.simStep;

%% 初期化
[scenarioViewer,futureTrajectory,actorID,actorPoses,egoID,egoPoses,stepPerUpdate,EgoGlobalState,isRunning,lineHandles] = ...
    InitializeSimulator(scenario,capList,refPath,replanRate);


numActors = numel(actorID);
gapForwardLog = []; 
gapBehindLog  = []; 
leadIdxLog    = [];    
behindIdxLog  = [];  
timeLog       = [];       
speedLog      = []; 

% 周辺車両のログボックス
actorSpeedLog = [];
actorAccLog = [];
actorJerkLog  = [];

% 時間
TimeLog   = [];
LabelLog  = strings(0,1);  
modeBinLog    = zeros(0,3);      
yawLog        = [];  
latAccLog     = [];   

% 横偏差コストログ
CruiseLatCostLog      = [];
FollowingLatCostLog   = [];
ChangeLatCostLog      = [];

% 速度コストログ
CruiseSpeedCostLog    = [];
FollowingSpeedCostLog = [];
ChangeSpeedCostLog    = [];

% 時間コストログ
CruiseTimeCostLog     = [];
FollowingTimeCostLog  = [];
ChangeTimeCostLog     = [];

% 全コストログ
CruiseTotalCostLog    = [];
FollowingTotalCostLog = [];
ChangeTotalCostLog    = [];

% s座標，l座標ログ
allS = []; 
allL = [];

%% ===== 逐次軌道計画 ===== %%
tic 

videoFile = P.videoFile;
videoWriter = VideoWriter(videoFile, 'MPEG-4');
videoWriter.FrameRate = P.videoFPS;
open(videoWriter);

%% 周辺車両ごとの目標速度 [m/s]
actorVTargets = [P.Vel_actor1, P.Vel_actor2, P.Vel_actor3, P.Vel_actor4, P.Vel_actor5, P.Vel_actor6]; 

% ステップ2で作成した変数も準備
actorJerks     = zeros(1, numActors); % 現在のジャークを保持
actorAccels    = zeros(1, numActors); % 各Actorの前回の加速度を保持
actorSpeeds    = zeros(1, numActors);
actorPositions = zeros(numActors, 3);

for i = 1:numActors
    actorSpeeds(i) = norm(scenario.Actors(i+1).Velocity);
    actorPositions(i, :) = scenario.Actors(i+1).Position;
end


%% while文 開始
while isRunning
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    simStep = simStep + 1;
    
    %% IDMを用いた周辺車両の状態更新
    for i = 1:numActors
        % 1. 各データの準備
        gap_raw = EgoGlobalState(1) - actorPositions(i, 1);
        gap = gap_raw - carLen; % バンパー間距離
        v_target = actorVTargets(i); 
        dt = scenario.SampleTime;

        % 2. 自車両（Leader）の実際の加速度を取得
        a_leader_current = EgoGlobalState(6); 

        % 3. 条件分岐とIDM計算
        if abs(actorPositions(i, 2) - EgoGlobalState(2)) < 3.4 && gap_raw > 0 % ここの3.4についてはどの程度横移動したら後方車両に影響があるか知りたい
            v_leader = EgoGlobalState(5); 
            [acc, jerk] = IDM_improve(actorSpeeds(i), v_leader, gap, v_target, a_leader_current, actorAccels(i), dt);
        else
            % 自車が関係ない場合
            a_max = 0.73; % 論文Table 1のパラメータ
            delta = 4;
            acc = a_max * (1 - (actorSpeeds(i) / v_target)^delta);
            jerk = (acc - actorAccels(i)) / dt;
        end

        % 4. 加速度の更新（次時刻のジャーク計算用）
        actorAccels(i) = acc;
        actorJerks(i)  = jerk;


        % 5. 速度と位置の更新
        actorSpeeds(i) = actorSpeeds(i) + acc * dt;
        actorPositions(i, 1) = actorPositions(i, 1) + actorSpeeds(i) * dt;

        scenario.Actors(i+1).Position = actorPositions(i, :);
        scenario.Actors(i+1).Velocity = [actorSpeeds(i) 0 0];
    end
    
    %% 周辺車両の真値の取得
    
    %%%%%%%% 真値 %%%%%%%%
    % [AllActorGlobalState,futureTrajectory, isRunning] = RetrieveActorGroundTruth(...
    %     scenario,futureTrajectory,replanRate,maxHorizon);
    
    
    %% ===== 1. IDMで更新した周辺車両状態を AllActorGlobalState に反映 =====
    for i = 1:numActors
        idx = i;

        AllActorGlobalState(idx,1) = actorPositions(i,1);   % x
        AllActorGlobalState(idx,2) = actorPositions(i,2);   % y
        AllActorGlobalState(idx,3) = 0;                     % yaw
        AllActorGlobalState(idx,4) = 0;                     % curvature
        AllActorGlobalState(idx,5) = actorSpeeds(i);        % speed
        AllActorGlobalState(idx,6) = actorAccels(i);        % acceleration
    end
    
    %% 2. 確定した AllActorGlobalState を使って予測軌道（等速直線）を生成
    [AllActorGlobalState, futureTrajectory, isRunning] = RetrieveActorGroundTruth(...
        scenario, futureTrajectory, maxHorizon, AllActorGlobalState);

    %% 3. シナリオのアクター位置も同期
    for i = 1:numActors
        scenario.Actors(i+1).Position(1:2) = AllActorGlobalState(i,1:2);
        scenario.Actors(i+1).Velocity(1:2) = [cos(AllActorGlobalState(i,3)) sin(AllActorGlobalState(i,3))] * AllActorGlobalState(i,5);
        scenario.Actors(i+1).Yaw           = AllActorGlobalState(i,3)*180/pi;
        scenario.Actors(i+1).AngularVelocity(3) = AllActorGlobalState(i,4)*AllActorGlobalState(i,5);
    end
    
    %%%%%%%% 真値 %%%%%%%%
    disp('========== 全アクターの真値状態 ===========');
    disp(AllActorGlobalState);
    disp('========== 自車両の真値状態 ===========');
    disp(EgoGlobalState);
    
    %% ===== 軌道計算 ===== %%
    % 巡航
    [termStatesCC,timesCC,leadGapsCC, behindGapsCC] = TSCruiseControl(...
        refPath,laneWidth,EgoGlobalState,SpeedLimit,timeHorizons, AllActorGlobalState);
    
    
    % 先行車追従
    [termStatesF,timesF,leadGapsF, behindGapsF] = TSLeadVehicleFollow(...
        refPath,laneWidth,forwardsafetyGap,EgoGlobalState,AllActorGlobalState,carLen, rearAxleRatio, timeHorizons);
    
    
    % 車線変更
    [termStatesLC, timesLC, leadGapsLC, behindGapsLC] = TSLaneChange( ...
        refPath, laneWidth, forwardsafetyGap, behindsafetyGap, ...
        EgoGlobalState, AllActorGlobalState, carLen, rearAxleRatio, SpeedLimit, timeHorizons);
    
   
    Labels = [repmat("Cruise",     size(termStatesCC,1), 1);
              repmat("Follow",     size(termStatesF, 1), 1);
              repmat("LaneChange", size(termStatesLC,1), 1)];
    AllTime = [timesCC; timesF; timesLC];
    AllForwardGaps = [leadGapsCC; leadGapsF; leadGapsLC];
    AllBehindGaps  = [behindGapsCC; behindGapsF; behindGapsLC];
    
    allTS = [termStatesCC; termStatesF; termStatesLC];
    numTS = [numel(timesCC); numel(timesF); numel(timesLC)];
    
    EgoFrenetState = global2frenet(refPath,EgoGlobalState);
    [frenetTraj,globalTraj] = connect(connector,EgoFrenetState,allTS,AllTime);
    
    [gapForwardLog, gapBehindLog, timeLog, speedLog] ...
         = ComputeGaps(EgoFrenetState, carLen, rearAxleRatio, refPath, AllActorGlobalState, ...
         laneWidth, numActors, gapForwardLog, gapBehindLog, leadIdxLog, behindIdxLog, timeLog, simStep, scenario, EgoGlobalState, speedLog);
    disp('後続車両との車間距離');
    disp(gapBehindLog(end));
    
    %% ===== 軌道選定 ===== %%
    % min-max正規化
    [costTS, latCostTS, timeCostTS, speedCostTS, ForwardSafeCostTS, BehindSafeCostTS] = ...
    EvaluateTSCostNormalization1(allTS, AllTime, laneWidth, SpeedLimit, ...
                                SpeedWeight, LatWeight, TimeWeight, ForwardSafetyWeight, BehindSafetyWeight, c1,...
                                AllForwardGaps, AllBehindGaps, forwardsafetyGap, behindsafetyGap, EgoFrenetState, carLen, rearAxleRatio);
    
    % 制約を違反している軌道の排除
    isValid = EvaluateTrajectory(globalTraj,maxAcceleration,maxCurvature,minVelocity); 
    
    
    %% ===== 衝突判定 ===== %%
    for i = 1:numel(actorPoses)
        actorPoses(i).States = futureTrajectory(i).Trajectory(:,1:3);
    end
    updateObstaclePose(capList,actorID,actorPoses);
    
    
    % 評価順序の決定
    [cost, idx] = sort(costTS);
    optimalTrajectory = [];
    
    trajectoryEvaluation = nan(numel(isValid),1); 
    
    
    % 最小コストの計算
    for i = 1:numel(idx)
        if isValid(idx(i))
            egoPoses.States = globalTraj(idx(i)).Trajectory(:,1:3);
            updateEgoPose(capList,egoID,egoPoses);
    
            % 衝突チェック
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
    
    disp("経過時間 " + num2str(simStep/10-0.1));
    ModeTable = table (Labels, AllTime, costTS, latCostTS, timeCostTS, speedCostTS, ForwardSafeCostTS, BehindSafeCostTS, isValid, ...
        'VariableNames', {'Mode','PredictedTime','TotalCost','LateralCost','TimeCost','SpeedCost','ForwardGapCost','BeindGapCost','ConstraintViolation'});
    % disp(a);  
    
    %% 周辺車両の速度, 加速度，ジャークを記録 (IDM更新後に挿入)
    actorSpeedLog(end+1, :) = actorSpeeds;    %#ok<SAGROW>
    actorAccLog(end+1, :)   = actorAccels;            %#ok<SAGROW>
    actorJerkLog(end+1, :)  = actorJerks;           %#ok<SAGROW>

    %% ===== 選定モードの決定 ===== %%
    selectedIdx = find(trajectoryEvaluation==1, 1);  % First valid candidate trajectory
    
    selLabel = "";        
    onehot   = [0 0 0];      % [Cruise, Follow, LaneChange]
    
    if ~isempty(selectedIdx)
        selLabel = Labels(selectedIdx);  % "Cruise" / "Follow" / "LaneChange"
        switch selLabel
            case "Cruise"
                onehot = [1 0 0];
            case "Follow"
                onehot = [0 2 0];
            case "LaneChange"
                onehot = [0 0 3];
            otherwise
                onehot = [0 0 0];
        end
    end
    
    TimeLog(end+1,1)  = (simStep-1)*scenario.SampleTime; %#ok<SAGROW>
    LabelLog(end+1,1) = selLabel;                        %#ok<SAGROW>
    modeBinLog(end+1,:)   = onehot;                          %#ok<SAGROW>
    

    %% ===== 軌道の可視化 ===== %%
    lineHandles = VisualizeScene(lineHandles,globalTraj,isValid,trajectoryEvaluation);

    hold on;
    show(capList,'TimeStep',1:capList.MaxNumSteps,'FastUpdate',1);
    hold off;
    
    if isempty(optimalTrajectory)
            error('No valid trajectory has been found.');
    else
        % 元のループ方法
        % for i  = (2+(0:(stepPerUpdate-1)))
        %     dt = toc;
        %     if scenario.SampleTime-dt > 0
        %         pause(scenario.SampleTime-dt);
        %     end
        % 
        %     EgoGlobalState                              = optimalTrajectory(i,:);
        % 
        %     allS(end+1, 1)                              = EgoFrenetState(1); %#ok<SAGROW>
        %     allL(end+1, 1)                              = EgoFrenetState(4); %#ok<SAGROW>
        % 
        %     scenarioViewer.Actors(1).Position(1:2)      = EgoGlobalState(1:2);
        %     scenarioViewer.Actors(1).Velocity(1:2)      = [cos(EgoGlobalState(3)) sin(EgoGlobalState(3))]*EgoGlobalState(5);
        %     scenarioViewer.Actors(1).Yaw                = EgoGlobalState(3)*180/pi;
        %     scenarioViewer.Actors(1).AngularVelocity(3) = EgoGlobalState(4)*EgoGlobalState(5);
        % 
        %     % ヨー角 [rad]
        %     yawLog(end+1,1) = EgoGlobalState(3)*180/pi;  %#ok<SAGROW>
        % 
        %     % 横方向加速度 [m/s^2]
        %     latAcc             = EgoGlobalState(5)^2 * EgoGlobalState(4);
        %     latAccLog(end+1,1) = latAcc; %#ok<SAGROW>
        % 
        %     hold on;
        %     show(capList,'TimeStep',i:capList.MaxNumSteps,'FastUpdate',1);
        %     hold off;
        % 
        % 
        %     advance(scenarioViewer);
        % 
        %     frame = getframe(gcf);
        %     writeVideo(videoWriter, frame);
        % 
        %     tic;
        % end
        
        % AllActorGlobalStateに従った状態として直方体が描画されるよう修正したVer.
        for i = (2+(0:(stepPerUpdate-1)))
            dt_toc = toc;
            if scenario.SampleTime - dt_toc > 0
                pause(scenario.SampleTime - dt_toc);
            end
        
            % --- 自車の状態更新 ---
            EgoGlobalState = optimalTrajectory(i,:);
            scenarioViewer.Actors(1).Position(1:2) = EgoGlobalState(1:2);
            scenarioViewer.Actors(1).Yaw         = EgoGlobalState(3)*180/pi;
            scenarioViewer.Actors(1).Velocity(1:2) = [cos(EgoGlobalState(3)) sin(EgoGlobalState(3))]*EgoGlobalState(5);
        
            % --- 周辺車両の状態更新（シナリオの軌道を無視して上書き） ---
            for j = 1:numActors
                scenarioViewer.Actors(j+1).Position(1:2) = AllActorGlobalState(j, 1:2);
                scenarioViewer.Actors(j+1).Yaw         = AllActorGlobalState(j, 3)*180/pi;
                scenarioViewer.Actors(j+1).Velocity(1:2) = [cos(AllActorGlobalState(j,3)) sin(AllActorGlobalState(j,3))]*AllActorGlobalState(j,5);
            end
        
            % --- 描画の強制更新 (advanceは使わない) ---
            updatePlots(scenarioViewer); 
            
            % 動画保存とログ
            frame = getframe(gcf);
            writeVideo(videoWriter, frame);
            
            % (以下、既存のログ記録処理)
            allS(end+1, 1) = EgoFrenetState(1);
            allL(end+1, 1) = EgoFrenetState(4);
            yawLog(end+1,1) = EgoGlobalState(3)*180/pi;
            latAcc = EgoGlobalState(5)^2 * EgoGlobalState(4);
            latAccLog(end+1,1) = latAcc;
        
            tic;
        end
    end
end

% 動画ファイルを閉じる
close(videoWriter);
fprintf('Simulation video saved to: %s\n', videoFile);

%% ===== 周辺車両の速度・加速度・ジャークログ ===== %%
if ~isempty(timeLog) && ~isempty(actorSpeedLog) && ~isempty(actorAccLog) && ~isempty(actorJerkLog)

    outFigName = fullfile(pwd, 'actor_speed_acc_jerk_plot.png');

    myColors = [
        0.87, 0.33, 0.0;   % Actor 1
        0.93, 0.69, 0.13;  % Actor 2
        0.52, 0.09, 0.82;  % Actor 3
        0.23, 0.67, 0.20;  % Actor 4
        0.18, 0.75, 0.94;  % Actor 5
        0.82, 0.02, 0.55   % Actor 6
    ];

    hFig = figure('Name','Surrounding Vehicle Logs','NumberTitle','off','Color', 'w', "Position",[70, 70, 900, 900]);
    clf(hFig);

    legendLabels = arrayfun(@(x) sprintf('Actor %d', x), 1:numActors, 'UniformOutput', false);

    % 速度ログ
    subplot(3,1,1);
    set(gca, 'ColorOrder', myColors, 'NextPlot', 'replacechildren','Box', 'on', 'LineWidth', 0.5,'XColor', 'k', 'YColor', 'k');
    grid on; hold on;

    plot(timeLog, actorSpeedLog, 'LineStyle', '-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Speed (m/s)');
    title('Surrounding Vehicle Speeds (IDM)');
    legend(legendLabels, 'Location','northeast');

    xlim([0 max(timeLog)]);
    ylim([min(actorSpeedLog, [], 'all')-1, max(actorSpeedLog, [], 'all')+5]);
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 10.5);

    % 加速度ログ
    subplot(3,1,2);
    set(gca, 'ColorOrder', myColors, 'NextPlot', 'replacechildren','Box', 'on', 'LineWidth', 0.5,'XColor', 'k', 'YColor', 'k');
    grid on; hold on;

    plot(timeLog, actorAccLog, 'LineStyle', '-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    title('Surrounding Vehicle Accelerations (IDM)');
    legend(legendLabels, 'Location','northeast');

    xlim([0 max(timeLog)]);
    ylim([min(actorAccLog, [], 'all')-0.5, max(actorAccLog, [], 'all')+8]);
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 10.5);

    % ジャークログ
    subplot(3,1,3);
    set(gca, 'ColorOrder', myColors, 'NextPlot', 'replacechildren','Box', 'on', 'LineWidth', 0.5,'XColor', 'k', 'YColor', 'k');
    grid on; hold on;

    plot(timeLog, actorJerkLog, 'LineStyle', '-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Jerk (m/s^3)');
    title('Surrounding Vehicle Jerks (IDM)');
    legend(legendLabels, 'Location','northeast');

    xlim([0 max(timeLog)]);
    ylim([min(actorJerkLog, [], 'all')-1, max(actorJerkLog, [], 'all')+20]);
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 10.5);

    % 保存
    saveas(hFig, outFigName);
    fprintf('Combined plot saved to: %s\n', outFigName);
end


%% ===== 周辺ドライバー不快感の評価 （※論文を簡略化した評価関数）(Actor 1-6) ===== %%
fontName  = 'Times New Roman';
fontSize  = 10.5;

% 論文の閾値設定 (Table 1 )
thresholds = [0.28, 0.56, 0.89, 1.23, 1.89, 2.12];
labels = {'Excellent', 'Very Good', 'Good', 'So-so', 'Bad', 'Very Bad', 'Terrible'};
colors = [
    0.466 0.674 0.188; % Excellent
    0.635 0.750 0.100; % Very Good
    0.929 0.694 0.125; % Good
    1.000 0.850 0.000; % So-so
    1.000 0.500 0.000; % Bad
    0.850 0.325 0.098; % Very Bad
    1.000 0.000 0.000  % Terrible
];

for targetIdx = 1:numActors
    % データの抽出（関数内での判定のため、accは符号付きのまま渡す ）
    raw_acc  = actorAccLog(:, targetIdx);
    jerk_val = abs(actorJerkLog(:, targetIdx));
    
    % 不快感スコア (ME_norm) の算出
    me_log = arrayfun(@(a, j) Compute_Discomfort_Score(a, j), raw_acc, jerk_val);
    
    % 評価用の絶対値加速度
    acc_abs = abs(raw_acc);

    figure('Name', sprintf('Actor %d', targetIdx), 'Color', 'w', 'Position', [100, 50, 600, 900]);

    % --- 1. 加速度と7段階の閾値  ---
    subplot(3,1,1);
    plot(timeLog, acc_abs, 'LineWidth', 1.5, 'Color', [0 0.447 0.741]); hold on;
    for i = 1:length(thresholds)
        yline(thresholds(i), '--', labels{i}, 'Color', colors(i,:), ...
              'FontName', fontName, 'LineWidth', 1.1, 'LabelVerticalAlignment', 'bottom');
    end
    ylabel('Acceleration (abs) [m/s^2]');
    title('Acceleration & 7 Comfort Thresholds'); grid on;
    set(gca, 'FontName', fontName, 'FontSize', fontSize);

    % --- 2. 不快感指数 (ME_norm) の推移 [cite: 507] ---
    subplot(3,1,2);
    plot(timeLog, me_log, 'LineWidth', 1.5, 'Color', [0.635 0.078 0.184]);
    ylabel('Discomfort Score (ME_{norm})');
    ylim([1 8]); % スコア範囲に合わせる
    title('Predicted Subjective Discomfort Score'); grid on;
    set(gca, 'FontName', fontName, 'FontSize', fontSize);

    % --- 3. 7段階の滞在時間割合  ---
    subplot(3,1,3);
    counts = zeros(1, 7);
    counts(1) = sum(acc_abs <= thresholds(1));
    counts(2) = sum(acc_abs > thresholds(1) & acc_abs <= thresholds(2));
    counts(3) = sum(acc_abs > thresholds(2) & acc_abs <= thresholds(3));
    counts(4) = sum(acc_abs > thresholds(3) & acc_abs <= thresholds(4));
    counts(5) = sum(acc_abs > thresholds(4) & acc_abs <= thresholds(5));
    counts(6) = sum(acc_abs > thresholds(5) & acc_abs <= thresholds(6));
    counts(7) = sum(acc_abs > thresholds(6));
    
    p = pie(counts);
    colormap(colors);
    delete(findobj(p, 'Type', 'text')); % グラフ内のテキストを一旦消して整理
    pct = 100 * counts / max(sum(counts), eps);  % 各カテゴリの割合[%]
    legendWithPct = cellfun(@(lab, p) sprintf('%s (%.1f%%)', lab, p), ...
                            labels, num2cell(pct), 'UniformOutput', false);
    legend(legendWithPct, 'Location', 'eastoutside', 'FontName', fontName);

    title('Comfort Level Distribution (7 Levels)');
    set(gca, 'FontName', fontName, 'FontSize', fontSize);

    sgtitle(sprintf('Passenger Comfort Detailed Evaluation - Actor %d', targetIdx), ...
            'FontName', fontName, 'FontSize', 12, 'FontWeight', 'bold');
    
    figName = fullfile(pwd, sprintf('Actor_%d_Comfort_Evaluation.png', targetIdx));
    saveas(gcf, figName);
    fprintf('周辺ドライバー不快感の評価 saved to: %s\n', figName);
end

%% ===== 論文に基づく快適性分析 (3D, 周波数, A-J分布) ===== %%
for targetIdx = 1:numActors
    % データの準備
    raw_acc  = actorAccLog(:, targetIdx);
    jerk_val = abs(actorJerkLog(:, targetIdx));
    acc_abs  = abs(raw_acc);
    me_log   = arrayfun(@(a, j) Compute_Discomfort_Score(a, j), raw_acc, jerk_val);
    
    % 特性周波数の計算 (f = J / (2*pi*A))
    % ※ ゼロ除算を避けるため微小値を加算
    freq_log = jerk_val ./ (2 * pi * (acc_abs + 1e-6)); 
    % 加速度で正規化した不快感 (ME / A)
    norm_me_by_acc = me_log ./ (acc_abs + 1e-6);

    % 新しい解析用フィギュア
    figAnal = figure('Name', sprintf('Advanced Analysis: Actor %d', targetIdx), ...
                     'Color', 'w', 'Position', [800, 50, 600, 900]);

    % --- 1. 3Dコンフォート・ランドスケープ ---
    % 加速度、ジャーク、不快感の3次元相関
    subplot(3,1,1);
    scatter3(acc_abs, jerk_val, me_log, 20, me_log, 'filled');
    xlabel('A_{max} [m/s^2]'); ylabel('J_{max} [m/s^3]'); zlabel('ME_{norm}');
    title('ME_{norm} Landscape');
    colorbar; colormap(jet); grid on; view(45, 30);
    set(gca, 'FontName', fontName);

    % --- 2. 周波数依存性プロット ---
    % パルス時間の短さ（周波数）による不快感の変化
    subplot(3,1,2);
    semilogx(freq_log, norm_me_by_acc, 'o', 'MarkerSize', 3, 'Color', [0 0.447 0.741]);
    xlabel('Pulse sharpness [Hz]');
    ylabel('ME_{norm} / A_{max}');
    title('Frequency vs Normalized Discomfort');
    grid on; xlim([0.1, 10]);
    % 論文の知見：周波数が高い（右に行く）ほど、加速度あたりの不快感は下がる傾向にある
    set(gca, 'FontName', fontName);

    % --- 3. A_max vs J_max イベント分布マップ
    % 発生した挙動がどの快適領域に分布しているか
    subplot(3,1,3);
    scatter(acc_abs, jerk_val, 25, me_log, 'filled');
    xlabel('A_{max} [m/s^2]');
    ylabel('J_{max} [m/s^3]');
    title('A-J Distribution Map');
    colorbar; colormap(jet); grid on;
    % 快適領域の目安線を引く (J=8における閾値) [cite: 584, 645]
    for i = 1:length(thresholds)
        xline(thresholds(i), '--', 'Color', [0.5 0.5 0.5], 'Alpha', 0.5);
    end
    set(gca, 'FontName', fontName);

    sgtitle(sprintf('Multi-Dimensional Comfort Analysis - Actor %d', targetIdx), ...
            'FontName', fontName, 'FontWeight', 'bold');
    figName = fullfile(pwd, sprintf('Actor_%d_Comfort_Evaluation_3D_Hz_A-j.png', targetIdx));
    saveas(gcf, figName);
    fprintf('快適性分析 (3D, 周波数, A-J分布)  saved to: %s\n', figName);
end
%% ===== 車間距離ログ（図） ===== %%
figName = fullfile(pwd, 'gap_plot.png');

if ~isempty(timeLog)
    hFig = figure('Name','Ego to lead gaps','NumberTitle','off', "Position",[300, 300, 900, 300]);
    clf(hFig); hold on; grid on;

    % plot(timeLog, gapForwardLog, '-', 'Color', 'b', 'LineWidth', 4);
    plot(timeLog, gapBehindLog,  '-', 'Color', 'r', 'LineWidth', 4);
    % yline(forwardsafetyGap,      '--','Color', 'b', 'LineWidth', 2);
    yline(behindsafetyGap,       '--','Color', 'r', 'LineWidth', 2);

    xlabel('Time (s)');
    ylabel('Gap (m)');
    title('Vehicle to Vehicle Gaps');
    legend('Forward Gap','Behind Gap','Forward Safety Gap','Behind Safety Gap','Location','northeast');

    xlim([0 max(timeLog)]);
    ylim([0 max(gapBehindLog)+10]);

    set(gca,'FontName','Times New Roman','FontSize',10.5);
    saveas(hFig, figName);
    fprintf('Gap plot saved to: %s\n', figName);
end


 
%% ===== 速度ログ（図） ===== %%
speedFigName = fullfile(pwd, 'speed_plot.png');

if ~isempty(timeLog)
    hFig = figure('Name','Ego speed','NumberTitle','off', "Position",[350, 350, 900, 300]);
    clf(hFig); grid on; hold on;
    
    plot(timeLog, speedLog, '-', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 4);
    
    xlabel('Time (s)');
    ylabel('Speed (m/s)');
    title('Ego vehicle speed');
    legend('Ego speed', 'Location','northeast');

    xlim([0 max(timeLog)]);
    ylim([min(speedLog)-10 max(speedLog)+10]);
    
    set(gca,'FontName','Times New Roman','FontSize',10.5);
    saveas(hFig, speedFigName);
    fprintf('Speed plot saved to: %s\n', speedFigName); 
end



%% ===== ヨー角ログ（図） ===== %%
yawFigName = fullfile(pwd, 'yaw_plot.png');

if ~isempty(timeLog) && ~isempty(yawLog)
    hFig = figure('Name','Ego yaw angle','NumberTitle','off', "Position",[400, 400, 900, 300]);
    clf(hFig); grid on; hold on;

    plot(timeLog, yawLog, '-', 'Color', [0.3010 0.7450 0.9330], 'LineWidth', 4);

    xlabel('Time (s)');
    ylabel('Yaw angle (rad)');
    title('Ego vehicle yaw angle');
    legend('Yaw angle', 'Location','northeast');

    xlim([0 max(timeLog)]);
    ylim([-5 5]);

    set(gca,'FontName','Times New Roman','FontSize',10.5);
    saveas(hFig, yawFigName);
    fprintf('Yaw plot saved to: %s\n', yawFigName);
end

%% ===== 横方向加速度ログ（図） ===== %%
latAccFigName = fullfile(pwd, 'lateral_acc_plot.png');

if ~isempty(timeLog) && ~isempty(latAccLog)
    hFig = figure('Name','Lateral Acceleration','NumberTitle','off', "Position",[450, 450, 900, 300]);
    clf(hFig); grid on; hold on;

    plot(timeLog, latAccLog, '-', 'Color', [0, 0.5, 0], 'LineWidth', 4);

    xlabel('Time (s)');
    ylabel('Lateral Acceleration (m/s^2)');
    title('Lateral Acceleration');
    legend('Yaw angle', 'Location','northeast');

    xlim([0 max(timeLog)]);
    ylim([-1.5 1.5]);

    set(gca,'FontName','Times New Roman','FontSize',10.5);
    saveas(hFig, latAccFigName);
    fprintf('Lateral acc plot saved to: %s\n', latAccFigName);
end




%% ===== 走行モードログ（図） ===== %%
modeFigName = fullfile(pwd, 'mode_plot.png');

if ~isempty(timeLog)
    hFig = figure('Name','Selected Mode','NumberTitle','off', "Position",[500, 500, 900, 300]); 
    clf(hFig); grid on; hold on;

    hCruise     = stairs(timeLog, modeBinLog(:,1), 'Color', 'r',       'LineStyle', '-','LineWidth', 2);
    hFollow     = stairs(timeLog, modeBinLog(:,2), 'Color', 'b',       'LineStyle', '-','LineWidth', 2);
    hLaneChange = stairs(timeLog, modeBinLog(:,3), 'Color', [0 0.4 0], 'LineStyle', '-','LineWidth', 2);

    xlabel('Time (s)');
    ylabel('Mode');

    title('Selected Mode');
    legend([hCruise, hFollow, hLaneChange], {'Cruise','Follow','LaneChange'}, 'Location','northeast');
    
    xlim([0 max(timeLog)]);
    ylim([0 3.5]);

    set(gca,'FontName','Times New Roman','FontSize',10.5);
    saveas(hFig, modeFigName);
    fprintf('Mode plot saved to: %s\n', modeFigName);
end
    
%% ===== sl座標ログ（図）===== %%
figNameSL = fullfile(pwd, 'TrajectoryLog.png'); 
if ~isempty(allS) && ~isempty(allL)
    hFig = figure('Name','Frenet Trajectory','NumberTitle','off', "Position",[550, 550, 900, 300]);
    clf(hFig); hold on; grid on;
    
    z = zeros(size(allS));
    s_line = surface([allS'; allS'], [allL'; allL'], [z'; z'], [timeLog'; timeLog'], ...
        'EdgeColor', 'flat', 'FaceColor', 'none', 'LineWidth', 4);
    
    view(2); 
    colormap(jet);
    cb = colorbar;
    ylabel(cb, 'Time (s)', 'FontName', 'Times New Roman');
    
    xlabel('s (Longitudinal) [m]');
    ylabel('l (Lateral) [m]');
    title('Ego Vehicle Trajectory (Frenet Frame)');
    
    xlim([0, max(allS)]); 
    ylim([-laneWidth, laneWidth + 1]);
    
    % legend(s_line, 'Ego Path', 'Location', 'northeast');
    
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 10.5);
    saveas(hFig, figNameSL);
    fprintf('Frenet trajectory plot saved to: %s\n', figNameSL);
end

%% ===== 車間距離データログ（動画） ===== %%
if ~isempty(timeLog)
    v = VideoWriter('gap_bar.mp4', 'MPEG-4');
    v.FrameRate = 10; open(v);
    fig = figure('Name','Gap with Red Bar','NumberTitle','off', "Position",[200, 100, 900, 300]); 
    clf(fig); hold on; grid on;

    plot(timeLog, gapForwardLog, '-', 'Color', 'b', 'LineWidth', 3, 'DisplayName','Forward Gap');
    plot(timeLog, gapBehindLog,  '-', 'Color', 'r', 'LineWidth', 3, 'DisplayName','Behind Gap');
    yline(forwardsafetyGap, 'b--', 'LineWidth', 2, 'DisplayName', 'Forward Safety Gap');
    yline(behindsafetyGap, 'r--', 'LineWidth', 2, 'DisplayName', 'Behind Safety Gap');

    xlabel('Time (s)'); 
    ylabel('Gap (m)');
    title('Vehicle to Vehicle Gaps');
    legend('Forward Gap','Behind Gap','Forward Safety Gap', 'Behind Safety Gap','FontName','Times New Roman','FontSize',10.5,'Location','northeast');

    xlim([0 max(timeLog)]);
    ylim([0 max(gapForwardLog)+10]);
     
    set(gca,'FontName','Times New Roman','FontSize',10.5);
    barLine = xline(timeLog(1), 'r-', 'LineWidth', 2, 'HandleVisibility','off');

    timeFrames = timeLog;
    for i = 1:length(timeFrames)
        set(barLine, 'Value', timeFrames(i));
        title('Vehicle to Vehicle Gaps','FontName','Times New Roman','FontSize',14);
        frame = getframe(fig); 
        writeVideo(v, frame);
    end
    close(v);
    disp('✅ 車間距離データログが保存されました');
else
    warning('車間距離データログは記録されませんでした');
end


%% ===== ヨー角ログ（動画） ===== %%
if ~isempty(timeLog) && ~isempty(yawLog)
    v = VideoWriter('yaw_bar.mp4', 'MPEG-4');
    v.FrameRate = 10; open(v);
    fig = figure('Name','Yaw with Red Bar','NumberTitle','off', "Position",[250, 150, 900, 300]);
    clf(fig); hold on; grid on;

    plot(timeLog, yawLog, '-', 'Color', [0.3010 0.7450 0.9330], 'LineWidth', 3, 'DisplayName', 'Yaw Angle');

    xlabel('Time (s)');
    ylabel('Yaw Angle (rad)');
    title('Ego Vehicle Yaw Angle');
    legend('Yaw Angle','FontName','Times New Roman','FontSize',10.5,'Location','northeast');
 
    xlim([0 max(timeLog)]); 
    ylim([-5 5]);

    set(gca,'FontName','Times New Roman','FontSize',10.5);
    set(findall(gca,'Type','text'),'FontName','Times New Roman');

    barLine = xline(timeLog(1), 'r-', 'LineWidth', 2, 'HandleVisibility','off');

    timeFrames = timeLog;
    for i = 1:length(timeFrames)
        set(barLine, 'Value', timeFrames(i));
        title('Yaw Angle', 'FontName','Times New Roman','FontSize',14);
        frame = getframe(fig); 
        writeVideo(v, frame);
    end
    close(v);
    disp('✅ ヨー角データログが保存されました');
else
    warning('ヨー角データログは記録されませんでした');
end

%% ===== 横方向加速度ログ（動画） ===== %%
if ~isempty(timeLog) && ~isempty(latAccLog)
    v = VideoWriter('lateral_acc_bar.mp4', 'MPEG-4');
    v.FrameRate = 10; open(v);

    fig = figure('Name','Lateral Acceleration with Red Bar','NumberTitle','off', "Position",[300, 200, 900, 300]);
    clf(fig); hold on; grid on;

    plot(timeLog, latAccLog, 'k-', 'LineWidth', 3);

    xlabel('Time (s)');
    ylabel('Lateral Acceleration (m/s^2)');
    title('Ego Vehicle Lateral Acceleration');
    legend('Yaw Angle','FontName','Times New Roman','FontSize',10.5,'Location','northeast');

    xlim([0 max(timeLog)]);
    ylim([-2 2]);

    set(gca,'FontName','Times New Roman','FontSize',10.5);

    barLine = xline(timeLog(1), 'r-', 'LineWidth', 2, 'HandleVisibility','off');

    for i = 1:length(timeLog)
        set(barLine, 'Value', timeLog(i));
        title('Lateral Acc','FontName','Times New Roman','FontSize',14);
        frame = getframe(fig);
        writeVideo(v, frame);
    end
    close(v);
    disp('✅ 横方向加速度データログが保存されました');
else
    warning('横方向加速度データログは記録されませんでした');
end

%% ===== 速度ログ（動画） ===== %%
if ~isempty(timeLog) && ~isempty(speedLog)
    v = VideoWriter('speed_bar.mp4', 'MPEG-4');
    v.FrameRate = 10; open(v);
    fig = figure('Name','Speed with Red Bar','NumberTitle','off', "Position",[350, 250, 900, 300]); 
    clf(fig); hold on; grid on;

    plot(timeLog, speedLog, '-', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 3, 'DisplayName', 'Ego Speed');
    xlabel('Time (s)'); ylabel('Speed (m/s)');
    title('Ego Vehicle Speed');
    legend({'Ego Speed'},'Location','northeast','FontName','Times New Roman','FontSize',10.5);

    xlim([0 max(timeLog)]);
    ylim([min(speedLog)-10 max(speedLog)+10]);

    set(gca,'FontName','Times New Roman','FontSize',10.5);
    
    barLine = xline(timeLog(1), 'r-', 'LineWidth', 2, 'HandleVisibility','off');
    
    timeFrames = timeLog;
    for i = 1:length(timeFrames)
        set(barLine, 'Value', timeFrames(i));
        title('Ego Speed ',  'FontName','Times New Roman','FontSize',14);
        frame = getframe(fig); 
        writeVideo(v, frame);
    end
    close(v);
    disp('✅ 速度データログが保存されました');
else
    warning('速度データは記録されませんでした');
end

%% ===== 走行モードログ（動画） ===== %%
if ~isempty(timeLog)
    v = VideoWriter('mode_bar.mp4', 'MPEG-4');
    v.FrameRate = 10; open(v);
    fig = figure('Name','Mode with Red Bar','NumberTitle','off', "Position",[400, 300, 900, 300]); 
    clf(fig); hold on; grid on;

    hCruise = stairs(timeLog, modeBinLog(:,1), 'Color', 'r',       'LineStyle','-', 'LineWidth', 2);
    hFollow = stairs(timeLog, modeBinLog(:,2), 'Color', 'b',       'LineStyle','-', 'LineWidth', 2);
    hLane   = stairs(timeLog, modeBinLog(:,3), 'Color', [0 0.4 0], 'LineStyle','-', 'LineWidth', 2);

    xlabel('Time (s)');
    ylabel('Mode');
    title('Mode Transition Over Time');
    legend([hCruise, hFollow, hLane],'Cruise','Follow','LaneChange', 'Location','northeast', 'Box','on', 'FontName','Times New Roman','FontSize',10.5);

    ylim([0 4]); 
    xlim([0 max(timeLog)]); 

    set(gca,'FontName','Times New Roman','FontSize',10.5);

    barLine = xline(timeLog(1), 'r-', 'LineWidth', 2, 'HandleVisibility','off');

    timeFrames = timeLog;
    for i = 1:length(timeFrames)
        set(barLine, 'Value', timeFrames(i));
        title('Mode Transition', 'FontName','Times New Roman','FontSize',14);
        frame = getframe(fig);
        writeVideo(v, frame);
    end
    close(v);
    disp('✅ 走行モードデータログが保存されました');
else
    warning('走行モードデータは記録されませんでした');
end
