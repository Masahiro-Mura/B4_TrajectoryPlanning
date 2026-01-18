%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% ===== メインコード ===== %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc; clear; close all;
addpath('..\Scenario\m_file_GraduationThesis\');

scenario = scenario1;

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

%% パラメータ定義
replanRate          = P.replanRate;
timeHorizons        = P.timeHorizons;
maxHorizon          = P.maxHorizon;

% コスト関数のパラメータ
LatWeight           = P.LatWeight;
TimeWeight          = P.TimeWeight;
SpeedWeight         = P.SpeedWeight;
ForwardSafetyWeight = P.ForwardSafetyWeight;
BehindSafetyWeight  = P.BehindSafetyWeight;
MEscoreWeight       = P.MEscoreWeight; 

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
gapForwardLog_Conv = []; 
gapBehindLog_Conv  = []; 
leadIdxLog_Conv    = [];    
behindIdxLog_Conv  = [];  
timeLog_Conv       = [];       
speedLog_Conv      = []; 

% 周辺車両のログボックス
actorSpeedLog_Conv = [];
actorAccLog_Conv = [];
actorJerkLog_Conv  = [];

% 時間
TimeLog_Conv   = [];
LabelLog_Conv  = strings(0,1);  
modeBinLog_Conv    = zeros(0,3);      
yawLog_Conv        = [];  
latAccLog_Conv     = [];   

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
allS_Conv = []; 
allL_Conv = [];

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
        if abs(actorPositions(i, 2) - EgoGlobalState(2)) < 1.8 && gap_raw > 0 % ここの3.4についてはどの程度横移動したら後方車両に影響があるか知りたい
            v_leader = EgoGlobalState(5); 
            [acc, jerk, s_star] = IDM_improve(actorSpeeds(i), v_leader, gap, v_target, a_leader_current, actorAccels(i), dt);
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

        % scenario.Actors(i+1).Position = actorPositions(i, :);
        % scenario.Actors(i+1).Velocity = [actorSpeeds(i) 0 0];
    end
    
    %% 周辺車両の真値の取得
    % ===== 1. IDMで更新した周辺車両状態を AllActorGlobalState に反映 =====
    for i = 1:numActors
        idx = i;

        AllActorGlobalState(idx,1) = actorPositions(i,1);%#ok<SAGROW>   % x 
        AllActorGlobalState(idx,2) = actorPositions(i,2);%#ok<SAGROW>   % y
        AllActorGlobalState(idx,3) = 0;                  %#ok<SAGROW>   % yaw
        AllActorGlobalState(idx,4) = 0;                  %#ok<SAGROW>   % curvature
        AllActorGlobalState(idx,5) = actorSpeeds(i);     %#ok<SAGROW>   % speed
        AllActorGlobalState(idx,6) = actorAccels(i);     %#ok<SAGROW>   % acceleration
    end
    
    % 2. 確定した AllActorGlobalState を使って予測軌道（等速直線）を生成
    [AllActorGlobalState, futureTrajectory, isRunning] = RetrieveActorGroundTruth(...
        scenario, futureTrajectory, maxHorizon, AllActorGlobalState, EgoGlobalState);
    
    
    % 3. シナリオのアクター位置も同期
    for i = 1:numActors
        scenario.Actors(i+1).Position(1:2) = AllActorGlobalState(i,1:2);
        scenario.Actors(i+1).Velocity(1:2) = [cos(AllActorGlobalState(i,3)) sin(AllActorGlobalState(i,3))] * AllActorGlobalState(i,5);
        scenario.Actors(i+1).Yaw           = AllActorGlobalState(i,3)*180/pi;
        scenario.Actors(i+1).AngularVelocity(3) = AllActorGlobalState(i,4)*AllActorGlobalState(i,5);
    end
    
    format short g
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
        refPath,laneWidth,forwardsafetyGap,EgoGlobalState,AllActorGlobalState,carLen, timeHorizons);
    

    
    % 車線変更
    [termStatesLC, timesLC, leadGapsLC, behindGapsLC] = TSLaneChange_Conv( ...
    refPath, laneWidth, EgoGlobalState, AllActorGlobalState, carLen, rearAxleRatio, SpeedLimit, timeHorizons);
    
    
    Labels = [repmat("Cruise",     size(termStatesCC,1), 1);
              repmat("Follow",     size(termStatesF, 1), 1);
              repmat("LaneChange", size(termStatesLC,1), 1)
             ];
    AllTime = [timesCC; timesF; timesLC];
    AllForwardGaps = [leadGapsCC; leadGapsF; leadGapsLC];
    AllBehindGaps  = [behindGapsCC; behindGapsF; behindGapsLC];
    
    allTS = [termStatesCC; termStatesF; termStatesLC];
    numTS = [numel(timesCC); numel(timesF); numel(timesLC)];
    
    EgoFrenetState = global2frenet(refPath,EgoGlobalState);
    [frenetTraj,globalTraj] = connect(connector,EgoFrenetState,allTS,AllTime);
    
    [gapForwardLog_Conv, gapBehindLog_Conv, timeLog_Conv, speedLog_Conv] ...
         = ComputeGaps(EgoFrenetState, carLen, rearAxleRatio, refPath, AllActorGlobalState, ...
         laneWidth, numActors, gapForwardLog_Conv, gapBehindLog_Conv, leadIdxLog_Conv, behindIdxLog_Conv, timeLog_Conv, simStep, scenario, EgoGlobalState, speedLog_Conv);
    
    disp('後続車両との車間距離');
    disp(gapBehindLog_Conv(end));
    
    %% ===== 軌道選定 ===== %%
    % コスト関数 + min-max正規化
    [costTS, latCostTS, timeCostTS, speedCostTS] = ...
    CostFunction_Conv(allTS, AllTime, SpeedLimit, SpeedWeight, LatWeight, TimeWeight, EgoFrenetState);
    
    
    % 物理的制約を違反している軌道の排除
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
    ModeTable = table (Labels, AllTime, costTS, latCostTS, timeCostTS, speedCostTS, isValid, ...
        'VariableNames', {'Mode','PredictedTime','TotalCost','LateralCost','TimeCost','SpeedCost','ConstraintViolation'});
    disp(ModeTable);  
    
    %% 周辺車両の速度, 加速度，ジャークを記録 (IDM更新後に挿入)
    actorSpeedLog_Conv(end+1, :) = actorSpeeds;    %#ok<SAGROW>
    actorAccLog_Conv(end+1, :)   = actorAccels;            %#ok<SAGROW>
    actorJerkLog_Conv(end+1, :)  = actorJerks;           %#ok<SAGROW>

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
    
    TimeLog_Conv(end+1,1)  = (simStep-1)*scenario.SampleTime; %#ok<SAGROW>
    LabelLog_Conv(end+1,1) = selLabel;                        %#ok<SAGROW>
    modeBinLog_Conv(end+1,:)   = onehot;                      %#ok<SAGROW>
    

    %% ===== 軌道の可視化 ===== %%
    lineHandles = VisualizeScene(lineHandles,globalTraj,isValid,trajectoryEvaluation);

    hold on;
    show(capList,'TimeStep',1:capList.MaxNumSteps,'FastUpdate',1);
    hold off;
    
    if isempty(optimalTrajectory)
            error('No valid trajectory has been found.');
    else
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
            allS_Conv(end+1, 1) = EgoFrenetState(1);               %#ok<SAGROW>
            allL_Conv(end+1, 1) = EgoFrenetState(4);               %#ok<SAGROW>
            yawLog_Conv(end+1,1) = EgoGlobalState(3)*180/pi;       %#ok<SAGROW>
            latAcc = EgoGlobalState(5)^2 * EgoGlobalState(4); 
            latAccLog_Conv(end+1,1) = latAcc;                      %#ok<SAGROW>
            
            tic;
        end
    end
end

% 動画ファイルを閉じる
close(videoWriter);
fprintf('Simulation video saved to: %s\n', videoFile);


%% ===== 周辺車両の速度・加速度・ジャークログ (個別出力) ===== %%
if ~isempty(timeLog_Conv) && ~isempty(actorSpeedLog_Conv) && ~isempty(actorAccLog_Conv) && ~isempty(actorJerkLog_Conv)

    myColors = [0.87, 0.33, 0.0; 0.93, 0.69, 0.13; 0.52, 0.09, 0.82; 
                0.23, 0.67, 0.20; 0.18, 0.75, 0.94; 0.82, 0.02, 0.55];
    legendLabels = arrayfun(@(x) sprintf('Actor %d', x), 1:numActors, 'UniformOutput', false);
    
    % 共通設定用関数（コード重複回避）
    applyStyle = @(ax) set(ax, 'ColorOrder', myColors, 'NextPlot', 'replacechildren', ...
        'Box', 'on', 'LineWidth', 0.5, 'XColor', 'k', 'YColor', 'k', ...
        'FontName', 'Times New Roman', 'FontSize', 10.5, 'XGrid', 'on', 'YGrid', 'on');

    % --- 1. 速度ログ ---
    hFig1 = figure('Name','Speed','Color','w','Position',[70, 70, 450, 250]);
    applyStyle(gca); hold on;
    plot(timeLog_Conv, actorSpeedLog_Conv, 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Speed (m/s)'); title('Surrounding Vehicle Speeds');
    legend(legendLabels, 'Location','northeast');
    xlim([0 max(timeLog_Conv)]);
    ylim([min(actorSpeedLog_Conv, [], 'all')-1, max(actorSpeedLog_Conv, [], 'all')+5]);
    saveas(hFig1, fullfile(pwd, 'actor_speed_plot.png'));

    % --- 2. 加速度ログ ---
    hFig2 = figure('Name','Acceleration','Color','w','Position',[70, 70, 450, 250]);
    applyStyle(gca); hold on;
    plot(timeLog_Conv, actorAccLog_Conv, 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Acceleration (m/s^2)'); title('Surrounding Vehicle Accelerations');
    legend(legendLabels, 'Location','northeast');
    xlim([0 max(timeLog_Conv)]);
    ylim([min(actorAccLog_Conv, [], 'all')-0.5, max(actorAccLog_Conv, [], 'all')+8]);
    saveas(hFig2, fullfile(pwd, 'actor_acc_plot.png'));

    % --- 3. ジャークログ ---
    hFig3 = figure('Name','Jerk','Color','w','Position',[70, 70, 450, 250]);
    applyStyle(gca); hold on;
    plot(timeLog_Conv, actorJerkLog_Conv, 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Jerk (m/s^3)'); title('Surrounding Vehicle Jerks');
    legend(legendLabels, 'Location','northeast');
    xlim([0 max(timeLog_Conv)]);
    ylim([min(actorJerkLog_Conv, [], 'all')-1, max(actorJerkLog_Conv, [], 'all')+20]);
    saveas(hFig3, fullfile(pwd, 'actor_jerk_plot.png'));

    fprintf('Individual plots saved to current directory.\n');
end


%% ===== 各Actorの3指標（速度・加速度・ジャーク）個別出力スクリプト ===== %%
if ~isempty(timeLog_Conv) && ~isempty(actorSpeedLog_Conv) && ...
   ~isempty(actorAccLog_Conv) && ~isempty(actorJerkLog_Conv)

    % パラメータ（目標速度）の読み込み
    P = parameters(); 

    % カラーパレット定義
    myColors = [0.87, 0.33, 0.0; 0.93, 0.69, 0.13; 0.52, 0.09, 0.82; 
                0.23, 0.67, 0.20; 0.18, 0.75, 0.94; 0.82, 0.02, 0.55];

    % 全アクター共通のy軸範囲を固定
    vLim = [min(actorSpeedLog_Conv, [], 'all')-1, max(actorSpeedLog_Conv, [], 'all')+5];
    aLim = [min(actorAccLog_Conv, [], 'all')-0.5, max(actorAccLog_Conv, [], 'all')+8];
    jLim = [min(actorJerkLog_Conv, [], 'all')-1, max(actorJerkLog_Conv, [], 'all')+20];

    for i = 1:numActors
        currentColor = myColors(mod(i-1, size(myColors, 1)) + 1, :);
        
        % --- 1. 速度 (Speed) ---
        hFigV = figure('Name', sprintf('Actor %d Speed', i), 'Color', 'w', 'Position', [100, 100, 450, 300]);
        hold on;
        plot(timeLog_Conv, actorSpeedLog_Conv(:, i), 'Color', currentColor, 'LineWidth', 2, 'DisplayName', 'Actual Speed');
        targetVelField = sprintf('Vel_actor%d', i);
        if isfield(P, targetVelField)
            yline(P.(targetVelField), '--k', 'LineWidth', 1.5, 'DisplayName', 'Target Velocity');
        end
        title(sprintf('Actor %d: Speed', i)); ylabel('Speed (m/s)'); xlabel('Time (s)');
        % 注釈の枠線を黒・太さ 
        legend('Location', 'northeast', 'EdgeColor', 'k', 'LineWidth', 0.5);
        xlim([0 max(timeLog_Conv)]); ylim(vLim);
        grid on; box on;
        % グラフの枠線を黒・太さ 
        set(gca, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 0.5, 'FontName', 'Times New Roman', 'FontSize', 10.5);
        saveas(hFigV, fullfile(pwd, sprintf('actor_%d_speed.png', i)));

        % --- 2. 加速度 (Acceleration) ---
        hFigA = figure('Name', sprintf('Actor %d Acceleration', i), 'Color', 'w', 'Position', [150, 150, 450, 300]);
        plot(timeLog_Conv, actorAccLog_Conv(:, i), 'Color', currentColor, 'LineWidth', 2, 'DisplayName', 'Actual Accel');
        title(sprintf('Actor %d: Acceleration', i)); ylabel('Acceleration (m/s^2)'); xlabel('Time (s)');
        legend('Location', 'northeast', 'EdgeColor', 'k', 'LineWidth', 0.5);
        xlim([0 max(timeLog_Conv)]); ylim(aLim);
        grid on; box on;
        set(gca, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 0.5, 'FontName', 'Times New Roman', 'FontSize', 10.5);
        saveas(hFigA, fullfile(pwd, sprintf('actor_%d_acc.png', i)));

        % --- 3. ジャーク (Jerk) ---
        hFigJ = figure('Name', sprintf('Actor %d Jerk', i), 'Color', 'w', 'Position', [200, 200, 450, 300]);
        plot(timeLog_Conv, actorJerkLog_Conv(:, i), 'Color', currentColor, 'LineWidth', 2, 'DisplayName', 'Actual Jerk');
        title(sprintf('Actor %d: Jerk', i)); ylabel('Jerk (m/s^3)'); xlabel('Time (s)');
        legend('Location', 'northeast', 'EdgeColor', 'k', 'LineWidth', 0.5);
        xlim([0 max(timeLog_Conv)]); ylim(jLim);
        grid on; box on;
        set(gca, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 0.5, 'FontName', 'Times New Roman', 'FontSize', 10.5);
        saveas(hFigJ, fullfile(pwd, sprintf('actor_%d_jerk.png', i)));
        
        % close([hFigV, hFigA, hFigJ]); 
    end
    fprintf('Saved all figures with black borders (thickness 1.5) for %d actors.\n', numActors);
end

%% 1. 初期設定（共通パラメータ・閾値）
fontName  = 'Times New Roman';
fontSize  = 10.5;

% 論文の閾値設定 (Table 1)
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

% 2. 全Actorの加速度グラフ生成（保存用）
if ~isempty(timeLog_Conv)
    for targetIdx = 1:numActors
        figName = fullfile(pwd, sprintf('Actor_%d_Acceleration.png', targetIdx));
        acc_abs = abs(actorAccLog_Conv(:, targetIdx));
        
        hFig = figure('Name', sprintf('Actor %d Acc', targetIdx), 'NumberTitle', 'off', 'Position', [300, 300, 550, 250], 'Color', 'w');
        clf(hFig); hold on; grid on;
        
        % 加速度プロット
        plot(timeLog_Conv, acc_abs, 'LineWidth', 1.5, 'Color', [0 0.447 0.741]);
        
        % 閾値線の描画（成功例のスタイルを継承）
        for i = 1:length(thresholds)
            yline(thresholds(i), '--', labels{i}, 'Color', colors(i,:), ...
                  'LineWidth', 1.1, 'LabelVerticalAlignment', 'bottom', 'FontName', fontName);
        end
        
        xlabel('Time (s)');
        ylabel('Acceleration (abs) [m/s^2]');
        title(sprintf('Passenger Comfort: Acceleration - Actor %d', targetIdx));
        
        xlim([0 max(timeLog_Conv)]);
        ylim([0 max(acc_abs) + 0.5]);
        
        set(gca,'FontName','Times New Roman','FontSize',10.5);
        saveas(hFig, figName);
        fprintf('Acceleration plot saved to: %s\n', figName);
        % close(hFig); % 大量の図が開くのを防ぐ場合はコメントアウトを外してください
    end
end

%% 3. 全Actorの不快感指数 (ME_norm) グラフ生成
if ~isempty(timeLog_Conv)
    for targetIdx = 1:numActors
        figName = fullfile(pwd, sprintf('Actor_%d_ME_Score.png', targetIdx));
        raw_acc  = actorAccLog_Conv(:, targetIdx);
        jerk_val = abs(actorJerkLog_Conv(:, targetIdx));
        me_log = arrayfun(@(a, j) MEscore(a, j), raw_acc, jerk_val);
        
        hFig = figure('Name', sprintf('Actor %d ME', targetIdx), 'NumberTitle', 'off', 'Position', [300, 300, 550, 250], 'Color', 'w');
        clf(hFig); hold on; grid on;
        
        plot(timeLog_Conv, me_log, 'LineWidth', 1.5, 'Color', [0.635 0.078 0.184]);
        
        xlabel('Time (s)');
        ylabel('Discomfort Score (ME_{norm})');
        title(sprintf('Passenger Comfort: ME Score - Actor %d', targetIdx));
        
        xlim([0 max(timeLog_Conv)]);
        ylim([-1 8]); 
        
        set(gca,'FontName','Times New Roman','FontSize',10.5);
        saveas(hFig, figName);
        fprintf('ME Score plot saved to: %s\n', figName);
    end
end

%% 4. 全Actorの快適性レベル分布（円グラフ）生成
for targetIdx = 1:numActors
    figName = fullfile(pwd, sprintf('Actor_%d_Distribution.png', targetIdx));
    acc_abs = abs(actorAccLog_Conv(:, targetIdx));
    
    hFig = figure('Name', sprintf('Actor %d Dist', targetIdx), 'NumberTitle', 'off', 'Position', [300, 300, 550, 250], 'Color', 'w');
    clf(hFig); hold on;
    
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
    delete(findobj(p, 'Type', 'text')); 
    
    pct = 100 * counts / max(sum(counts), eps);
    legendWithPct = cellfun(@(lab, p) sprintf('%s (%.1f%%)', lab, p), ...
                            labels, num2cell(pct), 'UniformOutput', false);
    legend(legendWithPct, 'Location', 'eastoutside', 'FontName', fontName, 'FontSize', 8);
    
    title(sprintf('Comfort Level Distribution - Actor %d', targetIdx));
    
    set(gca,'FontName','Times New Roman','FontSize',10.5);
    saveas(hFig, figName);
    fprintf('Distribution plot saved to: %s\n', figName);
end




%% ===== 車間距離ログ（図） ===== %%
figName = fullfile(pwd, 'gap_plot.png');

if ~isempty(timeLog_Conv)
    hFig = figure('Name','Ego to lead gaps','NumberTitle','off', "Position",[300, 300, 550, 250]);
    clf(hFig); hold on; grid on;

    % plot(timeLog, gapForwardLog, '-', 'Color', 'b', 'LineWidth', 4);
    plot(timeLog_Conv, gapBehindLog_Conv,  '-', 'Color', 'r', 'LineWidth', 4);
    % yline(forwardsafetyGap,      '--','Color', 'b', 'LineWidth', 2);
    % yline(behindsafetyGap,       '--','Color', 'r', 'LineWidth', 2);

    xlabel('Time (s)');
    ylabel('Gap (m)');
    title('Vehicle to Vehicle Gaps');
    % legend('Forward Gap','Behind Gap','Forward Safety Gap','Behind Safety Gap','Location','northeast');
    legend('Behind Gap','Behind Safety Gap','Location','northeast');

    xlim([0 max(timeLog_Conv)]);
    ylim([0 max(gapBehindLog_Conv)+10]);

    set(gca,'FontName','Times New Roman','FontSize',10.5);
    saveas(hFig, figName);
    fprintf('Gap plot saved to: %s\n', figName);
end


 
%% ===== 速度ログ（図） ===== %%
speedFigName = fullfile(pwd, 'speed_plot.png');

if ~isempty(timeLog_Conv)
    hFig = figure('Name','Ego speed','NumberTitle','off', "Position",[350, 650, 550, 250]);
    clf(hFig); grid on; hold on;
    
    plot(timeLog_Conv, speedLog_Conv, '-', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 4);
    
    xlabel('Time (s)');
    ylabel('Speed (m/s)');
    title('Ego vehicle speed');
    legend('Ego speed', 'Location','northeast');

    xlim([0 max(timeLog_Conv)]);
    ylim([min(speedLog_Conv)-10 max(speedLog_Conv)+10]);
    
    set(gca,'FontName','Times New Roman','FontSize',10.5);
    saveas(hFig, speedFigName);
    fprintf('Speed plot saved to: %s\n', speedFigName); 
end



%% ===== ヨー角ログ（図） ===== %%
yawFigName = fullfile(pwd, 'yaw_plot.png');

if ~isempty(timeLog_Conv) && ~isempty(yawLog_Conv)
    hFig = figure('Name','Ego yaw angle','NumberTitle','off', "Position",[400, 900, 550, 250]);
    clf(hFig); grid on; hold on;

    plot(timeLog_Conv, yawLog_Conv, '-', 'Color', [0.3010 0.7450 0.9330], 'LineWidth', 4);

    xlabel('Time (s)');
    ylabel('Yaw angle (rad)');
    title('Ego vehicle yaw angle');
    legend('Yaw angle', 'Location','northeast');

    xlim([0 max(timeLog_Conv)]);
    ylim([-5 5]);

    set(gca,'FontName','Times New Roman','FontSize',10.5);
    saveas(hFig, yawFigName);
    fprintf('Yaw plot saved to: %s\n', yawFigName);
end

%% ===== 横方向加速度ログ（図） ===== %%
latAccFigName = fullfile(pwd, 'lateral_acc_plot.png');

if ~isempty(timeLog_Conv) && ~isempty(latAccLog_Conv)
    hFig = figure('Name','Lateral Acceleration','NumberTitle','off', "Position",[450, 450, 550, 250]);
    clf(hFig); grid on; hold on;

    plot(timeLog_Conv, latAccLog_Conv, '-', 'Color', [0, 0.5, 0], 'LineWidth', 4);

    xlabel('Time (s)');
    ylabel('Lateral Acceleration (m/s^2)');
    title('Lateral Acceleration');
    legend('Yaw angle', 'Location','northeast');

    xlim([0 max(timeLog_Conv)]);
    ylim([-1.5 1.5]);

    set(gca,'FontName','Times New Roman','FontSize',10.5);
    saveas(hFig, latAccFigName);
    fprintf('Lateral acc plot saved to: %s\n', latAccFigName);
end




%% ===== 走行モードログ（図） ===== %%
modeFigName = fullfile(pwd, 'mode_plot.png');

if ~isempty(timeLog_Conv)
    hFig = figure('Name','Selected Mode','NumberTitle','off', "Position",[500, 700, 550, 250]); 
    clf(hFig); grid on; hold on;
    
    hCruise     = stairs(timeLog_Conv, modeBinLog_Conv(:,1), 'Color', 'r',       'LineStyle', '-','LineWidth', 2);
    hFollow     = stairs(timeLog_Conv, modeBinLog_Conv(:,2), 'Color', 'b',       'LineStyle', '-','LineWidth', 2);
    hLaneChange = stairs(timeLog_Conv, modeBinLog_Conv(:,3), 'Color', [0 0.4 0], 'LineStyle', '-','LineWidth', 2);
    
    xlabel('Time (s)');
    ylabel('Mode');
    
    title('Selected Mode');
    legend([hCruise, hFollow, hLaneChange], {'Cruise','Follow','LaneChange'}, 'Location','northeast');
    
    xlim([0 max(timeLog_Conv)]);
    ylim([0 3.5]);
    
    set(gca,'FontName','Times New Roman','FontSize',10.5);
    saveas(hFig, modeFigName);
    fprintf('Mode plot saved to: %s\n', modeFigName);
end

%% ===== sl座標ログ（図）===== %%
figNameSL = fullfile(pwd, 'TrajectoryLog.png'); 
if ~isempty(allS_Conv) && ~isempty(allL_Conv)
    hFig = figure('Name','Frenet Trajectory','NumberTitle','off', "Position",[650, 250, 550, 250]);
    clf(hFig); hold on; grid on;
    
    z = zeros(size(allS_Conv));
    s_line = surface([allS_Conv'; allS_Conv'], [allL_Conv'; allL_Conv'], [z'; z'], [timeLog_Conv'; timeLog_Conv'], ...
        'EdgeColor', 'flat', 'FaceColor', 'none', 'LineWidth', 4);
    
    view(2); 
    colormap(jet);
    cb = colorbar;
    ylabel(cb, 'Time (s)', 'FontName', 'Times New Roman');
    
    xlabel('s (Longitudinal) [m]');
    ylabel('l (Lateral) [m]');
    title('Ego Vehicle Trajectory (Frenet Frame)');
    
    xlim([0, max(allS_Conv)]); 
    ylim([-laneWidth, laneWidth + 1]);
    
    % legend(s_line, 'Ego Path', 'Location', 'northeast');
    
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 10.5);
    saveas(hFig, figNameSL);
    fprintf('Frenet trajectory plot saved to: %s\n', figNameSL);
end