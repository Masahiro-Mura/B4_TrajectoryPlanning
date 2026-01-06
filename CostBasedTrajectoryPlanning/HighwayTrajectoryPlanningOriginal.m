%%%%% ===== メインコード ===== %%%%%

clc; clear; close all;
addpath('..\Scenario\m_file\');
scenario = scenario_20251223;

P = parameters();

carLen        = P.carLen;
carWidth      = P.carWidth;
rearAxleRatio = P.rearAxleRatio;
laneWidth     = P.laneWidth;

plot(scenario);
waypoints = [0 0; 1000 0]; 
refPath = referencePathFrenet(waypoints); 

connector = trajectoryGeneratorFrenet(refPath);

capList = dynamicCapsuleList;

egoID = 1; 
[egoID, egoGeom] = egoGeometry(capList,egoID); 

egoGeom.Geometry.Length = carLen; 
egoGeom.Geometry.Radius = carWidth/2; 
egoGeom.Geometry.FixedTransform(1,end) = -carLen*rearAxleRatio; 

updateEgoGeometry(capList,egoID,egoGeom);

actorID = (1:6); 
actorGeom = repelem(egoGeom,6,1); 
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
    InitializeSimulator(scenario,capList,refPath,laneWidth,replanRate,carLen);

numActors = numel(actorID);
gapForwardLog = []; 
gapBehindLog  = []; 
leadIdxLog    = [];    
behindIdxLog  = [];  
timeLog       = [];       
speedLog      = [];     

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

%% while文 開始
while isRunning
    simStep = simStep + 1;

    [AllActorGlobalState,futureTrajectory,isRunning] = RetrieveActorGroundTruth(...
        scenario,futureTrajectory,replanRate,maxHorizon);
      
    %% ===== Trajectory planning ===== %%
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
              repmat("LaneChange", size(termStatesLC,1), 1)
             ];
    AllTime = [timesCC; timesF; timesLC];
    AllForwardGaps = [leadGapsCC; leadGapsF; leadGapsLC];
    AllBehindGaps  = [behindGapsCC; behindGapsF; behindGapsLC];

    allTS = [termStatesCC; termStatesF; termStatesLC];
    numTS = [numel(timesCC); numel(timesF); numel(timesLC)];
    
    EgoFrenetState = global2frenet(refPath,EgoGlobalState);
    [frenetTraj,globalTraj] = connect(connector,EgoFrenetState,allTS,AllTime);
    
    [AllActorFrenetState, EgoFrenetFront, EgoFrenetRear, AllActorFrenetFront, AllActorFrenetRear, ForwardGap, RearGap, gapForwardLog, gapBehindLog, timeLog, speedLog] ...
         = ComputeGaps(EgoFrenetState, carLen, rearAxleRatio, refPath, AllActorGlobalState, ...
         laneWidth, numActors, gapForwardLog, gapBehindLog, leadIdxLog, behindIdxLog, timeLog, simStep, scenario, EgoGlobalState, speedLog);


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

    disp("経過時間 " + num2str(simStep/10));
    a = table (Labels, AllTime, costTS, latCostTS, timeCostTS, speedCostTS, ForwardSafeCostTS, BehindSafeCostTS, isValid, ...
        'VariableNames', {'Mode','PredictedTime','TotalCost','LateralCost','TimeCost','SpeedCost','ForwardGapCost','BeindGapCost','ConstraintViolation'});
    disp(a);  
      



    %% ===== 横偏差コスト ===== %%
    CruiseLatCost    = NaN(1,5); 
    FollowingLatCost = NaN(1,5); 
    ChangeLatCost    = NaN(1,10);  
    
    if any(strcmp(a.('Mode'), 'Cruise'))
        CruiseLatCost    = a.('LateralCost')(strcmp(a.('Mode'), 'Cruise'))'; 
    end

    if any(strcmp(a.('Mode'), 'Follow'))
        FollowingLatCost = a.('LateralCost')(strcmp(a.('Mode'), 'Follow'))';
    end
    
    if any(strcmp(a.('Mode'), 'LaneChange'))
        ChangeLatCost    = a.('LateralCost')(strcmp(a.('Mode'), 'LaneChange'))';
    end
    
   
    if length(CruiseLatCost) < 5
        CruiseLatCost    = [CruiseLatCost,    NaN(1, 5 - length(CruiseLatCost))]; 
    end
    
    if length(FollowingLatCost) < 5
        FollowingLatCost = [FollowingLatCost, NaN(1, 5 - length(FollowingLatCost))];  
    end
    
    if length(ChangeLatCost) < 10
        ChangeLatCost    = [ChangeLatCost,    NaN(1, 10 - length(ChangeLatCost))];  
    end
   

    CruiseLatCostLog(end+1,:)    = CruiseLatCost;    %#ok<SAGROW>
    FollowingLatCostLog(end+1,:) = FollowingLatCost; %#ok<SAGROW>
    ChangeLatCostLog (end+1,:)   = ChangeLatCost;    %#ok<SAGROW>
       
    %% ===== 速度差コスト ===== %%
    CruiseSpeedCost    = NaN(1,5); 
    FollowingSpeedCost = NaN(1,5); 
    ChangeSpeedCost    = NaN(1,10);  
    
    if any(strcmp(a.('Mode'), 'Cruise'))
        CruiseSpeedCost   = a.('SpeedCost')(strcmp(a.('Mode'), 'Cruise'))'; 
    end
    
    if any(strcmp(a.('Mode'), 'Follow'))
        FollowingSpeedCost = a.('SpeedCost')(strcmp(a.('Mode'), 'Follow'))'; 
    end
    
    if any(strcmp(a.('Mode'), 'LaneChange'))
        ChangeSpeedCost    = a.('SpeedCost')(strcmp(a.('Mode'), 'LaneChange'))'; 
    end
    

    if length(CruiseSpeedCost) < 5
        CruiseSpeedCost    = [CruiseSpeedCost,    NaN(1, 5 - length(CruiseSpeedCost))]; 
    end
    
    if length(FollowingSpeedCost) < 5
        FollowingSpeedCost = [FollowingSpeedCost, NaN(1, 5 - length(FollowingSpeedCost))]; 
    end
    
    if length(ChangeSpeedCost) < 10
        ChangeSpeedCost    = [ChangeSpeedCost,    NaN(1, 10 - length(ChangeSpeedCost))];
    end  
   
    
    CruiseSpeedCostLog(end+1,:)    = CruiseSpeedCost;    %#ok<SAGROW>
    FollowingSpeedCostLog(end+1,:) = FollowingSpeedCost; %#ok<SAGROW>
    ChangeSpeedCostLog (end+1,:)   = ChangeSpeedCost;    %#ok<SAGROW>

    %% ===== 時間コスト ===== %%
    CruiseTimeCost    = NaN(1,5); 
    FollowingTimeCost = NaN(1,5); 
    ChangeTimeCost    = NaN(1,10);  

    if any(strcmp(a.('Mode'), 'Cruise'))
        CruiseTimeCost    = a.('TimeCost')(strcmp(a.('Mode'), 'Cruise'))'; 
    end
    
    if any(strcmp(a.('Mode'), 'Follow'))
        FollowingTimeCost = a.('TimeCost')(strcmp(a.('Mode'), 'Follow'))'; 
    end
    
    if any(strcmp(a.('Mode'), 'LaneChange'))
        ChangeTimeCost    = a.('TimeCost')(strcmp(a.('Mode'), 'LaneChange'))'; 
    end
    

    if length(CruiseTimeCost) < 5
        CruiseTimeCost    = [CruiseTimeCost,    NaN(1, 5 - length(CruiseTimeCost))];  
    end
    
    if length(FollowingTimeCost) < 5
        FollowingTimeCost = [FollowingTimeCost, NaN(1, 5 - length(FollowingTimeCost))]; 
    end
    
    if length(ChangeTimeCost) < 10
        ChangeTimeCost    = [ChangeTimeCost,    NaN(1, 10 - length(ChangeTimeCost))];
    end
   
  
    CruiseTimeCostLog(end+1,:)    = CruiseTimeCost;    %#ok<SAGROW>
    FollowingTimeCostLog(end+1,:) = FollowingTimeCost; %#ok<SAGROW>
    ChangeTimeCostLog (end+1,:)   = ChangeTimeCost;    %#ok<SAGROW>


     %% ===== 全コスト ===== %%
    CruiseTotalCost    = NaN(1,5);  
    FollowingTotalCost = NaN(1,5);  
    ChangeTotalCost    = NaN(1,10);  

    if any(strcmp(a.('Mode'), 'Cruise'))
        CruiseTotalCost   = a.('TotalCost')(strcmp(a.('Mode'), 'Cruise'))'; 
    end
    
    if any(strcmp(a.('Mode'), 'Follow'))
        FollowingTotalCost = a.('TotalCost')(strcmp(a.('Mode'), 'Follow'))';
    end
  
    if any(strcmp(a.('Mode'), 'LaneChange'))
        ChangeTotalCost    = a.('TotalCost')(strcmp(a.('Mode'), 'LaneChange'))';
    end
    

    if length(CruiseTotalCost) < 5
        CruiseTotalCost    = [CruiseTotalCost,    NaN(1, 5 - length(CruiseTotalCost))];  
    end
    
    if length(FollowingTotalCost) < 5
        FollowingTotalCost = [FollowingTotalCost, NaN(1, 5 - length(FollowingTotalCost))]; 
    end
    
    if length(ChangeTotalCost) < 10
        ChangeTotalCost    = [ChangeTotalCost,    NaN(1, 10 - length(ChangeTotalCost))]; 
    end
   

    CruiseTotalCostLog(end+1,:)    = CruiseTotalCost;    %#ok<SAGROW>
    FollowingTotalCostLog(end+1,:) = FollowingTotalCost; %#ok<SAGROW>
    ChangeTotalCostLog (end+1,:)   = ChangeTotalCost;    %#ok<SAGROW>
    
 
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
        for i  = (2+(0:(stepPerUpdate-1)))
            dt = toc;
            if scenario.SampleTime-dt > 0
                pause(scenario.SampleTime-dt);
            end
            
            EgoGlobalState                              = optimalTrajectory(i,:);

            allS(end+1, 1)                              = EgoFrenetState(1); %#ok<SAGROW>
            allL(end+1, 1)                              = EgoFrenetState(4); %#ok<SAGROW>

            scenarioViewer.Actors(1).Position(1:2)      = EgoGlobalState(1:2);
            scenarioViewer.Actors(1).Velocity(1:2)      = [cos(EgoGlobalState(3)) sin(EgoGlobalState(3))]*EgoGlobalState(5);
            scenarioViewer.Actors(1).Yaw                = EgoGlobalState(3)*180/pi;
            scenarioViewer.Actors(1).AngularVelocity(3) = EgoGlobalState(4)*EgoGlobalState(5);

            % ヨー角 [rad]
            yawLog(end+1,1) = EgoGlobalState(3)*180/pi;  %#ok<SAGROW>
            
            % 横方向加速度 [m/s^2]
            latAcc             = EgoGlobalState(5)^2 * EgoGlobalState(4);
            latAccLog(end+1,1) = latAcc; %#ok<SAGROW>
            
            hold on;
            show(capList,'TimeStep',i:capList.MaxNumSteps,'FastUpdate',1);
            hold off;
          
            advance(scenarioViewer);

            frame = getframe(gcf);
            writeVideo(videoWriter, frame);

            tic;
        end
    end
end

% 動画ファイルを閉じる
close(videoWriter);
fprintf('Simulation video saved to: %s\n', videoFile);


%% ===== 横方向加速度ログ（図） ===== %%
figName = fullfile(pwd, 'LateralCost.png');

if ~isempty(timeLog)
    hFig = figure('Name','LateralCost','NumberTitle','off', "Position",[100, 100, 900, 300]);
    clf(hFig); hold on; grid on;

    for col = 1:size(CruiseLatCostLog, 2)  
        validCruiseLatData = ~isnan(CruiseLatCostLog(:, col));  
        validFollowLatData = ~isnan(FollowingLatCostLog(:, col)); 
        validChangeLatData = ~isnan(ChangeLatCostLog(:, col));

        if any(validCruiseLatData)
            plot(timeLog(validCruiseLatData), CruiseLatCostLog(validCruiseLatData, col), '-', 'Color', 'r', 'LineWidth', 1.5);
        end
        if any(validFollowLatData)
            plot(timeLog(validFollowLatData), FollowingLatCostLog(validFollowLatData, col), '--', 'Color', 'b', 'LineWidth', 1.5);
        end
        if any(validChangeLatData)
            plot(timeLog(validChangeLatData), ChangeLatCostLog(validChangeLatData, col), '--', 'Color', 'g', 'LineWidth', 1.5);
        end
    end

    xlabel('Time (s)');
    ylabel('LateralCost');
    title('LateralCost Log');
    legend('Cruise Mode', 'Following Mode', 'LaneChange Mode', 'Location', 'northeast');

    xlim([0 max(timeLog)]);
    ylim([-0.1 0.4]);

    set(gca, 'FontName', 'Times New Roman', 'FontSize', 10.5);
    saveas(hFig, figName);
    fprintf('Lateral Cost plot saved to: %s\n', figName);
end

%% ===== 速度差コストログ（図） ===== %%
figName = fullfile(pwd, 'SpeedCost.png');

if ~isempty(timeLog)
    hFig = figure('Name','SpeedCost','NumberTitle','off', "Position",[150, 150, 900, 300]);
    clf(hFig); hold on; grid on;

    for col = 1:size(CruiseSpeedCostLog, 2) 
        validCruiseSpeedData = ~isnan(CruiseSpeedCostLog(:, col));  
        validFollowSpeedData = ~isnan(FollowingSpeedCostLog(:, col));  
        validChangeSpeedData = ~isnan(ChangeSpeedCostLog(:, col));  

        if any(validCruiseLatData)
            plot(timeLog(validCruiseSpeedData), CruiseSpeedCostLog(validCruiseSpeedData, col), '-', 'Color', 'r', 'LineWidth', 1.5);
        end
        if any(validFollowLatData)
            plot(timeLog(validFollowSpeedData), FollowingSpeedCostLog(validFollowSpeedData, col), '--', 'Color', 'b', 'LineWidth', 1.5);
        end
        if any(validChangeLatData)
            plot(timeLog(validChangeSpeedData), ChangeSpeedCostLog(validChangeSpeedData, col), '--', 'Color', 'g', 'LineWidth', 1.5);
        end
    end

    xlabel('Time (s)');
    ylabel('SpeedCost');
    title('SpeedCost Log');
    legend('Cruise Mode', 'Following Mode', 'LaneChange Mode', 'Location', 'northeast');

    xlim([0 max(timeLog)]);
    ylim([-0.1 20]);

    set(gca, 'FontName', 'Times New Roman', 'FontSize', 10.5);
    saveas(hFig, figName);
    fprintf('Speed Cost plot saved to: %s\n', figName);
end

%% ===== 時間差コストログ（図） ===== %%
figName = fullfile(pwd, 'TimeCost.png');

if ~isempty(timeLog)
    hFig = figure('Name','TimeCost','NumberTitle','off', "Position",[200, 200, 900, 300]);
    clf(hFig); hold on; grid on;

    for col = 1:size(CruiseTimeCostLog, 2)  
        validCruiseTimeData = ~isnan(CruiseTimeCostLog(:, col)); 
        validFollowTimeData = ~isnan(FollowingTimeCostLog(:, col));  
        validChangeTimeData = ~isnan(ChangeTimeCostLog(:, col));  

        if any(validCruiseTimeData)
            plot(timeLog(validCruiseTimeData), CruiseTimeCostLog(validCruiseTimeData, col), '-', 'Color', 'r', 'LineWidth', 1.5);
        end
        if any(validFollowTimeData)
            plot(timeLog(validFollowTimeData), FollowingTimeCostLog(validFollowTimeData, col), '--', 'Color', 'b', 'LineWidth', 1.5);
        end
        if any(validChangeTimeData)
            plot(timeLog(validChangeTimeData), ChangeTimeCostLog(validChangeTimeData, col), '--', 'Color', 'g', 'LineWidth', 1.5);
        end
    end

    xlabel('Time (s)');
    ylabel('TimeCost');
    title('TimeCost Log');
    legend({'Cruise Mode', 'Following Mode', 'LaneChange Mode'}, 'Location', 'northeast');

    xlim([0 max(timeLog)]);
    ylim([0 40]);

    set(gca, 'FontName', 'Times New Roman', 'FontSize', 10.5);
    saveas(hFig, figName);
    fprintf('Time Cost plot saved to: %s\n', figName);
end

%% ===== 全コストログ（図） ===== %%
figName = fullfile(pwd, 'TotalCost.png');

if ~isempty(timeLog)
    hFig = figure('Name','TotalCost','NumberTitle','off', "Position",[250, 250, 900, 300]);
    clf(hFig); hold on; grid on;

    for col = 1:size(CruiseTotalCostLog, 2)  
        validCruiseTotalData = ~isnan(CruiseTotalCostLog(:, col));  
        validFollowTotalData = ~isnan(FollowingTotalCostLog(:, col)); 
        validChangeTotalData = ~isnan(ChangeTotalCostLog(:, col));  

        if any(validCruiseTotalData)
            plot(timeLog(validCruiseTotalData), CruiseTotalCostLog(validCruiseTotalData, col), '--', 'Color', 'r', 'LineWidth', 1.5);
        end
        if any(validFollowTotalData)
            plot(timeLog(validFollowTotalData), FollowingTotalCostLog(validFollowTotalData, col), '--', 'Color', 'b', 'LineWidth', 1.5);
        end
        if any(validChangeTotalData)
            plot(timeLog(validChangeTotalData), ChangeTotalCostLog(validChangeTotalData, col), '--', 'Color', 'g', 'LineWidth', 1.5);
        end
    end

    xlabel('Time (s)');
    ylabel('TotalCost');
    title('TotalCost Log');
    legend('Cruise Mode', 'Following Mode', 'LaneChange Mode', 'Location', 'northeast');

    xlim([0 max(timeLog)]);
    ylim([0 50]);

    set(gca, 'FontName', 'Times New Roman', 'FontSize', 10.5);
    saveas(hFig, figName);
    fprintf('Total Cost plot saved to: %s\n', figName);
end
%% ===== 車間距離ログ（図） ===== %%
figName = fullfile(pwd, 'gap_plot.png');

if ~isempty(timeLog)
    hFig = figure('Name','Ego to lead gaps','NumberTitle','off', "Position",[300, 300, 900, 300]);
    clf(hFig); hold on; grid on;

    plot(timeLog, gapForwardLog, '-', 'Color', 'b', 'LineWidth', 4);
    plot(timeLog, gapBehindLog,  '-', 'Color', 'r', 'LineWidth', 4);
    yline(forwardsafetyGap, '--','Color', 'b', 'LineWidth', 2);
    yline(behindsafetyGap,  '--','Color', 'r','LineWidth', 2);

    xlabel('Time (s)');
    ylabel('Gap (m)');
    title('Vehicle to Vehicle Gaps');
    legend('Forward Gap','Behind Gap','Forward Safety Gap','Behind Safety Gap','Location','northwest');

    xlim([0 max(timeLog)]);
    ylim([0 60]);

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
    ylim([14 20]);
    
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
    ylim([0 40]);
     
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
    ylim([0 25]);

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
