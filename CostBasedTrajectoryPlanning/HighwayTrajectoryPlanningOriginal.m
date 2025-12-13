clc; clear; close all;
addpath('../Scenario');
% scenario = SDpaper_scenario1;
% scenario = scenario1;
% scenario = scenario_20251113;
% scenario = scenario_20251113_1;
% scenario = scenario_To_Check_Whether_Ego_Car_Can_Change_To_Right_Lane;
% scenario = scenario_20251125;
% scenario = scenario_20251201;
% scenario = scenario_20251204;
scenario = scenario_20251212;

% デフォルトの車両プロパティ
carLen   = 4.7; % [m] 一般的な乗用車の全長
carWidth = 1.8; % [m] 一般的な乗用車の全幅
rearAxleRatio = 0.25; % 全長 4.7 m なら、後軸は後端から 1.175 m (4.7 × 0.25) の位置にある、という意味。

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

%% パラメータ定義
% 10 Hz = 0.1秒ごとに軌道を再計算。
replanRate = 10; % Hz

% 現在の状態と計画状態の間隔を定義する。
timeHorizons = 3:0.5:5; % [sec] 
maxHorizon = max(timeHorizons); % [sec] 

% コストパラメータの定義
latDevWeight    =  1000; % 横方向ずれ（車線中央からの逸脱）
timeWeight      =  0.001; % 短い時間を好む
speedWeight     =  0.1; % 目標速度からの逸脱

% 以下の制約に違反する軌道を排除する。
maxAcceleration = 3; % [m/s^2] 車両が出せる最大加速度を超えたら不適
maxCurvature    = 0.02; % [1/m], or [rad/m] 最小旋回半径 ≈ 1m
minVelocity     = 0; % [m/sec] 負の速度（バック）や停止を避ける

% クルーズコントロール動作および軌道コスト評価時に使用される目標速度設定値。
speedLimit = 18; % [m/sec] 巡航時の目標速度。コスト評価の基準値

% プランナーが追従動作において目標とする最小距離。
forwardsafetyGap = 16; % [m] 前車との最低距離。これを下回る軌道は不適切として棄却
behindsafetyGap = 12;



%% 初期化
[scenarioViewer,futureTrajectory,actorID,actorPoses,egoID,egoPoses,stepPerUpdate,EgoGlobalState,isRunning,lineHandles] = InitializeSimulator(scenario,capList,refPath,laneWidth,replanRate,carLen);

%% ログボックス
% --- prepare logging for longitudinal gap to the nearest forward vehicle ---
numActors = numel(actorID);
gapForwardLog = [];        % rows: timesteps, single column: nearest lead gap (m)
gapBehindLog = [];  % rows: timesteps, single column: nearest behind gap (m)
leadIdxLog = [];    % store which actor was the nearest lead (optional)
behindIdxLog = [];  % store which actor was the nearest behind (optional)
timeLog = [];
speedLog = [];      % rows: timesteps, single column: ego speed (m/s)
simStep = -0;

% --- モードログ用初期化ボックス ---
modeTimeLog  = [];              % Nx1 時刻[s]
modeLabelLog = strings(0,1);    % Nx1 選定モード（"Cruise","Follow","LaneChange",または "" ）
modeBinLog   = zeros(0,3);      % Nx3 [Cruise, Follow, LaneChange] を 0/1 で one-hot
labelsLog    = {};              % 各ステップの候補ラベル配列（可変長なので cell）
modeColLog   = {};              % 各ステップの ✓/"" ベクトル（可変長なので cell）
yawLog = [];  % [rad] 自車両のヨー角ログ


%% === 逐次軌道計算ループ=== %%
tic 

% --- 動画出力設定 ---
videoFile = fullfile(pwd, 'simulation_output.mp4'); % 出力ファイル名
videoWriter = VideoWriter(videoFile, 'MPEG-4');
videoWriter.FrameRate = 10; % 10fps（replanRateに合わせるのも可）
open(videoWriter);


while isRunning
    %% while文開始
    simStep = simStep + 1;
    
    %% アクター車両の現在の状態と計画ホライズンにわたる軌跡を取得する。
    [AllActorGlobalState,futureTrajectory,isRunning] = ...
        RetrieveActorGroundTruth(scenario,futureTrajectory,replanRate,maxHorizon);
    


    %% 終端状態の生成
    % 巡航制御の状態を生成
    [termStatesCC,timesCC] = TSCruiseControl(...
        refPath,laneWidth,EgoGlobalState,speedLimit,timeHorizons);
  
    % 前方車追従の状態を生成
    [termStatesF,timesF] = TSLeadVehicleFollow(...
        refPath,laneWidth,forwardsafetyGap,EgoGlobalState,AllActorGlobalState,carLen, rearAxleRatio, timeHorizons);
     
      % 車線変更の状態を生成
    % [termStatesLC,timesLC] = TSLaneChange(...
    %     refPath,laneWidth,EgoGlobalState, timeHorizons);

    [termStatesLC,timesLC] = TSLaneChange(...
    refPath, laneWidth, forwardsafetyGap, behindsafetyGap, ...
    EgoGlobalState, AllActorGlobalState, carLen, rearAxleRatio, timeHorizons);
    



    % 
    % disp('CC');
    % disp(termStatesCC);
    % disp('LC');
    % disp(termStatesLC);
    % disp('F');
    % disp(termStatesF);
    %上記のdispで確認した限り，生成された終端状態に問題は無し


    %% 終端状態をまとめる
     labels = [repmat("Cruise", size(termStatesCC,1), 1);
                repmat("Follow", size(termStatesF,1), 1);
                repmat("LaneChange", size(termStatesLC,1), 1)
               ];

    % disp(labels);
    allTime = [timesCC; timesLC; timesF];
    % disp(allTime);

     %% 終端状態と時間を組み合わせる。
    allTS = [termStatesCC; termStatesF; termStatesLC];
    allDT = [timesCC; timesF; timesLC];
    numTS = [numel(timesCC); numel(timesF); numel(timesLC)];
    
   
    % disp('allTS');
    % disp(allTS);s
    % disp('allDT');
    % disp(allDT);
    % disp('numTS');
    % disp(numTS);
    % 上記の終端状態と時間の組み合わせもOK
   

    %% すべての終端状態のコストを評価する。
    [costTS,latCostTS,timeCostTS,speedCostTS] = EvaluateTSCost(allTS,allDT,laneWidth,speedLimit,...
        speedWeight, latDevWeight, timeWeight);


    %% 軌道を生成
    EgoFrenetState = global2frenet(refPath,EgoGlobalState);
    [frenetTraj,globalTraj] = connect(connector,EgoFrenetState,allTS,allDT);
    
    % disp('egoFrenetState');
    % disp(EgoFrenetState);
    % disp('frenetTraj');
    % disp(frenetTraj);
    % disp('globalTraj');
    % disp(globalTraj);


    %%  制約条件に違反する軌道を排除 
    isValid = EvaluateTrajectory(globalTraj,maxAcceleration,maxCurvature,minVelocity);
    disp("Times " + num2str(simStep/10));
    a = table(costTS, latCostTS, timeCostTS, speedCostTS, isValid, 'VariableNames', {'TotalCost','LatCost','TimeCost','SpeedCost','制約違反'});
    disp(a);


    %% --- シーン内の全アクターの予測軌道を衝突チェッカーに反映 --- %%
    for i = 1:numel(actorPoses)
        actorPoses(i).States = futureTrajectory(i).Trajectory(:,1:3);
    end
    updateObstaclePose(capList,actorID,actorPoses);

    %% --- 同一レーンにいる最も近い前方車両との車間距離 --- %%
    try
        EgoFrenetFront = EgoFrenetState(1) + carLen*(1-rearAxleRatio);
        EgoFrenetRear  = EgoFrenetState(1) - carLen*rearAxleRatio;
        
        AllActorFrenetState = global2frenet(refPath,AllActorGlobalState);
      
        AllActorFrenetFront = AllActorFrenetState(:,1) + carLen*(1-rearAxleRatio);
        AllActorFrenetRear  = AllActorFrenetState(:,1) - carLen*rearAxleRatio;

        EgoLaneNum = PredictLane(EgoFrenetState, laneWidth, 1/replanRate);
        disp('REgoFrenetState');
        disp(EgoFrenetState);
        disp('AllActorFrenetState');
        disp(AllActorFrenetState);



        % % --- 同一車線，前方車との距離
        FrontGaps = nan(1, numActors);
        for a = 1:numActors
            AllActorLaneNum = PredictLane(AllActorFrenetState(a,:), laneWidth, 1/replanRate);
            if AllActorLaneNum == EgoLaneNum && AllActorFrenetRear(a,1) > EgoFrenetFront
                FrontGaps(a) = AllActorFrenetRear(a,1) - EgoFrenetFront;
            end
        end

        % 最も近い車両の選定
        validIdx = find(~isnan(FrontGaps)); % ~isnanは論理否定演算子，NaN出ない要素番号を返す
        if ~isempty(validIdx)
            [minGap, relIdx] = min(FrontGaps(validIdx));
            forwardactorIdx = validIdx(relIdx);
        else
            minGap = nan;
            forwardactorIdx = nan;
        end




        % % --- 同一車線，後方車両との距離
        RearGaps = nan(1,numActors);
        for a = 1:numActors
            AllActorLaneNum = PredictLane(AllActorFrenetState(a,:), laneWidth, 1/replanRate);
            if AllActorLaneNum == EgoLaneNum && AllActorFrenetFront(a,1) < EgoFrenetRear
                RearGaps(a) = EgoFrenetRear - AllActorFrenetFront(a,1);
            end
        end

        % 最も近い車両の選定
        validRear = find(~isnan(RearGaps));
        if ~isempty(validRear)
            [minRearGap, relRearIdx] = min(RearGaps(validRear));
            behindActorIdx = validRear(relRearIdx);
        else
            minRearGap = nan;
            behindActorIdx = nan;
        end


        
        gapForwardLog(end+1,1) = minGap;           %#ok<SAGROW>
        gapBehindLog(end+1,1) = minRearGap;             %#ok<SAGROW>
        leadIdxLog(end+1,1) = forwardactorIdx;          %#ok<SAGROW>
        behindIdxLog(end+1,1) = behindActorIdx;         %#ok<SAGROW>
        timeLog(end+1,1) = (simStep-1)*scenario.SampleTime;  %#ok<SAGROW>
        % Log ego speed (5th element of egoState is speed in this representation)
        if numel(EgoGlobalState) >= 5
            speedLog(end+1,1) = EgoGlobalState(5); %#ok<SAGROW>

        else
            speedLog(end+1,1) = nan; %#ok<SAGROW>
        end
    catch ME
        % If frenet conversion fails for any reason, skip logging this step
        warning('Gap logging skipped at step %d: %s', simStep, ME.message); 
        gapForwardLog(end+1,1) = nan; %#ok<SAGROW>
        leadIdxLog(end+1,1) = nan; %#ok<SAGROW>
        timeLog(end+1,1) = (simStep-1)*scenario.SampleTime; %#ok<SAGROW>
        speedLog(end+1,1) = nan; %#ok<SAGROW>
    end

%% --- 車間距離も考慮した有効軌道選定 --- （ルールベースの部分）%%
if minGap <= forwardsafetyGap
    % forwardsafetyGapよりminGapが小さい場合，
    % isValidの最初の5要素をFalseにする
    isValid(1:min(5, numel(isValid))) = false;
end


%% --- 評価順序を決定 --- %%
[cost, idx] = sort(costTS);
optimalTrajectory = [];

trajectoryEvaluation = nan(numel(isValid),1); % numelは要素数を返す


% 最小コストから順に、各軌道の衝突をチェックする。
for i = 1:numel(idx)
    if isValid(idx(i))
        % エゴオブジェクトの候補軌道でカプセルリストを更新する。
        egoPoses.States = globalTraj(idx(i)).Trajectory(:,1:3);
        updateEgoPose(capList,egoID,egoPoses);

        % 衝突をチェックする。
        isColliding = checkCollision(capList);

        if all(~isColliding)
            % 衝突が検出されない場合、これが最適軌道である。
            trajectoryEvaluation(idx(i)) = 1;
            optimalTrajectory = globalTraj(idx(i)).Trajectory;
            % disp('Frenet系での自車両の状態'); % 0s後のものになっている
            % disp(EgoFrenetState);
            % disp(gapForwardLog);
            disp('自車のフロント'); % 0s後のものになってる
            disp(EgoFrenetFront);
            disp('他車のリア'); % 0.1s後のものになってる
            disp(AllActorFrenetRear);
            disp(['前方車間距離: ' num2str(minGap) ' m']); % 0sと0.1sのミックス
            % disp('curA'); % 0.1s後のものになってる
            % disp(AllActorFrenetState);
            disp('有効性');
            disp(isValid);
            break;
        else
            trajectoryEvaluation(idx(i)) = 0;
        end
    end
end
 
%% === 選定モードを決定し、ログ化 === %%
% （Tdisp を作る前に modeCol を確定させる）
selectedIdx = find(trajectoryEvaluation==1, 1);  % 最初に有効と判定された候補

selLabel = "";           % 既定値（未選定）
onehot   = [0 0 0];      % [Cruise, Follow, LaneChange]

if ~isempty(selectedIdx)
    selLabel = labels(selectedIdx);  % "Cruise" / "Follow" / "LaneChange"

    % one-hot 化（順序は [Cruise, Follow, LaneChange] で固定）
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

% --- 時刻歴ログに追記 ---
modeTimeLog(end+1,1)  = (simStep-1)*scenario.SampleTime; %#ok<SAGROW>
modeLabelLog(end+1,1) = selLabel;                        %#ok<SAGROW>
modeBinLog(end+1,:)   = onehot;                          %#ok<SAGROW>

%% --- サンプリングされた軌跡を表示--- %%
    lineHandles = VisualizeScene(lineHandles,globalTraj,isValid,trajectoryEvaluation);


    hold on;
    show(capList,'TimeStep',1:capList.MaxNumSteps,'FastUpdate',1);
    hold off;
    
    if isempty(optimalTrajectory)
            error('No valid trajectory has been found.');
    else
        % 再計画の間の場面を視覚化する。
        for i = (2+(0:(stepPerUpdate-1)))
            % おおよそのリアルタイム可視化。
            dt = toc;
            if scenario.SampleTime-dt > 0
                pause(scenario.SampleTime-dt);
            end
            
            EgoGlobalState = optimalTrajectory(i,:);
            scenarioViewer.Actors(1).Position(1:2) = EgoGlobalState(1:2);
            scenarioViewer.Actors(1).Velocity(1:2) = [cos(EgoGlobalState(3)) sin(EgoGlobalState(3))]*EgoGlobalState(5);
            scenarioViewer.Actors(1).Yaw = EgoGlobalState(3)*180/pi;
            
            % --- ヨー角ログを記録（ラジアン単位） ---
            yawLog(end+1,1) = EgoGlobalState(3)*180/pi;  %#ok<SAGROW>

            scenarioViewer.Actors(1).AngularVelocity(3) = EgoGlobalState(4)*EgoGlobalState(5);
            
            % カプセルの可視化を更新
            hold on;
            show(capList,'TimeStep',i:capList.MaxNumSteps,'FastUpdate',1);
            hold off;
            
            % ドライビングシナリオの更新
            advance(scenarioViewer);

            % --- 現在のフレームをキャプチャして動画に追加 ---
            frame = getframe(gcf);
            writeVideo(videoWriter, frame);

            tic;
        end
    end
end

%% --- 動画ファイルを閉じる ---
close(videoWriter);
fprintf('Simulation video saved to: %s\n', videoFile);

%% --- 車間距離のログ ---%%
% Always attempt to produce and save the gap plot. If no data was logged, show a message.
figName = fullfile(pwd, 'gap_plot.png');
if ~isempty(timeLog)
    hFig = figure('Name','Ego to lead gaps','NumberTitle','off'); clf(hFig); hold on; grid on;
    % Plot nearest lead and nearest behind gaps as continuous lines
    hPlotLead = plot(timeLog, gapForwardLog, '-', 'Color', 'b', 'LineWidth', 3);
    hold on;
    hPlotBehind = plot(timeLog, gapBehindLog, '-', 'Color', 'r', 'LineWidth', 3); 
    
    % 前後のsafetyGapの表示
    hYlineLead = yline(forwardsafetyGap, 'b--');
    hYlineLead.Label = '';
    hYlineLead.LabelHorizontalAlignment = 'right';
    hYlineLead.LabelVerticalAlignment = 'bottom';
    hYlineLead.FontName = 'Times New Roman';
    hYlineLead.FontSize = 14;

    hYlineBehind = yline(behindsafetyGap, 'r--');
    hYlineBehind.LabelHorizontalAlignment = 'right';
    hYlineBehind.LabelVerticalAlignment = 'top';
    hYlineBehind.FontName = 'Times New Roman';
    hYlineBehind.FontSize = 14;

    % Set labels and title with Times New Roman
    hX = xlabel('Time (s)');
    hY = ylabel('Gap (m)');
    hT = title('Vehicle to Vehicle Gaps');
    % Legend: show lead/behind gap lines only
    hLeg = legend([hPlotLead hPlotBehind hYlineLead hYlineBehind], ...
    {'Forward Gap','Behind Gap','Forward Safety Gap', 'Behind Safety Gap'}, ...
    'Location','northeast');
    set([hX,hY,hT],'FontName','Times New Roman','FontSize',14);
    set(hLeg,'FontName','Times New Roman','FontSize',14);
    set(gca,'FontName','Times New Roman','FontSize',14);
    hold off;
    drawnow;
    try
        saveas(hFig, figName);
        fprintf('Gap plot saved to: %s\n', figName);
    catch
        warning('Failed to save gap plot to %s', figName);
    end
else
    % No log data collected — create an empty figure with message and save it
    hFig = figure('Name','Ego to lead gaps (no data)','NumberTitle','off'); clf(hFig);
        tHandle = text(0.1,0.5,'No gap data was logged during the simulation.','FontSize',12,'FontName','Times New Roman');
    axis off;
    try
        saveas(hFig, figName);
        fprintf('No-gap plot saved to: %s\n', figName);
    catch
        warning('Failed to save no-gap plot to %s', figName);
    end
end

 
%% --- 速度ログ--- %%
speedFigName = fullfile(pwd, 'speed_plot.png');
if ~isempty(timeLog)
    hFig2 = figure('Name','Ego speed','NumberTitle','off'); clf(hFig2); grid on; hold on;
    % Plot ego speed vs time as continuous line
    hSpeed = plot(timeLog, speedLog, '-', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2);
    % Labels and title
    hX2 = xlabel('Time (s)');
    hY2 = ylabel('Speed (m/s)');
    hT2 = title('Ego vehicle speed');
    hLeg2 = legend(hSpeed, {'Ego speed'}, 'Location','best');
    set([hX2,hY2,hT2],'FontName','Times New Roman','FontSize',9);
    set(hLeg2,'FontName','Times New Roman','FontSize',14);
    set(gca,'FontName','Times New Roman','FontSize',14);
    % 表示範囲: 速度を 15 m/s から 20 m/s の範囲で表示
    try
        ylim([0 25]);
        yticks(0:1:25);
    catch
        % 一部の環境で ylim 設定が失敗する場合は無視して続行
    end
    hold off;
    drawnow;
    try
        saveas(hFig2, speedFigName);
        fprintf('Speed plot saved to: %s\n', speedFigName);
    catch
        warning('Failed to save speed plot to %s', speedFigName);
    end
else
    hFig2 = figure('Name','Ego speed (no data)','NumberTitle','off'); clf(hFig2);
    text(0.1,0.5,'No speed data was logged during the simulation.','FontSize',14,'FontName','Times New Roman');
    axis off;
    try
        saveas(hFig2, speedFigName);
        fprintf('No-speed plot saved to: %s\n', speedFigName);
    catch
        warning('Failed to save no-speed plot to %s', speedFigName);
    end
end



%% --- ヨー角ログ --- %%
yawFigName = fullfile(pwd, 'yaw_plot.png');
if ~isempty(timeLog) && ~isempty(yawLog)
    % unwrap: ±πを跨ぐジャンプを補正して連続化
    yawLog = unwrap(yawLog);

    % グラフを作成
    hFig3 = figure('Name','Ego yaw angle','NumberTitle','off');
    clf(hFig3); grid on; hold on;

    % Plot yaw vs time
    hYaw = plot(timeLog, yawLog, '-', 'Color', [0.3010 0.7450 0.9330], 'LineWidth', 2);

    % Labels and title
    hX3 = xlabel('Time (s)');
    hY3 = ylabel('Yaw angle (rad)');
    hT3 = title('Ego vehicle yaw angle');
    hLeg3 = legend(hYaw, {'Yaw angle'}, 'Location','best');

    % Times New Roman フォント設定（9pt）
    set([hX3,hY3,hT3],'FontName','Times New Roman','FontSize',14);
    set(hLeg3,'FontName','Times New Roman','FontSize',14);
    set(gca,'FontName','Times New Roman','FontSize',14);

    % 表示範囲
    try
        ylim auto; % 必要に応じて調整可
    catch
    end

    hold off;
    drawnow;

    % ファイル保存
    try
        saveas(hFig3, yawFigName);
        fprintf('Yaw plot saved to: %s\n', yawFigName);
    catch
        warning('Failed to save yaw plot to %s', yawFigName);
    end
else
    % データが空の場合
    hFig3 = figure('Name','Ego yaw angle (no data)','NumberTitle','off');
    clf(hFig3);
    text(0.1,0.5,'No yaw data was logged during the simulation.', ...
        'FontSize',12,'FontName','Times New Roman');
    axis off;
    try
        saveas(hFig3, yawFigName);
        fprintf('No-yaw plot saved to: %s\n', yawFigName);
    catch
        warning('Failed to save no-yaw plot to %s', yawFigName);
    end
end






%% --- 選定モード（Cruise/Follow/LaneChange）0/1 時系列プロット --- %%
modeFigName = fullfile(pwd, 'mode_plot.png');

if ~isempty(modeTimeLog)
    hFig4 = figure('Name','Selected Mode','NumberTitle','off'); 
    clf(hFig4); grid on; hold on;

    % 0/1 の階段状に見せたいので stairs を使用
    hCruise     = stairs(modeTimeLog, modeBinLog(:,1), 'Color', 'r', 'LineStyle', ':','LineWidth', 1.5);
    hFollow     = stairs(modeTimeLog, modeBinLog(:,2), 'Color', 'b', 'LineStyle', ':','LineWidth', 1.5);
    hLaneChange = stairs(modeTimeLog, modeBinLog(:,3), 'Color', [0 0.4 0], 'LineStyle', ':', 'LineWidth', 1.5);
  

    xlabel('Time (s)');
    ylabel('Mode');
    title('Selected Mode');
    legend([hCruise, hFollow, hLaneChange], {'Cruise','Follow','LaneChange'}, 'Location','best');
    ylim([-0.1 3.1]);

    set(gca,'FontName','Times New Roman','FontSize',14);
    saveas(hFig4, modeFigName);
    fprintf('Mode plot saved to: %s\n', modeFigName);
    




%% ==== 車間距離（Gap） ====
if ~isempty(timeLog)
    v = VideoWriter('gap_bar.mp4', 'MPEG-4');
    v.FrameRate = 10; open(v);
    fig = figure('Name','Gap with Red Bar','NumberTitle','off'); 
    clf(fig); hold on; grid on;

    % --- メインプロット ---
    hPlotLead   = plot(timeLog, gapForwardLog, '-', 'Color', 'b', 'LineWidth', 3, 'DisplayName','Forward Gap');
    hPlotBehind = plot(timeLog, gapBehindLog,  '-', 'Color', 'r', 'LineWidth', 3, 'DisplayName','Behind Gap');

    % --- 安全車間距離ライン ---
    hYlineLead   = yline(forwardsafetyGap, 'b--', 'LineWidth', 2, 'DisplayName', 'Forward Safety Gap');
    hYlineBehind = yline(behindsafetyGap, 'r--', 'LineWidth', 2, 'DisplayName', 'Behind Safety Gap');

    % --- 軸設定 ---
    hX = xlabel('Time (s)');
    hY = ylabel('Gap (m)');
    hT = title('Vehicle to Vehicle Gaps');
    ylim([0 40]); % 縦軸を40までに設定
    xlim([min(timeLog) max(timeLog)]); % 横軸もデータ範囲に合わせて調整

    % --- 凡例を図内右上に配置 ---
    hLeg = legend([hPlotLead hPlotBehind hYlineLead hYlineBehind], ...
        {'Forward Gap','Behind Gap','Forward Safety Gap', 'Behind Safety Gap'}, ...
        'Location','northeast', 'Box','on');

    % --- フォント設定 ---
    set([hX,hY,hT],'FontName','Times New Roman','FontSize',14);
    set(hLeg,'FontName','Times New Roman','FontSize',12);
    set(gca,'FontName','Times New Roman','FontSize',14);

    % --- 赤バー ---
    barLine = xline(timeLog(1), 'r-', 'LineWidth', 2);

    % --- アニメーション ---
    timeFrames = timeLog;
    for i = 1:length(timeFrames)
        set(barLine, 'Value', timeFrames(i));
        title(sprintf('Vehicle to Vehicle Gaps (t = %.2f s)', timeFrames(i)), ...
              'FontName','Times New Roman','FontSize',14);
        frame = getframe(fig); writeVideo(v, frame);
    end
    close(v);
    disp('✅ Gap + red bar video saved.');
end



%% ==== 速度（Speed） ====
if ~isempty(timeLog) && ~isempty(speedLog)
    v = VideoWriter('speed_bar.mp4', 'MPEG-4');
    v.FrameRate = 10; open(v);
    fig = figure('Name','Speed with Red Bar','NumberTitle','off'); clf(fig); hold on; grid on;

    plot(timeLog, speedLog, '-', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2, 'DisplayName', 'Ego Speed');
    xlabel('Time (s)'); ylabel('Speed (m/s)');
    title('Ego Vehicle Speed');
    ylim([0 25]);
    set(gca,'FontName','Times New Roman','FontSize',14);
    set(findall(gca,'Type','text'),'FontName','Times New Roman');
    barLine = xline(timeLog(1), 'r-', 'LineWidth', 2);
    legend({'Ego Speed'},'Location','northeast','FontName','Times New Roman','FontSize',14);

    timeFrames = timeLog;
    for i = 1:length(timeFrames)
        set(barLine, 'Value', timeFrames(i));
        title(sprintf('Ego Speed (t = %.2f s)', timeFrames(i)), 'FontName','Times New Roman','FontSize',14);
        frame = getframe(fig); writeVideo(v, frame);
    end
    close(v);
    disp('✅ Speed + red bar video saved.');
end





%% ==== ヨー角（Yaw） ====
if ~isempty(timeLog) && ~isempty(yawLog)
    v = VideoWriter('yaw_bar.mp4', 'MPEG-4');
    v.FrameRate = 10; open(v);
    fig = figure('Name','Yaw with Red Bar','NumberTitle','off');
    clf(fig); hold on; grid on;

    % --- メインプロット ---
    plot(timeLog, yawLog, '-', 'Color', [0.3010 0.7450 0.9330], ...
        'LineWidth', 2, 'DisplayName', 'Yaw Angle');

    % --- 軸設定 ---
    xlabel('Time (s)');
    ylabel('Yaw Angle (rad)');
    title('Ego Vehicle Yaw Angle');
    ylim([-5 5]); % ← 縦軸範囲を±5に設定
    xlim([min(timeLog) max(timeLog)]); % ← 横軸をデータ範囲に設定

    set(gca,'FontName','Times New Roman','FontSize',14);
    set(findall(gca,'Type','text'),'FontName','Times New Roman');

    % --- 赤バー（初期位置） ---
    barLine = xline(timeLog(1), 'r-', 'LineWidth', 2);

    % --- 凡例（Yaw Angle のみ） ---
    legend({'Yaw Angle'},'Location','northeast', ...
        'FontName','Times New Roman','FontSize',14);

    % --- アニメーション ---
    timeFrames = timeLog;
    for i = 1:length(timeFrames)
        set(barLine, 'Value', timeFrames(i));
        title(sprintf('Yaw Angle (t = %.2f s)', timeFrames(i)), ...
            'FontName','Times New Roman','FontSize',14);
        frame = getframe(fig);
        writeVideo(v, frame);
    end

    % --- 終尺の位置を最後の時刻に固定 ---
    set(barLine, 'Value', timeFrames(end));

    close(v);
    disp('✅ Yaw + red bar video saved.');
end

%% ==== 走行モード（Mode） ====
% if ~isempty(modeTimeLog)
%     v = VideoWriter('mode_bar.mp4', 'MPEG-4');
%     v.FrameRate = 10; open(v);
%     fig = figure('Name','Mode with Red Bar','NumberTitle','off'); clf(fig); hold on; grid on;
% 
%     % --- プロット ---
%     hCruise = stairs(modeTimeLog, modeBinLog(:,1), 'r:', 'LineWidth', 2, 'DisplayName', 'Cruise');
%     hFollow = stairs(modeTimeLog, modeBinLog(:,2), 'b:', 'LineWidth', 2, 'DisplayName', 'Follow');
%     hLane   = stairs(modeTimeLog, modeBinLog(:,3), 'Color', [0 0.4 0], 'LineStyle', ':', 'LineWidth', 2, 'DisplayName', 'LaneChange');
% 
% 
%     % --- 軸・凡例・フォント設定 ---
%     xlabel('Time (s)'); ylabel('Mode');
%     title('Mode Transition Over Time');
%     legend([hCruise, hFollow, hLane], {'Cruise','Follow','LaneChange'}, ...
%         'Location','northeast', 'FontName','Times New Roman','FontSize',14);
%     axis([min(modeTimeLog) max(modeTimeLog) -0.1 3.1]);
%     set(gca,'FontName','Times New Roman','FontSize',14);
%     set(findall(gca,'Type','text'),'FontName','Times New Roman');
% 
%     % --- 赤バー ---
%     barLine = xline(modeTimeLog(1), 'r-', 'LineWidth', 2);
% 
%     % --- アニメーション ---
%     timeFrames = timeLog;
%     for i = 1:length(timeFrames)
%         set(barLine, 'Value', timeFrames(i));
%         title(sprintf('Mode Transition (t = %.2f s)', timeFrames(i)), 'FontName','Times New Roman','FontSize',14);
%         frame = getframe(fig); writeVideo(v, frame);
%     end
%     close(v);
%     disp('✅ Mode + red bar video saved.');
% end

%% ==== 走行モード（Mode） ====
if ~isempty(modeTimeLog)
    v = VideoWriter('mode_bar.mp4', 'MPEG-4');
    v.FrameRate = 10; open(v);
    fig = figure('Name','Mode with Red Bar','NumberTitle','off'); 
    clf(fig); hold on; grid on;

    % --- プロット ---
    hCruise = stairs(modeTimeLog, modeBinLog(:,1), 'r:', 'LineWidth', 2);
    hFollow = stairs(modeTimeLog, modeBinLog(:,2), 'b:', 'LineWidth', 2);
    hLane   = stairs(modeTimeLog, modeBinLog(:,3), 'Color', [0 0.4 0], ...
                     'LineStyle', ':', 'LineWidth', 2);

    % --- 軸・凡例・フォント設定 ---
    xlabel('Time (s)');
    ylabel('Mode');
    title('Mode Transition Over Time');
    ylim([0 4]); % 縦軸の範囲を0〜4に設定
    xlim([min(modeTimeLog) max(modeTimeLog)]); % 横軸をデータ範囲に設定

    % --- 凡例（data1などを排除して手動指定） ---
    legend([hCruise, hFollow, hLane], ...
        {'Cruise','Follow','LaneChange'}, ...
        'Location','northeast', 'Box','on', ...
        'FontName','Times New Roman','FontSize',14);

    set(gca,'FontName','Times New Roman','FontSize',14);
    set(findall(gca,'Type','text'),'FontName','Times New Roman');

    % --- 赤バー ---
    barLine = xline(modeTimeLog(1), 'r-', 'LineWidth', 2);

    % --- アニメーション ---
    timeFrames = modeTimeLog;
    for i = 1:length(timeFrames)
        set(barLine, 'Value', timeFrames(i));
        title(sprintf('Mode Transition (t = %.2f s)', timeFrames(i)), ...
              'FontName','Times New Roman','FontSize',14);
        frame = getframe(fig);
        writeVideo(v, frame);
    end

    close(v);
    disp('✅ Mode + red bar video saved.');
end




else
    warning('No mode data was logged.');
end
