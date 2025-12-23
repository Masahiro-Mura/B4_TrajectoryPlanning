clc; clear; close all;
addpath('..\Scenario\m_file\');
scenario = scenario_20251223;

% デフォルトの車両プロパティ
carLen   = 4.7; 
carWidth = 1.8; 
rearAxleRatio = 0.25; 
laneWidth   = carWidth*2;

plot(scenario);
waypoints = [0 0; 800 0]; 
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
LatWeight         =  0.5;  % 横偏差の重み
TimeWeight        =  0.5;  % 時間コストの重み
SpeedWeight       =  1.0;  % 速度差コストの重み
LongSafetyWeight  =  6.0;  % 縦方向安全コストの重み
c1                =  0.05;  % c1 はチューニングパラメータ（0.05〜0.2くらいで調整してみる）

% 以下の制約に違反する軌道を排除
maxAcceleration = 3; 
maxCurvature    = 0.02; 
minVelocity     = 0; 

% 目標速度設定値
SpeedLimit = 18; 

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
simStep = 0;

% --- モードログ用初期化ボックス ---
% modeTimeLog  = [];              % Nx1 時刻[s]
modeLabelLog = strings(0,1);    % Nx1 選定モード（"Cruise","Follow","LaneChange",または "" ）
modeBinLog   = zeros(0,3);      % Nx3 [Cruise, Follow, LaneChange] を 0/1 で one-hot
labelsLog    = {};              % 各ステップの候補ラベル配列（可変長なので cell）
modeColLog   = {};              % 各ステップの ✓/"" ベクトル（可変長なので cell）
yawLog = [];  % [rad] 自車両のヨー角ログ
latAccLog = [];   % 横方向加速度ログ [m/s^2]
prevTS = [];
prevT  = [];


% 横偏差コストログ
CruiseLatCostLog = [];
FollowingLatCostLog = [];
ChangeLatCostLog = [];

% 速度差コストログ
CruiseSpeedCostLog = [];
FollowingSpeedCostLog = [];
ChangeSpeedCostLog = [];

% 時間差コストログ
CruiseTimeCostLog = [];
FollowingTimeCostLog = [];
ChangeTimeCostLog = [];

% 一貫性コストログ
CruiseConsisCostLog = [];
FollowingConsisCostLog = [];
ChangeConsisCostLog = [];

% 全計コストログ
CruiseTotalCostLog = [];
FollowingTotalCostLog = [];
ChangeTotalCostLog = [];

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
    


    %% ===== 軌道計算 ===== %%
    % 巡航制御の状態を生成 ＋ 先行車両との車間距離
    [termStatesCC,timesCC,leadGapsCC] = TSCruiseControl(...
        refPath,laneWidth,EgoGlobalState,SpeedLimit,timeHorizons, AllActorGlobalState);

  
    % 前方車追従の状態を生成（追加出力: 各予測秒数ごとの先行車との車間距離 5x1）
    [termStatesF,timesF,leadGapsF] = TSLeadVehicleFollow(...
        refPath,laneWidth,forwardsafetyGap,EgoGlobalState,AllActorGlobalState,carLen, rearAxleRatio, timeHorizons);


      % 車線変更の状態を生成
    % [termStatesLC,timesLC] = TSLaneChange(...
    % refPath, laneWidth, forwardsafetyGap, behindsafetyGap, ...
    % EgoGlobalState, AllActorGlobalState, carLen, rearAxleRatio, SpeedLimit, timeHorizons);
    [termStatesLC, timesLC, leadGapsLC] = TSLaneChange( ...
    refPath, laneWidth, forwardsafetyGap, behindsafetyGap, ...
    EgoGlobalState, AllActorGlobalState, carLen, rearAxleRatio, SpeedLimit, timeHorizons);
   

    % 終端状態をまとめる
    Labels = [repmat("Cruise", size(termStatesCC,1), 1);
              repmat("Follow", size(termStatesF,1), 1);
              repmat("LaneChange", size(termStatesLC,1), 1)
             ];
    AllTime = [timesCC; timesF; timesLC];
    AllGaps = [leadGapsCC; leadGapsF; leadGapsLC];
   

    % 終端状態と時間を組み合わせる。
    allTS = [termStatesCC; termStatesF; termStatesLC];
    % allDT = [timesCC; timesF; timesLC];
    numTS = [numel(timesCC); numel(timesF); numel(timesLC)];
    
    
    % 軌道を生成
    EgoFrenetState = global2frenet(refPath,EgoGlobalState);
    [frenetTraj,globalTraj] = connect(connector,EgoFrenetState,allTS,AllTime);
    
        % 同一レーンにいる最も近い前方車両との車間距離 
    [EgoFrenetFront, EgoFrenetRear, AllActorFrenetRear, minGap, gapForwardLog, gapBehindLog, timeLog, speedLog] ...
         = ComputeGaps(EgoFrenetState, carLen, rearAxleRatio, refPath, AllActorGlobalState, ...
         laneWidth, numActors, gapForwardLog, gapBehindLog, leadIdxLog, behindIdxLog, timeLog, simStep, scenario, EgoGlobalState, speedLog);
    

    dObs = AllGaps;
    disp('dObs');
    disp(dObs);

    %% ===== 軌道選定 ===== %%

    % 元のコスト関数
    % [costTS,latCostTS,timeCostTS,speedCostTS] = EvaluateTSCost(allTS,allDT,laneWidth,SpeedLimit,...
    %     SpeedWeight, LatWeight, TimeWeight);
    
    % 正規化
    % [costTS,latCostTS,timeCostTS,speedCostTS] = EvaluateTSCostNormalization(allTS,allDT,laneWidth,SpeedLimit,...
    % SpeedWeight, LatWeight, TimeWeight);
    
    % 正規化＋一貫性コスト
    % [costTS, latCostTS, timeCostTS, speedCostTS, consCostTS] ...
    % = EvaluateTSCostNew(...
    % allTS, allDT, laneWidth, SpeedLimit, ...
    % SpeedWeight, LatWeight, TimeWeight, ConsistencyWeight, ...
    % prevTS, prevT, modeBinLog);

    % 正規化 + 縦方向安全コスト
    [costTS, latCostTS, timeCostTS, speedCostTS, longSafeCostTS] = ...
    EvaluateTSCostNormalization1(allTS, AllTime, laneWidth, SpeedLimit, ...
                                SpeedWeight, LatWeight, TimeWeight, ...
                                LongSafetyWeight, dObs, c1, forwardsafetyGap);
   

    %  制約条件に違反する軌道を排除 
    isValid = EvaluateTrajectory(globalTraj,maxAcceleration,maxCurvature,minVelocity); % ここを軌道計画の段階で行いたい
    
    % disp("Times " + num2str(simStep/10));
    a = table(Labels, AllTime, costTS, latCostTS, timeCostTS, speedCostTS, longSafeCostTS, isValid, ...
    'VariableNames', {'Mode', 'PredictTime', 'TotalCost', 'LatCost', 'TimeCost', 'SpeedCost', 'longSafeCostTS','制約違反'});
    % a = table(labels, allTime, costTS, latCostTS, timeCostTS, speedCostTS, consCostTS, isValid, ...
    % 'VariableNames', {'Mode', 'PredictTime', 'TotalCost', 'LatCost', 'TimeCost', 'SpeedCost', 'ConsistencyCost', '制約違反'});
    disp(a);
      



    %% 各時刻でモードごとのLatCostを抽出
    CruiseLatCost = NaN(1,5);  % デフォルトはNaN、5列分のサイズに合わせる
    FollowingLatCost = NaN(1,5);  % デフォルトはNaN、5列分のサイズに合わせる
    ChangeLatCost = NaN(1,10);  % ここでは10列分のNaNを初期値として設定
    
    % "Cruise" モードがあれば、その時刻のLatCostを抽出
    if any(strcmp(a.Mode, 'Cruise'))
        CruiseLatCost = a.LatCost(strcmp(a.Mode, 'Cruise'))';  % 該当する行を抽出
    end
    
    % "Following" モードがあれば、その時刻のLatCostを抽出
    if any(strcmp(a.Mode, 'Follow'))
        FollowingLatCost = a.LatCost(strcmp(a.Mode, 'Follow'))';  % 該当する行を抽出
    end
    
    % "LaneChange" モードがあれば、その時刻のLatCostを抽出
    if any(strcmp(a.Mode, 'LaneChange'))
        ChangeLatCost = a.LatCost(strcmp(a.Mode, 'LaneChange'))';  % 該当する行を抽出
    end
    

     % それぞれのLatCostが必要な行列サイズ（5x1または10x1）であることを確認
    % もし不足していればNaNで埋める
    if length(CruiseLatCost) < 5
        CruiseLatCost = [CruiseLatCost, NaN(1, 5 - length(CruiseLatCost))];  % サイズを5x1に揃える
    end
    
    if length(FollowingLatCost) < 5
        FollowingLatCost = [FollowingLatCost, NaN(1, 5 - length(FollowingLatCost))];  % サイズを5x1に揃える
    end
    
    if length(ChangeLatCost) < 10
        ChangeLatCost = [ChangeLatCost, NaN(1, 10 - length(ChangeLatCost))];  % サイズを10x1に揃える
    end
   
    
    % 各時刻の値を1x5または1x10の行列にして格納
    CruiseLatCostLog(end+1,:)    = CruiseLatCost;    %#ok<SAGROW>
    FollowingLatCostLog(end+1,:) = FollowingLatCost; %#ok<SAGROW>
    ChangeLatCostLog (end+1,:)   = ChangeLatCost;    %#ok<SAGROW>
       
    %% 各時刻でモードごとのSpeedを抽出
    CruiseSpeedCost = NaN(1,5);  % デフォルトはNaN、5列分のサイズに合わせる
    FollowingSpeedCost = NaN(1,5);  % デフォルトはNaN、5列分のサイズに合わせる
    ChangeSpeedCost = NaN(1,10);  % ここでは10列分のNaNを初期値として設定
    
    % "Cruise" モードがあれば、その時刻のLatCostを抽出
    if any(strcmp(a.Mode, 'Cruise'))
        CruiseSpeedCost = a.SpeedCost(strcmp(a.Mode, 'Cruise'))';  % 該当する行を抽出
    end
    
    % "Following" モードがあれば、その時刻のLatCostを抽出
    if any(strcmp(a.Mode, 'Follow'))
        FollowingSpeedCost = a.SpeedCost(strcmp(a.Mode, 'Follow'))';  % 該当する行を抽出
    end
    
    % "LaneChange" モードがあれば、その時刻のLatCostを抽出
    if any(strcmp(a.Mode, 'LaneChange'))
        ChangeSpeedCost = a.SpeedCost(strcmp(a.Mode, 'LaneChange'))';  % 該当する行を抽出
    end
    

     % それぞれのLatCostが必要な行列サイズ（5x1または10x1）であることを確認
    % もし不足していればNaNで埋める
    if length(CruiseSpeedCost) < 5
        CruiseSpeedCost = [CruiseSpeedCost, NaN(1, 5 - length(CruiseSpeedCost))];  % サイズを5x1に揃える
    end
    
    if length(FollowingSpeedCost) < 5
        FollowingSpeedCost = [FollowingSpeedCost, NaN(1, 5 - length(FollowingSpeedCost))];  % サイズを5x1に揃える
    end
    
    if length(ChangeSpeedCost) < 10
        ChangeSpeedCost = [ChangeSpeedCost, NaN(1, 10 - length(ChangeSpeedCost))];  % サイズを10x1に揃える
    end
   
    
    % 各時刻の値を1x5または1x10の行列にして格納
    CruiseSpeedCostLog(end+1,:)    = CruiseSpeedCost;    %#ok<SAGROW>
    FollowingSpeedCostLog(end+1,:) = FollowingSpeedCost; %#ok<SAGROW>
    ChangeSpeedCostLog (end+1,:)   = ChangeSpeedCost;    %#ok<SAGROW>

    %% 各時刻でモードごとのTimeを抽出
    CruiseTimeCost = NaN(1,5);  % デフォルトはNaN、5列分のサイズに合わせる
    FollowingTimeCost = NaN(1,5);  % デフォルトはNaN、5列分のサイズに合わせる
    ChangeTimeCost = NaN(1,10);  % ここでは10列分のNaNを初期値として設定
    
    % "Cruise" モードがあれば、その時刻のLatCostを抽出
    if any(strcmp(a.Mode, 'Cruise'))
        CruiseTimeCost = a.TimeCost(strcmp(a.Mode, 'Cruise'))';  % 該当する行を抽出
    end
    
    % "Following" モードがあれば、その時刻のLatCostを抽出
    if any(strcmp(a.Mode, 'Follow'))
        FollowingTimeCost = a.TimeCost(strcmp(a.Mode, 'Follow'))';  % 該当する行を抽出
    end
    
    % "LaneChange" モードがあれば、その時刻のLatCostを抽出
    if any(strcmp(a.Mode, 'LaneChange'))
        ChangeTimeCost = a.TimeCost(strcmp(a.Mode, 'LaneChange'))';  % 該当する行を抽出
    end
    

     % それぞれのLatCostが必要な行列サイズ（5x1または10x1）であることを確認
    % もし不足していればNaNで埋める
    if length(CruiseTimeCost) < 5
        CruiseTimeCost = [CruiseTimeCost, NaN(1, 5 - length(CruiseTimeCost))];  % サイズを5x1に揃える
    end
    
    if length(FollowingTimeCost) < 5
        FollowingTimeCost = [FollowingTimeCost, NaN(1, 5 - length(FollowingTimeCost))];  % サイズを5x1に揃える
    end
    
    if length(ChangeTimeCost) < 10
        ChangeTimeCost = [ChangeTimeCost, NaN(1, 10 - length(ChangeTimeCost))];  % サイズを10x1に揃える
    end
   
    
    % 各時刻の値を1x5または1x10の行列にして格納
    CruiseTimeCostLog(end+1,:)    = CruiseTimeCost;    %#ok<SAGROW>
    FollowingTimeCostLog(end+1,:) = FollowingTimeCost; %#ok<SAGROW>
    ChangeTimeCostLog (end+1,:)   = ChangeTimeCost;    %#ok<SAGROW>


     %% 各時刻でモードごとのTotalを抽出
    CruiseTotalCost = NaN(1,5);  % デフォルトはNaN、5列分のサイズに合わせる
    FollowingTotalCost = NaN(1,5);  % デフォルトはNaN、5列分のサイズに合わせる
    ChangeTotalCost = NaN(1,10);  % ここでは10列分のNaNを初期値として設定
    
    % "Cruise" モードがあれば、その時刻のLatCostを抽出
    if any(strcmp(a.Mode, 'Cruise'))
        CruiseTotalCost = a.TotalCost(strcmp(a.Mode, 'Cruise'))';  % 該当する行を抽出
    end
    
    % "Following" モードがあれば、その時刻のLatCostを抽出
    if any(strcmp(a.Mode, 'Follow'))
        FollowingTotalCost = a.TotalCost(strcmp(a.Mode, 'Follow'))';  % 該当する行を抽出
    end
    
    % "LaneChange" モードがあれば、その時刻のLatCostを抽出
    if any(strcmp(a.Mode, 'LaneChange'))
        ChangeTotalCost = a.TotalCost(strcmp(a.Mode, 'LaneChange'))';  % 該当する行を抽出
    end
    

     % それぞれのLatCostが必要な行列サイズ（5x1または10x1）であることを確認
    % もし不足していればNaNで埋める
    if length(CruiseTotalCost) < 5
        CruiseTotalCost = [CruiseTotalCost, NaN(1, 5 - length(CruiseTotalCost))];  % サイズを5x1に揃える
    end
    
    if length(FollowingTotalCost) < 5
        FollowingTotalCost = [FollowingTotalCost, NaN(1, 5 - length(FollowingTotalCost))];  % サイズを5x1に揃える
    end
    
    if length(ChangeTotalCost) < 10
        ChangeTotalCost = [ChangeTotalCost, NaN(1, 10 - length(ChangeTotalCost))];  % サイズを10x1に揃える
    end
   
    
    % 各時刻の値を1x5または1x10の行列にして格納
    CruiseTotalCostLog(end+1,:)    = CruiseTotalCost;    %#ok<SAGROW>
    FollowingTotalCostLog(end+1,:) = FollowingTotalCost; %#ok<SAGROW>
    ChangeTotalCostLog (end+1,:)   = ChangeTotalCost;    %#ok<SAGROW>

    %% --- シーン内の全アクターの予測軌道を衝突チェッカーに反映 --- %%
    for i = 1:numel(actorPoses)
        actorPoses(i).States = futureTrajectory(i).Trajectory(:,1:3);
    end
    updateObstaclePose(capList,actorID,actorPoses);



    % --- 評価順序を決定 --- %%
    [cost, idx] = sort(costTS);
    optimalTrajectory = [];
    
    trajectoryEvaluation = nan(numel(isValid),1); % numelは要素数を返す
    

    % 車間距離も考慮した有効軌道選定（ルールベースの部分）
    % if minGap < forwardsafetyGap 
    %     % forwardsafetyGapよりminGapが小さい場合，isValidの最初の5要素をFalseにする
    %     isValid(1:min(5, numel(isValid))) = false;
    % end
    

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
        selLabel = Labels(selectedIdx);  % "Cruise" / "Follow" / "LaneChange"
    
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
    % modeTimeLog(end+1,1)  = (simStep-1)*scenario.SampleTime; %#ok<SAGROW>
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
            
            % --- 横方向加速度 [m/s^2] ---
            latAcc = EgoGlobalState(5)^2 * EgoGlobalState(4);
            latAccLog(end+1,1) = latAcc; %#ok<SAGROW>

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


%% 横偏差コストのグラフ
figName = fullfile(pwd, 'LateralCost.png');

if ~isempty(timeLog)
    hFig = figure('Name','LateralCost','NumberTitle','off');
    clf(hFig); hold on; grid on;

    % CruiseLatCostLog, FollowingLatCostLog, ChangeLatCostLog の各列をプロット
    for col = 1:size(CruiseLatCostLog, 2)  % 各列に対してプロット
        % NaN でないデータのみをプロット
        validCruiseLatData = ~isnan(CruiseLatCostLog(:, col));  % NaNでないデータを選択
        validFollowLatData = ~isnan(FollowingLatCostLog(:, col));  % NaNでないデータを選択
        validChangeLatData = ~isnan(ChangeLatCostLog(:, col));  % NaNでないデータを選択

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

    % 軸・タイトル
    hX = xlabel('Time (s)');
    hY = ylabel('LateralCost');
    hT = title('LateralCost Log');

    % 縦軸範囲
    xlim([0 max(timeLog)]);
    ylim([-0.1 0.1]);

    % 凡例
    hLeg = legend({'Cruise Mode', 'Following Mode', 'LaneChange Mode'}, 'Location', 'northeast');

    % フォント設定
    set([hX, hY, hT], 'FontName', 'Times New Roman', 'FontSize', 14);
    set(hLeg, 'FontName', 'Times New Roman', 'FontSize', 14);
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 14);
    % 枠線を黒に設定
    % set(gca, 'Box', 'on', 'LineWidth', 1, 'EdgeColor', 'k');


    hold off;
    drawnow;

    try
        saveas(hFig, figName);
        fprintf('Lateral Cost plot saved to: %s\n', figName);
    catch
        warning('Failed to save LateralCost plot to %s', figName);
    end
else
    % データなしの場合
    hFig = figure('Name', 'LateralCost (no data)', 'NumberTitle', 'off');
    clf(hFig);
    text(0.1, 0.5, 'No lateral cost data was logged during the simulation.', ...
        'FontSize', 12, 'FontName', 'Times New Roman');
    axis off;

    try
        saveas(hFig, figName);
        fprintf('No LateralCost plot saved to: %s\n', figName);
    catch
        warning('Failed to save no LateralCost plot to %s', figName);
    end
end

%% 速度差コストのグラフ
figName = fullfile(pwd, 'SpeedCost.png');

if ~isempty(timeLog)
    hFig = figure('Name','SpeedCost','NumberTitle','off');
    clf(hFig); hold on; grid on;

    % CruiseSpeedCostLog, FollowingSpeedCostLog, ChangeSpeedCostLog の各列をプロット
    for col = 1:size(CruiseSpeedCostLog, 2)  % 各列に対してプロット
        % NaN でないデータのみをプロット
        validCruiseSpeedData = ~isnan(CruiseSpeedCostLog(:, col));  % NaNでないデータを選択
        validFollowSpeedData = ~isnan(FollowingSpeedCostLog(:, col));  % NaNでないデータを選択
        validChangeSpeedData = ~isnan(ChangeSpeedCostLog(:, col));  % NaNでないデータを選択

        if any(validCruiseLatData)
            plot(timeLog(validCruiseSpeedData), CruiseSpeedCostLog(validCruiseSpeedData, col), '--', 'Color', 'r', 'LineWidth', 1.5);
        end
        if any(validFollowLatData)
            plot(timeLog(validFollowSpeedData), FollowingSpeedCostLog(validFollowSpeedData, col), '--', 'Color', 'b', 'LineWidth', 1.5);
        end
        if any(validChangeLatData)
            plot(timeLog(validChangeSpeedData), ChangeSpeedCostLog(validChangeSpeedData, col), '--', 'Color', 'g', 'LineWidth', 1.5);
        end
    end

    % 軸・タイトル
    hX = xlabel('Time (s)');
    hY = ylabel('SpeedCost');
    hT = title('SpeedCost Log');

    % 縦軸範囲
    xlim([0 max(timeLog)]);
    ylim([-0.1 0.6]);

    % 凡例
    hLeg = legend({'Cruise Mode', 'Following Mode', 'LaneChange Mode'}, 'Location', 'northeast');

    % フォント設定
    set([hX, hY, hT], 'FontName', 'Times New Roman', 'FontSize', 14);
    set(hLeg, 'FontName', 'Times New Roman', 'FontSize', 14);
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 14);
    % set(gca, 'Box', 'on', 'LineWidth', 1, 'EdgeColor', 'k');

    hold off;
    drawnow;

    try
        saveas(hFig, figName);
        fprintf('Speed Cost plot saved to: %s\n', figName);
    catch
        warning('Failed to save SpeedCost plot to %s', figName);
    end
else
    % データなしの場合
    hFig = figure('Name', 'SpeedCost (no data)', 'NumberTitle', 'off');
    clf(hFig);
    text(0.1, 0.5, 'No Speed cost data was logged during the simulation.', ...
        'FontSize', 12, 'FontName', 'Times New Roman');
    axis off;

    try
        saveas(hFig, figName);
        fprintf('No SpeedCost plot saved to: %s\n', figName);
    catch
        warning('Failed to save no SpeedCost plot to %s', figName);
    end
end

%% 時間差コストのグラフ
figName = fullfile(pwd, 'TimeCost.png');

if ~isempty(timeLog)
    hFig = figure('Name','TimeCost','NumberTitle','off');
    clf(hFig); hold on; grid on;

    % CruiseTimeCostLog, FollowingTimeCostLog, ChangeTimeCostLog の各列をプロット
    for col = 1:size(CruiseTimeCostLog, 2)  % 各列に対してプロット
        % NaN でないデータのみをプロット
        validCruiseTimeData = ~isnan(CruiseTimeCostLog(:, col));  % NaNでないデータを選択
        validFollowTimeData = ~isnan(FollowingTimeCostLog(:, col));  % NaNでないデータを選択
        validChangeTimeData = ~isnan(ChangeTimeCostLog(:, col));  % NaNでないデータを選択

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

    % 軸・タイトル
    hX = xlabel('Time (s)');
    hY = ylabel('TimeCost');
    hT = title('TimeCost Log');

    % 縦軸範囲
    xlim([0 max(timeLog)]);
    ylim([0 0.3]);

    % 凡例
    hLeg = legend({'Cruise Mode', 'Following Mode', 'LaneChange Mode'}, 'Location', 'northeast');

    % フォント設定
    set([hX, hY, hT], 'FontName', 'Times New Roman', 'FontSize', 14);
    set(hLeg, 'FontName', 'Times New Roman', 'FontSize', 14);
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 14);
    % set(gca, 'Box', 'on', 'LineWidth', 1, 'EdgeColor', 'k');

    hold off;
    drawnow;

    try
        saveas(hFig, figName);
        fprintf('Time Cost plot saved to: %s\n', figName);
    catch
        warning('Failed to save TimeCost plot to %s', figName);
    end
else
    % データなしの場合
    hFig = figure('Name', 'TimeCost (no data)', 'NumberTitle', 'off');
    clf(hFig);
    text(0.1, 0.5, 'No Time cost data was logged during the simulation.', ...
        'FontSize', 12, 'FontName', 'Times New Roman');
    axis off;

    try
        saveas(hFig, figName);
        fprintf('No TimeCost plot saved to: %s\n', figName);
    catch
        warning('Failed to save no TimeCost plot to %s', figName);
    end
end

%% トータルコストのグラフ
figName = fullfile(pwd, 'TotalCost.png');

if ~isempty(timeLog)
    hFig = figure('Name','TotalCost','NumberTitle','off');
    clf(hFig); hold on; grid on;

    % CruiseTotalCostLog, FollowingTotalCostLog, ChangeTotalCostLog の各列をプロット
    for col = 1:size(CruiseTotalCostLog, 2)  % 各列に対してプロット
        % NaN でないデータのみをプロット
        validCruiseTotalData = ~isnan(CruiseTotalCostLog(:, col));  % NaNでないデータを選択
        validFollowTotalData = ~isnan(FollowingTotalCostLog(:, col));  % NaNでないデータを選択
        validChangeTotalData = ~isnan(ChangeTotalCostLog(:, col));  % NaNでないデータを選択

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

    % 軸・タイトル
    hX = xlabel('Time (s)');
    hY = ylabel('TotalCost');
    hT = title('TotalCost Log');

    % 縦軸範囲
    xlim([0 max(timeLog)]);
    ylim([0 0.8]);

    % 凡例
    hLeg = legend({'Cruise Mode', 'Following Mode', 'LaneChange Mode'}, 'Location', 'northeast');

    % フォント設定
    set([hX, hY, hT], 'FontName', 'Times New Roman', 'FontSize', 14);
    set(hLeg, 'FontName', 'Times New Roman', 'FontSize', 14);
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 14);
    % set(gca, 'Box', 'on', 'LineWidth', 1, 'EdgeColor', 'k');

    hold off;
    drawnow;

    try
        saveas(hFig, figName);
        fprintf('Total Cost plot saved to: %s\n', figName);
    catch
        warning('Failed to save Total Cost plot to %s', figName);
    end
else
    % データなしの場合
    hFig = figure('Name', 'Total Cost (no data)', 'NumberTitle', 'off');
    clf(hFig);
    text(0.1, 0.5, 'No Total cost data was logged during the simulation.', ...
        'FontSize', 12, 'FontName', 'Times New Roman');
    axis off;

    try
        saveas(hFig, figName);
        fprintf('No Total Cost plot saved to: %s\n', figName);
    catch
        warning('Failed to save no Total Cost plot to %s', figName);
    end
end
%% --- 車間距離のログ ---%%
figName = fullfile(pwd, 'gap_plot.png');

if ~isempty(timeLog)
    hFig = figure('Name','Ego to lead gaps','NumberTitle','off');
    clf(hFig); hold on; grid on;

    % --- Gap プロット ---
    hPlotLead   = plot(timeLog, gapForwardLog, '-', 'Color', 'b', 'LineWidth', 4);
    hPlotBehind = plot(timeLog, gapBehindLog,  '-', 'Color', 'r', 'LineWidth', 4);

    % --- Safety Gap（前方：青点線）---
    hYlineLead = yline(forwardsafetyGap, 'b--', 'LineWidth', 2);

    % --- Safety Gap（後方：赤点線）---
    hYlineBehind = yline(behindsafetyGap, 'r--', 'LineWidth', 2);

    % --- 軸・タイトル ---
    hX = xlabel('Time (s)');
    hY = ylabel('Gap (m)');
    hT = title('Vehicle to Vehicle Gaps');

    % 縦軸範囲
    ylim([0 40]);

    % --- 凡例（ここだけで名前を出す）---
    hLeg = legend([hPlotLead hPlotBehind hYlineLead hYlineBehind], ...
        {'Forward Gap','Behind Gap','Forward Safety Gap','Behind Safety Gap'}, ...
        'Location','northeast');

    % --- フォント設定 ---
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
    % データなしの場合
    hFig = figure('Name','Ego to lead gaps (no data)','NumberTitle','off');
    clf(hFig);
    text(0.1,0.5,'No gap data was logged during the simulation.', ...
        'FontSize',12,'FontName','Times New Roman');
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
    hSpeed = plot(timeLog, speedLog, '-', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 4);
    % Labels and title
    hX2 = xlabel('Time (s)');
    hY2 = ylabel('Speed (m/s)');
    hT2 = title('Ego vehicle speed');
    hLeg2 = legend(hSpeed, {'Ego speed'}, 'Location','northeast');
    set([hX2,hY2,hT2],'FontName','Times New Roman','FontSize',9);
    set(hLeg2,'FontName','Times New Roman','FontSize',14);
    set(gca,'FontName','Times New Roman','FontSize',14);
    % 表示範囲: 速度を 15 m/s から 20 m/s の範囲で表示
    try
        ylim([6 20]);
        yticks(6:2:20);
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
    hYaw = plot(timeLog, yawLog, '-', 'Color', [0.3010 0.7450 0.9330], 'LineWidth', 4);

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
        ylim([-5 5]);
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

%% --- 横方向加速度ログ --- %%
latAccFigName = fullfile(pwd, 'lateral_acc_plot.png');

if ~isempty(timeLog) && ~isempty(latAccLog)
    hFig = figure('Name','Lateral Acceleration','NumberTitle','off');
    clf(hFig); grid on; hold on;

    plot(timeLog, latAccLog, 'k-', 'LineWidth', 4);
    % yline(2.5, 'r--', 'LineWidth', 2);
    % yline(-2.5,'r--', 'LineWidth', 2);

    xlabel('Time (s)');
    ylabel('Lateral Acceleration (m/s^2)');
    title('Ego Vehicle Lateral Acceleration');

    set(gca,'FontName','Times New Roman','FontSize',14);
    hold off;

    try
        saveas(hFig, latAccFigName);
        fprintf('Lateral acc plot saved to: %s\n', latAccFigName);
    catch
        warning('Failed to save lateral acc plot to %s', latAccFigName);
    end
end




%% --- 選定モード（Cruise/Follow/LaneChange）0/1 時系列プロット --- %%
modeFigName = fullfile(pwd, 'mode_plot.png');

if ~isempty(timeLog)
    hFig4 = figure('Name','Selected Mode','NumberTitle','off'); 
    clf(hFig4); grid on; hold on;

    % 0/1 の階段状に見せたいので stairs を使用
    hCruise     = stairs(timeLog, modeBinLog(:,1), 'Color', 'r', 'LineStyle', ':','LineWidth', 1.5);
    hFollow     = stairs(timeLog, modeBinLog(:,2), 'Color', 'b', 'LineStyle', ':','LineWidth', 1.5);
    hLaneChange = stairs(timeLog, modeBinLog(:,3), 'Color', [0 0.4 0], 'LineStyle', ':', 'LineWidth', 1.5);
  

    xlabel('Time (s)');
    ylabel('Mode');
    title('Selected Mode');
    legend([hCruise, hFollow, hLaneChange], {'Cruise','Follow','LaneChange'}, 'Location','best');
    ylim([0 3.5]);

    set(gca,'FontName','Times New Roman','FontSize',14);
    saveas(hFig4, modeFigName);
    fprintf('Mode plot saved to: %s\n', modeFigName);
end
    



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

%% ==== 横方向加速度（Lateral Acceleration） ====
if ~isempty(timeLog) && ~isempty(latAccLog)
    v = VideoWriter('lateral_acc_bar.mp4', 'MPEG-4');
    v.FrameRate = 10; open(v);

    fig = figure('Name','Lateral Acceleration with Red Bar','NumberTitle','off');
    clf(fig); hold on; grid on;

    plot(timeLog, latAccLog, 'k-', 'LineWidth', 2);

    xlabel('Time (s)');
    ylabel('Lateral Acceleration (m/s^2)');
    title('Ego Vehicle Lateral Acceleration');
    ylim([-5 5]);
    xlim([min(timeLog) max(timeLog)]);

    set(gca,'FontName','Times New Roman','FontSize',14);
    barLine = xline(timeLog(1), 'r-', 'LineWidth', 2);

    for i = 1:length(timeLog)
        set(barLine, 'Value', timeLog(i));
        title(sprintf('Lateral Acc (t = %.2f s)', timeLog(i)));
        frame = getframe(fig);
        writeVideo(v, frame);
    end

    close(v);
    disp('✅ Lateral acceleration + red bar video saved.');
end

%% ==== 走行モード（Mode） ====
if ~isempty(timeLog)
    v = VideoWriter('mode_bar.mp4', 'MPEG-4');
    v.FrameRate = 10; open(v);
    fig = figure('Name','Mode with Red Bar','NumberTitle','off'); 
    clf(fig); hold on; grid on;

    % --- プロット ---
    hCruise = stairs(timeLog, modeBinLog(:,1), 'r:', 'LineWidth', 2);
    hFollow = stairs(timeLog, modeBinLog(:,2), 'b:', 'LineWidth', 2);
    hLane   = stairs(timeLog, modeBinLog(:,3), 'Color', [0 0.4 0], ...
                     'LineStyle', ':', 'LineWidth', 2);

    % --- 軸・凡例・フォント設定 ---
    xlabel('Time (s)');
    ylabel('Mode');
    title('Mode Transition Over Time');
    ylim([0 4]); % 縦軸の範囲を0〜4に設定
    xlim([min(timeLog) max(timeLog)]); % 横軸をデータ範囲に設定

    % --- 凡例（data1などを排除して手動指定） ---
    legend([hCruise, hFollow, hLane], ...
        {'Cruise','Follow','LaneChange'}, ...
        'Location','northeast', 'Box','on', ...
        'FontName','Times New Roman','FontSize',14);

    set(gca,'FontName','Times New Roman','FontSize',14);
    set(findall(gca,'Type','text'),'FontName','Times New Roman');

    % --- 赤バー ---
    barLine = xline(timeLog(1), 'r-', 'LineWidth', 2);

    % --- アニメーション ---
    timeFrames = timeLog;
    for i = 1:length(timeFrames)
        set(barLine, 'Value', timeFrames(i));
        title(sprintf('Mode Transition (t = %.2f s)', timeFrames(i)), ...
              'FontName','Times New Roman','FontSize',14);
        frame = getframe(fig);
        writeVideo(v, frame);
    end

    close(v);
    disp('✅ Mode + red bar video saved.');

else
    warning('No mode data was logged.');
end
