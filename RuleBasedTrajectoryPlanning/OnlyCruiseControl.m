scenario = scenario1;



% デフォルトの車両プロパティ
carLen   = 4.7; % [m] 一般的な乗用車の全長
carWidth = 1.8; % [m] 一般的な乗用車の全幅
rearAxleRatio = .25; % 全長 4.7 m なら、後軸は後端から 1.175 m (4.7 × 0.25) の位置にある、という意味。

% Define road dimensions
laneWidth   = carWidth*2; % [m] 車線幅

plot(scenario);



connector = trajectoryGeneratorFrenet(refPath);



capList = dynamicCapsuleList;



egoID = 1; % 自車両には「1」を割り振り，dynamicCapsuleList に登録する車両を一意に識別する番号
[egoID, egoGeom] = egoGeometry(capList,egoID); % dynamicCapsuleList（capList）内で、ID=1 の Ego 車両用の ジオメトリ情報を取り出します。

egoGeom.Geometry.Length = carLen; % 4.7 [m]
egoGeom.Geometry.Radius = carWidth/2; % 1.8/s=0.9 [m]
egoGeom.Geometry.FixedTransform(1,end) = -carLen*rearAxleRatio; % [m], -carLen * 0.25 = -1.175 m を指定すると、カプセルが 後輪軸を基準に配置される



updateEgoGeometry(capList,egoID,egoGeom);
show(capList)


actorID = (1:6)'; % 他車両（Actor）の ID を 1〜6 の連番で作成。actorID は列ベクトル [1; 2; 3; 4; 5; 6]。
actorGeom = repelem(egoGeom,6,1); % egoGeom → 先ほど定義した 車両の形状（長さ=4.7m, 幅=1.8m, 後輪基準）, repelem(...,5,1) → これを 6台分コピーして、6×1 の構造体配列にする
updateObstacleGeometry(capList,actorID,actorGeom) % capList に「ID=1〜6 の車両」を 障害物（Obstacle）として登録。それぞれの車両は actorGeom に基づいて、長さ4.7m・幅1.8m のカプセルで表される


% シミュレータの更新レートを軌道生成器の離散化間隔に合わせるように同期す
% scenario.SampleTime → シナリオのシミュレーション刻み時間
% connector.TimeResolution → Frenet軌道生成器が使う時間刻み
% これを揃えることで、軌道生成とシナリオ進行を同じタイミングで動かす
scenario.SampleTime = connector.TimeResolution; % in seconds


% 計画パラメータを定義する。
% 10 Hz = 0.1秒ごとに軌道を再計算。
replanRate = 10; % Hz

% 現在の状態と計画状態の間隔を定義する。
timeHorizons = 1:0.5:3; % [sec] 
maxHorizon = max(timeHorizons); % [sec] 

% コストパラメータの定義
latDevWeight    =  5; % 横方向ずれ（車線中央からの逸脱）
timeWeight      =  -1; % 短い時間を好む
speedWeight     =  1; % 目標速度からの逸脱

% 以下の制約に違反する軌道を排除する。
maxAcceleration =  10; % [m/s^2] 車両が出せる最大加速度を超えたら不適
maxCurvature    = 100; % [1/m], or [rad/m] 最小旋回半径 ≈ 1m
minVelocity     =   0; % [m/sec] 負の速度（バック）や停止を避ける。

% クルーズコントロール動作および軌道コスト評価時に使用される目標速度設定値。
speedLimit = 17; % [m/sec] 巡航時の目標速度。コスト評価の基準値

% プランナーが追従動作において目標とする最小距離。
forwardsafetyGap = 16; % [m] 前車との最低距離。これを下回る軌道は不適切として棄却
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
    % アクター車両の現在の状態と計画ホライズンにわたる軌跡を取得する。
    [curActorState,futureTrajectory,isRunning] = ...
        exampleHelperRetrieveActorGroundTruth(scenario,futureTrajectory,replanRate,maxHorizon);


    % 巡航制御の状態を生成
    [termStatesCC,timesCC] = exampleHelperBasicCruiseControl(...
        refPath,laneWidth,egoState,speedLimit,timeHorizons);
    
     % 前方車追従の状態を生成
    [termStatesF,timesF] = exampleHelperBasicLeadVehicleFollow(...
        refPath,laneWidth,forwardsafetyGap,egoState,curActorState,timeHorizons);
    
    % 車線変更の状態を生成
    % [termStatesLC,timesLC] = exampleHelperBasicLaneChange(...
    %     refPath,laneWidth,egoState, timeHorizons);
    


    %     % 終端状態をまとめる
    % labels = [repmat("Cruise", size(termStatesCC,1), 1); 
    %           repmat("LaneChange", size(termStatesLC,1), 1);
    %           repmat("Follow", size(termStatesF,1), 1)];
    % % allTS   = [termStatesCC; termStatesLC; termStatesF];
    % allTime = [timesCC; timesLC; timesF];

            % 終端状態をまとめる
    labels = [repmat("Cruise", size(termStatesCC,1), 1)];
    allTime = timesCC;



    %  % 終端状態と時間を組み合わせる。
    % allTS = [termStatesCC; termStatesLC; termStatesF];
    % allDT = [timesCC; timesLC; timesF];
    % numTS = [numel(timesCC); numel(timesLC); numel(timesF)];
    
      % 終端状態と時間を組み合わせる。
    allTS = termStatesCC;
    allDT = timesCC;
    numTS = [numel(timesCC)];

    % すべての終端状態のコストを評価する。
    [costTS,latCostTS,timeCostTS,speedCostTS] = exampleHelperEvaluateTSCost(allTS,allDT,laneWidth,speedLimit,...
        speedWeight, latDevWeight, timeWeight);


    % 軌道を生成
    egoFrenetState = global2frenet(refPath,egoState);
    [frenetTraj,globalTraj] = connect(connector,egoFrenetState,allTS,allDT);
    
    % 制約条件に違反する軌道を排除
    isValid = exampleHelperEvaluateTrajectory(globalTraj,maxAcceleration,maxCurvature,minVelocity);
    % disp(isValid);


    % シーン内の全アクターの予測軌道を衝突チェッカーに反映させる。
    for i = 1:numel(actorPoses)
        actorPoses(i).States = futureTrajectory(i).Trajectory(:,1:3);
    end
    updateObstaclePose(capList,actorID,actorPoses);

    % --- 同一レーンにいる最も近い前方車両との車間距離 ---
    try
        egoF = global2frenet(refPath, egoState);
        actorFs = global2frenet(refPath, curActorState);
        curEgoLane = exampleHelperPredictLane(egoF, laneWidth, 0);

        % compute gaps for actors that are in the same lane and ahead
        gaps = nan(1,numActors);
        for a = 1:numActors
            aLane = exampleHelperPredictLane(actorFs(a,:), laneWidth, 0);
            if aLane == curEgoLane && actorFs(a,1) > egoF(1)
                gaps(a) = actorFs(a,1) - egoF(1);
            end
        end

        % select the nearest forward vehicle (smallest positive gap)
        validIdx = find(~isnan(gaps));
        if ~isempty(validIdx)
            [minGap, relIdx] = min(gaps(validIdx));
            actorIdx = validIdx(relIdx);
        else
            minGap = nan;
            actorIdx = nan;
        end

        % compute rear gaps (ego - actor for actors behind in same lane)
        rearGaps = nan(1,numActors);
        for a = 1:numActors
            aLane = exampleHelperPredictLane(actorFs(a,:), laneWidth, 0);
            if aLane == curEgoLane && actorFs(a,1) < egoF(1)
                rearGaps(a) = egoF(1) - actorFs(a,1);
            end
        end
        validRear = find(~isnan(rearGaps));
        if ~isempty(validRear)
            [minRearGap, relRearIdx] = min(rearGaps(validRear));
            behindActorIdx = validRear(relRearIdx);
        else
            minRearGap = nan;
            behindActorIdx = nan;
        end

        gapForwardLog(end+1,1) = minGap;           %#ok<SAGROW> store single scalar per step (leading)
            % --- 安全距離を下回ったらシミュレーションを強制終了 ---
        % if ~isnan(minGap) && minGap < forwardsafetyGap
        %     disp('⚠️ Forward safety gap violated. Ending simulation early.');
        %     isRunning = false;  % whileループを抜ける条件を満たす
        % end

        gapBehindLog(end+1,1) = minRearGap; %#ok<SAGROW> store single scalar per step (behind)
        leadIdxLog(end+1,1) = actorIdx;      %#ok<SAGROW>
        behindIdxLog(end+1,1) = behindActorIdx; %#ok<SAGROW>
        timeLog(end+1,1) = (simStep-1)*scenario.SampleTime; %#ok<SAGROW>
        % Log ego speed (5th element of egoState is speed in this representation)
        if numel(egoState) >= 5
            speedLog(end+1,1) = egoState(5); %#ok<SAGROW>
            
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
    
    % 評価順序を決定する。
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

      

            break;
            
        else
            trajectoryEvaluation(idx(i)) = 0;
        end
    end
end

% === 選定モードを決定し、ログ化 ===
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
            onehot = [0 1 0];
        case "LaneChange"
            onehot = [0 0 1];
        otherwise
            onehot = [0 0 0];
    end
end

% --- 時刻歴ログに追記 ---
modeTimeLog(end+1,1)  = (simStep-1)*scenario.SampleTime; %#ok<SAGROW>
modeLabelLog(end+1,1) = selLabel;                        %#ok<SAGROW>
modeBinLog(end+1,:)   = onehot;                          %#ok<SAGROW>


%% --- サンプリングされた軌跡を表示--- %%
modeCol = repmat("", size(allTS,1), 1);
selectedIdx = find(trajectoryEvaluation==1,1);
if ~isempty(selectedIdx)
end

    lineHandles = exampleHelperVisualizeScene(lineHandles,globalTraj,isValid,trajectoryEvaluation);


    hold on;
    show(capList,'TimeStep',1:capList.MaxNumSteps,'FastUpdate',1);
    hold off;
    
    if isempty(optimalTrajectory)
        % すべての軌道は制約条件に違反するか、衝突を引き起こした。
        %
        %   計画が直ちに失敗した場合は、シミュレータ、プランナー、および動作プロパティを再検討。
        %
        %  シミュレーションの途中でプランナーが失敗した場合、より複雑な計画条件に対処するために追加の動作を導入できる。
        error('No valid trajectory has been found.');
    else
        % 再計画の間の場面を視覚化する。
        for i = (2+(0:(stepPerUpdate-1)))
            % おおよそのリアルタイム可視化。
            dt = toc;
            if scenario.SampleTime-dt > 0
                pause(scenario.SampleTime-dt);
            end
            
            egoState = optimalTrajectory(i,:);
            scenarioViewer.Actors(1).Position(1:2) = egoState(1:2);
            scenarioViewer.Actors(1).Velocity(1:2) = [cos(egoState(3)) sin(egoState(3))]*egoState(5);
            scenarioViewer.Actors(1).Yaw = egoState(3)*180/pi;

             % --- ヨー角ログを記録（ラジアン単位） ---
            yawLog(end+1,1) = egoState(3)*180/pi;  %#ok<SAGROW>

            scenarioViewer.Actors(1).AngularVelocity(3) = egoState(4)*egoState(5);
            
            % カプセルの可視化を更新
            hold on;
            show(capList,'TimeStep',i:capList.MaxNumSteps,'FastUpdate',1);
            hold off;
            
            % ドライビングシナリオの更新
            advance(scenarioViewer);
            tic;
        end
    end
   
end

%% --- 車間距離のログ ---%%
figName = fullfile(pwd, 'gap_plot.png');
if ~isempty(timeLog)
    hFig = figure('Name','Ego to lead gaps','NumberTitle','off'); 
    clf(hFig); hold on; grid on;

    % Forward gap（実測）を青線で描画
    hPlotLead = plot(timeLog, gapForwardLog, '-', 'Color', 'b', 'LineWidth', 3);

    % Safety gap の基準線（ラベルは空）
    hYlineLead = yline(forwardsafetyGap, 'b--', 'Label', '');
    hYlineLead.LabelHorizontalAlignment = 'right';
    hYlineLead.LabelVerticalAlignment = 'bottom';
    hYlineLead.FontName = 'Times New Roman';
    hYlineLead.FontSize = 14;

    % 軸ラベル・タイトル設定（フォント統一）
    hX = xlabel('Time (s)');
    hY = ylabel('Gap (m)');
    hT = title('forward vehicle gaps');

    % 凡例（グラフ上に文字は出さない）
    hLeg = legend([hPlotLead hYlineLead], ...
        {'Forward gap','SafetyGap between Forward vehicle'}, ...
        'Location','northeast');

    % フォント統一
    set([hX,hY,hT],'FontName','Times New Roman','FontSize',14);
    set(hLeg,'FontName','Times New Roman','FontSize',14);
    set(gca,'FontName','Times New Roman','FontSize',14);

    hold off;
    drawnow;

    % 保存処理
    try
        saveas(hFig, figName);
        fprintf('Gap plot saved to: %s\n', figName);
    catch
        warning('Failed to save gap plot to %s', figName);
    end
else
    % データが存在しない場合
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
    hSpeed = plot(timeLog, speedLog, '-', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2);
    % Labels and title
    hX2 = xlabel('Time (s)');
    hY2 = ylabel('Speed (m/s)');
    hT2 = title('Ego vehicle speed');
    hLeg2 = legend(hSpeed, {'Ego speed'}, 'Location','best');
    set([hX2,hY2,hT2],'FontName','Times New Roman','FontSize',14);
    set(hLeg2,'FontName','Times New Roman','FontSize',14);
    set(gca,'FontName','Times New Roman','FontSize',14);
    % 表示範囲: 速度を 15 m/s から 20 m/s の範囲で表示
    try
        ylim([10 20]);
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
    yawLog = unwrap(yawLog);  % yawLog は [rad] 単位

    % グラフを作成
    hFig3 = figure('Name','Ego yaw angle','NumberTitle','off');
    clf(hFig3); grid on; hold on;

    % Plot yaw vs time as continuous line
    hYaw = plot(timeLog, yawLog, '-', 'Color', [0.3010 0.7450 0.9330], 'LineWidth', 2);

    % Labels and title
    hX3 = xlabel('Time (s)');
    hY3 = ylabel('Yaw angle (rad)');
    hT3 = title('Ego vehicle yaw angle');
    hLeg3 = legend(hYaw, {'Yaw angle'}, 'Location','best');

    % Times New Roman フォント設定（14pt）
    set([hX3,hY3,hT3],'FontName','Times New Roman','FontSize',14);
    set(hLeg3,'FontName','Times New Roman','FontSize',14);
    set(gca,'FontName','Times New Roman','FontSize',14);

    % 表示範囲（自動）
    try
        ylim auto;
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
        'FontSize',14,'FontName','Times New Roman');
    axis off;
    try
        saveas(hFig3, yawFigName);
        fprintf('No-yaw plot saved to: %s\n', yawFigName);
    catch
        warning('Failed to save no-yaw plot to %s', yawFigName);
    end
end






%% --- 選定モード（Cruise/Follow/LaneChange）0/1 時系列プロット --- %%
% function h = plotStepsNoVertical(x, y, varargin)
%     % 横方向のステップ線のみ描く（縦線なし）
%     x = x(:); y = y(:);
%     if numel(x) ~= numel(y)
%         error('x and y must have the same length.');
%     end
%     if numel(x) < 2
%         h = plot(x, y, varargin{:}); 
%         return;
%     end
%     % 各サンプル区間 [x_i, x_{i+1}] を水平線で結ぶ（y は y_i を保持）
%     X = [x(1:end-1) x(2:end) nan(numel(x)-1,1)]';
%     Y = [y(1:end-1) y(1:end-1) nan(numel(y)-1,1)]';
%     h = plot(X(:), Y(:), varargin{:});
% end

modeFigName = fullfile(pwd, 'mode_plot.png');

if ~isempty(modeTimeLog)
    hFig4 = figure('Name','Selected Mode (binary time series)','NumberTitle','off'); 
    clf(hFig4); grid on; hold on;

    % 0/1 の階段状に見せたいので stairs を使用
    hCruise     = stairs(modeTimeLog, modeBinLog(:,1), 'Color', 'r', 'LineStyle', ':','LineWidth', 2);
    hFollow     = stairs(modeTimeLog, modeBinLog(:,2), 'Color', 'b', 'LineStyle', ':','LineWidth', 2);
    hLaneChange = stairs(modeTimeLog, modeBinLog(:,3), 'Color', [0 0.4 0], 'LineStyle', ':', 'LineWidth', 2);
    
    % hCruise     = plotStepsNoVertical(modeTimeLog, modeBinLog(:,1), 'r-', 'LineWidth', 2);
    % hFollow     = plotStepsNoVertical(modeTimeLog, modeBinLog(:,2), 'b-', 'LineWidth', 2);
    % hLaneChange = plotStepsNoVertical(modeTimeLog, modeBinLog(:,3), 'k-', 'LineWidth', 2);

    
    

    xlabel('Time (s)');
    ylabel('Mode');
    title('Selected Mode');
    legend([hCruise, hFollow, hLaneChange], {'Cruise','Follow','LaneChange'}, 'Location','best');
    ylim([-0.1 3.1]);

    set(gca,'FontName','Times New Roman','FontSize',14);
    saveas(hFig4, modeFigName);
    fprintf('Mode plot saved to: %s\n', modeFigName);

    % CSV でも残したい場合（解析・論文図表用）
    modeTbl = table(modeTimeLog, modeBinLog(:,1), modeBinLog(:,2), modeBinLog(:,3), modeLabelLog, ...
        'VariableNames', {'Time_s','Cruise','Follow','LaneChange','SelectedMode'});
    writetable(modeTbl, 'modeLog.csv');

    % 可変長の候補一覧は .mat で保存（後で詳細追跡したいとき用）
    save('modeCandidatesLog.mat', 'labelsLog', 'modeColLog', 'modeTimeLog');
else
    warning('No mode data was logged.');
end
