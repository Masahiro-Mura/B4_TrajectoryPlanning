function [terminalStates, times, leadGapsLC] = TSLaneChange( ...
    refPath, laneWidth, forwardsafetyGap, behindsafetyGap, ...
    EgoGlobalState, AllActorGlobalState, carLen, rearAxleRatio, SpeedLimit, dt)
    
    nDt = numel(dt);

    %% 自車速度ゼロなら車線変更は行わない
    if EgoGlobalState(5) == 0
        terminalStates   = [];
        times            = [];
        leadGapsLC       = [];
        return; % 関数の実行をストップして呼び出し元に戻る
    end

    %% 1. 自車の現Frenet状態 & 現在レーン取得
    EgoFrenet = global2frenet(refPath, EgoGlobalState);  % [s, ds, dds, l, dl, ddl]
    % ActorsFrenet = global2frenet(refPath, AllActorGlobalState);
    sEgo0     = EgoFrenet(1);                            % 自車両の現在のs座標
    dsEgo0    = EgoFrenet(2);                            % 自車両の現在のs方向速度
    EgoLaneNum0  = PredictLane(EgoFrenet, laneWidth, 0); % t=0 でのレーン

    %% 2. 隣接レーン（左・右）
    % ここでは 3 車線道路を前提に laneID ∈ {1,2,3}
    rawAdjLanes   = EgoLaneNum0 + [-1 1];               % rawAdjLanesは0 or 1 or 2 or 3 or 4
    validMask     = rawAdjLanes > 0 & rawAdjLanes <= 3; % ここで1~3に限定し，隣に車線があれば1，なければ0を返す
    AdjLanesIdx = rawAdjLanes(validMask);               % 「隣接」している車線の番号（車線に割り振られたインデックス）を返す
    AdjLanesNum    = numel(AdjLanesIdx);                % 「隣接」している車線の本数を返す

    if AdjLanesNum     == 0                             % 隣接している斜線が無ければ↓
        terminalStates = [];
        times          = [];
        leadGapsLC     = [];
        return;                                         % そもそもこの関数を実行しない（車線変更不可能だから）
    end


    %% 3. 周辺車両のFrenet座標 & 車線番号
    numActors = size(AllActorGlobalState, 1);           % 周辺車両の台数をnumActorsと設定
    
    AllActorsFrenetStates = zeros(numActors, 6);
    AllActorsLanes   = zeros(numActors, 1);
    for i = 1:numActors
        AllActorsFrenetStates(i,:) = global2frenet(refPath, AllActorGlobalState(i,:));
        AllActorsLanes(i,1)   = PredictLane(AllActorsFrenetStates(i,:), laneWidth, 0);
    end
  
    

    %% 4. 隣接レーンごとの隣接前方車・後方車の「dt秒後のs座標」を計算
    Front_s  = NaN(AdjLanesNum, 1);
    Front_v  = NaN(AdjLanesNum, 1);
    Behind_s = NaN(AdjLanesNum, 1);
    Behind_v = NaN(AdjLanesNum, 1);

    
    AdjacentForwardVehicle = NaN(AdjLanesNum, nDt);  % AdjLanesNumは隣接している車線の本数，nDtは予測秒数の個数
    AdjacentBehindVehicle  = NaN(AdjLanesNum, nDt);

    
    for laneIdx = 1:AdjLanesNum                      % AdjLanesNumは隣接している車線の本数
        laneID = AdjLanesIdx(laneIdx);
  

        if numActors == 0
            continue;
        end

        % この隣接レーン上の車両だけ抜き出し
        idxLane = find(AllActorsLanes == laneID);
        if isempty(idxLane)
            continue;  % このレーンには車両がいない
        end

        sActors = AllActorsFrenetStates(idxLane, 1);  % s 座標
        % vActors = AllActorsFrenetStates(idxLane, 2);  % s 方向速度 (global2frenet の仕様依存)

        % 先行車候補（時刻0で自車より前）
        forwardMask  = sActors > sEgo0;
        forwardIdxes = idxLane(forwardMask);
        if ~isempty(forwardIdxes)
            sF_all = AllActorsFrenetStates(forwardIdxes, 1);
            vF_all = AllActorsFrenetStates(forwardIdxes, 2);
            % 自車に最も近い先行車（s 差が最小）を選択
            [~, iMin] = min(sF_all - sEgo0);
            sF0 = sF_all(iMin);
            vF0 = vF_all(iMin);

            Front_s(laneIdx) = sF0;
            Front_v(laneIdx) = vF0;

            AdjacentForwardVehicle(laneIdx, :) = sF0 + vF0 * dt;  % 等速で予測
        end


        % 後方車候補（時刻0で自車より後ろ）
        behindMask  = sActors < sEgo0;
        behindIdxes = idxLane(behindMask);
        if ~isempty(behindIdxes)
            sB_all = AllActorsFrenetStates(behindIdxes, 1);
            vB_all = AllActorsFrenetStates(behindIdxes, 2);
            % 自車に最も近い後方車（s 差が最大のもの）を選択
            [~, iMax] = max(sB_all);  % sB_all は < sEgo0 なので最大値が最も近い
            sB0 = sB_all(iMax);
            vB0 = vB_all(iMax);

            % ← これを追加
            Behind_s(laneIdx) = sB0;
            Behind_v(laneIdx) = vB0;

            AdjacentBehindVehicle(laneIdx, :) = sB0 + vB0 * dt;  % 等速で予測
        end
    end

    %% ==== 5. 完全分岐で終端状態を生成 ====
    
    terminalStates = [];
    times          = [];
    leadGapsLC     = [];

    for laneIdx = 1:AdjLanesNum

        sF0 = Front_s(laneIdx);
        vF0 = Front_v(laneIdx);
        sB0 = Behind_s(laneIdx);
        vB0 = Behind_v(laneIdx);


        laneID   = AdjLanesIdx(laneIdx);
        % 3車線: laneID=1,2,3 のとき 中央レーン(2)を d=0 とする
        d_target = (2 - laneID) * laneWidth;

        for k = 1:nDt
            t = dt(k);
            
            % 隣接レーン上の最も近い先行車，後方車が dt 秒後にいる s 座標
            Af = AdjacentForwardVehicle(laneIdx, k);  % 先行車の s(t)
            Ab = AdjacentBehindVehicle(laneIdx, k);   % 後方車の s(t)

            hasForwardVehicle = ~isnan(Af);
            hasBehindVehicle  = ~isnan(Ab);


            % ✅ 隣接車線に車両がいない場合
            if ~hasForwardVehicle && ~hasBehindVehicle
                s_term = NaN;
                v_term = SpeedLimit;
                


            % ✅ 隣接先行車のみがいる && 場合
            elseif hasForwardVehicle && ~hasBehindVehicle && (sF0   - sEgo0) >= forwardsafetyGap + carLen
                if vF0 >= dsEgo0
                    s_term = NaN;
                    v_term = SpeedLimit;
                else 
                    % v_term = vF0; 
                    continue;
                end
               

            % ✅ 隣接後方車のみがいる場合
            elseif ~hasForwardVehicle && hasBehindVehicle && (sEgo0 - sB0) >= behindsafetyGap + carLen
                if vB0 <= dsEgo0
                    % s_term = Ab + carLen * (1 - rearAxleRatio) + behindsafetyGap + carLen * rearAxleRatio;
                    s_term = NaN;
                    v_term = SpeedLimit;
                else 
                    continue;
                end


            else    % ✅ 先行車・後方車どちらもいる場合   
                frontBoundary  = Af - carLen * rearAxleRatio - forwardsafetyGap;
                behindBoundary = Ab + carLen * (1 - rearAxleRatio) + behindsafetyGap;
                gap = frontBoundary - behindBoundary;                              

                % ギャップが carLen 以上のときのみ終端状態生成
                if gap >= carLen*1.5 && (sF0   - sEgo0) >= forwardsafetyGap + carLen && (sEgo0 - sB0) >= behindsafetyGap + carLen
                    if vF0 >= dsEgo0                 % 自車両の現在速度より隣接前方車両速度が速い
                        s_term = NaN;
                        v_term = SpeedLimit;           % 自車両の速度を前方車両に合わせる
                    elseif vF0 < dsEgo0 && vB0 < dsEgo0        % 自車両の現在速度より隣接後方車両速度が遅い
                        s_term = NaN;
                        v_term = vF0;
                    else
                        continue;
                    end
                else
                    % ギャップ不足 → この dt では終端状態を生成しない
                    continue;
                end
            end
            
            % ===== 隣接レーン先行車との lead gap を計算（TS行と同じ順で積む） =====
            sEgoPred = sEgo0 + dsEgo0 * t;  % 自車の s を等速で予測（必要ならここを別モデルに）
            
            if hasForwardVehicle
                % 「物理的な車間距離」：先行車の後端 - 自車の前端
                % （s が rear axle 基準だと仮定して rearAxleRatio で端点に変換）
                leadGap = (Af - carLen * rearAxleRatio) - (sEgoPred + carLen * (1 - rearAxleRatio));
            else
                leadGap = inf;  % 先行車がいないなら無限大扱い
            end

            % 終端状態の追加
            state = zeros(1,6);
            state(1) = s_term;     % s
            state(2) = v_term;     % v（目標速度）
            % state(3) = 0;        % 必要なら加速度など
            state(4) = d_target;   % d
            % state(5:6) は未使用のまま

            terminalStates = [terminalStates; state]; %#ok<AGROW>
            times          = [times; t];              %#ok<AGROW>
            leadGapsLC = [leadGapsLC; leadGap]; %#ok<AGROW>
        end
    end

    %% 6. Gapの計算
    % 車線変更の終端状態が1つも生成されていなければ「何も出力しない」
    % nTS = size(terminalStates, 1);
    % if nTS == 0
    %     leadGapsLC = [];                % 車線変更できないケース
    % else
    %     leadGapsLC = inf(nTS, 1);     % TS の個数 × 1 の 0 ベクトル
    % end
end
