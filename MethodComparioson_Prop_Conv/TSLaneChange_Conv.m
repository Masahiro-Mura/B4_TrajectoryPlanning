function [terminalStates, times, leadGaps, behindGaps] = TSLaneChange_Conv( ...
    refPath, laneWidth, EgoGlobalState, AllActorGlobalState, carLen, rearAxleRatio, SpeedLimit, dt)
    
    P = parameters();
    nDt = numel(dt);

    %% 自車速度ゼロなら車線変更は行わない
    if EgoGlobalState(5) == 0
        terminalStates = [];
        times          = [];
        leadGaps       = [];
        behindGaps     = [];  
        return;
    end
    
    %% 1. 自車の現Frenet状態 & 現在レーン取得
    EgoFrenet   = global2frenet(refPath, EgoGlobalState);  % [s, ds, dds, l, dl, ddl]
    sEgo0       = EgoFrenet(1);                            % 自車両の現在のs座標
    dsEgo0      = EgoFrenet(2);                            % 自車両の現在のs方向速度
    EgoLaneNum0 = PredictLane(EgoFrenet, laneWidth, 0);    % t=0 でのレーン

    %% 2. 隣接レーン（左・右）
    rawAdjLanes = EgoLaneNum0 + [-1 1];
    validMask   = rawAdjLanes > 0 & rawAdjLanes <= P.laneCount;      % 3車線前提
    AdjLanesIdx = rawAdjLanes(validMask);
    AdjLanesNum = numel(AdjLanesIdx);

    if AdjLanesNum == 0
        terminalStates = [];
        times          = [];
        leadGaps       = [];
        behindGaps     = []; 
        return;
    end

    %% 3. 周辺車両のFrenet座標 & 車線番号
    numActors = size(AllActorGlobalState, 1);
    
    AllActorsFrenetStates = zeros(numActors, 6);
    AllActorsLanes        = zeros(numActors, 1);
    for i = 1:numActors
        AllActorsFrenetStates(i,:) = global2frenet(refPath, AllActorGlobalState(i,:));
        AllActorsLanes(i,1)        = PredictLane(AllActorsFrenetStates(i,:), laneWidth, 0);
    end

    %% 4. 隣接レーンごとの隣接前方車・後方車の「dt秒後のs座標」を計算
    Front_s  = NaN(AdjLanesNum, 1);
    Front_v  = NaN(AdjLanesNum, 1);
    Behind_s = NaN(AdjLanesNum, 1);
    Behind_v = NaN(AdjLanesNum, 1);

    AdjacentForwardVehicle = NaN(AdjLanesNum, nDt);
    AdjacentBehindVehicle  = NaN(AdjLanesNum, nDt);

    for laneIdx = 1:AdjLanesNum
        laneID = AdjLanesIdx(laneIdx);

        if numActors == 0
            continue;
        end

        % この隣接レーン上の車両だけ抜き出し
        idxLane = find(AllActorsLanes == laneID);
        if isempty(idxLane)
            continue;  % このレーンには車両がいない
        end

        sActors = AllActorsFrenetStates(idxLane, 1);

        % --- 先行車候補（時刻0で自車より前） ---
        forwardMask  = sActors > sEgo0;
        forwardIdxes = idxLane(forwardMask);
        if ~isempty(forwardIdxes)
            sF_all = AllActorsFrenetStates(forwardIdxes, 1);
            vF_all = AllActorsFrenetStates(forwardIdxes, 2);
            [~, iMin] = min(sF_all - sEgo0);    % 最も近い先行車
            sF0 = sF_all(iMin);
            vF0 = vF_all(iMin);

            Front_s(laneIdx) = sF0;
            Front_v(laneIdx) = vF0;

            AdjacentForwardVehicle(laneIdx, :) = sF0 + vF0 * dt;  % 等速で予測
        end

        % --- 後方車候補（時刻0で自車より後ろ） ---
        behindMask  = sActors < sEgo0;
        behindIdxes = idxLane(behindMask);
        if ~isempty(behindIdxes)
            sB_all = AllActorsFrenetStates(behindIdxes, 1);
            vB_all = AllActorsFrenetStates(behindIdxes, 2);
            [~, iMax] = max(sB_all);           % sB_all < sEgo0 なので最大値が最も近い後方車
            sB0 = sB_all(iMax);
            vB0 = vB_all(iMax);

            Behind_s(laneIdx) = sB0;
            Behind_v(laneIdx) = vB0;

            AdjacentBehindVehicle(laneIdx, :) = sB0 + vB0 * dt;  % 等速で予測
        end
    end

    %% 5. 完全分岐で終端状態を生成
    terminalStates = [];
    times          = [];
    leadGaps       = [];
    behindGaps     = []; 

    for laneIdx = 1:AdjLanesNum

        sF0 = Front_s(laneIdx);
        vF0 = Front_v(laneIdx);
        sB0 = Behind_s(laneIdx);
        vB0 = Behind_v(laneIdx);

        laneID   = AdjLanesIdx(laneIdx);
        
        P = parameters();
        d_target = ((P.laneCount + 1) / 2 - laneID) * laneWidth;

        for k = 1:nDt
            t = dt(k);
            
            % 隣接レーン上の最も近い先行車，後方車が dt 秒後にいる s 座標
            Af = AdjacentForwardVehicle(laneIdx, k);  % 先行車の s(t)
            Ab = AdjacentBehindVehicle(laneIdx, k);   % 後方車の s(t)

            hasForwardVehicle = ~isnan(Af);
            hasBehindVehicle  = ~isnan(Ab);

            % ==== ロジック====
            s_term = NaN;
            v_term = SpeedLimit;
        
            
            % ======== ギャップ計算部分 =========
            sEgoPred = sEgo0 + dsEgo0 * t;  % 自車の s を等速で予測

            % --- 先行車との gap ---
            if hasForwardVehicle
                % 先行車の後端 - 自車の前端
                leadGap = (Af - carLen * rearAxleRatio) ...
                        - (sEgoPred + carLen * (1 - rearAxleRatio));
                % もしマイナスなら 0 
                if leadGap < 0
                    leadGap = 0;
                end
            else
                leadGap = inf;
            end

            % --- 後方車との gap ---
            if hasBehindVehicle
                % 自車の後端 - 後方車の前端
                behindGap = (sEgoPred - carLen * rearAxleRatio) ...
                          - (Ab + carLen * (1 - rearAxleRatio));
                if behindGap < 0
                    behindGap = 0;
                end
            else
                behindGap = inf;
            end
            % =====================================

            % 終端状態の追加
            state      = zeros(1,6);
            state(1)   = s_term;     % s
            state(2)   = v_term;     % v（目標速度）
            state(4)   = d_target;   % d

            terminalStates = [terminalStates; state];   %#ok<AGROW>
            times          = [times; t];                %#ok<AGROW>
            leadGaps       = [leadGaps;   leadGap];     %#ok<AGROW>
            behindGaps     = [behindGaps; behindGap];   %#ok<AGROW>
        end
    end
end
