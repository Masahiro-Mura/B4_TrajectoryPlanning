function [terminalStates, times, leadGaps, behindGaps] = TSLeadVehicleFollow( ...
    refPath, laneWidth, forwardsafetyGap, ...
    egoState, actorState, carLen, rearAxleRatio, dt)

    %% --- Frenet変換 ---
    frenetStateEgo    = global2frenet(refPath, egoState);
    curEgoLane        = PredictLane(frenetStateEgo, laneWidth, 0);
    frenetStateActors = global2frenet(refPath, actorState);

    nH        = numel(dt);
    numActors = size(actorState,1);

    %% --- 予測レーン ---
    predictedActorLanes = zeros(nH, numActors);
    for i = 1:numActors
        predictedActorLanes(:,i) = PredictLane(frenetStateActors(i,:), laneWidth, dt);
    end

    %% --- 初期化 ---
    terminalStates = zeros(nH,6);
    validTS        = false(nH,1);

    leadGaps   = inf(nH,1);
    behindGaps = inf(nH,1);

    sEgo0 = frenetStateEgo(1);
    sAct0 = frenetStateActors(:,1);
    vAct0 = frenetStateActors(:,2);

    

    %% --- 時刻 t ループ ---
    for i = 1:nH
        t = dt(i);

        % ★ 上のコードと同じロジック：
        % ego より前にいる & 将来レーン一致
        laneMatch = (curEgoLane == predictedActorLanes(i,:)');
        leadMask  = (sAct0 > sEgo0) & laneMatch(:);

        if any(leadMask)

            % 将来の s
            future_S = sAct0 + vAct0 * t;
            future_S(~leadMask) = inf;

            % 最も近い先行車
            [actor_S1, idxRel] = min(future_S);

            if isfinite(actor_S1)

                % ★ 上のコードと同じ：ego_S1 = actor_S1 - offset
                ego_S1 = actor_S1 - forwardsafetyGap - carLen;

                % 安全条件
                % if sEgo0 + offset < actor_S1

                    % ★ 終端状態生成
                    terminalStates(i,:) = [ego_S1, ...
                                            frenetStateActors(idxRel,2), ...   % 先行車速度
                                            0, ...
                                            frenetStateActors(idxRel,4:5), ...
                                            0];

                    validTS(i) = true;

                    % leadGaps
                    leadGaps(i) = actor_S1 - ego_S1;

                    % --- 後方車のギャップ計算 ---
                    behindMask = (sAct0 < sEgo0) & laneMatch(:);
                    if any(behindMask)
                        futureBehindS = sAct0(behindMask) + vAct0(behindMask) * t;
                        gapsBehind = ego_S1 - futureBehindS;
                        gapsBehind(gapsBehind < 0) = 0;
                        behindGaps(i) = min(gapsBehind);
                    end
                % end
            end
        end
    end

    %% --- valid のみ残す ---
    terminalStates(~validTS,:) = [];
    times = dt(validTS);
    times = times(:);
    leadGaps   = leadGaps(validTS);
    behindGaps = behindGaps(validTS);
end
