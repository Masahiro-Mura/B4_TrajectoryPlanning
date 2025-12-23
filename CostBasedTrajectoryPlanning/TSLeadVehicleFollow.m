function [terminalStates, times, leadGaps] = TSLeadVehicleFollow(refPath, laneWidth, forwardsafetyGap, egoState, actorState, carLen, rearAxleRatio, dt)
    frenetStateEgo = global2frenet(refPath, egoState);
    curEgoLane = PredictLane(frenetStateEgo, laneWidth, 0); % 例: 1,2,3,4
    frenetStateActors = global2frenet(refPath, actorState);


    predictedActorLanes = zeros(numel(dt),size(actorState,1));
    for i = 1:size(actorState,1)
        predictedActorLanes(:,i) = PredictLane(frenetStateActors(i,:),laneWidth,dt);
    end



    terminalStates = zeros(numel(dt),6);
    validTS = false(numel(dt),1);
    % prepare leadGaps (one per prediction horizon)
    nH = numel(dt);
    leadGaps = inf(nH,1);

    % fixed offset (先行車の将来Sからどれだけ後ろに置くか)
    offset = carLen*rearAxleRatio + forwardsafetyGap + carLen*(1-rearAxleRatio);

for i = 1:nH
    laneMatch = curEgoLane == predictedActorLanes(i,:)';

    % 先行車候補の将来S（線形予測はここでは「先行車の将来位置を得る」ためにだけ使う）
    future_S = frenetStateActors(:,1) + frenetStateActors(:,2)*dt(i);

    % 現時刻で自車より前方かつ同レーン（必要なら将来判定に変えてOK）
    leadVehicleNow = frenetStateEgo(1) < frenetStateActors(:,1);
    validMask = leadVehicleNow & laneMatch(:);

    if any(validMask)
        future_S(~validMask) = inf;
        [actor_S1, idxRel] = min(future_S);  % このdt(i)での最前方（最も近い）先行車の将来S

        if actor_S1 ~= inf
            % 終端状態（自車の目標S）
            ego_S1 = actor_S1 - offset;

            % 既存の安全条件（必要ならあなたのルールに合わせて調整）
            if frenetStateEgo(1) + offset < actor_S1
                terminalStates(i,:) = [ego_S1 frenetStateActors(idxRel,2) 0 frenetStateActors(idxRel,4:5) 0];
                validTS(i) = true;

                % ★ここが変更点：終端状態ベースのギャップ
                leadGaps(i) = actor_S1 - ego_S1;   % = offset（この設計だと常に一定）
            end
        end
    end
end

terminalStates(~validTS,:) = [];
times = dt(validTS(:));
times = times(:); % 縦ベクトル化
leadGaps = leadGaps(validTS(:));   % ★timesと対応させる


    % for i = 1:nH
    %     laneMatch = curEgoLane == predictedActorLanes(i,:)';


    %     leadVehicle = frenetStateEgo(1) < frenetStateActors(:,1);

    %     % 各 actor の将来の S 座標を単純に現在の速度で線形予測する
    %     % - future_S = S_now + v_now * dt
    %     future_S = frenetStateActors(:,1) + frenetStateActors(:,2)*dt(i);
    %     % 自車の将来 S
    %     sEgoFuture = frenetStateEgo(1) + frenetStateEgo(2)*dt(i);
        
        
    %     % 自車より後方、または別レーンの車は対象外にして inf に置き換える
    %     validMask = leadVehicle & laneMatch(:);
    %     if any(validMask)
    %         sLeadFuture = future_S(validMask);
    %         % 車間距離 = 先行車の将来S - 自車の将来S
    %         gaps = sLeadFuture - sEgoFuture;
    %         gaps(gaps < 0) = 0;
    %         % 最小の車間距離を記録
    %         leadGaps(i) = min(gaps);

    %         % 終端状態の計算は既存のロジック（最も近い先行車を使う）
    %         future_S(~validMask) = inf;
    %         [actor_S1, idxRel] = min(future_S); % 最小の将来S とその index を取得
    %         if actor_S1 ~= inf
    %             % idxRel is index into original actors array
    %             if frenetStateEgo(1) + carLen*(1-rearAxleRatio) + forwardsafetyGap + carLen*rearAxleRatio < actor_S1
    %                 ego_S1 = actor_S1 - carLen*rearAxleRatio - forwardsafetyGap - carLen*(1-rearAxleRatio);
    %                 terminalStates(i,:) = [ego_S1 frenetStateActors(idxRel,2) 0 frenetStateActors(idxRel,4:5) 0];
    %                 validTS(i) = true;
    %             end
    %         end
    %     else
    %         % no valid lead at this horizon -> leave leadGaps(i) = inf
    %     end
    % end

    % terminalStates(~validTS,:) = [];
    % times = dt(validTS(:));
    % times = times(:); % 縦ベクトル化
    % leadGaps = leadGaps(:);
% end

