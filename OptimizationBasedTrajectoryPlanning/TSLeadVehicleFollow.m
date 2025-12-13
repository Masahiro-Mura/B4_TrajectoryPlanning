function [terminalStates, times] = TSLeadVehicleFollow(refPath, laneWidth, forwardsafetyGap, egoState, actorState, carLen, rearAxleRatio, dt)
    frenetStateEgo = global2frenet(refPath, egoState);
    curEgoLane = PredictLane(frenetStateEgo, laneWidth, 0); % 例: 1,2,3,4
    frenetStateActors = global2frenet(refPath, actorState);


    predictedActorLanes = zeros(numel(dt),size(actorState,1));
    for i = 1:size(actorState,1)
        predictedActorLanes(:,i) = PredictLane(frenetStateActors(i,:),laneWidth,dt);
    end

    

    terminalStates = zeros(numel(dt),6);
    validTS = false(numel(dt),1); 
    for i = 1:numel(dt)
        laneMatch = curEgoLane == predictedActorLanes(i,:)';


        leadVehicle = frenetStateEgo(1) < frenetStateActors(:,1);

        % 各 actor の将来の S 座標を単純に現在の速度で線形予測する
        % - future_S = S_now + v_now * dt
        future_S = frenetStateActors(:,1) + frenetStateActors(:,2)*dt(i);
        
        
        % 自車より後方、または別レーンの車は対象外にして inf に置き換える
        future_S(~leadVehicle | ~laneMatch) = inf;
        [actor_S1, idx] = min(future_S); % 最小の将来S とその index を取得

        % 有効な先行車が存在するかチェック
        if actor_S1 ~= inf       % 同一車線に前方車がいたら
            if frenetStateEgo(1) + carLen*(1-rearAxleRatio) + forwardsafetyGap + carLen*rearAxleRatio < actor_S1
                ego_S1 = actor_S1 - carLen*rearAxleRatio - forwardsafetyGap - carLen*(1-rearAxleRatio) ; % 終端位置 S は (先行車の将来 S) - forwardsafetyGap に設定
                terminalStates(i,:) = [ego_S1 frenetStateActors(idx,2) 0 frenetStateActors(idx,4:5) 0];
                validTS(i) = true;
            end
        end
    end

    terminalStates(~validTS,:) = [];
    times = dt(validTS(:));
    times = times(:); % 縦ベクトル化
end

