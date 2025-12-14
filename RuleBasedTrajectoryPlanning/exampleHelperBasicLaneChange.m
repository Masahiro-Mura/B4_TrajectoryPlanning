% function [terminalStates, times] = exampleHelperBasicLaneChange(refPath, laneWidth, egoState, dt)
% 
%     if egoState(5) == 0
%         terminalStates = [];
%         times = [];
%     else
%         % Convert ego state to Frenet coordinates
%         frenetState = global2frenet(refPath, egoState);
% 
%         % Get current lane
%         curLane = exampleHelperPredictLane(frenetState, laneWidth, 0);
% 
%         % Determine if future lanes are available
%         adjacentLanes = curLane+[-1 1];
%         % validLanes = adjacentLanes > 0 & adjacentLanes <= 4; % 4車線用
%         validLanes = adjacentLanes > 0 & adjacentLanes <= 3; % 3車線用
% 
%         % Calculate lateral deviation for adjacent lanes
%         % lateralOffset = (2-adjacentLanes(validLanes)+.5)*laneWidth; % 4車線用
%         lateralOffset = (adjacentLanes(validLanes)-2)*laneWidth; % 3車線用
%         numLane = nnz(validLanes);
% 
%         % Calculate terminal states
%         terminalStates = zeros(numLane*numel(dt),6);
%         terminalStates(:,1) = nan;
%         terminalStates(:,2) = egoState(5);
%         terminalStates(:,4) = repelem(lateralOffset(:),numel(dt),1);
%         times = repmat(dt(:),numLane,1);
%     end
% end



function [terminalStates, times] = exampleHelperBasicLaneChange( ...
    refPath, laneWidth, forwardsafetyGap, behindsafetyGap, ...
    egoState, actorState, carLen, rearAxleRatio, dt)

    if egoState(5) == 0
        terminalStates = []; 
        times = []; 
        return;
    end

    % Frenet states
    frEgo = global2frenet(refPath, egoState);      % 1x6
    frAct = global2frenet(refPath, actorState);    % Nx6
    numActors = size(frAct,1);

    % 自車の現在レーンと車線変更候補レーン
    curLane   = exampleHelperPredictLane(frEgo, laneWidth, 0);
    candLanes = curLane + [-1 1];
    candLanes = candLanes(candLanes > 0 & candLanes <= 3);   % 3車線用

    % 各アクターの将来レーンを事前計算（時間×アクター）
    predictedActorLanes = zeros(numel(dt), numActors);
    for k = 1:numActors
        predictedActorLanes(:,k) = exampleHelperPredictLane(frAct(k,:), laneWidth, dt);
    end

    terminalStates = [];
    times = [];

    for L = candLanes
        % 目標レーンの横方向オフセット（中央レーン=2）
        latOffset = (L - 2) * laneWidth;

        for ti = 1:numel(dt)
            t = dt(ti);

            % === 時刻 t にレーン L にいる車両を抽出 ===
            laneMatch = (predictedActorLanes(ti,:)' == L);
            if ~any(laneMatch)
                % ---- Case 0: 前後に車がいない → 従来法 ----
                S1 = frEgo(1) + frEgo(2)*t;
                terminalStates(end+1,:) = [S1 egoState(5) 0 latOffset 0 0];
                times(end+1,1) = t;
                continue;
            end

            actF  = frAct(laneMatch,:);                 % Mx6
            futS  = actF(:,1) + actF(:,2)*t;            % 将来S
            egoS  = frEgo(1) + frEgo(2)*t;              % t秒後の自車S

            % 前方 / 後方判定（未来の自車S基準）
            frontIdx = futS > egoS;
            backIdx  = futS < egoS;

            hasFront = any(frontIdx);
            hasBack  = any(backIdx);

            neededGap = forwardsafetyGap + behindsafetyGap + carLen;

            if hasFront && ~hasBack
                % ---- Case 1: 前方のみ ----
                S_front = min(futS(frontIdx));
                % 前方車とのforwardsafetyGapだけ確保して終端生成
                egoS1 = S_front - forwardsafetyGap - carLen*(1-rearAxleRatio);
                if egoS1 > egoS   % あまりにも後退する終端は除外したければ条件を外してもOK
                    terminalStates(end+1,:) = [egoS1 egoState(5) 0 latOffset 0 0];
                    times(end+1,1) = t;
                end

            elseif ~hasFront && hasBack
                % ---- Case 2: 後方のみ ----
                S_back = max(futS(backIdx));
                egoS1  = S_back + behindsafetyGap + carLen*(1-rearAxleRatio);
                terminalStates(end+1,:) = [egoS1 egoState(5) 0 latOffset 0 0];
                times(end+1,1) = t;

            elseif hasFront && hasBack
                % ---- Case 3: 前後両方 ----
                S_front = min(futS(frontIdx));
                S_back  = max(futS(backIdx));

                if S_front - S_back > neededGap
                    % 前後両方に safetyGap + 車長 分の余裕があるので間に挿入
                    egoS1 = S_back + behindsafetyGap + carLen*(1-rearAxleRatio);
                    terminalStates(end+1,:) = [egoS1 egoState(5) 0 latOffset 0 0];
                    times(end+1,1) = t;
                end
                % ギャップが足りない場合は終端生成なし
            else
                % frontIdx/backIdx が数値誤差で壊れた場合などの保険
                S1 = frEgo(1) + frEgo(2)*t;
                terminalStates(end+1,:) = [S1 egoState(5) 0 latOffset 0 0];
                times(end+1,1) = t;
            end
        end
    end
end
