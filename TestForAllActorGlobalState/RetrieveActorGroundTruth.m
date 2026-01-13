% function [AllActorGlobalState, futureTrajectory, isRunning] = RetrieveActorGroundTruth...
%     (scenario, futureTrajectory, replanRate, maxHorizon)
% 
%     numActor = numel(futureTrajectory);
%     AllActorGlobalState = zeros(numActor,6);
% 
%     % Number of states needed
%     minUpdateSteps = (1/replanRate)/scenario.SampleTime;
%     maxNumStates = maxHorizon/scenario.SampleTime;
%     statesNeeded = max(maxNumStates-size(futureTrajectory(1).Trajectory,1),minUpdateSteps);
% 
% 
%     for i = 1:statesNeeded
%         % Advance the scenario
%         isRunning = advance(scenario);
% 
%         % Retrieve actor information
%         poses = scenario.actorPoses;
%         for j = 1:numActor
%             actIdx = j+1;
%             xy = poses(actIdx).Position(1:2);
%             v  = norm(poses(actIdx).Velocity,2);
%             th = atan2(poses(actIdx).Velocity(2),poses(actIdx).Velocity(1));
%             k = poses(actIdx).AngularVelocity(3)/v/180*pi;
%             % Assume acceleration = 0
%             futureTrajectory(j).Trajectory(i,:) = [xy th k v 0];
%         end
%         if ~isRunning
%             break;
%         end
%     end
% 
%     % Reorder the states
%     for i = 1:numActor
%         futureTrajectory(i).Trajectory = circshift(futureTrajectory(i).Trajectory,-statesNeeded,1);
%         AllActorGlobalState(i,:) = futureTrajectory(i).Trajectory(1,:);
%     end
% end

function [AllActorGlobalState, futureTrajectory, isRunning] = RetrieveActorGroundTruth...
    (scenario, futureTrajectory, maxHorizon, AllActorGlobalState)
    % 引数に AllActorGlobalState を追加しました

    numActor = numel(futureTrajectory);
    dt = scenario.SampleTime;
    isRunning = true;
    
    P = parameters();
    % 周辺車両の到達判定 (AllActorGlobalState(:,1) が 各車のX座標)
    if any(AllActorGlobalState(:,1) >= P.roadLen)
        isRunning = false;
    end

    if ~isRunning
        return;
    end


    for j = 1:numActor
        % --- IDMで更新された現在の状態を取得 ---
        x0  = AllActorGlobalState(j,1);
        y0  = AllActorGlobalState(j,2);
        th  = AllActorGlobalState(j,3); % Yaw (rad)
        v   = AllActorGlobalState(j,5); % Speed (m/s)
        k   = AllActorGlobalState(j,4); % Curvature
        acc = AllActorGlobalState(j,6); % Acceleration

        % --- 未来の全ステップを「現在の速度で等速運動」として計算 ---
        % 注意: すでに futureTrajectory(j).Trajectory のサイズが決まっていると仮定
        numSteps = round(maxHorizon/dt); 
        newTraj = zeros(numSteps, 6);

        for i = 1:numSteps
            t_future = (i-1) * dt;

            % 等速直線運動の仮定
            futureX = x0 + v * t_future * cos(th);
            futureY = y0 + v * t_future * sin(th);

            % [x, y, theta, kappa, speed, accel]
            newTraj(i, :) = [futureX, futureY, th, k, v, acc];
        end
        % 予測軌道を上書き
        futureTrajectory(j).Trajectory = newTraj;
    end
end