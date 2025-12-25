function [curState, futureTrajectory, isRunning] = RetrieveActorGroundTruth(scenario, futureTrajectory, replanRate, maxHorizon)

    numActor = numel(futureTrajectory);
    curState = zeros(numActor,6);

    % Number of states needed
    minUpdateSteps = (1/replanRate)/scenario.SampleTime;
    maxNumStates = maxHorizon/scenario.SampleTime;
    statesNeeded = max(maxNumStates-size(futureTrajectory(1).Trajectory,1),minUpdateSteps);


    for i = 1:statesNeeded
        % Advance the scenario
        isRunning = advance(scenario);

        % Retrieve actor information
        poses = scenario.actorPoses;
        for j = 1:numActor
            actIdx = j+1;
            xy = poses(actIdx).Position(1:2);
            v  = norm(poses(actIdx).Velocity,2);
            th = atan2(poses(actIdx).Velocity(2),poses(actIdx).Velocity(1));
            k = poses(actIdx).AngularVelocity(3)/v/180*pi;
            % Assume acceleration = 0
            futureTrajectory(j).Trajectory(i,:) = [xy th k v 0];
        end
        if ~isRunning
            break;
        end
    end

    % Reorder the states
    for i = 1:numActor
        futureTrajectory(i).Trajectory = circshift(futureTrajectory(i).Trajectory,-statesNeeded,1);
        curState(i,:) = futureTrajectory(i).Trajectory(1,:);
    end
end

