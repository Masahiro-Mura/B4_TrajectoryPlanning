function isValid = EvaluateTrajectory(globalTrajectory, maxAcceleration, maxCurvature, minVelocity)
    % 加速度・曲率・速度の制約違反を除外
    isValid = true(numel(globalTrajectory),1); % 初期状態として最初はすべてTrueと設定
    for i = 1:numel(globalTrajectory) % すべての軌道について加速度・曲率・速度の制約をチェック

        % 加速度制約
        accViolated  = any(abs(globalTrajectory(i).Trajectory(:,6)) > abs(maxAcceleration));

        % 曲率制約
        curvViolated = any(abs(globalTrajectory(i).Trajectory(:,4)) > abs(maxCurvature));

        % 速度制約
        velViolated  = any(globalTrajectory(i).Trajectory(:,5) < minVelocity);



        isValid(i) = ~accViolated && ~curvViolated && ~velViolated;
    end
end


