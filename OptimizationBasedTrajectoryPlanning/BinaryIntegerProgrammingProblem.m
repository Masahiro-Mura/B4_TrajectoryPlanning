%% ===== 軌道選定における二値整数計画問題 ===== %%
function BinaryIntegerProgrammingProblem
    
    % 加速度制約
    accViolated  = any(abs(globalTrajectory(i).Trajectory(:,6)) > abs(maxAcceleration));


    % 曲率制約
    curvViolated = any(abs(globalTrajectory(i).Trajectory(:,4)) > abs(maxCurvature));


    % 速度制約
    velViolated  = any(globalTrajectory(i).Trajectory(:,5) < minVelocity);

    
    % 選定できる軌道を1本に限定
    N = numel(costTS);      % 候補軌道の本数

    % === 決定変数 x_i はすべて整数（0 or 1） ===
    intcon = 1:N;           % 整数に指定 1~N
    
    lb = zeros(N,1);        % x_i >= 0 (lower bound)
    ub = ones(N,1);         % x_i <= 1 (upper bound)
    % → これで x_i ∈ {0,1}
    
    % === 「選択できる軌道は1本だけ」の制約 ===
    Aeq = ones(1,N);        % [1 1 1 ... 1]
    beq = 1;                % 合計 = 1


    % 車間距離制約
    egoFrenetTraj = global2frenet(refPath, globalTraj(i).Trajectory);
    actorFrenetTraj{j} = global2frenet(refPath, futureTrajectory(j).Trajectory);

    % 先行車両
    gap_front(k) = s_actor(k) - s_ego(k);
    min(gap_front) >= forwardsafetyGap
    
    % 隣接前方車両
    gap_adj_front(k) = s_actor(k) - s_ego(k);
    min(gap_adj_front) >= forwardsafetyGap

    % 隣接後方車両
    gap_adj_behind(k) = s_ego(k) - s_actor(k);
    min(gap_adj_behind) >= behindsafetyGap

    