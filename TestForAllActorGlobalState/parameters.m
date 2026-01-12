function P = parameters()

P = struct();

% ===== 道路長 ===== %
P.roadLen = 2000;

% ===== 車両パラメータ ===== % 
P.carLen        = 4.7;
P.carWidth      = 1.8;
P.rearAxleRatio = 0.25;
P.laneWidth     = P.carWidth * 2;

% ===== プランナー ===== %
P.replanRate   = 10;            % Hz
P.timeHorizons = 3:0.5:5;       % [s]
P.maxHorizon   = max(P.timeHorizons);

% ===== コスト関数の重み ===== %
P.LatWeight           = 0.1;
P.TimeWeight          = 20;
P.SpeedWeight         = 10;
P.ForwardSafetyWeight = 11;
P.BehindSafetyWeight  = 8;

% ===== 指数関数の定数 ===== %
P.c1 = 2.8;

% ===== 制約 ===== %
P.maxAcceleration = 3;
P.maxCurvature    = 0.02;
P.minVelocity     = 0;

% ===== 目標速度・車間距離 ===== %
P.SpeedLimit       = 33.3;
P.forwardsafetyGap = 25;
P.behindsafetyGap  = 24;

% ===== 周辺車両の目標速度 =====%
P.Vel_actor1 = 35;
P.Vel_actor2 = 35;
P.Vel_actor3 = 33.3;
P.Vel_actor4 = 35;
P.Vel_actor5 = 35;
P.Vel_actor6 = 33.3;

% ===== 動画 ===== %
P.videoFile  = fullfile(pwd, 'simulation_output.mp4');
P.videoFPS   = 10;
P.simStep    = 0;

end
