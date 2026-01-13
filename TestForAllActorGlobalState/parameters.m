function P = parameters()

P = struct();


% ===== 車両パラメータ ===== % 
P.carLen        = 4.7;
P.carWidth      = 1.8;
P.rearAxleRatio = 0.25;
P.laneWidth     = P.carWidth * 2;

% ===== 道路長 ===== %
P.roadLen = 1000;

% ===== 自車両の初期位置 ===== % 
P.Ini_s   = 67;
P.Ini_ds  = 33.3;
P.Ini_dds = 0;
P.Ini_l   = P.laneWidth*0;
P.Ini_dl  = 0;
P.Ini_ddl = 0;

% ===== 自車両の目標速度 ===== % 
P.SpeedLimit       = 33.3;

% ===== 周辺車両の目標速度 =====%
P.Vel_actor1 = 35;
P.Vel_actor2 = 35;
P.Vel_actor3 = 33.3;
P.Vel_actor4 = 35;
P.Vel_actor5 = 35;
P.Vel_actor6 = 33.3;

% ===== 周辺車両の台数 =====%
P.Number_of_Actor = 6;



% ===== 描画俯瞰位置 =====%
P.ViewLocation = P.carLen*13;
P.ViewHeight   = 20;
P.ViewPitch    = 22;

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

% ===== 前後車間距離 ===== %
P.forwardsafetyGap = 25;
P.behindsafetyGap  = 24;

% ===== 動画 ===== %
P.videoFile  = fullfile(pwd, 'simulation_output.mp4');
P.videoFPS   = 10;
P.simStep    = 0;

end
