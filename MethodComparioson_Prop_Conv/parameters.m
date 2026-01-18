function P = parameters()

P = struct();

% ===== 車線数 ===== %
P.laneCount = 2;

% ===== 車両パラメータ ===== % 
P.carLen        = 4.7;
P.carWidth      = 1.8;
P.rearAxleRatio = 0.25;
P.laneWidth     = P.carWidth * 2;

% ===== 道路長 ===== %
P.roadLen = 2000;

% ===== 自車両の初期Frenet位置 ===== % 
P.Ini_s   = 150;  %   s
P.Ini_ds  = 33.3;   %  ds
P.Ini_dds = 0;    % dds
P.Ini_l   = -1.8;  %   l
P.Ini_dl  = 0;    %  dl
P.Ini_ddl = 0;    % ddl

% ===== 自車両の目標速度 ===== % 
P.SpeedLimit       = 33.3;

% ===== 周辺車両の目標速度 =====%
P.Vel_actor1 = 28;
P.Vel_actor2 = 33.3;
P.Vel_actor3 = 35;
P.Vel_actor4 = 33.3;
P.Vel_actor5 = 35;
P.Vel_actor6 = 33.3;

% ===== 周辺車両の台数 =====%
P.Number_of_Actor = 3;



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

% ===== 周辺車両に対する追加コストの重み ===== %
P.MEscoreWeight       = 20;
P.TargetSpeedWeight   = 30; % 後方車両の目標速度乖離コストの重み
P.GapRatioWeight      = 70; % 車間距離比率コストの重み

% ===== 指数関数の定数 ===== %
P.c1 = 2.8;

% ===== 制約 ===== %
P.maxAcceleration = 3;
P.maxCurvature    = 0.02;
P.minVelocity     = 0;

% ===== IDMパラメータ =====%
P.T      = 1.6;   
P.S0     = 2.0; 
P.a      = 0.73;
P.b      = 1.67; 
P.delta  = 4;

% ===== 前後車間距離 ===== %
P.forwardsafetyGap = 25;
P.behindsafetyGap  = 24;

% ===== 動画 ===== %
P.videoFile  = fullfile(pwd, 'simulation_output.mp4');
P.videoFPS   = 10;
P.simStep    = 0;

end
