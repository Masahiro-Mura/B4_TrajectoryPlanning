function [a_idm, jerk] = Normal_IDM(v_follower, v_leader, gap, v0, a_prev, dt)
    % --- 一般的なIDMパラメータ (論文の値を参照) ---
    T     = 1.6;      % 希望時間車間距離 [s] [cite: 312]
    s0    = 2.0;      % 停止時車間距離 [m] [cite: 312]
    a_max = 0.73;     % 最大加速速度 [m/s^2] [cite: 312]
    b_comf = 1.67;    % 快適な減速度 [m/s^2] [cite: 312]
    delta = 4;        % 加速指数 [cite: 281]

    % 相対速度 (follower - leader)
    dv = v_follower - v_leader; 
    
    % 希望車間距離 s* の計算 [cite: 284, 285]
    % s* = s0 + v*T + (v*dv) / (2*sqrt(a*b))
    s_star = s0 + max(0, v_follower * T + (v_follower * dv) / (2 * sqrt(a_max * b_comf)));

    % 加速度の計算 [cite: 281, 282]
    % a = a_max * [ 1 - (v/v0)^delta - (s*/gap)^2 ]
    % gapが極端に小さい場合の不連続性を防ぐため max(0.1, gap) を使用
    a_idm = a_max * (1 - (v_follower / v0)^delta - (s_star / max(0.1, gap))^2);

    % ジャークの計算 (現在の加速度と前回の加速度の差分から算出)
    jerk = (a_idm - a_prev) / dt;
end