function [a_idm, s_star] = IDM_normal(v_follower, v_leader, gap, v_target)
    % パラメータ設定
    P = parameters();
    T      = P.T;    % 先行車両が減速しても反応しなくて大丈夫と感じる時間
    S0     = P.S0;   % どの速度帯でも最低限空けておきたい車間距離
    a      = P.a;    % 最大快適加速度
    b      = P.b;    % 最大快適減速度
    delta  = P.delta;

    dlta_v_n = v_follower - v_leader; 

    d = gap;
    v = v_follower;

    %% 【意識領域】 標準IDM（枠外）
    s_star = S0 + v*T + (v*dlta_v_n)/(2*sqrt(a*b));
    a_idm = a * (1 - (v/v_target)^delta - (s_star/d)^2);
    
end