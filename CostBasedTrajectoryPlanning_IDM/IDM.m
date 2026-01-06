function [a_idm, jerk] = IDM(v_follower, v_leader, gap, v0, a_leader, a_prev, dt)
    % --- パラメータ設定 ---
    T = 2; 
    s0 = 2.0; 
    a_max = 0.5;
    b_comf = 0.5;
    delta = 4;

    dv_idm = v_follower - v_leader; 
    dv_rel = v_leader - v_follower; 

    d = max(0.1, gap);
    v = max(0.1, v_follower);

    % --- 閾値（アクションポイント）の計算 ---
    OPDV = 0.062 * d;
    CLDV = -0.020 * d;
    ABX = 2.070 + 2.994 * sqrt(v);
    SDX = 2.892 + 6.341 * sqrt(v);

    % --- 領域判定 ---
    is_unconscious = (d > ABX && d < SDX) && (dv_rel > CLDV && dv_rel < OPDV);

    if ~is_unconscious
        %% 【意識領域】 標準IDM
        s_star = s0 + max(0, v*T + (v*dv_idm)/(2*sqrt(a_max*b_comf)));
        a_idm = a_max * (1 - (v/v0)^delta - (s_star/d)^2);
    else
        %% 【無意識領域】 螺旋挙動（4プロセス）
        center_gap = (ABX + SDX) / 2;
        if dv_rel <= 0
            if d > center_gap
                term = dv_rel^4 - 8*(d - ABX)*a_leader*dv_rel^2;
                a_idm = (dv_rel^2 + sqrt(max(0, term))) / (4*(ABX - d));
                a_idm = max(a_idm, -0.6);
            else
                a_idm = min(CLDV^2 / (2 * (d - ABX)), 0.6);
            end
        else
            if d < center_gap
                term = dv_rel^4 - 8*(d - SDX)*a_leader*dv_rel^2;
                a_idm = (dv_rel^2 + sqrt(max(0, term))) / (4*(SDX - d));
                a_idm = min(a_idm, 0.6);
            else
                a_idm = max(OPDV^2 / (2 * (d - SDX)), -0.6);
            end
        end
    end

    % --- ジャークの計算 ---
    % ジャーク = (現在の加速度 - 前回の加速度) / 時間刻み
    jerk = (a_idm - a_prev) / dt;
end