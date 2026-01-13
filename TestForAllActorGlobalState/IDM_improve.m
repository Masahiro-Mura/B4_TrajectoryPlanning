function [a_idm, jerk] = IDM_improve(v_follower, v_leader, gap, v_target, a_leader, a_prev, dt)
    % パラメータ設定
    T      = 1.6;   % 先行車両が減速しても反応しなくて大丈夫と感じる時間
    S0     = 2.0;   % どの速度帯でも最低限空けておきたい車間距離
    a      = 0.73;  % 最大快適加速度
    b      = 1.67;  % 最大快適減速度
    delta  = 4;

    dlta_v_n = v_leader - v_follower; 

    d = gap;
    v = v_follower;

    % 閾値（アクションポイント）の計算
    OPDV = 0.062 * d;
    CLDV = -0.020 * d;
    ABX  = 2.070 + 2.994 * sqrt(v);
    SDX  = 2.892 + 6.341 * sqrt(v);

    % 意識領域の判定
    is_unconscious = (d > ABX && d < SDX) && (dlta_v_n > CLDV && dlta_v_n < OPDV);

    % 意識領域または無意識領域であっても20%の確率でIDMを選択する
    if ~is_unconscious || rand < 0.2 
        %% 【意識領域】 標準IDM（枠外）
        s_star = S0 + v*T - (v*dlta_v_n)/(2*sqrt(a*b));
        a_idm = a * (1 - (v/v_target)^delta - (s_star/d)^2);
        disp('v');
        disp(v);
        disp('v_target');
        disp(v_target);
        disp('s_star');
        disp(s_star);
        disp('d');
        disp(d);
    else
        %% 【無意識領域】 螺旋挙動（枠内）
        if dlta_v_n <= 0
            % 接近(リーダーより後方車両の方が速い)
            if d > ABX
                % A ➡ B
                term = dlta_v_n^4 - 8*(d - ABX)*a_leader*dlta_v_n^2;
                a_idm = (dlta_v_n^2 + sqrt(max(0, term))) / (4*(ABX - d));
            else
                % B ➡ C
                a_idm = OPDV^2 / (2 * (d - SDX));
            end
            a_idm = max(a_idm, -0.6);  
        
        else
            % 離反(リーダーより後方車両の方が遅い)
            if d < SDX
                % C ➡ D
                term = dlta_v_n^4 - 8*(d - SDX)*a_leader*dlta_v_n^2;
                a_idm = (dlta_v_n^2 + sqrt(max(0, term))) / (4*(SDX - d));
            else
                % D ➡ A
                a_idm = CLDV^2 / (2 * (d - ABX));
            end
            a_idm = min(a_idm, 0.6);  
        end
    end
    % ジャークの計算
    jerk = (a_idm - a_prev) / dt;
end