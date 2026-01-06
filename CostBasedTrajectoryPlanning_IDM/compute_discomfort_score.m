function Me_score = compute_discomfort_score(acc, jerk)
    % 論文1の統計モデル(式5)に基づく不快感指数算出
    % acc: 周辺車両の加速度 [m/s^2] (絶対値)
    % jerk: 周辺車両の加躍度 [m/s^3] (絶対値)

    % 論文1の典型的な係数を使用
    beta_A = 2.297;  % 加速度の係数
    beta_J = -0.052; % ジャークの係数 (高いジャーク/短いパルスは不快感を下げる傾向)
    
    % 不快感スコア (ME_norm) の計算
    Me_score = beta_A * abs(acc) + beta_J * abs(jerk);
    
    % スコアが負にならないようガード
    Me_score = max(0, Me_score);
end