function Me_score = compute_discomfort_score_strict(acc, jerk, direction, profile)
    % 論文1 (de Winkel et al., 2023) Table 2 の Combined Model に基づく厳密な実装
    % -------------------------------------------------------------------------
    % acc      : 加速度ピーク [m/s^2] (絶対値)
    % jerk     : ジャークピーク [m/s^3] (絶対値)
    % direction: 0 = Forward(前後), 1 = Lateral(横), 2 = Backward(後退)
    % profile  : 0 = Sinusoidal(正弦波), 1 = Triangular(三角波)
    % -------------------------------------------------------------------------

    % 1. 基本係数 (Combined Model)
    beta_0 = 0.407;  % 切片
    beta_A = 2.297;  % 加速度主効果
    beta_J = -0.052; % ジャーク主効果

    % 2. カテゴリカル変数の係数
    beta_d_lat  = -0.177; % 方向(Lateral)
    beta_d_back = 0.231;  % 方向(Backward)
    beta_p_tri  = -0.111; % 波形(Triangular)

    % 3. 相互作用項の係数 (Interaction terms)
    beta_Ad_lat  = -0.245; % Accel x Lateral
    beta_Ad_back = 0.043;  % Accel x Backward
    beta_Ap_tri  = -0.248; % Accel x Triangular
    beta_Jp_tri  = 0.053;  % Jerk x Triangular

    % --- ダミー変数の設定 ---
    d_lat  = (direction == 1);
    d_back = (direction == 2);
    p_tri  = (profile == 1);

    % --- 厳密なモデル計算 (式5の完全再現) ---
    Me_score = beta_0 + ...
               beta_A * acc + ...
               beta_J * jerk + ...
               beta_d_lat * d_lat + ...
               beta_d_back * d_back + ...
               beta_p_tri * p_tri + ...
               beta_Ad_lat * (acc * d_lat) + ...
               beta_Ad_back * (acc * d_back) + ...
               beta_Ap_tri * (acc * p_tri) + ...
               beta_Jp_tri * (jerk * p_tri);

    % スコアの最小値ガード
    Me_score = max(0, Me_score);
end