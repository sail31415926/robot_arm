function [alpha_deg, beta_deg, a] = R2shootDir(R)
%R2SHOOTDIR  相机姿态 → 拍摄朝向（方位角/俯仰角）  —— shootDir2R 的逆
%
%   拍摄朝向描述规范 v1 —— 反解
%   从相机光学系旋转矩阵（或直接一个光轴向量）提取拍摄朝向角。
%   光轴 = 光学系 +z = R 的第 3 列。
%
%   输入
%     R  3x3 旋转矩阵（世界 ← 相机光学系），取第 3 列为光轴；
%        或 3x1 / 1x3 光轴向量（无需归一化）
%
%   输出
%     alpha_deg  方位角（度）= atan2(a_y,a_x)，对应云台 pan
%     beta_deg   俯仰角（度）= asin(a_z)，对应云台 tilt
%     a          3x1 光轴单位向量（拍摄朝向）
%
%   注意：光轴接近正上/正下（β→±90°）时方位角病态，α 数值不可靠
%         （对应万向锁；此时画面朝向应由约定另行给定）。
%
%   另见 SHOOTAXIS2R, SHOOTDIR2R, LOOKAT2R

    if isequal(size(R), [3, 3])
        a = R(:, 3);                 % 光学系 +z = 光轴
    elseif numel(R) == 3
        a = R(:);
    else
        error('R2shootDir:badInput', ...
              '输入须为 3x3 旋转矩阵或 3 维光轴向量。');
    end

    n = norm(a);
    if n < 1e-9
        error('R2shootDir:zeroAxis', '光轴向量长度为零。');
    end
    a = a / n;

    alpha_deg = atan2d(a(2), a(1));
    beta_deg  = asind(max(-1, min(1, a(3))));
end
