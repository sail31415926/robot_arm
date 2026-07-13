function [R, a, alpha_deg, beta_deg] = lookAt2R(p, T)
%LOOKAT2R  相机位置 + 被摄目标点 → 相机光学系旋转矩阵（roll = 0）
%
%   拍摄朝向描述规范 v1 —— 写法 B（看向目标 look-at）
%   适合做实拍指令：相机始终对准被摄主体点 T，画面保持水平。
%
%   光轴由几何关系确定（随相机位置变化）：
%       a(p) = (T - p) / ‖T - p‖
%   再交给 shootAxis2R 补出画面水平（roll=0）的完整姿态。
%
%   输入
%     p  3x1（或 1x3）相机位置，世界系，单位 m
%     T  3x1（或 1x3）被摄目标点，世界系，单位 m
%
%   输出
%     R          3x3 旋转矩阵（世界 ← 相机光学系），列 = [右 下 光轴]
%     a          3x1 光轴单位向量（拍摄朝向，由 p 指向 T）
%     alpha_deg  等价方位角（度）= atan2(a_y,a_x)，对应云台 pan
%     beta_deg   等价俯仰角（度）= asin(a_z)，对应云台 tilt
%
%   另见 SHOOTAXIS2R, SHOOTDIR2R, R2SHOOTDIR

    p = p(:);
    T = T(:);
    d = T - p;

    if norm(d) < 1e-9
        error('lookAt2R:coincident', ...
              '相机位置与目标点重合，拍摄朝向无法确定。');
    end

    a = d / norm(d);
    alpha_deg = atan2d(a(2), a(1));
    beta_deg  = asind(max(-1, min(1, a(3))));

    [R, a] = shootAxis2R(a, alpha_deg);
end
