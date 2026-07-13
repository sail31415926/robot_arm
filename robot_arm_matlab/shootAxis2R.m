function [R, a] = shootAxis2R(a, alphaHint_deg)
%SHOOTAXIS2R  拍摄光轴向量 → 相机光学系旋转矩阵（横滚锁定 0，画面水平）
%
%   ============================================================
%   拍摄朝向描述规范 v1（roll = 0）—— 核心基础件
%   ============================================================
%   本函数是"拍摄朝向"描述方法的唯一底层实现，shootDir2R / lookAt2R
%   都调用它，保证同一个光轴 a 得到唯一一致的相机姿态。
%
%   关键简化：横滚 ψ 钉死为 0（画面始终水平），于是相机姿态不再是
%   自由的 3-DOF 旋转，而是"拍摄朝向 a"的单值函数——只剩 2 个自由度。
%
%   坐标约定
%     世界系      : Z 朝上，zW = [0;0;1]
%     相机光学系  : 遵循 ROS 约定
%                     +z = 光轴 = 拍摄朝向（出镜头方向）
%                     +x = 画面右
%                     +y = 画面下
%
%   水平约束（roll = 0）
%     相机右轴 x_c 恒垂直于世界竖直方向 → 画面地平线水平。
%     构造：
%         z_c = a
%         x_c = normalize( z_c × ref )      % forward × up = right
%         y_c = z_c × x_c                    % down
%         R   = [x_c  y_c  z_c]              % 世界 ← 相机光学系
%     其中 ref 默认取世界上向量 zW。
%
%   退化情况（光轴接近正上/正下，a ∥ zW，即俯仰角 β → ±90°）
%     此时 z_c × zW ≈ 0，画面水平失去参考（万向锁）。改用方位角
%     alphaHint 指定的水平方向作为参考 ref，使画面朝向仍由 α 唯一确定，
%     并可认为"此姿态横滚无定义、由 α 约定"。
%
%   输入
%     a             3x1（或 1x3）光轴向量，无需已归一化；表示拍摄朝向
%     alphaHint_deg （可选）方位角提示，单位 度；仅在退化情况下使用，
%                   缺省时由 atan2d(a_y, a_x) 推出（退化时该值不可靠，
%                   建议由调用方显式传入）
%
%   输出
%     R  3x3 旋转矩阵，列 = [x_c y_c z_c] 在世界系下的坐标
%        （即 R_world_from_cameraOptical，把光学系向量转到世界系）
%     a  3x1 归一化后的光轴单位向量
%
%   另见 SHOOTDIR2R, LOOKAT2R, R2SHOOTDIR

    a = a(:);
    n = norm(a);
    if n < 1e-9
        error('shootAxis2R:zeroAxis', '光轴向量长度为零，无法确定拍摄朝向。');
    end
    a = a / n;

    if nargin < 2 || isempty(alphaHint_deg)
        alphaHint_deg = atan2d(a(2), a(1));
    end

    zW = [0; 0; 1];

    % 判断是否接近竖直（与世界上向量近似平行）
    VERT_TOL = 1e-6;                 % |a×zW| 小于此值视为退化
    ref = zW;
    if norm(cross(a, zW)) < VERT_TOL
        % 退化：用方位角对应的水平方向作参考，保证 α 仍决定画面朝向
        ref = [cosd(alphaHint_deg); sind(alphaHint_deg); 0];
    end

    z_c = a;
    x_c = cross(z_c, ref);           % forward × up = right（右轴）
    x_c = x_c / norm(x_c);
    y_c = cross(z_c, x_c);           % 已正交且单位（下轴）

    R = [x_c, y_c, z_c];
end
