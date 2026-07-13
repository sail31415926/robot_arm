function [R, a] = shootDir2R(alpha_deg, beta_deg)
%SHOOTDIR2R  拍摄朝向（方位角/俯仰角）→ 相机光学系旋转矩阵（roll = 0）
%
%   拍摄朝向描述规范 v1 —— 写法 A（角度）
%   适合做人机接口 / 云台指令：α↔pan，β↔tilt，语义与 GimbalCommand.msg 一致。
%
%   光轴单位向量由两个世界系角度给出：
%       a = [ cosβ·cosα ;  cosβ·sinα ;  sinβ ]
%   再交给 shootAxis2R 补出画面水平（roll=0）的完整姿态。
%
%   输入
%     alpha_deg  方位角（度），光轴在水平面内的朝向 = atan2(a_y,a_x)
%                取值 (-180, 180]，对应云台 pan
%     beta_deg   俯仰角（度），光轴抬/俯 = asin(a_z)
%                取值 [-90, 90]，对应云台 tilt
%                （注意本机云台硬件 tilt 实际范围约 [-90, +45]）
%
%   输出
%     R  3x3 旋转矩阵（世界 ← 相机光学系），列 = [右 下 光轴]
%     a  3x1 光轴单位向量（拍摄朝向）
%
%   另见 SHOOTAXIS2R, LOOKAT2R, R2SHOOTDIR

    a = [ cosd(beta_deg) * cosd(alpha_deg);
          cosd(beta_deg) * sind(alpha_deg);
          sind(beta_deg) ];

    [R, a] = shootAxis2R(a, alpha_deg);
end
