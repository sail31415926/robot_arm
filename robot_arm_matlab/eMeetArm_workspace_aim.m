function model = eMeetArm_workspace_aim()
%% 机械臂"定姿态"可达域 → 上层智能体用的可达性判定器
% 具身智能设备：上层智能体生成机械臂目标位置，需要判定该位置是否可达。
% 相机末端（camera_optical_frame 的 +z 为光轴）保持固定朝向拍摄
% （光轴朝前 + 画面水平 roll=0），求"位置能到 且 姿态满足"的位置集合，
% 并把它固化成一个可供上层调用的可达性判定器（reachability oracle）。
%
% 管线（对应具身智能三层做法）：
%   ① 离线建图：IK 扫出定姿态可达点云 → 占据栅格 occ
%   ② 在线廉价约束：拟合"内接椭球" (x-c)'A(x-c)<=1，椭球内 => 保证可达（可微护栏）
%   ③ 在线精确确认：占据查询 reachIsReachable，不可达时 reachProject 拉回最近可达点
% 产物 eMeetArm_reach_model.mat 供 reachIsReachable / reachProject 调用。
%
% 返回：model 结构体（同时存盘为 eMeetArm_reach_model.mat）

clc; close all;
cd(fileparts(mfilename('fullpath')));

%% 0. 可调参数 -----------------------------------------------------------
alphaStar_deg = 0;      % 拍摄方位角（pan），(-180,180]；默认水平朝前
betaStar_deg  = 0;      % 拍摄俯仰角（tilt），[-90,90]；朝下取负
tolOri_deg    = 5;      % 姿态容差（光轴方向 + 画面水平 roll），单位度
posTol        = 0.005;  % 位置容差，单位 m

Nmc      = 40000;       % 蒙特卡洛采样点数（圈定候选体素）
vRes     = 0.05;        % 体素/栅格分辨率，单位 m（越小越细、越慢）
nSeeds   = 8;           % 每点 IK 最大起点数（第 1 个为热启动解，其余备用；越大越少漏解）
closeVox = 1;           % 补洞半径（体素）：闭运算填补 IK 漏解针孔，仍多孔就调到 2
safetyMargin = 0.05;    % 安全裕度：可达域外边界整体往里收多少 m（要求 5cm）
ellShrink= 0.98;        % 内接椭球安全收缩系数（<1 更保守）

endLink  = 'camera_optical_frame';   % 相机光学系（+z = 光轴）
modelFile = 'eMeetArm_reach_model.mat';

% Joint1~Joint6 关节限位（来自 robot_arm_description 的 URDF）
% 2026-07-28 实机重标定：J2 新零点 = 旧 +0.181 rad，J3 = 旧 -0.176 rad，限位随刻度平移；
% 同日 J3 行程按新零点收紧为 [-2.5, 0]
lower = [-2.618, -0.981, -2.5,  -3.1,  -0.7854, -1.5];
upper = [ 2.618,  2.959,  0.02,  3.1,   0.7854,  0.5];

%% 1. 加载机器人模型 -----------------------------------------------------
urdfPath = fullfile('..', 'robot_arm_description', 'urdf', 'eMeetArm_models.urdf');
robot = importrobot(urdfPath);
robot.DataFormat = 'row';

%% 2. 目标完整姿态（光轴朝前 + 画面水平 roll=0） ------------------------
% shootDir2R 返回的 R 即目标姿态：列 = [右 下 光轴]，roll=0 → 画面水平。
[Rstar, aStar] = shootDir2R(alphaStar_deg, betaStar_deg);
aStar    = aStar(:);
quatStar = rotm2quat(Rstar);        % [w x y z]，世界 ← 相机光学系
fprintf('拍摄朝向: α=%.1f°, β=%.1f°  →  光轴 = [%.3f %.3f %.3f]，画面水平\n', ...
        alphaStar_deg, betaStar_deg, aStar(1), aStar(2), aStar(3));
fprintf('姿态容差 ±%.1f°，位置容差 %.0f mm\n', tolOri_deg, posTol*1000);

%% 3. 蒙特卡洛圈定候选体素（姿态可达 ⊂ 位置可达） ----------------------
fprintf('\n[1/2] 蒙特卡洛采样 %d 构型，圈定候选体素...\n', Nmc);
qMC = rand(Nmc, 6) .* (upper - lower) + lower;
posMC = zeros(Nmc, 3);
for i = 1:Nmc
    T = getTransform(robot, qMC(i,:), endLink);
    posMC(i,:) = T(1:3,4)';
end
voxIdx = floor(posMC / vRes);
candidates = (unique(voxIdx, 'rows') + 0.5) * vRes;
nCand = size(candidates, 1);
fprintf('      位置可达体素（候选点）: %d 个（分辨率 %.0f mm）\n', nCand, vRes*1000);

%% 4. 构造 GIK 求解器 ----------------------------------------------------
gik = generalizedInverseKinematics( ...
        'RigidBodyTree', robot, ...
        'ConstraintInputs', {'position', 'orientation'});
gik.SolverParameters.AllowRandomRestart = true;    % 开内部重启，减少定姿态 IK 漏解
gik.SolverParameters.MaxIterations      = 80;

posTgt = constraintPositionTarget(endLink);
posTgt.PositionTolerance = posTol;

oriCon = constraintOrientationTarget(endLink);      % 完整姿态约束，全程固定
oriCon.TargetOrientation    = quatStar;
oriCon.OrientationTolerance = deg2rad(tolOri_deg);

homeCfg   = homeConfiguration(robot);
randSeeds = lower + rand(max(0,nSeeds-2), 6) .* (upper - lower);

%% 5. 逐候选点 IK 求解（热启动 warm-start） -----------------------------
fprintf('[2/2] 对 %d 候选点做 GIK（热启动，每点最多 %d 起点）...\n', nCand, nSeeds);
reachable = false(nCand, 1);
qReach    = nan(nCand, 6);
qPrev     = homeCfg;
tic;
for c = 1:nCand
    posTgt.TargetPosition = candidates(c, :);      % 只有位置逐点变，姿态固定

    % 起点：热启动解 → home → 随机；夹到限位内避免"初值越限"告警
    seedList = max(lower, min(upper, [qPrev; homeCfg; randSeeds]));
    for s = 1:size(seedList, 1)
        [qSol, info] = gik(seedList(s,:), posTgt, oriCon);
        if max([info.ConstraintViolations.Violation, 0]) < 1e-4
            reachable(c) = true;
            qReach(c,:)  = qSol;
            qPrev        = qSol;
            break;
        end
    end

    if mod(c, max(1,round(nCand/20))) == 0
        fprintf('      进度 %5.1f%%  (%d/%d)，已可达 %d\n', ...
                100*c/nCand, c, nCand, sum(reachable));
    end
end
elapsed = toc;

posReachRaw = candidates(reachable, :);
nRaw        = size(posReachRaw, 1);
fprintf('\n姿态可达点(未收边): %d / %d  (%.1f%%)，耗时 %.1f 秒\n', ...
        nRaw, nCand, 100*nRaw/max(1,nCand), elapsed);
if nRaw == 0
    warning('未找到姿态可达点：可放宽 tolOri_deg、检查朝向、增大 nSeeds。');
    model = struct(); return;
end

%% 6. 补洞(闭运算) + 安全收边 --------------------------------------------
% 先把未收边可达点栅格化（corner 原点 o + floor 分桶，建图/查询同一约定：
% idx = floor((P-o)/vRes)+1，体素中心 = o + (idx-0.5)*vRes）。
% 定姿态 IK 会漏解 → raw 集合带"针孔"，故：①闭运算(先膨胀后腐蚀)补洞成实心，
% ②再用球形结构元腐蚀，把外边界整体内收 safetyMargin。
rVox   = max(1, round(safetyMargin / vRes));      % 收边体素数
padVox = closeVox + rVox + 1;                     % 栅格余量（容纳膨胀/腐蚀不越界）
bnd  = [min(posReachRaw); max(posReachRaw)];
o    = (floor(bnd(1,:)/vRes) - padVox) * vRes;
hi   = (ceil (bnd(2,:)/vRes) + padVox) * vRes;
dims = round((hi - o) / vRes);
occRaw = false(dims(1), dims(2), dims(3));
idx    = min(max(floor((posReachRaw - o)/vRes) + 1, 1), dims);
occRaw(sub2ind(dims, idx(:,1), idx(:,2), idx(:,3))) = true;

% ① 闭运算补洞：填补 IK 漏解造成的针孔/裂隙（半径 closeVox 体素）
[cx,cy,cz] = ndgrid(-closeVox:closeVox, -closeVox:closeVox, -closeVox:closeVox);
seC  = (cx.^2 + cy.^2 + cz.^2) <= closeVox^2;
occD = convn(double(occRaw), double(seC), 'same') > 0.5;              % 膨胀
occC = convn(double(occD),   double(seC), 'same') >= nnz(seC) - 0.5;  % 腐蚀 = 闭运算

% ② 安全收边：球形结构元腐蚀，外边界整体内收 safetyMargin
[bx,by,bz] = ndgrid(-rVox:rVox, -rVox:rVox, -rVox:rVox);
se   = (bx.^2 + by.^2 + bz.^2) <= rVox^2;
occ  = convn(double(occC), double(se), 'same') >= nnz(se) - 0.5;
fprintf('体素: raw %d → 补洞 %d → 收边%.0fcm %d\n', ...
        nnz(occRaw), nnz(occC), safetyMargin*100, nnz(occ));
if ~any(occ(:))
    warning('收边后无可达体素：safetyMargin 太大或 vRes 太粗。');
    model = struct(); return;
end

% 收边后的可达点（= 占据体素中心，之后统一以此为准）
[oi, oj, ok] = ind2sub(dims, find(occ));
occCenters = o + ([oi, oj, ok] - 0.5) * vRes;
posReach   = occCenters;
nReach     = size(posReach, 1);

%% 7. 统计信息（收边后） -------------------------------------------------
xRange = [min(posReach(:,1)), max(posReach(:,1))];
yRange = [min(posReach(:,2)), max(posReach(:,2))];
zRange = [min(posReach(:,3)), max(posReach(:,3))];
fprintf('\n=== 定姿态可达域统计（α=%.0f°, β=%.0f°, ±%.0f°, 画面水平, 收边%.0fcm）===\n', ...
        alphaStar_deg, betaStar_deg, tolOri_deg, safetyMargin*100);
fprintf('X:[%.3f, %.3f]  Y:[%.3f, %.3f]  Z:[%.3f, %.3f] m\n', ...
        xRange(1),xRange(2), yRange(1),yRange(2), zRange(1),zRange(2));
fprintf('臂展: [%.3f, %.3f] m，体积≈%.0f cm³\n', ...
        min(sqrt(sum(posReach.^2,2))), max(sqrt(sum(posReach.^2,2))), ...
        nReach*vRes^3*1e6);

%% 8. 内接椭球护栏 + 打包存盘 -------------------------------------------
% 8.1 内接椭球 (x-c)'A(x-c) <= 1，保证椭球内全部可达（保守的可微护栏）
% 沿点云主轴定向，取"到最近空体素的马氏距离"为最大尺度 → 椭球内不含任何空体素。
c  = mean(posReach, 1);
S  = cov(posReach);
if rcond(S) < 1e-9, S = S + 1e-6*eye(3); end        % 防退化
xc = o(1) + ((1:dims(1)) - 0.5) * vRes;             % 各轴体素中心坐标
yc = o(2) + ((1:dims(2)) - 0.5) * vRes;
zc = o(3) + ((1:dims(3)) - 0.5) * vRes;
[GX,GY,GZ] = ndgrid(xc, yc, zc);
Gfree = [GX(~occ), GY(~occ), GZ(~occ)];             % 所有空体素中心
dfree = sum(((Gfree - c) / S) .* (Gfree - c), 2);   % 马氏距离平方
sMax  = min(dfree) * ellShrink;                     % 最大安全尺度
A     = inv(sMax * S);                              % 椭球矩阵

% 8.2 打包并存盘
model = struct('vRes',vRes, 'origin',o, 'dims',dims, 'occ',occ, ...
    'occCenters',occCenters, 'c',c, 'A',A, ...
    'quatStar',quatStar, 'aStar',aStar', 'Rstar',Rstar, ...
    'alphaStar_deg',alphaStar_deg, 'betaStar_deg',betaStar_deg, ...
    'tolOri_deg',tolOri_deg, 'posTol',posTol, 'safetyMargin',safetyMargin);
save(fullfile(pwd, modelFile), 'model');
fprintf('\n可达性判定器已存: %s\n', modelFile);
fprintf('  用法: model=load(''%s'').model;\n', modelFile);
fprintf('        tf = reachIsReachable([x y z], model);   %% 是否可达\n');
fprintf('        p  = reachProject([x y z], model);        %% 不可达则拉回最近可达点\n');
fprintf('  椭球护栏（内部保证可达）: (x-c)*A*(x-c)'' <= 1\n');

%% 9. 可视化 -------------------------------------------------------------
figure('Name','定姿态可达域 + 判定器','NumberTitle','off','Position',[50 50 980 740]);
scatter3(posMC(1:8:end,1),posMC(1:8:end,2),posMC(1:8:end,3),1,[0.85 0.85 0.85],'filled');
hold on;
scatter3(posReach(:,1),posReach(:,2),posReach(:,3),22,posReach(:,3),'filled');
colormap(jet); colorbar;
% 内接椭球护栏
[Va,Da] = eig(A); radii = 1./sqrt(diag(Da));
[sx,sy,sz] = sphere(24);
E = Va*diag(radii)*[sx(:)'; sy(:)'; sz(:)'] + c(:);
surf(reshape(E(1,:),size(sx)), reshape(E(2,:),size(sx)), reshape(E(3,:),size(sx)), ...
     'FaceColor',[0.2 0.8 0.3],'FaceAlpha',0.15,'EdgeColor',[0.3 0.5 0.3],'EdgeAlpha',0.3);
plot3(0,0,0,'r*','MarkerSize',14,'LineWidth',2);
quiver3(c(1),c(2),c(3), aStar(1),aStar(2),aStar(3), 0.15,'k','LineWidth',2);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf(['定姿态可达域（灰=位置可达, 彩=收边后可达, 绿=内接椭球护栏）\n' ...
               'α=%.0f°, β=%.0f°, 画面水平, ±%.0f°, 收边%.0fcm, %d 点'], ...
               alphaStar_deg, betaStar_deg, tolOri_deg, safetyMargin*100, nReach));
axis equal; grid on; view(45,25);

%% 10. 自测：随机查几个点，演示判定 + 拉回 -------------------------------
fprintf('\n=== 判定器自测 ===\n');
test = c + [0 0 0; 0.25 0.25 0.15; -0.1 0.05 0.2];   % 第1个必在内、后两个可能在外
tf   = reachIsReachable(test, model);
proj = reachProject(test, model);
for i = 1:size(test,1)
    fprintf('  [%.2f %.2f %.2f]  可达=%d  →  拉回[%.2f %.2f %.2f]\n', ...
            test(i,:), tf(i), proj(i,:));
end

end
