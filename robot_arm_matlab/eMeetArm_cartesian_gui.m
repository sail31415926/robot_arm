function eMeetArm_cartesian_gui()
%% eMeetArm 笛卡尔轨迹规划 GUI
% 通过滑块设定目标偏移，点击按钮规划并播放直线轨迹动画

clc; close all;
cd(fileparts(mfilename('fullpath')));

%% 1. 加载机器人模型
robot = importrobot(fullfile('eMeetArm_models', 'urdf', 'eMeetArm_models.urdf'));
robot.DataFormat = 'row';
endLink = 'Link6';

ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.MaxIterations = 60;
weights = [1 1 1 1 1 1];

q_home  = homeConfiguration(robot);
T_home  = getTransform(robot, q_home, endLink);
p_home  = T_home(1:3, 4)';
R_fixed = T_home(1:3, 1:3);

% 状态变量
isRunning = false;
stopFlag  = false;

%% 2. 创建主窗口
fig = figure('Name','eMeetArm 笛卡尔轨迹规划', 'NumberTitle','off', ...
             'Position',[60 60 1200 680], ...
             'CloseRequestFcn', @onClose);

%% 3. 左侧：3D 机器人视图
axR = axes('Position',[0.02 0.08 0.58 0.88]);
show(robot, q_home, 'Visuals','on','Frames','off','Parent',axR);
hold(axR,'on');
hPath   = plot3(axR, NaN,NaN,NaN, 'b--','LineWidth',1.5,'DisplayName','规划路径');
hActual = plot3(axR, NaN,NaN,NaN, 'r-', 'LineWidth',2,  'DisplayName','实际轨迹');
hStart  = plot3(axR, p_home(1),p_home(2),p_home(3), ...
                'go','MarkerSize',10,'MarkerFaceColor','g','DisplayName','起点');
hEnd    = plot3(axR, NaN,NaN,NaN, ...
                'rs','MarkerSize',10,'MarkerFaceColor','r','DisplayName','终点');
legend(axR,'Location','best');
axis(axR,'equal'); grid(axR,'on'); view(axR,45,20);
xlabel(axR,'X (m)'); ylabel(axR,'Y (m)'); zlabel(axR,'Z (m)');
title(axR,'eMeetArm 笛卡尔轨迹规划');

%% 4. 右侧：控制面板
px = 0.63;   % 面板起始 X（归一化）

% ---- 目标位置偏移 ----
uicontrol('Style','text','Units','normalized', ...
          'Position',[px 0.91 0.35 0.05], ...
          'String','目标位置偏移（相对起点）', ...
          'FontSize',10,'FontWeight','bold','HorizontalAlignment','left');

[hDx, hDxVal] = makeSlider('Delta X (m)', px, 0.82, -0.5, 0.5, 0.15);
[hDy, hDyVal] = makeSlider('Delta Y (m)', px, 0.72, -0.5, 0.5, 0.00);
[hDz, hDzVal] = makeSlider('Delta Z (m)', px, 0.62, -0.5, 0.5,-0.10);

addlistener(hDx,'ContinuousValueChange',@updatePreview);
addlistener(hDy,'ContinuousValueChange',@updatePreview);
addlistener(hDz,'ContinuousValueChange',@updatePreview);

% ---- 轨迹参数 ----
uicontrol('Style','text','Units','normalized', ...
          'Position',[px 0.55 0.35 0.04], ...
          'String','轨迹参数', ...
          'FontSize',10,'FontWeight','bold','HorizontalAlignment','left');

[hDur, hDurVal] = makeSlider('运动时长 (s)', px, 0.47, 1, 8, 3);
[hStep,hStepVal]= makeSlider('路径点数',     px, 0.37, 20,150, 80);

% ---- 操作按钮 ----
hBtnRun = uicontrol('Style','pushbutton','Units','normalized', ...
    'Position',[px 0.27 0.16 0.07], ...
    'String','规划并执行','FontSize',10,'BackgroundColor',[0.2 0.7 0.3], ...
    'ForegroundColor','w','Callback',@runTraj);

hBtnStop = uicontrol('Style','pushbutton','Units','normalized', ...
    'Position',[px+0.18 0.27 0.10 0.07], ...
    'String','停止','FontSize',10,'BackgroundColor',[0.8 0.3 0.2], ...
    'ForegroundColor','w','Callback',@stopTraj,'Enable','off');

uicontrol('Style','pushbutton','Units','normalized', ...
    'Position',[px+0.18 0.18 0.10 0.07], ...
    'String','重置零位','FontSize',10,'Callback',@resetRobot);

uicontrol('Style','pushbutton','Units','normalized', ...
    'Position',[px 0.18 0.16 0.07], ...
    'String','显示关节曲线','FontSize',10,'Callback',@showJointPlot);

% ---- 状态信息栏 ----
uicontrol('Style','text','Units','normalized', ...
          'Position',[px 0.13 0.35 0.03], ...
          'String','状态信息', ...
          'FontSize',9,'FontWeight','bold','HorizontalAlignment','left');

hStatus = uicontrol('Style','text','Units','normalized', ...
    'Position',[px 0.01 0.36 0.12], ...
    'String','就绪。拖动滑块设定目标位置，点击「规划并执行」。', ...
    'FontSize',8,'HorizontalAlignment','left', ...
    'BackgroundColor',[0.95 0.95 0.95]);

% 存储最近一次轨迹供"显示关节曲线"使用
lastQtraj  = [];
lastTvec   = [];

%% -------- 回调函数 --------

    %% 预览：移动终点标记
    function updatePreview(~,~)
        dx = hDx.Value;  dy = hDy.Value;  dz = hDz.Value;
        hDxVal.String  = sprintf('%.3f', dx);
        hDyVal.String  = sprintf('%.3f', dy);
        hDzVal.String  = sprintf('%.3f', dz);
        p_end = p_home + [dx dy dz];
        set(hEnd,'XData',p_end(1),'YData',p_end(2),'ZData',p_end(3));
        drawnow limitrate;
    end

    %% 规划并执行
    function runTraj(~,~)
        if isRunning, return; end
        isRunning = true;
        stopFlag  = false;
        hBtnRun.Enable  = 'off';
        hBtnStop.Enable = 'on';

        dx  = hDx.Value;  dy = hDy.Value;  dz = hDz.Value;
        dur = hDur.Value;
        nSteps = round(hStep.Value);
        hDurVal.String  = sprintf('%.1f', dur);
        hStepVal.String = sprintf('%d',   nSteps);

        p_end = p_home + [dx dy dz];

        % 梯形速度曲线归一化参数
        t_norm = trapveltraj([0, 1], nSteps, 'EndTime', 1)';
        pos_traj = p_home + t_norm .* (p_end - p_home);

        % 更新规划路径显示
        set(hPath,'XData',pos_traj(:,1),'YData',pos_traj(:,2),'ZData',pos_traj(:,3));

        % IK 求解
        setStatus(sprintf('正在求解 %d 个路径点 IK...', nSteps));
        q_traj = zeros(nSteps, length(q_home));
        ik_ok  = true(nSteps,1);
        q_prev = q_home;

        for i = 1:nSteps
            if stopFlag, break; end
            T_i = eye(4);
            T_i(1:3,1:3) = R_fixed;
            T_i(1:3,4)   = pos_traj(i,:)';
            [q_sol, info]  = ik(endLink, T_i, weights, q_prev);
            q_traj(i,:)    = q_sol;
            ik_ok(i)       = info.PoseErrorNorm < 5e-3;
            q_prev         = q_sol;
        end

        nFail = sum(~ik_ok);
        if nFail > 0
            setStatus(sprintf('IK 完成，%d/%d 点未收敛（%.0f%%），仍执行动画', ...
                              nFail, nSteps, nFail/nSteps*100));
        else
            setStatus(sprintf('IK 全部收敛，开始执行动画...'));
        end

        % 动画播放
        t_vec   = linspace(0, dur, nSteps);
        actual  = zeros(nSteps,3);
        set(hActual,'XData',NaN,'YData',NaN,'ZData',NaN);

        for i = 1:nSteps
            if stopFlag, break; end

            show(robot, q_traj(i,:), 'Parent', axR, ...
                 'Visuals','on','Frames','off', ...
                 'FastUpdate',true,'PreservePlot',false);

            T_now    = getTransform(robot, q_traj(i,:), endLink);
            actual(i,:) = T_now(1:3,4)';
            set(hActual,'XData',actual(1:i,1), ...
                        'YData',actual(1:i,2), ...
                        'ZData',actual(1:i,3));

            title(axR, sprintf('t=%.2fs  末端(%.3f, %.3f, %.3f)m', ...
                  t_vec(i), actual(i,:)));
            drawnow limitrate;
            pause(dur / nSteps * 0.85);
        end

        if ~stopFlag
            err_mm = sqrt(sum((actual - pos_traj).^2, 2)) * 1000;
            setStatus(sprintf('完成！距离 %.3fm | 最大误差 %.2fmm | 均值误差 %.2fmm', ...
                              norm(p_end - p_home), max(err_mm), mean(err_mm)));
        else
            setStatus('已停止。');
        end

        % 保存供绘图
        lastQtraj = q_traj;
        lastTvec  = t_vec;

        isRunning = false;
        hBtnRun.Enable  = 'on';
        hBtnStop.Enable = 'off';
    end

    %% 停止动画
    function stopTraj(~,~)
        stopFlag = true;
    end

    %% 重置到零位
    function resetRobot(~,~)
        if isRunning, return; end
        show(robot, q_home, 'Visuals','on','Frames','off','Parent',axR, ...
             'FastUpdate',true,'PreservePlot',false);
        set(hPath,  'XData',NaN,'YData',NaN,'ZData',NaN);
        set(hActual,'XData',NaN,'YData',NaN,'ZData',NaN);
        set(hEnd,   'XData',NaN,'YData',NaN,'ZData',NaN);
        hDx.Value = 0.15;  hDy.Value = 0.0;  hDz.Value = -0.10;
        hDur.Value = 3;    hStep.Value = 80;
        hDxVal.String = '0.150';  hDyVal.String = '0.000';  hDzVal.String = '-0.100';
        hDurVal.String = '3.0';   hStepVal.String = '80';
        title(axR,'eMeetArm 笛卡尔轨迹规划');
        setStatus('已重置到零位。');
        drawnow;
    end

    %% 显示关节角曲线
    function showJointPlot(~,~)
        if isempty(lastQtraj)
            setStatus('请先执行一次轨迹规划。');
            return;
        end
        names = {'J1 腰转','J2 大臂','J3 小臂','J4 腕摆','J5 腕转','J6 末端'};
        lower = [-3.1,-0.8,-3.14,-3.1,-0.7854,-1.5];
        upper = [ 3.1, 3.14, 0.05, 3.1, 0.7854, 0.5];
        c6 = lines(6);
        figure('Name','关节角轨迹','NumberTitle','off','Position',[100 100 900 580]);
        for j = 1:6
            subplot(2,3,j);
            plot(lastTvec, rad2deg(lastQtraj(:,j)), 'Color',c6(j,:),'LineWidth',1.8);
            hold on;
            yline(rad2deg(lower(j)),'r--','LineWidth',1,'Alpha',0.7);
            yline(rad2deg(upper(j)),'r--','LineWidth',1,'Alpha',0.7);
            xlabel('时间 (s)'); ylabel('角度 (deg)');
            title(names{j}); grid on;
        end
        sgtitle('关节角随时间变化（红虚线 = 限位）');
    end

    %% 窗口关闭
    function onClose(~,~)
        stopFlag = true;
        delete(fig);
    end

    %% 辅助：更新状态栏
    function setStatus(msg)
        hStatus.String = msg;
        drawnow;
    end

    %% 辅助：创建滑块+标签+数值显示
    function [hSlider, hVal] = makeSlider(label, x, y, mn, mx, initVal)
        uicontrol('Style','text','Units','normalized', ...
                  'Position',[x, y+0.04, 0.22, 0.03], ...
                  'String', label, 'FontSize',9, ...
                  'HorizontalAlignment','left');
        hSlider = uicontrol('Style','slider','Units','normalized', ...
                  'Position',[x, y, 0.27, 0.03], ...
                  'Min',mn,'Max',mx,'Value',initVal);
        hVal = uicontrol('Style','text','Units','normalized', ...
                  'Position',[x+0.28, y, 0.07, 0.03], ...
                  'String',sprintf('%.3f',initVal), ...
                  'FontSize',9,'HorizontalAlignment','left');
    end

    % 初始化预览
    updatePreview([],[]);

end
