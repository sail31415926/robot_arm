function eMeetArm_joint_control()
%% eMeetArm 机械臂关节角度交互控制
% 通过滑块实时调节 6 个旋转关节，3D 视图同步更新

clc; close all;
cd(fileparts(mfilename('fullpath')));

%% 1. 加载机器人模型
robot = importrobot(fullfile('eMeetArm_models', 'urdf', 'eMeetArm_models.urdf'));
robot.DataFormat = 'row';

q = homeConfiguration(robot);

%% 2. 关节参数定义（限位来自 URDF）
jointLower  = [-3.1,   -0.8,   -3.14,  -3.1,    -0.7854,  -1.5];
jointUpper  = [ 3.1,    3.14,   0.05,   3.1,     0.7854,   0.5];
jointLabels = {'J1 腰转', 'J2 大臂', 'J3 小臂', 'J4 腕摆', 'J5 腕转', 'J6 末端旋转'};

nJoints = numel(jointLabels);

%% 3. 创建主界面布局
fig = figure('Name','eMeetArm 关节控制','NumberTitle','off', ...
             'Position',[100 100 1100 620]);  %#ok<NASGU>

% 左侧：3D 可视化区域
axRobot = axes('Position', [0.02 0.05 0.60 0.90]);
show(robot, q, 'Visuals','on', 'Frames','off', 'Parent', axRobot);
title(axRobot, 'eMeetArm 机械臂实时位姿');
axis(axRobot, 'equal'); view(axRobot, 45, 20); grid(axRobot, 'on');
xlabel(axRobot,'X (m)'); ylabel(axRobot,'Y (m)'); zlabel(axRobot,'Z (m)');

% 右侧：滑块控制面板
panelX = 0.64;
sliderH = 0.06;

hSlider = gobjects(nJoints, 1);
hValue  = gobjects(nJoints, 1);

for i = 1:nJoints
    yPos = 0.93 - (i-1) * (sliderH + 0.025);

    uicontrol('Style','text', 'Units','normalized', ...
              'Position',[panelX, yPos+0.01, 0.16, 0.04], ...
              'String', jointLabels{i}, ...
              'HorizontalAlignment','left', 'FontSize', 9);

    hSlider(i) = uicontrol('Style','slider', 'Units','normalized', ...
              'Position',[panelX, yPos-0.02, 0.26, 0.035], ...
              'Min', jointLower(i), 'Max', jointUpper(i), ...
              'Value', q(i));
    addlistener(hSlider(i), 'ContinuousValueChange', @(src,~) updateRobot(src, i));

    hValue(i) = uicontrol('Style','text', 'Units','normalized', ...
              'Position',[panelX+0.27, yPos-0.015, 0.08, 0.03], ...
              'String', sprintf('%.3f', q(i)), ...
              'FontSize', 8);
end

% 复位按钮
uicontrol('Style','pushbutton', 'Units','normalized', ...
          'Position',[panelX+0.05, 0.02, 0.15, 0.05], ...
          'String','复位到零位', 'FontSize',10, ...
          'Callback', @resetRobot);

%% 4. 回调：滑块滑动时更新机器人位姿
% FastUpdate=true 只刷新变换矩阵，不重建图形对象，速度远快于完整 show()
    function updateRobot(src, idx)
        q(idx) = src.Value;
        hValue(idx).String = sprintf('%.3f', q(idx));

        show(robot, q, 'Visuals','on', 'Frames','off', ...
             'Parent', axRobot, 'FastUpdate', true, 'PreservePlot', false);

        T   = getTransform(robot, q, 'Link6');
        pos = T(1:3, 4);
        title(axRobot, sprintf('末端位置  x=%.3f  y=%.3f  z=%.3f (m)', ...
              pos(1), pos(2), pos(3)));
        drawnow limitrate;   % 限制刷新率，避免事件积压拖慢拖动
    end

%% 5. 回调：复位按钮
    function resetRobot(~, ~)
        q = homeConfiguration(robot);
        for k = 1:nJoints
            hSlider(k).Value = q(k);
            hValue(k).String = sprintf('%.3f', q(k));
        end
        show(robot, q, 'Visuals','on', 'Frames','off', ...
             'Parent', axRobot, 'FastUpdate', true, 'PreservePlot', false);
        title(axRobot,'eMeetArm 机械臂实时位姿');
        drawnow;
    end

end
