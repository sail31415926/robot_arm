function [Pfix, moved] = reachProject(P, model)
%REACHPROJECT  把不可达目标"拉回"到最近的可达位置
%
%   [Pfix, moved] = reachProject(P, model)
%     P      N×3 目标位置（世界系，单位 m）
%     model  eMeetArm_workspace_aim 产出的判定器结构体
%     Pfix   N×3 修正后的位置：本就可达的原样返回；不可达的替换为
%            可达域内最近的占据体素中心
%     moved  N×1 位移量（m），0 表示原本就可达
%
%   用途：上层智能体生成的目标若不可达，用它投影到最近可达点再执行，
%         避免直接下发够不到的目标。
%
%   另见 REACHISREACHABLE, EMEETARM_WORKSPACE_AIM

    n     = size(P, 1);
    Pfix  = P;
    moved = zeros(n, 1);
    C     = model.occCenters;         % M×3 可达体素中心

    tf = reachIsReachable(P, model);
    for i = 1:n
        if tf(i)
            continue;                 % 已可达，不动
        end
        d2 = sum((C - P(i,:)).^2, 2); % 到各可达体素中心的距离平方
        [dm, k] = min(d2);
        Pfix(i,:) = C(k,:);
        moved(i)  = sqrt(dm);
    end
end
