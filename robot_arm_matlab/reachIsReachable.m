function tf = reachIsReachable(P, model)
%REACHISREACHABLE  判定位置是否落在"定姿态可达域"内（占据栅格查询）
%
%   tf = reachIsReachable(P, model)
%     P      N×3 待查位置（世界系，单位 m）
%     model  eMeetArm_workspace_aim 产出的判定器结构体
%            （model = load('eMeetArm_reach_model.mat').model）
%     tf     N×1 logical，true = 该位置在相机固定朝向下可达
%
%   说明：可达域对应"相机保持 model 里记录的固定朝向（光轴 + 画面水平）"。
%         这是栅格级判定，执行前建议再用 IK 精确确认。
%
%   另见 REACHPROJECT, EMEETARM_WORKSPACE_AIM

    % 与建图同一套约定：idx = floor((P - origin)/vRes) + 1
    idx = floor((P - model.origin) / model.vRes) + 1;

    sz  = model.dims;
    n   = size(P, 1);
    tf  = false(n, 1);

    inb = all(idx >= 1, 2) & all(idx <= sz, 2);
    if any(inb)
        ii  = idx(inb, :);
        lin = sub2ind(sz, ii(:,1), ii(:,2), ii(:,3));
        tf(inb) = model.occ(lin);
    end
end
