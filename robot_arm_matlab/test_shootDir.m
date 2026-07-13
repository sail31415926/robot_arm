function test_shootDir()
%TEST_SHOOTDIR  拍摄朝向描述规范 v1 —— 自检脚本
%
%   验证 shootAxis2R / shootDir2R / lookAt2R / R2shootDir 的正确性：
%     1. R 为合法旋转矩阵（正交、det=+1）
%     2. 画面水平（右轴 x_c 垂直于世界竖直）
%     3. 角度往返一致  (α,β) → R → (α,β)
%     4. look-at 与角度写法一致（同一光轴得到同一姿态）
%     5. 退化情况（正上/正下）可用且由 α 决定画面朝向
%
%   直接运行：>> test_shootDir

    clc;
    fprintf('===== 拍摄朝向描述规范 v1 自检 =====\n\n');
    tol = 1e-9;
    zW  = [0; 0; 1];
    nPass = 0; nFail = 0;

    check = @(name, cond) localCheck(name, cond);

    %% 1 & 2 & 3：遍历方位角/俯仰角网格
    alphas = -180:30:150;
    betas  = -80:20:40;              % 避开 ±90 退化点，放到第 5 部分单独测
    fprintf('[1-3] 遍历 %d 个朝向：旋转矩阵合法性 / 画面水平 / 角度往返\n', ...
            numel(alphas)*numel(betas));
    for al = alphas
        for be = betas
            [R, a] = shootDir2R(al, be);

            % 1) 合法旋转矩阵
            orthoErr = norm(R.'*R - eye(3), 'fro');
            detErr   = abs(det(R) - 1);
            nPass = nPass + localSilent(orthoErr < 1e-9 && detErr < 1e-9, ...
                sprintf('R 非法 @ (%d,%d): ortho=%.1e det=%.1e', al, be, orthoErr, detErr));

            % 2) 画面水平：右轴 x_c ⟂ 世界竖直
            horizErr = abs(dot(R(:,1), zW));
            nPass = nPass + localSilent(horizErr < 1e-9, ...
                sprintf('画面不水平 @ (%d,%d): x_c·zW=%.1e', al, be, horizErr));

            % 3) 光轴 = 第三列，且角度往返
            axisErr = norm(R(:,3) - a);
            [al2, be2] = R2shootDir(R);
            angErr = abs(wrap180(al2 - al)) + abs(be2 - be);
            nPass = nPass + localSilent(axisErr < 1e-9 && angErr < 1e-6, ...
                sprintf('角度往返失败 @ (%d,%d): axisErr=%.1e angErr=%.1e', ...
                        al, be, axisErr, angErr));
        end
    end
    fprintf('     子项累计通过：%d\n\n', nPass);

    %% 4：look-at 与角度写法一致
    fprintf('[4] look-at 与角度写法一致性\n');
    p = [0.2; -0.1; 0.5];
    dirs = {[1;0;0], [0;1;0], [1;1;1], [-2;0.5;-1]};
    ok4 = true;
    for i = 1:numel(dirs)
        a = dirs{i} / norm(dirs{i});
        T = p + 0.8 * a;                          % 目标点在光轴上
        [Rl, al_a] = lookAt2R(p, T);
        [alpha, beta] = R2shootDir(a);
        Rd = shootDir2R(alpha, beta);
        d = norm(Rl - Rd, 'fro') + norm(al_a - a);
        ok4 = ok4 && (d < 1e-9);
    end
    check('look-at 与角度写法得到同一姿态', ok4);

    %% 5：退化（正上/正下）
    fprintf('\n[5] 退化情况（正上 β=+90 / 正下 β=-90）\n');
    okUp = true; okDn = true;
    for al = [0 45 90 -120]
        Ru = shootDir2R(al, 90);                  % 正上
        Rd = shootDir2R(al, -90);                 % 正下
        okUp = okUp && ( norm(Ru(:,3) - [0;0;1]) < 1e-9 && ...
                         norm(Ru.'*Ru - eye(3),'fro') < 1e-9 );
        okDn = okDn && ( norm(Rd(:,3) - [0;0;-1]) < 1e-9 && ...
                         norm(Rd.'*Rd - eye(3),'fro') < 1e-9 );
    end
    check('正上：光轴=+Z 且姿态合法（α 决定画面朝向）', okUp);
    check('正下：光轴=-Z 且姿态合法（α 决定画面朝向）', okDn);

    fprintf('\n===== 自检完成 =====\n');
end

% ---- 辅助函数 ----
function n = localSilent(cond, failMsg)
    if cond
        n = 1;
    else
        n = 0;
        fprintf(2, '   ✗ %s\n', failMsg);
    end
end

function localCheck(name, cond)
    if cond
        fprintf('   ✓ %s\n', name);
    else
        fprintf(2, '   ✗ %s\n', name);
    end
end

function d = wrap180(d)
    d = mod(d + 180, 360) - 180;
end
