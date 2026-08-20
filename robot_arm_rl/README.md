# robot_arm_rl — 摄影机械臂强化学习

纯 MuJoCo 的 Gymnasium 环境 + Stable-Baselines3 训练 + ROS2 部署骨架。
两个独立任务：**构图跟拍**（SAC）与**环绕运镜**（PPO），共用 `sim/rl_env.xml` 场景。

配套学习教材：[docs/强化学习入门.md](docs/强化学习入门.md)

## 任务一：构图跟拍（2026-08-19 重构，SAC）

**运动主体的可配置构图跟拍**：每轮随机给定构图目标 (u\*, v\*, s\*)
（主体应出现在画面的位置与大小），被摄主体以随机速度在工作台上游走；
策略控制相机在球坐标轨道上运动并微调视轴偏置，使主体持续保持在目标
构图点，且运动平滑。

分层设计：解析 look-at 负责"大致对准"（基准朝向），RL 负责解析法做不了的
部分——构图落点、运动预判、平滑与精度的权衡。

| 要素 | 内容 |
|---|---|
| 动作 (5维) | Δθ / Δφ / Δr（球坐标轨道）+ Δpan / Δtilt（视轴偏置） |
| 观测 (34维) | 球坐标+偏置状态、上步动作、构图误差、投影、**构图目标**、图像速度、关节位置/速度 |
| 奖励 | 构图误差 + 可见性 + 尺寸 + 平滑 + 抖动 + 限位余量（详见 env 源码） |
| 终止 | 连续丢失主体 1s 提前终止（罚 -10）；500 步截断 |
| 决策频率 | 50 Hz（每步 = 10 × 2ms 物理步） |

坐标约定：投影/视轴以 **Cam0 body 系 = ROS 光学系**（+Z 光轴/+Y 下/+X 右）
为准；v 向下增大，v\*=0.67 即主体在画面下三分之一。

## 任务二：环绕运镜（2026-08-20 新增，PPO + 课程学习）

**eye-in-hand 定弧环绕**：目标物体静止，相机从弧线一端出发、以近似恒定
半径水平环绕目标一段弧，全程保持目标在画面中心、图像水平（roll≈0）。

**弧度目标不是 180°**——实机 Cam0 臂展 0.674m、底座固定，环绕弧上限由
主体位置与半径决定（2026-08-20 IK 可达性扫描，判据 位置<2cm+对准<5°）：

| 主体 x | r=0.35 | r=0.45 | r=0.55 |
|---|---|---|---|
| 0.60 | 145° | 130° | 120° |
| 0.70 | 115° | 115° | 110° |
| 0.80 |  85° |  95° |  95° |
| 0.90 |  50° |  70° |  80° |

→ 训练默认主体 x∈[0.60,0.75]、期望半径 0.40m、课程弧度上限 **120°**；
每轮 reset 按工作空间几何裁剪可行弧 + IK 验证端点，弧度目标动态钳制。

| 要素 | 内容 |
|---|---|
| 动作 (6维) | **Cam0 光学系下的末端笛卡尔速度** v(±0.2m/s) + ω(±0.5rad/s)，DLS 微分逆解→关节速度限幅→指令积分→PD |
| 观测 (30维) | 目标在相机系位置、归一化像素坐标、距离/方位/俯仰、进度/剩余弧/方向、上步动作、关节位置/速度 |
| 奖励 | 居中(w=10) + 进度(5) + 距离保持(2) + 平滑(0.5) + 安全:限位/奇异/水平(1)；碰撞/出视野/持续偏心/顶限位 -10 终止；完成 +50 |
| 课程 | ①静态对准 → ②小弧10-30° → ③长弧30°→arc_max(60°→120°渐进) → ④域随机化（主体微移+位姿扰动+观测噪声+半径抖动），按成功率自动推进 |
| 决策频率 | 20 Hz（每步 = 25 × 2ms 物理步，贴近实机 JTC 低频位置流） |

```bash
python3 train_ppo_orbit.py                    # 从阶段1自动课程（默认16环境）
python3 train_ppo_orbit.py --stage 3          # 固定阶段调试
python3 eval_orbit.py --arc 90                # 评估：固定90°弧口径
tensorboard --logdir logs/tb_orbit            # orbit/success_rate 等任务指标
```

### 训练结果（2026-08-20，300 万步全课程收敛，确定性策略 20 轮/条件）

| 评估条件 | 成功率 | 平均中心误差 | 说明 |
|---|---|---|---|
| 固定 90° 弧 | **100%** | 0.021 | 标准口径 |
| 固定 120° 弧 | **100%** | 0.021 | 部分轮被可达性钳到 98~110°（预期行为） |
| 阶段4 随机弧+观测噪声+主体微移 | 90% | 0.039 | 鲁棒性口径 |

课程自动推进全程无人工干预：阶段1 占 ~140 万步（成功率随 std 退火突增），
阶段2/3 各级 ~0.7 成功率即推进。产物 `models/orbit/ppo_orbit_final.zip`。

### 实机部署安全层（设计约定，未实现）

策略输出是笛卡尔速度，部署必须套安全壳，按外→内：

1. **总线约定**：只发 `/arm_controller/joint_trajectory`（微分逆解在部署节点内做），
   模式切换走 ModeManager，急停接 `/robot_arm/enable`；
2. **速度/加速度限幅 + 低通**：动作一阶低通（fc≈2Hz）后限幅，关节速度等比缩放保方向
   （环境内同款逻辑）；
3. **奇异规避**：σ_min < 0.05 起线性降速，< 0.02 冻结并报警（DLS 阻尼只能兜误差不能兜安全）；
4. **工作空间围栏**：Cam0 目标位置出 [H 0.20-0.60, Z 0.15-0.85] 内域即拒绝该步指令；
5. **看门狗**：感知（检测框）断流 >200ms 冻结在原位；JTC 流 50Hz→实机跟踪率问题见任务一 TODO。

### 已知简化（上实机前必读）

- MJCF 只配了臂↔世界碰撞对，**自碰撞检测缺失**；底座-地面常态接触已从判定排除。
- 主体是 mocap 体不参与碰撞；感知是几何真值投影（内参 K 由 fovy 推得，实机换标定值）。
- 速度积分基准是**指令状态**而非实测位置（实测基准会被 PD 稳态误差拖着下垂，
  70 步后云台蹭桌面，2026-08-20 实测）——部署时对应"以上一条指令为基准"。

## 快速开始（任务一）

```bash
cd ~/E7009_ws && source /opt/ros/humble/setup.bash && source .venv/bin/activate
cd src/E7009/robot_arm/robot_arm_rl/scripts

python3 train_sac.py --n-envs 8 --timesteps 2000000   # 训练（GPU 可用时）
tensorboard --logdir logs/tb                           # 看曲线
python3 eval_policy.py                                 # 评估（弹 MuJoCo 窗口）
python3 eval_policy.py --no-render --subject-motion --n-episodes 20
python3 sim_to_sim_test.py                             # MuJoCo 内分布外泛化测试
ros2 launch robot_arm_rl sim2sim_gazebo.launch.py      # Sim2Sim：策略进 Gazebo 闭环
```

也可 `colcon build --packages-select robot_arm_rl` 后用 `ros2 run robot_arm_rl train_sac` 等。

## 目录

```
sim/rl_env.xml        RL 本地 MuJoCo 场景（臂+云台V2 运动学与 eMeetArm.xml 逐点一致；
                      主体是 mocap 运动学体；网格跨包相对引用，须从源码目录加载）
sim/gazebo/rl_world.world    本包专用 Gazebo 世界（sim2sim 演示/验证）
scripts/envs/         eMeetArmEnv（任务的全部定义都在这一个文件里）
scripts/train_sac.py  SAC 训练（VecNormalize + 评估/checkpoint 回调）
scripts/eval_policy.py       策略评估
scripts/sim_to_sim_test.py   分布外泛化测试
scripts/deploy/rl_policy_node.py   部署节点（sim2sim 已闭环）
scripts/deploy/subject_mover.py    Gazebo 主体游走驱动+位姿转发
launch/sim2sim_gazebo.launch.py    独立 Gazebo 工程一键起
models/               训练产物；archive_*/、snapshot_*/ 是失效历史模型（见其 README）
```

## Sim2Sim 验收（2026-08-19，83 万步收敛模型）

| 条件 | MuJoCo（训练域） | Gazebo（跨仿真器） |
|---|---|---|
| 静止主体构图误差 | 0.092 | **0.096** |
| 运动主体构图误差 | 0.089 | **0.094~0.13** |
| 可见率 | 100% | **100%** |

链路：策略 50Hz → 孪生 IK → `/arm_controller/joint_trajectory` → JTC/gazebo_ros2_control，
感知 = 执行态关节角在 MuJoCo 孪生里做几何投影（真反馈，非指令回显）。

**独立 Gazebo 工程**（2026-08-19，emeet_arm.world 的高桌/红盒位置均在训练分布外，
不适合 RL 演示）：本包自带 `sim/gazebo/rl_world.world`（矮台 0.45 + planar_move
可移动红盒，几何与训练场景对齐、全 visual-only），launch 自拉
gzserver/gzclient/rsp/spawn/控制器；对外仅只读引用 robot_arm_description 的
xacro 与 bringup 的 controllers.yaml，`subject_mover` 驱动红盒路点游走并把
位姿转发给策略。运动主体（0.2m/s）下跟拍误差 0.094~0.10、可见率 100%。

## 已知事项 / TODO（实机上线前）

- **真实感知未接入**：u,v,s 需换成检测框（如 `/red_detector/feature`）；
  kinematic 感知在实机 = 相信指令即执行，遮挡/标定误差全被忽略。
- **实机 JTC 跟踪率**：50Hz 位置流实测只有 ~71-74%（2026-08-12），需降频缩增量
  或建模执行滞后重训；`traj_duration_s` 必须≈策略周期（0.1s 会稀释增量至 1/5）。
- **安全壳**：自碰撞/工作空间检查、丢检测框冻结、急停接 ModeManager。
- **部署守卫教训**：起步位姿常在可达域外，守卫必须允许"违规量单调下降"的
  回归更新，否则死锁（2026-08-19 sim2sim 实测，已修，含播种投影）。
- 模型权重与环境定义强耦合：**改动作/观测/奖励后旧 checkpoint 全部作废**，
  归档到 `models/archive_*/` 并写明原因（有先例可参考）。
- 历史教训（详见环境源码注释）：IK 必须还原 `data.qpos`（否则物理瞬移）；
  IK 目标必须挂 Cam0（挂 tool0 则云台失控）；reset 必须做可达性拒绝采样。
