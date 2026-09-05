# 智信2026 · 科目三仿真平台

「智信—2026」竞技类科目三 **自主导航与障碍穿越** 的官方配套仿真平台：6 机集群在密林中自主穿越，将不同种类物品投放到类型匹配的投放点。基于 **ROS 1 Noetic**（Ubuntu 20.04），提供 **Python** 与 **Gazebo** 两个可插拔物理后端，评分/任务栈对后端透明。

> 完整赛题、真机部署与算法分析见 `c:/Users/24882/Desktop/资料/非凸α_docs/` 下的《自主导航与障碍穿越_实验方案》系列文档。

---

## 特性

- **双物理后端**：轻量 Python 运动学后端（`world_node`）与 Gazebo 物理后端（`gzserver` ODE + 自研 `libdrone_vel_plugin`），两者共享同一消息契约，任务/评分代码零改动切换。
- **在线建图闭环导航（默认）**：lidar 点云 → 累积成全局持久占用栅格（SLAM 式，含滑窗剪枝与碰撞标记）→ 2.5D A* 重规划 → P 位置环。定位带随机游走漂移，导航只消费 `est_pose` 与自建地图，**不读场景真值**（真值 A* 保留为 god-mode 备胎：`closed_loop.enabled: false`）。
- **反应层防御栈（W1-W5 稳健化战役产物）**：机间分离障碍护栏、云避障符号修复、返航塌缩卫兵、救援互斥、STALL 看门狗、退出迟滞——每个机制独立配置开关、A/B 矩阵验证后翻默认。
- **6 机集群（Boids 速度层）**：分离 + 对齐 + 聚合（`swarm` 段，默认开）；静态编队 `formation.yaml` 退居备选、与群集互斥。
- **类型匹配投放**：投放点视觉标识类型码与机载物类型匹配，匹配成功才触发投放。
- **地面站（GCS）**：机载 agent + 地面聚合 hub + 操作编排 + PySide6 面板（见下文 [GCS 三部曲](#gcs-三部曲toolsgcs)）。
- **YAML 全配置**：场景拓扑 / 竞赛规则 / 机队 / 仿真参数全部 YAML 驱动，改配置无需重编译。

---

## 目录结构（8 个包）

| 包 | 职责 |
|---|---|
| `zx2026_common` | 共享消息（`Mission`/`Score`/`TaskUpdate`）、场景 AABB、几何/配置工具、YAML 配置 |
| `arena_world` | Python 后端：运动学 + 碰撞 + TF + 时钟（`world_node`）、场景 Marker、启动/可视化 launch |
| `arena_world_gazebo` | Gazebo 后端：`world_builder.py` 生成 `forest_world.world`、`drone_vel_plugin.cpp` 力控速度跟踪插件、`gazebo_collision_monitor` 碰撞事件源 |
| `arena_fleet` | 机队管理（6 机注册、错峰调度） |
| `arena_sensor` | 激光雷达模拟（`lidar_node`）、投放点 Tag 几何检测（`tag_detector_node`） |
| `arena_nav` | 在线建图闭环导航（`nav_node.py`：占用栅格累积 + 2.5D A*（`astar.py`）+ P 位置环 + 反应层防御栈） |
| `arena_mission` | 任务状态机（`mission_executor_node`）、任务生成、类型匹配、投放模拟、阶段控制 |
| `arena_score` | 计分（`scorekeeper_node`） |

工具：`tools/` 下有各战役的自检/矩阵脚本（`w5_selftest.py`、`w5_matrix_run.py` 等）与 `tools/gcs/` 地面站全家桶、`tools/fleet_monitor.py` 机群实时监测。

---

## 环境要求

- Ubuntu 20.04 + **ROS 1 Noetic**
- Gazebo 11（仅 Gazebo 后端需要，含 `libsdformat9` / SDF 1.5+）
- Python 3（`rospy`、`yaml`）
- GCS 面板（可选）：`pip3 install --user -i https://pypi.tuna.tsinghua.edu.cn/simple PySide6==6.2.4`（py3.8 上限 6.2.4）

> 平台使用独立 roscore，端口 **11411**（避免与其他仿真冲突），所有脚本已内置 `ROS_MASTER_URI=http://127.0.0.1:11411`。

---

## 编译

```bash
cd ~/zx2026_arena_ws
catkin_make
source devel/setup.bash
```

改 `.py` 无需重编译（`catkin_python_setup` 的 `__path__` 中继，源码即改即生效）；改 `.msg` / `CMakeLists.txt` / `drone_vel_plugin.cpp` 才需 `catkin_make`。

---

## 运行

### 方式一：Python 后端（默认，轻量快速）

```bash
source devel/setup.bash
roslaunch arena_world zx2026_all.launch
```

### 方式二：Gazebo 后端（物理仿真）

```bash
source devel/setup.bash
roslaunch arena_world_gazebo gazebo.launch            # headless
roslaunch arena_world_gazebo gazebo.launch viewer:=true  # 带 gzclient
```

> WSLg 下 `gzclient` 会因 OGRE FBO 模式死锁 `/clock`，请用 `tools/gazebo_live.sh`（已设 `OGRE_RTT_MODE=Copy` + `MESA_LOADER_DRIVER_OVERRIDE=llvmpipe`）。

### 可视化（RViz）

```bash
bash tools/viz_run.sh                 # 推荐（WSLg 软渲染）
# 或 roslaunch arena_world viz.launch
```

---

## 一键端到端验证

```bash
bash run_verify.sh
```

自动完成：清旧进程 → 启动全栈 → 等待节点就绪 → 跑 `verify_run.py`（150s 全流程）→ 校验 **6/6 阶段 DONE、6/6 类型匹配 MATCH、0 碰撞**。参考满分 **158**（110 基础类型分 + 48 路径质量奖励）。

> **同 seed 方差纪律**：单 run PASS/FAIL 有随机方差（W4 族冻结为小概率事件），机制 A/B 必须用 `tools/w5_matrix_run.py` 式同 seed 矩阵 + 触发日志证据判定，禁止单局定论。

---

## 闭环导航（默认模式）

`sim_settings.yaml → closed_loop.enabled: true`：

1. **感知**：订阅 `/drone_<id>/cloud`（lidar 模拟，含顶盲区）。
2. **建图**：点云累积成全局持久占用栅格（世界系），滑窗剪枝治"只加不删"（噪声/漂移错位格永生 → 树周涂抹），碰撞标记格强制占用防 lidar 盲区循环碰撞。
3. **定位**：真值位姿 + 随机游走漂移（`drift_rate`/`drift_max`）= `est_pose`；导航与避障全程只在 est 系。
4. **规划**：2.5D A*（分辨率 0.5m、膨胀 = 机半径 + 0.4m）对自建栅格重规划；目标被膨胀栅格封死时走"目标容忍孔径"退化（`de` 目标封锁直达）。
5. **防御栈**（每项独立开关，详见 `closed_loop` 段注释）：

| 机制 | 治什么 | 默认 |
|---|---|---|
| `swarm`（Boids） | 编队层：分离+对齐+聚合 | 开 |
| `sep_obs_guard` | 机间分离盲推挤树（去指向分量） | 开 |
| `cloud_avoidance.sign_fix` | 云避障符号反转=逃离惩罚吸引子（W5 围栏顶楔死根因） | 开 |
| `collapse_guard` | 返航前瞻航点=自身 → 指令塌缩自锁（W3） | 开 |
| `rescue_mutex` + `bounce_corridor` | de/GOAL-SEAL 双救援打架仲裁 | 开 |
| STALL 看门狗 | 规划健康+机体冻结 2s 诊断 | 开 |
| 死端逃逸三层 + `exit_hyst_s` | 死端 flap 结构性消除 | 开 |
| P1 路径粘滞/翻转锁定 | 路径抖动 | 关（A/B 无净差异） |
| `drift_aware_margin`（dam） | 漂移裕度侵蚀 | 关（v2 教训：贴脸减速=延长暴露） |
| `near_stop_zone` | 近停区地板衰减 | 关（无净差异） |

---

## GCS 三部曲（`tools/gcs/`）

**三条铁律**（W4/W5 法证结论固化）：①ROS 图不过 WiFi（agent 主动外连地面）；②死活按数据年龄判（不信 TCP/旧数据）；③agent 只读不控。

| 步 | 文件 | 内容 |
|---|---|---|
| 1 看得见 | `gcs_agent.py` + `gcs_hub.py` | 机载只读采集（odom/phase/battery/fc/liveness/stage 全 profile 配置化）→ 5Hz TCP JSON 行流；hub 聚合 + 看门狗（LINK LOST/STALL/PLANNER DEAD 边沿触发）+ JSONL 法证 + ANSI 六格 + 1Hz 状态快照文件 |
| 2 编排 | `gcs_ops.py` | 四段健康门 PREFLIGHT→POSITIONING→AUTONOMY→READY（全部从快照判据）；命令层 start/takeoff/back/land/panic（模板含 `{id}`=逐机错峰+重试，不含=全局一次）；仿真 trigger=`rosservice /zx2026/start`，真机=ssh 模板 |
| 3 面板 | `gcs_panel.py` | PySide6 六机卡片+门灯+指令按钮+PANIC 双击确认；指令复用 Ops（Worker 线程，UI 不阻塞）；`--selftest` 自动 E2E 钩子 |

```bash
# 仿真联调三件套（三个终端，或参考 tools/gcs/ 内脚本）
python3 tools/gcs/gcs_hub.py --view dash
python3 tools/gcs/gcs_agent.py --profile tools/gcs/profile_sim.yaml
python3 tools/gcs/gcs_ops.py --profile tools/gcs/profile_sim.yaml preflight   # 四门绿后
python3 tools/gcs/gcs_ops.py --profile tools/gcs/profile_sim.yaml start       # 触发+盯飞
```

**教训**：liveness 活性源必须选恒频话题（`vel_cmd` 20Hz），事件型话题（`path` 重规划才发）= PLANNER_DEAD 假死误报。

---

## 稳健化战役（W1-W5）简史

| 战役 | 根因 | 修复 | 结论 |
|---|---|---|---|
| W1 贴树 | 共享越界点压树干=吸引子；分离盲推 | via_slots 散点槽位 + sep_obs_guard | 12/12 PASS，col=0 |
| W2 走廊 | 44 起碰撞相位归因：北缘三腿共患 48% | P2/P1 部分 parked | 教训：双日志格式归因 |
| W3 返航冻结 | 前瞻航点=自身伺服自锁 | collapse_guard | 6 起实弹零误触发 |
| W4 交付死亡 | plan 健康+cmd 活+机体冻（CG 零触发） | **未破案**，签名已归档，活体取证口诀在案 | 小概率方差 |
| W5 围栏吸引子 | 云避障符号反转（四起"幻影墙"全是围栏顶楔死） | sign_fix | A/B 决定性，翻默认 |

每案方法论：活体/法证取证 → 根因（非现象）→ selftest 数值复现 → 同 seed A/B 矩阵 → 触发日志证据 → 翻默认 → 提交推送。

---

## 真机路线（Route 3：算法整体移植）

真机（Jetson + PX4 + MAVROS + MID360/D435）比赛规则为**自定位建图**——与仿真闭环模式同构：点云→建图→规划。移植清单：

1. `nav_node` + 占用栅格核心搬上 Jetson（rospy/Noetic 同栈，零改动）
2. 点云源适配：faster_lio 配准云（`/cloud_registered`）↔ 仿真 raycast 云（同 PointCloud2 接口；冠簇点由 2.5D z 带天然过滤）
3. 控制链末梢：`vel_cmd` ↔ px4ctrl 设定点接口（薄适配节点；`/px4ctrl/takeoff_land` 已确认用 quadrotor_msgs，规划器→px4ctrl 话题类型待 Diff-Planner 源码核实）
4. 任务层：multipoint ↔ mission_executor 移植版
5. GCS 真机 profile 话题名/现场 IP/ops 脚本路径勘误（profile 内 TODO 标注）
6. HIL：仿真内先换"faster_lio 同构噪声云"跑一轮，再上机

> 仿真闭环模式下调出的**魔法数不可直接上机**（估计漂移特性与动力链不同），阈值全部重标定；防御栈思想与法证流水线原样带走。

---

## 配置

全部在 `src/zx2026_common/config/`：

| 文件 | 内容 |
|---|---|
| `scene_topology.yaml` | 场景（密林/穿越区/投放点/起降区/围栏）拓扑 |
| `competition_rules.yaml` | 评分细则、高度约定、阶段时限、错峰间隔、类型匹配策略 |
| `fleet.yaml` | 机队（6 机）配置 |
| `sim_settings.yaml` | 仿真参数（nav / closed_loop / swarm / collision recovery 各段，每个机制独立开关） |

类型分值：`TYPE_A=10 … TYPE_E=30`，详见 `competition_rules.yaml` 的 `score` 段。

---

## 已知约束

- **碰撞即终结**：`world_node` 对任意碰撞（障碍或机间 `d < 2×机半径`）置 `collided=True` 并冻结速度——导航必须**预防**碰撞，`world_node` 是兜底而非避让。请勿改为非致命。
- **同 seed 方差**：单 run 有小概率 W4 族冻结，机制判定必须多 run 矩阵 + 触发日志。
- 9P/UNC 路径编辑 `.py` 会重置可执行位，`run_verify.sh` 启动前已自动 `chmod +x`。
- 调试日志 `/tmp/zx2026_smoke.log` 含 ROS 颜色码，`grep` 需加 `-a`。
