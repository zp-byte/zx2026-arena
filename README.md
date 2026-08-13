# 智信2026 · 科目三仿真平台

「智信—2026」竞技类科目三 **自主导航与障碍穿越** 的官方配套仿真平台：6 机集群在密林中自主穿越，将不同种类物品投放到类型匹配的投放点。基于 **ROS 1 Noetic**（Ubuntu 20.04），提供 **Python** 与 **Gazebo** 两个可插拔物理后端，评分/任务栈对后端透明。

> 完整赛题、真机部署与算法分析见 `c:/Users/24882/Desktop/资料/非凸α_docs/` 下的《自主导航与障碍穿越_实验方案》系列文档。

---

## 特性

- **双物理后端**：轻量 Python 运动学后端（`world_node`）与 Gazebo 物理后端（`gzserver` ODE + 自研 `libdrone_vel_plugin`），两者共享同一消息契约，任务/评分代码零改动切换。
- **密林自主穿越**：2.5D A* 全局规划 + 四层避障防御（全局规划 → 位置环 → 机间分离 → 动量感知静态避障）。
- **6 机集群**：去中心化 + 时间错峰（起飞/进林/返航三段错峰），机间软斥力 + 硬速度障碍分离。
- **类型匹配投放**：投放点视觉标识类型码与机载物类型匹配，匹配成功才触发投放。
- **YAML 全配置**：场景拓扑 / 竞赛规则 / 机队 / 仿真参数全部 YAML 驱动，改配置无需重编译。

---

## 目录结构（8 个包）

| 包 | 职责 |
|---|---|
| `zx2026_common` | 共享消息（`Mission`/`Score`/`TaskUpdate`）、场景 AABB、几何/配置工具、YAML 配置 |
| `arena_world` | Python 后端：运动学 + 碰撞 + TF + 时钟（`world_node`）、场景 Marker、启动/可视化 launch |
| `arena_world_gazebo` | Gazebo 后端：`world_builder.py` 生成 `forest_world.world`、`drone_vel_plugin.cpp` 力控速度跟踪插件 |
| `arena_fleet` | 机队管理（6 机注册、错峰调度） |
| `arena_sensor` | 激光雷达模拟（`lidar_node`）、投放点 Tag 几何检测（`tag_detector_node`） |
| `arena_nav` | 2.5D A* 规划（`astar.py`）、P 位置环 + 机间分离 + 动量避障（`nav_node.py`） |
| `arena_mission` | 任务状态机（`mission_executor_node`）、任务生成、类型匹配、投放模拟、阶段控制 |
| `arena_score` | 计分（`scorekeeper_node`） |

---

## 环境要求

- Ubuntu 20.04 + **ROS 1 Noetic**
- Gazebo 11（仅 Gazebo 后端需要，含 `libsdformat9` / SDF 1.5+）
- Python 3（`rospy`、`yaml`）

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

---

## 关键机制

### 避障（四层防御）

1. **2.5D A\*** 全局规划：分辨率 0.5m，障碍膨胀 = 机半径 + 0.4m，八邻域对角代价 1.414。
2. **P 位置环**：`nav_node` 跟踪 A* 路径点。
3. **机间分离**：软斥力（`sep_radius`）+ 硬速度障碍（`hard_r = 2×机半径 + 1.1`）。
4. **动量感知静态避障**：EMA 估计速度 → 制动距离 `v²/2a` 前瞻，防止机间分离把机子横向推入树（此前主要碰撞源）。

### 集群（去中心化 + 时间错峰）

- 起飞错峰 `3.0s` / 进林错峰 `2.5s` / 返航错峰 `3.0s`（`competition_rules.yaml`），避免六机同挤入口与返航走廊。
- 各机独立 `nav_node` + `mission_executor`，无中心协调器。

### 类型匹配投放

投放点预设视觉标识（注明投放类型），`tag_detector_node` 几何检测 → 解析类型码 → 与本机载物类型比对 → 匹配成功触发投放；位置门限 + 连续帧确认 + 悬停超时三重安全（`type_match` 配置）。

---

## 配置

全部在 `src/zx2026_common/config/`：

| 文件 | 内容 |
|---|---|
| `scene_topology.yaml` | 场景（密林/穿越区/投放点/起降区）拓扑 |
| `competition_rules.yaml` | 评分细则、高度约定、阶段时限、错峰间隔、类型匹配策略 |
| `fleet.yaml` | 机队（6 机）配置 |
| `sim_settings.yaml` | 仿真参数 |

类型分值：`TYPE_A=10 … TYPE_E=30`，详见 `competition_rules.yaml` 的 `score` 段。

---

## 已知约束

- **碰撞即终结**：`world_node` 对任意碰撞（障碍或机间 `d < 2×机半径`）置 `collided=True` 并冻结速度——导航必须**预防**碰撞，`world_node` 是兜底而非避让。请勿改为非致命。
- 9P/UNC 路径编辑 `.py` 会重置可执行位，`run_verify.sh` 启动前已自动 `chmod +x`。
- 调试日志 `/tmp/zx2026_smoke.log` 含 ROS 颜色码，`grep` 需加 `-a`。
