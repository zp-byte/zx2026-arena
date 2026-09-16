# 地面站（GCS）启动与操作手册

| 项 | 内容 |
|---|---|
| 项目 | 智信2026 竞技类科目三《自主导航与障碍穿越》 |
| 系统 | GCS（`tools/gcs/`：agent + hub + ops + panel + stack_up.sh） |
| 版本 | v1.0（2026-09-16） |
| 配套文档 | `docs/gcs_srs.md`（FR 规格书） |
| 适用对象 | 联调日现场操作员、比赛日操作员 |

命令均基于仓库根 `~/zx2026_arena_ws`，WSL Ubuntu-20.04 / ROS Noetic，ROS_MASTER_URI 端口 11411。

---

## 1 sim 模式启动（三步）

### 第 1 步 — 一键起栈

```bash
cd ~/zx2026_arena_ws
nohup bash tools/gcs/stack_up.sh > /tmp/stack_up.log 2>&1 & disown
tail -f /tmp/stack_up.log          # 等到打印 "STACK READY"
```

`stack_up.sh` 自动完成：pkill 清场 → `roslaunch arena_world zx2026_all.launch` → 等 35 节点 + stage_controller → 起 `gcs_agent`（profile_sim）→ 起 `gcs_hub`（六机，端口 9870）→ 校验 `run_logs/gcs_status.json` 后退出。

**宿主 = 发起命令的终端会话：关终端即停。**

### 第 2 步 — 另开交互终端起面板（勿塞进 stack）

```bash
cd ~/zx2026_arena_ws
source devel/setup.bash
python3 tools/gcs/gcs_panel.py --profile tools/gcs/profile_sim.yaml
```

⚠️ **设计要点**：面板的 ops 子进程要继承本终端的 ROS 环境。塞进 nohup 栈里，点 START 会报 `Unable to communicate with master`。

### 第 3 步 — 面板操作

点 START（等价 ops 命令行，见 §2）→ 六格卡片 / 态势图 / 比分条 / 倒计时自动上屏；PANIC 按钮随时六机同帧 ABORT。

---

## 2 命令行等价（不开面板时）

```bash
P=tools/gcs/profile_sim.yaml
python3 tools/gcs/gcs_ops.py --profile $P status       # 四段健康门
python3 tools/gcs/gcs_ops.py --profile $P preflight
python3 tools/gcs/gcs_ops.py --profile $P start
python3 tools/gcs/gcs_ops.py --profile $P takeoff --ids 0,1,2
python3 tools/gcs/gcs_ops.py --profile $P back         # 全队返航
python3 tools/gcs/gcs_ops.py --profile $P land
python3 tools/gcs/gcs_ops.py --profile $P panic --yes  # 急停，--yes 免确认
python3 tools/gcs/gcs_ops.py --profile $P debug <名>   # 面板调试窗同款只读命令
```

常用参数：`--ids`（缺省=全队）、`--out`（导出取证）、`--preflight-timeout 90`、`--monitor-timeout`。

---

## 3 real 模式（命令同款，前置未清前只能"面板起得来、连不上机"）

命令形态与 sim 完全一样，profile 换 `tools/gcs/profile_real_lio.yaml`（VIO 机换 `profile_real_vio.yaml`）。**架构差别**：

| 组件 | sim 模式 | real 模式 |
|---|---|---|
| gcs_agent | 本机随栈启动 | **每台飞机上各跑一个**（Jetson 上 `python3 gcs_agent.py --profile profile_real_lio.yaml`） |
| gcs_hub / panel / ops | 本机 | 本机（地面端），经链路聚合六机 |
| 仿真栈 | stack_up.sh 的 SIM 段 | **不需要**（stack_up 只适用于 sim；real 手动分步起） |

**真机联调前置三项 TODO（未清前 real 起不来）**：

1. **IP 网段**——agent→hub 上报地址与实际链路网段对齐（hub 默认收 9870 端口）；六机 IP 规划 + 机侧静态 IP / DHCP 静态绑定。
2. **ssh 公钥**——面板 DEBUG 窗 / ops ssh 调试命令需 `nv@192.168.1.10x` 免密（公钥分发 + `ssh-copy-id`）。
3. **no-prop bench**——`profile_real_lio.yaml` 里 real topic 实配核对（LIO `/Odometry`、twist 恒零时机体自动位置差分）+ 无螺旋桨台架过一遍 agent/hub 通路。

---

## 4 铁律（实战法证换来的）

1. **化石 status 防线**：读 `run_logs/gcs_status.json` 诊断前先 `stat` 看 **mtime**——hub 死后文件冻结成化石，别拿旧状态当活状态。
2. **master 通信错误九成是环境没 source**：面板/ops 报 `Unable to communicate with master` → 回第 2 步的交互终端（`source devel/setup.bash`）。
3. **UNC 编辑掉 exec bit**：从 Windows 侧改过 `tools/gcs/*.py` 或 `stack_up.sh` 后，WSL 侧 `chmod +x` 再启动，否则 roslaunch/脚本 spawn 静默失败。
4. **E2E 前确认零残留**：`pgrep -f 'rosmaster|gzserver|gcs_'` 应为空；stack_up 自带 pkill 清场，但手动排查时先查这条。

---

## 5 关停

- 栈：关掉宿主终端会话即停；或 `pkill -f 'gcs_agent|gcs_hub|gzserver|roslaunch'`。
- 面板：直接关窗口。
- 诊断日志：`~/zx2026_arena_ws/run_logs/`（hub 状态快照 + 各组件输出）。
