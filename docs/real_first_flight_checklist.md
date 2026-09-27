# 单机试飞清单（A/B 双线，2026-09-19）

适用：单架非凸-α（Jetson + MID360 + PX4 + px4ctrl，~/Diff-planner 架构）
的首次试飞与 zx2026 栈上机。前置阅读：gcs_runbook.md §3/§3.1/§3.2；
配套脚本 tools/gcs/{provision_real,bench_check,vel_bridge,cloud_adapter}。

## ⛔ 红线（置顶，触一条即 land，不商量）

- **桨在 bench_check PASS 之前不许装**——顺序不可反。
- **RC 全程在手**——它是唯一不依赖通信链路的保底（panic=land.sh 走
  网络，网络死了就只剩 RC）。异常时 RC 立切 MANUAL，优先级 > 一切。
- **B 线放行闸门 = 反应层静态验证的 push 符号核对**——W5 符号反转=
  逃离惩罚吸引子=炸机级教训，没核过符号不许注速。

## 一、试飞分层（每次只引入一个新变量）

| 线 | 内容 | 验证目标 | 放行条件 |
|---|---|---|---|
| **A 线**（第 1 次飞） | Diff-planner 原生栈：takeoff.sh → 悬停 → land.sh（可选 trigger.sh 航点） | 机体-飞控-RC-链路-LIO 健康度 | 全流程复飞一遍顺 |
| **B 线**（第 2 次飞） | zx2026 栈：cloud_adapter + vel_bridge + 悬停注速 + 低速航点 | 避障算法链路上机 | A 线全绿 + §四.10 符号核对过 |

## 二、上机前桌面动作（半天，勾完再出门）

- [ ] 仓库最新版 pull（含 vel_bridge/cloud_adapter/provision/bench 四脚本）
- [ ] 拷 4 文件上机（scp 到 `~/Diff-planner/tools/`）：`vel_bridge.py`
      `cloud_adapter.py` `gcs_agent.py` `profile_real_lio.yaml`
- [ ] profile 改两处：`server:` 填地面站笔记本实际 IP；`ids: [0]`
- [ ] 首飞限速：B 线 vel_bridge 用 `--max-vel 1.0`（命令抄进试飞单）
- [ ] px4ctrl 两未核项定谳（翻 Diff-planner 源码即可，别留到现场）：
      ① cmd 超时参数名（停发后回悬停的依据）② CMD_CTRL 进档方式
      （RC 拨杆 or `/px4ctrl/track` 服务）
- [ ] 起飞点标记道具 / 真树干或纸箱道具 / 备用桨 备齐

## 三、装备清单

- [ ] 机侧：机体+动力、Jetson、MID360、PX4、**电池×2+充电器**、桨叶备件
- [ ] 地面：笔记本（hub/panel/ops）、**遥控器**（唯一保底）
- [ ] 场地：室内净空 ≥5×5m（**留几面墙/纸箱**——全空旷 LIO 退化）；
      起飞点标记（LIO 原点=起飞点重力对齐，cloud_adapter 地面滤锚依赖）
- [ ] 人员（最少 2）：飞手（RC 在手）+ 地面站操作（panic 在手）；建议+1 安全员

## 四、当天时序

### 无桨阶段（装桨前全部打勾）

- [ ] 1. `IDS="101" bash tools/gcs/provision_real.sh`——免密+探针过
- [ ] 2. 机侧起栈：mavros → faster_lio → px4ctrl（traj_server 可不起）
- [ ] 3. `bash tools/gcs/bench_check.sh --id 0` ——**四段全绿**
       （话题/服务在列 ｜ LIO ≥20Hz ｜ mavros connected ｜ 桥链路回环）
- [ ] 4. panic 预演：`ops panic`（=land.sh）dry-run，服务通+机体语义确认

### 装桨阶段（红线：bench PASS 之后才许装桨）

- [ ] 5. 电池 ≥30%（bat_min 门）；机体放起飞点、静置水平
- [ ] 6. LIO 初始化 + 30s 静置：位置漂移 <0.1m 才算过（超了换起飞点/
       查特征，别硬飞）
- [ ] 7. GCS 链路：机载 `gcs_agent.py --profile profile_real_lio.yaml`
       → 地面 hub/panel 起 → 面板 d0 格遥测活
- [ ] 8. **A 线首飞**：takeoff.sh → 1m AUTO_HOVER 悬停 30s（目视漂移）
       → RC 逐档空切演练（MANUAL↔HOVER↔CMD_CTRL，不接舵）
       → land.sh 落地 → **复飞一遍全流程**
- [ ] 9. **B 线首飞**（A 线全绿后）：机载起 cloud_adapter + vel_bridge
       → `rostopic echo /drone_0/cloud` 抽查：有点且 **z 无地面带**（无
       (z−odom_z)<−0.6m 的点）→ RC 扳 CMD_CTRL → 悬停注速 0.5m/s 顶推
       验响应方向 → 低速航点 → land.sh
- [ ] 10. **反应层静态验证（B 线放行闸门）**：树干/纸箱置前方 2m，
       观察悬停时机体推离方向 = **背向障碍**才算过；推近/吸拢 = 符号
       反转，立即停，查 sign（W5 教训）再约下次

## 五、中止红线 + 紧急处置

| 触发 | 处置 |
|---|---|
| LIO 漂移 >0.3m | land.sh → RC 待命 |
| mavros 断连 | RC 立切 MANUAL |
| 电池 <30% | land.sh |
| 链路断 >3s（hub link_lost_s 门） | RC 接管，查链路后再飞 |
| 姿态/异响异常 | **RC 立切 MANUAL，不商量** |

## 六、试飞记录（当天留档）

- 机侧 log：`~/.ros/log/` 最新目录 + `/tmp/bench_*.log` 收进 `run_logs/`
- 登记：日期/机体号/电池号/A 线结果/B 线结果/中止原因/下一步，附面板截图
- 采数任务（下次标定用）：/Odometry 频率与延迟、/cloud_registered 点数
  与 topic 名、悬停 60s 漂移——mem6 定量律与 react_r 重算的原始输入

——勾完全部 = 具备 zx2026 栈单机自由飞资格；六机另案（机间观测来源
未解决前不上多机）。

## 九、2026-09-27 定谳补录（px4ctrl 两未核项闭环 + 新雷三枚）

> 原预登记"px4ctrl 两未核项定谳（翻 Diff-planner 源码即可，别留到现场）"——
> 本日连同新发现共五项闭环，全部源码/实弹取证，无需到场再查。

**① cmd 超时**：`ctrl_param_fpv.yaml` → `msg_timeout.cmd: 0.5`（s）。vel_bridge
50Hz 发流远超门限；停发 0.5s 后 CMD_CTRL→AUTO_HOVER（HOLD 定点语义成立）。

**② 进档方式 = RC 拨杆，无 track 服务**（PX4CtrlFSM.cpp 实锤）：
MANUAL(L1) →（RC hover 档）AUTO_HOVER(L2) →（RC command 档 + FCU OFFBOARD +
cmd 流在）CMD_CTRL(L3)；L3 退档 = RC 切回或 cmd 断 0.5s → AUTO_HOVER。
AUTO_LAND 只能在 AUTO_HOVER 触发（CMD_CTRL 里 reject）。
发布 TRIGGER（publish_trigger）是给规划器的，不是进档服务——勿混。

**③ vel_bridge 注入话题错位（首飞级隐患，已修 real_fleet.launch）**：
run_ctrl_lio.launch 把 px4ctrl `~cmd` remap 到 **/setpoints_cmd**（源码实锤），
vel_bridge 默认 `--out /position_cmd` 是历史误记——不纠偏注入打空、CMD_CTRL
永不生效。已在 real_fleet.launch 显式 `--out /setpoints_cmd` 纠偏，本文档与
vel_bridge.py 用法行同步改。

**④ mavros 是 LIO/EKF 的 IMU 硬依赖（数据链，非控制链）**：
faster_lio mid360.yaml `imu_topic: /mavros/imu/data`（Option 2，/livox/imu 被注
释）；ekf_lidar.launch 同订 mavros IMU。mavros 不起则 faster_lio 永不出
odometry。**nv 曾不在 dialout 组 → serial open Permission denied**（本日根治
usermod -aG dialout）；FCU 口定谳 = 默认 `/dev/ttyTHS1:921600`（connected:True
+ IMU 265Hz；/dev/ttyUSB0 CP2102 反而不通）。mavros 在 boot_all 中仅作被动数
据源，无解锁/OFFBOARD 指令通路。

**⑤ ekf 包真名 = `ekf`**（源码目录叫 ekf_pose，pkg 声明是 ekf）——
`roslaunch ekf ekf_lidar.launch`，ekf_pose 是 RLException。

**上电一键**：本日固化 `tools/realboot/`（boot_zx.sh 一键 → boot_all.zsh 传感器
链 9 检 + d435_watchdog.zsh 看门狗），机上落 `~/boot/`。依赖次序：对时→
roscore→mavros(IMU)→faster_lio→ekf→cloud_adapter→D435→detector，全绿=传感器
链就绪。飞行栈（px4ctrl/vel_bridge/real_fleet.launch）不在其中，按分层另起。
