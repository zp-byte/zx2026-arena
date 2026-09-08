# 非凸-α 六机地面站（GCS）规格需求说明书

| 项 | 内容 |
|---|---|
| 项目 | 智信2026 竞技类科目三《自主导航与障碍穿越》 |
| 系统 | GCS 地面控制站（`tools/gcs/`：agent + hub + ops + panel + profiles） |
| 版本 | v1.0（2026-09-07） |
| 现状基线 | GCS v0（320b1e4 … c1fe995，离线 42/42 + sim E2E PASS） |
| 适用对象 | 地面站开发、联调日现场操作员、比赛日操作员 |

需求状态标记：✅ 已实现并验证 ｜ 🔶 部分实现 ｜ ⬜ 待实现（附优先级 P0/P1/P2/P3）。
验证方式标记：T=离线测试 ｜ D=实弹演示 ｜ I=人工检查。

---

## 1 引言

### 1.1 目的

规定科目三六机集群地面站的功能、接口、非功能与验收要求，作为真机联调与比赛日部署的依据。需求来源三类：**比赛规则强制**（competition_rules.yaml）、**安全链强制**（真机炸机风险）、**通信物理强制**（WiFi 不可靠）。

### 1.2 范围

GCS 覆盖赛前自检 → 起飞编排 → 飞行监视 → 应急处置 → 赛后取证的全流程操作支撑。**不含**：机载自主导航/任务算法本身（属 arena_* 包）；裁判计分系统；场地设施。

### 1.3 术语

| 术语 | 定义 |
|---|---|
| agent | 机载只读采集节点（gcs_agent.py），每机一个，随 Jetson 部署 |
| hub | 地面聚合服务器（gcs_hub.py）：TCP 聚合 + 看门狗 + 法证 |
| ops | 操作编排（gcs_ops.py）：健康门 + 命令层 + 盯飞 |
| panel | PySide6 操作面板（gcs_panel.py） |
| profile | 换场配置（profile_sim / real_lio / real_vio.yaml），话题/命令模板全配置化 |
| 数据年龄 | 最近一帧遥测距现在的时长；**死活唯一判据**（不信 TCP 连接） |
| ABORT | 操作员急停终态；与 FAILED（任务失败）语义分离 |
| 化石数据 | hub 死后冻结的 status/日志；诊断前必验 mtime |

### 1.4 参考文件

- `src/zx2026_common/config/competition_rules.yaml`（评分/时限/错峰/类型匹配）
- `tools/gcs/README 章节：repo README.md "GCS 三部曲"`
- W3/W4/W5 法证记录（架构铁律出处）

---

## 2 总体描述

### 2.1 运行场景（比赛日时间轴）

```
赛前(≥30min)        联调勘误→起栈→probe自检→门检查→[READY]
裁判发令(0s)        START：错峰takeoff→离地确认→trigger（600s 计时开始）
飞行中(0-600s)      六机穿越/识别/投放/返航；操作员盯屏，异常处置
终态                6/6 DONE（或 FAIL/ABORT）→取证导出
```

操作员 1 名盯 6 机，比赛决胜链 `tiebreak: [total_score, completion_time, first_correct_drop_time]`——**时间与比分同为一级战力**。

### 2.2 角色

| 角色 | 职责 | GCS 支撑 |
|---|---|---|
| 操作员（1 名） | 起飞放行、监视、急停、现场决策 | panel 全部功能 |
| 地面站保障（可兼） | 起栈、勘误、日志导出 | stack_up.sh / hub CLI / JSONL |

### 2.3 运行环境

- 机载：Jetson Orin NX，Ubuntu 20.04 + ROS Noetic，agent 随自主栈常驻（systemd 自启）
- 地面：笔记本，Ubuntu 20.04（WSL2 亦可）+ Python3.8 + PySide6 6.2.4
- 链路：现场 WiFi（机→地单向 TCP；ssh 地→机单向）

### 2.4 设计约束（铁律，违反=返工）

| # | 约束 | 出处 |
|---|---|---|
| C-1 | **ROS 图不过 WiFi**：机载 agent 主动 TCP 外连地面，地面侧不进 ROS 图 | W4/W5 法证 |
| C-2 | **死活按数据年龄判**：一切活性判定用遥测时间戳，不用 TCP 连接态 | 同上 |
| C-3 | **agent 只读不控**：控制命令唯一通道 = ops 命令层（sim=话题/服务，real=ssh） | 同上 |
| C-4 | **活性源选恒频话题**（如 vel_cmd 20Hz）；事件型话题（path 类）= 假死误报 | GCS v0 教训 |
| C-5 | **panic 永不设障碍**：不排队、不二次确认（面板双击已是下限）、CLI 不需 --yes | 安全纪律 |
| C-6 | **危险命令两段确认**：start/takeoff/back/land 必须 --yes / arm 确认 | 同上 |
| C-7 | **诊断先验化石**：读 status/日志前必验 mtime + 进程存活 | 2026-09-06 事故 |

---

## 3 功能需求

### 3.1 态势感知域（FR-1）

| ID | 需求 | 状态 | 验证 |
|---|---|---|---|
| FR-1.1 | 每机遥测采集：位姿/速度/相位/电量/FC 状态/规划活性/全局阶段，5Hz，话题名 profile 配置化 | ✅ | T/D |
| FR-1.2 | 活性判定：数据年龄 > link_lost_s(3s) 判失联，恢复自动解除；面板卡片同语义（NO LINK） | ✅ | T |
| FR-1.3 | 告警四类边沿触发+恢复成对：LINK LOST / STALL（相位活跃且 v<0.05 持续 5s）/ PLANNER DEAD（活性源年龄>5s）/ LOW_BAT（持续 5s 防瞬时压降，--bat-min 开） | ✅ | T/D |
| FR-1.4 | 合并建图态势图：六机占用栅格世界坐标合并、未知/已探索分离着色、机位航向箭头/轨迹尾迹、分机配色、比例尺、建图占比；缩略图点击高亮单机；500ms 自刷新 | ✅(sim) ⬜(real, P1) | D |
| FR-1.5 | 比分与任务指派实时上屏：总分/正确/错误投放、每机指派（类型→投放点→分值）、卡片分值；真机无 scorekeeper 显示 -- | ✅ | T/D |
| FR-1.6 | **全场倒计时**（600s）与各机节奏指示（相位腿推进）；tiebreak 时间敏感，操作员须实时知道"这个比分花了多少秒" | ⬜ P1 | D |
| FR-1.7 | 碰撞计数与最小 clearance 上屏（nav_metrics 11 项已发布，agent 捎带即可）；碰撞为一级告警 | ⬜ P1 | T |
| FR-1.8 | 投放事件与首投时间戳记录（tiebreak 第三键 first_correct_drop_time） | 🔶 P2 | T |
| FR-1.9 | 类型匹配过程可视化：识别置信度/连续帧计数/悬停等待超时（12s）状态，非仅终态 MATCH | ⬜ P3 | D |

### 3.2 编排控制域（FR-2）

| ID | 需求 | 状态 | 验证 |
|---|---|---|---|
| FR-2.1 | 健康门：sim 四门 PREFLIGHT→POSITIONING→AUTONOMY→READY；real 六门（+POWER 电量≥bat_min、+FC mavros connected），全部从 hub 快照判定，面板门灯随 profile 动态 | ✅ | T/D |
| FR-2.2 | START 编排：sim=trigger（rosservice /zx2026/start）；real=错峰 takeoff→离地确认（z≥airborne_z 0.5m，超时 60s abort 宁可不起飞）→trigger（模板 null=任务自启跳过） | ✅ | T/D |
| FR-2.3 | 命令层模板化：status/preflight/start/takeoff/back/land/panic/debug；模板含 {id}=逐机错峰+重试+逐机汇报，不含=全局一次；三态汇报（None=未执行/全局广播≠逐机确认/逐机明细） | ✅ | T |
| FR-2.4 | 危险命令两段确认：CLI --yes；panel arm 5s+弹窗；real 全按钮两段确认；**panic 豁免**（C-5） | ✅ | T/D |
| FR-2.5 | 空中误发保护：real start 时任一机 z≥airborne_z 拒绝（--force 放行） | ✅ | T |
| FR-2.6 | 回滚：离地确认失败自动对已离地机 land；回滚失败必显式喊人（ROLLBACK_FAIL 列出未确认机） | ✅ | T |
| FR-2.7 | 盯飞：start 阻塞至终态，5Hz 行输出+MISSION_END 打分；real phase=null 时按 MONITOR_TIMEOUT 语义转人工盯 | ✅ | D |
| FR-2.8 | 真机阶段推进推断：无状态机话题时以航迹到达事件（投放点邻域悬停/返航向）推断腿推进，替代 phase | ⬜ P2 | D |

### 3.3 安全应急域（FR-3）

| ID | 需求 | 状态 | 验证 |
|---|---|---|---|
| FR-3.1 | PANIC：sim=全局话题 /zx2026/abort 一帧广播六机同秒 ABORT + hover-lock 悬停锁定 + 任务冻结；real=逐机 ssh land 模板链（单机失败不中止）；面板双击确认+5s 解除；CLI 免 --yes | ✅ | D |
| FR-3.2 | panic 三态汇报：None=未送达（rc=2 喊人）/ 全局广播（明示无逐机确认语义）/ 逐机 INCOMPLETE 列名 | ✅ | T |
| FR-3.3 | ABORT≠FAILED：终态三分 DONE/FAILED/ABORT，盯飞只计 FAILED 为败 | ✅ | T/D |
| FR-3.4 | FC 模式切换告警：OFFBOARD 丢失/模式变更（=有人接管或失控）边沿告警，一级 | ⬜ **P0** | T |
| FR-3.5 | 围栏越界告警：真机位置出场地边界（含高度带）边沿告警 | ⬜ P2 | T |
| FR-3.6 | 估计器健康告警：VINS 输出断流/跳变检测 | ⬜ P2 | T |

### 3.4 通信架构域（FR-4，约束性）

| ID | 需求 | 状态 | 验证 |
|---|---|---|---|
| FR-4.1 | agent 主动 TCP 外连（C-1）：5Hz JSON 行流，消息即心跳，seq 单调，断线自动重连，只发最新快照 | ✅ | T/D |
| FR-4.2 | 带宽预算：遥测 ≤5Hz×6 机 JSON 行；栅格 RLE base64 1Hz 捎带（单图 ≤1.1KB） | ✅ | D |
| FR-4.3 | agent 只读（C-3）：不订阅/不发布任何控制话题 | ✅ | I |
| FR-4.4 | hub 工程健壮性：TCP keepalive（半开连接回收）、allow_reuse_address、写盘失败事件化不静默 | ✅ | T/D |

### 3.5 法证取证域（FR-5）

| ID | 需求 | 状态 | 验证 |
|---|---|---|---|
| FR-5.1 | 三层全量 JSONL：遥测流 / 事件流（告警+命令+异常）/ ops 命令流水，含时间戳可回放 | ✅ | T |
| FR-5.2 | hub 1Hz 原子状态快照文件：ops/panel 的**唯一**数据源（编排进程不进 ROS 图） | ✅ | T |
| FR-5.3 | 取证导出：赛后一键打包 run_logs（遥测/事件/比分/终端态对账） | 🔶 P2 | D |

### 3.6 现场调试域（FR-6）

| ID | 需求 | 状态 | 验证 |
|---|---|---|---|
| FR-6.1 | ssh 只读命名命令 9 条：ping/ws/ros/hz/echo/node/proc/res/log（{args} 参数槽）；ssh 三参数纪律 BatchMode+ConnectTimeout+accept-new | ✅ | T/D |
| FR-6.2 | 面板 DEBUG 窗：选机+选命令+RUN 多行回显；sim 模式置灰；与主命令互斥（忙时可见拒绝，panic 豁免） | ✅ | T |
| FR-6.3 | preflight probe 启动自检：按 profile 探针名单自动跑（ping/ws/ros），失败告警**不阻断门** | ✅ | D |
| FR-6.4 | 只读约定：DEBUG 窗命令约定只读，写操作一律走命令层两段确认 | ✅ | I |

### 3.7 界面域（FR-7）

| ID | 需求 | 状态 | 验证 |
|---|---|---|---|
| FR-7.1 | 六机卡片：LINK/STALL/PLANNER(/LOW_BAT) 徽章、相位、速度、分值；数据超龄=NO LINK 与 hub 同语义 | ✅ | T/D |
| FR-7.2 | sim/real 视觉区分（蓝/红头幅）；空模板按钮禁用置灰（不可达命令不给点） | ✅ | T |
| FR-7.3 | 告警分级呈现：一级（碰撞/OFFBOARD 丢失/LINK LOST/PANIC 需求）视觉强化+**声音**；二级观察；三级信息滚动 | 🔶 P1（声音未做） | D |
| FR-7.4 | --selftest E2E 钩子：READY 自动 START、全终态自动退出 | ✅ | T |

---

## 4 接口需求

### 4.1 输入（agent 侧，全 profile 配置化）

| 数据 | sim | real_lio / real_vio |
|---|---|---|
| odom | /drone_{id}/odom | LIO/VIO 里程计（如 /vins/imu_propagate） |
| phase | /drone_{id}/phase | **null**（multipoint 无状态机话题→FR-2.8） |
| battery | null（自动跳过） | /mavros/battery |
| fc | null | /mavros/state（connected + 模式，FR-3.4 依赖） |
| liveness | /drone_{id}/vel_cmd（恒频 20Hz，C-4） | 候选 /position_cmd（100Hz，联调日 hz 探针定谳） |
| stage | /zx2026/state | null |
| grid | /drone_{id}/occ_grid（OccupancyGrid） | null（待 PointCloud2 适配器，FR-1.4） |
| score/mission | /zx2026/score/{id} / mission/{id} | null |

### 4.2 输出（ops 命令层，模板化）

| 命令 | sim | real |
|---|---|---|
| trigger | rosservice /zx2026/start | ssh pub_trigger.sh |
| takeoff/back/land | —（任务内含） | ssh sh_files/*.sh（cd ~/Diff-planner） |
| panic | rostopic pub -1 /zx2026/abort（全局一帧） | ssh 逐机 land |
| debug | 置灰 | ssh 只读命令（FR-6.1） |

### 4.3 数据链与文件

- TCP：机→地 JSON 行流（端口 9870，hub 监听）
- 文件：`run_logs/gcs_status.json`（1Hz 原子）、`gcs_telem_*.jsonl`、`gcs_ops_*.jsonl`
- ssh：地→机（BatchMode 免密；联调日 ssh-copy-id，禁 sshpass）

---

## 5 非功能需求

| ID | 类别 | 需求 | 验证 |
|---|---|---|---|
| NFR-1 | 性能 | 遥测端到端延迟 ≤1s（agent 采集→panel 显示）；panel 轮询 2Hz | D |
| NFR-2 | 性能 | PANIC 从点击到指令发出 ≤1s；六机同帧翻转（sim 已实测同秒） | D |
| NFR-3 | 可靠性 | hub 单进程长时存活（比赛全程）；异常退出可 10s 内重启恢复（快照文件续读） | D |
| NFR-4 | 可靠性 | 化石免疫：一切诊断先验 mtime（C-7）；status 写失败必须可见 | T/D |
| NFR-5 | 安全 | panic 路径零依赖：不依赖 hub 存活、不依赖 panel 存活（CLI 直发）、不依赖门状态 | T/D |
| NFR-6 | 可用性 | 单操作员 6 机：告警边沿化不刷屏；一级告警 1s 内不可错过 | D |
| NFR-7 | 可移植性 | 换场只换 profile；代码零改动 | T |
| NFR-8 | 资源 | agent 机载 CPU 占用可忽略（只读订阅+5Hz 打包）；Jetson 上不影响自主栈 | D |
| NFR-9 | 可测试性 | 离线注入测试覆盖门判定/告警形态/命令三态/畸形数据防御（现 42/42）；sim --selftest 全自动 E2E | T |

---

## 6 验收准则

### 6.1 离线（每版本发布门）

- [ ] 离线断言全绿（含：四门/六门判定、告警边沿、panic 三态、畸形分值防御、DEBUG 互斥）
- [ ] panel offscreen T1-T6 全过

### 6.2 sim 实弹 E2E

- [ ] stack_up → panel --selftest：四门绿→自动 START→6/6 DONE→SELFTEST_PASS
- [ ] 中途 panic：六机同帧 ABORT、hover-lock 生效、ABORT 终态正确归类
- [ ] MAP 窗：栅格 1Hz 刷新、六机合并正确、起飞前 "no grid yet" 为预期

### 6.3 真机联调日（比赛资格门）

- [ ] **勘误四件套落实**：机 IP 网段、ssh-copy-id 免密、trigger 形式、脚本绝对路径（profile TODO 逐项销账）
- [ ] probe 三探针（ping/ws/ros）全绿；**ws 探针复核目录大小写 Diff-planner**
- [ ] 六门到 READY；FC 门实测 mavros connected
- [ ] **panic 无桨台架实弹验证**（profile 既定纪律）：逐机 land 送达+汇报三态
- [ ] start 流程：错峰 takeoff→离地确认→trigger；模拟离地失败演示 land 回滚+ROLLBACK_FAIL 喊人
- [ ] liveness 活性源 hz 探针定谳（恒频贯穿悬停+任务两态）
- [ ] agent systemd 自启；断电重启后自动回连

### 6.4 比赛日晨检

- [ ] 起栈→probe→门检查全绿；倒计时/比分/MAP 三屏可用
- [ ] JSONL 落盘确认（赛后取证依赖）

---

## 7 需求-依据追溯（关键条）

| 需求 | 依据 |
|---|---|
| FR-1.6 倒计时 | competition_rules: time_limit_s=600 + tiebreak 三元组 |
| FR-1.7 碰撞上屏 | 碰撞即终结（Python 后端）/碰撞预算 8（Gazebo）；734 聚类法证价值 |
| FR-1.9 匹配过程 | type_match: min_confidence 0.8 / consecutive_frames 3 / hover_timeout 12s / position_gate |
| FR-2.2 起飞先行 | takeoff_stagger 3s + phase_timeout 60s |
| FR-2.5 空中误发保护 | 真机安全（起飞中重复 trigger=二次起飞指令风险） |
| FR-3.4 OFFBOARD 告警 | 人工接管/失控的唯一 mavros 可见信号 |
| FR-4.x 通信架构 | W4/W5 法证：ROS 图过 WiFi=不可诊断的静默死 |
| C-4 恒频活性源 | v0 事故：path 事件型话题→PLANNER_DEAD 假死抖动 |
| NFR-5 panic 零依赖 | 急停链路不允许单点（hub/panel 死亡时 CLI 仍可发） |

---

## 8 开放项与排期建议

| 优先级 | 项 | 需求 | 工作量 |
|---|---|---|---|
| **P0** | 联调勘误四件套 | FR-6/4.2 | 半天（现场） |
| **P0** | OFFBOARD 丢失告警 | FR-3.4 | 小（agent 已订阅 mavros/state，加边沿对） |
| P1 | 真机态势图适配器（PointCloud2→栅格→RLE 复用） | FR-1.4 | 中 |
| P1 | 碰撞计数/clearance 上屏（nav_metrics 捎带） | FR-1.7 | 小 |
| P1 | 倒计时+节奏指示 | FR-1.6 | 小 |
| P1 | 一级告警声音 | FR-7.3 | 小 |
| P2 | 真机阶段推断 / 围栏越界 / 估计器健康 / 取证一键打包 | FR-2.8/3.5/3.6/5.3 | 中 |
| P3 | 匹配过程可视化 | FR-1.9 | 小 |

> 排期原则：P0=没有它比赛日发不出/停不下；P1=显著提升操作员决策质量；P2/P3=锦上添花，不阻塞比赛资格。
