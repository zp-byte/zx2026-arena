# ② 枝梢端点缺口——云记忆时窗 A/B（2026-09-18）

立案：cdd6075 法证（WaveB H 臂残余 0.7 col）——branch_tip t=1.00 端点擦碰
1-5mm、I 臂同梢 1cm 复现；**云记忆 0.3s 过期 vs 1.8m/s 贴脸时窗是残余机制缺口**。

## 机制定量（20260918 复核）

- 反应半径 react_r = drone_radius 0.35 + 0.15 + ca_react_extra 0.6 = **1.10m**
  （nav_node `_apply_cloud_avoidance`）。
- 1.8m/s 巡航穿越反应窗至接触（0.37m）需 **0.41s**。
- 云记忆 `cloud_mem_frames: 3` @ 雷达 10Hz = **0.3s** < 0.41s——梢点在窗入口
  Seen 一次，0.3s 后遗忘时还剩 ~0.19m 贴脸；叠加"进近末段 ~1/3 帧全盲"
  （sim 504 射线/帧 vs 真机 Mid-360 20000 点/帧），反应层在最后一窗失明。

## 修复

1. **杠杆（本矩阵自变量）**：`cloud_mem_frames` 3→6（0.6s = 1.08m ≥ 0.41s
   穿窗 + 盲帧余量）。推力公式/方向零改动——**同一推力更早开始**，非
   J_hiso 式新增强推，无风险再分布面。身后过期点由 v_in>0 门天然豁免
   （sign_fix 分支）；sep_obs_guard 有 into>0 门；建图只走本帧。
2. **伴生修复（先行落地，两臂共同）**：孤立点阈值 ≤3 → ≤max(3, frames)
   （nav_node 20260918）——注释自述"与 cloud_mem_frames≤3 自洽"的耦合在
   延窗后兑现：6 帧回声的梢点不再被 ③ 误判密簇。③（ca_iso_floor）默认关
   维持（J_hiso 臂已毙：硬地板=风险再分布）。

## A/B 设计

- 臂：A_mem3（现默认 3）/ B_mem6，seed{42,43,45} × 2 遍 = 12 cell
  （H 臂残余 0.7 col/run 效应量小，双遍加功效），rep 交错防时段漂移。
- 基线 = 滚动基线（W1 三旗 + W2-P2 boundary_margin 全 ON），WaveB ①③
  pilot 旗显式钉住默认关。
- 工具：tools/tip_matrix_run.py（驱动，克隆 w2_matrix_run，含 report_settle
  race 修复）+ tools/tip_matrix_post.py（归因：trunk 面/枝段 t≥0.85 判
  _TIP/fence/curb，按 cell seed 分组实例化 Scene——枝形随 run_seed 变）。

## 毙杀口径（预登记）

- **B 翻默认**：Σcol 降（或 branch_tip 起数降且 Σcol 不升）∧ score 无 seed
  劣化 ∧ 零 FAIL ∧ done_t 方差带内。
- **B 判毙**：任一 FAIL / score 降 / 碰撞挪移到别的障碍类（J_hiso 风险再
  分布教训，归因对账）/ done_t 回归 >20s。
- minclr 仅参考：记忆加长让身后点驻留 0.6s，读数偏保守（指标敏感非行为
  敏感）。

## 结果（2026-09-18，run_logs/tip_matrix_20260918_163047，12/12 零超时零 FAIL）

| 臂 | Σcol | 逐 cell col | score | done_t 均值 | minclr 最低 |
|---|---|---|---|---|---|
| A_mem3 | **12** | 0/3/1/5/2/1 | 100×4 + **90×2** | 185s | **0.060**（r1_s42） |
| B_mem6 | **7** | 0/3/2/0/2/0 | **100×6** | 179s | 0.269 |

归因（tip_matrix_post.py，按 cell seed 分组实例化 Scene）：

- **A 臂 12 起 100% branch_TIP**（t=0.96-1.00）——cdd6075 立案的梢端点
  形态完全主导基线残余，形态画像精确复现。
- **B 臂 7 起也 100% branch_TIP——类分布零挪移**（J_hiso 对账过）：碰撞
  少发 5 起、没有跑到 trunk/fence/curb 上，机制改善是"少发生"不是"换
  地方"。
- 同梢复现证据仍在：A_r1_s43 tree#19 seg#4 两起（d 1.267/0.330）、
  B_r0_s45 tree#30 seg#0 两起（d 0.929/0.631）。
- 同 seed 跨 run 方差再显形：A_s42 r0=0 col → r1=5 col（90 分）——
  单 run 判读不可靠的纪律第三次实证。
- minclr 注意项被证伪：预登记担心"记忆加长→身后点驻留→读数偏保守"，
  实测 B 5/6 cell minclr 反而更高——真实 clearance 改善幅度大于读数
  保守偏差。

## 判词（2026-09-18）

- **cloud_mem_frames 翻默认 3→6**。预登记翻默认条件全过：Σcol 12→7、
  score 无 seed 劣化（B 100×6 vs A 90×2）、零 FAIL、done_t 无回归
  （均值 179 vs 185s，逐 seed 均差最大 +7.5s 在方差内）。伴生修复
  （孤立点阈值随帧数缩放）同 commit 落地。
- 时窗定量结论入库：**记忆窗必须 ≥ 反应窗穿越时间**（react_r 1.10m @
  1.8m/s = 0.41s → ≥5 帧；6 帧 + 盲帧余量）。改巡航速度或 react_r 时
  此约束须重算。
- 残余立案（未排期）：B 残余 7 起仍全是 branch_TIP——延窗后仍有盲序列
  （>0.5s）或切向 v_in≈0 弱推形态；③硬地板已毙（J_hiso），残余若再
  立案须从"梢端点预测性减速"（O4 UNC 同族）方向找杠杆，不加推力。
