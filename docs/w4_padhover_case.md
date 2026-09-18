# W4 pad-hover 立案（2026-09-18）

立案动机：W3 会话遗留的"pad-hover"疑点——drone 到 pad 段后悬停不落、
落地迟滞（landed_n<6、TOUCHDOWN 缺失）。本文用 34 run 全量对账定谳形态，
登记修复机制候选与 A/B 计划；**实现未排期**。

取证工具：tools/padhover_mine.py（对账 score.yaml 逐机 landed/retired +
executor 日志的 return start / TOUCHDOWN / RECON begin / RECON LOCK 时刻，
算 ret_s / recon_s / sched_s / slack_s 四元组）。数据集：w2 12 cell +
tip 12 cell + genval 10 cell = 34 run × 6 机 = 204 机·run。

## 形态定谳：原假设证伪，真身 = late-return / no-time-to-land

原假设"到 pad 后悬停不落"**不存在**：

- **零 LONG_RET**（>90s 返航飞行，204 机·run 中 0 起）——pad 段没有
  悬停卡滞；到 pad 后 "at pad, descending to hover" → TOUCHDOWN 全部
  正常（正常返航飞行 37-74s）。
- 3 起 NO_TOUCHDOWN（tip A_mem3_r1_s42 d5 / tip A_mem3_r1_s43 d4 /
  genval S47_r0 d3）+ w2 C/D 臂 3 起 no-TOUCHDOWN，**executor 日志齐刷
  刷停在返航中段**（return start / RETURN-VIA reached, cruise to pad），
  末速 1.0-1.5 m/s = 还在飞，非冻结。
- 真身：**RECON 游走过长 → LOCK 晚 → 返航段撞任务终点 T_end**。

## 预算模型（本轮定量的核心）

返航时刻 = LOCK 时刻 + sched，sched = 0.6 + 3.0×d **精确成立**
（204 机·run 全部：d0=0.6, d1=3.6, d2=6.6, d3=9.6, d4=12.6, d5=15.6，
即返航错峰 3s/机）。所以：

```
slack[d] = T_end − LOCK − (0.6 + 3d)      # return start 距终点的余量
落地需要  flight 37-74s（成功样本带），最低成功 slack = 41.3s
```

- 三起 NO_TOUCHDOWN 的 slack = **19.0 / 26.8 / 36.3**（全部 < 41.3）；
  换算 LOCK 距终点仅 34.6 / 39.4 / 46.0s——飞行本身就要 45s+，必撞。
- **LOW_SLACK(<70s) 67/204 ≈ 33%**——三分之一机·run 贴着 40-70s 薄余量
  飞，其中 S48_r1 d4 slack=41.3 ret=39.3（余 2s 惊险落地）。任何把
  LOCK 推迟 ~30s 的扰动都会把一批 d3-d5 转成 NO_TOUCHDOWN。
- 推论（修复的目标线）：LOCK 距 T_end < (0.6+3d) + flight_est + 余量
  ⇒ 必不落地。flight_est 取 45-55s、余量 10s ⇒ **LOCK 截止线 ≈
  T_end − 80s**（d5 最保守）。

RECON 游走时长（RECON begin→LOCK）：正常带 23-60s，离群 77-108s。
LONG_RECON(>80s) 5 起，其中 3 起就是 NO_TOUCHDOWN 三案（90.5/107.8/
101.7s）。慢根因（个案取证）：**"platform freed" 抢占重扫**（RECON
段 plat2→3→1→2→3 五次下降）+ **entry-gate GOTO_LO 30s+ 不 settle**
（diag 行 horiz=... arrived= settled= 长期不齐）。

## 34 run 全量形态清单

| 形态 | 数量 | 明细 |
|---|---|---|
| NO_TOUCHDOWN | 3 | tip A_mem3_r1_s42 d5 / A_mem3_r1_s43 d4 / genval S47_r0 d3 |
| NOT_LANDED | 7 | 上 3 起 + w2 C_p1_s43 d3、D_p1p2_s43 d0/d3、D_p1p2_s45 d3 |
| LONG_RET(>90s) | **0** | — |
| LONG_RECON(>80s) | 5 | 上 3 起 + w2 A_base_s45 d3(82.3)、B_p2_s43 d3(81.3) |
| LOW_SLACK(<70s) | 67 | 结构性集中于 d3/d4/d5（sched 9.6/12.6/15.6） |

w2 C/D 臂 4 起（return_route 判毙臂）统一归因：

- C_p1_s43 d3、D_p1p2_s45 d3：**route 版 no-time-to-land**——LOCK 距
  终点 38/65s，RETURN-VIA 绕行再吃 ~20s，cruise to pad 被终点截断。
  return_route 绕行放大暴露（W2 判毙理由的又一佐证）。
- D_p1p2_s43 d0：retired hover-lock（UNAUTHORIZED_LANDING 姊妹形态，
  W2 已知，另案）。
- D_p1p2_s43 d3：**TOUCHDOWN DONE 距终点 1.6s，scorekeeper 日志明确
  "drone 3 LANDED counted for S2"，但 score.yaml landed=False**——
  判分事件与逐机 flag 脱钩（race 疑点，同 landed56 report race 家族）。
  该臂已判毙不影响现役风险，但 scoring 管线完整性信号，记档待查。

## 修复机制候选（A/B 预登记，未排期）

1. **返航预算预留（return budget guard）**：LOCK 时若
   T_end − now < sched[d] + flight_est + margin → 跳过 scheduled wait
   立即返航。省最多 15.6s（d5），单独不够（三案缺 20-45s）。
2. **RECON 硬截止（recon deadline）**：RECON 游走超过
   T_end − 80s 仍无 LOCK → lock best-so-far（取最近 "RECON over" 的
   平台）强制返航。治本（三案游走 90-108s），但 best-so-far 若颜色
   未确认 → match 错 → 同样 -10：**净收益须 A/B 实证**。
3. **entry-gate settle 修复**：GOTO_LO 30s+ 不 settle 的慢根因单查。
   若可修，LONG_RECON 大头消失，①② 可能都不需要——**优先侦查项**。

A/B 设计：基线 vs 机制臂（先 ③ 单独侦查 entry-gate，再组合臂），
seeds 42/43/45 × 2 + genval seeds 复用。判据：NO_TOUCHDOWN 归零、
score/col 无回归。**毙杀口径预登记**：若 best-so-far 降级导致 match
错误（-10 换 -10 无净收益）或触发新形态（错平台降落），否决；
LOW_SLACK 群体的 slack 分布须左移不明显恶化（>10s 级）。

## 遗留

- entry-gate settle 慢根因（GOTO_LO 30s+）未查——机制③侦查项。
- scorer "LANDED counted for S2 vs landed=False" 脱钩疑点未查
  （D_p1p2_s43 d3，判毙臂内；若现役臂复现再升级）。
- slack 67/204 的薄余量是**结构性**的（sched 错峰 + RECON 常规 30-60s
  就占掉一半预算），机制落地前新矩阵判读应对 d3-d5 的 NO_TOUCHDOWN
  保持警觉。
