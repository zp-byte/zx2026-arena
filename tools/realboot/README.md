# realboot — 真机上电一键三件套（2026-09-27 实弹 9/9 全绿固化）

## 角色

| 文件 | 跑在哪 | 作用 |
|---|---|---|
| `boot_zx.sh` | 笔记本 WSL | **唯一入口**：烘 epoch → scp 到机 `~/boot/` → 跑 boot_all → 拉看门狗 |
| `boot_all.zsh` | 机上 | 对时→清场→roscore→mavros(仅 IMU)→faster_lio→ekf→cloud_adapter→D435→color_detector，9 项流判据，退出码=FAIL 数 |
| `d435_watchdog.zsh` | 机上(常驻) | 色流断 2 检≈11s → 重拉 rs+detector（同拉！空壳教训）；仅检出流断 → 只重拉 detector；日志 /tmp/d435_watchdog.log |

## 用法

```bash
# 上电后等 ~30s（Jetson 起来），笔记本 WSL 侧：
bash tools/realboot/boot_zx.sh
# 期望尾部：OK=9 FAIL=0 / WATCHDOG_UP / BOOT_RC=0
```

机上本地重跑（无笔记本时）：`zsh ~/boot/boot_all.zsh`（无参用装订 epoch）。

## 边界（红线）

**boot_all 不含飞行栈**（px4ctrl / vel_bridge / real_fleet.launch 一概不起）。
mavros 在此仅作 **LIO/EKF 的 IMU 被动数据源**（mid360.yaml imu_topic 定谳），
无解锁/OFFBOARD 任何指令通路。任务链按试飞分层另起 real_fleet.launch。

## 依赖次序（缺一不可，2026-09-27 逐级实弹定谳）

```
对时(1970 必修) → roscore → mavros(/dev/ttyTHS1:921600, IMU 265Hz)
  → faster_lio(/laserMapping/odometry；IMU 缺=永不出)
  → ekf(包真名 ekf 非 ekf_pose；订 mavros IMU+LIO odom)
  → cloud_adapter(src=/laserMapping/cloud_registered 前缀定谳)
  → D435(降流参数必带) → color_detector(与 rs 同拉)
```

## 一次性环境依赖（已修，换机重查）

- **nv ∈ dialout 组**（usermod -aG dialout nv；否则 mavros serial open
  Permission denied——2026-09-27 根治）
- FCU 口 = 默认 /dev/ttyTHS1:921600（connected:True 实锤；/dev/ttyUSB0
  CP2102 不通）
- 无 RTC 板：每次上电必须对时（脚本自动，epoch 由笔记本烘入）
