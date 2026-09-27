#!/usr/bin/env bash
# =============================================================================
# deploy_real.sh — 适配包一键导送非凸α真机（2026-09-26）
# =============================================================================
# 在 WSL 侧跑（工作空间根所在机器），把四个 ROS 包 + tools/gcs 打 tar 经
# scp 导到机载 Jetson，机载侧解包+配置换名+catkin build+探针自检。
# 机载无需 rsync/git/外网——只要求 ssh 免密已通（tools/gcs/provision_real.sh
# 负责）。
#
# 布局（机载，overlay 工作空间，官方栈 ~/Diff-planner 为 underlay）：
#   ~/zx2026_ws/src/{zx2026_common, arena_nav, arena_mission, arena_bridges}
#   ~/zx2026_ws/tools/gcs/           （agent/profile/missions/launch 参数引用件）
#   ~/zx2026_ws/src/zx2026_common/config/sim_settings.yaml
#       ← settings_real.yaml 换名拷贝（cfg.load 硬编码文件名，零代码侵入；
#         实测定谳 2026-09-26：import 命中 src 源树，config/ 在包内即生效）
#   ~/zx2026_ws/src/zx2026_common/config/scene_topology.yaml
#       ← scene_topology_real.yaml 换名拷贝（真场地 50×50 角原点/零树；
#         WSL sim 的杨树林文件不上机，sim 零影响）
#   ~/zx2026_ws/src/zx2026_common/config/competition_rules.yaml
#       ← competition_rules_real.yaml 换名拷贝（真机高度链/1500s/真场 geofence）
#
# 用法（WSL 侧）：
#   bash tools/gcs/deploy_real.sh nv@192.168.31.76          # 全量导送+构建
#   bash tools/gcs/deploy_real.sh nv@192.168.2.50 --no-build  # 只导码不构建
#   bash tools/gcs/deploy_real.sh dry                        # 本机演练（不 ssh）
#
# 重复执行幂等（tar 解包覆盖）；机载改过的 sim_settings.yaml 会被 WSL 源
# 覆盖——现场手调参数后先回拷 WSL 再重导（防覆盖丢失）。
# =============================================================================
set -euo pipefail

TARGET="${1:?用法: deploy_real.sh <user@host|dry> [--no-build]}"
NO_BUILD=false
[ "${2:-}" = "--no-build" ] && NO_BUILD=true
DRY=false
[ "$TARGET" = "dry" ] && DRY=true && TARGET=dryhost

HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(dirname "$(dirname "$HERE")")"          # 工作空间根
PKGS="src/zx2026_common src/arena_nav src/arena_mission src/arena_sensor src/arena_bridges"
TGZ=/tmp/zx2026_deploy.tgz

echo "== [1/5] 打包（四包 + tools/gcs）=="
cd "$WS"
for p in $PKGS; do [ -d "$p" ] || { echo "缺包 $p"; exit 1; }; done
[ -f src/zx2026_common/config/settings_real.yaml ] || {
  echo "缺 settings_real.yaml"; exit 1; }
python3 -c "import cv2" 2>/dev/null || echo "⚠ WSL 无 cv2（仅影响本机干测；机载 YOLO 栈自带）"
tar czf "$TGZ" --exclude='__pycache__' --exclude='*.pyc' $PKGS tools/gcs
echo "   $(du -h "$TGZ" | cut -f1) -> $TGZ（已排 __pycache__：WSL 陈年 pyc 的
   mtime 与源严格配对会被机载 Python 3.8 直接吃掉——跨机导送一律不带缓存）"

if $DRY; then
  echo "== [dry] 本机演练：解到 /tmp/zx2026_dry 验布局 =="
  rm -rf /tmp/zx2026_dry && mkdir -p /tmp/zx2026_dry
  tar xzf "$TGZ" -C /tmp/zx2026_dry
  ROOT=/tmp/zx2026_dry
  cp "$ROOT"/src/zx2026_common/config/settings_real.yaml \
     "$ROOT"/src/zx2026_common/config/sim_settings.yaml
  find "$ROOT" -maxdepth 2 -type d | sort | sed 's/^/   /'
  echo "== dry OK：布局符合预期，去掉 dry 即真导送 =="
  exit 0
fi

echo "== [2/5] scp -> $TARGET:/tmp =="
# 时钟体检先行（实弹教训 2026-09-26：无 RTC/NTP 的 Jetson 时钟停在 epoch，
# tar 报"时间戳在未来"数十亿秒——代码无损，但日志时间线全 1970 无法取证）
L_EPOCH=$(date +%s)
R_EPOCH=$(ssh -o BatchMode=yes -o ConnectTimeout=5 \
    -o StrictHostKeyChecking=accept-new "$TARGET" "date +%s" 2>/dev/null || echo "")
if [ -n "$R_EPOCH" ]; then
  SKEW=$(( L_EPOCH - R_EPOCH ))
  if [ "${SKEW#-}" -gt 300 ]; then
    echo "   ⚠ 机载时钟偏 ${SKEW}s（未来文件告警根因）——一键对时："
    echo "     ssh $TARGET 'sudo date -s @$L_EPOCH'"
    echo "     （免密 sudo 不通则机载本地执行 sudo date -s @$L_EPOCH）"
  else
    echo "   时钟差 ${SKEW}s（正常）"
  fi
fi
ssh -o BatchMode=yes -o ConnectTimeout=5 \
    -o StrictHostKeyChecking=accept-new "$TARGET" "echo   链路 OK: \$(hostname)"
scp -o BatchMode=yes -o ConnectTimeout=5 \
    -o StrictHostKeyChecking=accept-new "$TGZ" "$TARGET:/tmp/"

echo "== [3/5] 机载解包 + 配置换名 =="
ssh -o BatchMode=yes -o ConnectTimeout=5 \
    -o StrictHostKeyChecking=accept-new "$TARGET" bash -s <<'REMOTE'
set -euo pipefail
mkdir -p ~/zx2026_ws
tar xzf /tmp/zx2026_deploy.tgz -C ~/zx2026_ws
cd ~/zx2026_ws
cp -f src/zx2026_common/config/settings_real.yaml \
      src/zx2026_common/config/sim_settings.yaml
# 真场地换名覆盖（Scene/rules 均硬编码文件名；WSL sim 的杨树林文件不上机）
cp -f tools/gcs/scene_topology_real.yaml \
      src/zx2026_common/config/scene_topology.yaml
cp -f tools/gcs/competition_rules_real.yaml \
      src/zx2026_common/config/competition_rules.yaml
chmod +x src/*/scripts/*.py tools/gcs/*.py tools/gcs/*.sh 2>/dev/null || true
echo "   解包 OK: $(ls src) / sim_settings+scene_topology+rules 换名就位"
REMOTE

if $NO_BUILD; then
  echo "== [4/5] 跳过构建（--no-build）=="
else
  echo "== [4/5] 机载 catkin build（msgs 生成；官方栈为 underlay）=="
  ssh -o BatchMode=yes -o ConnectTimeout=5 \
      -o StrictHostKeyChecking=accept-new "$TARGET" bash -s <<'REMOTE'
# 勿加 -u：ROS profile.d/1.ros_distro.sh 引用未绑定的 ROS_DISTRO，set -u 直接炸
# （2026-09-26 实弹：build 块 source underlay 即崩，catkin build 未跑到）
set -eo pipefail
cd ~/zx2026_ws
# 先 source 官方栈（underlay：quadrotor_msgs/mavros_msgs 从这里来）再构建，
# devel/setup.bash 自动链 underlay，运行期 vel_bridge import 无忧
if [ -f ~/Diff-planner/devel/setup.bash ]; then
  . ~/Diff-planner/devel/setup.bash
elif [ -f ~/Diff-Planner/devel/setup.bash ]; then
  . ~/Diff-Planner/devel/setup.bash   # 官方文档大写 P 版本兜底
fi
. /opt/ros/noetic/setup.bash
if command -v catkin >/dev/null 2>&1; then
  catkin build zx2026_common arena_bridges --no-status
else
  catkin_make --pkg zx2026_common --pkg arena_bridges
fi
REMOTE
fi

echo "== [5/5] 探针自检（msgs 生成/配置挂点/包发现/underlay 链）=="
ssh -o BatchMode=yes -o ConnectTimeout=5 \
    -o StrictHostKeyChecking=accept-new "$TARGET" bash -s <<'REMOTE'
# 勿加 -u：ROS profile.d/1.ros_distro.sh 引用未绑定的 ROS_DISTRO（与 build 块同坑）
set -o pipefail
if [ -f ~/Diff-planner/devel/setup.bash ]; then
  . ~/Diff-planner/devel/setup.bash
elif [ -f ~/Diff-Planner/devel/setup.bash ]; then
  . ~/Diff-Planner/devel/setup.bash
fi
. /opt/ros/noetic/setup.bash 2>/dev/null || true
. ~/zx2026_ws/devel/setup.bash
FAIL=0
python3 -c "from zx2026_common.msg import Mission, TaskUpdate" \
  && echo "   [ok] zx2026_common.msg 可导入" || FAIL=1
python3 -c "from quadrotor_msgs.msg import PositionCommand" \
  && echo "   [ok] quadrotor_msgs（underlay 链）可导入" || FAIL=1
python3 - <<'PY' && echo "   [ok] config 挂点=src 源树" || FAIL=1
import zx2026_common.config as c
d = c.pkg_config_dir()
assert d.endswith("src/zx2026_common/config"), d
import yaml, os
s = yaml.safe_load(open(os.path.join(d, "sim_settings.yaml")))
assert s["use_sim_time"] is False and s["closed_loop"]["drift_rate"] == 0.0 \
    and s["comms"]["enabled"] is True, "sim_settings.yaml 不是真机版！"
PY
python3 - <<'PY' && echo "   [ok] 真场地 scene/rules 挂点" || FAIL=1
import yaml, os
import zx2026_common.config as c
d = c.pkg_config_dir()
sc = yaml.safe_load(open(os.path.join(d, "scene_topology.yaml")))
dps = [list(dp["xyz"]) for dp in sc["drop_points"]]
assert dps == [[50.0, 0.0, 1.5], [50.0, 2.0, 1.5], [50.0, 4.0, 1.5]], dps
assert not any(z.get("kind") == "forest" for z in sc["zones"]), "真机场景不得带树"
rl = yaml.safe_load(open(os.path.join(d, "competition_rules.yaml")))
h = rl["heights"]
assert h["drop_hover_z"] == 1.5 and h["identify_z"] == 1.9, h
assert rl["time_limit_s"] == 1500, rl["time_limit_s"]
assert rl["rule"]["geofence"][2] == [50.35, 50.35], rl["rule"]["geofence"]
PY
rospack find arena_bridges >/dev/null 2>&1 \
  && echo "   [ok] rospack 发现 arena_bridges" || FAIL=1
ls ~/zx2026_ws/tools/gcs/profile_real_lio.yaml ~/zx2026_ws/tools/gcs/missions.yaml >/dev/null \
  && echo "   [ok] profile/missions 就位" || FAIL=1
[ "$FAIL" = 0 ] && echo "== DEPLOY OK ==" || { echo "== DEPLOY 有 FAIL 项，勿起飞 =="; exit 1; }
REMOTE

echo "
下一步（机载）：
  1. source ~/zx2026_ws/devel/setup.bash   # 自链官方 underlay
  2. 复核odom源: rostopic list | grep ekf   # 须见 /ekf/ekf_odom
  3. 起官方栈: sh_files 里 mavros+LIO 照旧
  4. 起注入栈: roslaunch arena_bridges real_fleet.launch \\
       id:=<本机号> yaw0_deg:=<标定值> server:=<地面站IP>:9870 \\
       stage_local:=true   # 仅1号机
  5. 拨杆铁律: 启动前 5/6/8 通最下 7 通最上 油门中位
地面站:
  python3 tools/gcs/gcs_hub.py --ids 0,1,2,3,4,5 --fleet-hz 5 --bat-min 30 --view dash"
