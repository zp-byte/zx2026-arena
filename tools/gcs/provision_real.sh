#!/bin/bash
# tools/gcs/provision_real.sh — 真机联调日前置 TODO①② 机械化（2026-09-18）。
#
#   ① IP 网段：六机 nv@192.168.1.101-106 + 地面站 .10（profile 实注网段），
#      机侧静态 IP / DHCP 静态绑定在路由器或 Jetson netplan 做——本脚本负责
#      验证连通并回填 profile 的 server 字段。
#   ② ssh 公钥：地面站公钥分发到六机（ssh-copy-id），BatchMode 复验。
#
# 用法（地面站笔记本）：
#   bash provision_real.sh                # 全六机
#   IDS="101 103" bash provision_real.sh  # 指定机
#   bash provision_real.sh --server 192.168.1.55   # 顺带回填 profile server
# 纪律（profile 实注）：勿用 sshpass——免密失败即报错人工介入。
set -u
SSH_OPTS="-o BatchMode=no -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new"
IDS="${IDS:-101 102 103 104 105 106}"
SERVER=""

while [ $# -gt 0 ]; do
  case "$1" in
    --server) SERVER="$2"; shift 2 ;;
    *) echo "未知参数: $1" >&2; exit 2 ;;
  esac
done

# 1) 本机公钥（无则生成 ed25519）
if [ ! -f ~/.ssh/id_ed25519.pub ]; then
  echo "[key] 无公钥，生成 ~/.ssh/id_ed25519 ..."
  ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519
fi

# 2) 逐机：copy-id → BatchMode 免密复验 → 网段/目录探针
fail=0
for last in $IDS; do
  host="192.168.1.$last"
  echo "== $host =="
  if ! ping -c1 -W2 "$host" >/dev/null 2>&1; then
    echo "   [FAIL] ping 不通——查机侧静态 IP/同网段"; fail=1; continue
  fi
  ssh-copy-id $SSH_OPTS "nv@$host" || { echo "   [FAIL] ssh-copy-id"; fail=1; continue; }
  if ssh -o BatchMode=yes -o ConnectTimeout=5 "nv@$host" \
      'echo "   [ok] 免密: $(hostname)"; ls -d ~/Diff-planner >/dev/null && echo "   [ok] ~/Diff-planner 在位"' 2>/dev/null; then
    : # 探针输出即证据
  else
    echo "   [FAIL] BatchMode 免密复验（copy-id 后仍要密码= authorized_keys 权限，机侧 chmod 700 ~ ~/.ssh / 600 authorized_keys）"
    fail=1
  fi
done

# 3) profile server 字段回填（联调日地面站实际 IP 定谳后）
if [ -n "$SERVER" ]; then
  dir="$(cd "$(dirname "$0")" && pwd)"
  sed -i "s|^server: .*|server: $SERVER:9870                       # provision_real.sh 回填 $(date +%F)|" \
    "$dir/profile_real_lio.yaml" "$dir/profile_real_vio.yaml" 2>/dev/null \
    && echo "[server] profile_*.yaml → $SERVER:9870"
fi

exit $fail
