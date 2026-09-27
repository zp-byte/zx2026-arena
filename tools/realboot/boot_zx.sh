#!/bin/bash
# =============================================================================
# boot_zx.sh — 笔记本侧一键装+起（WSL 里跑；本文件即源码，跑它=装到机上并拉起）
# 用法：bash tools/realboot/boot_zx.sh          （WSL 任意目录，SRC 自动取本文件所在目录）
# 动作：烘 epoch → scp 三件到机 ~/boot/ → 跑 boot_all（对时+传感器链 9 检，
#       不碰飞控）→ 拉起 d435_watchdog
# 前提：机上 nv 免密 ssh；机上一次性 dialout 组已修（2026-09-27 定谳已修）
# =============================================================================
set -e
D=nv@192.168.31.76
SRC=${SRC:-$(cd "$(dirname "$0")" && pwd)}
E=$(date +%s)
BAKED=/tmp/boot_all_baked.zsh
sed "s/__BOOT_EPOCH__/$E/" $SRC/boot_all.zsh > $BAKED
ssh $D 'mkdir -p ~/boot'
scp -q $BAKED $D:boot/boot_all.zsh
scp -q $SRC/d435_watchdog.zsh $D:boot/d435_watchdog.zsh
ssh $D 'chmod +x ~/boot/*.zsh; pkill -f "d435_watchdo[g]" 2>/dev/null; true'
BOOT_RC=0
ssh $D "zsh ~/boot/boot_all.zsh $E" || BOOT_RC=$?
ssh $D 'nohup zsh ~/boot/d435_watchdog.zsh > /dev/null 2>&1 & sleep 1; pgrep -f "d435_watchdo[g]" >/dev/null && echo WATCHDOG_UP || echo WATCHDOG_DOWN'
echo "BOOT_RC=$BOOT_RC"
