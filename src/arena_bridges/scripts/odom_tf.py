#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""roslaunch shim -> tools/gcs/odom_tf.py（单一源纪律：实现在 GCS 工具目录，
本包 scripts/ 只做 roslaunch 可发现性包装；改逻辑去 tools/gcs/，勿在此堆码）。"""
import os
import runpy
import sys

_d = os.environ.get("ZX2026_TOOLS_DIR")
if not _d:
    # 相对回退：<ws>/src/arena_bridges/scripts -> <ws>/tools/gcs（机上
    # ~/zx2026_ws 与 WSL ~/zx2026_arena_ws 双布局通吃，不依赖环境变量/软链）
    _c = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))), "tools", "gcs")
    _d = _c if os.path.isdir(_c) else os.path.expanduser(
        "~/zx2026_arena_ws/tools/gcs")
sys.path.insert(0, _d)   # 实现内 import odom_tf（cloud_adapter）同目录解析
# 剥 roslaunch 注入参数（__name:=/__log:=）——本体 argparse 不识别会炸
# （首飞实证 2026-09-26：五 shim 全灭于此，sim 手跑故从未暴露）
sys.argv = [sys.argv[0]] + [a for a in sys.argv[1:] if not a.startswith("__")]
runpy.run_path(os.path.join(_d, "odom_tf.py"), run_name="__main__")
