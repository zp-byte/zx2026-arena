#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""roslaunch shim -> tools/gcs/mission_loader.py（单一源纪律同包 README）。"""
import os
import runpy
import sys

_d = os.environ.get("ZX2026_TOOLS_DIR")
if not _d:
    # 相对回退：<ws>/src/arena_bridges/scripts -> <ws>/tools/gcs（双布局通吃）
    _c = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))), "tools", "gcs")
    _d = _c if os.path.isdir(_c) else os.path.expanduser(
        "~/zx2026_arena_ws/tools/gcs")
sys.path.insert(0, _d)
# 剥 roslaunch 注入参数（__name:=/__log:=）——本体 argparse 不识别会炸
# （首飞实证 2026-09-26：五 shim 全灭于此，sim 手跑故从未暴露）
sys.argv = [sys.argv[0]] + [a for a in sys.argv[1:] if not a.startswith("__")]
runpy.run_path(os.path.join(_d, "mission_loader.py"), run_name="__main__")
