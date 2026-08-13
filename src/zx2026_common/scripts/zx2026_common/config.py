# -*- coding: utf-8 -*-
"""YAML 配置加载器。

所有 YAML 位于 zx2026_common/config/。任何 arena 子包通过本模块加载，
并支持命令行 --param 覆盖（key.path=value），保证不改文件即可做参数网格。
"""
import os
import sys
import yaml


def pkg_config_dir():
    """返回 zx2026_common 包内 config 目录绝对路径。"""
    here = os.path.dirname(os.path.abspath(__file__))          # .../scripts/zx2026_common
    return os.path.normpath(os.path.join(here, os.pardir, os.pardir, "config"))


_OVERRIDES = None  # 全局命令行覆盖 {dotpath: str}


def _load_overrides():
    global _OVERRIDES
    if _OVERRIDES is not None:
        return _OVERRIDES
    _OVERRIDES = {}
    for arg in sys.argv[1:]:
        if "=" in arg and not arg.startswith("__") and "--" not in arg:
            key, val = arg.split("=", 1)
            _OVERRIDES[key.strip()] = val.strip()
    return _OVERRIDES


def _apply(d, path_parts, value):
    """把 value 按类型写入 dict 路径 path_parts（递归）。"""
    key = path_parts[0]
    if len(path_parts) == 1:
        d[key] = _coerce(value)
        return
    if key not in d or not isinstance(d[key], dict):
        d[key] = {}
    _apply(d[key], path_parts[1:], value)


def _coerce(s):
    try:
        if s.lower() in ("true", "false"):
            return s.lower() == "true"
        return int(s)
    except ValueError:
        try:
            return float(s)
        except ValueError:
            return s


def load(filename):
    """加载 config/<filename>.yaml，应用命令行覆盖，返回 dict。"""
    path = os.path.join(pkg_config_dir(), filename)
    if not os.path.exists(path):
        # 允许调用方传入绝对路径
        if os.path.exists(filename):
            path = filename
        else:
            raise FileNotFoundError("config not found: %s" % path)
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    for key, val in _load_overrides().items():
        if not key.startswith(filename.split(".")[0] + "."):
            continue
        path_parts = key.split(".")
        _apply(data, path_parts[1:], val)
    return data


# ---- 类型映射 ---------------------------------------------------------------
TYPE_TO_UINT8 = {
    "TYPE_A": 0, "TYPE_B": 1, "TYPE_C": 2,
    "TYPE_D": 3, "TYPE_E": 4, "UNKNOWN": 255,
}
UINT8_TO_TYPE = {v: k for k, v in TYPE_TO_UINT8.items()}


def type_to_uint8(t):
    return TYPE_TO_UINT8.get(t, 255)


def uint8_to_type(u):
    return UINT8_TO_TYPE.get(u, "UNKNOWN")
