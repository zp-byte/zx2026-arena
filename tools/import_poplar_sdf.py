#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""forest_poplar.sdf → forest_poplar_table.yaml 一次性转换器。

解析杨树林 Gazebo 世界（D:/gazebo_forest/forest_poplar.sdf，204 棵杨树），
提取每棵树的位姿+碰撞几何（树干圆柱/冠包络圆柱）+视觉 mesh 元数据（变体/
缩放/朝向），输出 Scene table 生成器消费的显式树表。同时自动挑选 3 个投放
平台位（东林缘 x≈27 一列，距最近树干表面 ≥1.6m 的无树点）。

用法: python3 tools/import_poplar_sdf.py [sdf路径] [输出路径]
"""
import os
import sys
import xml.etree.ElementTree as ET

import yaml

SDF = sys.argv[1] if len(sys.argv) > 1 else "/mnt/d/gazebo_forest/forest_poplar.sdf"
OUT = (sys.argv[2] if len(sys.argv) > 2 else
       "/home/ubuntu/zx2026_arena_ws/src/zx2026_common/config/forest_poplar_table.yaml")

DROP_CLEAR = 1.6          # 平台位要求的最小树干表面净空 (m)
DROP_TARGETS = [(27.0, -9.0), (27.0, 0.0), (27.0, 9.0)]   # 东林缘一列


def parse_cylinder(col):
    g = col.find("geometry/cylinder")
    p = [float(v) for v in col.find("pose").text.split()]
    return (float(g.find("radius").text), float(g.find("length").text), p[2])


def main():
    root = ET.parse(SDF).getroot()
    trees = []
    for model in root.iter("model"):
        name = model.get("name", "")
        if not name.startswith("tree_"):
            continue
        pose = [float(v) for v in model.find("pose").text.split()]
        cols = {c.get("name"): c for c in model.iter("collision")}
        tr, th, _ = parse_cylinder(cols["c_trunk"])
        cr, ch, cz = parse_cylinder(cols["c_crown"])
        wood = model.find(".//visual[@name='v_wood']/geometry/mesh")
        uri = wood.find("uri").text                 # .../tree_{vi}_wood.stl
        vi = int(uri.rsplit("tree_", 1)[1].split("_")[0])
        s = float(wood.find("scale").text.split()[0])
        trees.append(dict(x=round(pose[0], 3), y=round(pose[1], 3),
                          yaw=round(pose[5], 3), variant=vi, scale=round(s, 3),
                          trunk_r=round(tr, 4), trunk_h=round(th, 3),
                          crown_r=round(cr, 4), crown_z=round(cz, 3),
                          crown_h=round(ch, 3)))

    # ---- 统计与健康检查 -----------------------------------------------------
    xs = [t["x"] for t in trees]
    ys = [t["y"] for t in trees]
    crown_bot = [t["crown_z"] - t["crown_h"] / 2.0 for t in trees]
    print("trees=%d  x[%.2f,%.2f] y[%.2f,%.2f]" %
          (len(trees), min(xs), max(xs), min(ys), max(ys)))
    print("trunk_r [%.3f,%.3f]  trunk_h [%.2f,%.2f]  crown_bot_min=%.2f" %
          (min(t["trunk_r"] for t in trees), max(t["trunk_r"] for t in trees),
           min(t["trunk_h"] for t in trees), max(t["trunk_h"] for t in trees),
           min(crown_bot)))
    assert min(crown_bot) > 2.6, "冠下缘低于 2.6m：高度带 [0.7,2.6] 会被冠污染"
    west = [t for t in trees if t["x"] < -23.3]
    assert not west, "清理带 (x<-23.3) 内发现 %d 棵树" % len(west)

    def min_clear(px, py):
        best = 1e9
        for t in trees:
            d = ((px - t["x"]) ** 2 + (py - t["y"]) ** 2) ** 0.5 - t["trunk_r"]
            if d < best:
                best = d
        return best

    # ---- 投放平台位：目标附近 0.25m 栅格扫净空 ≥1.6 的最近点 -----------------
    drops = []
    for (tx, ty) in DROP_TARGETS:
        cands = []
        for ix in range(-8, 13):          # x ∈ tx-2 .. tx+3
            for iy in range(-20, 21):     # y ∈ ty-5 .. ty+5
                px, py = tx + ix * 0.25, ty + iy * 0.25
                if not (-29.0 < px < 29.0 and -29.0 < py < 29.0):
                    continue
                c = min_clear(px, py)
                if c >= DROP_CLEAR:
                    cands.append((round((px - tx) ** 2 + (py - ty) ** 2, 4),
                                  round(px, 2), round(py, 2), round(c, 2)))
        assert cands, "平台目标 (%.1f,%.1f) 附近找不到净空点" % (tx, ty)
        cands.sort()
        _, px, py, c = cands[0]
        drops.append(dict(x=px, y=py, clear=c))
        print("drop (%.1f,%.1f) -> (%.2f,%.2f) clear=%.2f" % (tx, ty, px, py, c))
    for i, d in enumerate(drops):        # 平台间互距核查
        for d2 in drops[i + 1:]:
            dd = ((d["x"] - d2["x"]) ** 2 + (d["y"] - d2["y"]) ** 2) ** 0.5
            print("drop pair dist=%.2f" % dd)

    # ---- 输出树表 ------------------------------------------------------------
    doc = dict(
        meta=dict(source=SDF, seed=42, area=[60.0, 60.0], clearing_w=6.0,
                  n_trees=len(trees),
                  drop_candidates=[[d["x"], d["y"]] for d in drops]),
        trees=trees)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=False,
                       default_flow_style=False)
    print("wrote %s (%d trees)" % (OUT, len(trees)))


if __name__ == "__main__":
    main()
