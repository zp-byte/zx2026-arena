#!/usr/bin/env python3
# 诊断：drone 1 卡住点 (-12.1, 6.7) 附近的障碍，及起降区到穿越区走廊的树
from zx2026_common.scene import Scene

s = Scene()
px, py = -12.1, 6.7
print("venue:", s.venue)
print("drone_radius:", s.drone_radius)

print("--- obstacles near (-12.1, 6.7) r<4 ---")
for ob in s.obstacles:
    if hasattr(ob, "cx"):
        d = ((ob.cx - px) ** 2 + (ob.cy - py) ** 2) ** 0.5
    else:
        cx = (ob.lo[0] + ob.hi[0]) / 2
        cy = (ob.lo[1] + ob.hi[1]) / 2
        d = ((cx - px) ** 2 + (cy - py) ** 2) ** 0.5
    if d < 4:
        kind = getattr(ob, "kind", "?")
        tr = getattr(ob, "trunk_r", None)
        cr = getattr(ob, "crown_r", None)
        print("  kind=%s d=%.2f trunk_r=%s crown_r=%s lo=%s hi=%s" % (
            kind, d, tr, cr, getattr(ob, "lo", None), getattr(ob, "hi", None)))

print("--- trees in corridor x[-25,-10] y[4,10] (drone1 route) ---")
for ob in s.obstacles:
    if hasattr(ob, "cx") and -25 <= ob.cx <= -10 and 4 <= ob.cy <= 10:
        print("  tree cx=%.2f cy=%.2f trunk_r=%.2f crown_r=%.2f" % (
            ob.cx, ob.cy, ob.trunk_r, ob.crown_r))
