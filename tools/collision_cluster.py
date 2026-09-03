#!/usr/bin/env python3
# 全史碰撞聚类归因：事件 → 最近树 / 无树伴生 → 热点排名 + 形态分类
import re, sys, math, collections

EV = '/tmp/all_collisions.txt'
gz_re = re.compile(r'drone (\d+) COLLISION #\d+ at \(([-\d.]+),([-\d.]+),([-\d.]+)\) dir=\(([-\d.]+),([-\d.]+)')
py_re = re.compile(r'drone (\d+) collision marker at \(([-\d.]+),([-\d.]+)\) cell \((\d+),(\d+)\)')
ep_re = re.compile(r'\[INFO\] \[([\d.]+)')

events = []  # dict: kind, run, drone, x, y, dirx, diry, epoch
for line in open(EV, encoding='utf-8', errors='ignore'):
    parts = line.rstrip('\n').split('|', 2)
    if len(parts) < 3: continue
    kind, run, rest = parts
    m = gz_re.search(rest)
    if m and kind == 'gz':
        events.append(dict(kind='gz', run=run, drone=int(m.group(1)),
                           x=float(m.group(2)), y=float(m.group(3)),
                           dx=float(m.group(5)), dy=float(m.group(6)), epoch=None))
        continue
    m = py_re.search(rest)
    if m and kind == 'py':
        e = ep_re.search(rest)
        events.append(dict(kind='py', run=run, drone=int(m.group(1)),
                           x=float(m.group(2)), y=float(m.group(3)),
                           dx=None, dy=None,
                           epoch=float(e.group(1)) if e else None))

# python 事件按 (run,drone,0.5m格) 去重（同一碰撞多 marker）
seen = set(); dedup = []
for ev in events:
    if ev['kind'] == 'py':
        k = (ev['run'], ev['drone'], ev['x'], ev['y'])
        if k in seen: continue
        seen.add(k)
    dedup.append(ev)
events = dedup

sys.path.insert(0, '/home/ubuntu/zx2026_arena_ws/src/zx2026_common/scripts')
from zx2026_common.scene import Scene
sc = Scene()
trees = [(ob.id, ob.cx, ob.cy, ob.trunk_r) for ob in sc.obstacles if ob.kind == 'tree' and ob.cx is not None]
drops = [(d.id, d.xyz[0], d.xyz[1]) for d in sc.drop_points]

DR = sc.drone_radius
def nearest_tree(x, y):
    best = None; bd = 1e9
    for tid, tx, ty, tr in trees:
        d = math.hypot(x-tx, y-ty)
        if d < bd: bd = d; best = (tid, tx, ty, tr)
    return best, bd

nog = [e for e in events if e['kind']=='gz']
npy = [e for e in events if e['kind']=='py']
print(f'== 事件量: gazebo={len(nog)} (run数={len(set(e["run"] for e in nog))}), python去重后={len(npy)} (run数={len(set(e["run"] for e in npy))})')

tree_hits = collections.Counter(); tree_kind = collections.Counter()
loose = []
for ev in events:
    (tid, tx, ty, tr), d = nearest_tree(ev['x'], ev['y'])
    ev['tree'] = (tid, tx, ty, tr); ev['tdist'] = d
    if d < tr + DR + 0.55:
        tree_hits[(tid, round(tx,1), round(ty,1), round(tr,2))] += 1
        tree_kind[ev['kind']] += 1
    else:
        loose.append(ev)

print('== 树热点 TOP15（碰撞点距树干 < trunk_r+0.9）==')
for (tid, tx, ty, tr), c in tree_hits.most_common(15):
    # 该树命中的方向（gazebo 有 dir）
    dirs = [(e['dx'], e['dy']) for e in events if e.get('tree') and e['tree'][0]==tid and e['kind']=='gz']
    dstr = ' '.join(f'({dx:.2f},{dy:.2f})' for dx,dy in dirs[:6])
    print(f'  tree#{tid} ({tx},{ty}) trunk_r={tr} hits={c} gz_dir=[{dstr}]')

print(f'== 无树伴生事件: {len(loose)} ==')
lc = collections.Counter((round(e['x']*2)/2, round(e['y']*2)/2) for e in loose)
for (x, y), c in lc.most_common(20):
    ds = sorted(set(e['drone'] for e in loose if (round(e['x']*2)/2, round(e['y']*2)/2)==(x,y)))
    print(f'  ({x},{y}) x{c} drones={ds}')

print('== 按机统计 ==')
dc = collections.Counter(e['drone'] for e in events)
for d in sorted(dc): print(f'  drone{d}: {dc[d]}')

# 碰撞点相对 drop 点 / 回程段判定：x>0 且 y>0 近 drop 区?
print('== drop 点坐标 ==')
for did, x, y in drops:
    print(f'  {did}: ({x:.1f},{y:.1f})')

# 事件时间分布粗看（python 有 epoch 的按小时桶）
hc = collections.Counter()
for e in npy:
    if e['epoch']:
        hc[int(e['epoch']//3600)] += 1
print('== python 碰撞按小时分布（epoch//3600）==')
for h in sorted(hc):
    import time
    print(f'  {time.strftime("%m-%d %H:00", time.localtime(h*3600))}: {hc[h]}')
