import json, math
from collections import deque

S = json.load(open('config/scene_home.json', encoding='utf-8'))
g = S['grid']; res = g['resolution']; W = g['width']; H = g['height']
ox = g['origin_x']; oy = g['origin_y']
print(f"grid {W}x{H} res={res} origin=({ox},{oy})")

obs = S['obstacles']
R = S.get('robot',{}).get('radius_planning', 0.3)
rc = math.ceil(R/res)
print(f"radius_planning={R} -> radius_cells={rc}  ({len(obs)} obstacles)")

occ = [[0]*W for _ in range(H)]
for o in obs:
    x0,x1 = o['xmin'], o['xmax']; y0,y1 = o['ymin'], o['ymax']
    i0 = max(0, int((x0-ox)/res)); i1 = min(W-1, int((x1-ox)/res))
    j0 = max(0, int((y0-oy)/res)); j1 = min(H-1, int((y1-oy)/res))
    for j in range(j0, j1+1):
        for i in range(i0, i1+1):
            occ[j][i] = 1

inf = [[0]*W for _ in range(H)]
for j in range(H):
    for i in range(W):
        if occ[j][i]:
            for dj in range(-rc, rc+1):
                for di in range(-rc, rc+1):
                    jj,ii = j+dj, i+di
                    if 0<=jj<H and 0<=ii<W: inf[jj][ii]=1

def w2g(x,y): return int((x-ox)/res), int((y-oy)/res)

pts = S['patrol_targets']
print("patrol_targets sample:", pts[0])
names=[]; cells=[]
for p in pts:
    n = p.get('name','?'); x=p['x']; y=p['y']
    i,j = w2g(x,y)
    b_raw = occ[j][i] if (0<=j<H and 0<=i<W) else 2
    b_inf = inf[j][i] if (0<=j<H and 0<=i<W) else 2
    names.append(n); cells.append((i,j))
    print(f"  {n:14s} ({x:+.2f},{y:+.2f}) cell=({i},{j}) raw={'OBS' if b_raw else 'free'} inflated={'BLOCKED' if b_inf else 'free'}")

def bfs(src):
    si,sj = src
    seen=[[False]*W for _ in range(H)]
    if not (0<=si<W and 0<=sj<H) or inf[sj][si]: return seen, False
    q=deque([(si,sj)]); seen[sj][si]=True
    while q:
        i,j=q.popleft()
        for di,dj in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            ii,jj=i+di,j+dj
            if 0<=ii<W and 0<=jj<H and not seen[jj][ii] and not inf[jj][ii]:
                seen[jj][ii]=True; q.append((ii,jj))
    return seen, True

print("\n=== connectivity (inflated %dcm) ===" % int(R*100))
for k,(n,c) in enumerate(zip(names,cells)):
    seen, ok = bfs(c)
    if not ok:
        print(f"  {n}: GOAL CELL BLOCKED by inflation -> planner must snap start/goal")
        continue
    unreach=[names[m] for m in range(len(cells)) if m!=k and not seen[cells[m][1]][cells[m][0]]]
    print(f"  {n}: UNREACHABLE={unreach if unreach else 'none (all reachable)'}")

# also: for each blocked goal, distance to nearest free cell
print("\n=== nearest free-cell snap distance for blocked goals ===")
free_cells=[(i,j) for j in range(H) for i in range(W) if not inf[j][i]]
for n,c in zip(names,cells):
    i,j=c
    if 0<=i<W and 0<=j<H and inf[j][i]:
        best=min(((abs(i-fi)+abs(j-fj), fi,fj) for fi,fj in free_cells))
        d=math.hypot(best[1]-i, best[2]-j)*res
        print(f"  {n}: nearest free cell {d:.2f}m away")
