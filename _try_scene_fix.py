import json, math, copy
from collections import deque

BASE = json.load(open('config/scene_home.json', encoding='utf-8'))

def build_and_check(S, label, R=None):
    g=S['grid']; res=g['resolution']; W=g['width']; H=g['height']
    ox=g['origin_x']; oy=g['origin_y']
    if R is None: R=S['robot']['radius_planning']
    rc=math.ceil(R/res)
    occ=[[0]*W for _ in range(H)]
    for o in S['obstacles']:
        i0=max(0,int((o['xmin']-ox)/res)); i1=min(W-1,int((o['xmax']-ox)/res))
        j0=max(0,int((o['ymin']-oy)/res)); j1=min(H-1,int((o['ymax']-oy)/res))
        for j in range(j0,j1+1):
            for i in range(i0,i1+1): occ[j][i]=1
    inf=[[0]*W for _ in range(H)]
    for j in range(H):
        for i in range(W):
            if occ[j][i]:
                for dj in range(-rc,rc+1):
                    for di in range(-rc,rc+1):
                        jj,ii=j+dj,i+di
                        if 0<=jj<H and 0<=ii<W: inf[jj][ii]=1
    def w2g(x,y): return int((x-ox)/res), int((y-oy)/res)
    pts=S['patrol_targets']
    cells=[w2g(p['x'],p['y']) for p in pts]; names=[p['name'] for p in pts]
    # BFS from dock
    si,sj=cells[0]
    seen=[[False]*W for _ in range(H)]
    okstart = not inf[sj][si]
    if okstart:
        q=deque([(si,sj)]); seen[sj][si]=True
        while q:
            i,j=q.popleft()
            for di,dj in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                ii,jj=i+di,j+dj
                if 0<=ii<W and 0<=jj<H and not seen[jj][ii] and not inf[jj][ii]:
                    seen[jj][ii]=True; q.append((ii,jj))
    bad=[]
    for n,(i,j) in zip(names,cells):
        if inf[j][i]: bad.append(n+"(GOAL-BLOCKED)")
        elif not seen[j][i]: bad.append(n+"(UNREACH)")
    print(f"[{label}] R={R}m rc={rc}  problems={bad if bad else 'NONE - all 5 targets reachable from dock'}")
    return not bad

print("=== baseline ===")
build_and_check(BASE, "baseline")

# Candidate fix: widen bathroom doorway + move sink to north wall
S2 = copy.deepcopy(BASE)
for o in S2['obstacles']:
    if o['name']=='wall_h_mid':      o['xmax']=1.50   # was 1.80 -> pull left 0.30
    if o['name']=='wall_h_bath_r':   o['xmin']=3.20   # was 2.80 -> push right 0.40
    if o['name']=='sink':
        o.update(xmin=1.60, xmax=2.40, ymin=2.90, ymax=3.30)  # move to north wall
print("\n=== candidate fix (widen door 1.0m->1.7m, sink -> north wall) ===")
ok = build_and_check(S2, "fix")
# stress: check with larger inflation to confirm margin
for r in (0.30, 0.35, 0.40):
    build_and_check(S2, f"fix @R={r}", R=r)

if ok:
    json.dump(S2, open('config/scene_home_candidate.json','w',encoding='utf-8'), ensure_ascii=False, indent=2)
    print("\nwrote config/scene_home_candidate.json")
