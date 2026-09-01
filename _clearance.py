import json, math, heapq
d=json.load(open('config/scene_home.json',encoding='utf-8'))
obs=[(o['name'],o['xmin'],o['ymin'],o['xmax'],o['ymax']) for o in d['obstacles']]
def dist_box(px,py,b):
    _,x0,y0,x1,y1=b
    dx=max(x0-px,0.0,px-x1); dy=max(y0-py,0.0,py-y1)
    return math.hypot(dx,dy)
def clear(px,py):
    return min(dist_box(px,py,b) for b in obs)
res=0.02
xs=[-4.5+res*i for i in range(int(9.0/res)+1)]
ys=[-3.5+res*j for j in range(int(7.0/res)+1)]
W,H=len(xs),len(ys)
C=[[clear(xs[i],ys[j]) for j in range(H)] for i in range(W)]
def idx(x,y): return (min(range(W),key=lambda i:abs(xs[i]-x)), min(range(H),key=lambda j:abs(ys[j]-y)))
# widest path (maximize bottleneck clearance) via max-min Dijkstra
def widest(s,t):
    si,sj=idx(*s); ti,tj=idx(*t)
    best=[[-1.0]*H for _ in range(W)]
    pq=[(-C[si][sj],si,sj)]; best[si][sj]=C[si][sj]
    while pq:
        nb,i,j=heapq.heappop(pq); b=-nb
        if b<best[i][j]-1e-12: continue
        if (i,j)==(ti,tj): return b
        for di,dj in ((1,0),(-1,0),(0,1),(0,-1)):
            ni,nj=i+di,j+dj
            if not(0<=ni<W and 0<=nj<H): continue
            nbot=min(b,C[ni][nj])
            if nbot>best[ni][nj]+1e-12:
                best[ni][nj]=nbot; heapq.heappush(pq,(-nbot,ni,nj))
    return -1.0
pts=d['patrol_targets']
print('clearance at each patrol target:')
for p in pts: print('  %-12s (%5.2f,%5.2f) clear=%.3f'%(p['name'],p['x'],p['y'],clear(p['x'],p['y'])))
print('\nwidest-path bottleneck between consecutive targets (loop):')
mn=9
for k in range(len(pts)):
    a=pts[k]; b=pts[(k+1)%len(pts)]
    w=widest((a['x'],a['y']),(b['x'],b['y']))
    mn=min(mn,w)
    print('  %-12s -> %-12s bottleneck=%.3f m'%(a['name'],b['name'],w))
print('\nGLOBAL LOOP BOTTLENECK = %.3f m  -> max usable inflation radius'%mn)
