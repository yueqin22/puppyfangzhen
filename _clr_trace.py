import json, math, csv, collections
d=json.load(open('config/scene_home.json',encoding='utf-8'))
obs=[(o['name'],o['xmin'],o['ymin'],o['xmax'],o['ymax']) for o in d['obstacles']]
def near(px,py):
    best=(9e9,'')
    for n,x0,y0,x1,y1 in obs:
        dx=max(x0-px,0.0,px-x1); dy=max(y0-py,0.0,py-y1)
        e=math.hypot(dx,dy)
        if e<best[0]: best=(e,n)
    return best
def chebyIn(px,py,R):
    for n,x0,y0,x1,y1 in obs:
        if x0-R<=px<=x1+R and y0-R<=py<=y1+R: return n
    return None
rows=list(csv.DictReader(open('trace_val.csv')))
clr=[]; sq=[]
for r in rows:
    x=float(r['true_x']); y=float(r['true_y'])
    e,n=near(x,y); clr.append((e,n,int(r['frame']),x,y))
    if chebyIn(x,y,0.30): sq.append((int(r['frame']),x,y,chebyIn(x,y,0.30),e))
clr.sort()
print('frames=%d'%len(rows))
print('\n--- true Euclidean clearance (disc model) ---')
for lo,hi in ((0,.25),(.25,.28),(.28,.30),(.30,.32),(.32,.35),(.35,.40),(.40,9)):
    c=sum(1 for e,_,_,_,_ in clr if lo<=e<hi)
    print('  [%.2f,%.2f) : %5d  %5.1f%%'%(lo,hi,c,100*c/len(rows)))
print('  min=%.3f  p1=%.3f  p5=%.3f  median=%.3f'%(clr[0][0],clr[len(clr)//100][0],clr[len(clr)//20][0],clr[len(clr)//2][0]))
print('\n--- 20 tightest frames ---')
for e,n,f,x,y in clr[:20]: print('  f=%4d (%6.3f,%6.3f) clear=%.3f  near=%s'%(f,x,y,e,n))
print('\n--- frames failing SQUARE-inflated test @0.30 (what UE currently flags) ---')
print('  count=%d'%len(sq))
agg=collections.Counter(s[3] for s in sq)
print('  by obstacle:',dict(agg))
for f,x,y,n,e in sq[:20]: print('    f=%4d (%6.3f,%6.3f) obs=%-14s trueEuclid=%.3f'%(f,x,y,n,e))
print('\n--- frames failing DISC test @0.30 (what UE SHOULD flag) ---')
disc=[(f,x,y,n,e) for e,n,f,x,y in clr if e<0.30]
print('  count=%d'%len(disc))
for f,x,y,n,e in sorted(disc)[:20]: print('    f=%4d (%6.3f,%6.3f) obs=%-14s clear=%.3f'%(f,x,y,n,e))
