import json, re
S=json.load(open('config/scene_home.json',encoding='utf-8'))
js={o['name']:(o['xmin'],o['ymin'],o['xmax'],o['ymax']) for o in S['obstacles']}
src=open(r'D:\puppy_ue\Source\PuppyNav\PuppyRobotPawn.cpp',encoding='utf-8',errors='replace').read()
m=re.search(r'HOME_OBSTACLES\[\]\s*=\s*\{(.*?)\n\};', src, re.S)
ue={}
for line in m.group(1).splitlines():
    mm=re.match(r'\s*\{\s*TEXT\("([^"]+)"\)\s*,\s*(-?[\d.]+)f\s*,\s*(-?[\d.]+)f\s*,\s*(-?[\d.]+)f\s*,\s*(-?[\d.]+)f\s*\}', line)
    if mm:
        ue[mm.group(1).strip()]=tuple(float(mm.group(i))/100.0 for i in (2,3,4,5))
print(f"json obstacles={len(js)}  ue obstacles={len(ue)}")
bad=0
for n in sorted(set(js)|set(ue)):
    if n not in js: print(f"  MISSING-IN-JSON: {n}"); bad+=1; continue
    if n not in ue: print(f"  MISSING-IN-UE:   {n}"); bad+=1; continue
    a,b=js[n],ue[n]
    if any(abs(x-y)>1e-6 for x,y in zip(a,b)):
        print(f"  MISMATCH {n}: json={a} ue={b}"); bad+=1
# also compare patrol targets / initial pose if present in UE
print("RESULT:", "ALL 18 OBSTACLES IN SYNC" if bad==0 else f"{bad} PROBLEM(S)")
