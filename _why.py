import json,math,csv
d=json.load(open('config/scene_home.json',encoding='utf-8'))
obs=[(o['name'],o['xmin'],o['ymin'],o['xmax'],o['ymax']) for o in d['obstacles']]
def near(px,py):
    b=(9e9,'')
    for n,x0,y0,x1,y1 in obs:
        dx=max(x0-px,0.0,px-x1); dy=max(y0-py,0.0,py-y1)
        e=math.hypot(dx,dy)
        if e<b[0]: b=(e,n)
    return b
rows=list(csv.DictReader(open('trace_val.csv')))
cols=['frame','true_x','true_y','est_x','est_y','err','cmd_vx','cmd_vy','la_x','la_y','clearance','r31_bound','r31_dmin','r31_scale','r31_dev','path_pts','escape']
def show(a,b,tag):
    print('\n===== %s ====='%tag)
    print('  frm  true_x true_y  est_x  est_y   err  cmd_vx cmd_vy   la_x   la_y  clrCol r31b r31dmin r31sc r31dev npts esc | TRUEclr near')
    for r in rows:
        f=int(r['frame'])
        if not(a<=f<=b): continue
        e,n=near(float(r['true_x']),float(r['true_y']))
        print('  %4d %7.3f%7.3f%7.3f%7.3f%6.3f %7.3f%7.3f%7.3f%7.3f %7.3f %4s %7s %5s %6s %4s %3s | %.3f %s'%(
            f,float(r['true_x']),float(r['true_y']),float(r['est_x']),float(r['est_y']),float(r['err']),
            float(r['cmd_vx']),float(r['cmd_vy']),float(r['la_x']),float(r['la_y']),float(r['clearance']),
            r['r31_bound'],r['r31_dmin'],r['r31_scale'],r['r31_dev'],r['path_pts'],r['escape'],e,n))
show(1844,1856,'wall_h_mid corner  (UE COLLIDE #1, min clearance 0.266)')
show(384,398,'wall_h_bed_l corner (0.270, NOT counted by UE)')
show(1456,1466,'coffee_table (0.272, NOT counted by UE)')
