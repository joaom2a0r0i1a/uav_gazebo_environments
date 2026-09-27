#!/usr/bin/env python3
# gen_multistory.py: 3-floor office, ALL colored primitives (exact analytic GT).
# Fixed pinwheel floor plan around an OPEN corridor ring: 7 rooms + stairwell per floor, EVERY room
# opens to the corridor with one UNIFORM door; open central atrium shaft; per-floor window bands (black
# glass); exterior entrance + canopy on the south wall; furniture/clutter. Emitted as SEPARATE per-floor
# models (building_shell / _L1 / _L2 / _L3 / _core / _roof) so floors can be hidden in the GUI.
import os, struct, random
WORLDS=os.path.dirname(os.path.abspath(__file__))
FW=os.path.join(WORLDS,'grass_plane_multistory.world')
# The ground truth cloud is consumed by the evaluation package, so it is written there by
# default and GT_OUT overrides it when this package is used on its own.
PLY=os.environ.get('GT_OUT', os.path.join(WORLDS,'..','..','UAV_3D_reconstruction','single',
                   'motion_planning','data','gt_multistory_processed.ply'))
X0,X1,Y0,Y1=-10.0,10.0,-8.0,8.0
FH=3.3; SLAB=0.3; WT=0.15; ROOFZ=3*FH; FLOORZ=[0.0,FH,2*FH]
ATR=(-2.5,2.5,-2.5,2.5); STR=(4.0,10.0,-8.0,-2.0); D=0.06; WBOT,WTOP=1.1,2.3
SHOLE=(5.2,9.6,-5.4,-3.6)            # stair-well opening in the L2/L3 floor slabs (rest of the shaft IS floored)
HX0,HX1,HY0,HY1=-4.0,4.0,-4.0,4.0   # corridor ring outer boundary (THIN 1.5 m ring around the atrium)
DW=1.1                               # UNIFORM interior door width
GREY=(205,205,205);DGREY=(95,95,95);WHITE=(240,240,240);WOOD=(150,100,55);GLASS=(95,190,180)
DKGRN=(60,120,70);CARP=[(95,115,145),(150,120,90),(110,140,120)]
boxes=[]; ALL=['+x','-x','+y','-y','+z','-z']; CURGRP='shell'
def add(n,cx,cy,cz,sx,sy,sz,gtf,m,c): boxes.append((n,cx,cy,cz,sx,sy,sz,gtf,m,c,CURGRP))
def inside(x,y,z): return X0-0.03<=x<=X1+0.03 and Y0-0.03<=y<=Y1+0.03 and -0.03<=z<=ROOFZ+0.03

def wall_run(tag,x0,y0,x1,y1,zb,h,t=WT,gtf=None,mat='Gazebo/White',rgb=WHITE,doors=()):
    horiz=abs(x1-x0)>abs(y1-y0); lo,hi=(min(x0,x1),max(x0,x1)) if horiz else (min(y0,y1),max(y0,y1))
    sf = gtf if gtf is not None else (['+y','-y'] if horiz else ['+x','-x'])
    cuts=sorted((p-w/2,p+w/2) for p,w in doors); segs=[]; cur=lo
    for c0,c1 in cuts:
        if c0>cur: segs.append((cur,c0))
        cur=max(cur,c1)
    if cur<hi: segs.append((cur,hi))
    for i,(s0,s1) in enumerate(segs):
        if horiz: add(f'{tag}_{i}',(s0+s1)/2,y0,zb+h/2,s1-s0,t,h,sf,mat,rgb)
        else:     add(f'{tag}_{i}',x0,(s0+s1)/2,zb+h/2,t,s1-s0,h,sf,mat,rgb)
    for j,(p,w) in enumerate(doors):
        if h-2.2<=0.02: continue
        if horiz: add(f'{tag}_L{j}',p,y0,zb+2.2+(h-2.2)/2,w,t,h-2.2,sf,mat,rgb)
        else:     add(f'{tag}_L{j}',x0,p,zb+2.2+(h-2.2)/2,t,w,h-2.2,sf,mat,rgb)

# ---------- shell with real window openings (teal glass) ----------
def shell_wall(tag,const,axis,edge,fin,gdoors=()):
    L0,L1=(X0,X1) if axis=='x' else (Y0,Y1)
    pil=[-10,-5,0,5,10] if axis=='x' else [-8,-4,0,4,8]
    wins=[(pil[i]+pil[i+1])/2 for i in range(len(pil)-1)]; ww=1.7   # windows BETWEEN pilasters
    wbands=[(fz+1.0,fz+2.2) for fz in FLOORZ]          # a window band on EVERY floor
    zb=0.0; solid=[]
    for wlo,whi in wbands:
        if wlo>zb+0.01: solid.append((zb,wlo))
        zb=whi
    if zb<ROOFZ-0.01: solid.append((zb,ROOFZ))
    dcuts=sorted((p-w/2,p+w/2) for p,w in gdoors)      # exterior ground-floor entrance openings (z<2.2)
    def in_door(pos,half): return any(c0<pos+half and pos-half<c1 for c0,c1 in dcuts)
    def band_segs(lo,hi):                              # split [lo,hi] removing door cuts
        segs=[]; cur=lo
        for c0,c1 in dcuts:
            if c0>cur: segs.append((cur,c0))
            cur=max(cur,c1)
        if cur<hi: segs.append((cur,hi));
        return segs
    for i,(z0,z1) in enumerate(solid):                 # solid full-length bands between windows
        if z0<2.2-1e-6 and dcuts:                       # ground band: carve door gap (no lintel here; z2.2+ band is the lintel)
            for si,(s0,s1) in enumerate(band_segs(L0,L1)):
                if axis=='x': add(f'{tag}sb{i}_{si}',(s0+s1)/2,const,(z0+z1)/2,s1-s0,WT,z1-z0,[fin],'Gazebo/White',WHITE)
                else:         add(f'{tag}sb{i}_{si}',const,(s0+s1)/2,(z0+z1)/2,WT,s1-s0,z1-z0,[fin],'Gazebo/White',WHITE)
        elif axis=='x': wall_run(f'{tag}sb{i}',L0,const,L1,const,z0,z1-z0,gtf=[fin])
        else:           wall_run(f'{tag}sb{i}',const,L0,const,L1,z0,z1-z0,gtf=[fin])
    cuts=sorted((p-ww/2,p+ww/2) for p in wins); segs=[]; cur=L0
    for c0,c1 in cuts:
        if c0>cur: segs.append((cur,c0))
        cur=max(cur,c1)
    if cur<L1: segs.append((cur,L1))
    for bi,(wlo,whi) in enumerate(wbands):             # mullions + glass in each floor's opening
        gnd=wlo<2.2-1e-6
        for i,(s0,s1) in enumerate(segs):
            if gnd and in_door((s0+s1)/2,(s1-s0)/2): continue
            if axis=='x': add(f'{tag}m{bi}_{i}',(s0+s1)/2,const,(wlo+whi)/2,s1-s0,WT,whi-wlo,[fin],'Gazebo/White',WHITE)
            else:         add(f'{tag}m{bi}_{i}',const,(s0+s1)/2,(wlo+whi)/2,WT,s1-s0,whi-wlo,[fin],'Gazebo/White',WHITE)
        for i,p in enumerate(wins):
            if gnd and in_door(p,ww/2): continue
            if axis=='x': add(f'{tag}g{bi}_{i}',p,const,(wlo+whi)/2,ww,0.05,whi-wlo,['+y','-y'],'Gazebo/FlatBlack',(20,20,20))
            else:         add(f'{tag}g{bi}_{i}',const,p,(wlo+whi)/2,0.05,ww,whi-wlo,['+x','-x'],'Gazebo/FlatBlack',(20,20,20))
    for p in pil:
        if axis=='x': add(f'{tag}pil{p}',p,edge,ROOFZ/2,0.5,0.35,ROOFZ,[],'Gazebo/Grey',(180,180,180))
        else:         add(f'{tag}pil{p}',edge,p,ROOFZ/2,0.35,0.5,ROOFZ,[],'Gazebo/Grey',(180,180,180))
CURGRP='shell'
ENTRY=2.5                                              # exterior entrance x on the south wall (into lobby room S2)
shell_wall('wN',Y1,'x',Y1,'-y'); shell_wall('wS',Y0,'x',Y0,'+y',gdoors=[(ENTRY,1.6)])
shell_wall('wE',X1,'y',X1,'-x'); shell_wall('wW',X0,'y',X0,'+x')
add('canopy',ENTRY,Y0-0.9,2.6,3.0,1.8,0.15,[],'Gazebo/DarkGrey',DGREY)
add('canopyL',ENTRY-1.4,Y0-1.7,1.3,0.15,0.15,2.6,ALL,'Gazebo/DarkGrey',DGREY)
add('canopyR',ENTRY+1.4,Y0-1.7,1.3,0.15,0.15,2.6,ALL,'Gazebo/DarkGrey',DGREY)

# ---------- slabs / roof / skylight / core ----------
def slab(tag,z,t,holes,col,zf=('+z','-z'),finish=True):
    xs=sorted({X0,X1}|{h[0] for h in holes}|{h[1] for h in holes}); ys=sorted({Y0,Y1}|{h[2] for h in holes}|{h[3] for h in holes}); k=0
    for i in range(len(xs)-1):
        for j in range(len(ys)-1):
            cx=(xs[i]+xs[i+1])/2; cy=(ys[j]+ys[j+1])/2
            if any(h[0]<cx<h[1] and h[2]<cy<h[3] for h in holes): continue
            sx=xs[i+1]-xs[i]; sy=ys[j+1]-ys[j]
            add(f'{tag}_{k}',cx,cy,z-t/2,sx,sy,t,list(zf),'Gazebo/Grey',GREY)
            if finish: add(f'{tag}c_{k}',cx,cy,z+0.01,sx,sy,0.02,['+z'],'Gazebo/Grey',col)
            k+=1
CURGRP='L1'; slab('floor0',0.02,0.04,[ATR],CARP[0])
CURGRP='L2'; slab('floor1',FLOORZ[1],SLAB,[ATR,SHOLE],CARP[1])   # stairwell IS floored except the SHOLE well
CURGRP='L3'; slab('floor2',FLOORZ[2],SLAB,[ATR,SHOLE],CARP[2])
CURGRP='roof'; slab('roof',ROOFZ,SLAB,[ATR],GREY,zf=['-z'],finish=False)
add('skylight',0,0,ROOFZ+0.05,ATR[1]-ATR[0]+0.4,ATR[3]-ATR[2]+0.4,0.08,['-z'],'Gazebo/Grey',(225,225,225))
for (px,py,sx,sy) in [(0,Y1,20.6,0.5),(0,Y0,20.6,0.5),(X1,0,0.5,16.6),(X0,0,0.5,16.6)]:
    add(f'parapet{px}{py}',px,py,ROOFZ+0.35,sx,sy,0.7,[],'Gazebo/Grey',(170,170,170))
CURGRP='core'
def parapet(tag,z,hp=1.0,t=0.15):   # SOLID low wall around the atrium (no holes -> clean voxblox mapping)
    o=ATR[1]+t/2                    # inner face flush with the atrium hole edge (2.5)
    L=ATR[1]+t/2                    # reach to corners
    add(f'{tag}N',0, o,z+hp/2,2*L,t,hp,['+y','-y','+z'],'Gazebo/White',WHITE)
    add(f'{tag}S',0,-o,z+hp/2,2*L,t,hp,['+y','-y','+z'],'Gazebo/White',WHITE)
    add(f'{tag}E', o,0,z+hp/2,t,2*L,hp,['+x','-x','+z'],'Gazebo/White',WHITE)
    add(f'{tag}W',-o,0,z+hp/2,t,2*L,hp,['+x','-x','+z'],'Gazebo/White',WHITE)
parapet('pL2',FLOORZ[1]); parapet('pL3',FLOORZ[2])
# Switchback in shaft x[4,10] y[-8,-2]; THIN open treads (0.1 m). Lower flight climbs EAST, upper flight climbs
# WEST so you ARRIVE facing the corridor/centre (toward the door at x=4). Treads pass up through the SHOLE well.
TF=['+x','-x','+y','-y','+z']
for fl in range(2):
    zb=FLOORZ[fl]
    for k in range(11):                                     # flight A: +x (east) along south lane, 0 -> 1.65 m
        add(f'st{fl}a{k}',5.5+k*0.38,-6.8,zb+0.15*(k+1),0.40,1.6,0.10,TF,'Gazebo/DarkGrey',DGREY)
    add(f'st{fl}land',9.2,-5.5,zb+1.65,1.4,2.6,0.12,['+z','+x','+y','-y'],'Gazebo/DarkGrey',DGREY)  # half-landing (east)
    for k in range(11):                                     # flight B: -x (west) along north lane, 1.65 -> 3.3 m; arrive facing WEST
        add(f'st{fl}b{k}',9.3-k*0.38,-4.5,zb+1.65+0.15*(k+1),0.40,1.6,0.10,TF,'Gazebo/DarkGrey',DGREY)

# ---------- PRIMITIVE furniture + clutter ----------
def chair(t,cx,cy,z,fy=1):
    add(f'{t}s',cx,cy,z+0.24,0.46,0.46,0.48,ALL,'Gazebo/DarkGrey',DGREY); add(f'{t}b',cx,cy+0.2*fy,z+0.6,0.46,0.06,0.55,ALL,'Gazebo/DarkGrey',DGREY)
def desk(t,cx,cy,z):
    add(f'{t}d',cx,cy,z+0.38,1.4,0.7,0.76,ALL,'Gazebo/Wood',WOOD); add(f'{t}m',cx,cy+0.18,z+1.02,0.5,0.05,0.32,ALL,'Gazebo/DarkGrey',(30,30,30)); chair(f'{t}c',cx,cy-0.55,z,fy=-1)
def mtable(t,cx,cy,z): add(f'{t}t',cx,cy,z+0.38,2.2,1.05,0.76,ALL,'Gazebo/Wood',WOOD)
def bookshelf(t,cx,cy,z,horiz=True):
    sx,sy=(0.9,0.35) if horiz else (0.35,0.9); bx,by=(0.8,0.28) if horiz else (0.28,0.8)
    add(f'{t}f',cx,cy,z+0.9,sx,sy,1.8,['+x','-x','+y','-y','+z'],'Gazebo/Wood',WOOD)
    for k in range(3):
        col=[(160,60,60),(60,70,160),(60,140,70),(190,150,60)][ (hash(t)+k)%4 ]
        add(f'{t}b{k}',cx,cy,z+0.45+k*0.5,bx,by,0.32,ALL,'Gazebo/Grey',col)
def whiteboard(t,cx,cy,z,horiz=True,w=1.6):
    if horiz: add(f'{t}w',cx,cy,z+1.6,w,0.05,1.0,ALL,'Gazebo/White',(250,250,250)); add(f'{t}f',cx,cy,z+1.05,w+0.1,0.06,0.06,ALL,'Gazebo/DarkGrey',DGREY)
    else:     add(f'{t}w',cx,cy,z+1.6,0.05,w,1.0,ALL,'Gazebo/White',(250,250,250)); add(f'{t}f',cx,cy,z+1.05,0.06,w+0.1,0.06,ALL,'Gazebo/DarkGrey',DGREY)
def plant(t,cx,cy,z):
    add(f'{t}p',cx,cy,z+0.3,0.4,0.4,0.6,ALL,'Gazebo/Wood',WOOD); add(f'{t}g',cx,cy,z+0.85,0.55,0.55,0.55,ALL,'Gazebo/Green',DKGRN)
def sofa(t,cx,cy,z,horiz=True):
    if horiz: add(f'{t}s',cx,cy,z+0.25,1.9,0.85,0.4,ALL,'Gazebo/DarkGrey',DGREY); add(f'{t}b',cx,cy-0.35,z+0.55,1.9,0.15,0.5,ALL,'Gazebo/DarkGrey',DGREY)
    else:     add(f'{t}s',cx,cy,z+0.25,0.85,1.9,0.4,ALL,'Gazebo/DarkGrey',DGREY); add(f'{t}b',cx-0.35,cy,z+0.55,0.15,1.9,0.5,ALL,'Gazebo/DarkGrey',DGREY)

def furnish(a,b,c,dd,fl,rng,tag,dwall,dpos,extra_clear=()):
    z=FLOORZ[fl]; M=0.45; ax,cx_,ay,dy=a+M,c-M,b+M,dd-M
    if cx_-ax<0.9 or dy-ay<0.9: return
    placed=[]; dcx,dcy={'N':(dpos,dd),'S':(dpos,b),'E':(c,dpos),'W':(a,dpos)}[dwall]
    doors_xy=[(dcx,dcy)]+list(extra_clear)                                                         # corridor door + any connecting doors
    def free(x,y,fx,fy):
        if not (ax+fx/2-1e-6<=x<=cx_-fx/2+1e-6 and ay+fy/2-1e-6<=y<=dy-fy/2+1e-6): return False   # inside room-margin
        if any((x-ex)**2+(y-ey)**2<1.3**2 for ex,ey in doors_xy): return False                     # keep every doorway clear
        if STR[0]-0.3<x<STR[1]+0.3 and STR[2]-0.3<y<STR[3]+0.3: return False                       # stairwell keep-out
        for px,py,pfx,pfy in placed:
            if abs(x-px)<(fx+pfx)/2+0.15 and abs(y-py)<(fy+pfy)/2+0.15: return False               # no overlap
        return True
    def put(fn,fx,fy,x,y):
        x=min(max(x,ax+fx/2),cx_-fx/2); y=min(max(y,ay+fy/2),dy-fy/2)
        if free(x,y,fx,fy): fn(x,y); placed.append((x,y,fx,fy)); return True
        return False
    mx,my=(ax+cx_)/2,(ay+dy)/2
    def on_wall(fn,wall,depth,halfw):   # place item FLUSH against `wall`, centered
        if wall in ('N','S'):
            lo,hi=a+halfw+0.15,c-halfw-0.15
            if hi<=lo: return False
            y=(dd-WT/2-depth/2) if wall=='N' else (b+WT/2+depth/2); x=(lo+hi)/2
        else:
            lo,hi=b+halfw+0.15,dd-halfw-0.15
            if hi<=lo: return False
            x=(c-WT/2-depth/2) if wall=='E' else (a+WT/2+depth/2); y=(lo+hi)/2
        if any((x-ex)**2+(y-ey)**2<1.2**2 for ex,ey in doors_xy): return False
        if STR[0]-0.2<x<STR[1]+0.2 and STR[2]-0.2<y<STR[3]+0.2: return False
        for px,py,pfx,pfy in placed:
            if abs(x-px)<pfx/2+0.55 and abs(y-py)<pfy/2+0.55: return False
        fn(x,y); placed.append((x,y,0.9,0.9)); return True
    perp=['E','W'] if dwall in ('N','S') else ['N','S']; opp={'N':'S','S':'N','E':'W','W':'E'}[dwall]
    if fl==1:
        put(lambda x,y: desk(f'{tag}d',x,y,z),1.4,1.35,mx,ay+0.75)
        if rng.random()<0.6: put(lambda x,y: plant(f'{tag}pl',x,y,z),0.55,0.55,ax+0.3,dy-0.3)
    else:
        if cx_-ax>2.6 and dy-ay>1.5 and put(lambda x,y: mtable(f'{tag}mt',x,y,z),2.2,1.1,mx,my):
            for i,(ox,oy) in enumerate([(-1.35,0),(1.35,0),(0,-0.95),(0,0.95)]):
                put(lambda x,y,i=i,oy=oy: chair(f'{tag}c{i}',x,y,z,fy=1 if oy>0 else -1),0.5,0.5,mx+ox,my+oy)
        else:
            put(lambda x,y: desk(f'{tag}d',x,y,z),1.4,1.35,mx,my)
    for w in perp:   # bookshelf glued to a side wall
        if on_wall(lambda x,y,w=w: bookshelf(f'{tag}bs',x,y,z,horiz=(w in('N','S'))),w,0.35,0.45): break
    span=(c-a) if opp in('N','S') else (dd-b)   # whiteboard glued to the wall opposite the door
    if span>=1.6:
        wb=min(1.4,span-0.4); on_wall(lambda x,y: whiteboard(f'{tag}wb',x,y,z,horiz=(opp in('N','S')),w=wb),opp,0.06,wb/2)

# ---- Floor plan: 7 rooms + stairwell wrapped around a THIN open corridor ring (H=[-4,4]) ----
# Corridor = interior of H minus atrium: a 1.5 m ring the drone flies through. Rooms are a pinwheel of
# arms split into sensible rooms; EVERY room opens onto the ring with exactly ONE uniform DW door and no
# wall behind it. N1<->N2 also share a connecting door in their partition (extra keep-clear for furniture).
#   (x0,y0,x1,y1, door_wall, door_pos, tag)
ROOMS=[(-4,4, 2,8,'S',-1,'N1'),( 2,4,10,8,'S', 3,'N2'),        # north band  (2 rooms)
       (-10,1,-4,8,'E', 2.5,'W1'),(-10,-4,-4,1,'E',-1.5,'W2'),  # west band   (2 rooms)
       (-10,-8,-2,-4,'N',-3,'S1'),(-2,-8,4,-4,'N',1,'S2'),      # south band  (2; S2 = entrance lobby zone)
       (4,-2,10,4,'W', 1,'E1')]                                 # east room   (1 big; shortened so stair door clears)
CONNECT={'N1':[(2,6)],'N2':[(2,6)],'W1':[(-7,1)],'W2':[(-7,1)]}  # TWO connecting doors/floor: N1<->N2 (x=2,y=6) & W1<->W2 (x=-7,y=1)
def walls(fl):
    global CURGRP; CURGRP='L%d'%(fl+1); z=FLOORZ[fl]; h=FH-SLAB-0.1
    W=[(-4,4,4,4,[(-1,DW),(3,DW)]), (-4,-4,4,-4,[(-3,DW),(1,DW)]),  # corridor ring: top / bottom (doors)
       (-4,-4,-4,4,[(2.5,DW),(-1.5,DW)]), (4,-4,4,4,[(1,DW),(-3,DW)]),  # ring: left / right (right-lower door = stairwell, at y=-3)
       (4,4,10,4,[]), (4,-8,4,-4,[]), (-4,4,-4,8,[]), (-10,-4,-4,-4,[]),  # inter-arm solid seams
       (2,4,2,8,[(6,DW)]), (-10,1,-4,1,[(-7,DW)]), (-2,-8,-2,-4,[]), (4,-2,10,-2,[])]  # partitions: N1/N2 & W1/W2 connecting doors; E1/STAIR moved to y=-2
    for i,(x0,y0,x1,y1,dr) in enumerate(W): wall_run(f'w{fl}_{i}',x0,y0,x1,y1,z,h,doors=dr)
def furnish_floor(fl,skip=()):
    global CURGRP; CURGRP='L%d'%(fl+1); rng=random.Random(fl)
    for a,b,c,dd,dwall,dpos,tag in ROOMS:
        if tag in skip: continue
        furnish(a,b,c,dd,fl,rng,f'f{fl}_{tag}',dwall,dpos,extra_clear=CONNECT.get(tag,()))
walls(1); walls(2)
for fl in (1,2): furnish_floor(fl)
# ---- GROUND FLOOR: open plan (no interior walls), densely furnished ----
CURGRP='L1'
furnish_floor(0,skip=('S2',))                                  # furniture clusters in the 6 non-lobby zones
# entrance lobby in the open S2 zone (west of the entrance lane x~[1.7,3.3], which stays clear for fly-in)
add('reception',-0.4,-6.6,0.5,2.2,0.8,1.05,ALL,'Gazebo/Wood',WOOD)
add('recdesk_t',-0.4,-6.6,1.11,2.2,0.8,0.04,['+z'],'Gazebo/DarkGrey',(55,55,55))
chair('recch',-0.4,-7.3,0,fy=-1)
sofa('lsofaW',-1.5,-4.9,0,False); plant('lpW',0.7,-5.0,0); chair('lch2',-0.4,-5.6,0,fy=1)

# ================= write world (per-floor models) =================
HEAD=open(os.path.join(WORLDS,'grass_plane_school.world')).read()
HEAD=HEAD[:HEAD.index("    <model name='school'>")]
def link(n,cx,cy,cz,sx,sy,sz,m):
    return (f"      <link name='{n}'><pose>{cx:g} {cy:g} {cz:g} 0 0 0</pose>"
            f"<collision name='c'><geometry><box><size>{sx:g} {sy:g} {sz:g}</size></box></geometry></collision>"
            f"<visual name='v'><geometry><box><size>{sx:g} {sy:g} {sz:g}</size></box></geometry>"
            f"<material><script><uri>file://media/materials/scripts/Gazebo.material</uri><name>{m}</name></script></material></visual>"
            f"<kinematic>0</kinematic></link>\n")
groups={}
for b in boxes: groups.setdefault(b[10],[]).append(b)
with open(FW,'w') as f:
    f.write(HEAD)
    for g in ['shell','L1','L2','L3','core','roof']:
        f.write(f"    <model name='building_{g}'><static>1</static>\n")
        for (n,cx,cy,cz,sx,sy,sz,fa,m,c,gg) in groups.get(g,[]): f.write(link(n,cx,cy,cz,sx,sy,sz,m))
        f.write("      <pose>0 0 0 0 0 0</pose>\n    </model>\n")
    f.write("  </world>\n</sdf>\n")
print(f'wrote {FW} ({len(boxes)} links in {len(groups)} models: '+", ".join(f"{g}:{len(groups.get(g,[]))}" for g in ['shell','L1','L2','L3','core','roof'])+")")

# ================= GT =================
def fr(a,b):
    n=max(1,int(round((b-a)/D))); return [a+(i+0.5)*(b-a)/n for i in range(n)]
# ---- observable-surface filter ----
# GT samples ONLY box faces (surfaces, never interior volume). This additionally DROPS any face sample
# whose outward side is buried inside another solid box (floor under walls, furniture backs on walls,
# corner overlaps): those surfaces are occluded and can never be reconstructed by the sensor.
import math as _m
_CELL=1.0; _grid={}
for _bi,_b in enumerate(boxes):
    _cx,_cy,_cz,_sx,_sy,_sz=_b[1:7]
    for _gx in range(_m.floor((_cx-_sx/2)/_CELL),_m.floor((_cx+_sx/2)/_CELL)+1):
        for _gy in range(_m.floor((_cy-_sy/2)/_CELL),_m.floor((_cy+_sy/2)/_CELL)+1):
            for _gz in range(_m.floor((_cz-_sz/2)/_CELL),_m.floor((_cz+_sz/2)/_CELL)+1):
                _grid.setdefault((_gx,_gy,_gz),[]).append(_bi)
def occluded(px,py,pz,eps=0.02):
    for _bi in _grid.get((_m.floor(px/_CELL),_m.floor(py/_CELL),_m.floor(pz/_CELL)),()):
        _cx,_cy,_cz,_sx,_sy,_sz=boxes[_bi][1:7]
        if abs(px-_cx)<_sx/2-eps and abs(py-_cy)<_sy/2-eps and abs(pz-_cz)<_sz/2-eps: return True
    return False
OUT=0.05                                            # step just outside the face along its normal
pts=[]
def face(cx,cy,cz,sx,sy,sz,fc,c):
    x0,x1=cx-sx/2,cx+sx/2;y0,y1=cy-sy/2,cy+sy/2;z0,z1=cz-sz/2,cz+sz/2
    if fc in('+x','-x'):
        x=x1 if fc=='+x' else x0;nrm=(1 if fc=='+x' else -1,0,0)
        for u in fr(y0,y1):
            for v in fr(z0,z1):
                if inside(x,u,v) and not occluded(x+nrm[0]*OUT,u,v): pts.append((x,u,v,c,nrm))
    elif fc in('+y','-y'):
        y=y1 if fc=='+y' else y0;nrm=(0,1 if fc=='+y' else -1,0)
        for u in fr(x0,x1):
            for v in fr(z0,z1):
                if inside(u,y,v) and not occluded(u,y+nrm[1]*OUT,v): pts.append((u,y,v,c,nrm))
    else:
        z=z1 if fc=='+z' else z0;nrm=(0,0,1 if fc=='+z' else -1)
        for u in fr(x0,x1):
            for v in fr(y0,y1):
                if inside(u,v,z) and not occluded(u,v,z+nrm[2]*OUT): pts.append((u,v,z,c,nrm))
for (n,cx,cy,cz,sx,sy,sz,fa,m,c,gg) in boxes:
    for fc in fa: face(cx,cy,cz,sx,sy,sz,fc,c)
hdr=(f'ply\nformat binary_little_endian 1.0\nelement vertex {len(pts)}\n'
     'property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n'
     'property float nx\nproperty float ny\nproperty float nz\nproperty float scalar_Original_cloud_index\nend_header\n')
with open(PLY,'wb') as f:
    f.write(hdr.encode()); pk=struct.Struct('<fffBBBffff').pack
    for (x,y,z,c,nrm) in pts: f.write(pk(x,y,z,c[0],c[1],c[2],nrm[0],nrm[1],nrm[2],0.0))
print(f'wrote {PLY} ({len(pts)} pts)')
