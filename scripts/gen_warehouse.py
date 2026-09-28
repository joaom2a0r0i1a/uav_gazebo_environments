#!/usr/bin/env python3
# Builds the cluttered warehouse world and its analytic ground truth cloud.
# Real pallet racking with uprights, beams, decks, pallets and carton stacks.
# A raised loading dock with an open roll up door on the east and a personnel entrance west.
# Walls carry real openings so windows and doors read from both sides.
# Ground truth is analytic surface sampling of the same boxes, observable only, clamped to the interior box.
import os, struct, random
random.seed(7)

# layout params
LX, LY, LZ   = 40.0, 24.0, 7.0
WT, CT       = 0.2, 0.2
ROWS_Y       = [-8.0, -2.7, 2.7, 8.0]   # tighter aisles + a clear perimeter gap to the walls
RACK_DEPTH   = 3.0
RACK_H       = 3.5
RACK_X0, RACK_X1 = -14.0, 15.0
CROSS_X0, CROSS_X1 = 0.0, 2.0
BAY_PITCH    = 2.5
LEVELS_Z     = [0.20, 1.45, 2.70]       # shelf deck heights
FILL_PROB    = 0.62
D            = 0.06                       # GT sample spacing (m)
ROOT      = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_OUT = os.path.join(ROOT, 'worlds', 'grass_plane_warehouse.world')
# The evaluation package consumes this cloud so it lands there unless GT_OUT says otherwise.
PLY_OUT   = os.environ.get('GT_OUT', os.path.join(ROOT, '..', 'single', 'motion_planning', 'data',
                           'gt_warehouse_processed.ply'))
HX, HY = LX/2.0, LY/2.0                   # 20, 12 eval bounded_box (Warehouse.yaml)
IXW, IYW = HX-WT/2, HY-WT/2               # inner wall faces (19.9, 11.9)

# colours (used as BOTH the GT colour and the solid visual diffuse)
SIDING=(196,176,132); BASE=(66,78,98); PIL=(150,120,80)
DOOR=(46,96,150); DFRAME=(40,44,52); MAN=(60,70,86); GLASS=(150,196,232)
CEIL=(175,171,161); FLOOR=(150,148,142); TRUSS=(70,74,82)
ORANGE=(232,120,22); BEAM=(38,84,168); DECK=(92,92,98)
WOOD=(150,100,55); CARD=(202,176,132); DOCKPLAT=(122,124,128); BUMPER=(28,28,30)

# box collector. (name,cx,cy,cz,sx,sy,sz,faces,material_or_None,rgb,grp)
boxes=[]; CURGRP='shell'
def add(name,cx,cy,cz,sx,sy,sz,faces,mat,rgb):
    boxes.append((name,cx,cy,cz,sx,sy,sz,faces,mat,rgb,CURGRP))
ALL=['+x','-x','+y','-y','+z','-z']
def line(a,b,step):
    n=int(round((b-a)/step)); return [a+(b-a)*i/n for i in range(n+1)]

# SHELL. walls with real openings
CURGRP='shell'
WZ0,WZ1=5.0,6.2          # clerestory window band
WINW=1.6                 # window opening width
def band_x(tag,y,z0,z1,x0,x1,rgb,gf,openings=()):   # wall spanning X (at y), band z0..z1, gaps at openings
    cuts=sorted((p-w/2,p+w/2) for p,w in openings); segs=[]; cur=x0
    for c0,c1 in cuts:
        if c0>cur+1e-6: segs.append((cur,c0))
        cur=max(cur,c1)
    if cur<x1-1e-6: segs.append((cur,x1))
    for i,(s0,s1) in enumerate(segs): add(f'{tag}{i}',(s0+s1)/2,y,(z0+z1)/2,s1-s0,WT,z1-z0,[gf],None,rgb)
def band_y(tag,x,z0,z1,y0,y1,rgb,gf,openings=()):   # wall spanning Y (at x)
    cuts=sorted((p-w/2,p+w/2) for p,w in openings); segs=[]; cur=y0
    for c0,c1 in cuts:
        if c0>cur+1e-6: segs.append((cur,c0))
        cur=max(cur,c1)
    if cur<y1-1e-6: segs.append((cur,y1))
    for i,(s0,s1) in enumerate(segs): add(f'{tag}{i}',x,(s0+s1)/2,(z0+z1)/2,WT,s1-s0,z1-z0,[gf],None,rgb)

def wall_NS(tag,y,gf):     # north and south wall (spans X). tan siding + slate wainscot + clerestory windows
    winx=line(-16,16,4)
    band_x(f'{tag}base',y,0,1.2,-HX,HX,BASE,gf)
    band_x(f'{tag}sid', y,1.2,WZ0,-HX,HX,SIDING,gf)
    band_x(f'{tag}mul', y,WZ0,WZ1,-HX,HX,SIDING,gf,openings=[(p,WINW) for p in winx])
    band_x(f'{tag}top', y,WZ1,LZ,-HX,HX,SIDING,gf)
    for p in winx: add(f'{tag}g{p:.0f}',p,y,(WZ0+WZ1)/2,WINW,0.05,WZ1-WZ0,[gf],None,GLASS)   # glass in opening
    s=-1 if gf=='-y' else 1
    for p in line(-14,14,4): add(f'{tag}pil{p:.0f}',p,y+s*0.30,3.5,0.5,0.6,7.0,[gf,'+x','-x'],None,PIL)  # BETWEEN windows

def clerestory_Y(tag,x,gf):   # clerestory window band for east and west walls (spans Y)
    winy=line(-9,9,3)
    band_y(f'{tag}mul',x,WZ0,WZ1,-HY,HY,SIDING,gf,openings=[(p,WINW) for p in winy])
    band_y(f'{tag}top',x,WZ1,LZ,-HY,HY,SIDING,gf)
    for p in winy: add(f'{tag}g{p:.0f}',x,p,(WZ0+WZ1)/2,0.05,WINW,WZ1-WZ0,[gf],None,GLASS)

def door_frame_Y(tag,x,yc,w,h,gf):   # frame around a Y-wall door opening (both sides visible)
    add(f'{tag}lt',x,yc,h+0.12,WT+0.02,w+0.3,0.24,[gf,'+z','-z'],None,DFRAME)   # lintel
    for s in (-1,1): add(f'{tag}j{s}',x,yc+s*(w/2+0.1),h/2,WT+0.02,0.2,h,[gf,'+y','-y'],None,DFRAME)

# EAST wall = DOCK (spans Y). 4 roll up doors; the y=+7.5 door is OPEN (vehicle entry). Clerestory above.
DOORS_E=[-7.5,-2.5,2.5,7.5]; DW_E=3.0; DHZ=4.0; OPEN_E={7.5}
band_y('wEbase',HX,0,1.2,-HY,HY,BASE,'-x',openings=[(y,DW_E) for y in DOORS_E])
band_y('wElo', HX,1.2,DHZ,-HY,HY,SIDING,'-x',openings=[(y,DW_E) for y in DOORS_E])
band_y('wEmid',HX,DHZ,WZ0,-HY,HY,SIDING,'-x')
clerestory_Y('wE',HX,'-x')
for y in DOORS_E:
    door_frame_Y(f'dfE{y:.0f}',HX,y,DW_E,DHZ,'-x')
    if y not in OPEN_E: add(f'doorE{y:.0f}',HX-0.02,y,DHZ/2,0.10,DW_E,DHZ,['-x'],None,DOOR)   # closed roll up (blue)

# WEST wall (spawn end, spans Y) with a personnel man door (OPEN entry) + clerestory windows
MANY=0.0; MANW=1.4; MANH=2.4
band_y('wWbase',-HX,0,1.2,-HY,HY,BASE,'+x',openings=[(MANY,MANW)])
band_y('wWlo', -HX,1.2,MANH,-HY,HY,SIDING,'+x',openings=[(MANY,MANW)])
band_y('wWmid',-HX,MANH,WZ0,-HY,HY,SIDING,'+x')
clerestory_Y('wW',-HX,'+x')
door_frame_Y('dfW',-HX,MANY,MANW,MANH,'+x')
for p in line(-7.5,7.5,3): add(f'wWpil{p*10:.0f}',-HX+0.30,p,3.5,0.6,0.5,7.0,['+x','+y','-y'],None,PIL)  # clears man door

wall_NS('wN',HY,'-y'); wall_NS('wS',-HY,'+y')
# E wall has 4 dock doors + clerestory windows -> no interior pilasters (would foul the door openings)

# floor slab (concrete) stays with the shell
add('floor',0,0,0.0,LX-0.4,LY-0.4,0.04,['+z'],None,FLOOR)
# ROOF (ceiling + trusses) is its OWN model -> hide it in the GUI for a full top down view
CURGRP='roof'
add('ceiling',0,0,LZ-0.1,LX-0.4,LY-0.4,CT,['-z'],'Gazebo/CeilingTiled',CEIL)
for i,x in enumerate(line(-17,17,5)):
    add(f'truss{i}',x,0,6.6,0.2,23.2,0.35,['+x','-x','-z'],None,TRUSS)
CURGRP='shell'

# RACKS + LOADS
segs=[(RACK_X0,CROSS_X0),(CROSS_X1,RACK_X1)]
nload=0
def carton_stack(cx,cy,zbase,tag):
    global nload; CURGRP_local=None
    n=random.choice([1,1,2,2,3]); z=zbase
    for k in range(n):
        w=random.uniform(0.4,0.75); d=random.uniform(0.4,0.75); h=random.uniform(0.35,0.6)
        add(f'{tag}_c{k}',cx+random.uniform(-0.1,0.1),cy+random.uniform(-0.1,0.1),z+h/2,
            w,d,h,['+x','-x','+y','-y','+z'],None,CARD); z+=h
    nload+=1
def pallet_load(cx,cy,zdeck,tag):
    add(f'{tag}_p',cx,cy,zdeck+0.075,1.15,0.95,0.15,['+x','-x','+y','-y','+z'],None,WOOD)
    carton_stack(cx,cy,zdeck+0.15,tag)

CURGRP='racks'
rack_cells=[]   # (cx,cy,z) deck load positions -> filled in loads pass
for iy,yc in enumerate(ROWS_Y):
    yf,yb=yc-RACK_DEPTH/2, yc+RACK_DEPTH/2
    for ix,(x0,x1) in enumerate(segs):
        L=x1-x0; mid=(x0+x1)/2
        nb=max(1,int(round(L/BAY_PITCH))); ux=[x0+i*L/nb for i in range(nb+1)]
        tagS=f'r{iy}{ix}'
        for j,x in enumerate(ux):
            for y in (yf,yb): add(f'{tagS}_u{j}_{0 if y==yf else 1}',x,y,RACK_H/2,0.12,0.12,RACK_H,ALL,None,ORANGE)
        for li,z in enumerate(LEVELS_Z+[RACK_H-0.1]):
            for y in (yf,yb): add(f'{tagS}_b{li}_{0 if y==yf else 1}',mid,y,z,L,0.12,0.1,['+x','-x','+z','-z','+y','-y'],None,BEAM)
        for li,z in enumerate(LEVELS_Z):
            add(f'{tagS}_d{li}',mid,yc,z,L,RACK_DEPTH,0.05,['+z','-z'],None,DECK)
            for bi in range(nb):
                cx=(ux[bi]+ux[bi+1])/2
                for cy in (yc-0.6, yc+0.6):
                    if random.random()<FILL_PROB: rack_cells.append((cx,cy,z+0.05,f'{tagS}L{li}_{bi}_{0 if cy<yc else 1}'))
CURGRP='loads'
for (cx,cy,z,tag) in rack_cells: pallet_load(cx,cy,z,tag)

# DOCK (east staging)
CURGRP='dock'
# raised concrete dock platform against the east wall + edge bumpers + steps down to the floor
add('dock_plat',18.95,0,0.5,1.9,22.0,1.0,['+z','-x','+y','-y'],None,DOCKPLAT)   # x[18,19.9], GLUED to east wall
for y in DOORS_E:                                                             # dock leveller bumpers at each door
    for s in (-1,1): add(f'dock_bmp{y:.0f}{s}',18.0,y+s*1.6,0.5,0.08,0.25,1.0,['-x','+y','-y'],None,BUMPER)
for k in range(4): add(f'dock_step{k}',17.6-k*0.3,-10.4,0.5-k*0.125,0.3,1.6,1.0-k*0.25,['+z','-x'],None,DOCKPLAT)  # steps at south end
# staging pallets. kept well clear of walls so the camera can see all around them (no thin unreachable gaps)
for _ in range(9): pallet_load(random.uniform(16.3,17.2),random.uniform(-10.5,10.5),0.0,f'stg{_}')     # floor, clear of platform edge (x<17.9)
for _ in range(5): pallet_load(random.uniform(18.58,18.72),random.uniform(-10,10),1.0,f'onplat{_}')      # on platform, >=0.6 m off the east wall
for _ in range(5): pallet_load(random.uniform(-18.6,-15.5),random.uniform(-10.5,10.5),0.0,f'west{_}')   # >=0.6 m off the west wall

# WORLD (multi model)
HEAD='''<?xml version="1.0" ?>
<sdf version='1.7'>
  <world name='default'>
    <plugin name='mrs_gazebo_static_transform_republisher_plugin' filename='libMrsGazeboCommonResources_StaticTransformRepublisher.so'/>
    <spherical_coordinates><surface_model>EARTH_WGS84</surface_model><latitude_deg>47.3977</latitude_deg><longitude_deg>8.54559</longitude_deg><elevation>0</elevation><heading_deg>0</heading_deg></spherical_coordinates>
    <physics name='default_physics' default='0' type='ode'>
      <ode><solver><type>quick</type><iters>10</iters><sor>1.3</sor><use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling></solver>
        <constraints><cfm>0</cfm><erp>0.2</erp><contact_max_correcting_vel>1000</contact_max_correcting_vel><contact_surface_layer>0.001</contact_surface_layer></constraints></ode>
      <max_step_size>0.004</max_step_size><real_time_factor>1</real_time_factor><real_time_update_rate>250</real_time_update_rate></physics>
    <scene><shadows>0</shadows><ambient>0.4 0.4 0.4 1</ambient><background>0.7 0.7 0.7 1</background></scene>
    <light name='sun' type='directional'><pose>0 0 1000 0.4 0.2 0</pose><diffuse>1 1 1 1</diffuse><specular>0.6 0.6 0.6 1</specular><direction>0.1 0.1 -0.9</direction>
      <attenuation><range>20</range><constant>0.5</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation><cast_shadows>1</cast_shadows><spot><inner_angle>0</inner_angle><outer_angle>0</outer_angle><falloff>0</falloff></spot></light>
    <model name='ground_plane'><static>1</static><link name='link'>
      <collision name='collision'><pose>0 0 0 0 -0 0</pose><geometry><plane><normal>0 0 1</normal><size>250 250</size></plane></geometry>
        <surface><contact><ode><min_depth>0.01</min_depth><max_vel>0</max_vel></ode></contact><friction><ode/><torsional><ode/></torsional></friction><bounce/></surface><max_contacts>10</max_contacts></collision>
      <visual name='grass'><pose>0 0 0 0 -0 0</pose><cast_shadows>0</cast_shadows><geometry><mesh><uri>file://grass_plane/meshes/grass_plane.dae</uri></mesh></geometry></visual>
      <self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link></model>
    <model name='the_void'><static>1</static><link name='link'><pose>0 0 0.1 0 -0 0</pose>
      <visual name='the_void'><pose>0 0 2 0 -0 0</pose><geometry><sphere><radius>0.25</radius></sphere></geometry><material><script><uri>file://media/materials/scripts/Gazebo.material</uri><name>Gazebo/Black</name></script></material></visual>
      <self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic></link><pose>-1000 -1000 0 0 -0 0</pose></model>
    <plugin name='mrs_gazebo_rviz_cam_synchronizer' filename='libMrsGazeboCommonResources_RvizCameraSynchronizer.so'>
      <target_frame_id>gazebo_user_camera</target_frame_id><world_origin_frame_id>uav1/gps_origin</world_origin_frame_id><frame_to_follow>uav1</frame_to_follow></plugin>
    <gravity>0 0 -9.8066</gravity><magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field><atmosphere type='adiabatic'/><wind/>
    <gui fullscreen='0'><camera name='camera'><pose>-16 -9.5 5.9 0 0.22 0.5</pose><view_controller>orbit</view_controller><projection_type>perspective</projection_type></camera></gui>
'''
def mat_sdf(mat,rgb):
    if mat: return f"<material><script><uri>file://media/materials/scripts/Gazebo.material</uri><name>{mat}</name></script></material>"
    r,g,b=rgb[0]/255.,rgb[1]/255.,rgb[2]/255.
    return (f"<material><ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient><diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>"
            f"<specular>0.1 0.1 0.1 1</specular></material>")
def link_sdf(name,cx,cy,cz,sx,sy,sz,mat,rgb,layer=None):
    meta=f"<meta><layer>{layer}</layer></meta>" if layer is not None else ""   # GUI Layers tab toggle
    return (f"      <link name='{name}'><pose>{cx:g} {cy:g} {cz:g} 0 0 0</pose>"
            f"<collision name='c'><geometry><box><size>{sx:g} {sy:g} {sz:g}</size></box></geometry></collision>"
            f"<visual name='v'>{meta}<geometry><box><size>{sx:g} {sy:g} {sz:g}</size></box></geometry>{mat_sdf(mat,rgb)}</visual>"
            f"<self_collide>0</self_collide><kinematic>0</kinematic></link>\n")
groups={}
for b in boxes: groups.setdefault(b[10],[]).append(b)
GORDER=['shell','racks','loads','dock','roof']
LAYER={'roof':0}   # roof visuals -> layer 0 so the GUI "Layers" tab gives a show/hide checkbox
with open(WORLD_OUT,'w') as f:
    f.write(HEAD)
    for g in GORDER:
        f.write(f"    <model name='warehouse_{g}'><static>1</static>\n")
        for (n,cx,cy,cz,sx,sy,sz,faces,mat,rgb,gg) in groups.get(g,[]): f.write(link_sdf(n,cx,cy,cz,sx,sy,sz,mat,rgb,layer=LAYER.get(g)))
        f.write("      <pose>0 0 0 0 0 0</pose>\n    </model>\n")
    f.write("  </world>\n</sdf>\n")
print(f'wrote {WORLD_OUT}  ({len(boxes)} links in {len(GORDER)} models: '+", ".join(f"{g}:{len(groups.get(g,[]))}" for g in GORDER)+f"; {nload} loads)")

# GT CLOUD
def fr(a,b):
    n=max(1,int(round((b-a)/D))); return [a+(i+0.5)*(b-a)/n for i in range(n)]
# INTERIOR-only clamp. strictly inside the walls (never the exterior of the shell / anything past the BBX).
def inside(x,y,z): return -IXW-0.02<=x<=IXW+0.02 and -IYW-0.02<=y<=IYW+0.02 and -0.03<=z<=LZ+0.02
# observable surface filter (drops face samples whose outward side is buried inside another solid box)
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
OUT=0.05
pts=[]
def face(cx,cy,cz,sx,sy,sz,fc,rgb):
    x0,x1=cx-sx/2,cx+sx/2;y0,y1=cy-sy/2,cy+sy/2;z0,z1=cz-sz/2,cz+sz/2
    if fc in('+x','-x'):
        x=x1 if fc=='+x' else x0;n=(1 if fc=='+x' else -1,0,0)
        for y in fr(y0,y1):
            for z in fr(z0,z1): (pts.append((x,y,z,rgb,n)) if inside(x,y,z) and not occluded(x+n[0]*OUT,y,z) else None)
    elif fc in('+y','-y'):
        y=y1 if fc=='+y' else y0;n=(0,1 if fc=='+y' else -1,0)
        for x in fr(x0,x1):
            for z in fr(z0,z1): (pts.append((x,y,z,rgb,n)) if inside(x,y,z) and not occluded(x,y+n[1]*OUT,z) else None)
    else:
        z=z1 if fc=='+z' else z0;n=(0,0,1 if fc=='+z' else -1)
        for x in fr(x0,x1):
            for y in fr(y0,y1): (pts.append((x,y,z,rgb,n)) if inside(x,y,z) and not occluded(x,y,z+n[2]*OUT) else None)
for (n,cx,cy,cz,sx,sy,sz,faces,mat,rgb,gg) in boxes:
    for fc in faces: face(cx,cy,cz,sx,sy,sz,fc,rgb)

hdr=(f'ply\nformat binary_little_endian 1.0\nelement vertex {len(pts)}\n'
     'property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n'
     'property float nx\nproperty float ny\nproperty float nz\nproperty float scalar_Original_cloud_index\nend_header\n')
with open(PLY_OUT,'wb') as f:
    f.write(hdr.encode()); pk=struct.Struct('<fffBBBffff').pack
    for (x,y,z,rgb,nrm) in pts: f.write(pk(x,y,z,rgb[0],rgb[1],rgb[2],nrm[0],nrm[1],nrm[2],0.0))
print(f'wrote {PLY_OUT}  ({len(pts)} pts, ~{len(pts)*31/1e6:.0f} MB)')
