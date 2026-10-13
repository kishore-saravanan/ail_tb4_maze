#!/usr/bin/python3
"""Privileged offline clearance validation, never loaded into controller domain."""
from pathlib import Path
from collections import deque
import xml.etree.ElementTree as E
import math,json,hashlib
ROOT=Path(__file__).resolve().parents[1]
P=ROOT
C=json.loads((P/'config/challenge.json').read_text());radius=C['footprint_radius'];step=.1
world=E.parse(P/'worlds/maze_escape.sdf').getroot().find('world');boxes=[]
for model in world.findall('model'):
 if model.get('name') not in ('border','obstacles'):continue
 mx,my,*_=map(float,model.findtext('pose').split())
 for link in model.findall('link'):
  x,y,*_=map(float,link.findtext('pose').split());sx,sy,*_=map(float,link.findtext('collision/geometry/box/size').split())
  boxes.append((mx+x-sx/2,mx+x+sx/2,my+y-sy/2,my+y+sy/2))
def clearance(x,y):
 return min(math.hypot(max(a-x,0,x-b),max(c-y,0,y-d)) for a,b,c,d in boxes)
def point(i,j):return -9.5+i*step,-9.5+j*step
def free(i,j):
 if not (0<=i<=200 and 0<=j<=190):return False
 x,y=point(i,j);return clearance(x,y)>radius+.02
start=tuple(round((C['start'][k]+9.5)/step) for k in ('x','y'))
assert free(*start)
seen={start};q=deque([start]);reachable=False
while q:
 i,j=q.popleft();x,y=point(i,j)
 if x-radius>C['exit']['x'] and C['exit']['y_min']+radius<y<C['exit']['y_max']-radius:reachable=True
 for n in [(i-1,j),(i+1,j),(i,j-1),(i,j+1)]:
  if n not in seen and free(*n):seen.add(n);q.append(n)
assert reachable,'No inflated-footprint path to outside exit'
# Explicit closed circuit around an original isolated obstacle, with >radius clearance.
loop=[(-5.5,-2.5),(-3.5,-2.5),(-3.5,-.5),(-5.5,-.5),(-5.5,-2.5)]
loop_clearance=min(clearance(a[0]+(b[0]-a[0])*t/100,a[1]+(b[1]-a[1])*t/100) for a,b in zip(loop,loop[1:]) for t in range(101))
assert loop_clearance>radius
# Count dead ends and intersections on 1m lattice of clear passage centres.
coarse={(i,j) for i in range(20) for j in range(20) if clearance(-9.5+i,-9.5+j)>radius+.02}
degrees=[sum(n in coarse for n in [(i-1,j),(i+1,j),(i,j-1),(i,j+1)]) for i,j in coarse]
out={'world_sha256':hashlib.sha256((P/'worlds/maze_escape.sdf').read_bytes()).hexdigest(),'world_dimensions_m':[20,20],'wall_links':len(boxes),'exit_width_m':2,'footprint_radius_m':radius,'grid_resolution_m':step,'safety_margin_m':.02,'start_clearance_m':clearance(C['start']['x'],C['start']['y']),'reachable_exit':reachable,'reachable_grid_cells':len(seen),'dead_end_cells_1m_grid':degrees.count(1),'intersection_cells_1m_grid':sum(d>=3 for d in degrees),'loop_min_clearance_m':loop_clearance}
(ROOT/'.runtime/results/geometry.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
# Overview for infrastructure operator only. Do not give to navigation agent.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle
fig,ax=plt.subplots(figsize=(9,9))
for a,b,c,d in boxes:ax.add_patch(Rectangle((a,c),b-a,d-c,color='#23445b'))
ax.add_patch(Circle((C['start']['x'],C['start']['y']),radius,color='#f08832'))
ax.annotate('Start', (C['start']['x'],C['start']['y']),xytext=(-9,-11))
ax.plot([10.01]*2,[-3,-1],color='#32b67a',linewidth=5);ax.text(10.5,-2,'Exit',rotation=90,va='center')
ax.plot(*zip(*loop),color='#8869c8',linestyle='--',label='Clear loop around existing obstacle')
ax.set(xlim=(-11,12),ylim=(-12,11),aspect='equal',xlabel='x (metres)',ylabel='y (metres)',title='AI Legion · Official TurtleBot 4 maze adaptation\nPrivileged infrastructure overview')
ax.legend(loc='upper left');fig.tight_layout();fig.savefig(ROOT/'docs/maze_overview.png',dpi=160)
