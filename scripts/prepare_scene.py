#!/usr/bin/python3
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ail_tb4_maze.world_factory import generate
root=Path(__file__).resolve().parents[1]
c=generate(root,root/'.runtime/state',20,10,42,'official')
print('Prepared textured official maze:',c['clearance_validation'])
