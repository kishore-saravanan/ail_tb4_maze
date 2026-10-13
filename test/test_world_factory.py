from pathlib import Path
import pytest
from ail_tb4_maze.world_factory import generate
PKG=Path(__file__).resolve().parents[1]
@pytest.mark.parametrize('size,cells',[(12,12),(20,10),(40,20)])
def test_random_maze_clearance_and_topology(tmp_path,size,cells):
 c=generate(PKG,tmp_path,size,cells,31415,'random')
 assert c['clearance_validation']['passed']
 assert c['layout_stats']['dead_ends']>=3 and c['layout_stats']['independent_loops']>=1
 assert c['layout_stats']['connected_cells']==cells*cells
 assert c['layout_stats']['corridor_width_m']>.5

def test_reproducible_geometry_and_new_scene_names(tmp_path):
 a=generate(PKG,tmp_path/'a',20,10,7,'random');b=generate(PKG,tmp_path/'b',20,10,7,'random')
 assert a['wall_boxes']==b['wall_boxes'] and a['exit']==b['exit']
 assert a['scene_models']!=b['scene_models']

def test_different_seed_changes_layout(tmp_path):
 a=generate(PKG,tmp_path/'a',20,10,7,'random');b=generate(PKG,tmp_path/'b',20,10,8,'random')
 assert a['wall_boxes']!=b['wall_boxes']

def test_small_official_preset_clearance(tmp_path):
 c=generate(PKG,tmp_path,12,10,7,'official');assert c['clearance_validation']['passed']

def test_reject_narrow_grid(tmp_path):
 with pytest.raises(ValueError):generate(PKG,tmp_path,12,20,1,'random')
