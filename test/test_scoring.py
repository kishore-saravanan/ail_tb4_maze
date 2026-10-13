import json
from pathlib import Path
from ail_tb4_maze.scoring import Scoring

C=json.loads((Path(__file__).resolve().parents[1]/'config/challenge.json').read_text())
def trial():
 s=Scoring(C);s.start(20,(9,-2,0));return s

def test_complete_crossing_only():
 s=trial();s.update(21,10.1,-2);assert s.status=='running'
 s.update(22,10.3,-2);assert s.status=='success';assert abs(s.distance-1.3)<1e-9

def test_outside_aperture_fails():
 s=trial();s.update(21,10.5,0);assert s.status=='failure'

def test_timeout_sim_time():
 s=trial();s.tick(319.99);assert s.status=='running'
 s.tick(320);assert s.status=='timeout';assert s.elapsed==300

def test_time_reversal_fails():
 s=trial();s.tick(19);assert s.reason=='simulation_time_reversed'

def test_contact_episodes_and_multiple_walls():
 s=trial();s.contact(20,'wall1');s.contact(20.1,'wall1');assert s.collisions==1
 s.contact(20.5,'wall1');s.contact(20.5,'wall2');assert s.collisions==3

def test_terminal_result_immutable():
 s=trial();s.tick(320);before=s.result();s.update(321,10.5,-2);s.contact(321,'wall');assert s.result()==before

def test_spawn_outside_does_not_win():
 s=Scoring(C);s.start(0,(11,-2,0));s.update(1,11,-2);assert s.status=='failure'

def test_return_inside_cancels_crossing():
 s=trial();s.update(21,10.1,-2);s.update(22,9.9,-2);assert not s.crossed

def test_fallen_robot_fails():
 s=trial();s.update(21,9,-2,1);assert s.reason=='robot_fell_or_lifted'
