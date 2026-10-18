#!/usr/bin/python3
"""Reproducible, ground-truth-scored trial; controller receives sensors only."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from gz.transport13 import Node as GzNode,NodeOptions
from gz.msgs10.physics_pb2 import Physics
from gz.msgs10.boolean_pb2 import Boolean
from ail_tb4_maze.paths import PACKAGE_ROOT,PACKAGE_SHARE,RESULTS
from ail_tb4_maze.world_factory import generate


def stop(proc):
    if proc is None or proc.poll() is not None:return
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=6)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid,signal.SIGTERM)
        try:proc.wait(timeout=6)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--preset',choices=['official','random'],default='official')
    p.add_argument('--size',type=int,default=20);p.add_argument('--cells',type=int,default=10);p.add_argument('--seed',type=int,default=42)
    p.add_argument('--rtf',type=float,default=1.,help='Simulation speed target; trial remains 300 simulated seconds')
    p.add_argument('--visible',action='store_true');p.add_argument('--infra-domain',type=int,default=63);p.add_argument('--nav-domain',type=int,default=64)
    args=p.parse_args()
    if args.infra_domain==args.nav_domain:p.error('Infrastructure and navigation domains must differ')
    output=RESULTS/('experiment_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(time.monotonic_ns()));output.mkdir(parents=True)
    code_sha256={name:hashlib.sha256((PACKAGE_ROOT/'navigation_interface'/name).read_bytes()).hexdigest() for name in ['controller.py','wall_follower.py']}
    scene=output/'scene';c=generate(PACKAGE_SHARE,scene,args.size,args.cells,args.seed,args.preset)
    partition='maze_experiment_'+str(os.getpid());env=dict(os.environ,ROS_DOMAIN_ID=str(args.infra_domain),GZ_PARTITION=partition)
    rclpy.init(domain_id=args.infra_domain);node=Node('experiment_operator')
    sim=solver=None
    def call(path):
        client=node.create_client(Trigger,path)
        if not client.wait_for_service(timeout_sec=10):raise RuntimeError('Service unavailable: '+path)
        future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        if not future.done() or not future.result().success:raise RuntimeError('Service failed: '+path)
    def status():
        try:return json.loads((output/'status.json').read_text())
        except (OSError,ValueError):return {}
    try:
        # Distinct DDS domains AND partition prevent modifying an operator's GUI.
        for _ in range(10):rclpy.spin_once(node,timeout_sec=.1)
        if any(name=='/referee/start' for name,_ in node.get_service_names_and_types()):raise RuntimeError('Infrastructure domain is already in use; choose another domain pair')
        with (output/'simulation.log').open('w') as log:
            s=c['start']
            sim=subprocess.Popen(['ros2','launch','ail_tb4_maze','challenge.launch.py',f'headless:={str(not args.visible).lower()}','rviz:=false',f'infrastructure_domain:={args.infra_domain}',f'navigation_domain:={args.nav_domain}',f'gazebo_partition:={partition}',f'world_file:={scene}/world.sdf',f'config_file:={scene}/challenge.json',f'results_dir:={output}',f'x:={s["x"]}',f'y:={s["y"]}'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            end=time.monotonic()+75
            while time.monotonic()<end and not status().get('pose_ready'):
                if sim.poll() is not None:raise RuntimeError('Simulator stopped; inspect simulation.log')
                time.sleep(.2)
            if not status().get('pose_ready'):raise RuntimeError('Simulator pose never became ready')
            options=NodeOptions();options.partition=partition;gz=GzNode(options)
            ok,response=gz.request('/world/maze_escape/set_physics',Physics(real_time_factor=args.rtf,max_step_size=.001),Physics,Boolean,7000)
            if not ok or not response.data:raise RuntimeError('Could not set simulation speed')
            call('/referee/start')
            with (output/'controller.log').open('w') as control_log:
                solver=subprocess.Popen(['/usr/bin/python3',str(PACKAGE_ROOT/'navigation_interface/controller.py'),'--ros-args','-p','use_sim_time:=true','-p',f'recording_path:={output}/exploration.jsonl'],env=dict(env,ROS_DOMAIN_ID=str(args.nav_domain)),stdout=control_log,stderr=subprocess.STDOUT,start_new_session=True)
                end=time.monotonic()+900;previous=-30
                while time.monotonic()<end:
                    s=status()
                    if s.get('status') in ('success','failure','timeout'):break
                    if solver.poll() is not None:raise RuntimeError('Solver stopped; inspect controller.log')
                    if sim.poll() is not None:raise RuntimeError('Simulator stopped; inspect simulation.log')
                    if s.get('elapsed_sim_seconds',0)-previous>=30:
                        print(json.dumps({key:s.get(key) for key in ['status','elapsed_sim_seconds','collision_episodes','distance_meters']}),flush=True);previous=s.get('elapsed_sim_seconds',0)
                    time.sleep(.5)
                else:raise RuntimeError('Trial exceeded wall-clock watchdog')
                report=dict(code_sha256=code_sha256,parameters=vars(args),algorithm='LiDAR right-hand wall following with swept-footprint collision checks',controller_has_maze_or_exit=False,result=s,telemetry='exploration.jsonl')
                (output/'experiment.json').write_text(json.dumps(report,indent=2)+'\n')
                print('RESULT '+json.dumps(report),flush=True);print('Evidence: '+str(output),flush=True)
    finally:
        stop(solver);stop(sim);node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
