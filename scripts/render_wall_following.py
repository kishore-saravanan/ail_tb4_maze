#!/usr/bin/python3
"""Render a portrait motion-graphics replay from actual controller telemetry."""
import argparse
import json
from pathlib import Path
import shutil
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.collections import LineCollection
from matplotlib.patches import Circle, FancyArrowPatch


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('telemetry',type=Path);parser.add_argument('--result',type=Path)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--duration',type=float,default=35)
    parser.add_argument('--fps',type=int,default=12)
    args=parser.parse_args()
    events=[json.loads(line) for line in args.telemetry.read_text().splitlines() if line.strip()]
    if not events or 'lidar_points' not in events[0]:parser.error('No wall-following telemetry in this file')
    if args.duration<10:parser.error('Use at least 10 seconds for a readable replay')
    ffmpeg=shutil.which('ffmpeg')
    if not ffmpeg:
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'.tools/video'))
        try:
            import imageio_ffmpeg
            ffmpeg=imageio_ffmpeg.get_ffmpeg_exe()
        except ImportError:parser.error('Install FFmpeg or the optional requirements-video.txt dependencies')
    matplotlib.rcParams['animation.ffmpeg_path']=ffmpeg
    result=json.loads(args.result.read_text()) if args.result else {}
    result=result.get('result',result)
    times=np.array([e['sim_seconds'] for e in events]);span=times[-1]-times[0]
    replay_duration=args.duration-5;speed=span/replay_duration
    bg='#0a151e';teal='#58e3b8';amber='#ffd06a';white='#edf3ee'
    fig=plt.figure(figsize=(6,10.6667),dpi=120,facecolor=bg)
    fig.text(.07,.945,'CAN MY ROBOT\nESCAPE THE MAZE?',color=white,fontsize=25,weight='bold',va='top')
    fig.text(.07,.835,'ONE RULE: KEEP THE WALL ON THE RIGHT',color=amber,fontsize=10,weight='bold')
    fig.text(.07,.805,'Actual LiDAR replay • reactive wall following',color='#a5bbc4',fontsize=9)
    ax=fig.add_axes([.07,.24,.86,.52],facecolor=bg);ax.set_aspect('equal');ax.set_xlim(-2.2,2.2);ax.set_ylim(-1.25,3.05)
    ax.set_xticks([]);ax.set_yticks([])
    for spine in ax.spines.values():spine.set_visible(False)
    ax.axvline(.45,ls='--',color=amber,alpha=.25,lw=1)
    rays=LineCollection([],colors=teal,linewidths=.5,alpha=.18);ax.add_collection(rays)
    dots=ax.scatter([],[],s=10,c=white,alpha=.8)
    side_line,=ax.plot([],[],color=amber,lw=2)
    front_line,=ax.plot([],[],color='#f08a7e',lw=2)
    ax.add_patch(Circle((0,0),.23,color=teal,zorder=5))
    heading=FancyArrowPatch((0,-.04),(0,.65),color=teal,arrowstyle='-|>',mutation_scale=22,lw=3,zorder=6);ax.add_patch(heading)
    sensor_label=ax.text(-2.05,2.8,'FRONT',color='#f08a7e',fontsize=9)
    state=fig.text(.07,.205,'',color=amber,fontsize=13,weight='bold')
    measures=fig.text(.07,.165,'',color=white,fontsize=11)
    clock=fig.text(.07,.12,'',color='#a5bbc4',fontsize=9)
    footer=fig.text(.07,.065,'No preloaded map. No exit coordinates.',color=teal,fontsize=10)
    overlay=fig.text(.5,.52,'',ha='center',va='center',color=white,fontsize=22,weight='bold',bbox=dict(facecolor=bg,edgecolor=teal,boxstyle='round,pad=.8'),zorder=20)
    overlay.set_visible(False)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    writer=FFMpegWriter(fps=args.fps,codec='libx264',bitrate=3500,extra_args=['-pix_fmt','yuv420p'])
    names={'back_up':'STUCK? BACK UP SAFELY','recovery_turn':'RECOVER AND TRY AGAIN','advance_corner':'CLEAR THE CORNER, THEN TURN','acquire_turn_right':'FIND THE RIGHT WALL','approach_wall':'APPROACH THE WALL','align_wall':'KEEP IT ON THE RIGHT','seek_wall':'FIND THE RIGHT WALL','follow_wall':'FOLLOW THE RIGHT WALL','turn_left':'FRONT BLOCKED → TURN LEFT','turn_right':'RIGHT OPENING → TURN RIGHT','obstacle_stop':'TOO CLOSE → STOP','sensor_stop':'SENSOR LOST → STOP'}
    with writer.saving(fig,str(args.output),dpi=120):
        for frame in range(int(args.duration*args.fps)):
            t=frame/args.fps;elapsed=max(0.,min(t-2,replay_duration));timestamp=times[0]+elapsed*speed
            index=min(len(events)-1,int(np.searchsorted(times,timestamp)))
            event=events[index];points=np.asarray(event['lidar_points'])
            xy=np.column_stack((-points[:,1],points[:,0]))
            dots.set_offsets(xy);rays.set_segments([[(0.,-.04),tuple(p)] for p in xy[::5]])
            side=event.get('side_m');front=event.get('front_m')
            side_line.set_data([0,min(side,2.)] if side is not None else [],[0,0] if side is not None else [])
            front_line.set_data([0,0] if front is not None else [],[0,min(front,2.5)] if front is not None else [])
            state.set_text(names.get(event['state'],event['state'].upper()))
            measures.set_text('Right: '+(f'{side:.2f} m' if side is not None else 'open')+'    Front: '+(f'{front:.2f} m' if front is not None else 'open'))
            clock.set_text(f'Sensor replay: {timestamp-times[0]:.1f} s    •    {speed:.1f}× simulated-time playback')
            overlay.set_visible(t<2 or t>=args.duration-3)
            if t<2:overlay.set_text('NO MAP.\n300 SIMULATED SECONDS.\nCAN IT ESCAPE?')
            elif t>=args.duration-3:
                status=result.get('status','unscored')
                label={'success':'ESCAPED','timeout':'TIME RAN OUT','failure':'TRIAL ENDED'}.get(status,'REPLAY COMPLETE')
                detail=f"{result.get('elapsed_sim_seconds',0):.1f} simulated seconds\n{result.get('collision_episodes',0)} collision episodes" if result else 'Outcome not supplied'
                overlay.set_text(label+'\n\n'+detail)
            writer.grab_frame()
    plt.close(fig)
    print(args.output)

if __name__=='__main__':main()
