"""Privileged scoring state machine; no ROS dependency."""
import math

class Scoring:
    def __init__(self, config):
        self.c = config
        self.status = 'waiting'
        self.reason = None
        self.started = None
        self.elapsed = 0.0
        self.distance = 0.0
        self.collisions = 0
        self.last = None
        self.contact_times = {}
        self.crossed = False
        self.pose_samples = 0

    def start(self, now, pose):
        if self.status != 'waiting':
            return False
        self.status = 'running'
        self.started = now
        self.last = (now, *pose)
        return True

    def finish(self, status, reason):
        self.status, self.reason = status, reason

    def tick(self, now):
        if self.status == 'running':
            if now < self.started:
                self.finish('failure', 'simulation_time_reversed')
            else:
                self.elapsed = now - self.started
                if self.elapsed >= self.c['time_limit']:
                    self.finish('timeout', 'simulation_time_limit')

    def update(self, now, x, y, z=0.0):
        if self.status != 'running':
            return
        self.tick(now)
        if self.status != 'running':
            return
        old_t, old_x, old_y, old_z = self.last
        if now < old_t:
            self.finish('failure', 'simulation_time_reversed')
            return
        self.distance += math.hypot(x-old_x, y-old_y)
        self.pose_samples += 1
        exit = self.c['exit']; r = self.c['footprint_radius']
        # Require an observed outward crossing within the eroded aperture.
        if old_x <= exit['x'] < x:
            u = (exit['x']-old_x)/(x-old_x)
            crossing_y = old_y + u*(y-old_y)
            self.crossed = exit['y_min']+r <= crossing_y <= exit['y_max']-r
        if x < exit['x']:
            self.crossed = False
        if self.crossed and x-r > exit['x'] and exit['y_min']+r <= y <= exit['y_max']-r:
            self.finish('success', 'fully_crossed_exit')
        elif abs(z) > 0.5:
            self.finish('failure', 'robot_fell_or_lifted')
        elif x > exit['x']+r and not self.crossed:
            self.finish('failure', 'escaped_outside_exit')
        elif x < self.c['bounds'][0]-r or y < self.c['bounds'][2]-r or y > self.c['bounds'][3]+r:
            self.finish('failure', 'escaped_outside_exit')
        self.last = (now, x, y, z)

    def contact(self, now, pair):
        if self.status != 'running':
            return
        previous = self.contact_times.get(pair, -math.inf)
        if now-previous > self.c['contact_gap']:
            self.collisions += 1
        self.contact_times[pair] = now

    def result(self):
        return dict(schema_version=1,status=self.status,reason=self.reason,
                    elapsed_sim_seconds=self.elapsed,distance_meters=self.distance,
                    collision_episodes=self.collisions,pose_samples=self.pose_samples,
                    time_limit_sim_seconds=self.c['time_limit'])
