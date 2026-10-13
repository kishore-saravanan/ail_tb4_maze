# Navigation agent contract

This directory is the only project material intended for the future navigation agent. Start that agent in a **fresh chat**, with this directory as its project context. Do not supply infrastructure documentation, tests, results, images, configuration, world files, or the prior setup conversation.

## Task

Drive a TurtleBot 4 standard robot out of an initially unknown maze. Discover both the layout and exit using only the observations below. Trial duration is 300 seconds of simulation time. There is no preloaded map, navigation stack, waypoint sequence, or solving controller supplied.

## ROS interface

Source `/opt/ros/jazzy/setup.bash`. Set `ROS_DOMAIN_ID=62`. Set `use_sim_time:=true` on every ROS node you create. The operator launches and starts the trial separately.

| Topic | Type | Direction | Meaning |
|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | Read | 2D LiDAR; 640 rays, full circle, nominal 62 Hz, range 0.164–12 m |
| `/odom` | `nav_msgs/msg/Odometry` | Read | Wheel encoder integration in local `odom`; initial pose near zero, can drift/slip |
| `/imu` | `sensor_msgs/msg/Imu` | Read | IMU measurements in `imu_link`; nominal 62 Hz |
| `/tf` | `tf2_msgs/msg/TFMessage` | Read | Wheel odometry transform `odom → base_link` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Read | Only fixed transforms connecting the base, LiDAR and IMU |
| `/clock` | `rosgraph_msgs/msg/Clock` | Read | Simulation time, including pauses |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Write | Differential drive: `twist.linear.x` metres/s, `twist.angular.z` radians/s |

Subscribe to scan, odometry, IMU and clock with sensor-data QoS / best effort. Subscribe to static TF with reliable/transient-local QoS so late subscribers receive the complete fixed transform set. LiDAR message frame is `turtlebot4/rplidar_link/rplidar`. Use its TF to interpret angles; do not assume its axes match the base axes.

Publish velocity commands at 10–20 Hz. The gateway caps forward/reverse speed to 0.30 m/s and angular speed to 1.50 rad/s. Other components are ignored. Invalid/nonfinite commands are rejected. The gateway stamps commands with current simulation time. The wheel controller brakes after 0.5 simulated seconds without a valid command; a separate wall-time watchdog brakes after a 0.5-second publisher gap. Publish zero velocity on shutdown.

Allowed transform edges:

```
odom → base_link → base_footprint
                 → imu_link
                 → shell_link → rplidar_link → turtlebot4/rplidar_link/rplidar
```

`/parameter_events` and `/rosout` are ordinary ROS metadata. No maze information is supplied through them by the gateway.

## Restrictions

Do not inspect files outside this navigation project to learn the maze. Do not change ROS domains or use Gazebo Transport, simulator services, ground-truth data, operator windows/screenshots, camera data, referee parameters/results, or infrastructure files. Do not query the operator for the exit position. Build your own map only from permitted measurements if your future solution needs one.

The interface is logically isolated, **not a security sandbox**. A process running as the operator's OS user can deliberately bypass the rules. Separate machine/container permissions and authenticated DDS policies would be needed for an adversarial competition. Follow the interface and challenge restrictions; the operator must enforce separate project context and review the solution's subscriptions and file accesses.

## Operator responsibilities

The operator verifies all sensors and transforms are available, starts the trial, and resets the entire infrastructure between attempts. The controller never receives privileged success-region details or reset services. A simulation restart resets clock and wheel odometry; restart the navigation node between trials too. Do not interpret a ROS discovery delay as an empty scan.
