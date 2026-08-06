# cmd_vel scripts for ROS 2

ROS 2 port of [cmd_vel-scripts](https://github.com/abdu7rahman/cmd_vel-scripts) —
teleop and waypoint-following nodes that drive a four-wheel base over `/cmd_vel`,
plus the Arduino firmware that receives it.

`rosserial` has no ROS 2 successor, so the boards move to **micro-ROS**.

## Layout

```
cmd_vel_ros2/                 ament_python package (host side)
  config/teleop_params.yaml   every node's parameters in one file
  launch/                     joy teleop and waypoint nav

arduino/
  cmd_controller_microros/       /cmd_vel -> four motors
  ps4_receive_microros/          /cmd_vel_pwm -> four motors
  mecanum_odom_cmd_microros/     /cmd_vel in, /odom out
  standalone/                    the sketches that never used ROS, unchanged
```

## Where each ROS 1 script went

| ROS 1 | ROS 2 | Notes |
|---|---|---|
| `scripts/ps.py` | `joy_twist` | axes → Twist |
| `scripts/cmd2.py` | `joy_twist` | same node with `scale_linear:=1.0`, `scale_angular:=1.0` |
| `scripts/cmd.py` | `joy_pwm_array` | publishes on `/cmd_vel_pwm`, not `/cmd_vel` |
| `scripts/psfull.py` | `joy_pwm_debug` | prints only, publishes nothing |
| `scripts/psarduino.py` | `joy_pwm_debug` | was byte-for-byte the same behaviour as psfull.py |
| `scripts/keyboard.py` | `keyboard_teleop` | same key bindings |
| `scripts/simple_planner.py` | `waypoint_navigator` | with `loop_goals:=false` |
| `scripts/nav_points.py` | `waypoint_navigator` | with `loop_goals:=true` |
| `scripts/coordinates.py` | `odom_echo` | |
| `scripts/swerve_controller.py` | `swerve_controller` | same node as in swerve_ros-ros2 |
| `arduino scripts/cmd_controller` | `arduino/cmd_controller_microros` | |
| `arduino scripts/ps4_receive_ros` | `arduino/ps4_receive_microros` | |
| `arduino scripts/odompub_cdmsub` | `arduino/mecanum_odom_cmd_microros` | |
| `arduino scripts/ros_controller___odom_publisher` | `arduino/mecanum_odom_cmd_microros` | was the same sketch as odompub_cdmsub |
| `arduino scripts/base_locomoton` | `arduino/standalone/` | no ROS in it — PS2X library direct, carried over unchanged |
| `arduino scripts/basic_locomotion_ps4` | `arduino/standalone/` | same |
| `arduino scripts/simple_odometery` | `arduino/standalone/` | prints to Serial, no ROS |

Five separate joy scripts collapse to three nodes because `cmd2.py` differed from
`ps.py` only in its scale factors, and `psarduino.py` was a copy of `psfull.py`.
Both are reachable through parameters — nothing was dropped.

## Build

```bash
mkdir -p ~/ros2_ws/src && cp -r cmd_vel_ros2 ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select cmd_vel_ros2
source install/setup.bash
sudo apt install ros-$ROS_DISTRO-micro-ros-agent ros-$ROS_DISTRO-joy
```

## Run

Gamepad teleop:

```bash
ros2 launch cmd_vel_ros2 joy_teleop.launch.py serial_port:=/dev/ttyACM0
```

Waypoint following:

```bash
ros2 launch cmd_vel_ros2 waypoint_nav.launch.py
```

Keyboard, no hardware:

```bash
ros2 run cmd_vel_ros2 keyboard_teleop
```

Check your axis mapping before wiring anything up:

```bash
ros2 run joy joy_node
ros2 run cmd_vel_ros2 joy_pwm_debug
```

## Topics

| Topic | Type | Notes |
|---|---|---|
| `/joy` | `sensor_msgs/msg/Joy` | from `joy_node` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | to `cmd_controller_microros` |
| `/cmd_vel_pwm` | `std_msgs/msg/Int64MultiArray` | to `ps4_receive_microros` |
| `/odom` | `nav_msgs/msg/Odometry` | from `mecanum_odom_cmd_microros` |
| `/finalvel` | `std_msgs/msg/Float32MultiArray` | from `swerve_controller` |

## What changed from ROS 1

| ROS 1 | ROS 2 |
|---|---|
| `rospy.init_node` | `rclpy.node.Node` subclass |
| `rospy.get_param("~x")` | `declare_parameter` + YAML |
| `rospy.Rate` + `while not is_shutdown()` | `create_timer` + `rclpy.spin` |
| `tf.transformations.euler_from_quaternion` | inline conversion (no `tf` in ROS 2) |
| `roslib.load_manifest` | gone |
| `publisher.get_num_connections()` | `publisher.get_subscription_count()` |
| `rosserial` | micro-ROS + `micro_ros_agent` |

### Fixes carried into the port

- **`/cmd_vel` carried the wrong type.** `cmd.py` and `ps4_receive_ros.ino` used
  `Int64MultiArray` on a topic that by convention is `geometry_msgs/Twist`. Any
  other node on the graph subscribing to `/cmd_vel` would fail to deserialise.
  That path is `/cmd_vel_pwm` now.
- **The base could only move on one axis at a time.** Both drive sketches ran an
  `if/else` chain on `linear.x`, then `linear.y`, then `angular.z`, so a
  command to drive forward *and* turn silently dropped the turn. The port mixes
  all three into per-wheel commands and normalises them.
- **"Stop" was a brake at full power.** `M1(1, 1, 255)` sets both H-bridge
  direction inputs high with full PWM — on most drivers that shorts the motor
  terminals rather than stopping. It ran on every stop and at boot.
- **Odometry twist held displacements, not velocities.** `odom.twist.twist` was
  filled with per-step distances; `nav_msgs/Odometry` specifies m/s and rad/s.
  Divided by the timestep now.
- **Odometry ran at ~1 kHz off `delay(1)`.** That saturated the serial link and
  made each integration step's dt whatever the loop happened to take. It is on a
  fixed 20 Hz timer.
- **Waypoint heading errors were never wrapped.** A robot at +170° heading for a
  −170° goal computed a 340° error and turned the long way round. Wrapped to
  [−π, π].
- **`nav_points.py` could only turn one direction.** It hard-coded
  `angular.z = 0.15` in the approach branch regardless of which way the goal
  was, so it converged only by overshooting all the way around.
  `simple_planner.py` had already fixed this; the port keeps the fix.
- **`time.sleep(1)` blocked the control loop.** Settling at a goal froze the
  whole node, odometry callbacks included. It is a deadline check now.
- **Publishers were constructed inside callbacks.** `cmd.py` built a new
  `rospy.Publisher` per joy message and published up to four times; discovery
  means the first messages after construction are dropped.
- **`cmd.py`'s stop-on-exit block was unreachable.** It sat inside
  `while rospy.is_shutdown():` after a loop that only exits when that condition
  is already true — but by then the node is shutting down and cannot publish.
  It runs in a `finally` now.
- **Joy axes were indexed unguarded.** `axes[3]` and `buttons[11]` raised
  `IndexError` on any pad reporting fewer. Reads are bounds-checked.
- **No command timeout on any board.** If the host died, the motors held their
  last throttle. All three sketches stop after 500 ms of silence.
- **`psarduino.py` would not parse.** It mixed tabs and spaces in the
  `while not rospy.is_shutdown()` body, and had a stray `rate.sleep()` at module
  scope after ~70 blank lines. Both are gone.

Odometry integration also now uses the midpoint heading over each step instead of
the starting heading, which reduces drift on curved paths. The mecanum mixing
matrix and the encoder decode tables are unchanged from the original.
