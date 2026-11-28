# Wheel odometry

`dog_toy_finder_odometry/odometry_node.py` converts the Pico's wheel feedback
into `nav_msgs/msg/Odometry` for the EKF.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | subscribe |
| `/odom_reset` | `std_msgs/msg/Empty` | subscribe |
| `/wheel/odom` | `nav_msgs/msg/Odometry` | publish |

The firmware publishes `left_wheel_joint` and `right_wheel_joint` positions in
radians. The node differentiates them into wheel distances, applies the
differential-drive forward kinematics, and reports forward speed and turn rate
in the twist. It does not publish TF by default because the EKF owns
`odom -> base_link`.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `left_joint_name` / `right_joint_name` | firmware joint names | `JointState` entries to read |
| `wheel_radius` | 0.0325 | drive-wheel radius, metres |
| `wheel_separation` | 0.133 | wheel contact spacing, metres |
| `invert_left` / `invert_right` | false | flip a side's counting direction |
| `publish_tf` | false | publish `odom -> base_link` directly |
| `linear_speed_std` / `turn_rate_std` | 0.03 / 0.10 | covariance for the EKF |
| `velocity_window` | 0.2 | seconds used to average speed |
| `max_joint_jump_rad` | 31.4159 | treat larger jumps as a Pico restart |
| `odom_topic` / `reset_topic` | `/wheel/odom` / `/odom_reset` | topic names |

## Run

```bash
ros2 run dog_toy_finder_odometry odometry_node
ros2 topic echo /wheel/odom --once
ros2 topic pub --once /odom_reset std_msgs/msg/Empty "{}"
```

Normally started by `dog_toy_finder_bringup`.
