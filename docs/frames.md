# Coordinate frames

All frames follow [REP-103](https://www.ros.org/reps/rep-0103.html) (x forward,
y left, z up; optical frames z forward, x right, y down) and
[REP-105](https://www.ros.org/reps/rep-0105.html) (`map` → `odom` → base).

```
map                        rtabmap        (only when slam:=true)
└── odom                   MecanumDrive via ros_gz_bridge (/tf)
    └── base_footprint
        └── base_link                  robot_state_publisher (/tf_static)
            ├── lidar_link             robot_state_publisher (/tf_static)
            ├── camera_link            robot_state_publisher (/tf_static)
            │   └── camera_optical_frame
            ├── imu_link               robot_state_publisher (/tf_static)
            ├── front_left_wheel_link  robot_state_publisher (/tf, from /joint_states)
            ├── front_right_wheel_link robot_state_publisher (/tf, from /joint_states)
            ├── back_left_wheel_link   robot_state_publisher (/tf, from /joint_states)
            └── back_right_wheel_link  robot_state_publisher (/tf, from /joint_states)

world                      Gazebo; ground truth only, never on /tf
```

## Frame table

| Frame | Parent | Publisher | Pose in parent | Purpose |
|---|---|---|---|---|
| `map` | – | – | – | Globally consistent SLAM frame. Origin = robot pose when RTAB-Map started. |
| `odom` | `map` | `rtabmap` (`publish_tf`) | SLAM correction | Continuous, drifting odometry frame. Origin = spawn pose. |
| `base_footprint` | `odom` | Gazebo MecanumDrive → bridge `/wsl/tf` → `/tf` | wheel odometry | Robot pose projected on the ground plane. Rotation centre (midpoint of the four wheels). |
| `base_link` | `base_footprint` | robot_state_publisher | z = 0.127 m | Body frame at axle height (wheel radius). Origin of the chassis mesh. |
| `lidar_link` | `base_link` | robot_state_publisher | (0, 0, 0.533) m → 0.66 m above ground | 3D LiDAR origin, on the body centre, 0.149 m above the chassis top. `/lidar/points` is expressed here. |
| `camera_link` | `base_link` | robot_state_publisher | (0.3755, 0, 0.29) m → 0.417 m above ground | Camera body on the front face, REP-103 orientation (x along the optical axis). Gazebo renders along its +x. |
| `camera_optical_frame` | `camera_link` | robot_state_publisher | rpy (−π/2, 0, −π/2) | Image and `CameraInfo` frame: z forward, x right, y down. |
| `imu_link` | `base_link` | robot_state_publisher | (0, 0, 0.161) m | IMU at the centre of the chassis box, axes aligned with `base_link`. |
| `{front,back}_{left,right}_wheel_link` | `base_link` | robot_state_publisher | (±0.2225, ±0.2045, 0) m | Continuous joints about +y; positive rotation rolls the wheel forward. |
| `world` | – | Gazebo | – | Simulator frame. Appears only as `header.frame_id` of `/ground_truth/odom`. |

## Single authority per transform

| Transform | Only authority | Why |
|---|---|---|
| `map → odom` | `rtabmap` | SLAM is the only component with a global estimate. |
| `odom → base_footprint` | MecanumDrive (bridged `/wsl/tf`) | The only odometry source. Nothing else bridges `/tf`. |
| `base_* → sensors` | robot_state_publisher | Generated from the same URDF Gazebo uses, so simulated sensor poses and TF cannot disagree. |

Ground truth is deliberately **not** a TF frame: `OdometryPublisher` writes its TF to
`/wsl/ground_truth/tf`, which is never bridged. Only `/ground_truth/odom` reaches ROS, and
only when `ground_truth:=true`.

## Verification

Automated:

- `wsl_description/test/test_urdf.py` checks the tree, the root, the optical rotation
  (optical z = base x, optical x = base −y, optical y = base −z) and the LiDAR height.
- `wsl_sim/test/test_system_smoke.launch.py` checks at runtime that `odom → camera_optical_frame`
  resolves, that `base_footprint → lidar_link` is (0, 0, 0.66) m, that sensor messages carry
  `lidar_link`, `camera_optical_frame` and `imu_link`, that the lowest LiDAR ring hits the floor
  0.66 m below `lidar_link`, that the LiDAR does not see the robot, and that without SLAM nothing
  publishes a transform from `map`.

Manual:

```bash
ros2 run tf2_tools view_frames
```

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```
