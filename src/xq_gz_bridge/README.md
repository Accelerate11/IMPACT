# xq_gz_bridge

`xq_gz_bridge` is the project-local C++ bridge between Gazebo Harmonic
(`gz-msgs10`, `gz-transport13`) and ROS 2 Humble. It does not use or modify the
existing `uav_slam_sim` bridge in `/home/accelerate/cuadc_ws`.

## Contract

| Direction | Gazebo topic and type | ROS 2 topic and type |
|---|---|---|
| Gazebo -> ROS | `/clock`, `gz.msgs.Clock` | `/clock`, `rosgraph_msgs/msg/Clock` |
| Gazebo -> ROS | `/xq/lidar/points`, `gz.msgs.PointCloudPacked` | `/xq/agent_01/sensors/lidar/points`, `sensor_msgs/msg/PointCloud2` |
| Gazebo -> ROS | `/xq/imu`, `gz.msgs.IMU` | `/xq/agent_01/sensors/imu`, `sensor_msgs/msg/Imu` |
| Gazebo -> ROS | `/model/xq_agent_01/odometry`, `gz.msgs.Odometry` | `/xq/eval/agent_01/ground_truth`, `nav_msgs/msg/Odometry` |
| ROS -> Gazebo | `/model/xq_agent_01/cmd_vel`, `gz.msgs.Twist` | `/xq/agent_01/cmd_vel`, `geometry_msgs/msg/Twist` |

All topics and output frame IDs are ROS parameters. Defaults are in
`config/bridge.yaml`.

LiDAR, IMU, and ground-truth messages keep the Gazebo sample timestamp. An
unstamped input is dropped with a throttled warning; the bridge never replaces
a missing sample time with its receive time.

The local LiDAR publisher uses reliable keep-last-20 QoS. Network-loss tests
remain confined to `xq_network_relay`; dropping frames on the on-board
sensor-to-estimator link would conflate DDS transport loss with estimator
performance.

Ground truth can alternatively consume Gazebo's world pose stream. Set:

```yaml
ground_truth_source_type: pose_v
gz_ground_truth_topic: /world/xq_indoor_office/pose/info
ground_truth_entity_name: xq_agent_01
```

The `Pose_V` mode publishes only the exact requested model pose; it does not
mistake a similarly named link for the model.

## Truth isolation rule

`/xq/eval/agent_01/ground_truth` is evaluation-only. Metric and recording
nodes may subscribe to it. Localization, mapping, planning, control, safety,
and mission nodes must never subscribe or remap it into their inputs. This
enforces the plan requirement that truth data does not enter navigation or
control. Set `publish_ground_truth:=false` for algorithm-only runs that do not
need scoring.

## Build and test

Use only the dedicated workspace; do not source an existing project overlay:

```bash
cd /home/accelerate/xuanqiong_x1_sim_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select xq_gz_bridge --symlink-install
colcon test --packages-select xq_gz_bridge --event-handlers console_direct+
colcon test-result --verbose
```

Run with the installed configuration:

```bash
source install/setup.bash
ros2 run xq_gz_bridge xq_gz_bridge_node --ros-args \
  --params-file "$(ros2 pkg prefix xq_gz_bridge)/share/xq_gz_bridge/config/bridge.yaml"
```

For every run, the parent workspace must set a project-only `ROS_DOMAIN_ID`
and unique `GZ_PARTITION`. Never use global process termination or host-wide
network emulation to manage this bridge.

## Conversion tests

`test/test_conversions.cpp` checks timestamp normalization, packed point-cloud
field/data preservation, IMU conversion, both ground-truth forms, and all six
velocity axes. `test/test_shutdown_race.sh` repeatedly sends Gazebo clock data
while stopping the node and rejects any non-zero exit, abort, or invalid RMW
publisher error. Neither test requires Gazebo GUI or a running simulation.
