# Official EGO-Planner ROS 2 snapshot

- Upstream: `https://github.com/ZJU-FAST-Lab/ego-planner-swarm`
- Branch: `ros2_version`
- Commit: `23a8d5a191711dd65633df689b0b37ac07718416`
- Snapshot date: 2026-08-23
- Upstream license: GPL-3.0 (preserved in `LICENSE`)

Only the dependency closure required by the single-vehicle planner is vendored:
`plan_env`, `path_searching`, `bspline_opt`, `traj_utils`, `ego_planner`, and
`quadrotor_msgs`.  The upstream simulator, swarm, dynamic-object, random-map,
and transport-bridge packages are intentionally excluded from IMPACT
`BASELINE_V1`.

The upstream package manifests contained placeholder license strings even
though the repository root declares GPLv3.  This snapshot replaces only those
manifest placeholders with the SPDX-compatible `GPL-3.0-only` declaration.

## Project-local ROS 2 integration patches

- EGO FSM, manager, and trajectory server timestamps use the owning node's
  ROS clock, keeping `use_sim_time` coherent with Gazebo.
- The manual-target subscription is parameterized as `fsm/target_topic`
  (the upstream default is retained), so P5 can prove goals come only from the
  autonomous Frontier selector.
- Manual targets retain the requested Z coordinate; upstream forced 1.0 m.
- Point-cloud obstacles are inflated by the configured vehicle radius in Z as
  well as XY, preventing centre trajectories from clipping obstacle tops.
