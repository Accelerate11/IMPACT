# xq_sim_bringup

This package launches the Xuanqiong-X1 Gazebo Harmonic software-in-the-loop
stack without reading or writing another project's worlds or models.

The launch file starts the exclusive `xq_indoor_office.sdf` world, the custom
Harmonic bridge, the autonomy proxy stack, the metrics collector, and
optionally the deterministic fault injector. Every ROS node uses simulation
time. Ground truth is published only below `/xq/eval/` and is consumed only by
the metrics node.

Use the repository scripts from WSL:

```bash
bash scripts/build_isolated.sh
bash scripts/run_smoke.sh --duration 30
bash scripts/run_smoke.sh --with-faults
```

The fault run defaults to 120 wall-clock seconds so the eight non-overlapping
F1--F8 windows (ending at 35 s of simulation time) can complete under WSL
software rendering.

The runner assigns a unique `ROS_DOMAIN_ID` and `GZ_PARTITION`, restricts DDS
to localhost, overrides `GZ_SIM_RESOURCE_PATH` with this package's asset paths,
and stops only its own process group. It records SHA256 snapshots of the known
pre-existing map/model directories before and after every run.

`ROS_LOCALHOST_ONLY=1` deliberately limits this runner to one WSL host. A pass
therefore validates single-machine SIL only; it is not evidence that DDS works
between multiple computers or flight vehicles.

For a direct launch, source only ROS Humble and this workspace, then pass an
explicit result directory:

```bash
source /opt/ros/humble/setup.bash
source xq_install/setup.bash
ros2 launch xq_sim_bringup xq_sil.launch.py \
  headless:=true inject_faults:=false run_dir:="$PWD/runs/manual"
```

Notable arguments are `headless`, `inject_faults`, `run_dir`, `seed`,
`ros_domain_id`, `gz_partition`, and the four `start_*` switches. The default
world and parameter files are resolved from their installed ROS packages, not
from absolute paths into another workspace.
