# FAST-LIVO2 research integration

Algorithm source: https://github.com/hku-mars/FAST-LIVO2,
observed author main `0d2c0346107b75b59934975adec9a6eeeb913c64`.

This package's initial C++ source is copied from the community ROS 2 Humble port:
https://github.com/Robotic-Developer-Road/FAST-LIVO2,
commit `837b7bbc1431cb04cf936528e52c83c835efba8e`.
It is not an author-maintained ROS 2 branch. Original file notices and LICENSE
are retained. The port's inherited package.xml BSD label was replaced with the
actual bundled GPL-2.0 license declaration.

Local changes support isolated replay: project-local message compatibility,
explicit instantaneous Gazebo point-cloud preprocessing, matching sensor QoS,
runtime output paths, common-state visual diagnostics and a control that disables
the visual correction while retaining the LIVO update schedule. Odometry uses
the measurement epoch and explicit covariance mapping. Replay does not publish
control commands, MAVROS pose, or TF.

Dependencies are pinned by scripts/prepare_livo_dependencies.sh and installed
into a research-only prefix. This baseline does not establish hardware timing,
calibration, integrity guarantees or real flight performance.
