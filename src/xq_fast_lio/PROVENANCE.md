# Source provenance

This package is an isolated, project-local adaptation of the BSD-licensed
`FAST_LIO_ROS2` source snapshot found at:

`/home/accelerate/cuadc_ws/src/FAST_LIO_ROS2`

Only the minimum algorithm source, IKFoM/ikd-Tree headers, message definition,
build files, and original `LICENSE` were copied. The source workspace is never
sourced, linked, built, or modified by this project.

Project-local changes are limited to package/namespace isolation, standard
PointCloud2 operation, P2 QoS compatibility, configurable output topic/frames,
and removal of runtime file logging. The FAST-LIO2 ESIKF, IMU propagation,
point-to-plane update, and ikd-Tree map algorithms remain the upstream code.
