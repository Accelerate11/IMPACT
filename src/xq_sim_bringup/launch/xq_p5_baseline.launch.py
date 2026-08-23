"""P5 FAST-LIO -> 0.10 m map/Frontier -> official EGO -> MAVROS stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bridge = Path(get_package_share_directory("xq_gz_bridge"))
    fast_lio = Path(get_package_share_directory("xq_fast_lio"))
    ego_parameters = {
        "use_sim_time": True,
        "fsm/flight_type": 1,
        "fsm/target_topic": "/xq/p5/frontier_goal",
        "fsm/thresh_replan_time": 1.0,
        "fsm/thresh_no_replan_meter": 0.6,
        "fsm/planning_horizon": 5.0,
        "fsm/planning_horizen_time": 3.0,
        "fsm/emergency_time": 1.0,
        "fsm/realworld_experiment": False,
        "fsm/fail_safe": True,
        "fsm/waypoint_num": 0,
        "grid_map/resolution": 0.10,
        "grid_map/map_size_x": 28.0,
        "grid_map/map_size_y": 28.0,
        "grid_map/map_size_z": 4.0,
        "grid_map/local_update_range_x": 6.0,
        "grid_map/local_update_range_y": 6.0,
        "grid_map/local_update_range_z": 3.5,
        "grid_map/obstacles_inflation": 0.35,
        "grid_map/local_map_margin": 10,
        "grid_map/ground_height": 0.0,
        "grid_map/cx": 0.0,
        "grid_map/cy": 0.0,
        "grid_map/fx": 1.0,
        "grid_map/fy": 1.0,
        "grid_map/use_depth_filter": False,
        "grid_map/depth_filter_tolerance": 0.15,
        "grid_map/depth_filter_maxdist": 14.0,
        "grid_map/depth_filter_mindist": 0.35,
        "grid_map/depth_filter_margin": 0,
        "grid_map/k_depth_scaling_factor": 1.0,
        "grid_map/skip_pixel": 1,
        "grid_map/p_hit": 0.65,
        "grid_map/p_miss": 0.35,
        "grid_map/p_min": 0.12,
        "grid_map/p_max": 0.90,
        "grid_map/p_occ": 0.80,
        "grid_map/min_ray_length": 0.1,
        "grid_map/max_ray_length": 14.0,
        "grid_map/virtual_ceil_height": 3.2,
        "grid_map/virtual_ceil_yp": -1.0,
        "grid_map/virtual_ceil_yn": -1.0,
        "grid_map/visualization_truncate_height": 3.5,
        "grid_map/show_occ_time": False,
        "grid_map/pose_type": 1,
        "grid_map/frame_id": "xq_lio_map",
        "grid_map/odom_depth_timeout": 1.0,
        "manager/max_vel": 0.65,
        "manager/max_acc": 1.0,
        "manager/max_jerk": 2.0,
        "manager/control_points_distance": 0.35,
        "manager/feasibility_tolerance": 0.05,
        "manager/planning_horizon": 5.0,
        "manager/use_distinctive_trajs": True,
        "manager/drone_id": 0,
        "optimization/lambda_smooth": 1.0,
        "optimization/lambda_collision": 0.8,
        "optimization/lambda_feasibility": 0.2,
        "optimization/lambda_fitness": 1.0,
        "optimization/dist0": 0.55,
        "optimization/swarm_clearance": 0.55,
        "optimization/max_vel": 0.65,
        "optimization/max_acc": 1.0,
        "bspline/limit_vel": 0.65,
        "bspline/limit_acc": 1.0,
        "bspline/limit_ratio": 1.1,
        "prediction/obj_num": 0,
        "prediction/lambda": 1.0,
        "prediction/predict_rate": 1.0,
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument("evaluation_result_file"),
            DeclareLaunchArgument("integrity_geometry_enable", default_value="false"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="xq_p5_base_to_livox",
                arguments=["--x", "0.04", "--y", "0", "--z", "0.12", "--roll", "0", "--pitch", "0", "--yaw", "0", "--frame-id", "xq_base_link", "--child-frame-id", "livox_frame"],
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="xq_p5_base_to_imu",
                arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0", "--frame-id", "xq_base_link", "--child-frame-id", "livox_imu"],
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="xq_gz_bridge",
                executable="xq_gz_bridge_node",
                parameters=[str(bridge / "config" / "p5_structured_room.yaml")],
            ),
            Node(
                package="xq_fast_lio",
                executable="fastlio_mapping",
                name="xq_fast_lio",
                parameters=[
                    str(fast_lio / "config" / "xq_p4.yaml"),
                    {
                        "use_sim_time": True,
                        "integrity_geometry.enable": LaunchConfiguration("integrity_geometry_enable"),
                    },
                ],
            ),
            Node(
                package="xq_autonomy",
                executable="xq_p4_external_nav",
                name="xq_p5_external_nav",
                parameters=[{"use_sim_time": False}],
            ),
            Node(
                package="xq_autonomy",
                executable="xq_p5_frontier",
                parameters=[{"use_sim_time": True, "map_half_extent_m": 14.0}],
            ),
            Node(
                package="ego_planner",
                executable="ego_planner_node",
                name="xq_p5_ego_planner",
                remappings=[
                    ("odom_world", "/xq/p5/ego_odom"),
                    ("grid_map/odom", "/xq/p5/ego_odom"),
                    ("grid_map/cloud", "/xq/p5/cloud_map"),
                    ("grid_map/occupancy_inflate", "/xq/p5/ego_occupancy_inflate"),
                    ("planning/bspline", "/planning/bspline"),
                ],
                parameters=[ego_parameters],
                output="screen",
            ),
            Node(
                package="ego_planner",
                executable="traj_server",
                name="xq_p5_traj_server",
                remappings=[("planning/bspline", "/planning/bspline")],
                parameters=[{"use_sim_time": True, "traj_server/time_forward": 1.0}],
                output="screen",
            ),
            Node(
                package="xq_autonomy",
                executable="xq_p5_ego_command",
                parameters=[{"use_sim_time": False}],
            ),
            Node(
                package="xq_autonomy",
                executable="xq_p5_evaluator",
                parameters=[{"use_sim_time": True, "result_file": LaunchConfiguration("evaluation_result_file")}],
            ),
        ]
    )
