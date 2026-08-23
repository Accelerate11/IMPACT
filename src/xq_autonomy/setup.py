from glob import glob
from setuptools import find_packages, setup


package_name = "xq_autonomy"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/config",
            glob("config/*.yaml") + glob("config/*.json") + glob("config/*.parm"),
        ),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Xuanqiong-X1 team",
    maintainer_email="xq-sim@invalid.local",
    description="Algorithm-level SIL proxies and safety logic for Xuanqiong-X1.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "xq_stack_node = xq_autonomy.stack_node:main",
            "xq_metrics_node = xq_autonomy.metrics_node:main",
            "xq_fault_injector = xq_autonomy.fault_injector_node:main",
            "xq_network_relay = xq_autonomy.network_relay_node:main",
            "xq_p1_flight_baseline = xq_autonomy.p1_flight_baseline_node:main",
            "xq_p2_sensor_validator = xq_autonomy.p2_sensor_validator_node:main",
            "xq_p3_trajectory = xq_autonomy.p3_trajectory_node:main",
            "xq_p3_evaluator = xq_autonomy.p3_evaluator_node:main",
            "xq_p4_external_nav = xq_autonomy.p4_external_nav_node:main",
            "xq_p4_mission = xq_autonomy.p4_mission_node:main",
            "xq_p5_frontier = xq_autonomy.p5_frontier_node:main",
            "xq_p5_ego_command = xq_autonomy.p5_ego_command_node:main",
            "xq_p5_mission = xq_autonomy.p5_mission_node:main",
            "xq_p5_evaluator = xq_autonomy.p5_evaluator_node:main",
            "xq_p6_directional_integrity = xq_autonomy.p6_directional_integrity_node:main",
            "xq_p6_integrity_evaluator = xq_autonomy.p6_integrity_evaluator_node:main",
            "xq_p7_calibration_collector = xq_autonomy.p7_calibration_collector_node:main",
            "xq_p8_alert_limit = xq_autonomy.p8_alert_limit_node:main",
            "xq_p8_alert_limit_evaluator = xq_autonomy.p8_alert_limit_evaluator_node:main",
            "xq_p9_integrity_margin = xq_autonomy.p9_integrity_margin_node:main",
            "xq_p9_trajectory_gate = xq_autonomy.p9_trajectory_gate_node:main",
            "xq_p9_gate_scenario = xq_autonomy.p9_gate_scenario_node:main",
            "xq_p9_gate_evaluator = xq_autonomy.p9_gate_evaluator_node:main",
            "xq_p9_replay_visualizer = xq_autonomy.p9_replay_visualizer_node:main",
            "xq_p10_active_perception = xq_autonomy.p10_active_perception_node:main",
            "xq_p10_flight_controller = xq_autonomy.p10_flight_controller_node:main",
            "xq_p10_flight_evaluator = xq_autonomy.p10_flight_evaluator_node:main",
            "xq_p10_information_map = xq_autonomy.p10_information_map_node:main",
            "xq_p10_replay_visualizer = xq_autonomy.p10_replay_visualizer_node:main",
            "xq_p10_gate_scenario = xq_autonomy.p10_gate_scenario_node:main",
            "xq_p10_gate_evaluator = xq_autonomy.p10_gate_evaluator_node:main",
        ],
    },
)
