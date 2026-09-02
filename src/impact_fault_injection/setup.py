from glob import glob
from setuptools import find_packages, setup


package_name = "impact_fault_injection"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.json")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Xuanqiong-X1 team",
    maintainer_email="xq-sim@invalid.local",
    description="Deterministic P14 fault injection and resilient autonomy.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "impact_fault_injector = impact_fault_injection.fault_injector_node:main",
            "impact_sensor_proxy = impact_fault_injection.sensor_proxy_node:main",
            "impact_p14_controller = impact_fault_injection.p14_controller_node:main",
            "impact_p14_evaluator = impact_fault_injection.p14_evaluator_node:main",
            "impact_p14_visualizer = impact_fault_injection.p14_visualizer_node:main",
        ]
    },
)
