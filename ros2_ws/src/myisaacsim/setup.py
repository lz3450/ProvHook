from setuptools import find_packages, setup

package_name = "myisaacsim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/task0.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/panda_moveit_config.rviz",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kzl",
    maintainer_email="lz3450@outlook.com",
    description="Isaac Sim ROS2 nodes",
    license="GPL-3.0-only",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "simple_joint_command_publisher = myisaacsim.simple_joint_command_publisher:main",
            "simple_joint_state_logger = myisaacsim.simple_joint_state_logger:main",
        ],
    },
)
