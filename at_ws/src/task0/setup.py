from setuptools import find_packages, setup

package_name = "task0"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/task0.launch.py"]),
        ("share/" + package_name + "/rviz2", ["rviz2/task0.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kzl",
    maintainer_email="lz3450@outlook.com",
    description="Task 0 package for RoboGuard project",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task0_logger = task0.logger:main"
        ],
    },
)
