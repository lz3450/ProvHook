from setuptools import setup

package_name = "myteleop"

setup(
    name=package_name,
    version="1.0",
    packages=[],
    py_modules=["myteleop"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="KZL",
    maintainer_email="lz3450@outlook.com",
    author="KZL",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: BSD",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="A robot-agnostic teleoperation node to convert keyboard commands to Twist messages.",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["myteleop = myteleop:main"],
    },
)
