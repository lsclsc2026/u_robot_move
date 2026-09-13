from setuptools import find_packages, setup


package_name = "u_robot_patrol"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/patrol.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="u_robot_move maintainers",
    maintainer_email="devnull@example.com",
    description="Persistent operator waypoints and guarded Nav2 patrol execution.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "waypoint_patrol_node = u_robot_patrol.waypoint_patrol_node:main",
        ],
    },
)
