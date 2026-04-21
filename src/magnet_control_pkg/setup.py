from glob import glob
import os

from setuptools import find_packages, setup

package_name = "magnet_control_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kian",
    maintainer_email="kian@example.com",
    description="ROS 2 package for controlling the active-low A4 battery magnet.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "magnet_control_node = magnet_control_pkg.magnet_node:main",
        ],
    },
)
