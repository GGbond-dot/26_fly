from glob import glob
import os

from setuptools import find_packages, setup

package_name = "dbh_visual_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kian",
    maintainer_email="kian@example.com",
    description="ROS2 wrapper package for opencv_init_v4.3 detector script.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dbh_visual_node = dbh_visual_pkg.opencv_init_v43:main",
        ],
    },
)
