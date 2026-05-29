from setuptools import setup

package_name = "barcode_camera_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/barcode_camera.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orangepi",
    maintainer_email="orangepi@todo.todo",
    description="Code128 barcode detector using a USB camera.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "barcode_camera_node = barcode_camera_pkg.barcode_camera_node:main",
        ],
    },
)
