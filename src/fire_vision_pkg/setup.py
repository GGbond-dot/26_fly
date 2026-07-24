import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'fire_vision_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='orangepi',
    maintainer_email='2449708401@qq.com',
    description='G题 空地协同智能消防：下视相机红色火花图案检测 + 像素反投影火源地面坐标',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fire_detector = fire_vision_pkg.fire_detector:main',
            'fire_color_tune = fire_vision_pkg.fire_color_tune:main',
        ],
    },
)
