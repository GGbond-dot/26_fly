from setuptools import find_packages, setup

package_name = 'qr_vision_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'pyzbar', 'numpy'],
    zip_safe=True,
    maintainer='orangepi',
    maintainer_email='2449708401@qq.com',
    description='D题 立体货架盘点：单相机二维码识别 + 激光指示 + 对准微调（移植自 24fly opencv01）',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'qr_vision = qr_vision_pkg.decoder_common:main',
            'qr_fine_tune = qr_vision_pkg.qr_fine_tune:main',
        ],
    },
)
