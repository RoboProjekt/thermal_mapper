from setuptools import setup
import os
from glob import glob

package_name = 'thermal_mapper'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bauya',
    maintainer_email='baau1001@stud.hs-kl.de',
    description='Thermal 3D mapping via voxel-to-pixel projection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gimbal_tf_broadcaster = thermal_mapper.gimbal_tf_broadcaster:main',
            'thermal_camera_publisher = thermal_mapper.thermal_camera_publisher:main',
            'thermal_projection_node = thermal_mapper.thermal_projection_node:main',
            'test_gimbal_sweep = thermal_mapper.test_gimbal_sweep:main',
        ],
    },
)
