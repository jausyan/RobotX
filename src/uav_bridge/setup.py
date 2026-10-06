import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'uav_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='you@example.com',
    description='UAV <-> GCS bridge: rx_msgs /system/... topics <-> internal UAV topics',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_bridge = uav_bridge.bridge_node:main',
        ],
    },
)
