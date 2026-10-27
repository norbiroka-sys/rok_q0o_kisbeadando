from setuptools import setup
import os
from glob import glob

package_name = 'rok_q0o_kisbeadando'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Norbi',
    maintainer_email='norbi@sze.hu',
    description='Sávkövető algoritmus ROS 2 kis beadandóhoz',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_tracker_node = rok_q0o_kisbeadando.lane_tracker_node:main',
        ],
    },
)
