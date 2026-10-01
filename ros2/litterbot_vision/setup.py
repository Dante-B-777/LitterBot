import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'litterbot_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Benji',
    maintainer_email='benji@litterbot.local',
    description='Camera ingestion and sensor support nodes for LitterBot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = litterbot_vision.camera_node:main',
        ],
    },
)
