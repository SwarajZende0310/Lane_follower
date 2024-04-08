from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lane_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saz',
    maintainer_email='zendeswaraj53@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_detector = scripts.lane_detector:main',
            'camera_test = scripts.camera_test:main',
            'lane_node = scripts.lane_node:main',
            'yellow_mask = scripts.yellow_mask:main',
        ],
    },
)
