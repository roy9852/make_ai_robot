from setuptools import setup
import os
from glob import glob

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 1. Install Launch Files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 2. Install Config Files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 3. Install Model Files
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='min_5214',
    maintainer_email='min_5214@snu.ac.kr',
    description='Perception module for CRAIP',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = perception.perception:main',
        ],
    },
)