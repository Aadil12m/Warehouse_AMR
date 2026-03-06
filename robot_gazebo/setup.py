from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name),
         glob('launch/*launch.[pxy][yma]*')),
         (os.path.join('share',package_name),
         glob('worlds/*')),
         

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aadil',
    maintainer_email='aadil@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'amcl_init_pose_publisher = robot_gazebo.amcl_init_pose.py:main',
        ],
    },
)
