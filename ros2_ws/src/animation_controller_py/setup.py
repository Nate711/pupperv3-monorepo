from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'animation_controller_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/animation_controller_py.launch.py']),
        # Include all animation CSV files from launch/animations folder
        (os.path.join('share', package_name, 'launch', 'animations'), 
            glob('launch/animations/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nathankau',
    maintainer_email='nathankau@gmail.com',
    description='Python animation controller that publishes to forward command controllers',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'animation_controller_py = animation_controller_py.animation_controller:main',
        ],
    },
)
