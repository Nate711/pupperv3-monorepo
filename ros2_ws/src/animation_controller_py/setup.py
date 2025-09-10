from setuptools import find_packages, setup

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
