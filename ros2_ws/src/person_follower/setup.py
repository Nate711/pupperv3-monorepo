from setuptools import find_packages, setup
import os
from glob import glob

package_name = "person_follower"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="nathankau",
    maintainer_email="nathankau@gmail.com",
    description="Visual servoing person follower for Pupper V3",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "person_follower_node = person_follower.person_follower_node:main",
        ],
    },
)