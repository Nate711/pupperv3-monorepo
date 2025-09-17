from setuptools import find_packages, setup
import os
from glob import glob

package_name = "hailo"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "opencv-python",
        "supervision",
        "loguru",
        "Pillow",
        # 'ultralytics',  # For YOLOv8 in simulation mode
    ],
    zip_safe=True,
    maintainer="nathankau",
    maintainer_email="nathankau@gmail.com",
    description="Hailo neural network package",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "example_node = hailo.example_node:main",
            "hailo_detection = hailo.hailo_detection:main",
            "mock_camera = hailo.mock_camera:main",
        ],
    },
)
