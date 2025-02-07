from setuptools import find_packages, setup

package_name = "openai_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "sounddevice",
        "openai[realtime]",
        "pydub",
        "pyaudio",
        "numpy",
    ],
    zip_safe=True,
    maintainer="parallels",
    maintainer_email="nathankau@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ros_node = openai_bridge.ros_node:main",
        ],
    },
)
