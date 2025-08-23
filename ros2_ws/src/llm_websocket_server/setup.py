from setuptools import setup

package_name = 'llm_websocket_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'psutil', 'websockets'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='nathankau@gmail.com',
    description='WebSocket server for LLM robot control integration',
    license='LGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'websocket_server = llm_websocket_server.websocket_server:main',
        ],
    },
)