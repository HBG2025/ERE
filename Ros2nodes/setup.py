from setuptools import find_packages, setup

package_name = 'esp32_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Henry B. Guerrero',
    maintainer_email='henry@example.com',
    description='ESP32 communication and control bridge for a wall-following robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_serial_bridge = esp32_bridge.esp32_serial_bridge:main',
            'lidar_left = esp32_bridge.lidarReadings:main',
            'wall_follower = esp32_bridge.wallFollower:main',
        ],
    },
)
