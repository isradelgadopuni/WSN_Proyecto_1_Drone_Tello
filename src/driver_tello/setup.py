from setuptools import find_packages, setup

package_name = 'driver_tello'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='israel',
    maintainer_email='you@example.com',
    description='ROS2 driver/gateway para DJI Tello: publica video y telemetría y expone servicios de control.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'driver_node = driver_tello.driver_node:main',
        ],
    },
)

