from setuptools import setup

package_name = 'battery_failsafe'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='israel',
    maintainer_email='israel@example.com',
    description='Nodo de seguridad para aterrizaje automático por batería baja del dron Tello.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_failsafe_node = battery_failsafe.battery_failsafe_node:main',
        ],
    },
)

