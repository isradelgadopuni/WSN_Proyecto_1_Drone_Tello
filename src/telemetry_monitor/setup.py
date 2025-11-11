from setuptools import setup, find_packages

package_name = 'telemetry_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Indexar el paquete en ament
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # Instalar package.xml
        ('share/' + package_name, ['package.xml']),
        # (Opcional) instalar launch/ si luego agregas archivos .launch.py
        # ('share/' + package_name + '/launch', ['launch/monitor.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Israel',
    author_email='you@example.com',
    maintainer='Israel',
    maintainer_email='you@example.com',
    description='Telemetry monitor node for Tello project.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor_node = telemetry_monitor.monitor_node:main',
        ],
    },
)

