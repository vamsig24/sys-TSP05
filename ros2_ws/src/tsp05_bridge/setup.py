from setuptools import find_packages, setup

package_name = 'tsp05_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tsp05.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Vamsi Grandhi',
    maintainer_email='vamsig@andrew.cmu.edu',
    description='TSP05 Arduino serial bridge for ROS 2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tsp05_serial_node = tsp05_bridge.serial_node:main',
            'tsp05 = tsp05_bridge.cli:main',
        ],
    },
)
