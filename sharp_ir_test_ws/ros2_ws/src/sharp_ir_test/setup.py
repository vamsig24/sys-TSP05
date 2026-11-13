from setuptools import find_packages, setup

package_name = 'sharp_ir_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sharp_ir.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='TSP05 Team',
    maintainer_email='student@example.com',
    description='Standalone Sharp IR Arduino serial bridge for ROS 2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sharp_ir_serial_node = sharp_ir_test.serial_node:main',
        ],
    },
)
