from setuptools import setup

package_name = 'seeed_python_reterminal_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Borong Yuan',
    maintainer_email='yuanborong@hotmail.com',
    description='ROS2 Driver for reTerminal',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reterminal_node = seeed_python_reterminal_ros2.reterminal_node:main',
        ],
    },
)
