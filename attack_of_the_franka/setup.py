from setuptools import setup

package_name = 'attack_of_the_franka'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml',
            'launch/realsense.launch.py',
            'launch/robot.launch.py',
            'launch/attack_of_the_franka.launch.py',
            'config/apriltag.yaml',
            'config/apriltag.rviz',
            'config/parameters.yaml',
            ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Attack of the Franka',
    maintainer_email='ngmorales97@gmail.com',
    description='Wield a lightsaber to vanquish enemies!',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_processor = attack_of_the_franka.camera_processor:entry',
            'robot_control = attack_of_the_franka.robot_control:entry'
        ],
    },
)
