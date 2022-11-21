from setuptools import setup

package_name = 'moveit_testing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/moveit_testing.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Attack of the Franka',
    maintainer_email='megansindelar2023@u.northwestern.edu',
    description='MoveIt Python API',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_move=moveit_testing.simple_move:simple_move_entry',
            'move_group=moveit_testing.move_group:movegroup_entry'
        ],
    },
)
