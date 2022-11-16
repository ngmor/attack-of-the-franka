from setuptools import setup

package_name = 'attack_of_the_franka'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml','launch/realsense.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ngm',
    maintainer_email='ngmorales97@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_processor = attack_of_the_franka.camera_processor:entry'
        ],
    },
)
