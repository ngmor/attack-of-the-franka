# Copyright 2022 Attack of the Franka.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='launch_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Selects whether or not to launch RVIZ to view transforms.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ),
            launch_arguments=[
                ('depth_module.profile', '1280x720x30'),
                ('align_depth.enable', 'true'),
                ('pointcloud.enable', 'true'),
            ]
        ),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            parameters=[PathJoinSubstitution(
            [FindPackageShare('attack_of_the_franka'), 'apriltag.yaml'])],
            remappings=[('/image_rect', '/camera/color/image_raw'),
                        ('/camera_info', '/camera/color/camera_info'),]
        ),
        Node(
            package='attack_of_the_franka',
            executable='camera_processor',
            parameters=[PathJoinSubstitution(
            [FindPackageShare('attack_of_the_franka'), 'parameters.yaml'])],
        ),
        # RVIZ node for viewing the robot only
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('attack_of_the_franka'),
                    'apriltag.rviz'
                ])
            ]
        ),
    ])
