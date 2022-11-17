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

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('franka_moveit_config'),
                    'launch',
                    'moveit.launch.py'
                ])
            ),
            launch_arguments=[('robot_ip', 'dont-care'), ('use_fake_hardware', 'true')]
        ),
        Node(
            package='moveit_testing',
            executable='simple_move',
        )
    ])
