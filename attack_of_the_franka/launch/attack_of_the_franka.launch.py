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
from launch.conditions import IfCondition,UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='simulation',
            default_value='false',
            choices=['true', 'false'],
            description='Selects whether to launch Franka in simulation or with the real robot',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('attack_of_the_franka'),
                    'launch',
                    'robot.launch.py'
                ])
            ),
            condition=UnlessCondition(LaunchConfiguration('simulation')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('attack_of_the_franka'),
                    'launch',
                    'realsense.launch.py'
                ])
            ),
            condition=UnlessCondition(LaunchConfiguration('simulation')),
        ),
        Node(
            package='attack_of_the_franka',
            executable='robot_control',
            parameters=[PathJoinSubstitution(
            [FindPackageShare('attack_of_the_franka'), 'parameters.yaml'])],
        )
    ])
