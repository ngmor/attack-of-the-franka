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

"""Integration test to make sure the robot_control node publishes dead enemy count at 100Hz."""

import unittest
import time
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy
import std_msgs.msg
import numpy as np


@pytest.mark.rostest
def generate_test_description():
    """Generate launch file for test."""
    robot_node = Node(
        package='attack_of_the_franka',
        executable='robot_control',
    )

    return (
        LaunchDescription([
            robot_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'robot_control': robot_node
        }
    )


class TestRobotNode(unittest.TestCase):
    """Node for integration test."""

    @classmethod
    def setUpClass(cls):
        """Init ROS communications."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS ."""
        rclpy.shutdown()

    def setUp(self):
        """Create test node."""
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        """Destroy test node."""
        self.node.destroy_node()

    def test_cmd_vel_hz(self, launch_service, robot_control, proc_output):
        """Test that cmd_vel is being published at 100Hz."""
        # Test code adapted from here:
        # https://github.com/ros2/launch_ros/blob/humble/launch_testing_ros/test/examples/talker_listener_launch_test.py

        msgs = []
        times = []

        # Create subscription
        sub = self.node.create_subscription(
            std_msgs.msg.Int16,
            'enemy_dead_count',
            lambda msg: msgs.append(msg),
            10
        )

        num_messages = 100

        # Wait until at least 10 messages are published
        end_time = time.time() + 10

        count = len(msgs)
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            # When we've received a new message, get the time it was received
            if len(msgs) > count:
                count += 1
                times.append(time.time_ns())

            # break once we've received a certain number of messages
            if count >= num_messages:
                break

        # check all the lengths match
        self.assertEqual(count, num_messages)
        self.assertEqual(count, len(msgs))
        self.assertEqual(count, len(times))

        # Get deltas between recorded times
        deltas = []
        for i in range(0, count - 1):
            deltas.append(times[i + 1] - times[i])

        # Get average (nanoseconds)
        delta_avg = np.average(deltas)

        # Convert to seconds
        delta_avg /= 1000000000.

        # get frequency
        frequency = 1. / delta_avg

        # Check that the node publishes at 100 Hz +/- 1Hz
        self.assertAlmostEqual(frequency, 100., delta=1.)

        self.node.destroy_subscription(sub)
