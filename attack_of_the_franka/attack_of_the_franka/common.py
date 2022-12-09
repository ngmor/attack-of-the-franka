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

"""A common library for functions/values used by all nodes."""

import geometry_msgs.msg
from math import sqrt, sin, cos
from enum import Enum, auto


class FRAMES():
    """Frame names."""

    PANDA_BASE = 'panda_link0'
    PANDA_EE = 'panda_hand_tcp'
    PANDA_TABLE = 'robot_table_reference'
    PANDA_TABLE_RAW = 'robot_table_reference_raw'
    WORK_TABLE1 = 'work_table_reference1'
    WORK_TABLE1_RAW = 'work_table_reference1_raw'
    WORK_TABLE2 = 'work_table_reference2'
    WORK_TABLE2_RAW = 'work_table_reference2_raw'
    CAMERA_COLOR = 'camera_color_frame'
    ALLY = 'ally'
    ENEMY = 'enemy'


class ObjectType(Enum):
    """Object types."""

    ALLY = auto(),
    ENEMY = auto(),


def angle_axis_to_quaternion(theta, axis):
    """
    Convert from angle-axis of rotation to a quaternion.

    Args:
    ----
       theta:  rotation angle, in radians
       axis: the rotational axis. This will be normalized

    Returns
    -------
       A Quaternion corresponding to the rotation

    """
    # https://github.com/m-elwin/me495_tf/blob/humble/me495_tf/quaternion.py
    magnitude = sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
    normalized = [v/magnitude for v in axis]
    sinTheta2 = sin(theta/2.0)
    return geometry_msgs.msg.Quaternion(x=normalized[0]*sinTheta2,
                                        y=normalized[1]*sinTheta2,
                                        z=normalized[2]*sinTheta2,
                                        w=cos(theta/2.0))
