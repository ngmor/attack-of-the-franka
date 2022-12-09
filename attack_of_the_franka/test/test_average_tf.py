# Copyright 2022 Nick Morales.
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

"""Test get distance function."""

from attack_of_the_franka.common import get_average_transformation
import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped


def test_average_transformations():
    tra_x_list = [
        20.669256034772168,
        20.028376780766987,
        20.65773394735431,
        21.037582034899962,
        21.174689933399655,
        20.40036654725622,
        20.545589334359757,
        21.441749529622115,
        21.304129719449442,
        21.0392505658185,
    ]
    tra_y_list = [
        -9.434016638942031,
        -9.704141680587574,
        -9.766174373695936,
        -8.741786620479354,
        -8.02038068563019,
        -8.704669794690412,
        -8.773251894177827,
        -8.56742906571982,
        -9.991706578742152,
        -9.416576226863622,
    ]
    tra_z_list = [
        1.4637052903802477,
        1.3236455745956035,
        1.2721978013237645,
        1.0128241254873478,
        1.4258663097001851,
        1.4791217207069953,
        1.499123622301522,
        1.1457475299875663,
        1.1752026387726608,
        1.0262820878552215,
    ]
    rot_w_list = [
        0.5365881010890708,
        0.5940808294988829,
        0.5835386349033613,
        0.5407812630692002,
        0.5095561379893379,
        0.5031758119843424,
        0.5257109690565638,
        0.5745896423040883,
        0.5772260180999637,
        0.5794715911343213,
    ]
    rot_x_list = [
        0.5531545741966415,
        0.5554192351245044,
        0.5192314251513603,
        0.5929706503596418,
        0.5861769492588189,
        0.5252985581739489,
        0.5249792612172679,
        0.55574809711441,
        0.5050563737993481,
        0.5002349017842206,
    ]
    rot_y_list = [
        0.5781851376273193,
        0.5418709974396169,
        0.5779422724707985,
        0.5700084484701806,
        0.5712268528157808,
        0.5208627176321892,
        0.5261315679877808,
        0.5516813385678202,
        0.5538189344636092,
        0.5562612190520171,
    ]
    rot_z_list = [
        0.5321958007864839,
        0.5096776472962267,
        0.5845480706636558,
        0.5197740866914693,
        0.5033476049126872,
        0.5260322853196008,
        0.5351233924564567,
        0.5088396486822415,
        0.5502232247690428,
        0.5194869206019882,
    ]

    tf_list = []
    for i in range(len(tra_x_list)):
        transform = TransformStamped().transform

        transform.translation.x = tra_x_list[i]
        transform.translation.y = tra_y_list[i]
        transform.translation.z = tra_z_list[i]
        transform.rotation.w = rot_w_list[i]
        transform.rotation.x = rot_x_list[i]
        transform.rotation.y = rot_y_list[i]
        transform.rotation.z = rot_z_list[i]

        tf_list.append(transform)

    avg_tf = get_average_transformation(tf_list)

    assert avg_tf.translation.x == pytest.approx(np.average(tra_x_list))
    assert avg_tf.translation.y == pytest.approx(np.average(tra_y_list))
    assert avg_tf.translation.z == pytest.approx(np.average(tra_z_list))
    assert avg_tf.rotation.w == pytest.approx(np.average(rot_w_list))
    assert avg_tf.rotation.x == pytest.approx(np.average(rot_x_list))
    assert avg_tf.rotation.y == pytest.approx(np.average(rot_y_list))
    assert avg_tf.rotation.z == pytest.approx(np.average(rot_z_list))
