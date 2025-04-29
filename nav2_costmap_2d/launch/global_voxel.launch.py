
# Copyright (c) 2018 Intel Corporation
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import NotEqualsSubstitution
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    namespace = LaunchConfiguration('prefix')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    declare_namespace_cmd = DeclareLaunchArgument(
        'prefix', default_value='global_costmap', description='prefix of voxel_grid'
    )

    load_node = GroupAction(
        actions=[
            Node(
                package='nav2_costmap_2d',
                executable='nav2_costmap_2d_cloud',
                name='nav2_costmap_2d_cloud',
                output='screen',
                namespace=namespace,
                remappings=remappings
            )
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    # Add the actions to launch all of the localiztion nodes
    ld.add_action(load_node)

    return ld
