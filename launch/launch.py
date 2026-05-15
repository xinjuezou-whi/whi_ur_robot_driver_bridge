# Copyright 2026 WheelHub Intelligent
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, LifecycleNode
from launch_ros.descriptions import ParameterFile
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import State
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Input parameters declaration
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    arm_model = LaunchConfiguration('arm_model')
    robot_ip = LaunchConfiguration('robot_ip')

    # Declare arguments
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace'
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_arm_model_arg = DeclareLaunchArgument(
        'arm_model', default_value='ur5e',
        description='Arm model'
    )
    declare_robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value='192.168.56.100',
        description='Robot IP address'
    )

    # ur_robot_driver launch file
    ur_robot_driver_launch_file = PathJoinSubstitution([
        FindPackageShare('ur_robot_driver'),
        'launch',
        PythonExpression([
            "'",
            arm_model,
            "' + '.launch.py'"
        ])
    ])
    
    # Path to the config file
    config_file = PathJoinSubstitution([
        FindPackageShare('whi_ur_robot_driver_bridge'),
        'config',
        'config.yaml'
    ])
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=config_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # whi_ur_robot_driver_bridge node
    start_whi_ur_robot_driver_bridge_node = LifecycleNode(
        package='whi_ur_robot_driver_bridge',
        executable='whi_ur_robot_driver_bridge_node',
        name='whi_ur_robot_driver_bridge',
        namespace=namespace,
        parameters=[
            configured_params,
            {'robot_ip': robot_ip},
        ],
        output='screen',
    )

    # life cycle manager
    lifecycle_nodes = ['whi_ur_robot_driver_bridge']
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_whi_ur_robot_driver_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': lifecycle_nodes},
        ]
    )

    # ur_robot_driver
    start_ur_robot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_robot_driver_launch_file),
        launch_arguments={
            'namespace': namespace,
            'robot_ip': robot_ip,
        }.items(),
    )

    start_ur_driver_when_bridge_active = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_whi_ur_robot_driver_bridge_node,
            goal_state='active',
            entities=[
                start_ur_robot_driver,
            ]
        )
    )
    
    return LaunchDescription([
        declare_namespace_arg,
        declare_use_sim_time_arg,
        declare_arm_model_arg,
        declare_robot_ip_arg,
        start_whi_ur_robot_driver_bridge_node,
        start_lifecycle_manager_cmd,
        start_ur_driver_when_bridge_active,
    ])
