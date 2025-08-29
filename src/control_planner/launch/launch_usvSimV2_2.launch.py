# V2_2 Modification  compared with V2(V2_1 is skipped): add initial transform from world to NED directly in the launch file instead of making so in a python node. 
# the counterpart python file is waterballControlBasisV9_2.py where we delete the corresponding code for transformation between world frlame and NED frame.

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os

def load_robot_description(context, *args, **kwargs):
    pkg_share = get_package_share_directory('control_planner')
    urdf_path = os.path.join(pkg_share, 'urdf', 'my_ball_with_rudder.urdf')
    with open(urdf_path, 'r') as inf:
        robot_desc = inf.read()
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc, 'publish_frequency': 20.0,'use_sim_time': LaunchConfiguration('use_sim_time')},
                ]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('control_planner')
    # 创建 launch 配置  
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='false'
    )
    
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(pkg_share, 'maps', 'empty_map.yaml')
    )
    
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(pkg_share, 'param', 'nav2_paramsV2.yaml')
    )

    

    # Nodes
    map_to_world = Node(
    package='tf2_ros', executable='static_transform_publisher', name='map_to_world',
    output='screen',
    arguments=[
        '--x', '0', '--y', '0', '--z', '0',
        '--roll', '0', '--pitch', '0', '--yaw', '0',
        '--frame-id', 'map', '--child-frame-id', 'world'
    ],
    parameters=[{'use_sim_time': use_sim_time}]  
    )

    map_to_odom = Node(
    package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
    output='screen',
    arguments=[
        '--x', '0', '--y', '0', '--z', '0',
        '--roll', '0', '--pitch', '0', '--yaw', '0',
        '--frame-id', 'world', '--child-frame-id', 'odom'
    ],
    parameters=[{'use_sim_time': use_sim_time}]  
    )

    world_to_ned = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_ned',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '3.141592653589793',        # π
            '--pitch', '0',
            '--yaw', '1.5707963267948966',        # π/2
            '--frame-id', 'world',
            '--child-frame-id', 'NED'
        ],
        # static TF doesn't need sim time; keep or remove as you like:
        # parameters=[{'use_sim_time': use_sim_time}],
    )


    #  usv_Simulation
    usv_sim = Node(
        package='control_planner', executable='usv_Simulation_rudder', name='usv_Simulation_rudder', 
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]  
    ) #usv_Simulation_rudder

    publish_joint_angle = Node(
        package='control_planner', executable='publish_joint_angle', name='publish_joint_angle',
        output='screen'
    )

    publish_joint_angleV2 = Node(
        package='control_planner', executable='publish_joint_angleV2', name='publish_joint_angleV2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]  
    )
 


    # 获取与拼接默认路径

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    # rviz_config_dir = os.path.join(
    #     nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_config_dir = os.path.join(pkg_share, 'launch', 'test_follow_rudder.rviz')
    


    return launch.LaunchDescription([
        # 声明新的 launch 参数
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),
        map_to_world,
        map_to_odom,
        world_to_ned,
        usv_sim,
        # publish_joint_angleV2,
        OpaqueFunction(function=load_robot_description),

        launch.actions.DeclareLaunchArgument(
            'map',
            default_value=map_yaml_path,
            description='Full path to map file to load'
        ),
        launch.actions.DeclareLaunchArgument(
            'params_file',
            default_value=nav2_param_path,
            description='Full path to param file to load'
        ),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']
            ),
            # 使用 launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path
                
            }.items(),
        ),
        # rviz,
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])

