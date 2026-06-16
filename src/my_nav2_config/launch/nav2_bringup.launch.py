import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_share = get_package_share_directory('my_nav2_config')
    
    # --------------------------------------------------------------------------
    # 启动配置参数
    # --------------------------------------------------------------------------
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml_file = LaunchConfiguration('map')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Nav2 参数文件的完整路径')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时钟 (Gazebo测试时设为true)')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='是否自动启动 Nav2 状态机')

    # [方案B] 静态地图 yaml 路径，由 map_server 加载
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='/home/victor/Desktop/carto_nav2_ws_05_comm/my_map.yaml',
        description='Nav2 map_server 使用的静态地图 yaml 文件完整路径')

    # --------------------------------------------------------------------------
    # 定义 Nav2 核心生命周期节点
    # [方案B] 加入 map_server；定位仍由 Cartographer 提供，不使用 amcl
    # --------------------------------------------------------------------------
    lifecycle_nodes = ['map_server',
                       'controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    # [方案B] 定位由 Cartographer 纯定位模式提供（map→odom TF），不使用 amcl
    # map_server 负责发布静态 /map 话题，Cartographer 不再运行 occupancy_grid_node
    
    # 配置文件动态重写（用于将 use_sim_time 等全局变量注入 yaml）
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    # --------------------------------------------------------------------------
    # 节点启动定义
    # --------------------------------------------------------------------------
    load_nodes = GroupAction(
        actions=[
            # [方案B] 0. 静态地图服务器 - 从 yaml 文件发布 /map 话题
            # 定位由 Cartographer 纯定位模式提供（map→odom TF），不使用 amcl
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[configured_params,
                            {'yaml_filename': map_yaml_file}]),

            # 1. 控制器服务器 (负责局部路径跟随)
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
                remappings=[('cmd_vel', 'cmd_vel_nav')]), # 将原始速度指令重映射给平滑器逻辑

            # 2. 规划器服务器 (负责全局路径规划)
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params]),

            # 3. 行为服务器 (负责卡住时的自动脱困及其他常规行为)
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params]),

            # 4. 行为树导航器 (负责管理导航逻辑流程)
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params]),

            # 5. 多点任务协调器
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params]),

            # 6. 速度平滑器 (防止突变，保护底盘电机)
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[configured_params],
                remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', '/cmd_vel')]),

            # 7. 生命周期管理器 (负责按顺序启动并激活上述所有节点)
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_map_yaml_cmd)  # [方案B] 静态地图路径参数
    ld.add_action(load_nodes)

    return ld
