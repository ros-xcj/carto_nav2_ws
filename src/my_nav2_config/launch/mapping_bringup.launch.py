import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            GroupAction, RegisterEventHandler, EmitEvent,
                            LogInfo, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from nav2_common.launch import RewrittenYaml
import lifecycle_msgs.msg

def generate_launch_description():
    # 路径获取
    chassis_pkg_dir = get_package_share_directory('chassis_driver')
    carto_pkg_dir   = get_package_share_directory('cartographer_ros')
    nav2_config_dir = get_package_share_directory('my_nav2_config')
    imu_pkg_dir     = get_package_share_directory('fdilink_ahrs')
    lidar_pkg_dir   = get_package_share_directory('lslidar_driver')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file  = LaunchConfiguration('params_file', 
                                      default=os.path.join(nav2_config_dir, 'config', 'nav2_params.yaml'))

    # ──────────────────────────────────────────────────────────────
    # 1. 启动底盘驱动
    # ──────────────────────────────────────────────────────────────
    chassis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(chassis_pkg_dir, 'launch', 'chassis_driver.launch.py'))
    )

    # ──────────────────────────────────────────────────────────────
    # 2. 启动 IMU 驱动
    # ──────────────────────────────────────────────────────────────
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(imu_pkg_dir, 'launch', 'ahrs_driver.launch.py'))
    )

    # ──────────────────────────────────────────────────────────────
    # 3. 启动雷达驱动 (LifecycleNode)
    #    直接定义节点而非 IncludeLaunchDescription，以便用 launch 事件
    #    自动管理 configure → activate 生命周期转换
    # ──────────────────────────────────────────────────────────────
    lidar_params_file = os.path.join(
        lidar_pkg_dir, 'params', 'lidar_net_ros2', 'lsn10_net.yaml')

    lidar_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[lidar_params_file],
    )

    # 生命周期自动管理 Step-1:
    # 当雷达完成 configure（到达 'inactive' 状态）后，自动触发 activate
    auto_activate_lidar = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lidar_node,
            goal_state='inactive',
            entities=[
                LogInfo(msg='[mapping_bringup] 雷达 configure 完成，正在 activate...'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lidar_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # 生命周期自动管理 Step-2:
    # 延迟 3 秒后触发 configure（等待节点进程启动并完成 ROS 注册）
    auto_configure_lidar = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='[mapping_bringup] 正在 configure 雷达...'),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=launch.events.matches_action(lidar_node),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            )),
        ],
    )

    # ──────────────────────────────────────────────────────────────
    # 4. 启动 Cartographer SLAM
    # ──────────────────────────────────────────────────────────────
    carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(carto_pkg_dir, 'launch', 'my_2d_slam.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ──────────────────────────────────────────────────────────────
    # 5. 启动速度平滑器 (用于键盘控制的电机保护)
    #    注意: lifecycle_manager 只管理 velocity_smoother，
    #    雷达生命周期由上方的 launch 事件独立管理
    # ──────────────────────────────────────────────────────────────
    param_substitutions = {'use_sim_time': use_sim_time}
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    smoother_nodes = GroupAction(
        actions=[
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[configured_params],
                remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', '/cmd_vel')]),
            
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_mapping',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': True},
                            {'node_names': ['velocity_smoother']}])
        ]
    )

    # ──────────────────────────────────────────────────────────────
    # 组装 LaunchDescription
    # 注意顺序：先注册事件处理器，再启动雷达节点
    # ──────────────────────────────────────────────────────────────
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(chassis_launch)
    ld.add_action(imu_launch)
    ld.add_action(auto_activate_lidar)     # 先注册 activate 事件监听
    ld.add_action(lidar_node)              # 启动雷达进程
    ld.add_action(auto_configure_lidar)    # 延迟触发 configure
    ld.add_action(carto_launch)
    ld.add_action(smoother_nodes)

    return ld
