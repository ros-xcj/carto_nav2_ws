import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # --------------------------------------------------------------------------
    # 1. 配置各功能包的路径
    # --------------------------------------------------------------------------
    chassis_pkg_dir = get_package_share_directory('chassis_driver')      # 底盘驱动包
    carto_pkg_dir   = get_package_share_directory('cartographer_ros')   # SLAM 建图包
    nav2_pkg_dir    = get_package_share_directory('my_nav2_config')      # 导航配置包
    imu_pkg_dir     = get_package_share_directory('fdilink_ahrs')        # IMU 驱动包
    lidar_pkg_dir   = get_package_share_directory('lslidar_driver')      # 雷达驱动包

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    localization = LaunchConfiguration('localization')
    map_file     = LaunchConfiguration('map_file')
    map_yaml     = LaunchConfiguration('map_yaml')   # [方案B] Nav2 map_server 使用的静态地图 yaml 路径

    # 逻辑选择：根据 localization 参数决定使用哪个 Lua 配置文件
    from launch.substitutions import PythonExpression
    carto_config = PythonExpression([
        "'my_localization.lua' if '", localization, "' == 'true' else 'my_2d_lidar.lua'"
    ])

    # --------------------------------------------------------------------------
    # 2. 依次包含 (Include) 各功能模块的启动文件
    # --------------------------------------------------------------------------

    # A. 启动底盘驱动
    chassis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(chassis_pkg_dir, 'launch', 'chassis_driver.launch.py'))
    )

    # B. 启动 IMU 驱动
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(imu_pkg_dir, 'launch', 'ahrs_driver.launch.py'))
    )

    # C. 启动雷达驱动
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_pkg_dir, 'launch', 'lsn10_net_launch.py'))
    )

    # D. 启动 Cartographer SLAM (支持动态切换模式)
    carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(carto_pkg_dir, 'launch', 'my_2d_slam.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'configuration_basename': carto_config,
            'load_state_filename': map_file
        }.items()
    )

    # E. 启动 Nav2 导航框架
    # [方案B] 同时传入 map_yaml，供 map_server 发布静态地图
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg_dir, 'launch', 'nav2_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml          # 静态地图 yaml → Nav2 map_server
        }.items()
    )

    # --------------------------------------------------------------------------
    # 3. 构造启动描述符
    # --------------------------------------------------------------------------
    ld = LaunchDescription()
    
    # 声明全局变量
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument('localization', default_value='false', description='是否开启纯定位模式'))
    ld.add_action(DeclareLaunchArgument('map_file', default_value='', description='pbstream地图文件绝对路径（给Cartographer定位用），支持 ~/ 前缀'))
    ld.add_action(DeclareLaunchArgument('map_yaml', default_value='/home/nvidia/iBal_ROS2/carto_nav2_ws_05_comm_yamlmap/my_map.yaml', description='[方案B] 静态地图yaml路径（给Nav2 map_server用）'))
    
    # 添加启动项
    ld.add_action(chassis_launch)
    ld.add_action(imu_launch)
    ld.add_action(lidar_launch)
    ld.add_action(carto_launch)
    ld.add_action(nav2_launch)

    return ld
