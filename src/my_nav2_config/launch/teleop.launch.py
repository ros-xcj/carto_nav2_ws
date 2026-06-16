from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # --------------------------------------------------------------------------
    # 键盘控制节点配置
    # --------------------------------------------------------------------------
    # 这个节点必须在带有标准输入的终端里启动 (通常是 SSH 窗口)
    # 逻辑：键盘 -> cmd_vel_nav (话题) -> Velocity Smoother (平滑) -> /cmd_vel (底盘)
    # --------------------------------------------------------------------------
    
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e' if False else '', # 在远程 SSH 下建议直接在当前 shell 运行
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav')  # 将键盘输出重映射到平滑器入口
        ],
        parameters=[{
            'speed': 0.3,    # 初始线速度 (m/s)
            'turn': 0.5,     # 初始角速度 (rad/s)
            'repeat_rate': 10.0, # 强制以 10Hz 频率发布，防止丢包导致机器人“卡住”不停
            'key_timeout': 0.5   # 0.5秒没按键则自动降速
        }],
        output='screen'
    )

    return LaunchDescription([
        teleop_node
    ])
