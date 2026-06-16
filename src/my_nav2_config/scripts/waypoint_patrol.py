#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多点闭环巡航节点 (Waypoint Patrol Node)
========================================
使用 Nav2 Simple Commander API 实现多点导航、定点停留、闭环循环。

用法:
  ros2 run my_nav2_config waypoint_patrol.py
  ros2 run my_nav2_config waypoint_patrol.py --ros-args -p dwell_time:=10.0
"""

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math
import time


class WaypointPatrol(Node):
    """多点闭环巡航控制器"""

    def __init__(self):
        super().__init__('waypoint_patrol')

        # ── ROS2 参数声明 ──────────────────────────────────────────────
        self.declare_parameter('dwell_time', 5.0)          # 每点停留秒数
        self.declare_parameter('max_loops', 0)             # 最大循环次数, 0=无限
        self.declare_parameter('skip_on_failure', True)    # 失败时跳过该点
        self.declare_parameter('max_retries', 1)           # 每个点最大重试次数

        self.dwell_time     = self.get_parameter('dwell_time').value
        self.max_loops      = self.get_parameter('max_loops').value
        self.skip_on_failure = self.get_parameter('skip_on_failure').value
        self.max_retries    = self.get_parameter('max_retries').value

        # ── Navigator 初始化 ──────────────────────────────────────────
        self.navigator = BasicNavigator()

        # ==================================================================
        # ★ 导航点定义 (map 坐标系, 机器人启动位置即原点)
        #   格式: (x 米, y 米, yaw 弧度)
        #   yaw: 0=朝+X  π/2=朝+Y  π=朝-X  -π/2=朝-Y
        #
        #   请根据实际环境修改以下坐标！
        #   提示: 在 RViz2 中鼠标悬停可读取坐标值。
        # ==================================================================
        self.waypoints_data = [
            ( 4.0,  0.0,   -1.571   ),   # WP-1: 正前方 2m
            ( 4.0,  -3.0,   3.142 ),   # WP-2: 右前方, 朝+Y
            ( 1.0,  -3.0,   1.893 ),   # WP-3: 左侧, 朝-X
            ( 0.0,  0.0,   0.0   ),   # WP-4: 回到起点 (闭环)
        ]

    # ─────────────────────────────────────────────────────────────────────
    # 工具方法
    # ─────────────────────────────────────────────────────────────────────
    def create_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        """构造 PoseStamped 消息 (仅绕 Z 轴旋转)"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def navigate_to_point(self, x: float, y: float, yaw: float,
                          wp_name: str) -> bool:
        """
        导航至单个目标点，返回是否成功。
        内含重试逻辑与实时反馈打印。
        """
        for attempt in range(1, self.max_retries + 1):
            suffix = f' (尝试 {attempt}/{self.max_retries})' if self.max_retries > 1 else ''
            self.get_logger().info(
                f'[{wp_name}] 导航至 ({x:.2f}, {y:.2f}), '
                f'朝向 {math.degrees(yaw):.0f}°{suffix}')

            goal_pose = self.create_pose(x, y, yaw)
            self.navigator.goToPose(goal_pose)

            # 等待导航完成，定期打印剩余距离
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback and hasattr(feedback, 'distance_remaining'):
                    self.get_logger().info(
                        f'[{wp_name}] 剩余距离: '
                        f'{feedback.distance_remaining:.2f} m',
                        throttle_duration_sec=3.0)
                time.sleep(0.1)   # 降低轮询频率

            result = self.navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'[{wp_name}] ✅ 到达目标点')
                return True
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f'[{wp_name}] ⚠️ 导航被外部取消')
                return False
            elif result == TaskResult.FAILED:
                self.get_logger().error(f'[{wp_name}] ❌ 导航失败')
                if attempt < self.max_retries:
                    self.get_logger().info(f'[{wp_name}] 将在 2s 后重试...')
                    time.sleep(2.0)

        return False   # 所有重试均失败

    # ─────────────────────────────────────────────────────────────────────
    # 主循环
    # ─────────────────────────────────────────────────────────────────────
    def run_patrol(self):
        """主巡航循环: 依次导航 → 停留 → 闭环循环"""
        self.get_logger().info('正在等待 Nav2 完全激活...')
        # ★ 重要: 本工程使用 Cartographer 定位, 不使用 AMCL。
        # waitUntilNav2Active() 默认会等待 amcl 节点 (永远等不到)。
        # 即使传 localizer='' 也会卡在 _waitForNodeToActivate('')。
        # 因此直接等待 bt_navigator 生命周期节点即可。
        self.navigator._waitForNodeToActivate('bt_navigator')
        self.get_logger().info('Nav2 已激活 ✔')

        total_wps = len(self.waypoints_data)
        self.get_logger().info(
            f'巡航配置: {total_wps} 个导航点 | '
            f'停留 {self.dwell_time}s | '
            f'循环次数: {"无限" if self.max_loops == 0 else self.max_loops}')

        loop_count = 0
        while rclpy.ok():
            loop_count += 1

            # 检查是否达到最大循环次数
            if self.max_loops > 0 and loop_count > self.max_loops:
                self.get_logger().info(
                    f'已完成 {self.max_loops} 次循环，巡航结束。')
                break

            self.get_logger().info(
                f'\n{"="*50}\n'
                f'  巡航循环 #{loop_count} 开始'
                f'{" (共 " + str(self.max_loops) + " 轮)" if self.max_loops > 0 else ""}\n'
                f'{"="*50}')

            for idx, (x, y, yaw) in enumerate(self.waypoints_data):
                wp_name = f'WP-{idx + 1}/{total_wps}'

                success = self.navigate_to_point(x, y, yaw, wp_name)

                if success:
                    # 到达后停留
                    self.get_logger().info(
                        f'[{wp_name}] 🕐 停留 {self.dwell_time} 秒...')
                    time.sleep(self.dwell_time)
                else:
                    if not rclpy.ok():
                        return
                    if self.skip_on_failure:
                        self.get_logger().warn(
                            f'[{wp_name}] 跳过失败点，继续下一个')
                        continue
                    else:
                        self.get_logger().error(
                            f'[{wp_name}] 停止巡航 (skip_on_failure=False)')
                        return

            self.get_logger().info(
                f'循环 #{loop_count} 完成 ✔')

        self.get_logger().info('巡航任务全部结束。')


# ─────────────────────────────────────────────────────────────────────────
# 入口
# ─────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = WaypointPatrol()
    try:
        node.run_patrol()
    except KeyboardInterrupt:
        node.get_logger().info('巡航被用户中断 (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
