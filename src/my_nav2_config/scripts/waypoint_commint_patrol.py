#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多点闭环巡航节点 - 带中断响应版 (Waypoint Commint Patrol Node)
=============================================================
在 waypoint_patrol.py 基础上新增中断逻辑，完全独立，不修改原文件。

话题输入（来自上位机 Jetson Orin Nano / ROS2 Humble / WiFi）:
  control/mode         (std_msgs/UInt8)   — 0=恢复巡航, 1=停止模式
  control/target_angle (std_msgs/UInt32) — [0,360]=有效转向角（仅唤醒/声源定位时发送）

行为逻辑（解耦版，基于场景深度分析报告 interrupt_control_analysis.md）:
  target_angle(>=0) 到来 → 立即进入 INTERACTING + 执行原地转向（不等 mode=1）
  mode=1 到来       → 进入 INTERACTING + 停止导航（不查 angle，不主动转向）
  mode=0 到来       → 恢复 previous_state，含等待时间补偿

设计要点:
  - 事件驱动: rclpy.Timer 10Hz 驱动状态机，无 time.sleep 阻塞
  - 角度/模式完全解耦: angle 在 _angle_callback 中即时消费，无跨回调存储
  - pending_resume: mode=0 在 min_interaction_duration 内到达则延迟恢复
  - 时间补偿: REACHED_WAITING 被打断后，wait_start_time 向后补偿中断时长

用法:
  ros2 run my_nav2_config waypoint_commint_patrol.py
  ros2 run my_nav2_config waypoint_commint_patrol.py --ros-args -p dwell_time:=10.0
"""

import math
from enum import Enum, auto

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.time import Time as RclpyTime

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from std_msgs.msg import UInt8, UInt32

import tf2_ros
from tf2_ros import TransformException


def transform_point_manual(point, t_msg):
    """
    手动对 Point 应用 TransformStamped 转换，避免 tf2_geometry_msgs 和 numpy 的潜在依赖问题
    """
    vx, vy, vz = point.x, point.y, point.z
    tx = t_msg.transform.translation.x
    ty = t_msg.transform.translation.y
    tz = t_msg.transform.translation.z
    qx = t_msg.transform.rotation.x
    qy = t_msg.transform.rotation.y
    qz = t_msg.transform.rotation.z
    qw = t_msg.transform.rotation.w

    t_x = 2 * (qy * vz - qz * vy)
    t_y = 2 * (qz * vx - qx * vz)
    t_z = 2 * (qx * vy - qy * vx)

    x_rot = vx + qw * t_x + (qy * t_z - qz * t_y)
    y_rot = vy + qw * t_y + (qz * t_x - qx * t_z)
    z_rot = vz + qw * t_z + (qx * t_y - qy * t_x)

    return x_rot + tx, y_rot + ty, z_rot + tz


# ─────────────────────────────────────────────────────────────────────────────
# 状态枚举
# ─────────────────────────────────────────────────────────────────────────────
class PatrolState(Enum):
    INIT            = auto()
    NAVIGATING      = auto()    # 正在前往目标点
    REACHED_WAITING = auto()    # 到达目标点，正在停留计时
    INTERACTING     = auto()    # 中断状态: 停止 / 原地转向


# ─────────────────────────────────────────────────────────────────────────────
# 主节点
# ─────────────────────────────────────────────────────────────────────────────
class WaypointCommintPatrol(Node):
    """带中断响应的多点闭环巡航控制器"""

    # ──────────────────────────────────────────────────────────────────────────
    # 初始化
    # ──────────────────────────────────────────────────────────────────────────
    def __init__(self):
        super().__init__('waypoint_commint_patrol')

        # ── ROS2 参数声明 ────────────────────────────────────────────────────
        self.declare_parameter('dwell_time',                10.0)   # 每点停留秒数
        self.declare_parameter('max_loops',                 0)      # 最大循环次数, 0=无限
        self.declare_parameter('skip_on_failure',           True)   # 导航失败时跳过
        self.declare_parameter('min_interaction_duration',  4.0)    # 最短中断保持时间(s)
        self.declare_parameter('xy_tolerance',              0.5)    # 到达判定半径(m)
        self.declare_parameter('interaction_distance',      0.8)    # 交互安全距离(m)
        self.declare_parameter('target_update_dist',        0.3)    # 目标更新最小移动距离(m)
        self.declare_parameter('target_update_time',        2.0)    # 目标更新最小时间间隔(s)

        self.dwell_time               = self.get_parameter('dwell_time').value
        self.max_loops                = self.get_parameter('max_loops').value
        self.skip_on_failure          = self.get_parameter('skip_on_failure').value
        self.min_interaction_duration = self.get_parameter('min_interaction_duration').value
        self.xy_tolerance             = self.get_parameter('xy_tolerance').value
        self.interaction_distance     = self.get_parameter('interaction_distance').value
        self.target_update_dist       = self.get_parameter('target_update_dist').value
        self.target_update_time       = self.get_parameter('target_update_time').value

        # ── Navigator & TF2 ─────────────────────────────────────────────────
        self.navigator  = BasicNavigator()
        self.tf_buffer  = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── 发布器 ───────────────────────────────────────────────────────────
        # 零速指令：停止时覆盖 Nav2 控制器输出
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── 订阅器（来自上位机 WiFi 话题）──────────────────────────────────
        self.sub_mode  = self.create_subscription(
            UInt8,   'control/mode',         self._mode_callback,  10)
        self.sub_angle = self.create_subscription(
            UInt32, 'control/target_angle', self._angle_callback, 10)
        self.sub_target = self.create_subscription(
            PointStamped, 'vision/target_point', self._target_point_callback, 10)

        # ── 导航点定义 (map 坐标系) ─────────────────────────────────────────
        #   格式: (x 米, y 米, yaw 弧度)
        #   yaw: 0=朝+X  π/2=朝+Y  π=朝-X  -π/2=朝-Y
        #   ★ 与原 waypoint_patrol.py 保持一致，按需修改
        self.waypoints_data = [
            ( 4.0,  0.0,  -1.571),   # WP-1
            ( 4.0, -3.0,   3.142),   # WP-2
            ( 1.0, -3.0,   1.893),   # WP-3
            ( 0.0,  0.0,   0.0  ),   # WP-4: 回到起点
        ]
        self.total_wps = len(self.waypoints_data)

        # ── 状态机核心变量 ───────────────────────────────────────────────────
        self.state          = PatrolState.INIT
        self.previous_state = PatrolState.INIT   # 中断前的状态（现场保护）

        # ── 导航进度追踪 ─────────────────────────────────────────────────────
        self.current_wp_index = 0
        self.loop_count       = 0
        self.patrol_active    = False

        # ── REACHED_WAITING 计时 ─────────────────────────────────────────────
        self.wait_start_time: RclpyTime | None = None

        # ── INTERACTING 计时与标志 ────────────────────────────────────────────
        self.sound_start_time: RclpyTime | None = None
        self.pending_resume = False    # mode=0 过早到达，延迟恢复
        self.is_rotating    = False    # 是否正在执行原地转向
        self.is_following_target = False  # 是否正在趋近目标
        self.last_nav_target_pos = None   # 上次发送的追踪目标点 (x, y)
        self.last_nav_target_time: RclpyTime | None = None

        # ── 角度说明 ──────────────────────────────────────────────────────────
        # target_angle 在 _angle_callback 中被立即消费执行，不存储跨回调状态
        # 无需 latest_angle / angle_update_time / angle_freshness_window 字段

        # ── 原地转向参数 (直接 cmd_vel 控制，绕过 Nav2 位置容差立即 SUCCEEDED 的问题) ────
        self.rotation_target_yaw: float | None = None   # 目标 yaw (map 坐标系, 弧度)
        self.ANGULAR_SPEED = 0.5     # rad/s 转向速度
        self.YAW_TOLERANCE = 0.05    # rad 到位容差 (~3°)

        # ── 状态机定时器 (10Hz) ──────────────────────────────────────────────
        self.control_timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info('WaypointCommintPatrol 节点已初始化')

    def _cancel_nav2_task(self):
        """异步取消 Nav2 任务，防止在回调中阻塞导致 RuntimeError"""
        try:
            if hasattr(self.navigator, 'goal_handle') and self.navigator.goal_handle:
                self.navigator.goal_handle.cancel_goal_async()
                self.get_logger().info('已异步发送取消导航目标请求')
        except Exception as e:
            self.get_logger().warn(f'取消导航任务异常: {e}')

    # ──────────────────────────────────────────────────────────────────────────
    # 话题回调
    # ──────────────────────────────────────────────────────────────────────────
    def _angle_callback(self, msg: UInt32):
        """
        收到声源角度话题：立即执行原地转向（不等 mode=1 到来）。
        angle < 0 视为哨兵值，忽略。
        这是角度与模式解耦的核心：角度是即时命令，收到即执行。
        """
        angle = float(msg.data)
        if angle < 0.0:
            self.get_logger().warn(
                f'[angleCallback] 收到哨兵值 angle={angle:.2f}，忽略')
            return

        self.get_logger().info(
            f'[angleCallback] 收到有效角度 {angle:.2f}°，立即进入 INTERACTING 并执行转向')

        # ★ 若尚未进入 INTERACTING，先做现场保护
        if self.state != PatrolState.INTERACTING:
            self.previous_state = self.state
            self.pending_resume = False
            self.is_rotating    = False
            self.is_following_target = False
            self.last_nav_target_pos = None
            self.sound_start_time = self.get_clock().now()
            self.state = PatrolState.INTERACTING
            self.get_logger().info(
                f'[angleCallback] 中断 {self.previous_state.name} '
                f'(WP-{self.current_wp_index + 1})，进入 INTERACTING')
        else:
            # 已在 INTERACTING（可能由 mode=1 触发），更新转向动作
            self.get_logger().info('[angleCallback] 已在 INTERACTING，更新转向目标')
            self.is_following_target = False
            self.last_nav_target_pos = None

        # ★ 先取消当前 Nav2 任务，确保机器人停止移动
        self._cancel_nav2_task()
        self._publish_zero_vel(count=3)

        # ★ 立即执行转向，角度在此即时消费，不存储
        self.is_rotating = True
        self._rotate_in_place(angle)

    def _mode_callback(self, msg: UInt8):
        """
        收到模式话题：仅负责停止/恢复状态控制，不查询角度，不主动触发转向。
        转向由 _angle_callback 独立触发。
        """
        mode = msg.data
        self.get_logger().info(
            f'[modeCallback] mode={mode}, state={self.state.name}, '
            f'rotating={self.is_rotating}')

        # ── mode=0: 非停止模式，恢复巡航 ─────────────────────────────────────
        if mode == 0:
            if self.state == PatrolState.INTERACTING:
                elapsed = self._elapsed_sec(self.sound_start_time)
                if elapsed < self.min_interaction_duration:
                    # mode=0 来得太早（转向可能尚未完成），标记延迟恢复
                    self.pending_resume = True
                    self.get_logger().info(
                        f'Mode 0 received early ({elapsed:.2f}s < '
                        f'{self.min_interaction_duration}s), pending resume')
                    return
                self._do_resume(elapsed)
            return

        # ── mode!=0: 停止模式（通常 mode=1）────────────────────────────────────
        # ★ 只做停止，不查 angle，不主动转向
        self.get_logger().info(f'[modeCallback] 停止模式 {mode}: cancelTask + 停止')

        if self.state != PatrolState.INTERACTING:
            # ★ 现场保护
            self.previous_state = self.state
            if self.state == PatrolState.NAVIGATING:
                self.get_logger().info(
                    f'Interrupted NAVIGATING to WP-{self.current_wp_index + 1}')
            elif self.state == PatrolState.REACHED_WAITING:
                waited = self._elapsed_sec(self.wait_start_time)
                self.get_logger().info(
                    f'Interrupted REACHED_WAITING at WP-{self.current_wp_index + 1}, '
                    f'already waited {waited:.2f}s')
            self.state          = PatrolState.INTERACTING
            self.pending_resume = False
            self.is_following_target = False
            self.sound_start_time = self.get_clock().now()
        else:
            # 已在 INTERACTING（可能 _angle_callback 已先触发），不刷新 sound_start_time
            self.get_logger().info('[modeCallback] Already INTERACTING.')

        # ★ 仅在非旋转时执行停止；若 _angle_callback 已发起转向则不干扰
        # 如果正在追踪目标，收到 mode=1 我们也强制停止追踪并原地等待
        self.is_following_target = False
        self.last_nav_target_pos = None

        if not self.is_rotating:
            self._cancel_nav2_task()
            self._publish_zero_vel(count=3)
            self.get_logger().info('[modeCallback] 原地停止完成')

    def _target_point_callback(self, msg: PointStamped):
        """处理持续下发的目标坐标，驱动机器人趋近"""
        if self.state != PatrolState.INTERACTING:
            # 方便测试视觉导航，当系统在巡航过程中突然收到视觉目标坐标时，模拟触发一次唤醒（角度0.0）
            self.get_logger().info('[targetPointCallback] 当前处于巡航中，检测到视觉目标。模拟触发唤醒（角度 0°）以开启目标跟随测试！')
            fake_angle = UInt32()
            fake_angle.data = 0
            self._angle_callback(fake_angle)
            return

        if self.is_rotating:
            return  # 等待唤醒转向完成后再开始趋近
            
        try:
            target_frame = msg.header.frame_id
            if target_frame == '':
                target_frame = 'camera_link' # 默认降级处理
                
            if target_frame != 'map':
                t = self.tf_buffer.lookup_transform('map', target_frame, rclpy.time.Time())
                target_x, target_y, _ = transform_point_manual(msg.point, t)
            else:
                target_x = msg.point.x
                target_y = msg.point.y
        except TransformException as e:
            self.get_logger().warn(f'TF转换失败，无法追踪目标: {e}', throttle_duration_sec=2.0)
            return

        cx, cy = self._get_robot_pose()
        if cx is None:
            return

        dist_to_robot = math.hypot(target_x - cx, target_y - cy)

        if dist_to_robot <= self.interaction_distance:
            # 已经到达安全交互距离内
            if self.is_following_target:
                self._cancel_nav2_task()
                self._publish_zero_vel(count=1)
                self.is_following_target = False
                self.get_logger().info(f'已到达交互距离 ({dist_to_robot:.2f}m <= {self.interaction_distance}m)，停止趋近')
            return

        self.is_following_target = True

        now = self.get_clock().now()
        should_update = False
        if self.last_nav_target_pos is None or self.last_nav_target_time is None:
            should_update = True
        else:
            last_x, last_y = self.last_nav_target_pos
            dist_moved = math.hypot(target_x - last_x, target_y - last_y)
            time_elapsed = (now - self.last_nav_target_time).nanoseconds / 1e9
            if dist_moved >= self.target_update_dist or time_elapsed >= self.target_update_time:
                should_update = True

        if should_update:
            # 计算目标点：沿着机器人到目标的连线，距离目标设定距离的位置
            angle_to_person = math.atan2(target_y - cy, target_x - cx)
            goal_x = target_x - self.interaction_distance * math.cos(angle_to_person)
            goal_y = target_y - self.interaction_distance * math.sin(angle_to_person)

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.navigator.get_clock().now().to_msg()
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y
            goal.pose.position.z = 0.0
            goal.pose.orientation.z = math.sin(angle_to_person / 2.0)
            goal.pose.orientation.w = math.cos(angle_to_person / 2.0)
            
            self.navigator.goToPose(goal)
            self.last_nav_target_pos = (target_x, target_y)
            self.last_nav_target_time = now
            self.get_logger().info(
                f'更新趋近目标: 人({target_x:.2f}, {target_y:.2f}), 导航至({goal_x:.2f}, {goal_y:.2f})')

    # ──────────────────────────────────────────────────────────────────────────
    # 恢复逻辑
    # ──────────────────────────────────────────────────────────────────────────
    def _do_resume(self, elapsed_interacting: float):
        """恢复到中断前的状态（含等待时间补偿）"""
        self.get_logger().info(
            f'Resuming from INTERACTING → previous_state={self.previous_state.name}')
        self.pending_resume = False
        self.is_rotating    = False
        self.is_following_target = False

        restored = self.previous_state
        self.state = restored

        if restored == PatrolState.NAVIGATING:
            self.get_logger().info(
                f'Resuming navigation to WP-{self.current_wp_index + 1}')
            self._publish_goal(self.current_wp_index)

        elif restored == PatrolState.REACHED_WAITING:
            # ★ 时间补偿：把中断消耗的时间加回 wait_start_time
            # 等价于 multi_goal_node.cpp 的: wait_start_time_ += elapsed
            if self.wait_start_time is not None:
                comp_ns = int(elapsed_interacting * 1e9)
                self.wait_start_time = RclpyTime(
                    nanoseconds=self.wait_start_time.nanoseconds + comp_ns,
                    clock_type=self.wait_start_time.clock_type)
                self.get_logger().info(
                    f'Resuming REACHED_WAITING at WP-{self.current_wp_index + 1}, '
                    f'compensated {elapsed_interacting:.2f}s of interruption')

    # ──────────────────────────────────────────────────────────────────────────
    # 状态机控制循环（10Hz Timer 驱动）
    # ──────────────────────────────────────────────────────────────────────────
    def _control_loop(self):
        """非阻塞状态机主循环，由 Timer 以 10Hz 调用"""
        if not self.patrol_active:
            return

        # ★ 核心修复：手动极短时间自旋 BasicNavigator 处理底层 Action 网络通信
        # 替代原有 navigator.isTaskComplete() 中长达 0.1s 的阻塞，彻底解决角度订阅回调被饿死的问题
        try:
            rclpy.spin_once(self.navigator, timeout_sec=0.005)
        except Exception:
            pass

        if self.state == PatrolState.INIT:
            self._handle_init()
        elif self.state == PatrolState.NAVIGATING:
            self._handle_navigating()
        elif self.state == PatrolState.REACHED_WAITING:
            self._handle_reached_waiting()
        elif self.state == PatrolState.INTERACTING:
            self._handle_interacting()

    def _is_nav2_complete(self):
        """非阻塞检查任务状态，不抢占线程池资源"""
        if not hasattr(self.navigator, 'result_future') or not self.navigator.result_future:
            return True
        if self.navigator.result_future.done():
            res = self.navigator.result_future.result()
            if res:
                self.navigator.status = res.status
            return True
        return False

    # ── INIT ──────────────────────────────────────────────────────────────────
    def _handle_init(self):
        self.get_logger().info(
            f'Starting patrol. Heading to WP-1/{self.total_wps}')
        self._publish_goal(self.current_wp_index)
        self.state = PatrolState.NAVIGATING

    # ── NAVIGATING ───────────────────────────────────────────────────────────────
    def _handle_navigating(self):
        """NAVIGATING 状态：检测到达或失败"""
        cx, cy = self._get_robot_pose()
        if cx is not None:
            gx, gy, _ = self.waypoints_data[self.current_wp_index]
            dist = math.hypot(gx - cx, gy - cy)
            if dist <= self.xy_tolerance:
                self.get_logger().info(
                    f'WP-{self.current_wp_index + 1} ✅ 距离触发到达 '
                    f'(dist={dist:.2f}m ≤ {self.xy_tolerance}m)')
                self._on_waypoint_reached()
                return

        if self._is_nav2_complete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(
                    f'WP-{self.current_wp_index + 1} ✅ Nav2 SUCCEEDED')
                self._on_waypoint_reached()
            elif result == TaskResult.FAILED:
                self.get_logger().error(
                    f'WP-{self.current_wp_index + 1} ❌ Nav2 FAILED')
                if self.skip_on_failure:
                    self.get_logger().warn('跳过失败点，继续下一个')
                    self._switch_to_next_goal()
                else:
                    self.get_logger().error('停止巡航 (skip_on_failure=False)')
                    self.patrol_active = False
            # CANCELED: 由 INTERACTING 发起，无需在此处理

    # ── REACHED_WAITING ───────────────────────────────────────────────────────
    def _handle_reached_waiting(self):
        """REACHED_WAITING 状态：停留计时，到期切下一点"""
        if self.wait_start_time is None:
            return
        elapsed = self._elapsed_sec(self.wait_start_time)
        self.get_logger().info(
            f'WP-{self.current_wp_index + 1} 已等待 {elapsed:.1f}s / {self.dwell_time}s',
            throttle_duration_sec=3.0)
        if elapsed >= self.dwell_time:
            self._switch_to_next_goal()

    # ── INTERACTING ───────────────────────────────────────────────────────────
    def _handle_interacting(self):
        """INTERACTING 状态：维持停止/等待 mode=0 恢复"""
        # 检查延迟恢复
        if self.pending_resume and self.sound_start_time is not None:
            elapsed = self._elapsed_sec(self.sound_start_time)
            if elapsed >= self.min_interaction_duration:
                self.get_logger().info(
                    f'Pending resume triggered after {elapsed:.2f}s')
                self._do_resume(elapsed)
                return

        if self.is_rotating:
            # ★ 直接 cmd_vel 角速度控制转向，绕过 Nav2 位置容差问题
            cx, cy, current_yaw = self._get_robot_pose_full()
            if cx is None or self.rotation_target_yaw is None:
                self.get_logger().warn(
                    '转向期间 TF 失败，暂停一转', throttle_duration_sec=1.0)
                self._publish_zero_vel()
                return

            # 计算剩余偏转角并归一化到 [-PI, PI]
            yaw_err = self.rotation_target_yaw - current_yaw
            yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

            if abs(yaw_err) <= self.YAW_TOLERANCE:
                # 转向完成
                self._publish_zero_vel(count=3)
                self.is_rotating = False
                self.rotation_target_yaw = None
                self.get_logger().info(
                    f'原地转向完成 ✅ current_yaw={math.degrees(current_yaw):.1f}°')
            else:
                # 持续发角速度指令
                cmd = Twist()
                cmd.angular.z = self.ANGULAR_SPEED if yaw_err > 0 else -self.ANGULAR_SPEED
                self.cmd_vel_pub.publish(cmd)
                self.get_logger().info(
                    f'转向中: yaw_err={math.degrees(yaw_err):.1f}°, '
                    f'angular.z={cmd.angular.z:.2f}',
                    throttle_duration_sec=0.5)
        elif not self.is_following_target:
            # 非旋转且非追踪期间持续发零速（防止 Nav2 控制器覆盖）
            self._publish_zero_vel(count=1)

        if self.sound_start_time is not None:
            elapsed = self._elapsed_sec(self.sound_start_time)
            self.get_logger().info(
                f'INTERACTING: 等待 mode=0 恢复. '
                f'elapsed={elapsed:.1f}s, rotating={self.is_rotating}, '
                f'pending={self.pending_resume}',
                throttle_duration_sec=5.0)

    # ──────────────────────────────────────────────────────────────────────────
    # 路点辅助方法
    # ──────────────────────────────────────────────────────────────────────────
    def _on_waypoint_reached(self):
        """到达路点：开始停留计时，切换状态"""
        self.wait_start_time = self.get_clock().now()
        self.state = PatrolState.REACHED_WAITING
        self.get_logger().info(
            f'WP-{self.current_wp_index + 1} 到达，开始停留 {self.dwell_time}s')

    def _switch_to_next_goal(self):
        """切换到下一个路点（闭环）"""
        next_index = (self.current_wp_index + 1) % self.total_wps
        if next_index == 0:
            self.loop_count += 1
            self.get_logger().info(f'循环 #{self.loop_count} 完成 ✔')
            if self.max_loops > 0 and self.loop_count >= self.max_loops:
                self.get_logger().info(
                    f'已完成 {self.max_loops} 次循环，巡航结束。')
                self.patrol_active = False
                return
        self.current_wp_index = next_index
        self._publish_goal(self.current_wp_index)
        self.state = PatrolState.NAVIGATING

    # ──────────────────────────────────────────────────────────────────────────
    # 动作执行方法
    # ──────────────────────────────────────────────────────────────────────────
    def _rotate_in_place(self, angle_deg: float):
        """
        计算目标 yaw 并存入 rotation_target_yaw。
        实际旋转由 _handle_interacting 10Hz 循环经 /cmd_vel 角速度持续进行，
        绕过 Nav2 goToPose 对原地转向立即返回 SUCCEEDED 的根本问题。
          angle_deg [0, 180]   → 顺时针 (负弧度)
          angle_deg (180, 360] → 逆时针 (正弧度)
        """
        cx, cy, current_yaw = self._get_robot_pose_full()
        if cx is None:
            self.get_logger().warn('获取机器人位姿失败，回退到原地停止')
            self._publish_zero_vel(count=3)
            self.is_rotating = False
            return

        if angle_deg <= 180.0:
            relative_yaw = -(angle_deg * math.pi / 180.0)
        else:
            relative_yaw = (360.0 - angle_deg) * math.pi / 180.0

        self.rotation_target_yaw = current_yaw + relative_yaw
        self.get_logger().info(
            f'设定转向目标: cur={math.degrees(current_yaw):.1f}° '
            f'rel={math.degrees(relative_yaw):.1f}° '
            f'target={math.degrees(self.rotation_target_yaw):.1f}°')

    def _publish_goal(self, wp_index: int):
        """向 Nav2 发布导航目标点"""
        if wp_index < 0 or wp_index >= self.total_wps:
            return
        x, y, yaw = self.waypoints_data[wp_index]
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.navigator.goToPose(goal)
        self.get_logger().info(
            f'Published WP-{wp_index + 1}/{self.total_wps}: '
            f'({x:.2f}, {y:.2f}), yaw={math.degrees(yaw):.0f}°')

    def _publish_zero_vel(self, count: int = 1):
        """发布零速度指令"""
        zero = Twist()
        zero.linear.x  = 0.0
        zero.angular.z = 0.0
        for _ in range(count):
            self.cmd_vel_pub.publish(zero)

    # ──────────────────────────────────────────────────────────────────────────
    # TF2 位姿查询
    # ──────────────────────────────────────────────────────────────────────────
    def _get_robot_pose_full(self):
        """从 TF 查询完整位姿 (x, y, yaw_rad)。失败返回 (None, None, None)"""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return x, y, yaw
        except TransformException as e:
            self.get_logger().warn(
                f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return None, None, None

    def _get_robot_pose(self):
        """从 TF 查询 2D 位置 (x, y)。失败返回 (None, None)"""
        x, y, _ = self._get_robot_pose_full()
        return x, y

    # ──────────────────────────────────────────────────────────────────────────
    # 工具方法
    # ──────────────────────────────────────────────────────────────────────────
    def _elapsed_sec(self, start_time: RclpyTime | None) -> float:
        """计算从 start_time 到现在的秒数，start_time=None 返回 0.0"""
        if start_time is None:
            return 0.0
        return (self.get_clock().now() - start_time).nanoseconds / 1e9

    # ──────────────────────────────────────────────────────────────────────────
    # 主入口
    # ──────────────────────────────────────────────────────────────────────────
    def run_patrol(self):
        """等待 Nav2 激活后启动巡航（spin 驱动 Timer 回调）"""
        self.get_logger().info('正在等待 Nav2 完全激活...')
        self.navigator._waitForNodeToActivate('bt_navigator')
        self.get_logger().info('Nav2 已激活 ✔')
        self.get_logger().info(
            f'巡航配置: {self.total_wps} 个导航点 | '
            f'停留 {self.dwell_time}s | '
            f'循环: {"无限" if self.max_loops == 0 else str(self.max_loops) + "次"}')
        self.patrol_active = True
        rclpy.spin(self)   # Timer 回调在 spin 内被事件循环驱动，无阻塞


# ─────────────────────────────────────────────────────────────────────────────
# 入口
# ─────────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = WaypointCommintPatrol()
    try:
        node.run_patrol()
    except KeyboardInterrupt:
        node.get_logger().info('巡航被用户中断 (Ctrl+C)')
    finally:
        try:
            node.navigator.cancelTask()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
