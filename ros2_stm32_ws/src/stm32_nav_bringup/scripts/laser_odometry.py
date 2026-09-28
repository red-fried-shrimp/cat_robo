#!/usr/bin/python3
import math
import numpy as np
from scipy.spatial import KDTree
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data


class LaserOdometry(Node):
    """用相邻两帧 2D 激光做简易里程计估计的 ROS2 节点。

    这个节点用于 `use_laser_odom=true` 的调试模式：订阅 `/scan`，估计机器人
    在 odom 坐标系下的增量运动，发布 `/odom` 和可选的 `odom -> base_link` TF。
    正式导航优先使用 STM32 轮速里程计；激光里程计适合低速、特征明显的环境。
    """

    def __init__(self):
        """声明参数，初始化状态量，并创建订阅、发布和 TF 广播器。"""
        super().__init__('laser_odometry')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('max_match_points', 240)
        self.declare_parameter('stationary_score_margin', 8)
        self.declare_parameter('stationary_score_ratio', 0.04)
        self.declare_parameter('motion_score_penalty', 1.5)
        self.declare_parameter('pure_rotation_score_margin', 10)
        self.declare_parameter('pure_rotation_score_ratio', 0.05)
        self.declare_parameter('match_thresh', 0.06)
        self.declare_parameter('min_match_score_ratio', 0.10)
        self.declare_parameter('search_trans', 0.06)
        self.declare_parameter('search_rot', 0.12)
        self.declare_parameter('step_trans', 0.02)
        self.declare_parameter('step_rot', 0.02)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        scan_topic = self.get_parameter('scan_topic').value
        self.max_match_points = int(self.get_parameter('max_match_points').value)
        self.stationary_score_margin = int(
            self.get_parameter('stationary_score_margin').value)
        self.stationary_score_ratio = float(
            self.get_parameter('stationary_score_ratio').value)
        self.motion_score_penalty = float(
            self.get_parameter('motion_score_penalty').value)
        self.pure_rotation_score_margin = int(
            self.get_parameter('pure_rotation_score_margin').value)
        self.pure_rotation_score_ratio = float(
            self.get_parameter('pure_rotation_score_ratio').value)
        self.match_thresh = float(self.get_parameter('match_thresh').value)
        self.min_match_score_ratio = float(
            self.get_parameter('min_match_score_ratio').value)
        self.search_trans = float(self.get_parameter('search_trans').value)
        self.search_rot = float(self.get_parameter('search_rot').value)
        self.step_trans = float(self.get_parameter('step_trans').value)
        self.step_rot = float(self.get_parameter('step_rot').value)

        self.x = 0.0       # odom 坐标系下的 x 位置，单位 m
        self.y = 0.0       # odom 坐标系下的 y 位置，单位 m
        self.yaw = 0.0     # odom 坐标系下的航向角，单位 rad
        self.prev_stamp = None
        self.prev_points = None
        self.last_motion_log_time = self.get_clock().now()

        self.sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'LaserOdometry started (scan={scan_topic}, '
            f'search_trans={self.search_trans:.2f}m, search_rot={self.search_rot:.2f}rad)'
        )

    def scan_to_points(self, scan):
        """把 LaserScan 极坐标数据转换为雷达坐标系下的二维点云。

        Args:
            scan: sensor_msgs/LaserScan，一帧激光扫描。

        Returns:
            numpy.ndarray: 形状为 (N, 2) 的 xy 点集；无有效点时返回空数组。
        """
        # angle_min/angle_max 与 ranges 一一对应，先生成每条激光束的角度。
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        ranges = np.array(scan.ranges, dtype=np.float32)

        # 只保留雷达声明量程内的点，过滤 0、inf 和过远/过近的读数。
        mask = (ranges >= scan.range_min) & (ranges <= scan.range_max)
        ranges = ranges[mask]
        angles = angles[mask]
        if len(ranges) == 0:
            return np.empty((0, 2), dtype=np.float32)

        # 极坐标 r/theta 转笛卡尔 x/y，仍然处在雷达局部坐标系中。
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        points = np.column_stack((xs, ys)).astype(np.float32)

        # 简易匹配只用于调试，降采样能避免 Python 粗搜索阻塞 TF 发布。
        if len(points) > self.max_match_points:
            idx = np.linspace(0, len(points) - 1, self.max_match_points, dtype=np.int32)
            points = points[idx]
        return points

    def match_scans(self, prev, curr):
        """粗搜索相邻两帧激光之间的平移和旋转增量。

        Args:
            prev: 上一帧二维点云。
            curr: 当前帧二维点云。

        Returns:
            tuple[float, float, float]: dx、dy、dtheta，表示上一帧到当前帧的相对运动。

        这里用 KDTree 统计变换后点的近邻数量作为匹配得分。算法简单但计算量较大，
        适合作为调试里程计；若要长期使用，建议换成成熟 ICP/scan matcher 库。
        """
        if len(prev) < 20 or len(curr) < 20:
            return 0.0, 0.0, 0.0

        # KDTree 加速“变换后的 prev 点”到当前帧 curr 点的最近邻查询。
        tree = KDTree(curr, leafsize=20)
        match_thresh = self.match_thresh

        zero_score = -1
        best_score = -1
        best_quality = float('-inf')
        best_dx = best_dy = best_dth = 0.0
        best_pure_rotation_score = -1
        best_pure_rotation_dth = 0.0

        search_trans = self.search_trans
        search_rot = self.search_rot
        step_trans = self.step_trans
        step_rot = self.step_rot

        def evaluate(dx, dy, dth):
            """计算某个候选位姿增量的匹配点数量。"""
            c, s = math.cos(dth), math.sin(dth)
            tx = prev[:, 0] * c - prev[:, 1] * s + dx
            ty = prev[:, 0] * s + prev[:, 1] * c + dy
            dists, _ = tree.query(np.column_stack((tx, ty)), distance_upper_bound=match_thresh)
            return int(np.isfinite(dists).sum())

        dth_vals = np.arange(-search_rot, search_rot + step_rot, step_rot)
        tx_vals = np.arange(-search_trans, search_trans + step_trans, step_trans)
        ty_vals = np.arange(-search_trans, search_trans + step_trans, step_trans)

        for dth in dth_vals:
            for dx in tx_vals:
                for dy in ty_vals:
                    score = evaluate(dx, dy, dth)
                    if abs(dx) < 1e-9 and abs(dy) < 1e-9 and abs(dth) < 1e-9:
                        zero_score = score
                    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
                        if score > best_pure_rotation_score:
                            best_pure_rotation_score = score
                            best_pure_rotation_dth = dth

                    # 候选运动越大，需要带来更明显的匹配收益；静止时优先选 0。
                    motion_cells = (
                        math.hypot(dx, dy) / step_trans +
                        abs(dth) / step_rot
                    )
                    quality = score - self.motion_score_penalty * motion_cells
                    if quality > best_quality:
                        best_quality = quality
                        best_score = score
                        best_dx, best_dy, best_dth = dx, dy, dth

        if best_score < len(prev) * self.min_match_score_ratio:
            return 0.0, 0.0, 0.0

        min_gain = max(
            self.stationary_score_margin,
            int(len(prev) * self.stationary_score_ratio)
        )
        if best_score - zero_score < min_gain:
            return 0.0, 0.0, 0.0

        pure_rotation_gain = max(
            self.pure_rotation_score_margin,
            int(len(prev) * self.pure_rotation_score_ratio)
        )
        if best_score - best_pure_rotation_score < pure_rotation_gain:
            return 0.0, 0.0, best_pure_rotation_dth

        return best_dx, best_dy, best_dth

    def scan_callback(self, scan):
        """处理一帧激光数据，估计运动增量并发布里程计。

        首帧只缓存不发布；后续每帧与上一帧做匹配，得到局部增量后积分到 odom。
        """
        curr = self.scan_to_points(scan)
        if len(curr) < 20:
            return

        if self.prev_points is None or self.prev_stamp is None:
            self.prev_points = curr
            self.prev_stamp = scan.header.stamp
            return

        dx, dy, dth = self.match_scans(self.prev_points, curr)

        # 使用消息时间戳计算速度；异常时间间隔用 0.1s 兜底，避免速度爆炸。
        dt = (scan.header.stamp.sec - self.prev_stamp.sec) + \
             (scan.header.stamp.nanosec - self.prev_stamp.nanosec) / 1e9
        if dt <= 0.0 or dt > 0.5:
            dt = 0.1

        # 把局部坐标系下的增量旋转到 odom 坐标系并累加。
        self.yaw += dth
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += dx * c - dy * s
        self.y += dx * s + dy * c

        vx = dx / dt
        vy = dy / dt
        wz = dth / dt

        self.publish_odometry(scan.header.stamp, vx, vy, wz)
        if self.publish_tf:
            self.publish_transform(scan.header.stamp)

        if abs(dx) > 1e-6 or abs(dy) > 1e-6 or abs(dth) > 1e-6:
            now = self.get_clock().now()
            if (now - self.last_motion_log_time).nanoseconds > 1_000_000_000:
                self.get_logger().info(
                    f'Laser odom motion: dx={dx:.3f} dy={dy:.3f} dth={dth:.3f} '
                    f'pose=({self.x:.3f}, {self.y:.3f}, {self.yaw:.3f})'
                )
                self.last_motion_log_time = now

        self.prev_points = curr
        self.prev_stamp = scan.header.stamp

    def publish_odometry(self, stamp, vx, vy, wz):
        """发布 `/odom` 消息。

        Args:
            stamp: 激光消息时间戳。
            vx: base_link 局部 x 方向速度。
            vy: base_link 局部 y 方向速度。
            wz: base_link 局部 yaw 角速度。
        """
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        q = self.euler_to_quat(0, 0, self.yaw)
        msg.pose.pose.orientation = q
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.02
        msg.twist.covariance[0] = 0.02
        msg.twist.covariance[7] = 0.02
        msg.twist.covariance[35] = 0.05
        if not rclpy.ok():
            return
        try:
            self.odom_pub.publish(msg)
        except Exception:
            if rclpy.ok():
                raise

    def publish_transform(self, stamp):
        """发布 `odom -> base_link` 动态 TF。"""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = self.euler_to_quat(0, 0, self.yaw)
        t.transform.rotation = q
        if not rclpy.ok():
            return
        try:
            self.tf_broadcaster.sendTransform(t)
        except Exception:
            if rclpy.ok():
                raise

    @staticmethod
    def euler_to_quat(roll, pitch, yaw):
        """将欧拉角转换为 geometry_msgs/Quaternion。"""
        q = Quaternion()
        q.x = math.sin(roll * 0.5) * math.cos(pitch * 0.5) * math.cos(yaw * 0.5) - \
              math.cos(roll * 0.5) * math.sin(pitch * 0.5) * math.sin(yaw * 0.5)
        q.y = math.cos(roll * 0.5) * math.sin(pitch * 0.5) * math.cos(yaw * 0.5) + \
              math.sin(roll * 0.5) * math.cos(pitch * 0.5) * math.sin(yaw * 0.5)
        q.z = math.cos(roll * 0.5) * math.cos(pitch * 0.5) * math.sin(yaw * 0.5) - \
              math.sin(roll * 0.5) * math.sin(pitch * 0.5) * math.cos(yaw * 0.5)
        q.w = math.cos(roll * 0.5) * math.cos(pitch * 0.5) * math.cos(yaw * 0.5) + \
              math.sin(roll * 0.5) * math.sin(pitch * 0.5) * math.sin(yaw * 0.5)
        return q


def main():
    """ROS2 Python 节点入口。"""
    rclpy.init()
    node = LaserOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
