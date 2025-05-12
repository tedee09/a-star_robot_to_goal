import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Path
from math import atan2, sqrt, pi
from std_msgs.msg import Float32
import time
import math 

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower_node')
        self.declare_parameter('speed', 0.5)

        # Publisher ke cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber ke /path
        self.path_sub = self.create_subscription(Path, 'path', self.path_callback, 10)

        # Subscriber ke posisi robot
        self.robot_sub = self.create_subscription(Point, 'robot_position', self.robot_callback, 10)

        # Subscriber ke heading dari IMU
        self.heading_sub = self.create_subscription(Float32, 'robot_heading', self.heading_callback, 10)

        # Path dan posisi saat ini
        self.path_points = []
        self.current_robot_pos = None
        self.current_heading = None
        self.target_index = 0

        self.pause_until = None

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Path Follower Node Started.")

    def path_callback(self, msg):
        self.path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.target_index = 0
        self.cmd_pub.publish(Twist())  # Hentikan dulu biar mulai fresh
        self.get_logger().info(f"Received path with {len(self.path_points)} points.")
        self.get_logger().info(f"Starting to follow new path from current position to {self.path_points[0]}")

    def robot_callback(self, msg):
        self.current_robot_pos = (msg.x, msg.y)

    def heading_callback(self, msg):
        self.current_heading = msg.data
        self.get_logger().debug(f"Received heading: {msg.data:.2f}")

    def shortest_angular_distance(self, from_angle, to_angle):
        """Wrap-around safe difference in angle"""
        delta = to_angle - from_angle
        while delta > pi:
            delta -= 2 * pi
        while delta < -pi:
            delta += 2 * pi
        return delta

    def control_loop(self):
        if self.current_robot_pos is None or not self.path_points or self.current_heading is None:
            self.get_logger().warn("Menunggu data posisi atau heading...")
            return

        # Jika sudah mencapai seluruh waypoint, hentikan
        if self.target_index >= len(self.path_points):
            self.get_logger().info("All waypoints reached. Robot stopped.")
            self.cmd_pub.publish(Twist())
            return

        if not isinstance(self.current_heading, float) or math.isnan(self.current_heading):
            self.get_logger().warn("Heading tidak valid!")
            return

        # Target saat ini
        tx, ty = self.path_points[self.target_index]
        rx, ry = self.current_robot_pos

        # Jarak dan arah
        dx = tx - rx
        dy = ty - ry
        distance = sqrt(dx**2 + dy**2)
        desired_angle = atan2(dy, dx)
        error_angle = self.shortest_angular_distance(self.current_heading, desired_angle)

        current_time = time.time()
        if self.pause_until and current_time < self.pause_until:
            return

        # Jika jarak cukup dekat, anggap waypoint tercapai
        if distance < 0.10 or (abs(error_angle) < 0.2 and distance < 0.25):
            self.get_logger().info(f"Waypoint {self.target_index + 1}/{len(self.path_points)} reached: ({tx:.2f}, {ty:.2f})")
            self.target_index += 1
            self.cmd_pub.publish(Twist())  # stop sejenak
            self.pause_until = time.time() + 0.2  # Delay 0.2 detik
            return

        # Tambahkan batas maksimal linear velocity dan minimal rotasi
        max_linear_speed = 3.0
        max_angular_speed = 1.0

        twist = Twist()
        # Pelankan linear jika error besar, tapi jangan sampai 0 total
        if abs(error_angle) > 0.4:
            twist.linear.x = 0.0  # fokus putar dulu
            twist.angular.z = max(min(max_angular_speed * error_angle, max_angular_speed), -max_angular_speed)
            self.get_logger().info("Belum sejajar, hanya berputar.")
        else:
            # Sudah cukup lurus, boleh maju
            twist.linear.x = max_linear_speed * (1 - abs(error_angle))  # semakin kecil error, makin cepat
            twist.linear.x = max(twist.linear.x, 0.2)  # minimal speed agar tidak stagnan
            twist.angular.z = max(min(max_angular_speed * error_angle, max_angular_speed), -max_angular_speed)

        self.cmd_pub.publish(twist)
        self.get_logger().info(f"   - Target {self.target_index + 1}/{len(self.path_points)} → ({tx:.2f}, {ty:.2f})")
        self.get_logger().info(f"   - Moving to ({tx:.2f}, {ty:.2f}) | Pos: ({rx:.2f}, {ry:.2f})")
        self.get_logger().info(f"   - Heading: {self.current_heading:.2f} | Desired: {desired_angle:.2f} | Δ: {error_angle:.2f}")
        self.get_logger().info(f"   - Speed: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

