import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32
from controller import Robot
from math import cos, sin

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')

        # Init Webots
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Get motors
        self.motor_kanan_depan = self.robot.getDevice("motor_kanan_depan")
        self.motor_kanan_belakang = self.robot.getDevice("motor_kanan_belakang")
        self.motor_kiri_depan = self.robot.getDevice("motor_kiri_depan")
        self.motor_kiri_belakang = self.robot.getDevice("motor_kiri_belakang")
        self.motors = [self.motor_kanan_depan, self.motor_kanan_belakang,
                       self.motor_kiri_depan, self.motor_kiri_belakang]
        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)

        # Encoder
        self.encoder_left = self.robot.getDevice("encoder_3")  # kiri belakang
        self.encoder_right = self.robot.getDevice("encoder_4")  # kanan belakang
        self.encoder_left.enable(self.timestep)
        self.encoder_right.enable(self.timestep)

        # IMU
        self.imu = self.robot.getDevice("imu")
        self.imu.enable(self.timestep)

        # GPS
        self.gps = self.robot.getDevice("global")
        self.gps.enable(self.timestep)

        # ROS2 publisher dan subscriber
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.heading_pub = self.create_publisher(Float32, 'robot_heading', 10)
        self.odom_pub = self.create_publisher(Point, 'odom_position', 10)
        self.gps_pub = self.create_publisher(Point, 'gps_position', 10)

        # Velocities
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Odometry states
        self.last_left = 0.0
        self.last_right = 0.0
        self.x = 0.0
        self.y = 0.0

        # Parameter
        self.wheel_radius = 0.02  # meter
        self.wheel_base = 0.10    # meter

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        # self.get_logger().info(f'cmd_vel received: linear={self.linear_velocity:.2f}, angular={self.angular_velocity:.2f}')

    def run(self):
        max_speed = 6.28
        while self.robot.step(self.timestep) != -1:
            rclpy.spin_once(self, timeout_sec=0.01)

            # Ambil data encoder
            left_pos = self.encoder_left.getValue()
            right_pos = self.encoder_right.getValue()
            d_left = (left_pos - self.last_left) * self.wheel_radius
            d_right = (right_pos - self.last_right) * self.wheel_radius
            self.last_left = left_pos
            self.last_right = right_pos

            # Ambil data IMU
            yaw = self.imu.getRollPitchYaw()[2]

            # Update posisi odometryRobotControllerNode
            d_center = (d_left + d_right) / 2.0
            self.x += d_center * cos(yaw)
            self.y += d_center * sin(yaw)

            # Publish heading
            yaw_msg = Float32()
            yaw_msg.data = yaw
            self.heading_pub.publish(yaw_msg)
            # self.get_logger().info(f"Published heading: {yaw_msg.data:.2f}")

            # Publish posisi odometry
            pos_msg = Point()
            pos_msg.x = self.x
            pos_msg.y = self.y
            pos_msg.z = 0.0
            self.odom_pub.publish(pos_msg)

            # Publish posisi GPS
            gps_values = self.gps.getValues()  # [x, y, z]
            gps_msg = Point()
            gps_msg.x = gps_values[0]
            gps_msg.y = gps_values[1]
            gps_msg.z = 0.0
            self.gps_pub.publish(gps_msg)

            # Kendali motor
            v_left = self.linear_velocity - self.angular_velocity * self.wheel_base / 2.0
            v_right = self.linear_velocity + self.angular_velocity * self.wheel_base / 2.0
            
            # konversi m/s → rad/s
            w_left = v_left / self.wheel_radius
            w_right = v_right / self.wheel_radius
            
            # Batasi ke max_speed (6.28 rad/s = ~1 putaran penuh)
            w_left = max(min(w_left, max_speed), -max_speed)
            w_right = max(min(w_right, max_speed), -max_speed)

            self.motor_kiri_depan.setVelocity(w_left)
            self.motor_kiri_belakang.setVelocity(w_left)
            self.motor_kanan_depan.setVelocity(w_right)
            self.motor_kanan_belakang.setVelocity(w_right)

            # self.get_logger().info(f"📍 Odom: ({self.x:.2f}, {self.y:.2f}) | Yaw: {yaw:.2f} rad")
            # self.get_logger().info(f"📡 Lokasi robot : (X = {gps_msg.x:.2f}, Y = {gps_msg.y:.2f})")
            # self.get_logger().info(f"⚙️  Motor command: L={v_left:.2f}, R={v_right:.2f}")

def main():
    rclpy.init()
    node = RobotControllerNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
