import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from controller import Robot
from std_msgs.msg import Float32

class CmdVelController(Node):
    def __init__(self):
        super().__init__('cmd_vel_controller')

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
        
        # Inertial Unit
        self.imu = self.robot.getDevice("imu")
        self.imu.enable(self.timestep)

        # Publisher yaw (heading)
        self.heading_pub = self.create_publisher(Float32, 'robot_heading', 10)


        # ROS2 subscriber to cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

        # Log for debug
        self.get_logger().info(f'cmd_vel received: linear={self.linear_velocity:.2f}, angular={self.angular_velocity:.2f}')

    def run(self):
        wheel_base = 0.2  # Adjust ini sesuai ukuran robotmu
        max_speed = 6.28  # sesuaikan dengan Webots

        while self.robot.step(self.timestep) != -1:
            rclpy.spin_once(self, timeout_sec=0.01)
            try:
                rclpy.spin_once(self, timeout_sec=0.01)
            except Exception as e:
                self.get_logger().warn(f"Spin error: {e}")

            # Ambil heading dari IMU dan publish
            imu_values = self.imu.getRollPitchYaw()
            yaw = imu_values[2]  # rotasi terhadap sumbu Z

            # Publish yaw ke ROS
            msg = Float32()
            msg.data = yaw
            self.heading_pub.publish(msg)

            # Basic differential drive formula
            v_left = self.linear_velocity - self.angular_velocity * wheel_base / 2.0
            v_right = self.linear_velocity + self.angular_velocity * wheel_base / 2.0

            # Limit sesuai max_speed
            v_left = max(min(v_left, max_speed), -max_speed)
            v_right = max(min(v_right, max_speed), -max_speed)


            # Set velocity ke motor kanan dan kiri
            self.motor_kiri_depan.setVelocity(v_left)
            self.motor_kiri_belakang.setVelocity(v_left)
            self.motor_kanan_depan.setVelocity(v_right)
            self.motor_kanan_belakang.setVelocity(v_right)
            
            self.get_logger().info(f"Motor command: L={v_left:.2f}, R={v_right:.2f}")

def main():
    rclpy.init()
    controller = CmdVelController()
    controller.run()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()