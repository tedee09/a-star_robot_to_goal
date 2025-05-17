import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from controller import Robot

class TeleopController(Node):
    def __init__(self):
        super().__init__('teleop_controller')

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

        # ROS2 subscriber to cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

        # Log for debug
        self.get_logger().info(f'cmd_vel received: linear={self.linear_velocity:.2f}, angular={self.angular_velocity:.2f}')
    
    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
    
        self.get_logger().info(f'[CMD_VEL] linear: {self.linear_velocity}, angular: {self.angular_velocity}')

    def run(self):
        wheel_base = 0.2  # Adjust ini sesuai ukuran robotmu

        while self.robot.step(self.timestep) != -1:
            rclpy.spin_once(self, timeout_sec=0.01)
            # Basic differential drive formula
            v_left = self.linear_velocity - self.angular_velocity * wheel_base / 2.0
            v_right = self.linear_velocity + self.angular_velocity * wheel_base / 2.0

            # Set velocity ke motor kanan dan kiri
            self.motor_kiri_depan.setVelocity(v_left)
            self.motor_kiri_belakang.setVelocity(v_left)
            self.motor_kanan_depan.setVelocity(v_right)
            self.motor_kanan_belakang.setVelocity(v_right)

def main():
    rclpy.init()
    controller = TeleopController()
    controller.run()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()