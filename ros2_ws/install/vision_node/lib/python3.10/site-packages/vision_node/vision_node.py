import rclpy
from rclpy.node import Node
from controller import Robot
import cv2
import numpy as np
import math
from geometry_msgs.msg import Point

# Konversi dari pixel ke koordinat dunia (disesuaikan dari path_planner_node)
def pixel_to_world(x_pixel, y_pixel):
    x_world = 0.00250 * x_pixel + 0.00000 * y_pixel - 1.00249
    y_world = 0.00000 * x_pixel - 0.00116 * y_pixel + 0.72330
    return x_world, y_world

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)

        self.get_logger().info("Vision node started. Waiting for image...")

        # Warna HSV untuk robot (merah)
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])

        # Warna HSV untuk goal (biru)
        self.lower_blue = np.array([100, 100, 100])
        self.upper_blue = np.array([130, 255, 255])

        self.robot_pub = self.create_publisher(Point, 'robot_position', 10)
        self.goal_pub = self.create_publisher(Point, 'goal_position', 10)

    def detect_red_position(self, image, label="ROBOT", color_draw=(0, 0, 255)):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(image, (cx, cy), 6, color_draw, -1)
                cv2.putText(image, label, (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                self.get_logger().info(f"[{label}] Detected at ({cx}, {cy})")
                return (cx, cy)
        return None

    def detect_blue_position(self, image, lower_color, upper_color, label="object", color_draw=(0, 255, 255)):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_color, upper_color)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(image, (cx, cy), 6, color_draw, -1)
                cv2.putText(image, label, (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                self.get_logger().info(f"[{label}] Detected at ({cx}, {cy})")
                return (cx, cy)
        return None

    def run(self):
        while self.robot.step(self.timestep) != -1:
            # Get image from camera
            width = self.camera.getWidth()
            height = self.camera.getHeight()
            img = self.camera.getImage()

            # Convert Webots image to OpenCV format
            image = np.frombuffer(img, np.uint8).reshape((height, width, 4))
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

            # Deteksi warna robot (merah)
            robot_pos = self.detect_red_position(
                image,
                label="ROBOT",
                color_draw=(0, 0, 255)
            )

            # Publish robot position
            if robot_pos is not None:
                x_world, y_world = pixel_to_world(robot_pos[0], robot_pos[1])
                msg = Point()
                msg.x = x_world
                msg.y = y_world
                msg.z = 0.0
                self.robot_pub.publish(msg)
                self.get_logger().info(f"Robot (world): ({x_world:.2f}, {y_world:.2f})")

            # Deteksi warna goal (biru)
            goal_pos = self.detect_blue_position(
                image,
                self.lower_blue,
                self.upper_blue,
                label="GOAL",
                color_draw=(255, 0, 0)
            )

            # Publish goal position
            if goal_pos is not None:
                x_world, y_world = pixel_to_world(goal_pos[0], goal_pos[1])
                msg = Point()
                msg.x = x_world
                msg.y = y_world
                msg.z = 0.0
                self.goal_pub.publish(msg)
                self.get_logger().info(f"Goal (world): ({x_world:.2f}, {y_world:.2f})")

            # Tampilkan gambar
            cv2.imshow("Camera View", image)
            key = cv2.waitKey(1)
            if key == ord('q'):
                break

def main(args=None):
    print("Vision node launching...")
    import sys
    filtered_args = [arg for arg in sys.argv if not arg.startswith("--webots-robot-name")]
    
    rclpy.init(args=filtered_args)
    node = VisionNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
