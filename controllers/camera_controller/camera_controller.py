from controller import Supervisor, Camera
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point

# Inisialisasi Webots
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# Inisialisasi Kamera
camera = supervisor.getDevice("TopCamera")
camera.enable(timestep)

# Load Dictionary ArUco
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

# Fungsi konversi pixel ke dunia (meter)
def pixel_to_world(x_pixel, y_pixel, width, height, arena_size=4.0):
    scale_x = arena_size / width
    scale_y = arena_size / height
    x_world = (x_pixel - width / 2) * scale_x
    y_world = -(y_pixel - height / 2) * scale_y
    return x_world, y_world

# Inisialisasi ROS2 Node
class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('aruco_publisher')
        self.publisher_ = self.create_publisher(String, 'aruco_data', 10)

    def publish_data(self, data):
        msg = Point()
        msg.x = x_aruco
        msg.y = y_aruco
        msg.z = id_aruco  # Gunakan z sebagai ID marker

        self.publisher_aruco.publish(msg)

rclpy.init()
node = MarkerPublisher()

# Loop utama Webots
while supervisor.step(timestep) != -1:
    img = camera.getImage()

    if img:
        width = camera.getWidth()
        height = camera.getHeight()
        frame = np.frombuffer(img, dtype=np.uint8).reshape((height, width, 4))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            marker_positions = []
            for i in range(len(ids)):
                c = corners[i][0]
                x_marker, y_marker = int(c[:, 0].mean()), int(c[:, 1].mean())

                x_world, y_world = pixel_to_world(x_marker, y_marker, width, height)
                marker_positions.append(f"{ids[i][0]}:{x_world:.2f},{y_world:.2f}")

                cv2.polylines(frame, [c.astype(np.int32)], True, (0, 255, 0), 2)
                cv2.putText(frame, f"ID {ids[i][0]}", (x_marker, y_marker),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Kirim data ke ROS2 dalam format "ID:x,y"
            marker_data = "|".join(marker_positions)
            node.publish_data(marker_data)

        cv2.imshow("Camera View", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()
rclpy.shutdown()




