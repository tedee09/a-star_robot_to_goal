import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
import heapq
from builtin_interfaces.msg import Time
import time
import math
from tf_transformations import quaternion_from_euler

def pixel_to_world(x_pixel, y_pixel):
    x_world = 0.00250 * x_pixel + 0.00000 * y_pixel - 1.00249
    y_world = 0.00000 * x_pixel - 0.00116 * y_pixel + 0.72330
    return x_world, y_world

def world_to_grid(x_world, y_world, grid_width=20, grid_height=15, world_width=1.86, world_height=0.68):
    grid_x = int((x_world + world_width / 2) / world_width * grid_width)
    grid_y = int((world_height - y_world) / world_height * grid_height)
    return (grid_y, grid_x)

def grid_to_world(grid_y, grid_x, grid_width=20, grid_height=15, world_width=1.86, world_height=0.68):
    x_world = (grid_x + 0.5) * (world_width / grid_width) - (world_width / 2)
    y_world = world_height - ((grid_y + 0.5) * (world_height / grid_height))
    return x_world, y_world

def inflate_obstacles(grid, radius=1):
    rows = len(grid)
    cols = len(grid[0])
    inflated = [row[:] for row in grid]  # duplikat grid

    for y in range(rows):
        for x in range(cols):
            if grid[y][x] == 1:
                for dy in range(-radius, radius + 1):
                    for dx in range(-radius, radius + 1):
                        ny = y + dy
                        nx = x + dx
                        if 0 <= ny < rows and 0 <= nx < cols:
                            inflated[ny][nx] = 1
    return inflated


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        original_grid = [
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ]

        self.grid = inflate_obstacles(original_grid, radius=1)
        # self.get_logger().info("✅ Grid setelah obstacle inflation:")
        # for row in self.grid:
        #     self.get_logger().info(''.join(str(cell) for cell in row))

        self.robot_pos = None
        self.goal_pos = None

        self.subscription_robot = self.create_subscription(
            Point,
            'robot_position',
            self.robot_callback,
            10
        )

        self.subscription_goal = self.create_subscription(
            Point,
            'goal_position',
            self.goal_callback,
            10
        )

        self.path_pub = self.create_publisher(Path, 'path', 10)

        self.get_logger().info("Path Planner Node Started.")

    def robot_callback(self, msg):
        new_robot_pos = world_to_grid(msg.x, msg.y)
    
        # Jika robot benar-benar pindah, baru generate ulang path
        if new_robot_pos != self.robot_pos:
            self.robot_pos = new_robot_pos
            self.check_and_plan()

    def goal_callback(self, msg):
        new_goal = world_to_grid(msg.x, msg.y)

        if new_goal != self.goal_pos:
            self.goal_pos = new_goal
            self.check_and_plan()

    def check_and_plan(self):
        if self.robot_pos is None or self.goal_pos is None:
            return

        self.get_logger().info(f"[GRID COORD] Robot: {self.robot_pos}, Goal: {self.goal_pos}")
        
        if self.robot_pos == self.goal_pos:
            self.get_logger().info("Start dan goal sama. Tidak perlu path.")
            return

        path_points = a_star(self.grid, self.robot_pos, self.goal_pos)
            
        if path_points:
            path_msg = Path()
            path_msg.header.frame_id = "map"

            if len(path_points) < 2:
                self.get_logger().warn("Path terlalu pendek. Abaikan.")
                return
            
            for i, (x, y) in enumerate(path_points):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = self.get_clock().now().to_msg()
                
                x_world, y_world = grid_to_world(x, y)
                self.get_logger().info(f"PATH POINT (grid): ({x},{y}) → (world): ({x_world:.2f},{y_world:.2f})")
                pose.pose.position.x = x_world
                pose.pose.position.y = y_world
                pose.pose.position.z = 0.0

                # Hitung orientasi (yaw) berdasarkan titik selanjutnya
                if i < len(path_points) - 1:
                    next_x, next_y = path_points[i+1]
                    dx = next_y - y
                    dy = next_x - x
                    yaw = math.atan2(dy, dx)
                else:
                    yaw = 0.0  # default arah akhir

                # Konversi yaw ke quaternion
                qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw

                path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)
            time.sleep(0.1) 
            self.get_logger().info(f"Published new path with {len(path_points)} points.")
            self.get_logger().info(f"Path generated with {len(path_points)} points.")
        else:
            self.get_logger().warn("No path found!")

def a_star(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    def h(p1, p2):
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])  # heuristic: Manhattan distance

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current:
                path.append(current)
                current = came_from.get(current)
            return path[::-1]

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            neighbor = (current[0]+dx, current[1]+dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue
                tentative_g = g_score[current] + 1
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + h(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))

    return []  # no path found

def main(args=None):
    print("path planner node launching...")
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
