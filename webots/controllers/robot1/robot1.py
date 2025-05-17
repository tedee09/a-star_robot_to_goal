import math
import heapq
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Inisialisasi aktuator (motor)
motor_kanan_depan = robot.getDevice("motor_kanan_depan")
motor_kiri_depan = robot.getDevice("motor_kiri_depan")
motor_kiri_belakang = robot.getDevice("motor_kiri_belakang")
motor_kanan_belakang = robot.getDevice("motor_kanan_belakang")

motors = [motor_kanan_depan, motor_kiri_depan, motor_kiri_belakang, motor_kanan_belakang]
for motor in motors:
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)

# Inisialisasi GPS dan IMU
gps = robot.getDevice("global")
gps.enable(timestep)

imu = robot.getDevice("imu")
imu.enable(timestep)

# ==== 1️⃣ PATH PLANNING: A-STAR ALGORITHM ====
def a_star(start, goal, obstacles, grid_size=0.2):  # Grid lebih besar agar lebih cepat
    def heuristic(a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for dx, dy in [(0, grid_size), (0, -grid_size), (grid_size, 0), (-grid_size, 0)]:
            neighbor = (round(current[0] + dx, 2), round(current[1] + dy, 2))
            if neighbor in obstacles:
                continue

            tentative_g_score = g_score[current] + heuristic(current, neighbor)
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_list, (f_score[neighbor], neighbor))

    return []

# ==== 2️⃣ ROBOT MOVEMENT ====
def move_to_waypoints(waypoints):
    angle_kp = 1.5  
    max_speed = 5.0  
    tolerance = 0.1  

    for target_x, target_y in waypoints:
        timeout = 500  # Tambahkan batas waktu (500 iterasi Webots)
        count = 0  

        while robot.step(timestep) != -1:
            current_pos = gps.getValues()
            current_x = current_pos[0]
            current_y = current_pos[1]
            current_angle = imu.getRollPitchYaw()[2]

            distance_to_target = euclidean_distance([current_x, current_y], [target_x, target_y])
            angle_to_target = get_angle_to_target([current_x, current_y], [target_x, target_y])
            angle_error = angle_to_target - current_angle

            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

            # Jika sudah sampai atau waktu habis, berhenti
            if distance_to_target < tolerance or count > timeout:
                set_motor_velocity(0.0, 0.0)
                print(f"Sampai di waypoint: ({target_x}, {target_y})")
                break  

            # Kontrol kecepatan
            correction = angle_kp * angle_error
            left_speed = max_speed - correction
            right_speed = max_speed + correction

            left_speed = max(min(left_speed, max_speed), -max_speed)
            right_speed = max(min(right_speed, max_speed), -max_speed)

            set_motor_velocity(left_speed, right_speed)

            robot.step(timestep * 2)  # **Kurangi frekuensi eksekusi** agar lebih ringan
            count += 1  # Hitung iterasi

# ==== 3️⃣ HELPER FUNCTIONS ====
def euclidean_distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

def get_angle_to_target(current_pos, target_pos):
    delta_x = target_pos[0] - current_pos[0]
    delta_y = target_pos[1] - current_pos[1]
    return math.atan2(delta_y, delta_x)

def set_motor_velocity(left_speed, right_speed):
    motor_kiri_depan.setVelocity(left_speed)
    motor_kiri_belakang.setVelocity(left_speed)
    motor_kanan_depan.setVelocity(right_speed)
    motor_kanan_belakang.setVelocity(right_speed)

# ==== 4️⃣ MAIN FUNCTION ====
def main():
    start_x, start_y = gps.getValues()[0], gps.getValues()[1]  
    target_x, target_y = 1.0, 1.0  

    # Contoh rintangan
    obstacles = {(0.5, 0.5), (0.7, 0.8)}  

    # Hitung path hanya sekali
    path = a_star((round(start_x, 1), round(start_y, 1)), (round(target_x, 1), round(target_y, 1)), obstacles)

    if path:
        print("Path ditemukan:", path)
        move_to_waypoints(path)
    else:
        print("Tidak ada path yang ditemukan!")

if __name__ == "__main__":
    main()
