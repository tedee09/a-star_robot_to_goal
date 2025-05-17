import math
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

# Fungsi menghitung jarak Euclidean
def euclidean_distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

# Fungsi untuk menghitung sudut menuju target
def get_angle_to_target(current_pos, target_pos):
    delta_x = target_pos[0] - current_pos[0]
    delta_y = target_pos[1] - current_pos[1]  # Menggunakan koordinat y untuk arah horizontal
    return math.atan2(delta_y, delta_x)

# Fungsi untuk menggerakkan robot ke target
def move_to_target(target_x, target_y):
    angle_kp = 2.0  # Faktor proporsional sudut
    max_speed = 6.0  # Kecepatan maksimum

    while robot.step(timestep) != -1:
        # Baca posisi GPS
        current_pos = gps.getValues()  # [x, y, z]
        current_x = current_pos[0]  # Koordinat x
        current_y = current_pos[1]  # Koordinat y (horizontal)
        current_angle = imu.getRollPitchYaw()[2]  # Mengambil yaw (rotasi horizontal)

        # Hitung jarak dan sudut
        distance_to_target = euclidean_distance([current_x, current_y], [target_x, target_y])
        angle_to_target = get_angle_to_target([current_x, current_y], [target_x, target_y])
        angle_error = angle_to_target - current_angle

        # Koreksi sudut agar dalam rentang [-pi, pi]
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        # Berhenti jika sudah dekat dengan target
        if distance_to_target < 0.01:  # Jarak threshold untuk berhenti
            set_motor_velocity(0.0, 0.0)
            print(f"Sudah sampai di target: ({target_x}, {target_y})")
            break

        # Kontrol kecepatan berdasarkan sudut error
        correction = angle_kp * angle_error
        left_speed = 5.0 - correction
        right_speed = 5.0 + correction

        # Batas kecepatan
        left_speed = max(min(left_speed, max_speed), -max_speed)
        right_speed = max(min(right_speed, max_speed), -max_speed)

        set_motor_velocity(left_speed, right_speed)

# Fungsi mengatur kecepatan roda
def set_motor_velocity(left_speed, right_speed):
    motor_kiri_depan.setVelocity(left_speed)
    motor_kiri_belakang.setVelocity(left_speed)
    motor_kanan_depan.setVelocity(right_speed)
    motor_kanan_belakang.setVelocity(right_speed)

# Main program
def main():
    target_x = 1.0  # Koordinat X tujuan
    target_y = 1.0  # Koordinat Y tujuan
    move_to_target(target_x, target_y)

if __name__ == "__main__":
    main()
