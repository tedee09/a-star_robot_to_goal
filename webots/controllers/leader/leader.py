#robot1 as leader setelah edit
from controller import Robot, Camera, GPS, InertialUnit, Emitter
import random
import json

TIME_STEP = 64

# Inisialisasi robot dan timestep
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Inisialisasi aktuator (motor)
motor_kanan_depan = robot.getDevice("motor_kanan_depan")
motor_kanan_belakang = robot.getDevice("motor_kanan_belakang")
motor_kiri_depan = robot.getDevice("motor_kiri_depan")
motor_kiri_belakang = robot.getDevice("motor_kiri_belakang")

# Atur mode motor ke velocity
motors = [motor_kanan_depan, motor_kanan_belakang, motor_kiri_depan, motor_kiri_belakang]
for motor in motors:
    motor.setPosition(float('inf'))  # Non-positional mode
    motor.setVelocity(0.0)          # Awalnya berhenti

# Inisialisasi sensor jarak (distance sensor)
ds_depan = robot.getDevice("ds_depan")
ds_kanan = robot.getDevice("ds_kanan")
ds_kiri = robot.getDevice("ds_kiri")
ds_belakang = robot.getDevice("ds_belakang")
distance_sensors = [ds_depan, ds_kanan, ds_kiri, ds_belakang]

# Aktifkan semua sensor jarak
for ds in distance_sensors:
    ds.enable(timestep)

# Inisialisasi GPS dan IMU
gps = robot.getDevice("global")
gps.enable(timestep)

imu = robot.getDevice("imu")
imu.enable(timestep)

# Inisialisasi kamera
camera = robot.getDevice("cam")
camera.enable(timestep)
camera.recognitionEnable(TIME_STEP)
camera_width = camera.getWidth()

# Inisialisasi emitter
emitter = robot.getDevice("emitter")
emitter.setChannel(1)  # Channel broadcast
emitter.setRange(-1)  # Tidak ada batasan jarak pengiriman

# Inisialisasi receiver
receiver = robot.getDevice("receiver")
receiver.setChannel(1)  # Channel broadcast
receiver.enable(timestep)

#focal length dan ukuran objek (cuma nebak aja njir)
focal_length = 1000
known_size = 0.0000137

chosen_direction = None # Variabel untuk menyimpan arah putar saat menghadapi rintangan

# Fungsi untuk mengatur kecepatan motor
def set_motor_velocity(left_speed, right_speed):
    motor_kiri_depan.setVelocity(left_speed)
    motor_kiri_belakang.setVelocity(left_speed)
    motor_kanan_depan.setVelocity(right_speed)
    motor_kanan_belakang.setVelocity(right_speed)

# Program utama
def main():
    global chosen_direction # variabel global untuk menyimpan arah putar

    while robot.step(timestep) != -1:
        
        # Baca data sensor jarak
        depan_value = ds_depan.getValue()
        kanan_value = ds_kanan.getValue()
        kiri_value = ds_kiri.getValue()
        belakang_value = ds_belakang.getValue()

        # Baca posisi GPS dan yaw (orientasi)
        current_pos = gps.getValues()  # [x, y, z]
        yaw = imu.getRollPitchYaw()[2]  # Ambil yaw (rotasi horizontal)
        data_to_send = {
            "x": current_pos[0],  # Koordinat x
            "y": current_pos[1],  # Koordinat y (horizontal)
            "yaw": yaw            # Sudut yaw
        }
        
        # Membaca lokasi GPS
        position = gps.getValues()
        x, y, z = position[0], position[1], position[2]
        
        # Format pesan untuk dikirim ke `robot2`
        message = json.dumps(data_to_send)
        emitter.send(message.encode("utf-8"))
        print(f"Mengirim koordinat GPS dan yaw: {data_to_send}")

        if depan_value < 1000:  # Jika ada rintangan di depan
            if chosen_direction  is None: # Pilih arah hanya sekali
                chosen_direction  = random.choice(["left", "right"])
            
            if chosen_direction  == "left":
                set_motor_velocity(-6.0, 6.0)  # Berputar
                # emitter.send(message.encode("utf-8"))  # Kirim pesan ke `robot2`
            else:
                set_motor_velocity(6.0, -6.0)  # Berputar
                # emitter.send(message.encode("utf-8"))  # Kirim pesan ke `robot2`
        elif kiri_value < 1000:  # Jika ada rintangan di kiri
            set_motor_velocity(6.0, 4.5)  # Belok kanan
            # emitter.send(message.encode("utf-8"))  # Kirim pesan ke `robot2`
        elif kanan_value < 1000:  # Jika ada rintangan di kanan
            set_motor_velocity(4.5, 6.0)  # Belok kiri
            # emitter.send(message.encode("utf-8"))  # Kirim pesan ke `robot2`
        else:
            pass
            print("Memutar memantau sekitar.")
            rotation_direction = random.choice(["left", "right"])  # Arah putar acak
            rotation_time = random.randint(10, 100)  # Durasi memutar acak dalam iterasi
            forward_time = random.randint(170, 900)  # Durasi bergerak lurus acak dalam iterasi
        
            for _ in range(rotation_time):
                if rotation_direction == "left":
                    set_motor_velocity(-2.0, 2.0)  # Berputar ke kiri
                    emitter.send(message.encode("utf-8"))  # Kirim pesan ke `robot2`
                else:
                    set_motor_velocity(2.0, -2.0)  # Berputar ke kanan
                    emitter.send(message.encode("utf-8"))  # Kirim pesan ke `robot2`
        
                robot.step(timestep)

            forward_time = random.randint(40, 500)
            for _ in range(forward_time):
                depan_value = ds_depan.getValue()
                kanan_value = ds_kanan.getValue()
                kiri_value = ds_kiri.getValue()
        
                
                if depan_value < 1000:  # Jika ada rintangan di depan
                    if chosen_direction  is None: # Pilih arah hanya sekali
                        chosen_direction  = random.choice(["left", "right"])
                    
                    if chosen_direction  == "left":
                        set_motor_velocity(-6.0, 6.0)  # Berputar
                        emitter.send(message.encode("utf-8"))  # Kirim pesan ke `robot2`
                    else:
                        set_motor_velocity(6.0, -6.0)  # Berputar
                        emitter.send(message.encode("utf-8"))  # Kirim pesan ke `robot2`
                elif kiri_value < 1000:  # Jika ada rintangan di kiri
                    set_motor_velocity(6.0, 4.5)  # Belok kanan
                    emitter.send(message.encode("utf-8"))  # Kirim pesan ke `robot2`
                elif kanan_value < 1000:  # Jika ada rintangan di kanan
                    set_motor_velocity(4.5, 6.0)  # Belok kiri
                    emitter.send(message.encode("utf-8"))  # Kirim pesan ke `robot2`
                else:
                    set_motor_velocity(6.0, 6.0)
                    emitter.send(message.encode("utf-8"))  # Kirim pesan ke `robot2`
        
                robot.step(timestep)

if __name__ == "__main__":
    main()