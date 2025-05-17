from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Inisialisasi perangkat
gps = robot.getDevice('global')
gps.enable(timestep)

emitter = robot.getDevice('emitter')
emitter.setChannel(1)  # Channel broadcast

receiver = robot.getDevice('receiver')
receiver.setChannel(1)  # Channel broadcast
receiver.enable(timestep)

while robot.step(timestep) != -1:
    # Membaca lokasi GPS
    position = gps.getValues()
    x, y, z = position[0], position[1], position[2]
    
    # Mengirimkan lokasi
    message = f"Lokasi Agent 1 :{x}, {y}, {z}"
    emitter.send(message.encode('utf-8'))

    # Menerima pesan dari robot lain
    while receiver.getQueueLength() > 0:
        received_message = receiver.getString()
        print(f"Robot 1 received: {received_message}")
        receiver.nextPacket()
