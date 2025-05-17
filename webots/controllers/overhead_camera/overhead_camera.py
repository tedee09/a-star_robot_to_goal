from controller import Supervisor

supervisor = Supervisor()
camera = supervisor.getDevice("OverheadCamera")
camera.enable(50)  # Refresh setiap 50ms

while supervisor.step(50) != -1:
    image = camera.getImage()  # Ambil gambar dari kamera
