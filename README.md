# SIMULASI PATH PLANNING DENGAN WEBOTS ROS2

Proyek ini merupakan simulasi pergerakan robot menggunakan algoritma **A\* path planning** di lingkungan Webots, dengan komunikasi antar-node menggunakan ROS2.

---

## 💻 Menjalankan Simulasi

Buka **6 tab terminal** dan jalankan perintah berikut:

### 🖥️ TAB 1 – Jalankan Webots
```bash
cd ~/path
source ros2_ws/install/setup.bash
webots worlds/path.wbt
```

### 🤖 TAB 2 – Jalankan Node Path Follower
```bash
cd ros2_ws
source install/setup.bash
ros2 run path_follower path_follower_node
```

### 🧠 TAB 3 – Jalankan Node Path Planner
```bash
cd ros2_ws
source install/setup.bash
ros2 run path_planner path_planner_node
```

### 📊 TAB 4 – Jalankan RViz
```bash
cd ros2_ws
source install/setup.bash
ros2 run rviz2 rviz2
```

### 📷 TAB 5 – Jalankan Node Vision
```bash
cd ros2_ws
source install/setup.bash
ros2 run vision_node vision_node
```

### 🧭 TAB 6 – Jalankan Node Odometry
```bash
cd ros2_ws
source install/setup.bash
ros2 run odometry_node odometry_node
```

---

## 🔍 Mengecek Node Berjalan
Gunakan perintah berikut untuk memeriksa komunikasi antar-node:

```bash
ros2 topic list
ros2 topic echo /<nama_topic>
```

---

## 🔄 Ilustrasi Alur Data

```
[vision_node.py]
      ⬇️  (deteksi posisi robot & goal)
[path_planner_node.py]
      ⬇️  (publikasi jalur ke topic /path)
[path_follower_node.py]
      ⬇️  (publikasi ke /cmd_vel)
[ros2_cmd_vel_controller.py]
      ⬇️  (kendalikan motor Webots)
[Robot di simulasi Webots]
```

---

## 🚀 Urutan Eksekusi
1. Jalankan **Webots**
2. Jalankan **vision_node**
3. Jalankan **path_follower_node**
4. Jalankan **path_planner_node**
