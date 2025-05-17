# SIMULASI PATH PLANNING DENGAN WEBOTS ROS2

Proyek ini merupakan simulasi pergerakan robot menggunakan algoritma **A\* path planning** di lingkungan Webots, dengan komunikasi antar-node menggunakan ROS2.

---

## 💻 Menjalankan Simulasi

Buka **5 tab terminal** dan jalankan perintah berikut:

### 🖥️ TAB 1 – Jalankan Webots
```
cd ~/a-star_robot_to_goal
source ros2_ws/install/setup.bash
webots webots/worlds/swarm.wbt
```

### 🤖 TAB 2 – Jalankan Node Path Follower
```
cd ~/a-star_robot_to_goal/ros2_ws
source install/setup.bash
ros2 run path_follower path_follower_node
```

### 🧠 TAB 3 – Jalankan Node Path Planner
```
cd ~/a-star_robot_to_goal/ros2_ws
source install/setup.bash
ros2 run path_planner path_planner_node
```

### 📊 TAB 4 – Jalankan RViz
```
cd ~/a-star_robot_to_goal/ros2_ws
source install/setup.bash
ros2 run rviz2 rviz2
```

### 📷 TAB 5 – Jalankan Node Vision
```
cd ~/a-star_robot_to_goal/ros2_ws
source install/setup.bash
ros2 run vision_node vision_node
```

---
### 📊 Panduan Setting RViz

🧭 1. Ubah Fixed Frame
Pada pojok kanan atas RViz, ubah nilai Fixed Frame menjadi:
```
map
```
➕ 2. Tambahkan Display

Klik tombol Add di panel kiri, lalu tambahkan dan atur display berikut:
📌 Path

    Type: Path

    Topic: /path

    Style: Lines

    Line Width: 0.03

    Color: Biru
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
