## Seamless integration of Rangenet and Patchwork 
--- 

```markdown
# 🚀 Patchwork++ and RangeNet++ Integration with ROS 2 (Humble)

This project showcases an advanced 3D LiDAR-based segmentation system, combining **Patchwork++** (ground segmentation) and **RangeNet++** (semantic segmentation) into a unified ROS 2 pipeline. It enables real-time switching between different modes and includes a custom **discrepancy checker** for analyzing mismatches between the two methods.

---

## 🛠️ Setup Instructions

### 1. Install ROS 2 Humble

Follow the [official installation guide](https://docs.ros.org/en/humble/Installation.html) for Ubuntu 22.04.

---

### 2. Create and Prepare Your Workspace

```bash
mkdir -p ~/patchwork_ws/src
cd ~/patchwork_ws
```

---

### 3. Clone Required Repositories

```bash
# Patchwork++
cd ~/patchwork_ws/src
git clone https://github.com/url-kaist/patchwork-plusplus.git

# RangeNet++ ROS2 Integration (your customized version)
git clone https://github.com/your_username/rangenet_ros2.git
```

---

### 4. Download Required Pre-trained Models

**Mandatory Pre-trained Models for RangeNet++:**

- [SemanticKITTI Dataset Models](http://semantic-kitti.org/)
- [squeezeSeg](https://github.com/PRBonn/rangenet_lib)
- [squeezeSeg + CRF](https://github.com/PRBonn/rangenet_lib)
- [squeezeSegV2](https://github.com/PRBonn/rangenet_lib)
- [squeezeSegV2 + CRF](https://github.com/PRBonn/rangenet_lib)
- [darknet21](https://github.com/PRBonn/rangenet_lib)
- [darknet53](https://github.com/PRBonn/rangenet_lib)
- [darknet53-1024](https://github.com/PRBonn/rangenet_lib)
- [darknet53-512](https://github.com/PRBonn/rangenet_lib)

👉 Place the models inside:

```bash
~/patchwork_ws/src/rangenet_ros2/models/
```

### ⚙️ Important:

- To **enable kNN post-processing** (which smooths segmentation outputs), edit the following:
  - Open `models/<your_model>/arch_cfg.yaml`
  - Set the boolean parameter:

```yaml
postproc:
  knn: True
```

### 5. Build the Workspace

```bash
cd ~/patchwork_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

**Explanation:**
- `source /opt/ros/humble/setup.bash` → makes sure ROS 2 environment is ready.
- `rosdep install --from-paths src --ignore-src -r -y` → installs missing ROS 2 package dependencies automatically.
- `colcon build --symlink-install` → builds all packages while keeping links to your source code (makes debugging easy).
- `source install/setup.bash` → makes your newly built packages available.

> ✅ Always **source** the `install/setup.bash` **after building**, before running nodes.

---

---


### 6. Install ROS 2 Dependencies

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---


## 📂 Project Structure

```
patchwork_ws/
├── src/
│   ├── patchwork-plusplus/         # Patchwork++ ground segmentation
│   └── rangenet_ros2/               # Customized RangeNet++ ROS 2 node + Discrepancy Checker
│       ├── node.py
│       ├── discrepancy_checker.py
│       ├── models/
│       └── launch/
├── install/
├── build/
└── log/
```

---

## 🚗 How Patchwork++ and KITTI Dataset Were Used

- **Patchwork++** was originally designed for **urban ground segmentation** under dynamic conditions (moving vehicles, slopes, obstacles).
- We utilized the **KITTI bag file** (pre-recorded driving dataset) to test the system.
- The **Patchwork++ KITTI bag** contains **urban, suburban, and highway scenarios**, making it highly robust for benchmarking ground segmentation.

### ❗ How Patchwork++ Is Different

- Unlike traditional ground segmentation (e.g., RANSAC-based), Patchwork++:
  - Handles uneven terrains (hills, slopes)
  - Deals with moving objects
  - Segments **only ground points** even in highly dynamic scenes
  - Reduces false positives using **elevation and curvature filters**

---

## 📋 What We Have Done

1. **Patchwork++ Setup:**
   - Installed and launched Patchwork++.
   - Subscribed to ROS 2 point cloud topics from KITTI bag.

2. **RangeNet++ Integration:**
   - Created a custom `rangenet_ros2` package.
   - Built a ROS 2 Python node (`node.py`) for:
     - Subscribing to incoming raw point clouds.
     - Performing real-time semantic segmentation using pre-trained models.
     - Publishing segmentation results.

3. **Dynamic Mode Switching:**
   - Added dynamic toggling between:
     - Raw point cloud
     - Patchwork++ segmentation
     - RangeNet++ segmentation
     - Combined overlays
   - Controlled by keyboard shortcuts.

4. **Discrepancy Checker:**
   - Developed a node (`discrepancy_checker.py`) to:
     - Compare Patchwork++ and RangeNet++ outputs at the point level.
     - Highlight **discrepant points** with color in RViz2.
     - Publish a new discrepancy topic `/segmentation_discrepancy`.

---

## 🛎️ How to Run Everything

### 1. Source your environment

```bash
source ~/patchwork_ws/install/setup.bash
```

### 2. Play KITTI Bag

```bash
ros2 bag play your_kitti_dataset.bag --rate 0.5
```

(Adjust `--rate` for better visualization)

### 3. Run Patchwork++

```bash
ros2 launch patchwork patchwork.launch.py
```

### 4. Run RangeNet++

```bash
ros2 run rangenet_ros2 node
```

### 5. Run Discrepancy Checker

```bash
ros2 run rangenet_ros2 discrepancy_checker
```

### 6. Visualize in RViz2

- Add `PointCloud2` display.
- Set topics:
  - `/patchwork/ground`
  - `/rangenet/segmented_points`
  - `/segmentation_discrepancy`
- Frame: `/velodyne` or `/os_cloud` depending on your source.

---

## 🧪 Example Usage Scenarios

- **Urban street detection**
- **Rural and slope-ground detection**
- **Pothole and anomaly detection**
- **Ground vs Non-Ground evaluation in dynamic environments**

---

## 🧩 Known Issues

- Ensure time synchronization between bag file and nodes.
- GPU memory exhaustion may occur for large point clouds.
- RangeNet++ inference speed depends heavily on GPU model.

---
### 🖼️ Now, **Professional System Diagram**

```
                +-----------------+
                |   KITTI ROS Bag  |
                |  (PointCloud2)    |
                +--------+---------+
                         |
                         v
        +--------------------------------+
        |                                |
+------------------+         +---------------------+
|   Patchwork++    |         |     RangeNet++       |
| (Ground Segmentation)      | (Semantic Segmentation) |
+------------------+         +---------------------+
        |                                |
        +--------------------------------+
                         |
                         v
                +------------------------+
                |   Discrepancy Checker   |
                | (Compare + Highlight)   |
                +-----------+------------+
                            |
                            v
                     +-------------+
                     |   RViz2      |
                     | (Visualization) |
                     +-------------+
```

- **KITTI Bag** feeds raw point clouds.
- Data is processed **in parallel** by **Patchwork++** and **RangeNet++**.



