# NIDAR AirMouse - Vision-Based 2D Mapping & Survivor Detection System

**Mission**: NIDAR 2.0 Mission 2 – Autonomous GPS-Denied Indoor Search, Mapping and Survivor Localisation (AirMouse)

**Core Constraint**: Camera + Raspberry Pi only. **No LiDAR**.

This repository provides a complete architecture, hardware list, software stack, and implementation guide for a fully vision-based system that:

- Performs real-time **2D Visual SLAM** (Simultaneous Localization and Mapping)
- Detects humans and dummies
- Tags detected survivors on a 2D occupancy grid
- Enables autonomous exploration, path planning, and navigation in a maze-like indoor environment
- Meets NIDAR AirMouse requirements (15×15 m arena, up to 6 survivors, 2D map generation, same entry/exit, fully autonomous)

---

## 1. System Overview

```
Camera → Raspberry Pi (or Jetson Nano/Orin Nano preferred for performance)
         ├── Visual SLAM (ORB-SLAM3 / RTAB-Map / OpenVSLAM / Isaac ROS Visual SLAM)
         ├── Object Detection (YOLOv8 / YOLOv11 nano or small)
         ├── Occupancy Grid Builder
         ├── Survivor Tracker & Geotagger
         └── Navigation Stack (Nav2 or custom A*/DWA)
```

**Output**:
- Live 2D occupancy grid map
- Survivor markers (ID + position) overlaid on the map
- Autonomous exploration + return-to-home path

---

## 2. Hardware Requirements (Minimum Viable + Recommended)

| Component              | Minimum                          | Recommended                          | Notes |
|------------------------|----------------------------------|--------------------------------------|-------|
| Compute                | Raspberry Pi 5 (8GB)            | NVIDIA Jetson Orin Nano / Xavier NX | Pi 5 works but marginal for real-time SLAM + YOLO |
| Camera                 | Raspberry Pi Camera Module 3 or USB 1080p | Stereo pair (2× IMX219 or Arducam) or Intel RealSense D435i (depth optional) | Mono works; stereo/VIO much better |
| IMU                    | MPU6050 / BNO055                | ICM-20948 or RealSense built-in     | Critical for VIO / visual-inertial SLAM |
| Flight Controller      | Pixhawk 6C / Cube Orange / SpeedyBee | Any PX4 or ArduPilot compatible     | For drone; or differential drive for ground robot |
| Power                  | 5V/5A for Pi + camera           | Proper BEC + LiPo                   | |
| Optional               | Optical Flow (PX4FLOW / Matek)  | Helps low-texture floors            | |

**No LiDAR is used.** All ranging and mapping comes from monocular/stereo visual odometry + IMU.

---

## 3. Software Stack

### Core Packages
- **ROS 2** (Humble or Jazzy) – recommended middleware
- **Visual SLAM**:
  - Preferred: [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) (monocular / stereo / inertial)
  - Alternative: RTAB-Map, OpenVSLAM, Isaac ROS Visual SLAM (if Jetson)
- **Object Detection**: Ultralytics YOLOv8n / YOLOv11n (fine-tuned on humans + dummies)
- **Mapping**: Convert SLAM poses + free-space estimation into 2D occupancy grid (`nav_msgs/OccupancyGrid`)
- **Navigation**: Nav2 (ROS 2) or custom A* + Dynamic Window Approach
- **Survivor Management**: Custom node that projects detections into map frame and maintains persistent tags

### Key Algorithms
1. **Visual-Inertial Odometry (VIO)** for robust pose estimation in low-texture / GPS-denied indoor spaces
2. **Loop Closure** for drift correction
3. **Occupancy Grid Mapping** from projected free space / obstacle features
4. **Multi-object Tracking** (ByteTrack or SORT) to avoid double-counting survivors
5. **Exploration Planner** (frontier-based or coverage path)
6. **Path Planning** (A* / Hybrid A* / Nav2 DWB)

---

## 4. Architecture

```
[Camera + IMU]
       ↓
[Visual SLAM Node] → Pose + Sparse Map / Point Cloud
       ↓
[Occupancy Grid Builder] → 2D Grid (resolution 5–10 cm)
       ↓
[YOLO Detector] → Bounding boxes of humans/dummies
       ↓
[Projection + Tracker] → Survivor positions in map frame + unique IDs
       ↓
[Mission Planner]
   ├── Exploration (cover arena)
   ├── Survivor tagging
   └── Return to launch / Exit
       ↓
[Local Planner + Controller] → Velocity commands to FC / motors
```

---

## 5. Implementation Roadmap

### Phase 1 – Perception Foundation (Week 1-2)
1. Set up Raspberry Pi / Jetson with ROS 2
2. Calibrate camera + IMU (Kalibr or imu_utils)
3. Run ORB-SLAM3 (or RTAB-Map) in mono/stereo/inertial mode
4. Verify loop closure and map quality in a test room

### Phase 2 – Detection & Tagging (Week 2-3)
1. Collect / download dataset of humans + rescue dummies in indoor settings
2. Fine-tune YOLOv8n
3. Write ROS node that:
   - Takes detections
   - Uses current camera pose + camera intrinsics to project to ground plane / map frame
   - Maintains a list of unique survivors (position + confidence + ID)
4. Overlay markers on the 2D map (RViz / custom GCS)

### Phase 3 – Mapping & Occupancy (Week 3)
1. Convert SLAM output into `nav_msgs/OccupancyGrid`
2. Fuse free-space estimation (from optical flow or simple ground plane assumption)
3. Publish map at 1–5 Hz

### Phase 4 – Navigation & Autonomy (Week 4)
1. Integrate Nav2 or implement simple A* + pure pursuit / DWA
2. Frontier-based exploration or systematic lawnmower / wall-following adapted for indoor maze
3. Mission state machine: Explore → Detect & Tag → Return-to-home
4. Fail-safes: low battery, stuck detection, emergency stop

### Phase 5 – Integration & Tuning (Week 5+)
- Full system test in mock maze
- Optimize for real-time on target hardware
- Tune detection confidence thresholds and tracking
- Generate final 2D map with survivor tags for scoring

---

## 6. Why No LiDAR Works (and Challenges)

**Advantages**:
- Lower cost, lower weight, lower power
- Rich semantic information (detect humans vs walls)
- Works with existing camera already needed for survivor detection

**Challenges & Mitigations**:
- Textureless walls / floors → Use strong IMU + optical flow + careful feature selection
- Scale drift (monocular) → Prefer stereo or strong VIO
- Lighting changes → Good exposure control + robust descriptors (ORB, SuperPoint)
- Compute load on Pi → Use quantized YOLO + efficient SLAM, or upgrade to Jetson

---

## 7. Quick Start (High-Level)

```bash
# On Raspberry Pi / Jetson with ROS 2
sudo apt install ros-humble-desktop
# Install ORB-SLAM3 / RTAB-Map / your chosen SLAM
# Install Ultralytics
pip install ultralytics

# Clone this repo
git clone https://github.com/hardikshrimali-ece/nidar-airmouse-2d-vision-mapper.git
cd nidar-airmouse-2d-vision-mapper

# Follow docs/ for detailed setup, calibration, and launch files
```

Detailed launch files, calibration procedures, and example nodes will be added in subsequent commits.

---

## 8. Scoring Alignment (NIDAR AirMouse)

- Generate and display a 2D map of the explored area
- Detect and tag up to 6 survivors (humans or dummies)
- Fully autonomous GPS-denied navigation
- Return to start / exit the maze
- Mission time ≤ 30 minutes

This stack is designed specifically to satisfy these objectives without LiDAR.

---

## 9. Contributing & Next Steps

This is a living project. Planned additions:
- Complete ROS 2 package structure
- Docker image for easy deployment
- Pre-trained YOLO weights for rescue dummies
- Example bag files and map outputs
- Tuning guide for different lighting / floor textures

Feel free to open issues or PRs.

**Good luck with NIDAR Mission 2 – AirMouse!**

---

*Built for students and teams competing in MeitY NIDAR 2.0*
