# NIDAR AirMouse - Vision-Based 2D Mapping & Survivor Detection System

**Mission**: NIDAR 2.0 Mission 2 – Autonomous GPS-Denied Indoor Search, Mapping and Survivor Localisation (AirMouse)

**Core Constraint**: Camera + Raspberry Pi only. **No LiDAR**.

Live working code is now available under `src/`.

---

## What the Code Does (Live)

| Component | File | Description |
|-----------|------|-------------|
| Real-time 2D Occupancy Grid | `src/occupancy_grid.py` | Builds live 2D grid (free / occupied / unknown). Tags survivors with unique IDs |
| Human + Dummy Detection | `src/detector.py` | YOLOv8/YOLOv11 based detector |
| Visual Odometry | `src/visual_odometry.py` | Camera-based localization (feature tracking + essential matrix) |
| Path Planner | `src/path_planner.py` | A* on the occupancy grid |
| **Main Live System** | `src/main_live.py` | Full pipeline: Camera → VO + YOLO → live occupancy grid + path + visualization |

---

## Quick Start – Run Live System

```bash
# 1. Clone
git clone https://github.com/hardikshrimali-ece/nidar-airmouse-2d-vision-mapper.git
cd nidar-airmouse-2d-vision-mapper

# 2. Install dependencies
pip install -r requirements.txt

# 3. Run (webcam / Pi camera)
cd src
python3 main_live.py --source 0

# Or from a video file
python3 main_live.py --source /path/to/video.mp4

# Optional flags
python3 main_live.py --source 0 --model yolov8n.pt --conf 0.4 --device cpu
```

**Controls while running**:
- `q` → Quit
- `r` → Reset map & odometry
- `s` → Save current occupancy map (`occupancy_map.png` + `.pgm`)

Two windows open:
1. **Camera + Detections** – live video with YOLO boxes
2. **Live 2D Occupancy Grid + Path** – real-time map, robot pose (orange), survivors (red), planned path (cyan)

---

## System Overview

```
Camera → Raspberry Pi / Jetson
         ├── Visual Odometry / vSLAM
         ├── YOLO Human/Dummy Detection
         ├── Real-time 2D Occupancy Grid
         ├── Survivor Tagging (unique IDs)
         └── A* Path Planning + Visualization
```

**Output**:
- Live 2D occupancy grid map
- Survivor markers (ID + position) overlaid on the map
- Path from current pose to nearest survivor (example)

---

## Hardware Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| Compute | Raspberry Pi 5 (8GB) | NVIDIA Jetson Orin Nano |
| Camera | Pi Camera Module 3 / USB 1080p | Stereo pair or RealSense |
| IMU | MPU6050 / BNO055 | ICM-20948 |
| FC | Any PX4 / ArduPilot board | - |

**No LiDAR is used.**

---

## Upgrading to Production-Grade vSLAM

The included `SimpleVisualOdometry` is good for demos and short-range indoor testing. For real competition performance replace it with:

- **ORB-SLAM3** (mono / stereo / inertial) – best accuracy
- **RTAB-Map** (easy ROS 2 integration)
- **Isaac ROS Visual SLAM** (if using Jetson)

The occupancy grid, detector, and planner stay the same – just feed better pose estimates into `grid.mark_free()` and survivor projection.

---

## Architecture (Current Code)

```
[Camera Frame]
      ↓
[SimpleVisualOdometry] → (x, y, yaw)
      ↓
[OccupancyGrid2D] ← mark free space around robot + forward ray
      ↓
[HumanDummyDetector (YOLO)] → detections
      ↓
Project detections → world coordinates → add_or_update_survivor()
      ↓
[AStarPlanner] → path to nearest survivor (example)
      ↓
Live OpenCV windows (camera + map)
```

---

## Next Development Steps

1. Replace simple VO with ORB-SLAM3 / RTAB-Map (ROS 2)
2. Fine-tune YOLO on real rescue dummies
3. Add proper camera-to-ground projection using camera height + pitch
4. Integrate with flight controller (MAVLink / ROS 2 control)
5. Add frontier-based exploration planner
6. Full mission state machine (Explore → Tag → Return Home)

---

## Scoring Alignment (NIDAR AirMouse)

- Generate and display a 2D map of the explored area ✅
- Detect and tag up to 6 survivors (humans or dummies) ✅
- Fully autonomous GPS-denied navigation (path planning ready) ✅
- Return to start / exit the maze (path planner supports it)
- Mission time ≤ 30 minutes

---

**Good luck with NIDAR Mission 2 – AirMouse!**

Repo: https://github.com/hardikshrimali-ece/nidar-airmouse-2d-vision-mapper
