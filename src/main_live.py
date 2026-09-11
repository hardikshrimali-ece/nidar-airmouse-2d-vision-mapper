#!/usr/bin/env python3
"""
NIDAR AirMouse - Live Vision System
===================================
Camera → Visual Odometry → YOLO Detection → Real-time 2D Occupancy Grid
+ Survivor tagging + simple path planning visualization.

Run:
    python3 main_live.py --source 0          # webcam / Pi camera
    python3 main_live.py --source video.mp4  # video file

Press 'q' to quit, 'r' to reset map, 's' to save map image.
"""

import argparse
import time
import cv2
import numpy as np
import sys
import os

# Allow running from src/ or project root
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from occupancy_grid import OccupancyGrid2D
from detector import HumanDummyDetector
from visual_odometry import SimpleVisualOdometry
from path_planner import AStarPlanner


def main():
    parser = argparse.ArgumentParser(description="NIDAR AirMouse Live Vision Mapper")
    parser.add_argument("--source", default="0", help="Camera index or video path")
    parser.add_argument("--model", default="yolov8n.pt", help="YOLO model path")
    parser.add_argument("--conf", type=float, default=0.45, help="Detection confidence")
    parser.add_argument("--device", default="cpu", help="cpu or cuda")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    # Open camera / video
    src = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(src)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    if not cap.isOpened():
        print(f"Cannot open source: {args.source}")
        return

    # Core modules
    grid = OccupancyGrid2D(width_m=20.0, height_m=20.0, resolution=0.05, origin_x=-2.0, origin_y=-2.0)
    detector = HumanDummyDetector(model_path=args.model, conf_thres=args.conf, device=args.device)
    vo = SimpleVisualOdometry(focal=600.0, pp=(args.width / 2, args.height / 2))
    planner = AStarPlanner(grid)

    print("Live system started. Press 'q' to quit, 'r' to reset, 's' to save map.")

    path = None
    last_plan_time = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("End of stream")
            break

        # 1. Visual Odometry → current robot pose
        x, y, yaw = vo.process(frame)

        # 2. Mark free space around robot and a short ray forward
        grid.mark_free(x, y, radius_m=0.4)
        # Simple free-space ray in front of robot
        fx = x + 1.5 * np.cos(yaw)
        fy = y + 1.5 * np.sin(yaw)
        grid.mark_ray_free(x, y, fx, fy)

        # 3. Detect humans / dummies
        detections = detector.detect(frame)
        det_frame = detector.draw(frame, detections)

        # 4. Project detections into map (simple pinhole assumption: bottom of bbox ≈ ground contact)
        h, w = frame.shape[:2]
        for d in detections:
            cx, cy = d["center"]
            # Very rough depth estimate from bbox height (tune for your camera height)
            bbox_h = d["bbox"][3] - d["bbox"][1]
            if bbox_h < 10:
                continue
            # Approximate distance (empirical)
            dist = 1.8 * (h / bbox_h) * 0.4  # tune this constant
            # Direction relative to camera yaw
            angle_offset = ((cx - w / 2) / w) * 1.0  # approx FOV factor
            sx = x + dist * np.cos(yaw + angle_offset)
            sy = y + dist * np.sin(yaw + angle_offset)

            sid = grid.add_or_update_survivor(sx, sy, conf=d["conf"])
            # Mark a small occupied region at survivor (so path avoids them if needed)
            grid.mark_occupied(sx, sy, radius_m=0.25)

        grid.age_survivors()

        # 5. Periodic path planning example (to first survivor or a fixed goal)
        if time.time() - last_plan_time > 2.0:
            if grid.survivors:
                goal = (grid.survivors[0]["x"], grid.survivors[0]["y"])
                path = planner.plan((x, y), goal)
            last_plan_time = time.time()

        # 6. Live visualization
        map_img = grid.get_occupancy_image(scale=3)

        # Draw robot pose on map
        rgx, rgy = grid.world_to_grid(x, y)
        rgx *= 3
        rgy *= 3
        cv2.circle(map_img, (rgx, rgy), 6, (255, 100, 0), -1)
        # Heading line
        hx = int(rgx + 20 * np.cos(yaw))
        hy = int(rgy + 20 * np.sin(yaw))
        cv2.line(map_img, (rgx, rgy), (hx, hy), (255, 100, 0), 2)

        # Draw planned path
        if path:
            for i in range(len(path) - 1):
                p1 = grid.world_to_grid(*path[i])
                p2 = grid.world_to_grid(*path[i + 1])
                cv2.line(map_img, (p1[0]*3, p1[1]*3), (p2[0]*3, p2[1]*3), (0, 200, 255), 2)

        # Status text
        status = f"Pose: ({x:.2f}, {y:.2f}) yaw={np.degrees(yaw):.1f}deg | Survivors: {len(grid.survivors)}"
        cv2.putText(det_frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Show windows
        cv2.imshow("Camera + Detections", det_frame)
        cv2.imshow("Live 2D Occupancy Grid + Path", map_img)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("r"):
            grid = OccupancyGrid2D(width_m=20.0, height_m=20.0, resolution=0.05, origin_x=-2.0, origin_y=-2.0)
            vo.reset()
            path = None
            print("Map & VO reset")
        elif key == ord("s"):
            cv2.imwrite("occupancy_map.png", map_img)
            grid.save_pgm("occupancy_map.pgm")
            print("Map saved")

    cap.release()
    cv2.destroyAllWindows()
    print(f"Final survivors tagged: {len(grid.survivors)}")
    for s in grid.survivors:
        print(f"  S{s['id']}: ({s['x']:.2f}, {s['y']:.2f}) conf={s['conf']:.2f}")


if __name__ == "__main__":
    main()
