#!/usr/bin/env python3
"""
Lightweight Visual Odometry (feature tracking + essential matrix)
Good enough for demo / indoor short-range. Replace with ORB-SLAM3 / RTAB-Map for production.
"""

import cv2
import numpy as np
from typing import Optional, Tuple


class SimpleVisualOdometry:
    def __init__(self, focal: float = 700.0, pp: Tuple[float, float] = (320.0, 240.0)):
        self.focal = focal
        self.pp = pp  # principal point

        self.detector = cv2.ORB_create(nfeatures=1500)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        self.prev_gray = None
        self.prev_kps = None
        self.prev_des = None

        # Current pose in 2D (x, y, yaw) - assume camera looking forward, ground plane motion
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Scale is unknown in pure monocular VO → we use a fixed scale factor (tune it)
        self.scale = 0.05  # meters per unit of translation estimate

    def process(self, frame: np.ndarray) -> Tuple[float, float, float]:
        """
        Process a new frame. Returns current (x, y, yaw) in meters / radians.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        kps, des = self.detector.detectAndCompute(gray, None)

        if self.prev_gray is None or des is None or self.prev_des is None:
            self.prev_gray = gray
            self.prev_kps = kps
            self.prev_des = des
            return self.x, self.y, self.yaw

        # Match
        matches = self.matcher.knnMatch(self.prev_des, des, k=2)
        good = []
        for m_n in matches:
            if len(m_n) == 2:
                m, n = m_n
                if m.distance < 0.75 * n.distance:
                    good.append(m)

        if len(good) < 12:
            self.prev_gray = gray
            self.prev_kps = kps
            self.prev_des = des
            return self.x, self.y, self.yaw

        pts1 = np.float32([self.prev_kps[m.queryIdx].pt for m in good])
        pts2 = np.float32([kps[m.trainIdx].pt for m in good])

        E, mask = cv2.findEssentialMat(
            pts1, pts2,
            focal=self.focal,
            pp=self.pp,
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.0,
        )

        if E is None:
            self.prev_gray = gray
            self.prev_kps = kps
            self.prev_des = des
            return self.x, self.y, self.yaw

        _, R, t, mask = cv2.recoverPose(E, pts1, pts2, focal=self.focal, pp=self.pp)

        # Extract yaw change and forward motion (simplified planar assumption)
        # R is rotation of camera; for ground robot / low-altitude drone we take yaw
        dyaw = np.arctan2(R[1, 0], R[0, 0])  # approximate

        # Translation direction (camera z is forward)
        tx, ty, tz = t.flatten()
        # Prefer forward component
        forward = tz if abs(tz) > abs(tx) else tx

        # Update pose (very simplified)
        self.yaw += dyaw * 0.5  # dampen
        dist = self.scale * np.linalg.norm(t)
        self.x += dist * np.cos(self.yaw)
        self.y += dist * np.sin(self.yaw)

        self.prev_gray = gray
        self.prev_kps = kps
        self.prev_des = des

        return self.x, self.y, self.yaw

    def reset(self):
        self.x = self.y = self.yaw = 0.0
        self.prev_gray = None
