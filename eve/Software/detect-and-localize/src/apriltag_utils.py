import cv2
import numpy as np
from pupil_apriltags import Detector

detector = Detector(families="tag16h5")

def load_tag_positions(csvfile):
    tags = {}
    with open(csvfile) as f:
        next(f)
        for line in f:
            tid, x, y, z = line.strip().split(',')
            tags[int(tid)] = np.array([float(x), float(y)])
    return tags

def detect_tags(gray):
    return [r for r in detector.detect(gray) if r.decision_margin >= 15]

def estimate_homography(detections, tag_world_positions):
    img_pts, world_pts = [], []
    for d in detections:
        tid = int(d.tag_id)
        if tid not in tag_world_positions:
            continue
        img_pts.append(d.center)
        world_pts.append(tag_world_positions[tid])
    if len(img_pts) >= 4:
        H, _ = cv2.findHomography(np.array(img_pts, np.float32),
                                  np.array(world_pts, np.float32),
                                  cv2.RANSAC, 3.0)
        return H
    return None
