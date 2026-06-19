#!/usr/bin/env python3
import sys
sys.path.insert(0, '/usr/lib/python3/dist-packages')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
import os


class ViewerNode(Node):
    def __init__(self):
        super().__init__('viewer_node')
        self.latest_frame     = None
        self.confirmed_tracks = {}

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.create_subscription(
            CompressedImage, 'camera/image_raw/compressed',
            self.image_cb, qos)
        self.create_subscription(
            Float32MultiArray, '/trash_coords',
            self.coords_cb, 10)
        self.get_logger().info(f'Viewer ready (cv2 {cv2.__version__}) — press Q to quit')

    def image_cb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            self.latest_frame = frame

    def coords_cb(self, msg):
        flat = list(msg.data)
        new = {}
        for i in range(0, len(flat) - 1, 2):
            new[i // 2] = np.array([flat[i], flat[i + 1]])
        self.confirmed_tracks = new


def draw_camera(frame, tracks):
    out = frame.copy()
    h, w = out.shape[:2]
    for tid, pos in tracks.items():
        px = int(pos[0] * w)
        py = int((1.0 - pos[1]) * h)
        cv2.circle(out, (px, py), 14, (0, 255, 0), 2)
        cv2.putText(out, f'#{tid}', (px + 16, py + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(out, f'Confirmed: {len(tracks)}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    return out


def draw_map(tracks):
    size, margin = 600, 50
    canvas = np.ones((size, size, 3), np.uint8) * 240
    inner  = size - 2 * margin

    for i in range(1, 5):
        v = int(margin + i * inner / 5)
        cv2.line(canvas, (v, margin), (v, size-margin), (200,200,200), 1)
        cv2.line(canvas, (margin, v), (size-margin, v), (200,200,200), 1)

    cv2.rectangle(canvas, (margin, margin),
                  (size-margin, size-margin), (120,120,120), 2)
    cv2.putText(canvas, '(0,0)', (margin-5, size-margin+15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (80,80,80), 1)
    cv2.putText(canvas, '(1,1)', (size-margin-25, margin-5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (80,80,80), 1)

    for tid, pos in tracks.items():
        cx = int(margin + float(pos[0]) * inner)
        cy = int(margin + (1.0 - float(pos[1])) * inner)
        cv2.circle(canvas, (cx, cy), 10, (0, 180, 0), -1)
        cv2.circle(canvas, (cx, cy), 10, (0, 80, 0), 2)
        cv2.putText(canvas, f'#{tid} ({pos[0]:.2f},{pos[1]:.2f})',
                    (cx+12, cy+5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,80,0), 1)

    cv2.circle(canvas, (margin+10, 20), 7, (0,180,0), -1)
    cv2.putText(canvas, f'Confirmed trash: {len(tracks)}',
                (margin+22, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,0), 1)
    return canvas


def main(args=None):
    rclpy.init(args=args)
    node = ViewerNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    # Create named windows explicitly with NORMAL flag so they're resizable
    # and GTK actually allocates a real backing buffer
    cv2.namedWindow('Camera Feed', cv2.WINDOW_NORMAL)
    cv2.namedWindow('World Map — confirmed trash', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Camera Feed', 960, 540)
    cv2.resizeWindow('World Map — confirmed trash', 600, 600)

    blank = np.ones((600, 600, 3), np.uint8) * 60
    cv2.putText(blank, 'Waiting for stream...', (80, 300),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (180, 180, 180), 2)
    cv2.imshow('Camera Feed', blank)
    cv2.imshow('World Map — confirmed trash', blank.copy())
    # Pump the event loop several times to force GTK to actually paint
    for _ in range(10):
        cv2.waitKey(50)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)

            if node.latest_frame is not None:
                cv2.imshow('Camera Feed',
                           draw_camera(node.latest_frame, node.confirmed_tracks))
            cv2.imshow('World Map — confirmed trash',
                       draw_map(node.confirmed_tracks))

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
