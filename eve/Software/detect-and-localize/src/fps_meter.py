import time
import cv2
import numpy as np

class FPSMeter:
    """
    Utility class to measure and display FPS in a live loop.
    Keeps a rolling average over the last N frames.
    """
    def __init__(self, avg_len=100):
        self.avg_len = avg_len
        self.frame_times = []
        self.last_time = None
        self.fps = 0.0
        self.avg_fps = 0.0

    def update(self):
        """
        Call this once per loop iteration to update timing.
        Returns (fps, avg_fps)
        """
        current_time = time.perf_counter()

        if self.last_time is not None:
            delta = current_time - self.last_time
            if delta > 0:
                self.fps = 1.0 / delta
                self.frame_times.append(self.fps)
                if len(self.frame_times) > self.avg_len:
                    self.frame_times.pop(0)
                self.avg_fps = sum(self.frame_times) / len(self.frame_times)
        self.last_time = current_time

        return self.fps, self.avg_fps

    def draw_on_frame(self, frame, extra_text=None):
        """
        Draws FPS information (and optional extra text) onto a given frame.
        """
        text = f"FPS: {self.avg_fps:.1f}"
        if extra_text:
            text += f" | {extra_text}"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 0), 2)
        return frame