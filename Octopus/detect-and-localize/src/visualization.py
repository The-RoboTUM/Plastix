import cv2
import numpy as np

class Visualizer:
    def __init__(self, canvas_size=500, margin=50):
        self.canvas_size = canvas_size
        self.margin = margin

    def world_to_canvas(self, pt, world_center, scale):
        x, y = pt - world_center
        cx = int(self.canvas_size / 2 + x * scale)
        cy = int(self.canvas_size / 2 - y * scale)
        return (cx, cy)

    def compute_viewport(self, tag_positions):
        xs = [p[0] for p in tag_positions.values()]
        ys = [p[1] for p in tag_positions.values()]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        center = np.array([(min_x + max_x)/2, (min_y + max_y)/2])
        scale = (self.canvas_size - 2*self.margin) / max(max_x - min_x, max_y - min_y, 1e-6) * 0.5
        return center, scale

    def draw_canvas(self, tag_positions, detections_world, confirmed_tracks, world_center, scale):
        canvas = np.ones((self.canvas_size, self.canvas_size, 3), np.uint8) * 255
        for tid, pos in tag_positions.items():
            pt = self.world_to_canvas(pos, world_center, scale)
            cv2.circle(canvas, pt, 6, (0,0,255), -1)
            cv2.putText(canvas, f"Tag {tid}", (pt[0]+5, pt[1]-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,0), 1)

        for pt in detections_world:
            ptc = self.world_to_canvas(pt, world_center, scale)
            cv2.circle(canvas, ptc, 4, (255,0,0), -1)

        for t in confirmed_tracks.values():
            ptc = self.world_to_canvas(np.array(t), world_center, scale)
            cv2.circle(canvas, ptc, 6, (0,255,0), -1)
        return canvas
