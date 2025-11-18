import numpy as np

class GarbageTracker:
    def __init__(self, max_lost=10, confirm_frames=10, dist_thresh=0.1):
        self.tracks = []
        self.next_id = 0
        self.max_lost = max_lost
        self.confirm_frames = confirm_frames
        self.dist_thresh = dist_thresh

    def update(self, detections_world):
        updated_tracks = []
        for det in detections_world:
            matched = False
            for track in self.tracks:
                dist = np.linalg.norm(np.array(det) - np.array(track["pos"]))
                if dist < self.dist_thresh:
                    track.update(pos=det, missed=0, age=track["age"] + 1)
                    matched = True
                    updated_tracks.append(track)
                    break
            if not matched:
                self.tracks.append({
                    "id": self.next_id,
                    "pos": det,
                    "age": 1,
                    "missed": 0,
                    "confirmed": False
                })
                self.next_id += 1

        for t in self.tracks:
            if t not in updated_tracks:
                t["missed"] += 1
        self.tracks = [t for t in self.tracks if t["missed"] < self.max_lost]

        new_confirmed = []
        for t in self.tracks:
            if not t["confirmed"] and t["age"] >= self.confirm_frames:
                t["confirmed"] = True
                new_confirmed.append(t)

        

        confirmed = [t for t in self.tracks if t["confirmed"]]
        return confirmed, new_confirmed
