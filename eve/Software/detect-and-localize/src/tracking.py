import numpy as np

class GarbageTracker:
    def __init__(self, max_lost=10, confirm_frames=5, dist_thresh=0.1, move_thresh=0.02):
        self.tracks = []
        self.next_id = 0
        self.max_lost = max_lost
        self.confirm_frames = confirm_frames
        self.dist_thresh = dist_thresh
        # max movement (in world/normalized units) between frames for an object
        # to count as "stationary". A track is only confirmed once it has been
        # stationary for `confirm_frames` consecutive frames, so the locked
        # position is where the trash comes to rest, not where it was mid-throw.
        self.move_thresh = move_thresh

    def update(self, detections_world):
        updated_tracks = []
        for det in detections_world:
            det = np.array(det, dtype=float)
            matched = False
            for track in self.tracks:
                moved = np.linalg.norm(det - np.array(track["pos"]))
                if moved < self.dist_thresh:
                    # count consecutive stationary frames; reset on movement
                    if moved < self.move_thresh:
                        track["stable"] += 1
                    else:
                        track["stable"] = 0
                    # keep following the object until it settles; once confirmed
                    # the resting position stays locked
                    if not track["confirmed"]:
                        track["pos"] = det
                    track["missed"] = 0
                    track["age"] += 1
                    matched = True
                    updated_tracks.append(track)
                    break
            if not matched:
                self.tracks.append({
                    "id": self.next_id,
                    "pos": det,
                    "age": 1,
                    "missed": 0,
                    "stable": 0,
                    "confirmed": False
                })
                self.next_id += 1

        for t in self.tracks:
            if t not in updated_tracks:
                t["missed"] += 1
        self.tracks = [t for t in self.tracks if t["missed"] < self.max_lost]

        new_confirmed = []
        for t in self.tracks:
            if not t["confirmed"] and t["stable"] >= self.confirm_frames:
                t["confirmed"] = True
                new_confirmed.append(t)

        confirmed = [t for t in self.tracks if t["confirmed"]]
        return confirmed, new_confirmed
