# Octopus Dashboard — Setup ohne Drohne / ohne ROS

So läuft das komplette Dashboard auf einem beliebigen Rechner (Linux/macOS/Windows)
**ohne Drohne, ohne Pixhawk, ohne ROS** — mit einem Test-Kamera-Feed aus einem Bild,
Video oder der Webcam.

Es laufen nur **zwei Prozesse**:

1. **Backend** (`api.py`, FastAPI) — serviert das Dashboard und hält den neuesten
   Kamera-Frame + die Detektionen im Speicher.
2. **Test-Feed** (`test_camera_feed.py`) — ersetzt die ganze ROS-Pipeline
   (Kamera-Node + Detektor + Bridge): liest Bild/Video/Webcam, optional durch YOLO,
   und schickt Frames + Detektionen per HTTP ans Backend.

```
test_camera_feed.py ──HTTP POST──▶ api.py ──serviert──▶ dashboard.html (Browser)
   (Bild/Video/YOLO)                 (Port 8000)
```

---

## 1. Was auf das Zielgerät kopieren

Es reicht der Ordner **`octopus-dashboard/`** komplett, insbesondere:

```
api.py                 dashboard.html        live_data.js
vendor/                (Leaflet, offline)    octopusfinal.db   (Tasks/Fleet-Demo-Daten)
requirements.txt       test_camera_feed.py   SETUP_NO_DRONE.md
```

Optional, nur für **echte** YOLO-Detektion statt Demo-Boxen:
ein Modell aus `eve/Software/detect-and-localize/data/models/` (z. B. `indoor_v8s.pt`).

---

## 2. Installation

```bash
cd octopus-dashboard

python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate

pip install -r requirements.txt    # Backend (fastapi, uvicorn, ...)
pip install opencv-python          # für den Test-Feed (Bild/Video/Webcam)

# optional, nur für echte Detektion:
pip install ultralytics
```

---

## 3. Starten

**Terminal 1 — Backend:**
```bash
cd octopus-dashboard
source .venv/bin/activate
python -m uvicorn api:app --host 127.0.0.1 --port 8000
```

**Terminal 2 — Test-Feed** (drei Varianten):
```bash
cd octopus-dashboard
source .venv/bin/activate

# a) nur Feed, ein Standbild (keine Boxen):
python test_camera_feed.py --source mein_bild.jpg

# b) Feed + Demo-Boxen (KEIN ML nötig) — testet Overlay + Karten-Projektion:
python test_camera_feed.py --source mein_bild.jpg --demo

# c) Feed + ECHTE YOLO-Detektion auf einem Video:
python test_camera_feed.py --source clip.mp4 --model data/models/indoor_v8s.pt

# d) Feed + YOLO von der Webcam:
python test_camera_feed.py --source 0 --model data/models/indoor_v8s.pt
```

Nützliche Optionen: `--fps 5`, `--conf 0.25`, `--backend http://127.0.0.1:8000`.

**Browser:**
```
http://127.0.0.1:8000/dashboard.html
```

---

## 4. Bedienung im Dashboard

- View steht auf **Mission Overview**: links die Karte, rechts der **Camera Feed**
  mit neongelbem Grid, grünen Detektions-Boxen + Center-Dot.
- **Kamera/Pipeline-Panel** (unten links): im reinen Test-Setup nicht nötig — der
  Feed kommt schon vom `test_camera_feed.py`.
- **Detektionen auf die Karte projizieren:**
  1. In der Mission-Map-Toolbar **Set Eve** klicken, dann auf die Karte klicken
     (oder den Eve-Marker ziehen) → Eve wird platziert.
  2. Mit dem **Eve-yaw**-Regler die Blickrichtung einstellen (0° = Norden).
  3. Die projizierten Trash-Marker + der Kamera-Footprint erscheinen links auf der Karte.

---

## 5. Troubleshooting

- **„Waiting for camera feed"** im Dashboard → läuft `test_camera_feed.py`? Zeigt es
  „posted N frames"? Stimmt `--backend`/der Port?
- **`POST failed`** im Feed-Skript → Backend (Terminal 1) läuft nicht oder falscher Port.
- **Keine Projektion auf der Karte** → Eve wurde noch nicht platziert (Schritt 4).
- **Tasks/Fleet leer** → `octopusfinal.db` mitkopiert? (nur für die Demo-Fleet-Daten,
  der Kamera-Feed funktioniert auch ohne).
- **Zugriff von einem anderen Rechner im Netzwerk** → Backend mit
  `--host 0.0.0.0` starten und im Browser die IP des Backend-Rechners verwenden.
