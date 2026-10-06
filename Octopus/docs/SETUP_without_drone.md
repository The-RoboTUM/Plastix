# Octopus ohne Drohne starten

> **Pfade.** Alle Pfade in diesem Dokument sind relativ zur Repo-Wurzel — dorthin wechseln,
> und zwar in *jedem* Terminal, das du dafür öffnest.

Das Dashboard ohne Drohne, ohne Pi und ohne Detektor. Gedacht für alles, was man am
Schreibtisch prüfen will: Kartenanzeige, Detektions-Overlay, Projektion auf die Mission
Map — und optional den Sammelroboter GripperX darauf.

**Die vollständige Demo mit Drohne steht in [SETUP.md](SETUP.md).** Dieses Dokument ist die
kleine Variante und wiederholt daraus nichts.

Zwei Stufen. **Stufe 1** ist das Dashboard allein: zwei Prozesse, kein ROS, läuft auf
jedem Rechner (Linux/macOS/Windows). **Stufe 2** baut darauf auf und holt GripperX auf die
Mission Map — dafür braucht es ROS 2 Humble und drei weitere Terminals.

Wer nur das Dashboard sehen will, hört nach Stufe 1 auf.

---

## Stufe 1: Nur Dashboard

**Keine Drohne, kein Pixhawk, kein ROS.** Das Kamerabild kommt aus einem Video, Bild oder
von der Webcam.

```
test_camera_feed.py ──HTTP POST──▶ api.py ──serviert──▶ dashboard.html (Browser)
   (Bild/Video/YOLO)                 (Port 8000)
```

Es laufen nur **zwei Prozesse**:

1. **Backend** (`api.py`, FastAPI) — serviert das Dashboard und hält den neuesten
   Kamera-Frame + die Detektionen im Speicher.
2. **Test-Feed** (`test_camera_feed.py`) — ersetzt die ganze ROS-Pipeline
   (Kamera-Node + Detektor + Bridge): liest Bild/Video/Webcam, optional durch YOLO,
   und schickt Frames + Detektionen per HTTP ans Backend.

### Repo holen

```bash
git clone https://gitex.itq.de/cirqmind/PlastiX.git
cd PlastiX
git checkout eve-octopus      # WICHTIG: das Dashboard liegt auf diesem Branch, nicht main
```

- **git-lfs ist nicht nötig** — das Dashboard (inkl. Leaflet unter `vendor/`) besteht aus
  normalen Dateien.
- Die **YOLO-Modelle sind nicht im Repo** (bewusst per `.gitignore` ausgeschlossen, weil groß).
  Für den `--demo`-Modus braucht man sie nicht. Für **echte** YOLO-Detektion ein Modell aus
  `Octopus/detect-and-localize/data/models/` (z. B. `best_model_10_08_26.pt`) separat auf das
  Zielgerät kopieren.
- `octopusfinal.db` (Demo-Daten für Tasks/Fleet) ist im Repo und wird mitgeholt.

### Installation

```bash
cd Octopus/octopus-dashboard

python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate

pip install -r requirements.txt    # Backend (fastapi, uvicorn, ...)
pip install opencv-python          # für den Test-Feed (Bild/Video/Webcam)

# optional, nur für echte Detektion:
pip install ultralytics
```

### Starten — zwei Terminals

**Terminal 1 — Backend:**

```bash
cd Octopus/octopus-dashboard && source .venv/bin/activate && python -m uvicorn api:app --host 127.0.0.1 --port 8000
```

**Terminal 2 — Kamera-Feed**

**Schritt 1:** in den Ordner wechseln und das venv aktivieren.

```bash
cd Octopus/octopus-dashboard && source .venv/bin/activate
```

**Schritt 2:** eine der vier Varianten starten — je nachdem, ob du Bild oder Video willst
und ob echte Detektion laufen soll. `DEIN_BILD.jpg` und `DEIN_VIDEO.mp4` durch eigene
Dateien ersetzen.

```bash
# a) nur Feed, keine Boxen — Standbild:
python test_camera_feed.py --source DEIN_BILD.jpg

# b) Feed + Demo-Boxen (KEIN ML nötig) — testet Overlay + Karten-Projektion:
python test_camera_feed.py --source DEIN_BILD.jpg --demo
python test_camera_feed.py --source DEIN_VIDEO.mp4 --demo --fps 5

# c) Feed + ECHTE YOLO-Detektion — Video:
python test_camera_feed.py --source DEIN_VIDEO.mp4 --model ../detect-and-localize/data/models/best_model_10_08_26.pt 


# d) Feed + ECHTE YOLO-Detektion — Webcam:
python test_camera_feed.py --source 0 --model ../detect-and-localize/data/models/best_model_10_08_26.pt 
```

`--source` und die Box-Quelle sind unabhängig: jede Quelle (Standbild, Video, Webcam-Index)
lässt sich mit `--demo`, mit `--model` oder mit keinem von beiden kombinieren.

Ein Video wird am Ende automatisch zurückgespult und endlos wiederholt, ein Standbild
endlos gesendet, der Feed läuft also in jedem Fall dauerhaft. `--demo` legt drei feste
Demo-Boxen auf das Bild, unabhängig vom Bildinhalt. Weitere nützliche Optionen:
`--fps 5`, `--conf 0.25`, `--backend http://127.0.0.1:8000`.

### Browser

```
http://127.0.0.1:8000/dashboard.html
```

### Bedienung im Dashboard

- View steht auf **Mission Overview**: links die Karte, rechts der **Camera Feed**
  mit neongelbem Grid, grünen Detektions-Boxen + Center-Dot.
- **Kamera/Pipeline-Panel** (unten links): im reinen Test-Setup nicht nötig — der
  Feed kommt schon vom `test_camera_feed.py`.
- **Detektionen auf die Karte projizieren:**
  1. In der Mission-Map-Toolbar **Set Eve** klicken, dann auf die Karte klicken
     (oder den Eve-Marker ziehen) → Eve wird platziert.
  2. Mit dem **Eve-yaw**-Regler die Blickrichtung einstellen (0° = Norden).
  3. Die projizierten Trash-Marker + der Kamera-Footprint erscheinen links auf der Karte.

### Troubleshooting

- **„Waiting for camera feed"** im Dashboard → läuft `test_camera_feed.py`? Zeigt es
  „posted N frames"? Stimmt `--backend`/der Port?
- **`POST failed`** im Feed-Skript → Backend (Terminal 1) läuft nicht oder falscher Port.
- **Keine Projektion auf der Karte** → Eve wurde noch nicht platziert.
- **Tasks/Fleet leer** → `octopusfinal.db` mitkopiert? (nur für die Demo-Fleet-Daten,
  der Kamera-Feed funktioniert auch ohne).
- **Zugriff von einem anderen Rechner im Netzwerk** → Backend mit
  `--host 0.0.0.0` starten und im Browser die IP des Backend-Rechners verwenden.

---

## Stufe 2 (optional): GripperX auf der Karte

Kleinste Konfiguration, in der der Sammelroboter auf der Mission Map erscheint und sich
bewegt. Stufe 1 läuft weiter — dazu kommen rosbridge und zwei Bridge-Nodes. Die Position
kommt aus dem GripperX-Twin.

**Voraussetzung:** ROS 2 Humble auf diesem Rechner. Stufe 1 braucht das nicht, Stufe 2
schon.

```
test_camera_feed.py ──HTTP──▶ Backend :8000 ──▶ dashboard.html
                                   ▲   │
              Eve gesetzt (Browser) │   │ POST /api/devices/gripperx/status
                                    │   ▼
                    eve_fake_gps_bridge_node    device_status_backend_bridge_node
                                    │                        ▲
                    /octopus/fake_eve_gps_start   /octopus/devices/gripperx/status
                                    │                        │
                                    └──────── rosbridge :9090 ┘
                                                    │
                                          GripperX-Twin (ROS 2 Jazzy, Domain 220)
```

Der Transport und seine Argumente stehen in
[gripperx_rosbridge_link.md](gripperx_rosbridge_link.md), die Payload-Übersetzung ebenfalls
dort. Die GripperX-Seite ist **nicht** Teil dieses Dokuments — siehe „GripperX-Seite" unten.

### Einmalig

Alle Befehle hier laufen in einer ROS-2-Humble-Umgebung.

```bash
./Octopus/scripts/build_rosbridge.sh
```

Nur nötig, wenn `ros-humble-rosbridge-suite` nicht installiert ist — das Skript baut Tag
2.0.7 in ein eigenes, gitignoriertes Overlay. Mit root ist
`sudo apt install ros-humble-rosbridge-suite` gleichwertig und schneller; `run_rosbridge.sh`
nimmt das apt-Paket automatisch, sobald es da ist.

```bash
cd Octopus/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select octopus_backend_bridge --symlink-install
```

Nur dieses eine Paket — es hängt an `rclpy`/`std_msgs`/`sensor_msgs` und braucht **kein**
`px4_msgs`, das für diese Variante gar nicht geholt werden muss.

### Starten — drei weitere Terminals

Terminal 1 und 2 aus Stufe 1 laufen weiter.

**Terminal 3 — rosbridge:**

```bash
./Octopus/scripts/run_rosbridge.sh
```

Einzige Stelle, an der die Startargumente stehen. Muss auf **derselben** `ROS_DOMAIN_ID`
laufen wie die Bridge-Nodes, hier `0` — ein Domain-Mismatch sieht von der Roboterseite wie
ein Netzproblem aus.

**Terminal 4 — Datum ins ROS (Backend → ROS):**

```bash
cd Octopus/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 && ros2 run octopus_backend_bridge eve_fake_gps_bridge_node
```

Pollt das Backend und publiziert `/octopus/fake_eve_gps_start`. Bevor du Eve gesetzt hast,
sendet er das Fallback-Datum und sagt das auch — das ist kein Fehler.

**Terminal 5 — Robotertelemetrie ins Backend (ROS → Backend):**

```bash
cd Octopus/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 && ros2 run octopus_backend_bridge device_status_backend_bridge_node
```

Übersetzt dabei den GripperX-Dialekt in die Form, die das Dashboard liest.

### Eve setzen

In der Mission-Map-Toolbar **Set Eve** klicken, dann auf die Karte — das setzt den
gemeinsamen Nullpunkt für die ganze Flotte. **Erst danach** kann GripperX eine echte
Position melden; ohne Datum meldet er `no_datum` und bekommt bewusst keinen Marker.

### Prüfen

Beide Skripte hier brauchen **websockets >= 13** (sie importieren
`websockets.asyncio.client`). Die GripperX-Seite braucht umgekehrt eine Version **< 14** —
warum, und was zu tun ist, wenn beide auf einem Rechner liegen, steht in
[gripperx_rosbridge_link.md](gripperx_rosbridge_link.md#websockets-versionen).

```bash
python3 Octopus/scripts/check_rosbridge.py
```

Beide Richtungen ohne ROS auf der Clientseite. `NO DATA` bei `/octopus/trash_goal` und
`/octopus/trash_gps` ist in dieser Variante **erwartet** — es läuft kein Detektor und kein
`trash_gps_goal_node`. Das abschließende `RESULT: FAIL` bezieht sich darauf.

```bash
curl -s http://127.0.0.1:8000/api/devices/status | python3 -m json.tool
```

Kommt hier ein `gripperx` mit `pose.status: "ok"` und einer `lat`/`lon` an, ist die ganze
Kette dicht. Steht dort `no_datum`, hast du Eve noch nicht gesetzt.

Ohne Roboter testbar — sendet die Form des echten Roboters und geht dadurch durch die
Übersetzung:

```bash
python3 Octopus/scripts/simulate_gripperx.py --dialect gripperx --no-collect
```

`--no-collect` ist hier wichtig: ohne das meldet der Simulator Ziele als eingesammelt, und
auf einem laufenden Demo-Stack ist das nicht zurücknehmbar.

### Stoppen

Terminals einzeln mit Ctrl-C. Der Debug-Stack-Stopper gehört zur vollen Demo und wird hier
nicht gebraucht.

### GripperX-Seite

Nicht Teil dieses Dokuments. Startpunkte im GripperX-Ordner des Repos — der liegt auf Branch
`GripperX`, nicht auf `eve-octopus`:

| Was | Wo |
|---|---|
| Simulation hochfahren, 3-Terminal-Satz, Teardown | `GripperX/Software/ros2/src/gripperx_gazebo/README.md` |
| Twin + Octopus-Link zusammen, Schritt für Schritt | `GripperX/documentation/TWIN_OCTOPUS_RUNBOOK.md` |
| Echter Roboter, Inbetriebnahme | `GripperX/documentation/DEPLOYMENT.md` |

Zwei Dinge, die die GripperX-Seite braucht und die man leicht übersieht:
`python3-websockets` muss installiert sein (der Link stirbt sonst beim Start mit einem
`ModuleNotFoundError`, und pip ist ausdrücklich der falsche Weg), und die STL-Meshes liegen
per Git-LFS im Repo — ohne `git lfs pull` ist der Roboter in Gazebo unsichtbar, fährt aber.

Damit GripperX **auf** Eve startet statt daneben, muss der Twin auf dem Kartenursprung
gespawnt werden: `spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0`. Die Vorgabe von
`sim_mapping.launch.py` ist `spawn_y:=-5.0`, also 5 m südlich — dann steht der Marker
korrekt, aber eben 5 m unter Eve.
