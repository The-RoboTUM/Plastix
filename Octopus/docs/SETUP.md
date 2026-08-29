# Octopus Indoor-Demo starten

> **Pfade.** Alle Pfade in diesem Dokument sind relativ zur Repo-Wurzel — dorthin wechseln, und zwar in *jedem* Terminal, das du dafür öffnest.

Primäres Start-Dokument für die Indoor-Demo: Eve-Kamera → Detektor → Projektion auf den
Boden → Global Mission Grid → Dashboard.

```
Pi: camera_node ──┐
                  ├──▶ detector_node ──▶ flight_camera_transform ──▶ grid_map_builder
Pi: MicroXRCEAgent┘        (Laptop)            (Laptop)                  (Laptop)
    (PX4-Attitude)                                                          │
                                                                            ▼
                                                    Backend :8000 ──▶ dashboard.html
```

Erstinstallation auf einem neuen Laptop — Basiswerkzeuge, Repo, `px4_msgs`, Detektor-Modell,
Python-Umgebung, Workspace-Build — steht in [../README.md](../README.md), Abschnitte 2 bis 4.
Konzepte (Local Camera Grid, Global Mission Grid, Indoor Static Mission Map) in Abschnitt 1,
Presentation Flow in Abschnitt 8.

Ohne Drohne, ohne Pi, ohne Detektor — nur das Dashboard, optional mit GripperX auf der
Mission Map: [Variante ohne Drohne](#variante-ohne-drohne) weiter unten.

---

## Variante ohne Drohne

Zwei Stufen. **Stufe 1** ist das Dashboard allein: zwei Prozesse, kein ROS, läuft auf
jedem Rechner (Linux/macOS/Windows). **Stufe 2** baut darauf auf und holt GripperX auf die
Mission Map — dafür braucht es ROS 2 Humble und drei weitere Terminals.

Wer nur das Dashboard sehen will, hört nach Stufe 1 auf.

---

### Stufe 1: Nur Dashboard

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

#### Repo holen

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

#### Installation

```bash
cd Octopus/octopus-dashboard

python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate

pip install -r requirements.txt    # Backend (fastapi, uvicorn, ...)
pip install opencv-python          # für den Test-Feed (Bild/Video/Webcam)

# optional, nur für echte Detektion:
pip install ultralytics
```

#### Starten — zwei Terminals

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

#### Browser

```
http://127.0.0.1:8000/dashboard.html
```

#### Bedienung im Dashboard

- View steht auf **Mission Overview**: links die Karte, rechts der **Camera Feed**
  mit neongelbem Grid, grünen Detektions-Boxen + Center-Dot.
- **Kamera/Pipeline-Panel** (unten links): im reinen Test-Setup nicht nötig — der
  Feed kommt schon vom `test_camera_feed.py`.
- **Detektionen auf die Karte projizieren:**
  1. In der Mission-Map-Toolbar **Set Eve** klicken, dann auf die Karte klicken
     (oder den Eve-Marker ziehen) → Eve wird platziert.
  2. Mit dem **Eve-yaw**-Regler die Blickrichtung einstellen (0° = Norden).
  3. Die projizierten Trash-Marker + der Kamera-Footprint erscheinen links auf der Karte.

#### Troubleshooting

- **„Waiting for camera feed"** im Dashboard → läuft `test_camera_feed.py`? Zeigt es
  „posted N frames"? Stimmt `--backend`/der Port?
- **`POST failed`** im Feed-Skript → Backend (Terminal 1) läuft nicht oder falscher Port.
- **Keine Projektion auf der Karte** → Eve wurde noch nicht platziert.
- **Tasks/Fleet leer** → `octopusfinal.db` mitkopiert? (nur für die Demo-Fleet-Daten,
  der Kamera-Feed funktioniert auch ohne).
- **Zugriff von einem anderen Rechner im Netzwerk** → Backend mit
  `--host 0.0.0.0` starten und im Browser die IP des Backend-Rechners verwenden.

---

### Stufe 2 (optional): GripperX auf der Karte

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

#### Einmalig

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

#### Starten — drei weitere Terminals

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

#### Eve setzen

In der Mission-Map-Toolbar **Set Eve** klicken, dann auf die Karte — das setzt den
gemeinsamen Nullpunkt für die ganze Flotte. **Erst danach** kann GripperX eine echte
Position melden; ohne Datum meldet er `no_datum` und bekommt bewusst keinen Marker.

#### Prüfen

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

#### Stoppen

Terminals einzeln mit Ctrl-C. Der Debug-Stack-Stopper gehört zur vollen Demo und wird hier
nicht gebraucht.

#### GripperX-Seite

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

---

## Volle Indoor-Demo mit Drohne

Ab hier die vollständige Demo mit Drohne, Pi und Detektor.

**Voraussetzung:** Ubuntu 22.04 mit ROS 2 Humble auf dem Laptop und gebautem
`Octopus/ros2_ws`, ROS 2 Humble auf der Pi mit gebautem `eve/Software/ros2_ws`.

---

## 0. Sicherheit

- **Propeller abnehmen.**
- Drohne so sichern, dass sie sich nicht bewegen kann.
- Erst dann Pixhawk und Raspberry Pi mit Strom versorgen.
- Ein gut sichtbares Objekt ins Kamerabild legen, gute Beleuchtung.
- Keine Fake-Detektionen nebenher laufen lassen.
- Octopus erst starten, wenn die Drohne physisch ausgerichtet ist.

Die Indoor-Map-Konvention und die festen Positionsparameter stehen in
[../README.md](../README.md), Abschnitt 6.

---

## Einmalig: SSH-Alias für die Pi

Ohne diesen Alias funktionieren die Befehle unten und der Button *„Connect Eve Camera"*
im Dashboard nicht — `api.py` erwartet den Host `eve-pi`. `<PI-IP>` durch die aktuelle IP
der Drohne ersetzen:

```bash
printf 'Host eve-pi\n  HostName <PI-IP>\n  User eve\n  IdentityFile ~/.ssh/id_ed25519\n  ServerAliveInterval 15\n' >> ~/.ssh/config && chmod 600 ~/.ssh/config
```

Testen:

```bash
ssh eve-pi
```

```bash
hostname; ls ~/PlastiX/eve/Software/ros2_ws
```

Der `eve/`-Pfad ist hier kein Tippfehler: die Pi läuft auf Branch `eve_ros_development`
mit dem Ordner `eve/`, der Laptop auf `eve-octopus` mit `Octopus/`. Die Pi-Seite ist nicht
Teil des Octopus-Setups — sie muss nur Kamera- und PX4-Topics liefern.

## Einmalig: Workspace neu bauen

Nur nötig, wenn `install/` fehlt **oder das Repo verschoben wurde** — `colcon --symlink-install`
hinterlässt beim Verschieben tote Symlinks, und dann ist kein `octopus_*`-Node startbar:

```bash
cd Octopus/ros2_ws && rm -rf build install log && source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

Ein Workspace genügt: `detection_pkg` liegt darin, der Laptop braucht keinen zweiten.
`px4_msgs` dauert dabei ~4 Minuten. Prüfen, ob alles da ist:

```bash
source /opt/ros/humble/setup.bash && source Octopus/ros2_ws/install/setup.bash && ros2 pkg executables octopus_camera_transform
```

Erwartet: vier Einträge, darunter `flight_camera_transform_node`.

Beim Verschieben des Repos bricht auch das Detektor-`.venv` — `VIRTUAL_ENV` in
`Octopus/detect-and-localize/.venv/bin/activate` zeigt dann auf den alten Pfad. Entweder
das venv neu anlegen oder die Pfade in `.venv/bin/` umschreiben.

---

## Terminal 1 — Pi: PX4-Brücke

Läuft auf der Pi. Der Code dazu liegt auf Branch `eve_ros_development` im Ordner `eve/`
und ist nicht Teil des Octopus-Setups.

```bash
ssh eve-pi
```

```bash
sudo pkill -f "[M]icroXRCEAgent"; export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0; sudo --preserve-env=ROS_DOMAIN_ID,ROS_LOCALHOST_ONLY MicroXRCEAgent serial --dev /dev/serial0 -b 921600 -v 6
```

**Terminal offen lassen.** Danach publiziert `/fmu/out/vehicle_odometry` mit ca. 70–100 Hz.

## Terminal 2 — Pi: Kamera

Läuft auf der Pi. `camera_pkg` gehört zur Pi-Seite (Branch `eve_ros_development`, Ordner
`eve/`) — im Octopus-Ordner gibt es dieses Paket nicht.

```bash
ssh eve-pi
```

```bash
pkill -f "[c]amera_node"; cd ~/PlastiX/eve/Software/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 && ros2 run camera_pkg camera_node --ros-args -p publish_raw:=false -p device_index:=0
```

**Terminal offen lassen.** Danach publiziert `/camera/image_raw/compressed` mit ca. 7–15 Hz.

`camera_node` kennt `device_index`, `frame_rate`, `frame_width` und `frame_height`. Die
Auflösung ist die einzige Stelle, an der echte Bildschärfe entsteht — Default ist 640×480:

```bash
ros2 run camera_pkg camera_node --ros-args -p publish_raw:=false -p device_index:=0 -p frame_width:=1280 -p frame_height:=960
```

⚠️ Nur ändern, wenn die Kamera dabei denselben Bildwinkel behält. Viele USB-Kameras
wechseln mit der Auflösung den Sensor-Ausschnitt; dann stimmen die Intrinsics
(`OCTOPUS_HBVCAM_640X480` in `octopus-dashboard/live_data.js`) nicht mehr und Footprint,
Grid und Projektion sind falsch. Bei gleichem Bildwinkel ist keine Anpassung nötig: das
Dashboard rechnet nur mit Verhältnissen (`cx/fx`), die sich beim proportionalen Skalieren
nicht ändern.

## Terminal 3 — Laptop: Detektor

```bash
cd Octopus/detect-and-localize && source .venv/bin/activate && source /opt/ros/humble/setup.bash && source ../ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 && python ../ros2_ws/src/detection_pkg/detection_pkg/detector_node.py --ros-args -p detect_localize_path:="$PWD" -p model:=data/models/best_model_10_08_26.pt -p input_topic:=/camera/image_raw/compressed -p output_frame:=camera -p show_ui:=false -p thresh:=0.60 -p confirm_frames:=3 -p max_lost:=5 -p yolo_frameskip:=0 -p dist_thresh:=0.10 -p move_thresh:=0.10 -p confirmed_republish_period_sec:=1.0
```

**Terminal offen lassen.** Danach publiziert `/detector_node/confirmed` mit ca. 1 Hz.

Zur Wahl von `thresh` und `confirm_frames` siehe „Detektions-Schwelle einstellen" weiter unten.

Sieht das Bild im Dashboard klotzig aus, ist die JPEG-Kompression schuld, nicht die
Auflösung. `debug_image_jpeg_quality` (1–100, Default 80) steuert sie und wird an den
Detektor gehängt, **nicht** an `camera_node`:

```bash
... -p confirmed_republish_period_sec:=1.0 -p debug_image_jpeg_quality:=95
```

Betrifft nur `/detector_node/debug_image/compressed`, also ausschließlich die Anzeige —
Detektion, `confirmed` und Mapping bleiben unberührt.

## Terminal 4 — Laptop: Octopus-Stack

Startet das Dashboard-Backend, alle elf ROS-Nodes **und** rosbridge:

```bash
OCTOPUS_MAPPING_MODE=indoor_static_mission ./Octopus/scripts/start_octopus_debug_stack.sh
```

Das Skript beendet vorher alte Prozesse, kann also gefahrlos wiederholt werden. Es löst die
Nodes per `setsid` von der Terminal-Session — das Terminal darf zu, ohne dass etwas stirbt.

Logs liegen in `/tmp/octopus_logs/`.

## GripperX dazu (optional)

**Auf der Octopus-Seite ist nichts weiter zu tun.** Das Stack-Skript aus Terminal 4 startet
alles, was der Sammelroboter braucht, schon mit: `eve_fake_gps_bridge_node`,
`device_status_backend_bridge_node` und rosbridge. Die Adresse, die die GripperX-Seite
anwählt, druckt das Skript am Ende selbst aus:

```text
GripperX link (rosbridge):
ws://<ip-dieses-rechners>:9090
```

Der Unterschied zur [Variante ohne Drohne](#variante-ohne-drohne): dort erscheint GripperX
nur als Marker auf der Karte. Hier läuft zusätzlich `trash_gps_goal_node`, also fließen
**echte Müllziele aus dem Detektor** über `/octopus/trash_goal` und `/octopus/trash_gps` —
der eigentliche Anwendungsfall. Die Topics stehen in
[octopus_to_robot_interface.md](octopus_to_robot_interface.md).

Vor dem Start der GripperX-Seite im Dashboard **Set Eve** setzen. Ohne dieses Datum meldet
GripperX `no_datum` und bekommt bewusst keinen Marker.

Die GripperX-Seite selbst ist nicht Teil dieses Dokuments — siehe „GripperX-Seite" in der
Variante ohne Drohne. Sie läuft auf ROS 2 Jazzy und `ROS_DOMAIN_ID=220`; das ist **kein**
Widerspruch zur `0` hier, weil rosbridge die beiden Graphen per WebSocket verbindet und
nicht per DDS. `octopus_link.launch.py` bricht mit `env:=twin` sogar ab, wenn die Domain
nicht 220 ist.

Prüfen, ob die Kette steht:

```bash
curl -s http://127.0.0.1:8000/api/devices/status | python3 -m json.tool
```

Ein `gripperx` mit `pose.status: "ok"` und `lat`/`lon` heißt: dicht. `no_datum` heißt:
Set Eve fehlt.

---

## Browser

```bash
xdg-open http://127.0.0.1:8000/dashboard.html
```

Im Dashboard einstellen:

- **Mapping Settings → Mission mapping mode**: `Indoor Static Mission Map`
- **Local Grid Map → Grid source**: `Global Mission Grid`

---

## Health-Check

```bash
source /opt/ros/humble/setup.bash && source Octopus/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 && python3 Octopus/scripts/octopus_pipeline_health.py
```

Gut ist, wenn `/camera/image_raw/compressed`, `/fmu/out/vehicle_odometry`,
`/detector_node/confirmed`, `/octopus/detections_world` und `/octopus/map_patch` je einen
Publisher **und** einen Subscriber haben und beide Backend-Endpunkte `reachable` sind.

`/fmu/out/vehicle_local_position: no publisher` ist im Indoor-Modus in Ordnung — siehe unten.

Den entscheidenden Zustand einzeln abfragen:

```bash
source /opt/ros/humble/setup.bash && source Octopus/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 && ros2 topic echo --once /octopus/flight_camera_transform/status --field data | head -n 1 | python3 -m json.tool | grep -E '"state"|"transform_ready"|"last_input_detection_count"|"last_transformed_detection_count"|"last_projection_error"'
```

Erwartet:

```text
"state": "ready"
"transform_ready": true
"last_input_detection_count": 3
"last_transformed_detection_count": 3
"last_projection_error": null
```

Wichtig ist, dass `last_transformed_detection_count` **nicht 0** ist, wenn Detektionen
reinkommen.

Läuft die Karte wirklich live? Der Zeitstempel muss sich ändern:

```bash
curl -s http://127.0.0.1:8000/api/map_patch/latest | python3 -m json.tool | grep timestamp
```

```bash
sleep 5; curl -s http://127.0.0.1:8000/api/map_patch/latest | python3 -m json.tool | grep timestamp
```

Die x/y-Werte dürfen gleich bleiben, wenn sich das Objekt nicht bewegt. Der Zeitstempel nicht.

---

## Was du sehen solltest

- **Camera Feed** rechts: Live-Bild mit Grid und grünen Boxen
- **Map Patch** oben: eine Zahl statt `No patch yet`
- **Detections/Tasks**: zählt die bestätigten Detektionen
- Auf der **Mission Map**: projizierte Trash-Marker plus Kamera-Footprint

---

## Detektions-Schwelle einstellen

`thresh` filtert die YOLO-Detektionen in `Octopus/detect-and-localize/src/yolo_detector.py` nach der
Inferenz. Er wirkt, wird in der Praxis aber vom Tracker ausgehebelt, wenn `confirm_frames`
zu niedrig steht:

Mit `confirm_frames:=1` genügt **ein einziger** Frame über der Schwelle, um ein Objekt dauerhaft
zu bestätigen. Danach lebt der Track noch `max_lost` Frames weiter (Default 10), auch wenn die
Konfidenz längst darunter liegt — und `confirmed_republish_period_sec` sendet ihn weiterhin.

Für eine wirksame Schwelle von 0,6 also:

```text
-p thresh:=0.60 -p confirm_frames:=3 -p max_lost:=5
```

Zwei Dinge, die dabei wichtig sind:

- Nach dem Bestätigen existiert **keine Konfidenz mehr**. `pipeline._to_world` reduziert jede
  Detektion auf einen (x,y)-Punkt, und `/detector_node/confirmed` ist ein `PoseArray`.
- Die `confidence` und `trash_probability` im Map-Patch sind **feste Werte** aus dem Parameter
  `confidence` (Default 0.8) von `world_posearray_to_json_bridge_node` — sie haben nichts mit
  der YOLO-Konfidenz zu tun. Auf der Karte lässt sich deshalb nicht nach Konfidenz filtern.

Auf die Karte kommt außerdem nur die Klasse `rubbish`; `_to_world` verwirft alle anderen. Im
Kamerabild des Dashboards erscheinen dagegen alle Klassen des Modells — bei
`best_model_10_08_26.pt` also auch `apriltag`, `human` und `robot`.

---

## Alles stoppen

```bash
./Octopus/scripts/stop_octopus_debug_stack.sh
```

Terminals 1–3 mit Ctrl+C beenden. In den beiden Pi-Terminals danach `exit`, um die
SSH-Sitzung zu schließen. Das Stop-Skript lässt gelegentlich Nodes stehen; dann hinterher
**auf dem Laptop**:

```bash
pkill -9 -f "[g]rid_map_builder_node"; pkill -9 -f "[m]ap_patch_backend_bridge_node"; pkill -9 -f "[u]vicorn api:app"
```

Prüfen, ob wirklich alles weg ist:

```bash
pgrep -af "octopus_|uvicorn api" | grep -v pgrep
```

Die Klammern um den ersten Buchstaben verhindern, dass `pkill` sein eigenes Suchmuster trifft.
`pkill` nie in dieselbe Zeile schreiben, in der der zu killende Befehl nochmal vorkommt — dann
steht der echte String in der Kommandozeile und die Shell schießt sich selbst ab.

---

## Wenn etwas nicht geht

Weitere Fälle — `indoor_static_yaw_zero_rad is null`, `Extra data`,
`rcl_shutdown already called` — in [../README.md](../README.md), Abschnitt 10.

### `/fmu/out/vehicle_odometry` publiziert nicht

Terminal 1 neu starten. Kommt meist von einer abgebrochenen seriellen Verbindung.

### `/detector_node/confirmed` publiziert nicht

Der Detektor läuft, aber nichts wird „confirmed". Objekt sichtbar und gut beleuchtet?
Zum Gegentest `-p thresh:=0.20 -p confirm_frames:=1` — das sind die empfindlichsten Werte.

### `state: "pose_only"`, `last_transformed_detection_count: 0`

Bedeutet: Odometrie ist frisch, aber das Gate für die Projektion ist zu. Ursache ist fast
immer, dass PX4 `/fmu/out/vehicle_local_position` nicht über DDS publiziert.

Im **Indoor-Modus** wird dieses Topic nicht gebraucht — x/y kommen aus dem festen Origin, die
Höhe aus `manual_height_above_ground_m`. `flight_camera_transform_node` berücksichtigt das und
blockiert nicht mehr darauf.

Für den **Flug-Modus** (`OCTOPUS_MAPPING_MODE=flight_global_mission`) wird das Topic dagegen
wirklich gebraucht — dort kommen x/y aus der Odometrie und werden nicht eingefroren. Dann muss
`vehicle_local_position` in der uXRCE-DDS-Topic-Liste der PX4-Firmware freigeschaltet sein.
Das ist eine Pixhawk-Konfiguration, nicht am Laptop zu lösen.

### `/api/map_patch/latest` bleibt `empty`

Der Reihe nach prüfen: publiziert `/detector_node/confirmed`? Publiziert
`/fmu/out/vehicle_odometry`? Steht der Transform-Status auf `ready`?

### Backend stirbt direkt nach dem Start

Zeigt sich als „Connection refused" im Health-Check, obwohl das Log sauber
`Uvicorn running on http://0.0.0.0:8000` meldet. Passiert, wenn die Nodes an einer Session
hängen, die sich sofort beendet. Das Start-Skript verwendet deshalb `setsid`. Log ansehen mit:

```bash
tail -30 /tmp/octopus_logs/backend.log
```

### Pi und Laptop sehen sich nicht

`ROS_DOMAIN_ID=0` und `ROS_LOCALHOST_ONLY=0` müssen in **jedem** Terminal gesetzt sein — auch
in denen auf der Pi. Beide Rechner müssen im selben Subnetz sein.

### Kamerabild aktualisiert sich langsam

Die Anzeige ist doppelt gedeckelt: `camera_debug_backend_bridge_node` postet höchstens alle
`image_post_period_sec` (Default 0.5 s), und das Dashboard pollt mit
`setInterval(refreshCameraDebug, 1000)`. Mehr als ein Bild pro Sekunde ist ohne Anpassung
dieser beiden Werte nicht möglich.

Ist es deutlich langsamer, ist der Detektor der Engpass — YOLO läuft auf der CPU. Mit
`ros2 topic hz /detector_node/debug_image/compressed` messen und dabei prüfen, dass nicht
mehrere Detektoren parallel laufen oder ein Training die CPU belegt.

`yolo_frameskip` hilft hier **nicht**: bei übersprungenen Frames wird in `pipeline.py` das
alte annotierte Bild wiederverwendet, das Bild käme also nur öfter mit veraltetem Inhalt.

### Kamerabild ist pixelig

Nichts in der Kette skaliert das Bild runter — `yolo_detector.py` gibt das Originalbild
zurück, der Detektor und die Bridge ändern die Auflösung nicht. Es sind drei andere Ursachen,
in dieser Reihenfolge zu prüfen:

1. **Klotzige 8×8-Blöcke** → JPEG-Kompression. `debug_image_jpeg_quality:=95` am Detektor
   (siehe Terminal 3). Billigster Gewinn, kostet nur Bandbreite.
2. **Grundsätzlich grob** → die Quelle ist nur 640×480 und wird im Dashboard-Panel auf ein
   Mehrfaches aufgeblasen. Nur über `frame_width`/`frame_height` am `camera_node` zu lösen
   (siehe Terminal 2, mit der Warnung dort).
3. **Erst seit dem Croppen grob** → unvermeidbar. Ein Crop auf 512×384 im selben Panel
   bedeutet 25 % mehr Vergrößerung pro Pixel.

Das zweite JPEG-Encoding im Crop-Pfad ist **nicht** die Ursache: bei Quality 85 liegt die
zweite Generation bei ~49 dB PSNR, also unsichtbar. Höher zu gehen ist kontraproduktiv — bei
95 wird der beschnittene Frame größer als der ungeschnittene, den er ersetzt.

---

## Kamera-Crop

Im Dashboard unter *Camera & Pipeline* → **Camera Crop**: vier Slider für Top/Bottom/Left/
Right in Prozent (max. 45 % pro Seite), „Reset Crop", plus Anzeige der effektiven Auflösung
und des Footprints. Der Wert wird im Browser gespeichert.

Der Schnitt passiert **an der Quelle**, nicht erst im Browser: das Dashboard postet den Crop
an `POST /api/camera_debug/crop`, `camera_debug_backend_bridge_node` pollt ihn alle 2 s und
schneidet die Ränder ab, bevor der Frame rausgeht. Bei 10 % rundum sind das 512×384 statt
640×480, also deutlich weniger Bytes über die Leitung.

Der Kamera-Footprint wird über die beschnittene Sensorfläche gerechnet, damit Local-/GPS-Grid,
Zellennamen, Footprint-Umriss auf der Karte und die Detection-Projektion alle zur kleineren
Sicht passen. Detektionen in den weggeschnittenen Rändern werden verworfen — der Detektor läuft
weiter auf dem Vollbild — und als „N hidden" am Feed angezeigt.

Fällt der Crop an der Quelle aus (kein cv2, kaputtes JPEG, Crop lässt nichts übrig), geht der
Frame unverändert raus und der Browser schneidet selbst. Es fällt also nichts aus, das Bild
wird nur nicht kleiner. Das *Camera Debug*-Panel zeigt bewusst weiter das rohe Vollbild.

---

## Robot-relevante Ausgaben

Ein Sammelroboter, der Müll anfahren soll, benutzt die GPS-Topics:

```text
/octopus/fake_eve_gps_start    gemeinsamer Startpunkt (Datum)
/octopus/trash_goal            nächstes Ziel als NavSatFix
/octopus/trash_gps             alle Ziele als JSON
/octopus/trash_goal_done       Rückkanal: erledigte Ziel-id
```

Vollständiger Vertrag mit Payload-Schema, Parametern und Testkommandos:
[`octopus_to_robot_interface.md`](octopus_to_robot_interface.md).

Wer stattdessen direkt in Map-Metern arbeitet, nimmt `/octopus/detections_world`.

`/octopus/global_map`, `/octopus/coverage_grid` und `/octopus/trash_grid` werden zwar
publiziert, haben aber aktuell **keine Subscriber** — das Dashboard baut seine Karte im
Backend aus den Map-Patches. Nutzbar sind sie für RViz.

**Nicht** für Navigation benutzen: `/api/local_camera_grid/latest`, `Local Camera Grid`,
Frame `camera_footprint`. Das ist reine Kamera-/Debug-Visualisierung.
