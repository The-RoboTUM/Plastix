# Octopus Indoor-Demo starten

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

**Voraussetzung:** Ubuntu 22.04 mit ROS 2 Humble auf dem Laptop, ROS 2 Humble auf der Pi,
beide ROS-Workspaces gebaut.

Erstinstallation auf einem neuen Laptop — Basiswerkzeuge, Repo, `px4_msgs`, Detektor-Modell,
Python-Umgebung, Workspace-Build — steht in [../README.md](../README.md), Abschnitte 2 bis 4.
Konzepte (Local Camera Grid, Global Mission Grid, Indoor Static Mission Map) in Abschnitt 1,
Presentation Flow in Abschnitt 8.

Reiner Dashboard-Test ohne Drohne: [../octopus-dashboard/SETUP_NO_DRONE.md](../octopus-dashboard/SETUP_NO_DRONE.md).

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

## Einmalig: Workspaces neu bauen

Nur nötig, wenn `install/` fehlt **oder das Repo verschoben wurde** — `colcon --symlink-install`
hinterlässt beim Verschieben tote Symlinks, und dann ist kein `octopus_*`-Node startbar:

```bash
cd ~/projects/PlastiX/Octopus/ros2_ws && rm -rf build install log && source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

```bash
cd ~/projects/PlastiX/eve/Software/ros2_ws && rm -rf build install log && source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

`px4_msgs` dauert dabei je ~4 Minuten. Prüfen, ob alles da ist:

```bash
source /opt/ros/humble/setup.bash && source ~/projects/PlastiX/Octopus/ros2_ws/install/setup.bash && ros2 pkg executables octopus_camera_transform
```

Erwartet: vier Einträge, darunter `flight_camera_transform_node`.

Beim Verschieben des Repos bricht auch das Detektor-`.venv` — `VIRTUAL_ENV` in
`eve/Software/detect-and-localize/.venv/bin/activate` zeigt dann auf den alten Pfad. Entweder
das venv neu anlegen oder die Pfade in `.venv/bin/` umschreiben.

---

## Terminal 1 — Pi: PX4-Brücke

```bash
ssh eve-pi
```

```bash
sudo pkill -f "[M]icroXRCEAgent"; export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0; sudo --preserve-env=ROS_DOMAIN_ID,ROS_LOCALHOST_ONLY MicroXRCEAgent serial --dev /dev/serial0 -b 921600 -v 6
```

**Terminal offen lassen.** Danach publiziert `/fmu/out/vehicle_odometry` mit ca. 70–100 Hz.

## Terminal 2 — Pi: Kamera

```bash
ssh eve-pi
```

```bash
pkill -f "[c]amera_node"; cd ~/PlastiX/eve/Software/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 && ros2 run camera_pkg camera_node --ros-args -p publish_raw:=false -p device_index:=0
```

**Terminal offen lassen.** Danach publiziert `/camera/image_raw/compressed` mit ca. 7–15 Hz.

## Terminal 3 — Laptop: Detektor

```bash
cd ~/projects/PlastiX/eve/Software/detect-and-localize && source .venv/bin/activate && source /opt/ros/humble/setup.bash && source ~/projects/PlastiX/eve/Software/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 && python ~/projects/PlastiX/eve/Software/ros2_ws/src/detection_pkg/detection_pkg/detector_node.py --ros-args -p detect_localize_path:=$HOME/projects/PlastiX/eve/Software/detect-and-localize -p model:=data/models/indoor_v8s.pt -p input_topic:=/camera/image_raw/compressed -p output_frame:=camera -p show_ui:=false -p thresh:=0.60 -p confirm_frames:=3 -p max_lost:=5 -p yolo_frameskip:=0 -p dist_thresh:=0.10 -p move_thresh:=0.10 -p confirmed_republish_period_sec:=1.0
```

**Terminal offen lassen.** Danach publiziert `/detector_node/confirmed` mit ca. 1 Hz.

Zur Wahl von `thresh` und `confirm_frames` siehe „Detektions-Schwelle einstellen" weiter unten.

## Terminal 4 — Laptop: Octopus-Stack

Startet das Dashboard-Backend **und** alle sieben ROS-Nodes:

```bash
cd ~/projects/PlastiX && OCTOPUS_MAPPING_MODE=indoor_static_mission ./Octopus/scripts/start_octopus_debug_stack.sh
```

Das Skript beendet vorher alte Prozesse, kann also gefahrlos wiederholt werden. Es löst die
Nodes per `setsid` von der Terminal-Session — das Terminal darf zu, ohne dass etwas stirbt.

Logs liegen in `/tmp/octopus_logs/`.

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
source /opt/ros/humble/setup.bash && source ~/projects/PlastiX/Octopus/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 && python3 ~/projects/PlastiX/Octopus/scripts/octopus_pipeline_health.py
```

Gut ist, wenn `/camera/image_raw/compressed`, `/fmu/out/vehicle_odometry`,
`/detector_node/confirmed`, `/octopus/detections_world` und `/octopus/map_patch` je einen
Publisher **und** einen Subscriber haben und beide Backend-Endpunkte `reachable` sind.

`/fmu/out/vehicle_local_position: no publisher` ist im Indoor-Modus in Ordnung — siehe unten.

Den entscheidenden Zustand einzeln abfragen:

```bash
source /opt/ros/humble/setup.bash && source ~/projects/PlastiX/Octopus/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 && ros2 topic echo --once /octopus/flight_camera_transform/status --field data | head -n 1 | python3 -m json.tool | grep -E '"state"|"transform_ready"|"last_input_detection_count"|"last_transformed_detection_count"|"last_projection_error"'
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

`thresh` filtert die YOLO-Detektionen in `detect-and-localize/src/yolo_detector.py` nach der
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
`best_with_inside_trash.pt` also auch `apriltag`, `human` und `robot`.

---

## Alles stoppen

```bash
cd ~/projects/PlastiX && ./Octopus/scripts/stop_octopus_debug_stack.sh
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
