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

**Voraussetzung:** Ubuntu 22.04 mit ROS 2 Humble auf dem Laptop und gebautem
`Octopus/ros2_ws`, ROS 2 Humble auf der Pi mit gebautem `eve/Software/ros2_ws`. Was dafür
auf der Pi einmal einzurichten ist, steht in
[Einmalig auf der Eve-Pi](#einmalig-auf-der-eve-pi).

Ohne Drohne, ohne Pi, ohne Detektor — nur das Dashboard, optional mit GripperX auf der
Mission Map: **[SETUP_without_drone.md](SETUP_without_drone.md)**. Dieses Dokument hier
setzt die Drohne voraus.

Wer es eilig hat: [Der eine Befehl](#der-eine-befehl) startet alles auf einmal. Die
Abschnitte davor sind Einmal-Einrichtung, die danach Fehlersuche.

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

## Einmalig (Laptop): SSH-Alias für die Pi

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

## Einmalig auf der Eve-Pi

Auf der aktuellen Drohne ist das alles schon eingerichtet — dieser Abschnitt ist für eine
neue Pi, eine neue SD-Karte oder wenn ein Schritt nachweislich fehlt. Der Prüfblock am Ende
sagt in einem Rutsch, was davon steht.

Stand der laufenden Pi: Ubuntu 22.04 (arm64), ROS 2 Humble, Benutzer `eve`, Hostname
`eve-desktop`. Alle Befehle hier laufen **auf der Pi** (`ssh eve-pi`), sofern nicht anders
vermerkt.

### 1. Repo und Workspace

Die Pi steht auf Branch `eve_ros_development` mit dem Ordner `eve/` — nicht auf
`eve-octopus`. `px4_msgs` hängt dort als Submodul im Workspace, ohne `--recurse-submodules`
baut nichts:

```bash
git clone --recurse-submodules https://gitex.itq.de/cirqmind/PlastiX.git ~/PlastiX && cd ~/PlastiX && git checkout eve_ros_development && git submodule update --init --recursive
```

```bash
cd ~/PlastiX/eve/Software/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

`px4_msgs` dauert auf der Pi deutlich länger als auf dem Laptop. Danach müssen
`camera_pkg`, `detection_pkg` und `px4_msgs` unter `install/` liegen. Die drei
Kamera-Skripte, die das Dashboard aufruft, kommen mit demselben Klon
(`eve/Software/scripts/`) und brauchen keinen weiteren Schritt. Der Octopus-Ordner
wird auf der Pi **nicht** gebraucht: sie liefert nur `/camera/image_raw/compressed` und
`/fmu/out/vehicle_odometry`.

### 2. SSH-Key vom Laptop

Der SSH-Alias oben ist die Laptop-Seite; hier fehlt noch der öffentliche Schlüssel in
`~/.ssh/authorized_keys` der Pi. **Vom Laptop aus:**

```bash
ssh-copy-id eve-pi
```

Passwortloser Key-Login ist keine Bequemlichkeit, sondern Pflicht: `api.py` ruft die Pi mit
`ssh -o BatchMode=yes` auf, und BatchMode bricht bei jeder Passwortabfrage sofort ab. Der
Button *„Connect Eve Camera"* im Dashboard meldet dann nur `offline`.

### 3. Kamera-Skripte

**Hier ist nichts zu tun.** `api.py` startet, stoppt und pollt die Kamera über drei
Skripte auf der Pi, und die liegen im Pi-Branch unter `eve/Software/scripts/` — mit
Schritt 1 sind sie also schon da, ausführbar und am richtigen Ort. Das Dashboard ruft genau
diesen Pfad auf.

| Skript in `eve/Software/scripts/` | Endpunkt | Erwartete Ausgabe |
|---|---|---|
| `octopus_camera_status.sh` | `GET /api/eve/status` | `camera_running` / `camera_not_running` |
| `octopus_start_camera.sh` | `POST /api/eve/start_camera` | `camera_started` |
| `octopus_stop_camera.sh` | `POST /api/eve/stop_camera` | `camera_stopped` |

Das Dashboard entscheidet allein an diesen Wörtern im stdout, nicht am Exit-Code — wer die
Skripte anpasst, muss die Wörter stehen lassen. Das Log, das *„Camera Log"* im Dashboard
zeigt, ist fest `/tmp/octopus_camera_node.log`.

Liegt das Repo auf der Pi woanders als unter `~/PlastiX`, muss das Backend das wissen —
gesetzt wird das auf dem **Laptop**, vor dem Start des Stacks:

```bash
export OCTOPUS_EVE_SCRIPT_DIR='~/mein/pfad/eve/Software/scripts'
```

Ebenso `OCTOPUS_EVE_SSH_TARGET`, falls der SSH-Alias nicht `eve-pi` heißt. Stimmt der Pfad
nicht, meldet das Dashboard `camera_failed` und im *„Camera Log"* steht ein
`No such file or directory` der Remote-Shell.

Kein `sudo` in den Skripten, und das ist Absicht: der Dashboard-Aufruf ist nicht
interaktiv, ein `sudo` mit Passwortabfrage würde ihn hängen lassen, bis der Timeout greift.
Der `MicroXRCEAgent` aus Terminal 1 lässt sich deshalb nicht auf demselben Weg starten.

Das Start-Skript sucht die Kamera selbst (bevorzugt über `/dev/v4l/by-id`, sonst das erste
`/dev/videoX` mit MJPG) und startet den Node mit **640×480**. Diese Auflösung hängt an den
Intrinsics `OCTOPUS_HBVCAM_640X480` in `octopus-dashboard/live_data.js` — sie hochzudrehen
hat dieselbe Konsequenz wie in Terminal 2, die Warnung dort gilt genauso.

Der Button startet die Kamera also **ohne** Terminal 2. Beide Wege parallel zu benutzen ist
unnötig, aber ungefährlich: das Start-Skript killt einen alten `camera_node` vorher.

Details zum Vertrag zwischen den Skripten und `api.py` stehen in
`eve/Software/scripts/README.md` auf dem Pi-Branch.

### 4. UART für den MicroXRCEAgent

Terminal 1 spricht über `/dev/serial0` mit dem Pixhawk. Dafür muss dreierlei stimmen:

```bash
grep -E "enable_uart|disable-bt" /boot/firmware/config.txt
```

Erwartet `enable_uart=1` und `dtoverlay=disable-bt` — ohne das zweite hängt Bluetooth an
`ttyAMA0` und die serielle Verbindung ist entweder weg oder unzuverlässig. Fehlt eine
Zeile, anhängen und **neu starten**.

```bash
sudo systemctl disable --now serial-getty@ttyAMA0.service
```

Die serielle Konsole muss aus sein, sonst greifen Login-Prompt und Agent nach derselben
Schnittstelle. `console=serial0` darf aus demselben Grund nicht in
`/boot/firmware/cmdline.txt` stehen (auf der laufenden Pi steht dort nur `console=tty1`).

```bash
groups | grep -q dialout || sudo usermod -aG dialout $USER
```

Wirkt erst nach neuem Login.

Der Agent selbst liegt auf der laufenden Pi unter `/usr/local/bin/MicroXRCEAgent`, kommt
also aus einem Quell-Build und **nicht** aus apt. Falls er fehlt:

```bash
sudo apt install -y build-essential cmake git && git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ~/Micro-XRCE-DDS-Agent && cd ~/Micro-XRCE-DDS-Agent && mkdir build && cd build && cmake .. && make -j2 && sudo make install && sudo ldconfig /usr/local/lib/
```

Der Agent braucht **kein** `sudo`: `/dev/ttyAMA0` gehört `root:dialout`, und der Benutzer
`eve` ist in dieser Gruppe. Deshalb lässt sich die PX4-Brücke genauso per Dashboard-Button
starten wie die Kamera.

### 5. USB-Kamera

```bash
sudo apt install -y v4l-utils
groups | grep -q video || sudo usermod -aG video $USER
```

`v4l2-ctl` braucht das Start-Skript für seinen Fallback-Pfad. Ob die Kamera erkannt wird:

```bash
ls /dev/v4l/by-id/
```

Erwartet einen Eintrag auf `-video-index0` (auf der Drohne
`usb-Generic_USB_camera_...-video-index0`). Das Skript nimmt bevorzugt diesen stabilen
Pfad, weil sich `/dev/videoX` zwischen zwei Boots verschieben kann.

### 6. ROS-Umgebung in der `.bashrc`

```bash
grep -qF 'source /opt/ros/humble/setup.bash' ~/.bashrc || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
grep -qF 'export ROS_LOCALHOST_ONLY=0' ~/.bashrc || echo 'export ROS_LOCALHOST_ONLY=0' >> ~/.bashrc
```

`ROS_DOMAIN_ID` bleibt ungesetzt und damit auf dem Default `0` — genau dem Wert, den der
Laptop benutzt. Wer ihn auf der Pi setzt, muss ihn auf beiden Seiten gleich setzen, sonst
sehen sich Pi und Laptop nicht.

### Alles prüfen

Vom Laptop aus, in einem Rutsch:

```bash
ssh -o BatchMode=yes eve-pi 'hostname; ls ~/PlastiX/eve/Software/scripts/octopus_*.sh | wc -l; ls ~/PlastiX/eve/Software/ros2_ws/install | head -3; ls -l /dev/serial0; ls /dev/v4l/by-id/; which MicroXRCEAgent v4l2-ctl; id -nG'
```

Gut ist: der Befehl fragt **nicht** nach einem Passwort (sonst Schritt 2), meldet `3`
Skripte, zeigt `camera_pkg` unter `install/`, `/dev/serial0`, mindestens ein
`*-video-index0`, beide Binaries und `dialout` sowie `video` in den Gruppen.

Funktionstest der Kamera ohne Dashboard:

```bash
ssh -o BatchMode=yes eve-pi '~/PlastiX/eve/Software/scripts/octopus_start_camera.sh' && ssh -o BatchMode=yes eve-pi '~/PlastiX/eve/Software/scripts/octopus_camera_status.sh'
```

Erwartet `camera_started …` und danach `camera_running pids=…`. Kommt `camera_failed`,
steht der Grund in `/tmp/octopus_camera_node.log` — im Dashboard unter *„Camera Log"*,
oder direkt mit `ssh eve-pi 'tail -80 /tmp/octopus_camera_node.log'`.

---

## Einmalig (Laptop): Workspace neu bauen

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

## Einmalig (Laptop): venv für den Detektor

**Terminal 3 startet ohne diesen Schritt nicht.** Das venv ist per `.gitignore` ausgeschlossen,
kommt also mit keinem `git clone` und mit keinem `git pull` mit — auf einem frisch geholten Repo
ist es nie da. Der Fehler sieht so aus:

```text
bash: .venv/bin/activate: Datei oder Verzeichnis nicht gefunden
```

Anlegen (kein `sudo` nötig):

```bash
cd Octopus/detect-and-localize && python3 -m venv .venv && source .venv/bin/activate && pip install --upgrade pip && pip install -r requirements.txt
```

Das dauert: `ultralytics` zieht `torch` nach, das sind ein paar GB. `rclpy` gehört bewusst
**nicht** in das venv — es kommt über `PYTHONPATH` aus `/opt/ros/humble/setup.bash`, deshalb
werden in Terminal 3 beide Quellen nacheinander gesourct.

Prüfen:

```bash
cd Octopus/detect-and-localize && source .venv/bin/activate && python -c "import ultralytics, cv2, pupil_apriltags; print('ok')"
```

Zwei Fälle, in denen das venv **erneut** fällig ist:

- **Das Repo wurde verschoben.** `VIRTUAL_ENV` in `.venv/bin/activate` zeigt dann auf den alten
  Pfad. Entweder neu anlegen oder die Pfade in `.venv/bin/` umschreiben.
- **`detect-and-localize` wurde innerhalb des Repos verschoben** — der Ordner lag früher unter
  `eve/Software/`. Ein ignoriertes venv wandert bei so einem Move nicht mit und bleibt am alten
  Ort liegen, falls es den noch gibt.

Und noch eine Voraussetzung für Terminal 3, aus demselben Grund: **das YOLO-Modell ist nicht im
Repo** (siehe oben, `.gitignore`). Der Befehl in Terminal 3 zeigt auf
`data/models/best_model_10_08_26.pt` — die Datei muss separat nach
`Octopus/detect-and-localize/data/models/` kopiert werden, sonst startet der Node auch mit
fertigem venv nicht. Was lokal liegt, zeigt `ls Octopus/detect-and-localize/data/models/`.

---

## Der eine Befehl

```bash
OCTOPUS_MAPPING_MODE=indoor_static_mission ./Octopus/scripts/start_octopus_debug_stack.sh
```

Das ist alles. Der Befehl startet in dieser Reihenfolge:

1. **Laptop:** Dashboard-Backend, elf ROS-Nodes, rosbridge
2. **Pi (per SSH):** PX4-Brücke und Kamera
3. **Laptop:** den YOLO-Detektor

Danach steht die Demo unter `http://127.0.0.1:8000/dashboard.html`. Die vier Terminals
weiter unten braucht man dafür **nicht** — sie stehen da, weil man sie einzeln zum Debuggen
oder mit abweichenden Parametern starten können will.

**Ist die Pi nicht erreichbar, läuft der Befehl trotzdem durch** und meldet es:

```text
WARNING: Eve (eve-pi) not reachable - continuing without her.
         The dashboard is up. Start the PX4 bridge and the camera later
         from the 'Camera & Pipeline' panel once Eve is on the network.
```

Das Dashboard ist dann oben, und sobald die Drohne im Netz ist, holt man sie über die
Buttons dazu. Dasselbe gilt für den Detektor: fehlt das venv oder das Modell, sagt das
Skript das und macht weiter.

### Vom Dashboard aus steuern

Im Panel *Camera & Pipeline* gibt es je einen Block mit Start/Stop/Status/Log für:

| Block | Was | Wo |
|---|---|---|
| **Eve Camera** | `camera_node` | Pi |
| **Eve PX4 Bridge** | `MicroXRCEAgent` | Pi |
| **Detector (this machine)** | YOLO | Laptop |

Damit lässt sich jedes der drei Teile einzeln neu starten, ohne den Stack anzufassen —
praktisch nach einer geänderten Detektionsschwelle oder wenn die serielle Verbindung
abgerissen ist.

Zwei Dinge, die die Anzeige bewusst unterscheidet:

- **`Detector loading model`** ist kein Fehler. YOLO braucht nach dem Start ein paar
  Sekunden; erst danach steht `Detector running`.
- **`agent up, but no Pixhawk session yet`** heißt: der Agent läuft, aber der Pixhawk
  meldet sich nicht. Fast immer ist er dann stromlos oder die serielle Verbindung ist
  abgezogen — nicht ein Problem der Brücke selbst.

### Was der Befehl noch kennt

| Variable | Default | Wirkung |
|---|---|---|
| `OCTOPUS_START_EVE` | `true` | `false` = Pi gar nicht anfassen, nur Laptop-Seite |
| `OCTOPUS_START_DETECTOR` | `true` | `false` = Detektor weglassen |
| `OCTOPUS_EVE_SSH_TARGET` | `eve-pi` | anderer SSH-Alias |
| `OCTOPUS_EVE_SCRIPT_DIR` | `~/PlastiX/eve/Software/scripts` | anderer Repo-Pfad auf der Pi |

Beim Stoppen bleibt die Pi absichtlich unangetastet — meistens startet man nur den Laptop
neu, und eine mitten in der Demo abgeschaltete Kamera wäre die unangenehmere Überraschung:

```bash
./Octopus/scripts/stop_octopus_debug_stack.sh              # nur Laptop
OCTOPUS_STOP_EVE=true ./Octopus/scripts/stop_octopus_debug_stack.sh   # Pi mit
```

---

## Die vier Terminals einzeln

Ab hier die manuelle Variante: dieselben vier Prozesse, einzeln gestartet, jeder in einem
eigenen Terminal mit sichtbarer Ausgabe. Für den normalen Demo-Aufbau nicht nötig — für
Fehlersuche und abweichende Parameter schon.

## Terminal 1 — Pi: PX4-Brücke

Läuft auf der Pi. Der Code dazu liegt auf Branch `eve_ros_development` im Ordner `eve/`
und ist nicht Teil des Octopus-Setups. Agent und UART müssen einmal eingerichtet sein —
[Einmalig auf der Eve-Pi, Schritt 4](#4-uart-für-den-microxrceagent).

```bash
ssh eve-pi
```

```bash
pkill -f "[M]icroXRCEAgent"; export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0; MicroXRCEAgent serial --dev /dev/serial0 -b 921600 -v 6
```

**Terminal offen lassen.** Danach publiziert `/fmu/out/vehicle_odometry` mit ca. 70–100 Hz.

**Kein `sudo`** — frühere Fassungen dieses Dokuments hatten es, nötig ist es nicht:
`/dev/ttyAMA0` gehört `root:dialout` und der Benutzer `eve` ist in `dialout`. Das ist keine
Kosmetik, sondern die Voraussetzung dafür, dass der Start-Button im Dashboard überhaupt
funktionieren kann: der SSH-Aufruf von dort ist nicht interaktiv und würde an einer
Passwortabfrage bis zum Timeout hängen.

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

Statt dieses Terminals tut es auch der Button *„Connect Eve Camera"* im Dashboard — er ruft
`eve/Software/scripts/octopus_start_camera.sh` auf der Pi auf, das die Kamera selbst sucht und den Node im
Hintergrund startet ([Einmalig auf der Eve-Pi, Schritt 3](#3-kamera-skripte)).
Beides zugleich ist nicht nötig; das Skript beendet einen laufenden `camera_node` vorher.

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

Setzt „Einmalig (Laptop): venv für den Detektor" oben voraus, sonst scheitert der Befehl an
`.venv/bin/activate` — und danach am fehlenden Modell.

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

Derselbe Befehl wie oben — er ist die Grundlage, Terminal 1 bis 3 sind seine Handarbeit-
Variante. Startet Dashboard-Backend, alle elf ROS-Nodes, rosbridge und (sofern nicht
abgeschaltet) Pi-Seite und Detektor:

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

Der Unterschied zur [Variante ohne Drohne](SETUP_without_drone.md): dort erscheint GripperX
nur als Marker auf der Karte. Hier läuft zusätzlich `trash_gps_goal_node`, also fließen
**echte Müllziele aus dem Detektor** über `/octopus/trash_goal` und `/octopus/trash_gps` —
der eigentliche Anwendungsfall. Die Topics stehen in
[octopus_to_robot_interface.md](octopus_to_robot_interface.md).

Vor dem Start der GripperX-Seite im Dashboard **Set Eve** setzen. Ohne dieses Datum meldet
GripperX `no_datum` und bekommt bewusst keinen Marker.

Die GripperX-Seite selbst ist nicht Teil dieses Dokuments — siehe „GripperX-Seite" in
[SETUP_without_drone.md](SETUP_without_drone.md#gripperx-seite). Sie läuft auf ROS 2 Jazzy und `ROS_DOMAIN_ID=220`; das ist **kein**
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
