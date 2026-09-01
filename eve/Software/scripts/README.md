# Octopus-Kamera-Skripte (Pi-Seite)

Diese Skripte laufen auf der Raspberry Pi der Drohne und werden vom
Octopus-Dashboard per SSH aufgerufen. Sie starten, stoppen und beobachten die
beiden Prozesse, die Octopus von der Pi braucht -- den `camera_node` aus
`../ros2_ws/src/camera_pkg` und den `MicroXRCEAgent` zum Pixhawk -- ohne dass
jemand ein Terminal offen halten muss.

Das Gegenstück ist `Octopus/octopus-dashboard/api.py` auf Branch
**`eve-octopus`** — dieser Branch hier kennt es nicht. Wer eines der beiden
Enden ändert, muss das andere mitändern.

### Kamera

| Skript | Aufruf aus `api.py` | Erwartete Ausgabe |
|---|---|---|
| `octopus_camera_status.sh` | `GET /api/eve/status` | `camera_running` / `camera_not_running` |
| `octopus_start_camera.sh` | `POST /api/eve/start_camera` | `camera_started` |
| `octopus_stop_camera.sh` | `POST /api/eve/stop_camera` | `camera_stopped` / `camera_not_running` |
| — | `GET /api/eve/camera_log` | `tail -80 /tmp/octopus_camera_node.log` |

### PX4-Brücke (MicroXRCEAgent)

| Skript | Aufruf aus `api.py` | Erwartete Ausgabe |
|---|---|---|
| `octopus_px4_bridge_status.sh` | `GET /api/eve/px4_bridge/status` | `px4_bridge_running` / `px4_bridge_not_running` |
| `octopus_start_px4_bridge.sh` | `POST /api/eve/px4_bridge/start` | `px4_bridge_started` plus `pixhawk=connected` oder `pixhawk=waiting` |
| `octopus_stop_px4_bridge.sh` | `POST /api/eve/px4_bridge/stop` | `px4_bridge_stopped` / `px4_bridge_not_running` |
| — | `GET /api/eve/px4_bridge/log` | `tail -80 /tmp/octopus_px4_bridge.log` |

`pixhawk=waiting` heißt: der Agent läuft, aber es hat sich keine Session
gebildet. Der Agent selbst ist dann in Ordnung -- fast immer ist der Pixhawk
stromlos oder das serielle Kabel ab.

`octopus_camera_env.sh` und `octopus_px4_bridge_env.sh` halten die gemeinsamen
Pfade und werden von den jeweils drei Skripten gesourct.

## Installation auf der Pi

Keine. Ein `git pull` auf diesem Branch genügt — das Dashboard ruft die Skripte unter
ihrem Repo-Pfad auf.

Liegt das Repo nicht unter `~/PlastiX`, muss das Backend das wissen. Gesetzt wird das
dort, wo `api.py` läuft, also auf dem Laptop:

```bash
export OCTOPUS_EVE_SCRIPT_DIR='~/mein/pfad/eve/Software/scripts'
```

Stimmt der Pfad nicht, meldet das Dashboard `camera_failed`; die Fehlermeldung der
Remote-Shell steht dann im *„Camera Log"*.

## Was man beim Ändern wissen muss

- **Die Ausgabewörter sind der Vertrag.** Das Dashboard entscheidet allein
  daran, nicht am Exit-Code. `camera_started`, `camera_stopped`,
  `camera_running`, `camera_not_running` müssen so im stdout stehen bleiben.
- **Kein `sudo`, in keinem der Skripte.** Der Aufruf aus dem Dashboard ist
  nicht interaktiv (`ssh -o BatchMode=yes`); eine Passwortabfrage würde ihn bis
  zum Timeout hängen lassen. Der `MicroXRCEAgent` kommt ohne aus, weil
  `/dev/ttyAMA0` `root:dialout` gehört und der Benutzer in `dialout` ist -- wer
  das ändert, macht den Start-Button unbrauchbar.
- **Der Logpfad `/tmp/octopus_camera_node.log` ist in `api.py` verdrahtet.**
- **640×480 hängt an den Kamera-Intrinsics** der Octopus-Seite
  (`OCTOPUS_HBVCAM_640X480`). Auflösung nur ändern, wenn die Kamera denselben
  Bildwinkel behält.
- **Workspace überschreiben:** `EVE_WS=/pfad/zum/ros2_ws ./octopus_start_camera.sh`.
  Ohne die Variable nehmen die Skripte den `ros2_ws` neben diesem Ordner und
  als letzten Ausweg `~/PlastiX/eve/Software/ros2_ws`.

## Ohne Dashboard testen

```bash
cd ~/PlastiX/eve/Software/scripts

./octopus_start_camera.sh && ./octopus_camera_status.sh
ros2 topic hz /camera/image_raw/compressed
./octopus_stop_camera.sh

./octopus_start_px4_bridge.sh && ./octopus_px4_bridge_status.sh
ros2 topic hz /fmu/out/vehicle_odometry
./octopus_stop_px4_bridge.sh
```

Die beiden `ros2 topic hz` laufen auch vom Laptop aus, wenn dort
`ROS_DOMAIN_ID=0` und `ROS_LOCALHOST_ONLY=0` gesetzt sind -- das prüft
gleichzeitig, dass die Topics übers Netz ankommen.
