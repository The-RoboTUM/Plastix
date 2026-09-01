# Octopus-Kamera-Skripte (Pi-Seite)

Diese Skripte laufen auf der Raspberry Pi der Drohne und werden vom
Octopus-Dashboard per SSH aufgerufen. Sie starten, stoppen und beobachten den
`camera_node` aus `../ros2_ws/src/camera_pkg`, ohne dass jemand ein Terminal
offen halten muss.

Das Gegenstück ist `Octopus/octopus-dashboard/api.py` auf Branch
**`eve-octopus`** — dieser Branch hier kennt es nicht. Wer eines der beiden
Enden ändert, muss das andere mitändern.

| Skript | Aufruf aus `api.py` | Erwartete Ausgabe |
|---|---|---|
| `octopus_camera_status.sh` | `GET /api/eve/status` | `camera_running` / `camera_not_running` |
| `octopus_start_camera.sh` | `POST /api/eve/start_camera` | `camera_started` |
| `octopus_stop_camera.sh` | `POST /api/eve/stop_camera` | `camera_stopped` / `camera_not_running` |
| — | `GET /api/eve/camera_log` | `tail -80 /tmp/octopus_camera_node.log` |

`octopus_camera_env.sh` hält die gemeinsamen Pfade und wird von allen dreien
gesourct.

## Installation auf der Pi

Keine. Ein `git pull` auf diesem Branch genügt — das Dashboard ruft die Skripte unter
ihrem Repo-Pfad auf.

Liegt das Repo nicht unter `~/PlastiX`, muss das Backend das wissen. Gesetzt wird das
dort, wo `api.py` läuft, also auf dem Laptop:

```bash
export OCTOPUS_EVE_SCRIPT_DIR='~/mein/pfad/eve/Software/scripts'
```

Findet das Backend dort nichts Ausführbares, fällt es auf `~/octopus_*.sh` im Home der Pi
zurück. Das ist nur der Übergang für eine Pi, auf der die Skripte noch als lose Kopien
liegen; neu einrichten muss man das nicht.

## Was man beim Ändern wissen muss

- **Die Ausgabewörter sind der Vertrag.** Das Dashboard entscheidet allein
  daran, nicht am Exit-Code. `camera_started`, `camera_stopped`,
  `camera_running`, `camera_not_running` müssen so im stdout stehen bleiben.
- **Kein `sudo`.** Der Aufruf aus dem Dashboard ist nicht interaktiv
  (`ssh -o BatchMode=yes`); eine Passwortabfrage würde ihn bis zum Timeout
  hängen lassen. Der `MicroXRCEAgent` braucht deshalb weiterhin ein
  interaktives Terminal und hat hier kein Skript.
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
```
