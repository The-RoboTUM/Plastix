# GripperX-Link über rosbridge

> **Pfade.** Alle Pfade in diesem Dokument sind relativ zur Repo-Wurzel — dorthin wechseln, und zwar in *jedem* Terminal, das du dafür öffnest.

Betriebsdokument für die WebSocket-Brücke, über die der Sammelroboter GripperX an Octopus hängt.
Der Vertrag selbst — welche Topics, welche Felder — steht in
[`octopus_to_robot_interface.md`](octopus_to_robot_interface.md). Hier steht, wie der Transport
läuft, warum die Argumente so aussehen, und was kaputt aussieht, aber keins ist.

Die Anfrage des GripperX-Teams liegt in `~/projects/OCTOPUS_ROSBRIDGE_SETUP.md`, die Antwort
darauf in `~/projects/OCTOPUS_ROSBRIDGE_ANSWERS.md`.

## Was der Link ist

```text
GripperX                        Octopus-Host (ITQLM125, 10.42.0.158)
   │
   │  ws://10.42.0.158:9090
   ├──── subscribe ──────────►  /octopus/fake_eve_gps_start   Datum
   ├──── subscribe ──────────►  /octopus/trash_goal           nächstes Ziel
   ├──── subscribe ──────────►  /octopus/trash_gps            alle Ziele
   │
   ├──── publish ────────────►  /octopus/trash_goal_done      Ziel erledigt
   └──── publish ────────────►  /octopus/devices/gripperx/status   eigener Zustand
                                        │
                                        ▼
                               device_status_backend_bridge_node
                                        │  POST /api/devices/gripperx/status
                                        ▼
                               Dashboard: GripperX auf der Mission Map
```

Kein ROS auf der Roboterseite nötig, kein gemeinsames DDS, kein `ROS_DOMAIN_ID`-Abgleich über das
Netz. rosbridge exponiert nur, was schon auf dem Graphen liegt.

## Starten

Normalfall — die Brücke hängt am Debug-Stack:

```bash
./Octopus/scripts/start_octopus_debug_stack.sh
```

Nur die Brücke, im Vordergrund, mit Log auf dem Terminal:

```bash
./Octopus/scripts/run_rosbridge.sh
```

`run_rosbridge.sh` ist die **einzige** Stelle, an der die Startargumente stehen. Debug-Stack,
systemd-Unit und Handstart gehen alle darüber, damit der Link nicht auf drei leicht
unterschiedliche Weisen laufen kann. Überschreibbar per Umgebung:
`OCTOPUS_ROSBRIDGE_PORT`, `OCTOPUS_ROSBRIDGE_ADDRESS`, `OCTOPUS_ROSBRIDGE_TOPICS_GLOB`.

### Nach einem Reboot

```bash
sudo cp config/systemd/octopus-rosbridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now octopus-rosbridge
```

**Noch nicht installiert** — braucht root, und auf diesem Laptop gibt es kein passwortloses sudo.
Bis dahin kommt der Link mit dem Debug-Stack hoch.

`ROS_DOMAIN_ID` steht **absichtlich** in der Unit. Eine Unit erbt nichts von der interaktiven
Shell: eine Domain, die aus `.bashrc` kommt und beim Handstart funktioniert, fehlt dort — und
genau dieser Fehler sieht von der Roboterseite wie ein Netzproblem aus. Auch bei `0` hinschreiben.

## Prüfen

```bash
# vollständige Prüfung beider Richtungen, ohne ROS auf der Clientseite
python3 scripts/check_rosbridge.py            # gegen 127.0.0.1
python3 scripts/check_rosbridge.py 10.42.0.158

# der Link im Health-Check des Gesamtstacks
python3 scripts/octopus_pipeline_health.py

# GripperX simulieren: fährt zum Ziel, meldet es erledigt, Ziel rückt weiter
python3 scripts/simulate_gripperx.py
python3 scripts/simulate_gripperx.py --no-collect   # fährt, meldet aber nichts
```

`check_rosbridge.py` spricht genau das, was der echte Client spricht: `subscribe`, `advertise`,
`publish`, kurze Typform (`std_msgs/String`, nicht `std_msgs/msg/String`), kein rosapi, keine
Services, keine Parameter. Ein Durchlauf ist eine echte Probe, keine Annäherung.

**Die Dashboard-Seite dieser Kette hat einen eigenen Test ohne Browser:

```bash
node octopus-dashboard/tests/fleet_device_status.mjs
```

Er prüft, was auf der Mission Map falsch aussehen würde: dass ein Roboter ohne Datum **keine**
Position bekommt, dass eine fehlende Batterie nicht als 0 % erscheint, und dass ein stiller Link
die Bereitschaftsanzeige nicht weiter auf „ready" stehen lässt. `node` ist auf dem Demo-Laptop
nicht installiert.

`simulate_gripperx.py` meldet standardmäßig Ziele als erledigt.** Auf einem laufenden Demo-Stack
markiert das echte Detektionen als eingesammelt, und das ist **nicht zurücknehmbar**: ein Neustart
von `trash_gps_goal_node` löscht die Markierungen, aber ohne laufenden Detektor auch die Ziele
selbst. Mit `--no-collect` fährt der Simulator nur. Wer die Einsammel-Schleife testen will, nimmt
eine eigene Domain:

```bash
export ROS_DOMAIN_ID=42          # eigener Graph, rührt den Demo-Stack nicht an
ros2 run octopus_camera_transform trash_gps_goal_node &
OCTOPUS_ROSBRIDGE_PORT=9091 scripts/run_rosbridge.sh &
# synthetische Detektionen einspeisen, dann simulate_gripperx.py --port 9091
```

## Die Argumente, und warum sie so aussehen

Alle drei Punkte kommen aus dem Vorschlag des GripperX-Teams. Zwei davon **funktionieren so nicht**
auf Humble, einer fehlt dort.

| Argument | Was es tut |
|---|---|
| `--topics_glob "['/octopus/*']"` | nur `/octopus/…` ist erreichbar, in **beide** Richtungen |
| `--services_glob "[]"` | kein Service aufrufbar |
| `--actions_glob "[]"` | **kein Action-Goal** absetzbar |

### Als String, nicht als Liste

Auf rosbridge 2.0.7 (das, was Humble liefert) sind die Globs **String**-Parameter, die rosbridge
selbst parst. `ros2 launch ... topics_glob:="[/octopus/*]"` wird von ROS 2 zu `STRING_ARRAY`
umgetypt, und der Node stirbt beim Start:

```text
InvalidParameterTypeException: Trying to set parameter 'topics_glob' to
'['/octopus/*']' of type 'STRING_ARRAY', expecting type 'STRING'
```

Deshalb `ros2 run` mit `--topics_glob` statt `ros2 launch` — das Launch-File typt den Wert auf dem
Weg durch ebenfalls um.

### `/octopus/*` deckt auch das verschachtelte Topic

rosbridge matcht mit `fnmatch`, dort spannt `*` auch `/`. Der eine Eintrag deckt damit auch
`/octopus/devices/gripperx/status`. **Nachgemessen, nicht angenommen** — über den Link publiziert
und mit `ros2 topic echo` gegengelesen. Ein zweiter Eintrag `/octopus/devices/*` ist nicht nötig.

### `params_glob` gibt es auf 2.0.7 nicht mehr

Der Parameter wurde entfernt. Ihn zu übergeben wird **stillschweigend ignoriert** — keine Warnung.
Was Parameterzugriff tatsächlich zumacht, ist `services_glob` plus der nicht laufende
`rosapi`-Node, denn Parameter gehen über `/rosapi/get_param`.

### `actions_glob` ist der Punkt, der nicht im Vorschlag stand

Ohne gesetzten Action-Glob heißt auf 2.0.7 **nicht** "keine Actions", sondern "jeder
Action-Server auf dem Graphen". Aktuell liegt dort keiner, es war also nichts offen — aber die
Eigenschaft hing an einem leeren Graphen statt an der Konfiguration. Mit `[]` ist sie
konfiguriert, und eine Öffnung wäre ein sichtbarer Diff.

### `services_glob "[]"` schaltet rosapi nicht allein ab

rosbridge **hängt** `/rosapi/*` an jeden nicht-leeren Services-Glob an, aus `[]` wird also
`["/rosapi/*"]`. Harmlos ist das, weil wir den `rosapi`-Node nicht starten:

```json
{"op":"service_response","service":"/rosapi/topics",
 "values":"Service /rosapi/topics does not exist","result":false}
```

Alles außerhalb des Patterns bekommt gar keine Antwort.

## rosbridge ist hier ein Quell-Build

`ros-humble-rosbridge-suite` ist **nicht** installiert — kein passwortloses sudo auf dem Laptop.
Stattdessen Tag **2.0.7** (dieselbe Version wie das Debian-Paket) in einem eigenen Overlay:

```bash
scripts/build_rosbridge.sh        # klont, holt die pip-Abhängigkeiten, baut
```

Liegt in `ros2_ws_rosbridge/` und ist damit vom Octopus-Workspace getrennt — ein `colcon build` im
Hauptworkspace fasst es nicht an.

`tornado`, `pymongo` (liefert `bson`) und `cbor2` kommen aus `pip --user`; das apt-Paket hätte sie
als Systempakete mitgebracht. rosbridge importiert alle drei auf Modulebene, ein fehlendes ist
also ein Traceback beim Start, keine fehlende Teilfunktion.

**Wenn jemand mit root das Paket nachinstalliert, ändert sich am Link nichts.** `run_rosbridge.sh`
nimmt automatisch das apt-Paket, sobald `ros2 pkg prefix rosbridge_server` es findet; das Overlay
kann dann weg.

## Die Payload-Übersetzung

GripperX und das Dashboard benennen dieselben Dinge verschieden. Beide Formen sind
begründet und keine Seite ist umgezogen — `OCTOPUS_INTERFACE_PROPOSAL.md` führt das
als offenen Punkt. Übersetzt wird deshalb in `device_status_backend_bridge_node`,
der Naht zwischen Drahtformat und Dashboard; `live_data.js` sieht dadurch nur **eine**
Form.

| GripperX auf dem Draht | Dashboard liest |
|---|---|
| `device_id` | `robot_id` |
| `stamp` | `timestamp` |
| `nav_state`, `active_goal_id` (flach) | `nav.status`, `nav.active_goal_id` |
| `link_ok` | `link.connected` |
| `pose.status` + `pose.latlon_status` | `pose.status` |

`pose.lat`/`lon`, `x`/`y`/`yaw_deg` und der Batterie-Block passten schon vorher und
werden nicht angefasst.

**Die zwei Pose-Flags werden zu einem, und die Reihenfolge ist die Aussage.**
GripperX meldet Karten-Pose und lat/lon getrennt, weil die Pose exakt bekannt sein
kann, während das Datum fehlt — ein daraus gerechnetes lat/lon wäre eine erfundene
Position. Geprüft wird **Pose zuerst**: `goal_gateway_node` setzt bei ungültiger Pose
auch `latlon_valid = False` und kopiert die Pose-Begründung hinüber, ein kaputtes TF
macht also beides ungültig. Das als `no_datum` zu melden schickt den Betreiber auf
die Suche nach einem Datum, das in Ordnung ist.

| Zustand | `pose.status` | auf der Karte |
|---|---|---|
| Pose ungültig | `no_pose` | kein Marker |
| Pose gültig, kein Datum | `no_datum` | kein Marker, „waiting for datum" |
| beides gültig | `ok` | Marker |

Zwei Eigenschaften, die nicht verloren gehen dürfen, wenn jemand das anfasst:

- **Erkannt wird an der Form, nicht am Roboternamen** — `device_id` vorhanden und
  `robot_id` nicht. Ein zweiter Roboter, der die Dashboard-Form schon spricht, geht
  unverändert durch. Genau dafür ist `status_topics` ein Parameter.
- **Additiv** — die Originalfelder bleiben stehen, `pose.status` wird als
  `pose.source_status` aufgehoben. Im Backend ist damit die rohe Roboter-Payload
  weiter lesbar.

`nav.distance_remaining_m` bleibt `null`: GripperX publiziert es nicht, und eine
erfundene Zahl wäre schlechter als eine fehlende.

Testbar ohne Roboter:

```bash
python3 scripts/simulate_gripperx.py --dialect gripperx --no-collect
```

`--dialect dashboard` (Vorgabe) umgeht die Übersetzung, `gripperx` sendet die Form
des echten Roboters. Die Übersetzung selbst hat Unit-Tests in
`ros2_ws/src/octopus_backend_bridge/test/test_normalise_device_payload.py`.

## websockets-Versionen

**Die zwei Seiten des Links brauchen gegenläufige Versionen derselben Bibliothek.** Das ist
kein Versehen und lässt sich nicht durch ein Upgrade auflösen.

| Seite | Import | Braucht |
|---|---|---|
| Octopus: `scripts/check_rosbridge.py`, `scripts/simulate_gripperx.py` | `websockets.asyncio.client` | **>= 13** (davor gibt es das Modul nicht) |
| GripperX: `gripperx_external/rosbridge_client.py` | `websockets.client` | **< 14** (ab 14 entfernt) |

Auf einem Ubuntu-24.04-Host liefert `python3-websockets` die **10.4** — genau das, was
GripperX braucht, und zu alt für die beiden Prüfskripte. Läuft Octopus in einer eigenen
Umgebung (distrobox, Container, venv) mit einer neueren Version, geht beides gleichzeitig
auf; auf diesem Laptop ist das so, Octopus hat dort 16.1.

Die Prüfskripte gehören deshalb in die **Octopus**-Umgebung, nicht auf den GripperX-Host.
Dort aufgerufen sterben sie mit

```text
ModuleNotFoundError: No module named 'websockets.asyncio'
```

Das ist kein kaputtes Setup, sondern das falsche Terminal. Umgekehrt gilt dasselbe: ein
`pip install -U websockets` auf dem GripperX-Host repariert die Skripte und **zerlegt den
Link**, weil `rosbridge_client.py` die alte API importiert. Der Kommentar dort sagt es
ausdrücklich — nicht per pip installieren.

## Fehlerbilder

| Symptom | Fast sicher |
|---|---|
| `connection refused` | rosbridge läuft nicht, oder Port/Netz. `ss -ltn \| grep 9090` |
| verbindet, dann **Stille** | Domain-Mismatch: rosbridge hängt nicht am Graphen der Nodes. Oder das Topic publiziert wirklich nichts |
| `status`-Frames mit `"level":"error"` und Topicname | Glob deckt das Topic nicht |
| Node stirbt beim Start mit `InvalidParameterTypeException` | Globs als Liste statt als String übergeben — siehe oben |
| `ModuleNotFoundError: bson` / `cbor2` / `tornado` | pip-Abhängigkeiten fehlen, `scripts/build_rosbridge.sh` |
| `ModuleNotFoundError: websockets.asyncio` | Prüfskript auf dem GripperX-Host statt in der Octopus-Umgebung gestartet — siehe oben |
| Link stirbt beim Start mit `ImportError` auf `websockets` | Auf der GripperX-Seite fehlt `python3-websockets`, oder es ist eine Version >= 14 installiert |
| `bson` da, aber "does not support all necessary features" | falsches `bson`-Paket. rosbridge braucht das von **pymongo**, nicht das PyPI-`bson` |
| GripperX im Dashboard, aber ohne Marker | Roboter meldet `pose.status: no_datum` — das ist korrekt und steht so im Panel |
| GripperX da, Status steht auf `pose available` | Die Übersetzung lief nicht — Payload kam mit `robot_id` an, also nicht als GripperX-Dialekt erkannt |
| Marker fehlt, Panel sagt `no_pose` | Nicht das Datum, sondern TF `map→base_footprint` auf der Roboterseite |
| GripperX-Karte amber, `link lost` | seit >6 s kein Status. Roboter oder Link weg, nicht das Dashboard |
| GripperX fehlt im Dashboard ganz | `device_status_backend_bridge_node` läuft nicht, oder das Backend ist neu gestartet und noch nichts angekommen |

**Der Domain-Mismatch ist das teuerste Fehlerbild**, weil er von außen wie ein Netzproblem
aussieht: der Client verbindet sich, bekommt aber nie Daten. rosbridge muss auf derselben
`ROS_DOMAIN_ID` laufen wie die Octopus-Nodes — hier `0`.

## Gemessene Zeiten

Client-seitig durch rosbridge, 30 s, ruhiger Graph:

| Topic | Rate | mittlere Lücke | **größte Lücke** | Jitter σ |
|---|---|---|---|---|
| `/octopus/fake_eve_gps_start` | 1.03 Hz | 0.985 s | 1.001 s | 0.081 s |
| `/octopus/trash_goal` | 1.03 Hz | 0.988 s | 1.001 s | 0.063 s |
| `/octopus/trash_gps` | 1.00 Hz | 1.000 s | 1.001 s | 0.000 s |

Gemessen über Loopback und die eigene LAN-Adresse. **Der Weg von GripperX aus ist damit nicht
gemessen** — dafür braucht es die zweite Maschine, und WLAN bringt Jitter mit, der hier nicht
drinsteht.

## Was der Link nicht ist

- **Kein TLS, keine Authentifizierung.** Reines `ws://` im lokalen Netz, beidseitig bewusst
  zurückgestellt. Verlässt der Link ein vertrautes Netz, wird das neu aufgemacht.
- **Kein Bewegungskanal.** Keine Action, kein Service, kein Parameter, keine Topics außerhalb
  `/octopus/*` — und das jetzt per Konfiguration, nicht nur, weil gerade kein Action-Server läuft.
- **Kein Task-Management.** Zwei Sammelroboter bekommen dasselbe Ziel; das ist eine Eigenschaft
  von `trash_gps_goal_node`, nicht des Transports.
