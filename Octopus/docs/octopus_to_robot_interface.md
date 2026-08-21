# Octopus → Roboter Interface (GPS-Ziele)

Gegenstück zu [`drone_to_octopus_interface.md`](drone_to_octopus_interface.md): dort schickt
Eve ihre Detektionen **an** Octopus, hier gibt Octopus die gefundenen Müllpositionen **an
einen Sammelroboter** weiter, der mit Nav2 hinfährt.

Alles in diesem Dokument bezieht sich auf die **Indoor-Fake-GPS-Demo**. Es gibt keinen
Satellitenempfang; die Koordinaten sind konstruiert. Nutzbar sind sie, weil Drohne und
Roboter denselben Bezugspunkt teilen — siehe [Das Datum](#das-datum).

## Topics auf einen Blick

| Topic | Typ | QoS | Richtung | Inhalt |
|---|---|---|---|---|
| `/octopus/fake_eve_gps_start` | `sensor_msgs/NavSatFix` | latched, 1 Hz | Octopus → Roboter | gemeinsamer Startpunkt (Datum) |
| `/octopus/trash_goal` | `sensor_msgs/NavSatFix` | latched, 1 Hz | Octopus → Roboter | **nächstes** Ziel |
| `/octopus/trash_gps` | `std_msgs/String` (JSON) | 1 Hz | Octopus → Roboter | **alle** bekannten Ziele |
| `/octopus/trash_goal_done` | `std_msgs/String` | — | Roboter → Octopus | erledigte Ziel-`id` |
| `/octopus/devices/<id>/status` | `std_msgs/String` (JSON) | 2 Hz | Roboter → Octopus | Zustand des Roboters, fürs Dashboard |

`latched` = `TRANSIENT_LOCAL`, Tiefe 1. Ein Roboter, der später bootet, bekommt Datum und
aktuelles Ziel sofort beim Verbinden statt erst beim nächsten Zyklus. Ein Subscriber mit
`VOLATILE` funktioniert trotzdem.

Erzeugt werden die Topics von zwei Nodes:

```text
Eve-Marker im Dashboard ziehen
  → POST /api/eve/fake_gps
  → eve_fake_gps_bridge_node        (octopus_backend_bridge)
  → /octopus/fake_eve_gps_start
                                     ↘
/octopus/detections_world            → trash_gps_goal_node   (octopus_camera_transform)
                                     ↘
                          /octopus/trash_goal + /octopus/trash_gps
```

## Transport

Zwei Wege, dieselben Topics:

- **Gemeinsames DDS.** Roboter auf derselben `ROS_DOMAIN_ID` (hier `0`) sieht die Topics direkt.
- **rosbridge.** WebSocket auf `ws://<host>:9090`, kein ROS auf der Roboterseite nötig. So hängt
  GripperX dran. Betrieb, Argumente und Fehlerbilder:
  [`gripperx_rosbridge_link.md`](gripperx_rosbridge_link.md).

Über rosbridge ist nur `/octopus/*` erreichbar, in beide Richtungen; Services, Parameter und
Actions sind zu. Am Vertrag in diesem Dokument ändert der Transport nichts — auch nicht an den
Typen. Zu beachten ist nur die **kurze Typform** im Protokoll: `std_msgs/String`, nicht
`std_msgs/msg/String`.

## Das Datum

Der Roboter wird **auf demselben Punkt gestartet wie Eve**. Dieser Punkt ist das Datum: alle
Müllkoordinaten sind relativ zu ihm ausgedrückt. Ohne diese gemeinsame Verankerung wären die
erfundenen lat/lon-Werte wertlos.

Das Datum ist die Position des Eve-Markers auf der Mission Map. Wird der Marker im Dashboard
verschoben, wandern **alle** Ziele mit — die Positionen liegen intern in Map-Metern und werden
erst beim Publizieren in lat/lon umgerechnet.

Map (0, 0) entspricht per Konstruktion exakt dem Datum. Das gilt in beide Richtungen:
**die eigene lokale Koordinate ist immer `x = 0, y = 0`** — Eve kann sich im Frame, den sie
selbst aufspannt, nicht woanders befinden. Deshalb steht sie im `datum`-Block von
`/octopus/trash_gps` und im Dashboard (Inspector, Grid) ausdrücklich als `0, 0`, statt gegen
irgendeinen anderen Bezugspunkt gerechnet zu werden. Nur `lat`/`lon` des Datums bewegen sich.

Das Dashboard verankert seinen lokalen Frame ebenfalls auf Eve (`eveDatum()` in
`live_data.js`) — nicht auf der Ecke einer gezeichneten Suchfläche. Von der Suchfläche kommt
nur noch ihre **Größe**; ihre Lage folgt Eve. Andernfalls würde dasselbe „x = 3,6 m" im
Dashboard und in ROS auf zwei verschiedene Punkte am Boden zeigen.

Solange das Dashboard noch nichts gemeldet hat, publiziert `eve_fake_gps_bridge_node` einen
Fallback (`DEMO_MAP_ORIGIN` aus `live_data.js`, Garching) und loggt eine Warnung. Im JSON von
`/octopus/trash_gps` zeigt `datum.from_topic`, welcher Fall gerade gilt.

## `/octopus/trash_gps` — alle Ziele

```json
{
  "source_id": "trash_gps_goal_node",
  "frame_id": "map",
  "timestamp": 1786977824.861,
  "datum": { "lat": 46.694667, "lon": 11.840481, "x": 0.0, "y": 0.0, "from_topic": true },
  "goal_id": 1,
  "open_count": 2,
  "targets": [
    {
      "id": 1,
      "class_name": "trash",
      "lat": 46.6946675,
      "lon": 11.8405280,
      "x": 3.59,
      "y": 0.03,
      "confidence": 0.8,
      "collected": false,
      "is_goal": true,
      "last_seen": 1786977824.126
    }
  ]
}
```

| Feld | Bedeutung |
|---|---|
| `id` | stabil über die Laufzeit des Nodes; Bezugsgröße für `trash_goal_done` |
| `lat` / `lon` | WGS84, aus `x`/`y` relativ zum Datum berechnet |
| `x` / `y` | Position im `map`-Frame in Metern (x = Ost, y = Nord) |
| `datum.x` / `datum.y` | die eigene lokale Koordinate, konstant `0.0` / `0.0` |
| `confidence` | YOLO-Konfidenz der Detektion, kann `null` sein |
| `collected` | vom Roboter über `trash_goal_done` gemeldet |
| `is_goal` | dieses Ziel liegt gerade auf `/octopus/trash_goal` |
| `goal_id` | `id` des aktuellen Ziels, `null` wenn nichts offen ist |

Ids sind **nicht** über einen Neustart des Nodes hinweg stabil. Nach einem Neustart wird bei
1 begonnen und alle `collected`-Markierungen sind weg.

## `/octopus/trash_goal` — nächstes Ziel

Ein `NavSatFix` mit dem Ziel, das der Roboter als nächstes anfahren soll. Auswahl über den
Parameter `goal_selection`:

- `nearest` (Default) — das dem Datum nächstgelegene offene Ziel, also das dem Roboterstart
  nächstgelegene
- `first` — das zuerst entdeckte offene Ziel

Ist nichts offen, wird nichts mehr publiziert; die letzte Nachricht bleibt im Latch stehen.
Wer das unterscheiden muss, prüft `open_count` in `/octopus/trash_gps`.

### Felder von NavSatFix

```yaml
header: { stamp: <jetzt>, frame_id: "map" }
status: { status: 0, service: 1 }          # 0 = STATUS_FIX, 1 = SERVICE_GPS
latitude: 46.6946675
longitude: 11.8405280
altitude: 0.0
position_covariance: [0.25, 0, 0,  0, 0.25, 0,  0, 0, 1.0]
position_covariance_type: 1                # APPROXIMATED
```

`status: 0` ist **kein** Fehler, sondern `STATUS_FIX`. Kein Fix wäre `-1`.

`position_covariance` ist eine 3×3-Matrix, zeilenweise, in m², Achsen ENU. Die Diagonale ist
die Varianz σ² je Achse: `0.25 m²` = σ 0,5 m horizontal, `1.0 m²` = σ 1 m vertikal.

Diese Werte sind **geschätzt, nicht gemessen** — daher `position_covariance_type: 1`
(`APPROXIMATED`). Es gibt keinen Empfänger, der eine echte Fehlerschätzung liefern könnte.
`UNKNOWN` (0) wäre formal ehrlicher, führt aber bei `robot_localization` und ähnlichen
Consumern dazu, dass die Nachricht verworfen oder mit einem sehr großen Default ersetzt wird.

Ist der Roboter beim Anfahren zu träge, ist das die Stellschraube: kleinere Werte = der EKF
vertraut der Koordinate mehr. Die Homographie-Projektion liegt indoor realistisch bei
5–20 cm, `0.25` ist also konservativ. Aktuell hartcodiert in beiden Nodes.

## `/octopus/trash_goal_done` — Rückkanal

Der Roboter meldet die erledigte `id`. Ohne diese Meldung bleibt das Ziel für immer auf
demselben Stück Müll stehen.

```bash
ros2 topic pub --once /octopus/trash_goal_done std_msgs/msg/String "{data: '1'}"
```

Akzeptiert wird die nackte Id (`"1"`) oder JSON (`{"id": 1}`). Das Ziel wird auf
`collected: true` gesetzt und `/octopus/trash_goal` rückt sofort auf das nächste vor.

Unbekannte Ids werden mit einer Warnung ignoriert.

## `/octopus/devices/<id>/status` — Rückkanal für den Zustand

Der Roboter beschreibt sich selbst; Octopus fragt nicht nach. Ein JSON-String, 2 Hz, `<id>` ist
die Roboterkennung (`gripperx`). `device_status_backend_bridge_node` leitet das ans Backend
weiter, das Dashboard zeigt den Roboter damit auf der Mission Map.

```json
{
  "source_id": "gripperx_demo",
  "robot_id": "gripperx",
  "timestamp": 1787234926.3,
  "pose": {"status": "ok", "frame_id": "map", "x": 1.2, "y": 0.4,
           "yaw_deg": 35.0, "lat": 48.2513, "lon": 11.6359},
  "nav": {"status": "idle", "active_goal_id": null, "distance_remaining_m": null},
  "armed": false,
  "battery": {"status": "unavailable", "reason": "NO_SENSOR_INSTALLED",
              "percent": null, "voltage_v": null},
  "link": {"connected": true, "last_rx_age_sec": 0.7}
}
```

| Feld | Bedeutung fürs Dashboard |
|---|---|
| `robot_id` | welcher Roboter das ist; der Topicname ist nur der Fallback |
| `pose.lat` / `pose.lon` | der Marker auf der Mission Map |
| `pose.status` | ist er nicht `ok`, sagt das Panel **warum** kein Marker da ist |
| `nav.status` | das Zustands-Pill (`idle`, `navigating`, …) |
| `nav.active_goal_id`, `nav.distance_remaining_m` | „goal #3 · 2.3 m to go" |
| `armed` | wird nur angezeigt; nichts auf Octopus-Seite reagiert darauf |
| `battery.status` / `battery.reason` | `unavailable` wird zu `n/a (no sensor installed)`, **nicht** zu 0 % |

Zwei Festlegungen, die bewusst so sind:

- **`pose.lat`/`lon` `null` setzt den Roboter nicht aufs Datum.** Wer `no_datum` meldet, bekommt
  gar keinen Marker, und das Panel schreibt „no datum yet, not on the map". Ein Fallback auf eine
  konfigurierte Position hätte einen live aussehenden Marker auf einen erfundenen Punkt gemalt —
  und zwar genau auf Eves Startpunkt, weil eine fehlende Koordinate dorthin umrechnet.
- **Ein stiller Roboter wird `stale`, nicht unsichtbar.** Nach ~6 s ohne Status steht `link lost`
  und die Karte wird amber. Verschwinden würde aussehen wie „nie konfiguriert".

Zusätzliche Felder werden unverändert durchgetragen; das Dashboard ignoriert, was es nicht kennt.

Weitere Roboter brauchen keinen Code, nur einen Parameter:

```bash
ros2 run octopus_backend_bridge device_status_backend_bridge_node --ros-args \
  -p status_topics:="['/octopus/devices/gripperx/status','/octopus/devices/robby/status']"
```

## Umrechnung lat/lon ↔ map

Bewusst dieselbe Flat-Earth-Näherung wie `localToLatLng()` im Dashboard, damit ein Ziel auf
der Mission Map und das an den Roboter geschickte Ziel dieselbe Koordinate sind:

```text
lat = datum_lat + y / 111320
lon = datum_lon + x / (111320 · cos(datum_lat))
```

Über die paar Dutzend Meter der Demofläche liegt der Fehler weit unter der Genauigkeit des
Detektors. Für Außeneinsätze über größere Flächen muss das durch eine richtige Projektion
ersetzt werden.

Der Roboter kann entweder diese zwei Zeilen invertieren oder `navsat_transform_node/fromLL`
benutzen, um zu einer `map`-Pose für `NavigateToPose` zu kommen.

## Warum NavSatFix und nicht GeoPoseStamped

`geographic_msgs` und `nav2_msgs` sind auf dem Demo-Laptop **nicht installiert** (geprüft mit
`ros2 pkg prefix`). `sensor_msgs/NavSatFix` ist Teil jeder ROS-2-Installation und der
Standard-Input von `robot_localization`/`navsat_transform_node`.

Wer `nav2_gps_waypoint_follower` mit `geographic_msgs/GeoPoseStamped` benutzen will:
`sudo apt install ros-humble-geographic-msgs`, dann sind es etwa zehn Zeilen zusätzlich in
`trash_gps_goal_node`.

Ebenso ist `std_msgs/String` mit JSON kein sauberer Typ für strukturierte Daten — ein
Consumer muss parsen und hat keine Schema-Garantie. Es ist so gebaut, weil der gesamte
Octopus-Stack es so macht (`detections_world`, `map_patch`, `local_camera_grid_patch`).
Für eine produktive Anbindung wären eigene `.msg`-Definitionen der bessere Weg.

## Parameter

`trash_gps_goal_node` (Paket `octopus_camera_transform`):

| Parameter | Default | Bedeutung |
|---|---|---|
| `input_topic` | `/octopus/detections_world` | Quelle der Detektionen |
| `datum_topic` | `/octopus/fake_eve_gps_start` | Quelle des Datums |
| `datum_lat` / `datum_lon` | Garching | nur Bootstrap, bis das Topic etwas liefert |
| `goal_selection` | `nearest` | `nearest` oder `first` |
| `merge_radius_m` | `0.25` | zwei Detektionen darunter gelten als dasselbe Objekt |
| `min_confidence` | `0.0` | Detektionen darunter werden ignoriert |
| `publish_period_sec` | `1.0` | Publish-Rate |

`eve_fake_gps_bridge_node` (Paket `octopus_backend_bridge`):

| Parameter | Default | Bedeutung |
|---|---|---|
| `backend_url` | `http://127.0.0.1:8000/api/eve/fake_gps` | Quelle der Eve-Position |
| `output_topic` | `/octopus/fake_eve_gps_start` | |
| `poll_period_sec` | `1.0` | |
| `fallback_lat` / `fallback_lon` | Garching | bis das Dashboard etwas meldet |

Beide Nodes starten automatisch mit `scripts/start_octopus_debug_stack.sh`.

## Testen ohne Roboter

```bash
source /opt/ros/humble/setup.bash
source ~/projects/PlastiX/Octopus/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0

# Datum
ros2 topic echo --once /octopus/fake_eve_gps_start

# aktuelles Ziel
ros2 topic echo --once /octopus/trash_goal

# alle Ziele
ros2 topic echo --once --full-length /octopus/trash_gps

# Ziel 1 als erledigt melden, danach muss trash_goal weiterrücken
ros2 topic pub --once /octopus/trash_goal_done std_msgs/msg/String "{data: '1'}"
```

Prüfen, dass das Datum wirklich durchschlägt — Eve verschieben, ohne das Dashboard zu
benutzen:

```bash
curl -X POST http://127.0.0.1:8000/api/eve/fake_gps \
  -H "Content-Type: application/json" \
  -d '{"lat":48.25258,"lon":11.63073,"manual":true,"source":"test"}'
```

Danach müssen `/octopus/fake_eve_gps_start` **und** die Ziele in `/octopus/trash_gps` auf den
neuen Punkt gesprungen sein.

## Bekannte Einschränkungen

- **Kein Task-Management.** Wer zwei Sammelroboter anschließt, bekommt beiden dasselbe Ziel.
  Eine Zuweisung gibt es nicht.
- **Ids überleben keinen Node-Neustart**, `collected`-Markierungen ebenso wenig.
- **Ziele werden nie vergessen.** Müll gilt als statisch; ein Ziel verschwindet nur durch
  `trash_goal_done`.
- **Der Yaw-Slider** aus dem Dashboard wird als `yaw_deg` mitgeschickt, aber von niemandem
  ausgewertet. Falls der Roboter auch Eves Ausrichtung übernehmen soll, liegt der Wert bereit.
- **Die Einsammel-Schleife ist über rosbridge geprüft**, aber mit einem Simulator
  (`scripts/simulate_gripperx.py`) und synthetischen Detektionen auf einer eigenen
  `ROS_DOMAIN_ID`, nicht mit dem echten Roboter: fahren → `trash_goal_done` → Ziel als
  `collected` → `trash_goal` rückt weiter.
- **Nicht gegen echtes Nav2 getestet.** Auf dem Demo-Laptop läuft kein Nav2 und kein zweiter
  Roboter. Verifiziert ist alles bis einschließlich der publizierten Topics und des
  Weiterrückens nach `trash_goal_done`.
