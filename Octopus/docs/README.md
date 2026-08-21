# Octopus Dokumentation

Einstiegspunkt für alle Dokumente in diesem Ordner.

**Willst du die Demo starten? → [`SETUP.md`](SETUP.md).** Das ist das primäre und am
aktuellsten gehaltene Dokument.

Die meisten älteren Dokumente sind auf Englisch, die neueren auf Deutsch. Das ist historisch
gewachsen und nicht vereinheitlicht.

## Betrieb

| Dokument | Inhalt |
|---|---|
| [`SETUP.md`](SETUP.md) | **Start hier.** Indoor-Demo hochfahren, Terminal für Terminal, mit Health-Check und Fehlersuche |
| [`how_to_connect_pi_camera.md`](how_to_connect_pi_camera.md) | Kamera-Node auf der Pi starten, Bild auf dem Laptop ansehen |

## Schnittstellen

Verträge zwischen den Teilsystemen. Das braucht, wer etwas anschließt.

| Dokument | Inhalt |
|---|---|
| [`drone_to_octopus_interface.md`](drone_to_octopus_interface.md) | Eve → Octopus: wie Detektionen in Map-Koordinaten hereinkommen |
| [`octopus_to_robot_interface.md`](octopus_to_robot_interface.md) | Octopus → Sammelroboter: Müllpositionen als GPS-Ziele für Nav2 |
| [`gripperx_rosbridge_link.md`](gripperx_rosbridge_link.md) | Der WebSocket-Transport zu GripperX: starten, prüfen, Fehlerbilder |
| [`detector_posearray_bridge.md`](detector_posearray_bridge.md) | Umwandlung der Detektor-`PoseArray` in das Octopus-JSON-Format |
| [`coordinate_frames.md`](coordinate_frames.md) | Frames und Achsenkonventionen, inklusive der Normalisiert-vs-Welt-Falle bei `PoseArray` |

## Konzepte

| Dokument | Inhalt |
|---|---|
| [`architecture.md`](architecture.md) | Gesamtbild der Mapping-Pipeline und die geplante Baureihenfolge |
| [`map_layers.md`](map_layers.md) | Aufbau des Grid-Maps: Layer, Auflösung, Zellkonvention |

## Testanleitungen

Einzelne Teilstücke isoliert prüfen. Für den normalen Demo-Start reicht `SETUP.md`.

| Dokument | Prüft |
|---|---|
| [`how_to_test_dashboard_bridge.md`](how_to_test_dashboard_bridge.md) | die komplette Kette ROS2 → Backend → Dashboard |
| [`how_to_test_octopus_mapping.md`](how_to_test_octopus_mapping.md) | den Grid-Map-Builder mit künstlichen Detektionen |
| [`how_to_test_detector_posearray_bridge.md`](how_to_test_detector_posearray_bridge.md) | die Detektor-Brücke |
| [`how_to_test_coverage_polygon.md`](how_to_test_coverage_polygon.md) | Coverage-Update für eine abgescannte Fläche |
| [`how_to_test_camera_marker_transform.md`](how_to_test_camera_marker_transform.md) | den AprilTag-Pfad. **Aktuell nicht der laufende Pfad** — die Indoor-Demo benutzt `flight_camera_transform_node` mit Drohnen-Pose statt Marker-Homographie |
| [`milestone_tests.md`](milestone_tests.md) | Abnahmekriterien der drei ersten Meilensteine |

## Historisch

| Dokument | Status |
|---|---|
| [`eve_current_status.md`](eve_current_status.md) | Momentaufnahme aus der Zeit, als die Eve-Schnittstelle noch offen war. Die Frage ist entschieden — siehe `drone_to_octopus_interface.md` |
