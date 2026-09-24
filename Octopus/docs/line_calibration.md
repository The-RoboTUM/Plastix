# Gemeinsame Referenzlinie mit GripperX (`octopus_line`)

> **Pfade.** Alle Pfade relativ zur Repo-Wurzel.

Octopus-Seite der Linienkalibrierung. Gegenstück und Quelle: `OCTOPUS_LINE_CALIBRATION.md`
von der GripperX-Seite, **Draft vom 2026-09-24**, dort implementiert in
`gripperx_external`, gegen das echte Octopus noch nicht getestet.

Beide Seiten vermessen dieselben zwei Pfosten des Alurahmens, an dem Eve hängt, und drücken
jede ausgetauschte Koordinate im Frame dieser Pfosten aus. Damit liegen die Meter beider
Systeme erstmals am selben Ort — bisher war der Roboter-Frame dort verankert, wo der Roboter
zufällig gestartet wurde.

```
                      +y  (links von A→B)
                       ^
                       |
      Pfosten A o------+------o Pfosten B      ---> +x  (von A nach B)
            x = -L/2  Ursprung  x = +L/2
                     (Mittelpunkt)

   von OBEN gesehen. z zeigt nach oben, aus der Seite heraus.
```

L ist der Pfostenabstand, **von GripperX' LiDAR gemessen**. Octopus misst L nicht, sondern
bekommt ihn genannt und rechnet damit Pixel in Meter um. L ist damit die einzige
Maßstabseingabe dieser Seite.

---

## Was auf unserer Seite existiert

| Teil | Ort |
|---|---|
| Geometrie (§3), ohne ROS | `Octopus/ros2_ws/src/octopus_camera_transform/octopus_camera_transform/line_frame.py` |
| ROS-Node | `.../octopus_camera_transform/line_calibration_node.py` |
| Status-Topic | `/octopus/line_calibration/status` (`std_msgs/String`, JSON, 1 Hz) |
| Eingabe Operator → ROS | `POST/GET /api/line_calibration` |
| Status ROS → Dashboard | `POST/GET /api/line_calibration/status` |
| Tests | `Octopus/ros2_ws/src/octopus_camera_transform/test/test_line_frame.py` |
| Markier-UI | Panel „Line Calibration" in `dashboard.html`, Logik in `live_data.js` (Ansicht *System / Debug*) |

Das Topic liegt unter `/octopus/*`, die rosbridge-Glob deckt es also bereits ab — GripperX kann
es lesen, ohne dass etwas freigeschaltet werden muss.

Der Node startet mit `start_octopus_debug_stack.sh` mit.

## Was bewusst NICHT gebaut ist

Umgesetzt sind **§1–§5** des Drafts, also Frame, Pixel-Meter-Transformation und Maßstab. Ab §6
ist das Dokument noch in Bewegung; diese Teile fehlen absichtlich:

- **Die Kalibrierung speist die Projektion nicht.** `flight_camera_transform_node` benutzt
  unverändert `manual_height_above_ground_m` und den PX4-Startup-Yaw. Der Node macht die
  Linie nur *sichtbar*, damit sich die Zahlen vergleichen lassen, bevor etwas umgehängt wird.
- **Das Datum bleibt der Eve-Marker.** §4 macht den Linienmittelpunkt zum Datum. Das ist eine
  Vertragsänderung, die GripperX, das Dashboard und `trash_gps_goal_node` gleichzeitig trifft
  — nichts, was man umlegt, solange das Dokument „Draft" sagt und die Demo auf dem jetzigen
  Verhalten läuft. Siehe [`octopus_to_robot_interface.md`](octopus_to_robot_interface.md#das-datum).
- **Kein Assistent für die Prozedur** (§7). Die Markier-UI gibt es (siehe unten), sie führt
  aber nicht durch die Reihenfolge, prüft keine Toleranzen und kennt die Nullbewegungs-Probe
  nicht — genau die Teile, die sich noch ändern.
- **Kein Fail-Closed-Verhalten** (§8). Laufende Ziele werden bei Rekalibrierung nicht
  abgebrochen. Das gehört in `trash_gps_goal_node` und setzt voraus, dass §8 steht.
- **Keine `TO-VERIFY`-Werte geraten.** Die Plausibilitätsgrenze 2,5–3,0 m ist als Parameter
  `min_length_m`/`max_length_m` abgebildet, nicht als Konstante.

## Benutzung heute

Marken setzen (Pixel im Kamerabild, L in Metern):

```bash
curl -X POST http://127.0.0.1:8000/api/line_calibration \
  -H "Content-Type: application/json" \
  -d '{"pixel_a":[120,240],"pixel_b":[520,240],"length_m":2.80}'
```

Ergebnis ansehen:

```bash
curl -s http://127.0.0.1:8000/api/line_calibration/status | python3 -m json.tool
```

Löschen: `-d '{"clear":true}'`. Alternativ ohne Backend über Parameter
`pixel_a`, `pixel_b`, `length_m`, `mirrored` am Node.

Bequemer über das Dashboard, Ansicht **System / Debug**, Panel **Line Calibration**:
*Mark A* drücken, den **Fuß** von Pfosten A im Kamerabild anklicken, dasselbe für B, L
eintragen, *Apply*. Die gesetzten Marken werden als Punkte mit Verbindungslinie über das
Bild gelegt, *Clear* verwirft sie. Escape bricht eine scharfgeschaltete Markierung ab.

Die Marken werden in **Vollbild-Sensorpixeln** gespeichert, nicht in Anzeigekoordinaten —
dort leben `fx`/`cx` und `line_frame.py`. Liefert Eve nur den Ausschnitt, rechnet die UI
über `effectiveCameraCrop()` zurück; ein nachträglich geänderter Crop verschiebt die
Marken dadurch nicht.

Eine fehlerhafte Eingabe wird abgewiesen und lässt eine bestehende gültige Kalibrierung
stehen — sie wird nicht stillschweigend gelöscht.

## Wofür das Ganze sich sofort lohnt: der Maßstab

Der Node rechnet aus dem gemessenen Maßstab die **implizite Kamerahöhe** zurück
(`h = m/px · fx`) und stellt sie der konfigurierten Höhe gegenüber:

```
"metres_per_pixel": 0.007,  "implied_camera_height_m": 2.515,
"configured_height_m": 2.5, "scale_deviation_percent": 0.6
```

Das ist der Punkt: heute ist die Höhe eine gesetzte Konstante, und die gesamte Karte skaliert
linear mit ihr. Das Wiki sagt „~3 Meter", die Konfiguration sagt 2,5 — eine Abweichung von 20 %,
die bisher niemand messen konnte. Ein positiver `scale_deviation_percent` heißt: die Linie sagt,
die Kamera hängt höher als die Projektion annimmt, die Pipeline meldet also zu kurze Distanzen.

Eine Schätzung, kein Ersatz: die Beziehung gilt für eine nadir blickende Kamera nahe dem
Bildmittelpunkt. Als Gegenprobe zur konfigurierten Höhe taugt sie, als Kalibrierwert noch nicht.

## Offene Punkte

Zusätzlich zu den `TO-VERIFY`-Punkten in §9 des Drafts:

**1. Welcher Punkt am Pfosten im Bild angeklickt wird, ist nicht dasselbe wie beim LiDAR — und
der Fehler ist groß.** Das LiDAR sieht den Pfosten in seiner Scanebene, also praktisch am Boden.
Die Kamera blickt von oben: ein Pfosten, der nicht genau unter der Drohne steht, erscheint als
Linie, die vom Bildmittelpunkt nach außen zeigt. Ein Punkt in Höhe `z` am Pfosten fällt auf
denselben Pixel wie ein Bodenpunkt im Radius `r · H/(H−z)`. Bei Kamera 2,5 m und Pfostenfuß
1,4 m neben dem Nadir:

| Klick bei z | scheinbarer Radius | Fehler in L (px) | Fehler in m/px und Höhe |
|---|---|---|---|
| 0,00 m (Fuß) | 1,40 m | ±0 % | ±0 % |
| 0,10 m | 1,46 m | +4,2 % | −4,0 % |
| 0,25 m | 1,56 m | +11,1 % | −10,0 % |
| 0,50 m | 1,75 m | +25,0 % | −20,0 % |

Da Eve *an* dem Rahmen hängt, reichen die Pfosten von unten bis über die Kamera — der optisch
naheliegende Klick auf den gut sichtbaren Teil des Pfostens ist also der falsche. **Angeklickt
werden muss der Pfostenfuß, dort wo der Pfosten den Boden trifft**, nicht die Mitte des
sichtbaren Pfostens. §2 des Drafts sagt „Mitte des Querschnitts", was für das LiDAR eindeutig
ist, für das Bild aber die Höhe offenlässt. Das sollte dort präzisiert werden.

**2. Rekalibrierung auf unserer Seite ist für GripperX nicht beobachtbar** — offener Punkt in
§8 des Drafts, auf unserer Seite ebenfalls offen. Das Status-Topic wäre der natürliche Ort:
es liegt bereits unter `/octopus/*` und trägt einen Zeitstempel.

**3. Ob das Kamerabild gespiegelt ist**, entscheidet über das Vorzeichen von `+y` (§3, §9).
Der Parameter `mirrored` existiert, der richtige Wert ist ungeprüft. Die Nullbewegungs-Probe
aus §7 Schritt 7 ist der Test dafür.

**4. Die Ähnlichkeitstransformation aus §3 modelliert weder Perspektive noch Verzeichnung.**
`flight_camera_transform_node` tut beides (die Intrinsics tragen `k1`, `k2`, `p1`, `p2`).
Wenn die Linie eines Tages die Projektion speist, sollte sie deren Unbekannte kalibrieren —
Höhe, Yaw, Ursprung — und nicht die Projektion ersetzen.
