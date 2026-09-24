# Gemeinsame Referenzlinie mit GripperX (`octopus_line`)

> **Pfade.** Alle Pfade relativ zur Repo-Wurzel.

Octopus-Seite der Linienkalibrierung. Gegenstück und Quelle: `OCTOPUS_LINE_CALIBRATION.md`
von der GripperX-Seite, Fassung vom 2026-09-24 (dort weiterhin als *Draft* gekennzeichnet),
implementiert in `gripperx_external`, gegen das echte Octopus noch nicht getestet.

**Umgesetzt sind §1–§8.** Was nicht geht, steht unter „Offene Punkte" — das sind durchweg
`TO-VERIFY`-Werte der Gegenseite oder Dinge, die sich nur am echten Aufbau klären lassen.

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
| Markier-UI | Unterer Teil des Panels **Camera Debug**, Ansicht *System / Debug*; Logik in `live_data.js` |

Das Topic liegt unter `/octopus/*`, die rosbridge-Glob deckt es also bereits ab — GripperX kann
es lesen, ohne dass etwas freigeschaltet werden muss.

Der Node startet mit `start_octopus_debug_stack.sh` mit.

## §4: Ziele in Linien-Metern, Datum = Linienmittelpunkt

Sobald die Kalibrierung vollständig ist (beide Marken **und** L), sind `x`/`y` in
`/octopus/trash_gps` und die daraus gerechneten `lat`/`lon` **Linien-Frame-Meter**, und das
Datum bezeichnet den Linienmittelpunkt statt der Drohne. Die Flat-Earth-Arithmetik bleibt
unverändert — was sich ändert, ist die Bedeutung des Bezugspunkts.

Die Map-Koordinaten laufen als `map_x`/`map_y` weiter mit, und `line_frame` im selben JSON
sagt, welcher Frame gerade gilt (`octopus_line` oder `map_legacy`). Ohne vollständige
Kalibrierung bleibt alles wie vorher — eine unkalibrierte Demo verschiebt sich nicht
stillschweigend.

**Hier wird der Maßstabsfehler tatsächlich herausgerechnet.** `metres_per_map_unit` ist
GripperX' L geteilt durch den projizierten Pfostenabstand. Im Messlauf oben: L = 2,800 m
gegen 2,848 projizierte Map-Einheiten, also Faktor **0,9833** — unsere Karte meldet Distanzen
rund 1,7 % zu groß, und genau um diesen Faktor korrigiert die Ausgabe.

Zwei Folgefehler, die der Frame-Wechsel eingeführt hätte und die mitbehoben sind:
`max_radius_m` und die Zielauswahl `nearest` messen ab **Datum**, nicht mehr ab Map (0,0) —
sonst läge der Reichweitenkreis um die Drohne statt um den Roboterstart.

## §5/§6: Abgleich mit GripperX' Telemetrie

GripperX schickt `line_calibration` `{status, reason, id, length_m}` auf
`/octopus/devices/gripperx/status`. `line_calibration_node` liest das über
`/api/devices/status` und vergleicht es mit dem eingetippten L: `length_delta_m`,
`length_match` (Toleranz `length_tolerance_m`, Default 1 cm — sie melden Millimeter) und ihre
`id`. Bei Abweichung eine Warnung im Log, einmal pro Änderung, nicht im Sekundentakt.

Die manuelle Eingabe bleibt: sie ist die Gegenprobe, nicht eine Formalität.

## §8: unsere Rekalibrierung erkennbar machen

§8 führt „Octopus-seitige Rekalibrierung ist für GripperX nicht beobachtbar" als offenen
Punkt für beide Teams. Unsere Hälfte ist erledigt: `/octopus/line_calibration/status` trägt
eine `calibration_id`, die sich bei jeder Änderung an Marken, L oder `mirrored` ändert. Das
Topic liegt unter `/octopus/*` und damit bereits in ihrer rosbridge-Glob — es braucht keinen
neuen Kanal, nur einen Subscriber. Dass sie so etwas schon tun, zeigt ihr `octopus_transform`-
Block, der unsere Yaw-Relocks mitzählt.

## Was bewusst NICHT gebaut ist

- **Die Kalibrierung speist die *Projektion* nicht.** `flight_camera_transform_node` rechnet
  weiter mit `manual_height_above_ground_m` und dem PX4-Startup-Yaw. Der Maßstabsfehler wird
  am Ausgang herausgerechnet (§4 oben), nicht in der Projektion selbst. Das ist bewusst: die
  Projektion ist der kritische Pfad der Demo, und das Ergebnis ist dasselbe.
- **Kein Assistent für die Prozedur** (§7). Die Markier-UI gibt es, sie führt aber nicht durch
  die Reihenfolge und kennt die Nullbewegungs-Probe nicht — deren Toleranz ist `TO-VERIFY`.
- **Kein Abbruch laufender Ziele bei Rekalibrierung** (§8). Auf unserer Seite gibt es kein
  „goal in flight", das man abbrechen könnte; der Vertrag kennt nur `trash_goal_done`.
- **Keine `TO-VERIFY`-Werte geraten.** Die Plausibilitätsgrenze ist Parameter
  (`min_length_m`/`max_length_m`), nicht Konstante — siehe unten.

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

Bequemer über das Dashboard, Ansicht **System / Debug** (Auswahlfeld oben in der Topbar),
Panel **Camera Debug**, unterer Abschnitt *Line calibration* — direkt unter dem Kamerabild,
damit Scharfschalten und Klicken im selben Blickfeld liegen:
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

## Geofence: die Demofläche

GripperX spannt seine Demofläche als **Quadrat symmetrisch um die Linie** auf. Dasselbe gibt
es jetzt auf unserer Seite, damit Müll außerhalb gar nicht erst als Ziel angeboten wird — ein
Ziel, das der Roboter bei der Validierung ablehnt, blockiert sonst die Warteschlange.

```
        +-------------------+     Seite = Pfostenabstand
        |         |         |     Mitte = Linienmittelpunkt
        A---------+---------B     Linie = Spiegelachse
        |         |         |
        +-------------------+
```

| Teil | Ort |
|---|---|
| Berechnung und Topic | `flight_camera_transform_node` → `/octopus/line_geofence` (JSON, 1 Hz) |
| Filter | `trash_gps_goal_node`, Parameter `use_line_geofence` (Default an) |
| Sichtbar im Vertrag | Block `geofence` in `/octopus/trash_gps` |

Das deckt sich mit §5a der Endfassung: Quadrat der Seite L, zentriert auf dem
Linienmittelpunkt, an der Linie ausgerichtet.

**Der Geofence braucht L nicht.** Die beiden Marken allein legen Mitte, Richtung und
Seitenlänge fest; L liefert nur den metrischen Maßstab. Solange GripperX den Wert nicht
gemeldet hat, steht die Kalibrierung auf `awaiting_length`, der Geofence ist aber schon aktiv.

**§5a begrenzt die Standposition des Roboters, nicht das Objekt.** Ein Objekt knapp außerhalb
kann bedient werden, wenn es dafür eine Standposition innerhalb gibt, und eines knapp innerhalb
kann abgelehnt werden, wenn es keine gibt. Unser Geofence ist deshalb kein exakter Prädiktor
der Annahme, sondern die von §5a empfohlene Gegenmaßnahme („keep detections inside the square,
or bound the detector to it") — weil ein abgelehntes Ziel mangels Fehlerkanal die Mission
blockiert.

**Warum das Quadrat in Map-Metern und nicht aus L gebaut wird.** Die beiden markierten
Pfosten laufen durch **dieselbe Projektion wie die Detektionen** (`detection_to_world_ned`,
nach der Pixel-Umrechnung mit `pixel_to_normalized`). Ist der Maßstab der Projektion falsch —
und die konfigurierte Höhe legt nahe, dass er es sein kann —, dann sind Pfosten und
Detektionen um denselben Faktor falsch, und der Drinnen-Draußen-Test bleibt trotzdem richtig.
Stattdessen GripperX' metrisches L einzusetzen würde zwei Bezugssysteme mischen und genau den
Fehler zurückholen, dessentwegen es die Linie gibt.

Mit `line_geofence_side_m > 0` lässt sich eine absolute Seitenlänge in Map-Metern erzwingen;
die unterliegt dann wieder dem Maßstabsfehler. Default `0.0` = Seite gleich projiziertem
Pfostenabstand.

**Unterschied zu GripperX: wir schließen offen, sie schließen zu.** §8 des Drafts lässt
GripperX ohne Kalibrierung *jedes* Ziel ablehnen. Bei uns heißt „kein Geofence" **nicht
filtern**, nicht „alles ablehnen" — sonst hätte das Einschalten dieses Nodes die bestehende
Demo, die ohne Marken läuft, stillschweigend lahmgelegt. Das ist eine bewusste Asymmetrie und
gehört mit der GripperX-Seite abgestimmt.

**Der Filter greift bei der Aufnahme, nicht rückwirkend.** Bereits registrierte Ziele bleiben
zunächst stehen; mit dem gesetzten `target_ttl_sec=2.0` verschwinden sie binnen zwei Sekunden,
weil sie nicht mehr bestätigt werden. Verworfene Detektionen werden gezählt
(`geofence.dropped_outside`) und höchstens alle 10 s mit Abstand längs und quer zur Linie
geloggt — eine leere Zielliste muss sich von einem kaputten Detektor unterscheiden lassen.

**Achtung, zweiter Filter:** `max_radius_m` steht im Startskript auf **1,25 m** um das Datum,
das ist GripperX' Reichweitenradius. Der ist kleiner als das Quadrat und damit derzeit die
bindende Grenze. Beide dürfen gleichzeitig aktiv sein — Reichweite und Fläche sind
verschiedene Dinge —, aber wer sich über eine kleine nutzbare Fläche wundert, schaut zuerst
dorthin.

## Plausibilitätsgrenze für L

**2,20–2,80 m** (Stand 2026-09-24). Ein angeklicktes Paar, dessen L außerhalb liegt, wird
abgelehnt — das fängt einen falschen Pfosten oder einen verrutschten Klick, mehr nicht. Es ist
keine Messung.

GripperX wurde am selben Tag auf denselben Bereich umgestellt, beide Seiten weisen also
dieselben Paare zurück. **Das Spezifikationsdokument nennt weiterhin 2,5–3,0 m und ist damit
der veraltete Stand** — beim nächsten Abgleich dort nachziehen.

Einstellbar über `min_length_m`/`max_length_m` am `line_calibration_node`. Die Zahl ist
inzwischen einmal gewandert; sie muss auf beiden Seiten gleich bleiben, sonst akzeptiert eine
Seite eine Kalibrierung, die die andere ablehnt.

## Offene Punkte

Zusätzlich zu den `TO-VERIFY`-Punkten in §9 des Drafts:

**1. §2 regelt links/rechts, aber nicht oben/unten.** Die Endfassung entscheidet, dass beide
Seiten die *sichtbare Kante* ohne Korrektur markieren, und beziffert den Restfehler mit der
Querschnittsdiagonale — vernachlässigbar gegen 0,5 m Sigma. Das betrifft die horizontale
Mehrdeutigkeit. Die vertikale bleibt offen und ist größer:
Das LiDAR sieht den Pfosten in seiner Scanebene, also praktisch am Boden.
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
werden muss der Pfostenfuß, dort wo der Pfosten den Boden trifft.** Anders als der
Kante-gegen-Mitte-Fehler ist das kein Rauschen, sondern ein systematischer Maßstabsfehler, und
bei 0,5 m Klickhöhe mit 25 % deutlich größer als die 0,5 m Sigma, mit denen §2 argumentiert.
Die UI sagt es an der Stelle, an der geklickt wird; im Dokument fehlt es.

**1a. `implied_camera_height_m` und `metres_per_map_unit` müssen nicht übereinstimmen.**
Ersteres kommt aus dem einfachen Lochkameramodell in `line_frame.py`, letzteres aus der echten
Projektion mit Verzeichnung und Neigung. Im Messlauf oben: 2,515 m gegen einen Korrekturfaktor
von 0,9833, was in die andere Richtung zeigt. Verlassen sollte man sich auf
`metres_per_map_unit` — das ist der Faktor, der die Ausgabe tatsächlich korrigiert; die
implizite Höhe ist eine grobe Gegenprobe.

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
