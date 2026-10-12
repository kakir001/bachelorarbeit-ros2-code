# Erweiterung nach der Abgabe: Wahl des Erkennungsverfahrens (klassisch / KI)

Stand 11.10.2026. Dieser Ordner ergänzt den Abgabestand in `../quellcode/`, der **unverändert** bleibt.

## Warum

Die Arbeit beschreibt die Bestimmung der Zielwelle mit YOLO11-seg. Im Gesamtzyklus des Versuchs (`tools/zyklus_gui.py`) lief dagegen die klassische Bildverarbeitung (`tools/welle_finden_schale.py`), weil der YOLO-Detektor auf dem Jetson Nano zu lange anlief und zu viel Speicher brauchte (README Abschnitte 2 und 7). Mit dieser Erweiterung sind **beide** Verfahren im Gesamtzyklus wählbar und unter gleichen Bedingungen vermessen.

**Zeitlicher Ablauf:** Bis zur Abgabe (03.10.2026) war das YOLO11-seg-Modell trainiert und als ROS-2-Detektorknoten im Einsatz, die Einbindung in den Gesamtzyklus aber nicht fertig. Nach der Abgabe und vor dem Kolloquium (11.10.2026) wurde YOLO11-seg in den Gesamtzyklus eingebunden. Seitdem sind beide Verfahren vorhanden und werden je nach Wahl beim Start ausgeführt.

## Was sich ändert

* **Beim Start der Ablaufsteuerung** fragt `tools/zyklus_gui.py` zuerst:
  1. „Wie sollen die Wellen bestimmt werden?“ – **Bildverarbeitung (klassisch)** oder **KI (YOLO11-seg)**
  2. „RViz (3D-Ansicht, braucht viel RAM)?“ – **RViz AN** / **RViz AUS**

  Erst danach sind AUTOMATIK/MANUELL frei. Die Wahl steht dauerhaft im Fenster („Erkennung: …“). `--verfahren klassisch|yolo` und `--rviz ja|nein` überspringen die Fragen.
* **KI gewählt:** Der Erkennungsdienst `tools/erkennung_dienst.py` startet sofort im Hintergrund, lädt das Modell **einmal je Sitzung** und wärmt es auf, während der Arm in die Kamerapose fährt. Ab der ersten Welle gibt es keine Aufwärmzeit mehr. Schritt 1 wartet, bis der Dienst bereit ist (`--ki-wartezeit`, Vorgabe 600 s), und fragt danach „weiter warten / auf Bildverarbeitung wechseln“. NOT-AUS und Fensterschließen beenden den Dienst (SIGTERM, nach 5 s SIGKILL).
* **Mit KI laufen alle Bildauswertungen des Zyklus über YOLO:** Wellensuche in der Schale (Körperfarbe = YOLO-Klasse; Streifenfarbe, Kopfseite und Bewertung 0–100 wie gehabt aus der Geometrie), Kopfkontrolle unter der Kamera (Schritt 4) und Trichterkontrolle (Schritt 6, `tools/kamera_pruefung.py --verfahren yolo`).
* **Gleiche Geometrie für beide Verfahren:** YOLO ersetzt in `welle_in_frame` nur den Segmentierungsblock. Stab, Kopfseite, Ringfarben, Nachbarprüfung, Bewertung und IK-Probe sind identisch, das Ausgabeformat `welle_ziel.json` ebenfalls (zusätzlich `verfahren`, `t_segmentierung_s`, `t_inferenz_s`, `t_gesamt_s`). Jeder Eintrag in `zyklus_log.jsonl` enthält `verfahren`, Schritt 1 außerdem `t_suche_s` und `t_segmentierung_s`.
* `start_mycobot.sh` fragt, ob RViz starten soll (oder `MIT_RVIZ=0/1`).

## Messergebnis (Kurzfassung, Einzelheiten in `ERKENNUNG_BENCHMARK.md`)

| Jetson Nano, Stack läuft, RViz aus, n = 10 je Verfahren, abwechselnd | klassisch | KI (YOLO11-seg) |
|---|---:|---:|
| Bereitschaft je Sitzung | sofort | ≈ 78 s (einmal) |
| Wellensuche je Welle (Median) | 20,5 s | 22,9 s |
| davon Segmentierung (Median) | 0,54 s | 0,94 s |

Beim Messen wurde ein Fehler gefunden, der YOLO auf dem Jetson lahmlegte: `model.predict(device=0)` lieferte **0 Detektionen**, weil die Conv-BN-Fusion auf der Jetson-CPU NaN-Gewichte erzeugt. `tools/yolo_segmentierung.py` fusioniert deshalb auf der GPU. Ein Ausschnitt um die Schale verdoppelt außerdem die Zahl der gefundenen Wellen (10 statt 5 von 11).

## Dateien

| Datei | Inhalt |
|---|---|
| `tools/yolo_segmentierung.py` | **neu.** YOLO-Segmentierung als reine Funktion (ohne ROS), eigener GPU-Weg, Schalen-Ausschnitt, Client des Dienstes |
| `tools/erkennung_dienst.py` | **neu.** Hält das Modell geladen; Unix-Socket `~/ros2_ws/.erkennung.sock`, Zustand in `~/ros2_ws/.erkennung_status.json` (nur Linux) |
| `tools/erkennung_diagnose.py` | **neu.** Zeitzerlegung des YOLO-Starts je Variante (Abschnitt 2 des Berichts) |
| `tools/erkennung_messreihe.py` | **neu.** N Wellensuchen je Verfahren am Aufbau, ohne Roboterbewegung |
| `tools/welle_finden_schale.py` | geändert: `--verfahren klassisch|yolo`, `--ki-direkt` |
| `tools/kamera_pruefung.py` | geändert: `--verfahren yolo` für Kopf- und Trichterkontrolle |
| `tools/zyklus_gui.py` | geändert: Verfahrens- und RViz-Wahl, Dienststeuerung, Zeiten im Protokoll |
| `tools/test_welle_finden.py` | geändert: `--verfahren klassisch|yolo|beide` (Vergleichsmosaike); behebt, dass Kommandozeilenargumente ignoriert wurden |
| `start_mycobot.sh` | geändert: RViz-Frage |
| `ERKENNUNG_BENCHMARK.md` | Messbericht |
| `messdaten/` | Rohdaten der Messungen (JSON) |
| `testdaten/live_schale_2026-10-11.npz` | am 11.10.2026 aufgenommene Szene der vollen Schale (848 × 480) |

Die Unterschiede zum Abgabestand: `git diff --no-index quellcode/tools/zyklus_gui.py erweiterung_erkennung/tools/zyklus_gui.py` (entsprechend für die anderen Dateien).

## Einsetzen auf dem Roboter

```bash
cp erweiterung_erkennung/tools/*.py ~/ros2_ws/tools/
cp erweiterung_erkennung/start_mycobot.sh ~/ros2_ws/
MIT_RVIZ=0 SCHALE_MONTIERT=1 TRICHTER_MONTIERT=1 ABLAGEPLATTE_MONTIERT=1 KAMERA_PROFIL=848 ./start_mycobot.sh
source ros_umgebung.sh && python3 tools/zyklus_gui.py          # zuerst Verfahren + RViz, dann Modus
```

## Ohne Roboter (Windows, Linux, macOS)

`../offline_demo/vergleich_offline.py` führt beide Verfahren auf den aufgezeichneten Szenen aus (Demo 3, siehe `../offline_demo/README.md`). Unter Windows rechnet YOLO im eigenen Prozess (der Dienst nutzt einen Unix-Socket und ist nur für den Jetson gedacht).

## Was nicht geprüft ist

* Ein vollständiger Zyklus mit KI **mit Roboterbewegung** (die Messungen liefen ohne Bewegung; der Arm stand über der Schale).
* Kopfkontrolle mit KI **mit** Welle im Greifer (das Modell ist auf liegende Wellen trainiert).
* Die IK-Probe lehnte in der Messung alle Kandidaten ab (beide Verfahren gleich); die Messreihen liefen daher mit `--ohne-ik-probe`.
