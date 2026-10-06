# Offline-Demo – Bildverarbeitung der Arbeit ohne Roboter, Kamera und ROS 2

Diese Demo führt die Bildverarbeitung der Bachelorarbeit auf **aufgezeichneten Kamerabildern** aus. Sie braucht nur Python (3.9 bis 3.12) und läuft unter Windows, Linux und macOS. Der Quellcode in `../quellcode/` wird dabei **unverändert** verwendet; die Skripte in diesem Ordner wurden nach der Abgabe ergänzt und sorgen nur dafür, dass er ohne ROS 2 startet.

## Start

**Windows:** Repository herunterladen (auf GitHub „Code → Download ZIP“, entpacken – am besten in einen kurzen Pfad wie `C:\ba`), dann im Ordner `offline_demo` die Datei **`START_WINDOWS.bat`** doppelklicken.

**Linux / macOS:**

```bash
cd offline_demo
./start.sh
```

Beide Startdateien legen eine virtuelle Python-Umgebung `.venv` an, installieren die Pakete aus `requirements-offline.txt` (beim ersten Mal einige Minuten, etwa 1 GB, überwiegend PyTorch), und starten nacheinander beide Demos. Die Ergebnisbilder liegen danach in `offline_demo/ausgabe/`.

Von Hand:

```bash
python -m venv .venv
.venv\Scripts\activate            # Linux/macOS: source .venv/bin/activate
pip install -r requirements-offline.txt
python wellenerkennung_offline.py
python yolo_offline.py
```

## Was die beiden Demos zeigen

| Skript | Inhalt | Abschnitt der Arbeit | Code der Arbeit, der ausgeführt wird |
|---|---|---|---|
| `wellenerkennung_offline.py` | modellbasierte Wellenerkennung in der Schale: Stabanpassung, Kopfseite aus Breitenprofil und Markierungsringen, Körper- und Streifenfarbe, Bewertung 0–100 je Kandidat | 5.3.3, 5.8.2, 5.9.1 | `quellcode/tools/test_welle_finden.py` → `welle_finden_schale.py` (`welle_in_frame`) |
| `yolo_offline.py` | Instanzsegmentierung mit dem trainierten YOLO11l-seg-Modell, Mittelpunkt und Längsachse je Maske per PCA | 5.3.2, 5.4.1, 5.4.4 | `quellcode/src/wellenerkennung/wellenerkennung/erkennung.py` (`wellen_erkennen`, `get_mask_orientation`, `kopf_bewusster_griff`) |

Eingabe sind die am 13.09.2026 am Versuchsaufbau aufgezeichneten RGB-D-Szenen in `quellcode/testdaten/` (Farbbild 1280 × 720, Tiefenbild, Kameraparameter, Kamerapose, Lage der Schale):

| Szene | Inhalt |
|---|---|
| `welle_weiss_rot_schale_2026-09-13` | 1 Welle |
| `wellen_11_kamerapose_2026-09-13` | 11 Wellen |
| `wellen_11_stueck_schale_2026-09-13` | 11 Wellen, gehäuft |
| `wellen_12_stueck_verteilt_2026-09-13` | 12 Wellen, verteilt |

## Erwartete Ausgabe

### Demo 1 (modellbasiert)

Je Szene entstehen `<szene>_mosaik.png` (jeder Kandidat als Ausschnitt, auf die Achse gedreht, **Kopf links**; rot = Kopfende, blau = Spitze, grün = Griffpunkt am Flansch, magenta = Achse) und `<szene>_uebersicht.png` (ganzes Kamerabild mit denselben Marken). Die Mosaike sind bildpunktgleich mit den am 19.09.2026 auf dem Jetson erzeugten Bildern in `quellcode/testdaten/mosaik/` (geprüft am 05.10.2026).

| Szene | Wellen in der Szene | gefundene Kandidaten | davon mit Bewertung > 0 |
|---|---:|---:|---:|
| `welle_weiss_rot_schale` | 1 | 1 | 1 |
| `wellen_11_kamerapose` | 11 | 11 | 7 |
| `wellen_11_stueck_schale` | 11 | 14 | 6 |
| `wellen_12_stueck_verteilt` | 12 | 13 | 7 |

Zur Einordnung: Die Zahl der Kandidaten ist in den gehäuften Szenen größer als die Zahl der Wellen, weil teilverdeckte Wellen in Bruchstücke zerfallen bzw. eine Welle doppelt erfasst wird. Solche Kandidaten erhalten die Bewertung 0 und werden nicht gegriffen (Meldungen wie „Laenge 25 mm (Soll 43, +-7)“ oder „Nachbar liegt an/auf der Welle“). In den Mosaiken liegt das Kopfende bei allen Kandidaten mit Bewertung > 0 auf der richtigen Seite (Sichtprüfung der Mosaike, 05.10.2026); unter den verworfenen Kandidaten gibt es Bruchstücke und in der Szene mit 12 Wellen einen Kandidaten (Nr. 13) mit vertauschter Kopfseite. Von Hand angeklickte Sollwerte (`*.soll.json`) liegen für diese Szenen nicht vor, die Beurteilung erfolgt daher am Bild.

### Demo 2 (YOLO11l-seg)

Je Szene entsteht `<szene>_yolo.png`: Instanzmasken in der Klassenfarbe, PCA-Längsachse (magenta), Mittelpunkt (grün), Kopfende (rot, falls das Breitenverhältnis eindeutig ist), Klasse und Konfidenz. Mit der Schwelle 0,70 des Detektorknotens wurden am 05.10.2026 (Jetson Nano, CPU, Ultralytics 8.4.33, etwa 10 s je Bild) gefunden:

| Szene | Wellen in der Szene | Detektionen (conf ≥ 0,70) |
|---|---:|---:|
| `welle_weiss_rot_schale` | 1 | 1 |
| `wellen_11_kamerapose` | 11 | 5 |
| `wellen_11_stueck_schale` | 11 | 6 |
| `wellen_12_stueck_verteilt` | 12 | 10 |

Mit `python yolo_offline.py --conf 0.5` lässt sich die Schwelle senken. Die Zahlen können je nach Ultralytics-/PyTorch-Version geringfügig abweichen. Gerechnet wird auf der CPU (`--gpu` für die Grafikkarte).

## Was die Demo nicht zeigt

Roboterbewegung, Bewegungsplanung, Kalibrierung und der Gesamtzyklus brauchen ROS 2 Galactic (Ubuntu 20.04) und für den realen Betrieb den Roboter. Die Simulation mit MoveIt 2 ist in Abschnitt 6 der `README.md` im Hauptordner beschrieben.

## Aufbau

| Datei | Zweck |
|---|---|
| `START_WINDOWS.bat`, `start.sh` | Umgebung anlegen, Pakete installieren, beide Demos starten |
| `requirements-offline.txt` | benötigte Python-Pakete (ohne `pymycobot`, `pyrealsense2`, ROS) |
| `wellenerkennung_offline.py` | Demo 1; lenkt die festen Pfade `~/ros2_ws/...` des Codes auf `../quellcode/` um |
| `yolo_offline.py` | Demo 2; verwendet `../quellcode/src/wellenerkennung/weights/welle_gross.pt` (fehlt die Datei, wird sie aus dem Release `v1.0-abgabe` geladen) |
| `ros_stubs/` | Platzhalter für `rclpy`, `tf2_ros` und die Nachrichtenpakete. Sie machen die Module in `quellcode/tools/` nur **importierbar**; die Bildverarbeitung selbst verwendet kein ROS. Wird eine ROS-Funktion aufgerufen, brechen sie mit einer Meldung ab. |

## Stand der Prüfung

* Geprüft am 05.10.2026 auf dem Jetson Nano (Ubuntu 20.04, aarch64, Python 3.8) in einer Umgebung **ohne ROS** (`import rclpy` schlägt dort fehl): beide Demos laufen durch.
* **Nicht getestet:** ein realer Windows- oder macOS-Rechner. Die Skripte verwenden nur plattformunabhängige Python-Funktionen (`pathlib`, keine festen Pfade, keine Unix-Module); die Dateinamen des Repositorys sind unter Windows zulässig.

## Bekannte Stolpersteine

* **Python nicht gefunden (Windows):** Python von <https://www.python.org> installieren und „Add python.exe to PATH“ ankreuzen.
* **Zu langer Pfad (Windows):** Repository in einen kurzen Pfad entpacken (z. B. `C:\ba`), da PyTorch tiefe Ordner anlegt.
* **Internet:** wird nur für die Installation der Python-Pakete gebraucht; die Modellgewichte liegen im Repository.
