# Bachelorarbeit – Quellcode des robotischen Greifsystems

**Entwicklung und Evaluierung eines robotischen Greifsystems mit KI-gestützter Bildverarbeitung und Bewegungsplanung zur Wellenentnahme**
Kaan Kirac · Fachhochschule Südwestfalen, Fachbereich Maschinenbau, Iserlohn · Erstprüfer Prof. Dr. Martin Skambraks · Zweitprüfer Prof. Dr. Tobias Ellermeyer · Bearbeitungszeitraum 01.08.–03.10.2026

Dieses Repository enthält den **gesamten selbst geschriebenen Quellcode** des Versuchssystems (Stand 28.09.2026) sowie das Entwicklungsprotokoll. Es ist der digitale Anhang zur Bachelorarbeit; die Abschnittsnummern unten beziehen sich auf die Arbeit (Kapitel 4 = Implementierung, Kapitel 5 = Evaluation).

## 1. Inhalt des Repositorys

| Datei / Ordner | Inhalt |
|---|---|
| `quellcode/` | 1:1-Kopie aller eigenen Quell-, Konfigurations- und Datendateien des ROS-2-Workspace `~/ros2_ws` (201 Dateien, 43 400 Zeilen) – Ordnerstruktur wie auf dem Roboter |
| `DATEIEN.md` | **Verzeichnis aller Dateien mit je einer Kurzbeschreibung** (aus den Docstrings erzeugt) – der schnellste Einstieg |
| `ALLE_CODES.txt` | Alle Dateien in **einer** Textdatei mit Inhaltsverzeichnis (zum Suchen, Lesen, Drucken) |
| `DEVLOG_de.md` | Entwicklungsprotokoll (deutsch), jede Sitzung von Mai bis September 2026: Befunde, Messungen, Entscheidungen, Unfälle und ihre Ursachen |
| `DEVLOG.md` | dasselbe Protokoll in der Arbeitssprache des Verfassers (türkisch/deutsch gemischt) |
| `upload/` | Werkzeug, mit dem dieses Repository von der instabilen Internetverbindung des Jetson Nano aus hochgeladen wurde (Datei für Datei über die GitHub-Blob-API) |

**Nicht enthalten** (Fremdcode oder Binärdaten): die unverändert übernommenen Upstream-Pakete `mycobot_ros2` (Elephant Robotics), `realsense2_camera`, `easy_handeye2` und `trac_ik` (für Galactic portiert), die YOLO-Gewichte (`welle_gross.pt`, 54 MB; `v2_best.pt`, 20 MB), der Bilddatensatz (287 Aufnahmen, separates Repository `-robot-vida-projesi_large` bzw. Datenträger) sowie `build/`, `install/`, `log/`.

## 2. Was das System tut

Sechsachsiger myCobot 280 JN (Jetson Nano, ROS 2 Galactic) mit stationärer RGB-D-Kamera Intel RealSense D435if (Eye-to-Hand). Ein Handhabungszyklus (`tools/zyklus_gui.py`, Abschnitt 4.10 der Arbeit):

1. Kamerapose prüfen (`tools/kamera_pruefung.py`, `tools/measure_camera_pose.py`)
2. Zielwelle in der Bereitstellungsschale bestimmen: Instanzsegmentierung mit YOLO11-seg (`src/wellenerkennung`) bzw. modellbasierte Erkennung (`tools/welle_finden_schale.py`), Bewertung 0–100 je Kandidat (Wandabstand, Nachbarabstand, Form, Kopfseite, Farbsicherheit, Lage)
3. Zielnest aus Körper- und Streifenfarbe (`tools/welle_farbe.py`, `tools/nest_fuer_farbe.py`, `farbplan.json`)
4. Welle greifen: IK mit TRAC-IK über `/compute_ik`, freie Anfahrt mit MoveIt 2/OMPL, gerade Absenkung über `/compute_cartesian_path` mit Stetigkeitsprüfung (`tools/hole_aus_schale.py`, `tools/bahnpruefung.py`)
5. Kontrollpose unter der Kamera, Kopfseite prüfen (`tools/kontrollpose.py`, `tools/kopfseite_pruefen.py`)
6. Übergabe an das trichterförmige Übergabeelement (`tools/lege_welle.py --nest trichter_ablager`)
7. Kamerakontrolle: steht die Welle im Trichter?
8. Wiederaufnahme am Kopf (`tools/hole_welle.py`)
9. Vertikale Ablage im Zielnest der Versuchsplatte in einer einzigen Absenkbahn mit Encoder-Vorhalt (`tools/lege_welle.py`, `tools/nachstellen.py`, `tools/gelenkspiel.py`)

Die Roboterhardware ist über `ros2_control` angebunden: das C++-Hardware-Interface (`src/mycobot_hardware/src/mycobot_hardware.cpp`) spricht über einen Unix-Socket mit der Python-Brücke `src/mycobot_hardware/scripts/mycobot_bridge.py`, die als einziger Prozess die serielle Schnittstelle (`pymycobot`) besitzt, den Encoder-Nullpunktversatz kompensiert und den NOT-AUS (`estop_relay.py`, `estop_button.py`) durchsetzt.

## 3. Eigene ROS-2-Pakete (`quellcode/src/`)

| Paket | Aufgabe | Arbeit |
|---|---|---|
| `mycobot_world` | URDF/Xacro der Gesamtszene: Roboter, Greifer, Kamera, Grundplatte, Schale, Trichter, Ablageplatte; Kamerapose; Reichweitenmarker | 4.6.1, 4.6.6 |
| `mycobot_moveit_config` | MoveIt-2-Konfiguration: SRDF, TRAC-IK (`kinematics.yaml`), OMPL/RRTConnect (`ompl_planning.yaml`), Controller, Startskript `demo.launch.py` | 4.6.2–4.6.3 |
| `mycobot_hardware` | ros2_control-Hardware-Interface + Python-Brücke zur seriellen Steuerung, NOT-AUS-Relais | 4.1, 4.10 |
| `wellenerkennung` | YOLO11-seg-Detektor als ROS-2-Knoten: Instanzmasken, PCA-Orientierung, robuste Tiefe (Median/MAD), Rückprojektion, Zielpose in `robot_base` | 4.3, 4.4 |
| `mycobot_calibration` | ChArUco-Detektor (solvePnP), Hand-Eye-Kalibrierung (calibrateHandEye: Park/Horaud/Daniilidis), Kalibrier-Launchfiles, Jog- und Klick-Werkzeuge, NOT-AUS-Taster | 4.5 |
| `mycobot_demo` | frühe C++-Demos (MoveGroupInterface): Anfahrt, Pick & Place, Klickpunkt | 4.11 |

## 4. Die wichtigsten Programme (`quellcode/tools/` und Wurzel)

| Datei | Zweck | Arbeit |
|---|---|---|
| `start_mycobot.sh` / `stop_mycobot.sh` | gesamten Stack starten (LC_ALL=C, CycloneDDS nur loopback, Kamera 848×480@15, Fixtures per Umgebungsvariable `SCHALE_MONTIERT`, `TRICHTER_MONTIERT`, `ABLAGEPLATTE_MONTIERT`) bzw. sauber beenden | 4.1 |
| `ros_umgebung.sh` | Terminalumgebung für alle `tools/*.py` | – |
| `tools/zyklus_gui.py` | grafische Ablaufsteuerung: AUTOMATIK/MANUELL, Start ab beliebigem Schritt, Farbwahl durch den Bediener, Nestabfrage, NOT-AUS, Protokoll `zyklus_protokoll.txt` | 4.10 |
| `tools/ablauf.py` | Kette der Einzelschritte ohne Fenster (`--von schale`, `--auto`) | 4.10 |
| `tools/welle_finden_schale.py` | modellbasierte Wellenerkennung in der Schale (Stabanpassung, Kopfseite aus Breitenprofil und Ringen, Konfidenz 0–100, IK-Probe je Kandidat) | 4.3.3, 4.8.2 |
| `tools/welle_farbe.py`, `tools/nest_fuer_farbe.py` | Körper- und Streifenfarbe (HSV-Referenzen `farbklassen.json`) → Zielnest (`farbplan.json`) | 4.9 |
| `tools/hole_aus_schale.py` | Griff aus der Schale: IK-Haltung, Pre-Grasp, gerade Absenkung, Greifer, Anheben | 4.6.4 |
| `tools/kontrollpose.py`, `tools/kopfseite_pruefen.py`, `tools/kamera_pruefung.py` | Kontrollpose unter der Kamera und Kopfseitenkontrolle | 4.8.2 |
| `tools/hole_welle.py` | Wiederaufnahme der Welle aus dem Trichter am Kopf | 4.8.3 |
| `tools/lege_welle.py` | Ablage im Nest (eine Absenkbahn mit Vorhalt, Nachstellen über dem Nest, Freigabegrenze 4 mm) und Übergabe an den Trichter | 4.9.2, 5.8 |
| `tools/nachstellen.py`, `tools/gelenkspiel.py`, `tools/j4_von_oben.py` | Kompensation von Totband und richtungsabhängigem Getriebespiel (Achsen 2 und 4 von oben, Vorhalt) | 5.7 |
| `tools/bahnpruefung.py` | Stetigkeitsprüfung jeder kartesischen Bahn (Pflicht seit dem Unfall vom 13.09.2026) | 4.6.5 |
| `tools/nullstellung_moveit.py`, `go_to_zero.sh`, `tools/hebe_und_parke.py` | kollisionsgeprüfte Fahrt in die reale Nullstellung | 4.10 |
| `tools/nest_anfahren.py`, `tools/versetze_soll.py`, `tools/teach_*.py`, `tools/servo_nullen.py` | Anlernen und Nachjustieren der Nester, Teach-Punkte, Servo-Nullpunkt | 4.9.2 |
| `tools/measure_camera_pose.py`, `tools/pruefe_kalibrierung_platte.py`, `tools/kamerapose.py` | plattenbasierte Bestimmung und Prüfung der Kamerapose (RANSAC-Ebene, Rechteckkanten) | 4.5.2, 4.5.3 |
| `tools/calibrate_handeye_auto.py`, `src/mycobot_calibration/scripts/calibrate_handeye.py` | Hand-Eye-Kalibrierung mit ChArUco (16 Posen) | 4.5.1 |
| `tools/kamera_roboter_abbildung.py` | Abbildung Kamera → Modell im Arbeitsbereich messen und fitten | 4.5 |
| `tools/schale_finden*.py`, `tools/schale_erreichbarkeit.py`, `tools/arbeitsraum_grenze.py` | Lage der Schale aus Tiefen-/Farbbild, Erreichbarkeit, nutzbarer Arbeitsraum | 4.6.6 |
| `tools/gelenkspiel_messreihe.py`, `tools/measure_*.py`, `measure_zero.py` | Messreihen: Gelenkspiel, Modellgenauigkeit, Kamerahöhe, Nachgiebigkeit | 5.7 |
| `tools/jog_gui.py`, `tools/jog_ohne_drehung.py`, `tools/servo_frei.py` | Handjog, Servo freischalten zum Führen von Hand | – |
| `tools/export_code_single_file.py` | erzeugt `ALLE_CODES.txt` | – |

Weitere Werkzeuge und alle Daten-/Konfigurationsdateien sind in `DATEIEN.md` beschrieben.

## 5. Wichtige Daten- und Konfigurationsdateien

| Datei | Inhalt |
|---|---|
| `teach_punkte.json` | angelernte Gelenkstellungen (Nullstellung, 20 Nester `nest_XY_ablage`, `trichter_greif`, `trichter_ablager`, Kontrollposen) – im Modellraum (Encoder − Versatz) |
| `gelenk_nullpunkte.json` | Encoder-Nullpunktversatz je Gelenk `[0, 2.29, 0.29, −0.29, 0, 0]°`, von der Brücke kompensiert |
| `farbplan.json`, `farbklassen.json` | Zuordnung Farbe → Nest; gelernte HSV-Referenzen |
| `platte_belegung.json` | aktuelle Belegung der Nester |
| `kamera_modell_platte.json`, `src/mycobot_world/urdf/camera_pose.xacro` | plattenbasiert bestimmte Kamerapose |
| `src/mycobot_moveit_config/config/*.yaml`, `mycobot.srdf` | IK, Planer, Controller, erlaubte Kollisionspaare |

## 6. Inbetriebnahme (Kurzfassung)

```bash
# Terminal 1 – Stack (Roboter, MoveIt 2, RViz 2, Kamera, Brücke, NOT-AUS)
SCHALE_MONTIERT=1 TRICHTER_MONTIERT=1 ABLAGEPLATTE_MONTIERT=1 KAMERA_PROFIL=848 ./start_mycobot.sh
# Terminal 2 – Umgebung + Ablaufsteuerung
source ros_umgebung.sh
python3 tools/zyklus_gui.py            # MANUELL: jeder Schritt wird bestätigt
```
Sicherheitsregeln des Projekts (Abschnitt 6.4): Bewegungen nur über MoveIt 2 (kollisionsgeprüft, Stetigkeitsprüfung), niedrige Geschwindigkeit, jede neue Bewegung zuerst in RViz 2 ansehen; der Herstellerbefehl `send_coords` wird nicht verwendet. Ein Stack-Neustart in der Nähe des Trichters ist zu vermeiden (automatische Nullstellung ist deaktiviert).

## 7. Hinweise zum Lesen des Codes

* Kommentare und Docstrings sind überwiegend deutsch; frühe Dateien (Mai/Juni 2026) sind englisch kommentiert, einige Werkzeuge verwenden ASCII-Umlaute (ae/oe/ue), damit sie in jedem Terminal lesbar bleiben.
* Jede Datei beginnt mit einem Docstring, der Zweck, Aufruf und – wo relevant – das Datum und den Anlass der Änderung nennt (z. B. `tools/bahnpruefung.py`: Unfall vom 13.09.2026).
* Die in der Arbeit abgedruckten Quelltexte 4-1 bis 4-11 und 5-1 sind gekürzte Auszüge aus `mycobot_bridge.py`, `erkennung.py`, `wellen_detektor_node.py`, `charuco_detector.py`, `calibrate_handeye.py`, `measure_camera_pose.py`, `kinematics.yaml`, `hole_aus_schale.py`, `bahnpruefung.py` und `nachstellen.py`.
* Das Entwicklungsprotokoll `DEVLOG_de.md` ist chronologisch rückwärts sortiert (neueste Sitzung oben) und enthält zu jeder Messung die Rohwerte.

## 8. Zugriff

Das Repository ist **öffentlich** lesbar (kein Sperrvermerk). Der vollständige Workspace mit Versionsgeschichte liegt zusätzlich unter `github.com/kakir001/ros2_ws` (privat).
