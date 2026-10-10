# Bachelorarbeit – Quellcode des robotischen Greifsystems

**Entwicklung und Evaluierung eines robotischen Greifsystems mit KI-gestützter Bildverarbeitung und Bewegungsplanung zur Wellenentnahme**
Kaan Kirac · Fachhochschule Südwestfalen, Fachbereich Maschinenbau, Iserlohn · Erstprüfer Prof. Dr. Martin Skambraks · Zweitprüfer Prof. Dr. Tobias Ellermeyer · Bearbeitungszeitraum 01.08.–03.10.2026

Dieses Repository enthält den **gesamten selbst geschriebenen Quellcode** des Versuchssystems. Es ist der digitale Anhang zur Bachelorarbeit; die Abschnittsnummern unten beziehen sich auf die Arbeit (Kapitel 5 = Implementierung, Kapitel 6 = Evaluation, Kapitel 7 = Diskussion).

> **Hinweis zum Stand.** Der Quellcode im Ordner `quellcode/` wurde zuletzt am **28.09.2026** geändert und ist seitdem unverändert (die Arbeit verweist auf das Repository mit „Stand: 01.10.2026“). Nach der Abgabe wurden **ausschließlich Begleitdokumente** ergänzt bzw. korrigiert (README, `DATEIEN.md`, Installationshinweise) und **fehlende Binärdateien nachgereicht** (STL-Meshes, aufgezeichnete Kameraszenen, die Protokolldateien der Bedienoberfläche, zwei leere Paket-Markerdateien, die trainierten Modellgewichte), die beim ursprünglichen Hochladen nicht übertragen worden waren. Außerhalb von `quellcode/` kamen eine **Offline-Demo** für Rechner ohne ROS 2 (`offline_demo/`, auch Windows) und die am Versuchssystem verwendeten Anpassungen der Upstream-Pakete als Patch-Dateien (`upstream_patches/`) hinzu. Am Quellcode selbst wurde nichts geändert. Die Änderungen sind in Abschnitt 9 aufgeführt und in der Versionsgeschichte des Repositorys nachvollziehbar.

## 1. Inhalt des Repositorys

| Datei / Ordner | Inhalt |
|---|---|
| `quellcode/` | 1:1-Kopie aller eigenen Quell-, Konfigurations- und Datendateien des ROS-2-Workspace `~/ros2_ws` (201 Text-Dateien mit rund 43 400 Zeilen sowie die STL-Meshes der Szene) – Ordnerstruktur wie auf dem Roboter |
| `DATEIEN.md` | **Verzeichnis aller Dateien mit je einer Kurzbeschreibung** (aus den Docstrings erzeugt) – der schnellste Einstieg |
| `ALLE_CODES.txt` | Alle Dateien in **einer** Textdatei mit Inhaltsverzeichnis (zum Suchen, Lesen, Drucken) |
| `quellcode/testdaten/` | aufgezeichnete RGB-D-Szenen der Bereitstellungsschale vom 13.09.2026 (u. a. 11 bzw. 12 Wellen, Abschnitt 5.8.2) als NumPy-Archive; damit lässt sich die Wellen- und Kopfseitenerkennung ohne Kamera nachvollziehen |
| `quellcode/zyklus_protokoll.txt`, `quellcode/zyklus_log.jsonl` | Protokoll der Ablaufsteuerung `tools/zyklus_gui.py` (Text bzw. ein JSON-Eintrag je Ereignis). Enthält **nur die über die Bedienoberfläche gestarteten Läufe** (14., 23. und 24.09.2026). Die Versuchsreihen der Tabelle 6-2 (je Teilschritt etwa 50 Versuche) wurden vom Verfasser handschriftlich im Laborbuch protokolliert und sind nicht aus diesen Dateien abgeleitet |
| `offline_demo/` | **Offline-Demo ohne Roboter, Kamera und ROS 2** (Windows, Linux, macOS): führt die Wellen-/Kopfseitenerkennung und die YOLO11l-seg-Segmentierung der Arbeit auf den aufgezeichneten Szenen aus – siehe Abschnitt 6.5 und `offline_demo/README.md` (nach der Abgabe ergänzt) |
| `upstream_patches/` | die am Versuchssystem vorgenommenen Anpassungen der Upstream-Pakete `mycobot_ros2`, `trac_ik`, `realsense-ros` und `easy_handeye2` als Patch-Dateien (Abschnitt 6.2) |
| `requirements.txt` | Python-Pakete des Versuchssystems |
| `quellcode/src/wellenerkennung/weights/welle_gross.pt` | trainierte YOLO11l-seg-Gewichte (Abschnitt 5.3.2; 54 MB, Training 30.03.2026 mit Ultralytics 8.4.31, 150 Epochen, Eingabegröße 640, Batchgröße 1) – an der Stelle, an der der Detektorknoten sie erwartet. Dieselbe Datei hängt zusätzlich am Release `v1.0-abgabe` |

**Zur Datei- und Zeilenzahl:** Die Arbeit nennt in der Einleitung zu Kapitel 5 „154 Dateien, rund 32 900 Zeilen“. Diese Zählung bezieht sich auf den Stand 14.09.2026 und umfasst nur Python-, C++-, Xacro-/URDF-, YAML-/SRDF- und Shell-Dateien. Die Zahl oben (201 Dateien) bezieht sich auf den Endstand 28.09.2026 und schließt zusätzlich JSON-Datendateien (Teach-Punkte, Messwerte, Farbreferenzen), Markdown-Notizen und RViz-Konfigurationen ein.

**Nicht enthalten** (Fremdcode oder Binärdaten): die übernommenen Upstream-Pakete `mycobot_ros2` (Elephant Robotics), `realsense2_camera`/`realsense2_description`, `easy_handeye2` und `trac_ik` (Bezugsquellen in Abschnitt 6; die projektspezifischen Anpassungen daran liegen als Patches in `upstream_patches/`), die frühere Modellvariante `v2_best.pt` (20 MB), der Bilddatensatz (287 Aufnahmen; separates Repository `-robot-vida-projesi_large` bzw. Datenträger) sowie `build/`, `install/`, `log/`.

## 2. Was das System tut

Sechsachsiger myCobot 280 JN (Jetson Nano, ROS 2 Galactic) mit stationärer RGB-D-Kamera Intel RealSense D435if (Eye-to-Hand). Ein Handhabungszyklus (`tools/zyklus_gui.py`, Abschnitt 5.10 der Arbeit):

1. Kamerapose prüfen (`tools/kamera_pruefung.py`, `tools/measure_camera_pose.py`)
2. Zielwelle in der Bereitstellungsschale bestimmen: im Gesamtzyklus mit **klassischer Bildverarbeitung ohne KI** (`tools/welle_finden_schale.py`: Differenz zum Referenzbild der leeren Schale, HSV-Farbklassen, Morphologie, Wasserscheide, Stabmodell der Welle), Bewertung 0–100 je Kandidat (Wandabstand, Nachbarabstand, Form, Kopfseite, Farbsicherheit, Lage). Der YOLO11-seg-Detektor (`src/wellenerkennung`) wurde in der ersten Pick-Kette (`run_pick_preview.sh`, Juli 2026) eingesetzt; im Gesamtzyklus wurde er wegen der Anlaufzeit (erste Inferenz 102 s bis über 4 min), des Speicherbedarfs und der Wärmeentwicklung auf dem Jetson Nano nicht verwendet (siehe Abschnitt 7)
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
| `mycobot_world` | URDF/Xacro der Gesamtszene: Roboter, Greifer, Kamera, Grundplatte, Schale, Trichter, Ablageplatte; Kamerapose; Reichweitenmarker | 5.6.1, 5.6.6 |
| `mycobot_moveit_config` | MoveIt-2-Konfiguration: SRDF, TRAC-IK (`kinematics.yaml`), OMPL/RRTConnect (`ompl_planning.yaml`), Controller, Startskript `demo.launch.py` | 5.6.2–5.6.3 |
| `mycobot_hardware` | ros2_control-Hardware-Interface + Python-Brücke zur seriellen Steuerung, NOT-AUS-Relais | 5.1, 5.10 |
| `wellenerkennung` | YOLO11-seg-Detektor als ROS-2-Knoten: Instanzmasken, PCA-Orientierung, robuste Tiefe (Median/MAD), Rückprojektion, Zielpose in `robot_base` | 5.3, 5.4 |
| `mycobot_calibration` | ChArUco-Detektor (solvePnP), Hand-Eye-Kalibrierung (calibrateHandEye: Park/Horaud/Daniilidis), Kalibrier-Launchfiles, Jog- und Klick-Werkzeuge, NOT-AUS-Taster | 5.5 |
| `mycobot_demo` | frühe C++-Demos (MoveGroupInterface): Anfahrt, Pick & Place, Klickpunkt | 5.11 |

## 4. Die wichtigsten Programme (`quellcode/tools/` und Wurzel)

| Datei | Zweck | Arbeit |
|---|---|---|
| `start_mycobot.sh` / `stop_mycobot.sh` | gesamten Stack starten (LC_ALL=C, CycloneDDS nur loopback, Kamera 848×480@15, Fixtures per Umgebungsvariable `SCHALE_MONTIERT`, `TRICHTER_MONTIERT`, `ABLAGEPLATTE_MONTIERT`) bzw. sauber beenden | 5.1 |
| `ros_umgebung.sh` | Terminalumgebung für alle `tools/*.py` | – |
| `tools/zyklus_gui.py` | grafische Ablaufsteuerung: AUTOMATIK/MANUELL, Start ab beliebigem Schritt, Farbwahl durch den Bediener, Nestabfrage, NOT-AUS, Protokoll `zyklus_protokoll.txt` | 5.10 |
| `tools/ablauf.py` | Kette der Einzelschritte ohne Fenster (`--von schale`, `--auto`) | 5.10 |
| `tools/welle_finden_schale.py` | Wellenerkennung in der Schale mit klassischer Bildverarbeitung (ohne KI, im Gesamtzyklus verwendet) (Stabanpassung, Kopfseite aus Breitenprofil und Ringen, Konfidenz 0–100, IK-Probe je Kandidat) | 5.3.3, 5.8.2 |
| `tools/welle_farbe.py`, `tools/nest_fuer_farbe.py` | Körper- und Streifenfarbe (HSV-Referenzen `farbklassen.json`) → Zielnest (`farbplan.json`) | 5.9 |
| `tools/hole_aus_schale.py` | Griff aus der Schale: IK-Haltung, Pre-Grasp, gerade Absenkung, Greifer, Anheben | 5.6.4 |
| `tools/kontrollpose.py`, `tools/kopfseite_pruefen.py`, `tools/kamera_pruefung.py` | Kontrollpose unter der Kamera und Kopfseitenkontrolle | 5.8.2 |
| `tools/hole_welle.py` | Wiederaufnahme der Welle aus dem Trichter am Kopf | 5.8.3 |
| `tools/lege_welle.py` | Ablage im Nest (eine Absenkbahn mit Vorhalt, Nachstellen über dem Nest, Freigabegrenze 4 mm) und Übergabe an den Trichter | 5.9.2, 6.8 |
| `tools/nachstellen.py`, `tools/gelenkspiel.py`, `tools/j4_von_oben.py` | Kompensation von Totband und richtungsabhängigem Getriebespiel (Achsen 2 und 4 von oben, Vorhalt) | 6.7 |
| `tools/bahnpruefung.py` | Stetigkeitsprüfung jeder kartesischen Bahn (Pflicht seit dem Unfall vom 13.09.2026) | 5.6.5 |
| `tools/nullstellung_moveit.py`, `go_to_zero.sh`, `tools/hebe_und_parke.py` | kollisionsgeprüfte Fahrt in die reale Nullstellung | 5.10 |
| `tools/nest_anfahren.py`, `tools/versetze_soll.py`, `tools/teach_*.py`, `tools/servo_nullen.py` | Anlernen und Nachjustieren der Nester, Teach-Punkte, Servo-Nullpunkt | 5.9.2 |
| `tools/measure_camera_pose.py`, `tools/pruefe_kalibrierung_platte.py`, `tools/kamerapose.py` | plattenbasierte Bestimmung und Prüfung der Kamerapose (RANSAC-Ebene, Rechteckkanten) | 5.5.2, 5.5.3 |
| `tools/calibrate_handeye_auto.py`, `src/mycobot_calibration/scripts/calibrate_handeye.py` | Hand-Eye-Kalibrierung mit ChArUco (16 Posen) | 5.5.1 |
| `tools/kamera_roboter_abbildung.py` | Abbildung Kamera → Modell im Arbeitsbereich messen und fitten | 5.5 |
| `tools/schale_finden*.py`, `tools/schale_erreichbarkeit.py`, `tools/arbeitsraum_grenze.py` | Lage der Schale aus Tiefen-/Farbbild, Erreichbarkeit, nutzbarer Arbeitsraum | 5.6.6 |
| `tools/gelenkspiel_messreihe.py`, `tools/measure_*.py`, `measure_zero.py` | Messreihen: Gelenkspiel, Modellgenauigkeit, Kamerahöhe, Nachgiebigkeit | 6.7 |
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

## 6. Installation und Ausführung

Das System wurde auf dem im myCobot 280 JN integrierten Jetson Nano entwickelt. Ohne Roboter und Kamera lassen sich **Robotermodell, Versuchsaufbau, Bewegungsplanung und Trajektorienausführung in der Simulation** (ros2_control-Fake-Hardware) nachvollziehen; die Bildverarbeitung lässt sich mit den aufgezeichneten Szenen **ohne ROS 2 auf jedem Rechner mit Python** ausführen (Offline-Demo, Abschnitt 6.5 – der einfachste Einstieg, auch unter Windows).

### 6.1 Voraussetzungen

| Komponente | Im Versuch verwendeter Stand (Tabelle 5-1 der Arbeit) | Bezugsquelle |
|---|---|---|
| Betriebssystem | Ubuntu 20.04 LTS (ROS 2 Galactic ist nur für 20.04 verfügbar) | – |
| ROS 2 | Galactic (`ros-galactic-desktop`, `ros-galactic-moveit` 2.3.4, `ros-galactic-ros2-control`, `ros-galactic-ros2-controllers`, `ros-galactic-xacro`) | apt (packages.ros.org) |
| `mycobot_ros2` (liefert `mycobot_description`) | Commit `3999e2c` + `upstream_patches/mycobot_ros2.patch` (Kollisionskörper, Greiferdrehung am Flansch, Tippfehler der Gelenkgrenze). Hinweis: Dieser Commit liegt im Upstream-Repository auf dem Zweig `humble`; er wurde unter ROS 2 Galactic gebaut und verwendet (Tabelle 5-1 der Arbeit nennt „galactic, Commit 3999e2c, projektspezifisch angepasst“) | <https://github.com/elephantrobotics/mycobot_ros2> |
| `trac_ik` | Zweig `rolling`, Commit `56136bb` (Release 2.2.0) + `upstream_patches/trac_ik_galactic.patch` (Portierung auf Galactic: ohne `generate_parameter_library`, Header `.h` statt `.hpp`) | <https://bitbucket.org/traclabs/trac_ik> |
| `realsense2_camera` | 4.51.1 (nur am realen Roboter nötig) | <https://github.com/IntelRealSense/realsense-ros> (Tag `4.51.1`) |
| `realsense2_description` | 4.51.1 + `upstream_patches/realsense-ros.patch` (reicht den Parameter `use_mesh` an das D435i-Makro durch) | <https://github.com/IntelRealSense/realsense-ros> (Tag `4.51.1`) |
| `easy_handeye2` (nur für die Kalibrier-Launchfiles) | Commit `b42cae6` + `upstream_patches/easy_handeye2.patch` (Typangaben für Python 3.8) | <https://github.com/marcoesposito1988/easy_handeye2> |
| Python-Pakete | siehe `requirements.txt` (u. a. `ultralytics` 8.4.33, `opencv` 4.8.0, `pymycobot` 4.0.4; im Versuch die Vorabversion 4.0.4b5) | pip |

> **Hinweis:** ROS 2 Galactic ist seit Dezember 2022 ohne Herstellerunterstützung (End of Life). Die Pakete sind weiterhin über das ROS-Archiv bzw. ein Docker-Image `ros:galactic` installierbar. Für einen Nachbau ohne Ubuntu 20.04 ist ein solches Docker-Image der einfachste Weg.

### 6.2 Workspace bauen

Die Skripte erwarten den Workspace unter `~/ros2_ws` (Ordnerstruktur wie auf dem Roboter):

```bash
git clone https://github.com/kakir001/bachelorarbeit-ros2-code.git ~/bachelorarbeit-ros2-code
mkdir -p ~/ros2_ws && cp -r ~/bachelorarbeit-ros2-code/quellcode/. ~/ros2_ws/
cd ~/ros2_ws
chmod +x *.sh tools/*.py src/*/scripts/*.py      # Ausführungsrechte (im Repository nicht gespeichert)

cd ~/ros2_ws/src
git clone https://github.com/elephantrobotics/mycobot_ros2.git   && git -C mycobot_ros2 checkout 3999e2c    # ca. 1,4 GB
git clone -b rolling https://bitbucket.org/traclabs/trac_ik.git  && git -C trac_ik checkout 56136bb
git clone -b 4.51.1 --depth 1 https://github.com/IntelRealSense/realsense-ros.git
git clone https://github.com/marcoesposito1988/easy_handeye2.git && git -C easy_handeye2 checkout b42cae6

# projektspezifische Anpassungen der Upstream-Pakete einspielen
P=~/bachelorarbeit-ros2-code/upstream_patches
git -C mycobot_ros2  apply $P/mycobot_ros2.patch
git -C trac_ik       apply $P/trac_ik_galactic.patch
git -C realsense-ros apply $P/realsense-ros.patch
git -C easy_handeye2 apply $P/easy_handeye2.patch

cd ~/ros2_ws
source /opt/ros/galactic/setup.bash
colcon build --symlink-install --packages-up-to mycobot_moveit_config mycobot_hardware \
    trac_ik_kinematics_plugin wellenerkennung mycobot_demo mycobot_calibration
```

`--packages-up-to` baut nur die 13 benötigten Pakete; die übrigen Robotermodelle aus `mycobot_ros2` und der Kameratreiber `realsense2_camera` werden für die Simulation nicht gebraucht (auf dem Jetson sind sie mit `COLCON_IGNORE` ausgeschlossen bzw. der Kameratreiber ist über apt installiert). Die Python-Pakete aus `requirements.txt` (`pip3 install -r ~/bachelorarbeit-ros2-code/requirements.txt`) werden für die Bildverarbeitung und den realen Roboter benötigt, nicht für die Simulation.

**Geprüft (05.10.2026, Jetson Nano, Ubuntu 20.04, ROS 2 Galactic):** Mit genau diesen Schritten wurde ein leerer Workspace aus einer Kopie von `quellcode/` und frisch aus dem Internet geklonten Upstream-Paketen aufgebaut (dort unter `/tmp/ws_test` statt `~/ros2_ws`); nach dem Einspielen der Patches sind die Upstream-Pakete identisch mit dem Stand des Versuchssystems, und alle 13 Pakete bauen fehlerfrei. Dabei aufgefallene Lücken des ursprünglichen Uploads sind behoben bzw. oben berücksichtigt: Ohne den Ordner `meshes/` und ohne die leeren Markerdateien `resource/<paket>` der beiden Python-Pakete bricht `colcon build` ab; ohne die Ausführungsrechte (`chmod +x`) findet das Startskript `arbeitsraum_marker.py` nicht.

Die Modellgewichte `welle_gross.pt` liegen bereits in `src/wellenerkennung/weights/` und werden beim Bauen des Pakets `wellenerkennung` mit installiert; für die Simulation werden sie nicht gebraucht.

### 6.3 Simulation ohne Roboter (MoveIt 2 + RViz 2, Fake-Hardware)

```bash
cd ~/ros2_ws && source install/setup.bash
export LC_ALL=C LC_NUMERIC=C LANG=C          # Qt/RViz: Dezimalpunkt statt Komma
USE_FAKE_HARDWARE=true SCHALE_MONTIERT=1 TRICHTER_MONTIERT=1 ABLAGEPLATTE_MONTIERT=1 \
  ros2 launch mycobot_moveit_config demo.launch.py use_camera:=false
```

RViz 2 zeigt den modellierten Versuchsaufbau (Abbildung 5-6 der Arbeit); über das MotionPlanning-Panel lassen sich Ziele planen und mit der Fake-Hardware ausführen. Mit `use_rviz:=false` startet das System ohne Fenster.

**Geprüft (05.10.2026)** im oben beschriebenen Test-Workspace mit `use_rviz:=false` (der Jetson Nano hat für RViz neben dem Build zu wenig Arbeitsspeicher): `joint_state_broadcaster`, `arm_controller` und `gripper_controller` sind `active`, `move_group` meldet „You can start planning now!“, als Kinematik-Plugin ist `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin` geladen, und eine Anfrage an `/compute_ik` liefert eine gültige Lösung. Das RViz-Fenster selbst wurde in diesem Test nicht geöffnet. `start_mycobot.sh sim` ist für den Jetson gedacht (startet zusätzlich Kamera und Kamera-Posenprüfung) und auf einem Rechner ohne Kamera nicht zu empfehlen.

**Bekannte Einschränkung beim Nachbau:** Einige Mess- und Kalibrierwerkzeuge in `tools/` (`calibrate_handeye_auto.py`, `measure_*.py`, `test_position_retry.py`, `zeichne_greifer.py`) sowie `go_to_zero.sh` enthalten den absoluten Pfad `/home/er/ros2_ws` des Jetson. Auf einem anderen Rechner diese Werkzeuge entweder unter einem Benutzer `er` ausführen oder einen symbolischen Link anlegen: `sudo mkdir -p /home/er && sudo ln -s ~/ros2_ws /home/er/ros2_ws`. Die ROS-2-Pakete unter `src/` und die Ablaufsteuerung (`tools/zyklus_gui.py`) sind davon nicht betroffen.

### 6.4 Betrieb am realen Roboter (Jetson Nano)

```bash
# Terminal 1 – Stack (Roboter, MoveIt 2, RViz 2, Kamera, Brücke, NOT-AUS)
SCHALE_MONTIERT=1 TRICHTER_MONTIERT=1 ABLAGEPLATTE_MONTIERT=1 KAMERA_PROFIL=848 ./start_mycobot.sh
# Terminal 2 – Umgebung + Ablaufsteuerung
source ros_umgebung.sh
python3 tools/zyklus_gui.py            # MANUELL: jeder Schritt wird bestätigt
```
Sicherheitsregeln des Projekts (Abschnitt 7.4): Bewegungen nur über MoveIt 2 (kollisionsgeprüft, Stetigkeitsprüfung), niedrige Geschwindigkeit, jede neue Bewegung zuerst in RViz 2 ansehen; der Herstellerbefehl `send_coords` wird nicht verwendet. Ein Stack-Neustart in der Nähe des Trichters ist zu vermeiden (automatische Nullstellung ist deaktiviert).

### 6.5 Offline-Demo ohne ROS 2 (Windows, Linux, macOS)

Für Rechner ohne ROS 2 enthält `offline_demo/` zwei Skripte, die den unveränderten Code der Arbeit auf den aufgezeichneten Szenen aus `quellcode/testdaten/` ausführen: die Wellen- und Kopfseitenerkennung mit klassischer Bildverarbeitung (`tools/welle_finden_schale.py`, Abschnitte 5.3.3 und 5.8.2) und die Instanzsegmentierung mit dem trainierten YOLO11l-seg-Modell (`wellenerkennung/erkennung.py`, Abschnitte 5.3.2 und 5.4). Benötigt wird nur Python 3.9 bis 3.12.

* **Windows:** Repository als ZIP herunterladen und entpacken, dann `offline_demo\START_WINDOWS.bat` doppelklicken.
* **Linux/macOS:** `cd offline_demo && ./start.sh`

Die Startdatei legt eine virtuelle Umgebung an, installiert die Pakete aus `offline_demo/requirements-offline.txt`, verwendet die Modellgewichte aus `quellcode/src/wellenerkennung/weights/` und legt die Ergebnisbilder in `offline_demo/ausgabe/` ab. Einzelheiten, erwartete Ergebnisse und deren Einordnung stehen in `offline_demo/README.md`.

Geprüft am 05.10.2026 auf dem Jetson Nano in einer Umgebung ohne ROS 2; die erzeugten Mosaike sind bildpunktgleich mit den am 19.09.2026 am Versuchssystem erzeugten (`quellcode/testdaten/mosaik/`). **Auf einem realen Windows- oder macOS-Rechner nicht getestet.**

### 6.6 Vollständiges ROS-2-System unter Windows

ROS 2 Galactic steht für Windows nicht als fertige Installation zur Verfügung. Das vollständige System (Abschnitte 6.2 und 6.3) lässt sich unter Windows nur in einer Linux-Umgebung ausführen, z. B. WSL 2 mit Ubuntu 20.04 oder Docker mit dem Image `ros:galactic`. **Beides wurde nicht getestet.** Für die Begutachtung unter Windows ist die Offline-Demo (Abschnitt 6.5) vorgesehen.

## 7. Hinweise zum Lesen des Codes

* Kommentare und Docstrings sind überwiegend deutsch; frühe Dateien (Mai/Juni 2026) sind englisch kommentiert, einige Werkzeuge verwenden ASCII-Umlaute (ae/oe/ue), damit sie in jedem Terminal lesbar bleiben.
* Jede Datei beginnt mit einem Docstring, der Zweck, Aufruf und – wo relevant – das Datum und den Anlass der Änderung nennt (z. B. `tools/bahnpruefung.py`: Unfall vom 13.09.2026).
* Die in der Arbeit abgedruckten Quelltexte 5-1 bis 5-11 und 6-1 sind gekürzte Auszüge aus `mycobot_bridge.py`, `erkennung.py`, `wellen_detektor_node.py`, `charuco_detector.py`, `calibrate_handeye.py`, `measure_camera_pose.py`, `kinematics.yaml`, `hole_aus_schale.py`, `bahnpruefung.py` und `nachstellen.py`.
* Abweichung Arbeit ↔ Code: In Abschnitt 7.10 und im Kommentar von Quelltext 6-1 ist eine Setzzeit von 3 s genannt. Im Ablauf übernimmt `tools/lege_welle.py` das Nachstellen mit bis zu 4 Runden (wie in der Arbeit) und einer Setzzeit von **4 s**; das eigenständige Werkzeug `tools/nachstellen.py` verwendet ohne Aufrufparameter 3 Runden und 5 s.
* Abweichung Arbeit ↔ Code, Wellenerkennung im Gesamtzyklus: Die Arbeit (Abschnitte 4.2, 5.1, 5.3–5.4, Abbildung 5-1) beschreibt die Zielbestimmung über YOLO11-seg mit Tiefe aus der Instanzmaske (Median/MAD) und Körperfarbe als Modellklasse. Im Gesamtzyklus (`tools/zyklus_gui.py`, `tools/ablauf.py`) arbeitet dagegen `tools/welle_finden_schale.py` **ohne KI**: Die Höhe wird aus dem bekannten Schalenboden bestimmt (Sehstrahl ∩ Bodenebene), nicht aus dem Tiefenbild; Körper- und Streifenfarbe kommen aus HSV-Klassen (`tools/welle_farbe.py`). Die YOLO-Kette (`src/wellenerkennung`) ist vollständig im Repository enthalten und mit der Offline-Demo nachvollziehbar.
* Abweichung Arbeit ↔ Code, Kamera → Modell: `tools/hole_aus_schale.py` überträgt das Kameraziel zusätzlich mit einer Ähnlichkeitsabbildung aus `kamera_modell_platte.json` (Maßstab ≈ 1,06, Drehung ≈ −1,5°) in den Modellraum, weil das Robotermodell infolge von Gelenkspiel und Durchhang radial zu weit rechnet (Abschnitt 6.7). Diese Abbildung ist in der Arbeit nicht beschrieben.
* Abweichung Arbeit ↔ Code, Geschwindigkeit: Freie MoveIt-Fahrten verwenden den Faktor 0,15 × `--tempo`; der Standardwert `--tempo 2` ergibt 0,30 (Arbeit, Abschnitt 7.10: 0,10 bis 0,15). Die kartesischen Absenk- und Hubbahnen werden über ihre Dauer zeitparametriert.
* Abweichung Arbeit ↔ Code, Bewertung 0–100 (Abschnitt 5.3.3): Im Code erreicht der Wandabstand die vollen 30 Punkte ab 30 mm (Arbeit: 40 mm); der Nachbarabstand ergibt 0 Punkte ab 3 mm und volle 20 Punkte ab Öffnung/2 + 13 mm (Arbeit: Öffnung/2 + 5,5 mm bzw. + 20 mm).
* Abweichung Arbeit ↔ Modelldatei: Die Arbeit nennt eine Precision von 0,96 (Box); in den Trainingsmetriken der Gewichtsdatei `welle_gross.pt` steht 0,944.
* Kopfseitenbestimmung (Abschnitt 5.8.2, „in allen Fällen richtig“): Die Aussage gilt für die als greifbar freigegebenen Kandidaten. Unter den verworfenen Kandidaten der aufgezeichneten Szenen finden sich Maskenbruchstücke und in der Szene mit 12 Wellen ein Kandidat mit umgekehrter Kopfseite; die Offline-Demo zeigt das nachvollziehbar.

## 8. Zugriff

Das Repository ist **öffentlich** lesbar (kein Sperrvermerk). Der vollständige Workspace mit Versionsgeschichte liegt zusätzlich unter `github.com/kakir001/ros2_ws` (privat).

## 9. Änderungen nach der Abgabe (Quellcode unverändert)

| Datum | Änderung |
|---|---|
| 04.10.2026 | README: Abschnittsnummern an die Kapitelzählung der Arbeit angepasst (Implementierung = Kapitel 5, Evaluation = Kapitel 6); Hinweis zum Stand und zur Datei-/Zeilenzahl; Installations- und Simulationsanleitung (Abschnitt 6); Hinweis auf die Setzzeit-Abweichung |
| 04.10.2026 | `DATEIEN.md` aus den Docstrings erzeugt, `requirements.txt` ergänzt |
| 05.10.2026 | Modellgewichte `welle_gross.pt` nachgereicht (im Repository unter `quellcode/src/wellenerkennung/weights/` und als Anhang des Release `v1.0-abgabe`); die Arbeit nennt sie als Bestandteil des Repositorys, beim ursprünglichen Hochladen fehlten sie |
| 05.10.2026 | Die README vom 28.09.2026 nannte zusätzlich ein Entwicklungsprotokoll (`DEVLOG_de.md`, `DEVLOG.md`). Diese Arbeitsnotizen waren nie im Repository enthalten und werden nicht veröffentlicht, da sie personenbezogene Angaben enthalten; der Hinweis wurde entfernt |
| 05.10.2026 | Binärdateien des Workspace nachgereicht, die beim ursprünglichen Hochladen (nur Textdateien) nicht übertragen worden waren: sechs STL-Meshes in `quellcode/src/mycobot_world/meshes/` (ohne diesen Ordner bricht `colcon build` des Pakets `mycobot_world` ab), aufgezeichnete Kameraszenen in `quellcode/testdaten/` sowie die Hintergrundaufnahmen `schale_hintergrund.npz` und `trichter_hintergrund.npz` |
| 05.10.2026 | Protokolldateien der Ablaufsteuerung nachgereicht: `quellcode/zyklus_protokoll.txt` und `quellcode/zyklus_log.jsonl` (von `tools/zyklus_gui.py` geschrieben, letzte Änderung 24.09.2026; nur Läufe über die Bedienoberfläche, nicht die Datengrundlage der Tabelle 6-2) |
| 05.10.2026 | Zwei leere Markerdateien nachgereicht, die beim ursprünglichen Hochladen fehlten (0 Byte, vom ROS-2-Buildsystem verlangt): `quellcode/src/wellenerkennung/resource/wellenerkennung` und `quellcode/src/mycobot_calibration/resource/mycobot_calibration` |
| 05.10.2026 | `upstream_patches/` ergänzt: die am Versuchssystem vorhandenen Anpassungen der Upstream-Pakete als Patch-Dateien (aus dem Workspace des Jetson mit `git diff` erzeugt) |
| 05.10.2026 | `offline_demo/` ergänzt (neue Hilfsskripte außerhalb von `quellcode/`): Ausführung der Bildverarbeitung auf den aufgezeichneten Szenen ohne ROS 2 |
| 05.10.2026 | README Abschnitt 6 anhand eines vollständigen Neuaufbaus auf dem Jetson berichtigt (Upstream-Stände und Patches, `chmod +x`, Paketauswahl für `colcon build`); `requirements.txt` um `scipy` ergänzt; die versehentlich hochgeladene Datei `repo_update.zip` entfernt |
| 10.10.2026 | README und `DATEIEN.md`: Herkunft der Daten in Tabelle 6-2 klargestellt (handschriftliches Laborbuch, die Protokolldateien enthalten nur Läufe über die Bedienoberfläche); Wellenerkennung im Gesamtzyklus ausdrücklich als klassische Bildverarbeitung ohne KI bezeichnet (vorher „modellbasiert“); weitere Abweichungen zwischen Arbeit und Code in Abschnitt 7 ergänzt. Quellcode unverändert |

Alle am 28.09.2026 vorhandenen Dateien in `quellcode/` sowie `ALLE_CODES.txt` sind **unverändert**; in `quellcode/` sind ausschließlich Dateien hinzugekommen. Nachprüfbar mit `git diff --stat 45b43a0 HEAD -- quellcode ALLE_CODES.txt` (zeigt nur neue Dateien, keine geänderten).
