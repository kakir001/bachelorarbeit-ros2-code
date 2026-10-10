# Verzeichnis aller Dateien

Automatisch erzeugt aus den Docstrings bzw. Kopfkommentaren der Dateien in `quellcode/` (201 Dateien, 43409 Zeilen). Die Beschreibung ist jeweils der erste Satz des Docstrings; Einzelheiten stehen in der Datei selbst.


## `(Wurzel des Workspace)`

| Datei | Zeilen | Beschreibung |
|---|---:|---|
| `KALIBRIERUNG_ANALYSE_2026-09-10.md` | 205 | Die Kamerakalibrierung: was schiefging, warum es so lange dauerte, und wie es geloest wurde |
| `REZEPT_welle_aus_trichter.md` | 66 | Welle aus dem Trichter holen — funktionierendes Rezept |
| `arbeitsraum_ring.json` | 147 | JSON-Daten; Schlüssel: z, frame, orientierung, ring, gemessen, totsektor |
| `box_map.yaml` | 12 | Mit teach_boxes.py eingelernt — Mittelpunkte der Sortierboxen je Farbe (Frame robot_base, in m). |
| `calib_check.py` | 203 | NUR-LESE WAHRNEHMUNGSTEST — Genauigkeit der Kamera->robot_base Zuordnung. |
| `calib_check_points.json` | 26 | JSON-Daten; Schlüssel: points |
| `calib_touch_results.md` | 34 | Pruefung der Transformation - Antast-Messungen am Roboter (2026-06-01, Sitzung 14) |
| `cam_viewer.py` | 149 | cam_viewer.py — D435i Kamera (Detector Overlay) in einem einzigen cv2-Fenster. |
| `camera_center.py` | 55 | Öffnet das D435i-Farbbild und markiert das exakte MITTE-Pixel mit Punkt + Kreuz. |
| `check_grasp.py` | 108 | check_grasp.py — diagnostiziere ein fehlgeschlagenes Abstiegsziel. |
| `confirm_key.py` | 84 | confirm_key.py — 'g'-Bestätigung in der RViz-Vorschau-Phase (da cam_viewer geschlossen ist). |
| `cyclonedds.xml` | 11 | CycloneDDS-Konfiguration (Netzwerkschnittstelle) |
| `cyclonedds_loopback.xml` | 25 | CycloneDDS NUR auf loopback (2026-09-11). |
| `farbklassen.json` | 103 | JSON-Daten; Schlüssel: _hinweis, streifen |
| `farbplan.json` | 26 | JSON-Daten; Schlüssel: _hinweis, zeilen, nester |
| `fixture_corner.json` | 35 | JSON-Daten; Schlüssel: corners, date |
| `gelenk_nullpunkte.json` | 63 | JSON-Daten; Schlüssel: _hinweis, joint3_to_joint2, joint4_to_joint3, joint5_to_joint4, _reale_nullstellung_soll_grad, _reale_nullstellung_ist_grad … |
| `go_to_zero.sh` | 36 | Fährt den echten Roboter in die REALE Nullstellung (teach_punkte.json "nullstellung", 20.9.: J2 +3, J3 +1 - Encoder-Nullpunkte verschoben; Rückfall 0 rad), Greifer ZU (die offenen Finger verdecken sonst die Kamerasicht). |
| `handsteuerung.sh` | 156 | handsteuerung.sh — HANDSTEUERUNG: Arm von Hand verfahren, Kamerabild live sehen. |
| `kalibrierposen_charuco.json` | 228 | JSON-Daten; Schlüssel: erzeugt, wozu, gilt_solange, board, gelenke, einheit … |
| `kalibrierposen_charuco_encoder_2026-09-20.json` | 227 | JSON-Daten; Schlüssel: erzeugt, wozu, gilt_solange, board, gelenke, einheit … |
| `kalibrierposen_charuco_vor_J5_2026-09-23.json` | 228 | JSON-Daten; Schlüssel: erzeugt, wozu, gilt_solange, board, gelenke, einheit … |
| `kamera_modell_platte.json` | 179 | JSON-Daten; Schlüssel: aehnlichkeit, affin, skalierung, nester, kamera_mm, modell_mm … |
| `kamera_roboter_abbildung.json` | 349 | JSON-Daten; Schlüssel: A, b_m, rest_mm, loo_mm, versatz_konstant_mm, messungen … |
| `klick_platte.json` | 158 | JSON-Daten; Schlüssel: klick_1, klick_2, klick_3, klick_4, _raster |
| `klick_punkte.json` | 26 | JSON-Daten; Schlüssel: klick_1, klick_2 |
| `measure_zero.py` | 113 | measure_zero.py — Gelenk-Null-Wiederholbarkeit: Kalibrierungsdrift oder Servo- Toleranz? |
| `messung_modell_2026-09-10.json` | 562 | JSON-Daten; Schlüssel: X, messungen, rest_mm, rest_grad |
| `messung_modell_2026-09-10b.json` | 364 | JSON-Daten; Schlüssel: X, messungen, rest_mm, rest_grad |
| `moveit_working.rviz` | 155 | RViz-2-Konfiguration |
| `platte_belegung.json` | 46 | JSON-Daten; Schlüssel: _hinweis, _zurueckgesetzt, nest_34, nest_21, nest_53, nest_32 … |
| `preview_reach.sh` | 119 | preview_reach.sh — NUR BILD-Vorschau (RUEHRT DEN ROBOTER NICHT AN, KEIN IK). |
| `reach_rings.py` | 104 | reach_rings.py — Reichweite-Grenzen des Roboters in RViz als Ring-Marker zeigen. |
| `record_cam_publisher.py` | 103 | record_cam_publisher.py — die USB-2.0-Kamera neben der Plattform (/dev/video3) auf ein ROS-Image-Topic veröffentlichen. |
| `ros_umgebung.sh` | 10 | Umgebung fuer ein Terminal, in dem tools/*.py gegen den laufenden Stack laufen sollen: source ~/ros2_ws/ros_umgebung.sh LC_ALL=C: de_DE-Locale zerlegt Gleitkommazahlen (RViz/MoveIt). |
| `run_approach.sh` | 304 | run_approach.sh — ANNAEHERUNG an Ziel (KEIN RViz — nur Kamerafenster + Terminal). |
| `run_approach_test.sh` | 190 | run_approach_test.sh — Wellen-ANNAEHERUNGSTEST (nur pre-grasp, KEIN Abstieg). |
| `run_pick_preview.sh` | 517 | run_pick_preview.sh — ZWEIPHASIGER pick: VISION → (detector aus) → RViz VORSCHAU. |
| `run_pick_test.sh` | 245 | run_pick_test.sh — Wellen-pick&place-Test, KOMPLETT über Terminal. |
| `run_pick_tilt.sh` | 326 | run_pick_tilt.sh — TILT-BEWUSSTER PICK (KEIN RViz — Kamerafenster + Terminal). |
| `run_place_calib.sh` | 242 | run_place_calib.sh — PLACE (Welle senkrecht einsetzen) XY-Offset-KALIBRIERUNG. |
| `rviz_preview.rviz` | 166 | RViz-2-Konfiguration |
| `start_mycobot.sh` | 172 | myCobot 280 JN: MoveIt2 + RViz + ros2_control + D435i camera. |
| `stop_mycobot.sh` | 31 | Stack sauber beenden - und NICHTS uebrig lassen. |
| `target_latch.py` | 201 | target_latch.py — /welle/ziel-Brücke für zweiphasigen (Vision→RViz) Pick. |
| `teach_boxes.py` | 171 | teach_boxes.py — die 5 Box-Koordinaten für das SORTIEREN nach Farbe anlernen (teach). |
| `teach_click.rviz` | 98 | RViz-2-Konfiguration |
| `teach_punkte.json` | 1850 | JSON-Daten; Schlüssel: _archiv_2026-09-10_11, nest_11, nest_11_ablage, nest_12, nest_13, nest_14 … |
| `teach_punkte_encoder_2026-09-20.json` | 1752 | JSON-Daten; Schlüssel: _archiv_2026-09-10_11, nest_11, nest_11_ablage, nest_12, nest_13, nest_14 … |
| `teach_punkte_vor_J5_2026-09-23.json` | 1810 | JSON-Daten; Schlüssel: _archiv_2026-09-10_11, nest_11, nest_11_ablage, nest_12, nest_13, nest_14 … |
| `teach_setup.sh` | 114 | teach_setup.sh — Lernumgebung für das Anlernen von 5 Boxen zur Farb-SORTIERUNG (ISOLIERT, KLICK-UND-GO). |
| `view_camera.sh` | 109 | view_camera.sh — NUR Kamerabild + Roboter-Reach-RINGE. |
| `view_overlay.py` | 79 | view_overlay.py — /welle/overlay in einem LEICHTGEWICHTIGEN Fenster live zeigen. |
| `visit_boxes.py` | 159 | visit_boxes.py — die 5 angelernten Boxen DER REIHE NACH abfahren (Prüfrunde). |
| `vo_rsp.launch.py` | 29 | Vision-only TF: nur robot_state_publisher (Kamera->robot_base statischer TF). |

## `src/mycobot_calibration`

| Datei | Zeilen | Beschreibung |
|---|---:|---|
| `src/mycobot_calibration/package.xml` | 24 | ROS-2-Paketbeschreibung: Hand-eye calibration helpers for the myCobot 280 JN + RealSense D435i (FAZ 5). |
| `src/mycobot_calibration/setup.cfg` | 4 | Installationspfade des ROS-2-Python-Pakets |
| `src/mycobot_calibration/setup.py` | 32 | Installationsskript des ROS-2-Python-Pakets (ament_python) |
| `src/mycobot_calibration/config/charuco_params.yaml` | 32 | ChArUco-Board-Parameter — muss mit dem physischen Board des Benutzers übereinstimmen. |
| `src/mycobot_calibration/config/charuco_params_gripper.yaml` | 26 | ChArUco 6x3 Greifer-Board-Parameter — muss mit dem gedruckten PDF übereinstimmen. |
| `src/mycobot_calibration/launch/calibrate_eye_on_base.launch.py` | 78 | Hand-Auge-Kalibrierung — eye-on-base (Kamera ortsfest, Board am Roboter/Greifer). |
| `src/mycobot_calibration/launch/calibrate_full.launch.py` | 254 | Phase 5 — Hand-eye-Kalibrierung kompletter Stack. |
| `src/mycobot_calibration/launch/calibrate_session.launch.py` | 184 | Phase-5-Kalibriersitzung — schlanker Stack OHNE easy_handeye2. |
| `src/mycobot_calibration/launch/calibrate_standalone.launch.py` | 146 | Eigenständiger Basis-Stack — Roboter + Slider-GUI + D435i-Kamera OHNE Detektor/handeye. |
| `src/mycobot_calibration/mycobot_calibration/__init__.py` | 1 | Python-Paketkennzeichnung |
| `src/mycobot_calibration/mycobot_calibration/charuco_detector.py` | 468 | ChArUco-Board-Detektor — publiziert die Transformation camera_optical_frame -> charuco_board als TF. |
| `src/mycobot_calibration/mycobot_calibration/estop_button.py` | 147 | NOT-AUS Taster — eigenständiges, immer-im-Vordergrund Fenster. |
| `src/mycobot_calibration/mycobot_calibration/joint_encoder_publisher.py` | 235 | myCobot 280 JN — bidirectional serial bridge for hand-eye calibration. |
| `src/mycobot_calibration/mycobot_calibration/joint_pose_gui.py` | 325 | Tkinter-GUI mit Gelenk-Schiebereglern für den myCobot 280 JN. |
| `src/mycobot_calibration/mycobot_calibration/nachfuehren.py` | 132 | Fuehrt die aktuelle Sollstellung nach, bis der Encoder sie bestaetigt. |
| `src/mycobot_calibration/mycobot_calibration/numerical_ik.py` | 254 | myCobot 280 JN — Multi-Seed-Wrapper um MoveIts compute_ik-Service. |
| `src/mycobot_calibration/mycobot_calibration/positionieren.py` | 338 | Aeussere Positionsschleife: faehrt ein Gelenkziel an, BIS es erreicht ist. |
| `src/mycobot_calibration/scripts/calibrate_handeye.py` | 371 | Terminal-based hand-eye calibration sampler for myCobot 280 JN + D435i. |
| `src/mycobot_calibration/scripts/camera_point_marker.py` | 292 | Kamera-Bild anklicken -> in RViz als 3D-Marker anzeigen. |
| `src/mycobot_calibration/scripts/cartesian_jog.py` | 240 | Kartesischer Jog GUI — myCobot 280 JN. |
| `src/mycobot_calibration/scripts/click_arc.py` | 725 | 3 Punkte ins Kamera-Bild klicken -> der hindurchgehende KREIS wird bestimmt -> Roboter zeichnet den VIERTELKREIS-Bogen durch diese Punkte (~3cm über der Plattform, Greifer senkrecht-nach-unten). |
| `src/mycobot_calibration/scripts/click_place.py` | 366 | Ein-Klick PLACE: Ablage-Punkt in der Kamera anklicken -> Roboter dreht die WAAGERECHT zwischen den Fingern gehaltene Welle SENKRECHT (Anflug waagerecht) und führt sie über diesen Punkt -> Sicht-Kontrolle (mit Roll aufric |
| `src/mycobot_calibration/scripts/click_place_moveit.py` | 521 | Ein-Klick PLACE — MoveIt KOLLISIONSFREIER Plan + RViz-Vorschau + BESTAETIGEN-ausführen. |
| `src/mycobot_calibration/scripts/click_to_go.py` | 846 | Kamera-Bild anklicken -> MoveIt Plan -> in RViz sehen -> ausführen. |
| `src/mycobot_calibration/scripts/generate_charuco_board.py` | 126 | ChArUco 5x7 board generator — Phase 5 hand-eye calibration. |
| `src/mycobot_calibration/scripts/generate_charuco_gripper.py` | 128 | ChArUco 6x3 Greifer-Marker-Generator — für die Neukalibrierung (eye-to-hand). |

## `src/mycobot_demo`

| Datei | Zeilen | Beschreibung |
|---|---:|---|
| `src/mycobot_demo/CMakeLists.txt` | 81 | Build-Konfiguration des ROS-2-Pakets (ament_cmake) |
| `src/mycobot_demo/package.xml` | 32 | ROS-2-Paketbeschreibung: MoveIt-2-basierte Demo-Nodes fuer den myCobot 280 JN. |
| `src/mycobot_demo/launch/approach_target.launch.py` | 118 | Ziel-Annäherungs-Demo (nur pre-grasp): 0-Punkt → 10cm über dem Ziel, 90° senkrecht, Greifer öffnen. |
| `src/mycobot_demo/launch/goto_clicked_point.launch.py` | 115 | Kalibrierungs-Verifikation: RViz-Klick → Roboter hover. |
| `src/mycobot_demo/launch/pick_place.launch.py` | 122 | Führt die Pick & Place Demo aus. |
| `src/mycobot_demo/launch/pick_tilt.launch.py` | 146 | Tilt-bewusster PICK (Sitzung 25): home → hover → Bestätigung → paralleler Abstieg → greifen → heben → halten. |
| `src/mycobot_demo/rviz/approach.rviz` | 179 | RViz-2-Konfiguration |
| `src/mycobot_demo/src/approach_target.cpp` | 242 | myCobot 280 JN — Wellen-/Ziel-ANNAEHERUNG (nur pre-grasp, KEIN descend/greifen). |
| `src/mycobot_demo/src/goto_clicked_point.cpp` | 337 | RViz-Klick → Greifer senkrecht 5 cm über den Punkt hovern. |
| `src/mycobot_demo/src/pick_place_cartesian.cpp` | 351 | myCobot 280 JN — Wellen Pick & Place (Cartesian Annäherung + orientation constraint). |
| `src/mycobot_demo/src/pick_tilt.cpp` | 627 | myCobot 280 JN — Tilt-bewusster Wellen-PICK (Hover + Bestätigung + paralleler Abstieg + greifen + heben). |

## `src/mycobot_hardware`

| Datei | Zeilen | Beschreibung |
|---|---:|---|
| `src/mycobot_hardware/CMakeLists.txt` | 74 | Build-Konfiguration des ROS-2-Pakets (ament_cmake) |
| `src/mycobot_hardware/mycobot_hardware_plugin.xml` | 10 | – |
| `src/mycobot_hardware/package.xml` | 29 | ROS-2-Paketbeschreibung: ros2_control SystemInterface for myCobot 280 JN. |
| `src/mycobot_hardware/include/mycobot_hardware/mycobot_hardware.hpp` | 102 | myCobot 280 JN ros2_control SystemInterface. |
| `src/mycobot_hardware/scripts/estop_relay.py` | 98 | NOT-AUS-Relay — übersetzt das ROS-Topic /estop in das "estop"-Wire-Kommando des mycobot_bridge-Prozesses (Unix-Kommando-Socket). |
| `src/mycobot_hardware/scripts/mycobot_bridge.py` | 441 | myCobot 280 JN bridge — owns pymycobot, exposes state/commands over Unix socket. |
| `src/mycobot_hardware/src/mycobot_hardware.cpp` | 472 | myCobot 280 JN SystemInterface — talks to mycobot_bridge.py subprocess over a Unix socket. |

## `src/mycobot_moveit_config`

| Datei | Zeilen | Beschreibung |
|---|---:|---|
| `src/mycobot_moveit_config/CMakeLists.txt` | 10 | Build-Konfiguration des ROS-2-Pakets (ament_cmake) |
| `src/mycobot_moveit_config/package.xml` | 32 | ROS-2-Paketbeschreibung: myCobot 280 JN için MoveIt 2 konfigürasyon paketi. |
| `src/mycobot_moveit_config/config/cartesian_limits.yaml` | 16 | Kartesische Limits für den Pilz-Industrial-Motion-Planner (PTP/LIN/CIRC). |
| `src/mycobot_moveit_config/config/joint_limits.yaml` | 48 | Gelenk-Geschwindigkeits- und Beschleunigungslimits für die Zeitparametrisierung. |
| `src/mycobot_moveit_config/config/kinematics.yaml` | 23 | Inverse-Kinematik-(IK-)Konfiguration für die Planungsgruppe "arm". |
| `src/mycobot_moveit_config/config/moveit_controllers.yaml` | 53 | MoveIt → ros2_control Brücke. |
| `src/mycobot_moveit_config/config/mycobot.srdf` | 427 | SRDF für myCobot 280 JN + adaptive gripper. |
| `src/mycobot_moveit_config/config/ompl_planning.yaml` | 69 | OMPL-Planungskonfiguration (Open Motion Planning Library). |
| `src/mycobot_moveit_config/config/ros2_controllers.yaml` | 73 | ros2_control controllers (fake execution mit mock components). |
| `src/mycobot_moveit_config/launch/demo.launch.py` | 352 | MoveIt 2 Demo Launch — Fake-Execution mit ros2_control fake components. |
| `src/mycobot_moveit_config/launch/moveit_rviz.launch.py` | 145 | RViz-only launch — um sich mit einem BEREITS LAUFENDEN move_group zu verbinden. |
| `src/mycobot_moveit_config/launch/rviz_only.launch.py` | 85 | Nur RViz (MotionPlanning-Panel) zu einem BEREITS LAUFENDEN Stack starten. |
| `src/mycobot_moveit_config/rviz/moveit.rviz` | 595 | RViz-2-Konfiguration |

## `src/mycobot_world`

| Datei | Zeilen | Beschreibung |
|---|---:|---|
| `src/mycobot_world/CMakeLists.txt` | 18 | arbeitsraum_marker.py zeichnet die GEMESSENE Reichweitengrenze in RViz. |
| `src/mycobot_world/package.xml` | 21 | ROS-2-Paketbeschreibung: Sahne paketi: myCobot 280 JN + adaptive gripper + eye-to-hand RealSense D435i. |
| `src/mycobot_world/config/arbeitsraum_ring.json` | 147 | JSON-Daten; Schlüssel: z, frame, orientierung, ring, gemessen, totsektor |
| `src/mycobot_world/launch/world.launch.py` | 126 | myCobot 280 JN + adaptive gripper + eye-to-hand RealSense D435i — RViz display. |
| `src/mycobot_world/rviz/world.rviz` | 291 | RViz-2-Konfiguration |
| `src/mycobot_world/scripts/arbeitsraum_marker.py` | 148 | Veroeffentlicht die GEMESSENE Reichweitengrenze als Marker fuer RViz. |
| `src/mycobot_world/scripts/arbeitsraum_wolke.py` | 99 | Punktwolke auf den Arbeitsraum beschneiden: nur Punkte ueber der Kaiser-Grundplatte. |
| `src/mycobot_world/urdf/ablageplatte.xacro` | 98 | ABLAGEPLATTE (Kollisionsmodell)  5 Zeilen x 4 Spalten Nester, in die die Wellen senkrecht gestellt werden. |
| `src/mycobot_world/urdf/ablageplatte_pose.xacro` | 40 | ERZEUGTE DATEI - NICHT VON HAND AENDERN  Lage der Ablageplatte (robot_base -> ablageplatte) und die Nestbosse, erzeugt von tools/ablageplatte_modell.py aus teach_punkte.json (20 Nester), 2026-09-12. |
| `src/mycobot_world/urdf/camera_pose.xacro` | 36 | AUTOMATISCH ERZEUGTE DATEI — NICHT VON HAND AENDERN  Kamera-Pose (robot_base -> camera_bottom_screw_frame). |
| `src/mycobot_world/urdf/mycobot_world.urdf.xacro` | 872 | mycobot_280 JN + adaptive gripper + eye-to-hand RealSense D435i. |
| `src/mycobot_world/urdf/schale.xacro` | 168 | BEREITSTELLUNGSSCHALE (Kollisionsmodell)  Die Schale, aus der die Wellen entnommen werden. |
| `src/mycobot_world/urdf/schale_pose.xacro` | 36 | ERZEUGTE DATEI - NICHT VON HAND AENDERN  Lage der Bereitstellungsschale (robot_base -> schale). |
| `src/mycobot_world/urdf/trichter.xacro` | 265 | TRICHTER (Vereinzelung / Aufrichten der Welle)  Kollisionsmodell. |
| `src/mycobot_world/urdf/trichter_pose.xacro` | 49 | LAGE DES TRICHTERS (robot_base -> trichter)  Diese Datei ist die EINZIGE Quelle der Trichterlage im Modell; mycobot_world.urdf.xacro liest sie hier ein. |

## `src/wellenerkennung`

| Datei | Zeilen | Beschreibung |
|---|---:|---|
| `src/wellenerkennung/package.xml` | 24 | ROS-2-Paketbeschreibung: Wellenerkennung mit YOLO11-seg auf der D435i und Erzeugung von 3D-Zielen fuer Greifen und Ablegen (myCobot 280 JN). |
| `src/wellenerkennung/setup.cfg` | 4 | Installationspfade des ROS-2-Python-Pakets |
| `src/wellenerkennung/setup.py` | 28 | Installationsskript des ROS-2-Python-Pakets (ament_python) |
| `src/wellenerkennung/launch/wellen_detektor.launch.py` | 97 | Startet den wellen_detektor Node (Phase 2 — nur Erkennung + Overlay). |
| `src/wellenerkennung/wellenerkennung/__init__.py` | 1 | Python-Paketkennzeichnung |
| `src/wellenerkennung/wellenerkennung/erkennung.py` | 202 | Wellen-Erkennung — Mittelpunkt + Winkel + Farbe aus YOLO11-seg Maske. |
| `src/wellenerkennung/wellenerkennung/wellen_detektor_node.py` | 1307 | wellen_detektor — Wellenerkennung mit YOLO11-seg + 3D-Zielerzeugung aus D435i. |

## `tools`

| Datei | Zeilen | Beschreibung |
|---|---:|---|
| `tools/README.md` | 12 | tools/ |
| `tools/ablage_mit_orientierung.py` | 79 | TCP an einen Ort fahren und dabei die AKTUELLE Werkzeug-Orientierung behalten (MoveIt, kollisions- geprueft, plan_only-Vorschau). |
| `tools/ablageplatte_modell.py` | 105 | Kollisionsmodell der ABLAGEPLATTE (5 x 4 Nester) aus den angelernten Nestern erzeugen. |
| `tools/ablauf.py` | 247 | Ablauf: Welle(n) holen und in Nester der Ablageplatte stellen - die Kette aus den Einzelskripten, mit ZWEI MODI, die an alle durchgereicht werden: MANUELL (Default) vor jedem Zyklus, vor jedem Teilschritt und in den Skri |
| `tools/acm_greifer.py` | 67 | Greifer-Glieder in der MoveIt-Kollisionsmatrix (ACM) freigeben oder zuruecksetzen. |
| `tools/arbeitsraum_grenze.py` | 101 | Aeussere und innere Reichweitengrenze in EINER Hoehe bestimmen - ohne den Roboter zu bewegen. |
| `tools/auto_camera_calibration.py` | 354 | Automatische Kamera-Nachfuehrung: messen -> vergleichen -> Modell nachziehen. |
| `tools/bahnpruefung.py` | 47 | Stetigkeitsprobe fuer kartesische Bahnen (compute_cartesian_path) - Pflicht vor jeder Fahrt. |
| `tools/calibrate_handeye_auto.py` | 191 | Hand-Auge-Kalibrierung, die ihre Stellungen selbst anfaehrt. |
| `tools/export_code_single_file.py` | 105 | Exportiert den GESAMTEN selbst geschriebenen Quellcode in EINE Textdatei. |
| `tools/gelenkspiel.py` | 55 | Gemeinsame Regeln fuer das Gelenkspiel (Spiel hinter dem Getriebe), gemessen am 2026-09-19. |
| `tools/gelenkspiel_messreihe.py` | 403 | Messreihe GELENKSPIEL: wie schief steht der "senkrechte" Greifer wirklich, und wie gross ist der Fehler, den der Encoder NICHT sieht - je Nest und je Anfahrrichtung. |
| `tools/greifer.py` | 30 | Nur den Greifer fahren (gripper_controller, ROS 2). |
| `tools/hebe_und_parke.py` | 196 | Erst SENKRECHT anheben, dann zur Nullstellung — ohne den Greifer zu drehen. |
| `tools/hole_aus_schale.py` | 634 | Welle aus der BEREITSTELLUNGSSCHALE holen (2026-09-12 Nacht Skelett; 2026-09-13 01:5x ERSTER GRIFF). |
| `tools/hole_welle.py` | 533 | Welle aus dem Trichter holen — das Rezept des ERSTEN VOLLEN ZYKLUS (2026-09-12, 01:5x). |
| `tools/j4_von_oben.py` | 45 | Achse 4 (und seit 20.9. |
| `tools/jog_gui.py` | 226 | Handjog-Fenster: Arm in kleinen Schritten fahren - Gelenke UND kartesisch (dx/dy/dz/radial). |
| `tools/jog_ohne_drehung.py` | 242 | Jog-Fenster, das den Greifer NICHT dreht. |
| `tools/kamera_pruefung.py` | 138 | Kamera-Pruefungen fuer den Zyklus (zyklus_gui.py). |
| `tools/kamera_roboter_abbildung.py` | 321 | KAMERA -> ROBOTER-Abbildung im Arbeitsbereich messen (ChArUco am Greifer) und fitten. |
| `tools/kamerapose.py` | 148 | KAMERAPOSE: Nullstellung, aber Gelenk 5 um 90 Grad gedreht, so dass der Greifer WAAGERECHT nach -y zeigt und NICHT unter der Kamera haengt (Benutzer 2026-09-10/14: in der Nullstellung ragen die Finger ins Bild, YOLO hiel |
| `tools/kippe_um_tcp.py` | 163 | Den Greifer um die FINGERSPITZE kippen - die Spitze bleibt, wo sie ist. |
| `tools/kontrollpose.py` | 181 | KONTROLLPOSE: mit der Welle im Greifer unter die Kamera fahren, Greifer WAAGERECHT, damit die Kamera die beiden herausragenden Wellenenden sieht (Kopf Ø 10 dick / Schaft Ø 7 duenn). |
| `tools/kopfseite_pruefen.py` | 162 | Sicherheitsstufe nach dem Griff (Benutzerwunsch 2026-09-14): sitzt der KOPF der Welle auf der richtigen Greiferseite? |
| `tools/lege_welle.py` | 442 | Welle in ein Nest der Ablageplatte stellen — das Rezept des ERSTEN VOLLEN ZYKLUS (2026-09-12). |
| `tools/measure_approach_backlash.py` | 259 | Misst, wie viel Spiel beim ANFAHREN uebrig bleibt — je nach Anfahrrichtung. |
| `tools/measure_camera_height.py` | 238 | Misst die WIRKLICHE Kamerahoehe ueber der Plattform — direkt aus der Tiefenkarte. |
| `tools/measure_camera_pose.py` | 564 | Misst die VOLLE 3D-Pose der Kamera ueber der bekannten Plattform-Platte. |
| `tools/measure_deflection_camera.py` | 399 | Misst mit der KAMERA, wie weit der Greifer nachgibt, wenn von Hand gezogen wird. |
| `tools/measure_joint_play.py` | 257 | Misst das Spiel/Nachgeben je Gelenk, waehrend der Arm von Hand ausgelenkt wird. |
| `tools/measure_model_accuracy.py` | 265 | Misst, wie genau das Modell den ECHTEN Roboter beschreibt. |
| `tools/measure_model_vs_charuco.py` | 231 | Misst den Modellfehler gegen das ChArUco-Board am Greifer. |
| `tools/messpose_frei.py` | 138 | Greifstellung im FREIEN Raum nachbilden - zum Messen der Greiferneigung. |
| `tools/nachstellen.py` | 128 | Nachstellen: die Gelenke so lange ueber das Ziel hinaus kommandieren, bis der Encoder das Ziel meldet. |
| `tools/nest_anfahren.py` | 316 | Ein Nest der Ablageplatte LEER anfahren — prueft, ob ein angelernter/gerechneter Punkt sitzt. |
| `tools/nest_fuer_farbe.py` | 54 | Zielnest fuer eine Welle aus Koerper- und Streifenfarbe (farbplan.json, Benutzer 12.9.). |
| `tools/nester_ins_bild.py` | 50 | Alle Nester (und beliebige Punkte) ins Kamerabild projizieren - Sichtpruefung der Kamerakalibrierung: liegen die projizierten Nestpunkte auf den echten Nestern? |
| `tools/nullstellung_moveit.py` | 35 | Kollisionsgeprueft in die Nullstellung (MoveIt). |
| `tools/pruefe_kalibrierung_platte.py` | 238 | Probe fuer JEDE Kamerakalibrierung: kommt die Grundplatte dort heraus, wo sie steht? |
| `tools/pruefe_nest_kamera.py` | 113 | Kamera-Pruefung: steht in einem Nest der Ablageplatte eine Welle (aufrecht), liegt sie, fehlt sie? |
| `tools/punkt_klicken.py` | 159 | Punkt im Kamerabild anklicken -> robot_base-Koordinate -> Marker in RViz -> hinfahren. |
| `tools/punkt_pruefen.py` | 149 | Angelernten Punkt OHNE Fahrt gegen das Kollisionsmodell pruefen (Vorschau in RViz). |
| `tools/schale_erreichbarkeit.py` | 200 | Erreichbarkeit der Bereitstellungsschale pruefen - OHNE den Arm zu bewegen. |
| `tools/schale_finden.py` | 747 | Die Bereitstellungsschale im Tiefenbild SELBST finden - ohne Klicken, ohne Lineal. |
| `tools/schale_finden_kontur.py` | 598 | Die Bereitstellungsschale ueber ihren UMRISS IM FARBBILD finden (zweiter Weg). |
| `tools/schale_mesh.py` | 126 | Bereitstellungsschale als STL-Mesh mit RUNDEN Kanten (Benutzer 2026-09-14: "keine eckige Kante, jede Ecke oval - unten, oben, aussen, innen"). |
| `tools/schale_pose_klicken.py` | 339 | Lage der Bereitstellungsschale bestimmen - mit Klicks in RViz, ohne Lineal. |
| `tools/senkrecht_stellen.py` | 127 | Greifer an einer Stelle SENKRECHT stellen: IK (senkrecht, fester Gierwinkel) fuer die gewuenschte TCP-Lage, Gelenke direkt anfahren, dann gedaempft nachstellen (Totband). |
| `tools/servo_frei.py` | 23 | EINEN Servo stromlos schalten (von Hand drehbar) oder wieder festsetzen - ueber den Kommando- Socket der Bruecke (kein Stack-Neustart, kein pymycobot-Zugriff am Port vorbei). |
| `tools/servo_nullen.py` | 25 | Servo-NULLPUNKT an der aktuellen Stellung setzen (set_servo_calibration, Potentialwert 2048). |
| `tools/teach_aus_ros.py` | 148 | Aktuelle Gelenkstellung aus /joint_states speichern — waehrend der Stack laeuft. |
| `tools/teach_freihand.py` | 160 | Anlernen von Stellungen VON HAND — Servos frei, Arm wird gefuehrt. |
| `tools/teach_gelenk_verschieben.py` | 42 | EIN Gelenk in allen Teach-Punkten (teach_punkte.json) und Kalibrierposen (kalibrierposen_charuco.json) um --grad verschieben. |
| `tools/teach_schritt.py` | 112 | Anlernen in EINZELSCHRITTEN — jeder Aufruf macht genau eine Sache. |
| `tools/test_position_retry.py` | 221 | Prueft, ob ERNEUTES SENDEN desselben Ziels den Ankunftsfehler verkleinert. |
| `tools/test_welle_finden.py` | 156 | Regressionsprobe fuer welle_finden_schale.py an GESICHERTEN Frames (testdaten/*.npz) - ohne Roboter. |
| `tools/trichter_lage_pruefen.py` | 120 | Trichterlage pruefen: Kegel als Planning-Scene-Objekt, Neigungsrichtung (yaw) durchdrehen. |
| `tools/trichterfuss_mesh.py` | 55 | Trichterfuss als STL: Halbscheibe (Radius r, Halbkreis nach +X) mit OVALEN Ecken an der geraden Kante (Benutzer 2026-09-14: die Ecken sind real rund, nicht spitz). |
| `tools/umlaut_convert.py` | 208 | Wandelt ASCII-Deutsch in echte Umlaute um - nur in Kommentaren und Docstrings. |
| `tools/versetze_soll.py` | 97 | Kleine gerade Verschiebung, gerechnet auf SOLL (gespeicherte Gelenkwinkel), nicht auf Ist. |
| `tools/versetze_tcp.py` | 147 | Verschiebt den TCP um einen kleinen Betrag — geradlinig, ohne den Greifer zu drehen. |
| `tools/welle_farbe.py` | 225 | Koerper- und STREIFENFARBE einer liegenden Welle aus dem Kamerabild (2026-09-13). |
| `tools/welle_finden_schale.py` | 890 | Eine Welle in der Schale im Kamerabild finden - ohne YOLO - und als MODELL in RViz zeigen. |
| `tools/zeichne_greifer.py` | 131 | Zeichnet das KOLLISIONSMODELL des Greifers massstaeblich, zum Vergleich mit dem Foto. |
| `tools/zeige_punkt.py` | 180 | Faehrt den TCP senkrecht ueber einen Punkt und LAESST IHN DORT STEHEN. |
| `tools/zyklus_gui.py` | 556 | PROGRAMMABLAUF mit Fenster (Benutzerwunsch 2026-09-14): Schale -> Griff -> Kamerakontrolle -> Trichter -> Kamerakontrolle -> Griff am Kopf -> Ablageplatte nach Farbe, bis die Schale leer ist. |


## Nach der Abgabe ergänzte Dateien (05.10.2026)

Die folgenden Dateien waren im Stand vom 28.09.2026 nicht im Repository. In `quellcode/` sind es Binär- und Datendateien des Workspace, die beim ursprünglichen Hochladen (nur Textdateien) nicht übertragen wurden; alles außerhalb von `quellcode/` wurde neu erstellt. Bereits vorhandene Dateien wurden nicht geändert.

### `quellcode/` (nachgereicht, unverändert vom Jetson übernommen)

| Datei | Beschreibung |
|---|---|
| `src/mycobot_world/meshes/schale_boden.stl`, `schale_wand.stl` | Bereitstellungsschale (Boden, Wand) als Mesh für Modell und Kollisionsprüfung |
| `src/mycobot_world/meshes/trichterfuss_r60_oval.stl`, `halbscheibe_r60.stl` | Fuß bzw. Halbscheibe des trichterförmigen Übergabeelements |
| `src/mycobot_world/meshes/finger_keil_pos.stl`, `finger_keil_neg.stl` | keilförmige Spitzen der angepassten Greiferfinger |
| `src/wellenerkennung/resource/wellenerkennung`, `src/mycobot_calibration/resource/mycobot_calibration` | leere Markerdateien (0 Byte) für den ament-Paketindex; ohne sie bricht `colcon build` ab |
| `src/wellenerkennung/weights/welle_gross.pt` | trainierte YOLO11l-seg-Gewichte (54 MB; Training 30.03.2026, Ultralytics 8.4.31), vom Detektorknoten und von `offline_demo/yolo_offline.py` geladen |
| `testdaten/wellen_11_kamerapose_2026-09-13.npz`, `wellen_11_stueck_schale_2026-09-13.npz`, `wellen_12_stueck_verteilt_2026-09-13.npz`, `welle_weiss_rot_schale_2026-09-13.npz` | aufgezeichnete RGB-D-Szenen (Farbbild 1280×720, Tiefenbild, Kameraparameter `K`, Kamerapose `q`/`t`, Lage der Schale) für `tools/test_welle_finden.py` |
| `testdaten/schale_hintergrund_2026-09-13.npz` | Referenzbild der leeren Schale zu diesen Szenen |
| `testdaten/mosaik/*_mosaik.png` | am 19.09.2026 von `tools/test_welle_finden.py` erzeugte Mosaike (ein Ausschnitt je gefundener Welle) |
| `schale_hintergrund.npz`, `trichter_hintergrund.npz` | Referenzbilder der leeren Schale bzw. des leeren Trichters, von `tools/welle_finden_schale.py` und `tools/kamera_pruefung.py` verwendet |
| `zyklus_protokoll.txt`, `zyklus_log.jsonl` | Protokoll der Ablaufsteuerung `tools/zyklus_gui.py` (Text bzw. ein JSON-Eintrag je Schritt); nur Läufe über die Bedienoberfläche (14., 23., 24.09.2026), nicht die Datengrundlage der Tabelle 6-2 (Laborbuch) |

### `offline_demo/` (neu)

| Datei | Beschreibung |
|---|---|
| `README.md` | Anleitung, erwartete Ergebnisse und Stand der Prüfung |
| `START_WINDOWS.bat`, `start.sh` | legt eine virtuelle Umgebung an, installiert die Pakete und startet beide Demos |
| `requirements-offline.txt` | Python-Pakete der Offline-Demo |
| `wellenerkennung_offline.py` | führt `quellcode/tools/test_welle_finden.py` ohne ROS 2 auf den Szenen in `quellcode/testdaten/` aus |
| `yolo_offline.py` | YOLO11l-seg-Inferenz mit `welle_gross.pt` und Auswertung mit `wellenerkennung/erkennung.py` |
| `ros_stubs/` | Platzhalter für `rclpy`, `tf2_ros` und Nachrichtenpakete, damit sich die Module in `quellcode/tools/` ohne ROS 2 importieren lassen |

### `upstream_patches/` (neu)

| Datei | Beschreibung |
|---|---|
| `mycobot_ros2.patch` | gegen Commit `3999e2c`: Kollisionskörper (Zylinder/Quader) für Armglieder und Greifer, Drehung des Greifers am Flansch (48,16°), Tippfehler in der Gelenkgrenze der ersten Achse, `setup.cfg`-Schreibweise |
| `trac_ik_galactic.patch` | gegen Commit `56136bb`: Portierung des Kinematik-Plugins auf ROS 2 Galactic |
| `realsense-ros.patch` | gegen Tag `4.51.1`: Parameter `use_mesh` im Makro `sensor_d435i` |
| `easy_handeye2.patch` | gegen Commit `b42cae6`: Typangaben `Optional[...]` für Python 3.8 |

### Wurzel des Repositorys

| Datei | Beschreibung |
|---|---|
| `requirements.txt` | Python-Pakete des Versuchssystems |
| `.gitignore`, `.gitattributes` | Git-Einstellungen (keine `__pycache__`-Ordner; `.bat` mit Windows-Zeilenenden) |
