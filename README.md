# Bachelorarbeit — Gesamter selbst geschriebener Quellcode

Export aus `~/ros2_ws` (ROS 2 Galactic, Jetson Nano) — Stand **2026-08-31**.
Thema: Wellen-Erkennung (YOLO11-seg + RealSense D435i) und Griff mit myCobot 280 JN.

## Inhalt

| Datei / Ordner | Beschreibung |
|---|---|
| `quellcode/` | 1:1-Kopie aller selbst geschriebenen Quelldateien (Ordnerstruktur wie im Workspace) |
| `ALLE_CODES.txt` | Alles in EINER Textdatei — zum Suchen und Kopieren in die Thesis |
| `ALLE_CODES.html` | Druckfertige Fassung (im Browser öffnen → Drucken → „Als PDF speichern") |
| `ALLE_CODES.pdf` | Dieselbe Fassung als PDF |
| `push_to_github.sh` | Lädt den Workspace auf GitHub (Konto `kakir001`), sobald Internet da ist |

## Was NICHT enthalten ist

Fremdcode (Upstream, unverändert übernommen) und Binärdaten:
`mycobot_ros2`, `realsense-ros`, `easy_handeye2`, `trac_ik` (portiert, aber fremder Ursprung),
YOLO-Gewichte (`weights/`, 54 MB), Datensätze, Bilder, Logs, `build/`, `install/`, `log/`.

## Eigene Pakete

| Paket | Aufgabe |
|---|---|
| `mycobot_world` | URDF/Xacro der Gesamtszene: Roboter, Stand, Säule, Kamera-Arm, Hand-Auge-Pose |
| `mycobot_moveit_config` | MoveIt-2-Konfiguration: SRDF, Kinematik (TRAC-IK), OMPL, Controller, RViz |
| `mycobot_hardware` | `ros2_control`-Hardware-Interface zur echten Hardware (Bridge über `/dev/ttyTHS1`) |
| `mycobot_calibration` | Hand-Auge-Kalibrierung (ChArUco, easy_handeye2, Park/Horaud) + Prüfwerkzeuge |
| `mycobot_demo` | Bewegungs-Demos, Not-Aus-Kette, Park-/Nullstellungs-Knoten |
| `vida_vision` | YOLO11-seg-Erkennung, 3D-Rückprojektion, PCA-Greifpose, Pick-Ablauf |
| `(ros2_ws-Skripte)` | Start-, Test- und Mess-Skripte im Workspace-Wurzelverzeichnis + `DEVLOG.md` |
