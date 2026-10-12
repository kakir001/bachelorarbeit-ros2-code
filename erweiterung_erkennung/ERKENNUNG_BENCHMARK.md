# Erkennung der Wellen: klassische Bildverarbeitung oder KI (YOLO11-seg) – Messbericht

Stand: 11.10.2026 · Jetson Nano 4 GB (MAXN, ohne `jetson_clocks`), Ubuntu 20.04, Python 3.8, torch 1.13 (NVIDIA-Build, CUDA 10.2, cuDNN 8.2), Ultralytics 8.4.33, Modell `welle_gross.pt` (YOLO11l-seg, 27,6 Mio. Parameter), `conf` 0,70, `imgsz` 640, FP32 · Kamera D435if 848 × 480 · RViz **aus** · Roboter **nicht bewegt**.

Alle Zahlen sind auf diesem Jetson gemessen. Rohdaten: `erkennung_diagnose_2026-10-11.json`, `erkennung_messreihe_2026-10-11_1242.json` (Endstand), frühere Reihen `…_1144.json`, `…_1217.json`, `…_1226.json`. Im Repository liegen sie in `erweiterung_erkennung/messdaten/`.

## 1. Ergebnis in Kürze

| | klassisch | KI (YOLO11-seg) |
|---|---:|---:|
| Bereitschaft je Sitzung (Laden + Aufwärmen, einmal) | sofort | 54–83 s (Median der vier Starts ≈ 78 s) |
| Wellensuche je Welle, wie Schritt 1 im Zyklus (Median, n = 10, abwechselnd) | **20,5 s** (12,6–37,6) | **22,9 s** (15,7–40,1) |
| davon Segmentierung (Median) | 0,54 s | 0,94 s (Inferenz im Median ≈ 0,76 s, einzelne bis 2,3 s) |
| gefundene Kandidaten in der vollen Schale (Median; tatsächlich ≈ 20 Wellen, Teile vom Greifer verdeckt) | 18 (4–21; frühere Reihe 4–25) | 12 (10–13) |
| häufigste gewählte Welle in 10 Suchen (Lage ± 12 mm) | 4/10 (6 verschiedene Wellen) | 7/10 (2 verschiedene Wellen) |
| kleinster freier Arbeitsspeicher während der Reihe | 350 MB | 352 MB (Dienst ≈ 2 GB geladen) |
| höchste Temperatur | 66 °C | 70 °C |

* Der größte Teil der Suchzeit sind bei **beiden** Verfahren Prozessstart, `rclpy`, Warten auf Kamera und TF; die Segmentierung selbst ist klein.
* Die KI kostet einmal je Sitzung etwa eine Minute. Danach ist sie je Welle nur ≈ 0,4 s langsamer als die klassische Bildverarbeitung.
* Die KI liefert in der vollen Schale eine stabile Zahl ganzer Wellen (10–13) und wählt öfter dieselbe Welle. Die klassische Bildverarbeitung schwankt mit dem Bild (4–25 Kandidaten; Bruchstücke von 25–35 mm, die als „Länge falsch“ verworfen werden).

## 2. Diagnose: Warum war YOLO im Versuch so langsam? (`tools/erkennung_diagnose.py`)

Jede Variante in einem frischen Prozess, Szene `wellen_11_kamerapose_2026-09-13` (11 Wellen) auf 848 × 480 skaliert, 20 Wiederholungen. Speicher vorher ≈ 2,1–2,8 GB frei, Swap = zram (keine SD-Karte), RViz aus.

| Variante | torch | CUDA-Kontext | 1. conv2d (cuDNN) | YOLO laden | 1. Inferenz | Inferenz Median | Wellen (conf ≥ 0,70) | Peak-RSS |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| u: Ultralytics `predict(device=0)` unverändert | 3,8 s | 12,3 s | 11,2 s | 2,2 s | 74,5 s | 0,54 s | **0** | 2258 MB |
| a: eigener GPU-Weg, FP32, ganzes Bild | 9,0 s | 12,3 s | 28,9 s | 13,4 s | 1,5 s | 0,57 s | 5 | 2091 MB |
| b: wie a + FP16 | 6,6 s | 11,9 s | 9,2 s | 18,3 s | 4,9 s | 0,67 s | 5 | 2316 MB |
| d: wie a + Ausschnitt um die Schale (ROI) | 7,2 s | 11,9 s | 8,8 s | 11,6 s | 4,2 s | 0,64 s | **10** | 2528 MB |
| e: zweiter Start direkt nach a | 7,2 s | 11,8 s | 8,7 s | 13,0 s | 2,3 s | 0,57 s | 5 | 2403 MB |
| f: wie a + `CUDA_CACHE_MAXSIZE` 2 GiB | 7,1 s | 11,9 s | 8,8 s | 12,3 s | 3,0 s | 0,57 s | 5 | 2438 MB |
| g: b + d | 7,0 s | 11,4 s | 8,9 s | 11,0 s | 3,5 s | 0,78 s | 10 | 2490 MB |

Bewertung:

1. **Fehler statt Langsamkeit (Variante u).** Auf diesem Jetson liefert `model.predict(device=0)` von Ultralytics **0 Detektionen**. Ursache gemessen: Ultralytics fusioniert Conv + BatchNorm auf der CPU; diese Fusion ergibt auf dem Jetson NaN-Gewichte (206 640 NaN im Rohausgang bei jeder Eingabe, mit und ohne cuDNN). Dasselbe Modell erst auf die GPU gelegt und **dort** fusioniert rechnet richtig (Klassenmaximum 0,91 wie unfusioniert, NMS 14 wie auf der CPU). `tools/yolo_segmentierung.py` (`GpuModell`) geht diesen Weg und prüft jede Ausgabe auf NaN. Verwandt: Ist die GPU ganz versteckt (`CUDA_VISIBLE_DEVICES=`), rechnet PyTorch auf der Jetson-CPU schon im ersten Faltungsblock NaN; mit sichtbarer GPU liefert die CPU dieselben Ergebnisse wie die GPU. Der CPU-Weg prüft das jetzt beim Laden und bricht mit Meldung ab.
2. **JIT ist nicht die Ursache.** `sm_53` ist im Torch-Build enthalten, der Compute-Cache blieb bei 168 kB, `CUDA_CACHE_MAXSIZE` (f) und ein zweiter Start (e) ändern nichts.
3. **Kaltstart.** CUDA-Kontext ≈ 12 s und erstes cuDNN ≈ 9–29 s fallen bei jedem neuen Prozess an. Die früher beobachteten 100 s bis über 4 min für die erste Inferenz traten bei vollem Speicher auf (Detektor ≈ 2,1 GB neben MoveIt und RViz); in dieser Messung (RViz aus, ≥ 2 GB frei) waren es 1,5–5 s nach dem Laden. Im Gesamtsystem (Stack läuft, RViz aus) brauchte der Dienst 54–83 s bis „bereit“.
4. **Modellgröße / Speicher.** Der Prozess belegt 2,1–2,5 GB. Mit laufendem Stack bleiben ≈ 0,3–0,5 GB frei, das System lagert aus (zram bis 0,8 GB). Das ist die Hauptquelle der Streuung. Mit RViz zusätzlich wird es knapp; die Oberfläche fragt deshalb nach RViz.
5. **FP16 bringt nichts** (b: 0,67 s statt 0,57 s). **Der Ausschnitt um die Schale verdoppelt die Trefferzahl** (d: 10 statt 5 von 11), weil die Wellen dann so groß sind wie in den Trainingsbildern (25–40 cm Abstand statt 60 cm). Vorgabe daher: FP32 + ROI.

## 3. Wirkung der umgesetzten Maßnahmen

| Maßnahme | vorher | nachher |
|---|---|---|
| Eigener GPU-Weg statt `predict` (NaN-Fehler) | 0 Wellen | 5 (ganzes Bild) bzw. 10 (ROI) von 11 |
| Ausschnitt um die Schale | 5 von 11 | 10 von 11 |
| Erkennungsdienst (Modell bleibt geladen) | Laden + Aufwärmen je Welle (Prozess a: 117 s) | einmal je Sitzung 54–83 s, danach Inferenz 0,65–0,75 s |
| Masken einmal lesen, Austausch über `/dev/shm` unkomprimiert | Anfrage 6–14 s (Inferenz 0,7 s) | Anfrage 0,7–1,3 s |
| FP16 | – | nicht übernommen (langsamer) |
| `CUDA_CACHE_MAXSIZE` 2 GiB | – | gesetzt, ohne messbare Wirkung (sm_53 im Build) |
| TensorRT-Engine | – | **nicht umgesetzt**: `tensorrt`-Python-Bindings für Python 3.8 fehlen auf JetPack 4 (nur `trtexec` vorhanden), `onnxruntime` nicht installiert. Ohne Rückfrage nichts installiert |
| `jetson_clocks`, Desktop aus | – | nicht umgesetzt (braucht sudo / Rückfrage) |

## 4. Offline-Vergleich auf den aufgezeichneten Szenen (`tools/test_welle_finden.py --verfahren beide`)

Szenen vom 13.09.2026 (1280 × 720), dieselbe Geometrie für beide Verfahren:

| Szene | Wellen | klassisch: Kandidaten / greifbar | KI: Kandidaten / greifbar | KI-Wellen auch klassisch gefunden | davon gleiche Kopfseite | gleiche Welle gewählt |
|---|---:|---:|---:|---:|---:|---|
| `welle_weiss_rot_schale` | 1 | 1 / 1 | 1 / 1 | 1 | 1 | ja |
| `wellen_11_kamerapose` | 11 | 11 / 7 | 10 / 6 | 10 | 10 | nein |
| `wellen_11_stueck_schale` | 11 | 14 / 6 | 11 / 7 | 11 | 8 | ja |
| `wellen_12_stueck_verteilt` | 12 | 13 / 7 | 11 / 8 | 11 | 11 | nein |

* Die klassischen Mosaike sind nach den Änderungen **bildpunktgleich** mit `testdaten/mosaik/` vom 19.09.2026: das Verhalten mit „Bildverarbeitung“ ist unverändert.
* Bei den greifbaren KI-Kandidaten liegt das Kopfende in den Mosaiken auf der richtigen Seite (Sichtprüfung). Die drei abweichenden Kopfseiten in `wellen_11_stueck_schale` betreffen Kandidaten, deren Enden die klassische Segmentierung verkürzt hat.
* Zeiten im Offline-Vergleich sind nicht aussagekräftig: Modell und Geometrie liefen in **einem** Prozess neben dem Stack, das System lagerte aus (Geometrie 6–23 s statt 1–2,5 s). Im Zyklus läuft das Modell im Dienst, die Geometrie in einem eigenen Prozess (Abschnitt 1).

## 5. Kopf- und Trichterkontrolle mit KI (`tools/kamera_pruefung.py --verfahren yolo`)

Einmalige Probe am Aufbau (Arm über der Schale, Greifer leer, Trichter leer):

| Prüfung | klassisch | KI |
|---|---|---|
| Trichter (leer) | „unklar“ (Differenz zum alten Referenzbild 22 %) | „leer“ (richtig), Inferenz 1,5 s |
| Kopf am TCP (Greifer leer) | „nein“ (richtig) | „nein“ (richtig), Inferenz 1,1 s |

Nicht geprüft: Kopfkontrolle **mit** Welle im Greifer. Das Modell ist auf liegende Wellen trainiert; das Wellenende von vorn kann es verfehlen. Dann meldet die Kontrolle „nein/unklar“, und der Bediener entscheidet (die Oberfläche fragt in Schritt 4 ohnehin immer).

## 6. Offene Punkte

* **IK-Probe:** In der Messung lehnte die IK-Probe (`/compute_ik`, nur Rechnung) alle greifbaren Kandidaten ab, auch klassisch. Die Messreihen liefen deshalb mit `--ohne-ik-probe` (für beide Verfahren gleich). Ursache ungeklärt; der Arm stand nicht in der Kamerapose.
* Messung mit dem Arm in der Kamerapose und ein vollständiger Zyklus mit KI am Roboter (MANUELL, NOT-AUS) stehen aus.
* Kleineres Modell (YOLO11s-seg, Bilder aus 60 cm) und TensorRT: erwartet deutlich weniger Speicher und kürzere Bereitschaft. **Nicht gemessen.**
