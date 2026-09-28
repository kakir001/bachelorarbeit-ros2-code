# Pruefung der Transformation - Antast-Messungen am Roboter (2026-06-01, Sitzung 14)

Verfahren: Detektor-Position in BASE gegen den angetasteten TCP (Referenz).
Delta = Detektor - TCP.

| Versuch | Detektor BASE (mm) | TCP angetastet (mm) | dx | dy | dz | Anmerkung |
|---------|--------------------|---------------------|----|----|----|-----------|
| 1 | (150, -70, -5) | (125, -62, -14) | +25 | -8 | +9 | niedrige Konfidenz (0.25-0.48); rueckwaerts (erst antasten, dann messen) |
| 2 | (176, -108, -9) | (135, -109, -9) | +41 | +1 | 0 | hohe Konfidenz (0.79-0.85); vorwaerts; Greiferausrichtung anders als in Versuch 1 |

## Auswertung (2 Versuche)
- dx: +25, +41 - der Detektor liegt in beiden Faellen VOR dem echten Antastpunkt
  (positiver Versatz), aber nicht konstant, sondern wachsend.
- dy: -8, +1 - klein, liegt im Rauschen des Antastens von Hand.
- dz: +9, 0 - klein.
- STOERGROESSE: Die Greiferausrichtung war in beiden Antastungen sehr verschieden
  (Quaternion), und der Rahmen `tcp` liegt nicht genau an der Fingerspitze. Ein Teil
  der x-Differenz kommt also aus der Drehung dieses Versatzes und ist kein reiner
  Transformationsfehler.
- Naechster Schritt: fuer ein sauberes Ergebnis ein bis zwei weitere Antastungen
  SENKRECHT von oben mit fester Ausrichtung; dann gilt tcp x-y = Fingerspitze x-y
  = Welle x-y.

## Kalibrierung Schritt 1 angewendet (2026-06-01)
- Datei: `src/mycobot_world/urdf/mycobot_world.urdf.xacro:152`
- Kamera-Ursprung x: **0.2575 -> 0.2245** (-33 mm = mittleres dx)
- TF-Pruefung: camera_color_optical_frame -> robot_base x: 0.245 -> **0.212** ok (-33 mm)
- Build: mycobot_world --symlink-install ok
- y und z unveraendert (liegen im Rauschen)
- OFFEN: Restfehler mit einer sauberen senkrechten Antastung messen, um Ueber- oder
  Unterkorrektur zu erkennen.
- WARNUNG: Die Daten sind ueberlagert (Greiferausrichtung + Versatz TCP zu
  Fingerspitze + Servoabweichung des Roboters ~30 mm). Fuer Schritt 2 empfiehlt sich
  eine kartesische Oberflaeche und eine feste Welle, senkrecht angetastet.
