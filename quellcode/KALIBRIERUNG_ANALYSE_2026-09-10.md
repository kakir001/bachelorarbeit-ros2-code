# Die Kamerakalibrierung: was schiefging, warum es so lange dauerte, und wie es geloest wurde

Aufgezeichnet am 2026-09-10. Gedacht als Vorlage fuer die Abschnitte
*Evaluation* und *Diskussion* der Bachelorarbeit.

---

## 1. Worum es geht

Die RealSense D435i steht fest ueber dem Arbeitsplatz und blickt nach unten
(**eye-to-hand**): die Kamera bewegt sich nicht mit dem Roboter. Damit aus einem
erkannten Bildpunkt ein Greifziel wird, muss eine einzige Groesse bekannt sein -
die Lage der Kamera gegenueber dem Roboterfuss, `robot_base -> camera`. Jeder
Fehler darin geht **unveraendert** in jede Greifkoordinate ein.

Zwei Wege dorthin standen zur Verfuegung:

| | Prinzip | Was es voraussetzt |
|---|---|---|
| **Plattenbasiert** (`auto_camera_calibration.py`) | Die Kamera vermisst die bekannte Grundplatte (500 x 400 mm) und rechnet daraus ihre eigene Lage. | dass der Roboter so auf der Platte sitzt, wie das URDF es behauptet |
| **Hand-Auge** (`calibrate_handeye_auto.py`) | Ein ChArUco-Board am Greifer wird in vielen Stellungen beobachtet; aus Roboter- und Kamerabewegung folgt die Kameralage. | nur eine ausreichend vielfaeltige Stellungsmenge |

Die Hand-Auge-Methode ist die theoretisch sauberere: sie misst Kamera gegen
Roboter unmittelbar und braucht die Montageannahme nicht.

---

## 2. Was am 2026-09-09 geschah

Die Hand-Auge-Kalibrierung wurde durchgefuehrt: 16 selbst angefahrene Stellungen,
in 15 davon alle 10 Ecken erkannt, Drehungsunterschiede 15 bis 77 Grad, Kamera auf
1280x720. Vier Verfahren wurden gerechnet:

| Verfahren | x [m] | y [m] | z [m] | Abstand zu Park |
|---|---|---|---|---|
| **Park** (uebernommen) | +0.1250 | -0.1359 | +0.6190 | - |
| Horaud | +0.1248 | -0.1359 | +0.6190 | **0.1 mm** |
| Daniilidis | +0.1208 | -0.1327 | +0.6195 | 5.3 mm |
| Andreff | +0.0734 | -0.0934 | +0.3742 | **254 mm - verworfen** |

Park und Horaud stimmten auf einen Zehntelmillimeter ueberein. Das Ergebnis wich
allerdings **44 mm** von der bisherigen plattenbasierten Messung ab. Die Abweichung
wurde damals so erklaert: die plattenbasierte Methode nehme ungeprueft an, der
Roboter sitze genau wie im URDF auf der Platte - die 44 mm seien im Wesentlichen
der Fehler dieser Annahme. Die Hand-Auge-Werte wurden uebernommen.

**Diese Erklaerung war plausibel und falsch.**

---

## 3. Wie der Fehler auffiel

Am 2026-09-10 gelang es nicht, die Bereitstellungsschale im Tiefenbild zu finden.
Bei der Fehlersuche wurde die Grundplatte selbst vermessen - und zwar nicht als
Kalibriergegenstand, sondern als Pruefkoerper:

* Die Platte ist **ortsfest**, ihre Oberflaeche ist im URDF als `z = 0` definiert,
  und **der Roboter ist auf genau diese Platte geschraubt**.
* Er kann ihr gegenueber also nicht verkippt stehen. Kommt die Platte gekippt
  heraus, ist die Kalibrierung gekippt - nicht die Platte.

Gemessen wurde, durch die uebernommene Hand-Auge-Kalibrierung gerechnet:

| Groesse | Sollwert | Gemessen |
|---|---|---|
| Neigung der Plattenebene gegen z=0 | 0 Grad | **4.77 Grad** |
| Hoehe der Plattenflaeche | 0 mm | **+21.6 mm** |
| Abweichung bei x = 0.30 m | 0 mm | **+21.1 mm** |

Damit die Kamera nicht faelschlich beschuldigt wird, wurde die Ebene **zuerst im
Kamera-Frame** gefittet: dort ist sie auf **1.5 mm rms** eben, also einwandfrei.
Die Tiefendaten waren in Ordnung; die Umrechnung nach `robot_base` war es nicht.

### Der unabhaengige Gegenbeleg

Am selben 2026-09-09 war um **11:48 Uhr** mit `measure_model_vs_charuco.py` der
Modellfehler zu **3.28 mm / 0.42 Grad** bestimmt worden - die Hand-Auge-Werte
wurden erst um **12:50 Uhr** uebernommen. Jene Messung lief also noch mit der
**plattenbasierten** Kalibrierung. Das Verfahren schaetzt den unbekannten
Befestigungsversatz des Boards selbst und zieht ihn ab; ein *konstanter* Versatz
verschwindet damit, eine **Kameradrehung von 4.8 Grad aber nicht** - die haette
einen grossen stellungsabhaengigen Rest hinterlassen. Der Rest war 3.28 mm.

---

## 4. Warum es so lange unentdeckt blieb

Das ist der eigentlich lehrreiche Teil.

**(a) Der Fehlschlag war stumm.** Keine Fehlermeldung, kein Absturz, kein Warnhinweis.
Der Stack startete, die Punktwolke sah plausibel aus, RViz zeigte ein
zusammenhaengendes Bild. Eine falsche Zahl an einer Stelle, an der niemand nachfragt,
sieht genauso aus wie eine richtige.

**(b) Jede Methode wurde nur gegen sich selbst geprueft.** Die plattenbasierte
Kalibrierung macht die Platte per Konstruktion eben - sie kann sich an ihr nicht
blamieren. Die Hand-Auge-Kalibrierung prueft sich am Vergleich ihrer Verfahren
untereinander. Beide Pruefungen benutzen genau die Information, aus der die
Kalibrierung schon gerechnet wurde. **Eine Probe, die keine neue Information
heranzieht, kann nichts widerlegen.**

**(c) Die Uebereinstimmung zweier Verfahren wurde als Guete gelesen.** Park und
Horaud stimmten auf 0.1 mm ueberein - das wirkte ueberzeugend. Beide sind aber eng
verwandt: sie loesen zuerst die Drehung ueber dieselbe Formulierung. Der
aussagekraeftigere Befund stand daneben und wurde als Eigenart des Aufbaus
abgetan: **Tsai-Lenz und Andreff lagen 254 mm daneben.** Bei gut konditionierten
Daten stimmen alle Verfahren ueberein. Vier von fuenf Verfahren uneinig heisst
nicht "drei Verfahren taugen nichts", sondern "der Stellungssatz traegt die
Drehung nicht".

**(d) Die falsche Erklaerung war bequem.** Fuer den 44-mm-Sprung gab es sofort eine
Geschichte - die ungepruefte Montageannahme -, und diese Geschichte war fuer sich
genommen richtig: die Annahme WAR ungeprueft. Nur war sie nicht die Ursache. Eine
Erklaerung, die zum Befund passt, ist noch kein Beweis, dass sie zutrifft.

**(e) Der Folgefehler tarnte sich als anderes Problem.** Ein Grossteil des
Aufwands am 2026-09-10 ging in die Schalensuche, die immer wieder fehlschlug. Die
4.8-Grad-Kippung verzerrt ueber die 40 cm breite Platte die scheinbare Hoehe um
rund 35 mm - genug, um ein Verfahren, das einen 17 mm hohen Schalenrand sucht,
zuverlaessig zu ersticken. Die Kalibrierung wurde also nicht gefunden, weil man sie
suchte, sondern weil etwas anderes nicht funktionierte.

---

## 5. Die Loesung

**Zurueckgesetzt** auf die plattenbasierte Messung vom 2026-09-08
(xyz 0.168610 / -0.126149 / 0.609088, rpy 1.746009 / 1.547885 / -1.388759).
Die zurueckgenommenen Werte samt Begruendung liegen in
`calibration_backups/2026-09-10_handauge_zurueckgenommen/`.

Wichtiger als die Ruecknahme ist aber, dass der Fehler **kuenftig auffaellt**.
Dafuer gibt es jetzt zwei Proben, die sich gegenseitig ergaenzen:

### Probe 1 - Plattenprobe (`tools/pruefe_kalibrierung_platte.py`)

Prueft, ob die Grundplatte dort herauskommt, wo das Modell sie hat. Braucht nur die
laufende Kamera. Faengt genau den Fehlertyp ab, der hier auftrat.

*Was sie kann:* Neigung und Hoehe - also zwei der drei Drehfreiheiten und den
z-Versatz.
*Was sie nicht kann:* Drehung um die Hochachse und Verschiebung in der Ebene. Und
sie kann eine plattenbasierte Kalibrierung **nicht bestaetigen** - fuer die ist sie
nur eine Rechenkontrolle.

### Probe 2 - Board am Greifer (`tools/measure_model_vs_charuco.py`)

Prueft alle sechs Freiheitsgrade, weil das Board sich mit dem Roboter bewegt.
Das ist die eigentliche Abnahme.

### Ergebnis nach dem Zuruecksetzen

| Probe | Ergebnis | Bewertung |
|---|---|---|
| Plattenprobe, 3 Laeufe | 0.58 / 0.56 / 0.54 Grad, -1.7 / -1.6 / -1.6 mm | bestanden (Grenze 1.5 Grad, 8 mm) |
| Board am Greifer, volle Sicht | **2.52 mm / 0.89 Grad** (n=3) | deckt sich mit den 3.28 mm vom 2026-09-09 |

---

## 6. Was die Messtechnik selbst noch beigetragen hat

Der erste Durchlauf der Abnahme ergab **26.07 mm / 7.13 Grad** - scheinbar ein
vernichtendes Ergebnis. Es lag nicht an der Kalibrierung, sondern an zwei Maengeln
im Messwerkzeug. Beide sind lehrreich, weil sie dieselbe Form haben wie der
Hauptfehler: **eine Pruefung, die nicht prueft.**

1. **Die Toleranz wurde nicht durchgesetzt.** Die Nachfuehrung versucht bis zu
   sechs Mal, das Ziel auf 1 Grad genau anzufahren. Schaffte sie es nicht, wurde die
   Stellung trotzdem uebernommen - die Ausgabe druckte bedingungslos "ok". In jenem
   Lauf blieben zwei Stellungen 8.7 und 12.3 Grad daneben, und genau diese beiden
   lieferten die groessten "Modellfehler" (72 und 31 mm). Gemessen wurde der
   Ankunftsfehler der Servos, nicht das Modell.
2. **Es wurde nicht auf Stillstand gewartet.** Die TF wurde gelesen, sobald das
   Fahrkommando zurueckkam. Die Kamera zeigt aber ein Bild von vor rund 100 ms,
   die TF den neuesten Gelenkstand - faehrt der Arm noch, sind das zwei
   verschiedene Stellungen.
3. **Teilweise erkannte Boards wurden zugelassen** (`--min-ecken 6`). Gemessen:
   Stellungen mit 7 bis 8 Ecken ergaben 8.12 mm Rest, die mit allen 10 Ecken
   2.52 mm - dieselbe Stunde, dieselbe Kalibrierung. Wer weniger Ecken zulaesst,
   misst vor allem seine eigene Eckenausbeute.

Alle drei sind behoben (Toleranz wird erzwungen, Stillstand wird abgewartet,
`--min-ecken` steht auf 10).

---

## 7. Was offen bleibt

* **Der TCP-Versatz von rund 25 mm** (`GRASP_Z_OFFSET = -0.025`) ist damit *nicht*
  erklaert. Er wurde eingefuehrt, als die plattenbasierte Kalibrierung bereits in
  Kraft war; seine Ursache liegt anderswo - TCP-Definition, Fingergeometrie,
  Nachgeben unter Last oder der Modellfehler selbst.
* **Die Stichprobe der Abnahme ist mit n=3 duenn.** Fuer eine belastbare Zahl
  gehoert der Lauf mit mehr vollstaendig erkannten Stellungen wiederholt.
* **Die Servos erreichen ihre Ziele unzuverlaessig.** Im letzten Lauf verfehlten
  5 von 10 Stellungen die Toleranz, zwei davon um mehr als 15 Grad.

---

## 8. Die Lehre in einem Satz

Eine Kalibrierung ist nicht dann gut, wenn ihre Verfahren untereinander
uebereinstimmen, sondern wenn sie an einer Groesse geprueft wird, die **nicht in
ihre Rechnung eingegangen ist** - hier: an einer ortsfesten Platte, auf die der
Roboter geschraubt ist.
