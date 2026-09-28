# Welle aus dem Trichter holen — funktionierendes Rezept

Erstmals erfolgreich am **2026-09-10**, dritter Versuch, vom Benutzer bestaetigt:
"aldi basariyla mili" (er hat die Welle erfolgreich genommen).

## Die Schritte, in dieser Reihenfolge

| # | Was | Wie |
|---|---|---|
| 1 | Nullstellung | alle Gelenke 0, ~16 s |
| 2 | Greifer auf | `gripper_controller = 0.15` (100 %) |
| 3 | ueber den Trichter | **mit IK**, `zeige_punkt.py --x 94.4 --y -164.9 --z 115 --yaw 180` |
| 4 | auf den Greifpunkt | **mit IK**, dieselbe x/y, `--z 8.2` |
| 5 | 6 s warten | die Servos laufen nach |
| 6 | Greifer zu | `gripper_controller = -0.62`, ueber **10 s** |
| 7 | senkrecht heraus | `hebe_und_parke.py --hoehe 124 --dauer 14` — **EIN** Aufruf |

## Die Zahlen, die zaehlen

* **Greifpunkt**: der angelernte Punkt `trichter_greif2` **plus 5 mm in z**
  (angelernt z=3.2, angefahren z=8.2). Tiefer greift der Greifer unterhalb des
  Ø-7-Kopfes in den Ø-10-Ringbereich — im 2. Versuch ging die Welle deshalb
  verloren.
* **Gierwinkel 180 Grad** festnageln (`--yaw 180`). Ohne das sucht sich die IK
  eine andere Handstellung und der Greifer dreht sich gegen den Trichter.
* **Greifer -0.62** (Firmware meldet 13 %, rund 4.2 mm Backenabstand). Bei -0.56
  (19-20 %) rutschte die Welle im 2. Versuch heraus. Der Benutzer wollte es
  ausdruecklich fester: "wenn sie am Ringbereich abrutscht, soll sie wenigstens
  am schmalen Kopf haengenbleiben."

## Umrechnung Greifer (aus der Messung 6.8 mm bei 21 %)

    Backenabstand [mm] -> Firmware [%] -> Gelenkwert
      7.0                    22            -0.548     nur Beruehrung
      6.0                    19            -0.575     haelt
      5.0                    15            -0.603     fest
      4.2                    13            -0.620     BEWAEHRT

## Was NICHT funktioniert

* **Mehrere kleine Hebe-Aufrufe hintereinander.** Jeder liest die gemessene Lage
  neu und nimmt das Durchhaengen mit — nach vier Aufrufen war der TCP 25 mm zur
  Seite gewandert. Ein einziger Aufruf auf die Endhoehe: 5.5 mm.
* **Seitliches Verschieben in Trichternaehe.** Der Trichter steht nicht im
  Kollisionsmodell; bei einem solchen Versuch verbog sich der Greifer.
* **`cartesian_jog.py`** — dreht den Greifer bei jedem Klick.

## Wiederholbarkeit (drei Durchlaeufe)

| Durchlauf | erreichter TCP | Greifer | seitliche Wanderung | Ergebnis |
|---|---|---|---|---|
| 1 | (91, -156, 4) | 19 % | 7.0 mm | Welle geholt |
| 2 | (87, -154, 0) | 20 % | 6.0 mm | im letzten Moment verloren |
| 3 | (88, -157, 3) | **13 %** | **5.5 mm** | **Welle geholt** |

Die Streuung des Greifpunkts betraegt rund 4 mm — das ist das Spiel in Achse 2,
vom Hersteller als bauartbedingt bestaetigt.

## Noch offen

* Trichter ins Kollisionsmodell (`trichter.xacro`) — Voraussetzung fuer alles,
  was seitlich am Trichter vorbei muss
* Greifer ist verbogen (2026-09-10): Fingerachse rund 5 Grad aus der Senkrechten
  laut Modell, sichtbar mehr in Wirklichkeit. Er greift schraeg, es funktioniert
  aber.
* Wie oft haelt es hintereinander? Bisher 2 von 3.
