"""Wellen-Erkennung — Mittelpunkt + Winkel + Farbe aus YOLO11-seg Maske.

Portiert aus inference_template.py des Vorgaengerprojekts.
Template-Mask-Refinement entfernt: für den Roboter genügen der Masken-Centroid
(PCA-Zentrum) und der Langachsen-Winkel. Dieses Modul ist ROS-unabhängig — reines numpy/cv2/ultralytics.
"""
import math

import numpy as np

# Exakt in der in best.pt eingebetteten Reihenfolge (mit model.names verifiziert):
CLASS_NAMES = ["gelb", "weiss", "schwarz", "gruen", "rot"]

# Physische Wellen-Abmessung (für mm/px Skalenschätzung)
WELLEN_LAENGE_MM = 44.0


def get_mask_orientation(mask_bool):
    """Mittelpunkt + Langachsen-Winkel via PCA (Hauptkomponentenanalyse) aus Binär-Maske.

    MATHEMATIK — 2D-PCA auf den Maskenpixeln:
      Die Menge der zur Maske gehörenden Pixel (x_i, y_i) wird als 2D-Punktwolke
      aufgefasst. Der Schwerpunkt (Centroid) (cx, cy) = Mittelwert aller Pixel ist der
      Wellen-Mittelpunkt. Nach Zentrierung (Abzug des Centroids) beschreibt die
      2x2-Kovarianzmatrix cov die räumliche Streuung der Punktwolke. Ihre Eigenwert-
      zerlegung (eigh, da cov symmetrisch) liefert zwei orthogonale Eigenvektoren:
        - der Eigenvektor zum GROESSTEN Eigenwert = Richtung der größten Streuung
          = LANGACHSE der Welle (Schaftrichtung),
        - der zum kleineren Eigenwert = KURZACHSE (Breite/Dicke).
      Der Winkel der Langachse folgt aus atan2(major_y, major_x) im Bildkoordinaten-
      system (x nach rechts, y nach UNTEN — daher ist der Winkel im Uhrzeigersinn positiv).

    Returns (cx, cy, angle_deg, major_len_px, minor_len_px) oder None.
    angle_deg: Winkel der Langachse in Bildkoordinaten (x rechts, y unten).
    """
    ys, xs = np.where(mask_bool)
    if len(xs) < 10:
        return None  # zu wenige Pixel für eine stabile Kovarianzschätzung

    # Schwerpunkt (Centroid) der Punktwolke = Wellen-Mittelpunkt in px
    cx = float(np.mean(xs))
    cy = float(np.mean(ys))

    # Zentrierte Koordinaten (Nx2) und 2x2-Kovarianzmatrix der Punktwolke
    coords = np.column_stack([xs - cx, ys - cy]).astype(np.float64)
    cov = np.cov(coords.T)
    # Eigenwertzerlegung der symmetrischen Kovarianz -> Hauptachsen der Streuung
    eigvals, eigvecs = np.linalg.eigh(cov)

    # Größter Eigenwert -> zugehöriger Eigenvektor = Langachse (Schaftrichtung)
    idx = int(np.argmax(eigvals))
    major = eigvecs[:, idx]
    angle = math.degrees(math.atan2(major[1], major[0]))

    # Achsenlängen ~ 2*Standardabweichung (sqrt Eigenwert), zusätzlich *2 als
    # Gesamtausdehnung (Näherung für die sichtbare Länge/Breite in px).
    major_len = 2.0 * math.sqrt(max(eigvals[idx], 0.0)) * 2.0
    minor_len = 2.0 * math.sqrt(max(eigvals[1 - idx], 0.0)) * 2.0
    return cx, cy, angle, major_len, minor_len


def estimate_scale(mask_bool, angle_deg):
    """mm/px Schätzung aus Maskenlänge (px) bezogen auf physische 44 mm.

    MATHEMATIK — Projektion auf die Langachse:
      Jeder Maskenpixel wird (nach Zentrierung um den Centroid) per Skalarprodukt
      mit dem Einheitsvektor (cos a, sin a) der Langachse auf diese projiziert:
        rx_i = (x_i - cx)*cos(a) + (y_i - cy)*sin(a).
      rx ist die vorzeichenbehaftete Koordinate ENTLANG der Wellenachse. Die
      Ausdehnung max(rx) - min(rx) ist die sichtbare Wellenlänge in Pixeln.
      Aus der bekannten physischen Länge (44 mm) folgt der Maßstab mm/px = 44 / length_px.
      (Fallback 0.1 mm/px bei zu kurzer/ungültiger Maske.)
    """
    ys, xs = np.where(mask_bool)
    if len(xs) < 10:
        return 0.1
    cx, cy = np.mean(xs), np.mean(ys)
    a = math.radians(angle_deg)
    cos_a, sin_a = math.cos(a), math.sin(a)
    # Projektion der zentrierten Pixel auf die Langachsenrichtung (Skalarprodukt)
    rx = (xs - cx) * cos_a + (ys - cy) * sin_a
    length_px = rx.max() - rx.min()   # Ausdehnung entlang der Achse = Länge in px
    if length_px < 5:
        return 0.1
    return float(WELLEN_LAENGE_MM / length_px)


def kopf_bewusster_griff(mask_bool, cx, cy, angle_deg, inset_frac=0.30, min_ratio=1.15):
    """Finde den Wellen-KOPF aus der Maskenform und verschiebe den Greifpunkt auf den Schaft unter dem Kopf.

    Der Wellenkopf ist breiter als der Schaft: vergleicht die Breite an beiden
    Enden entlang der Langachse (25%-Segment vom Ende); das breitere Ende = Kopf.
    Der Greifpunkt wird vom Kopf-Ende um inset_frac*L Richtung Spitze nach innen
    verschoben (direkt unter dem Kopf, gerader/paralleler Schaft = sicherster
    Parallel-Backen-Griff). Liegt das Kopf-Spitze-Breitenverhältnis unter min_ratio
    (unklar), wird der Mittelpunkt (Centroid) zurückgegeben und head_known=False.

    angle_deg = Langachsen-Winkel aus get_mask_orientation (x rechts, y unten).
    Returns dict: grip_px=(gx,gy), head_px=(hx,hy)|None, head_known(bool), ratio(float).
    """
    fallback = {"grip_px": (cx, cy), "head_px": None, "head_known": False, "ratio": 1.0}
    ys, xs = np.where(mask_bool)
    if len(xs) < 20:
        return fallback
    # Lokales, an der Welle ausgerichtetes Koordinatensystem: die zentrierten
    # Pixel werden per Skalarprodukt in "major" (entlang der Langachse) und "minor"
    # (senkrecht dazu = Breitenrichtung) zerlegt — eine 2D-Rotation um den Winkel a.
    a = math.radians(angle_deg)
    ca, sa = math.cos(a), math.sin(a)
    major = (xs - cx) * ca + (ys - cy) * sa       # vorzeichenbehaftet entlang der Langachse
    minor = -(xs - cx) * sa + (ys - cy) * ca      # senkrecht zur Langachse (Breite)
    t_min, t_max = float(major.min()), float(major.max())
    length = t_max - t_min                         # Gesamtlänge entlang der Achse (px)
    if length < 5:
        return fallback

    # Die beiden End-Segmente (je 25% der Länge) an den Achsenenden auswählen
    seg = 0.25 * length
    hi = major >= (t_max - seg)   # Segment am oberen Ende (großes major)
    lo = major <= (t_min + seg)   # Segment am unteren Ende (kleines major)

    def _width(sel):
        # Breite eines End-Segments = robustes Perzentil-Intervall (90%-10%) der
        # Kurzachsen-Koordinate; ausreißerrobust gegenüber Maskenrand-Rauschen.
        if np.count_nonzero(sel) < 5:
            return 0.0
        mv = minor[sel]
        return float(np.percentile(mv, 90) - np.percentile(mv, 10))

    w_hi, w_lo = _width(hi), _width(lo)
    if max(w_hi, w_lo) < 1e-6:
        return fallback
    # Verhältnis der breiteren zur schmaleren Seite. Kopf (breit) vs. Schaft (schmal).
    ratio = max(w_hi, w_lo) / max(min(w_hi, w_lo), 1e-6)
    if ratio < min_ratio:
        # Zu geringer Breitenunterschied -> Kopf unklar -> Centroid als Rückfall
        out = dict(fallback)
        out["ratio"] = ratio
        return out

    # Das breitere Ende ist der Kopf. Der Greifpunkt wird vom Kopf-Ende um
    # inset_frac*L nach INNEN (Richtung Spitze) verschoben -> gerader Schaft.
    head_at_max = w_hi >= w_lo
    t_head = t_max if head_at_max else t_min
    t_grip = t_head - inset_frac * length if head_at_max else t_head + inset_frac * length
    return {
        "grip_px": (cx + t_grip * ca, cy + t_grip * sa),
        "head_px": (cx + t_head * ca, cy + t_head * sa),
        "head_known": True,
        "ratio": ratio,
    }


def wellen_erkennen(bgr_img, model, conf=0.70):
    """Erkenne Wellen in einem BGR-Frame mit YOLO11-seg.

    Returns list[dict]: class_id, class_name, conf, center_px (cx,cy),
    angle_deg, bbox (x1,y1,x2,y2), mask (HxW uint8 0/255), mm_per_px.
    Absteigend nach Confidence sortiert.
    """
    results = model.predict(bgr_img, conf=conf, retina_masks=True, verbose=False)[0]
    out = []
    if results.masks is None:
        return out

    masks = results.masks.data.cpu().numpy()
    boxes = results.boxes
    h, w = bgr_img.shape[:2]

    import cv2  # lazy: in manchen Umgebungen Import-Kosten

    for i in range(len(masks)):
        cls_id = int(boxes.cls[i].item())
        conf_val = float(boxes.conf[i].item())
        x1, y1, x2, y2 = [float(v) for v in boxes.xyxy[i].cpu().numpy().tolist()]

        m = masks[i]
        if m.shape[:2] != (h, w):
            m = cv2.resize(m, (w, h))
        m_bool = m > 0.5

        orient = get_mask_orientation(m_bool)
        if orient is None:
            continue
        cx, cy, angle, major_len, minor_len = orient
        mm_per_px = estimate_scale(m_bool, angle)

        out.append({
            "class_id": cls_id,
            "class_name": CLASS_NAMES[cls_id] if cls_id < len(CLASS_NAMES) else f"cls_{cls_id}",
            "conf": conf_val,
            "center_px": (cx, cy),
            "angle_deg": float(angle),
            "bbox": (x1, y1, x2, y2),
            "mask": (m_bool * 255).astype(np.uint8),
            "major_len_px": float(major_len),
            "minor_len_px": float(minor_len),
            "mm_per_px": mm_per_px,
        })

    out.sort(key=lambda d: d["conf"], reverse=True)
    return out
