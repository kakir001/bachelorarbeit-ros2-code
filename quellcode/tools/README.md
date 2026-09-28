# tools/

## umlaut_convert.py
Einmaliges Migrations-Werkzeug (2026-07-15): wandelt ASCII-Deutsch (fuer/ueber/laedt)
in echte Umlaute (fuer->für usw.) um — **nur in Kommentaren und Docstrings**.
String-Literale (cv2-UI, Logs, grep-Vertrag) bleiben per Python-tokenize unberührt ASCII.

    python3 tools/umlaut_convert.py           # Dry-Run (zeigt Änderungen)
    python3 tools/umlaut_convert.py --apply    # wendet an

ss->ß nur ueber die handverifizierte FORCE-Tabelle im Skript. Bei neuen Woertern
mit ss/ae/oe/ue erst Dry-Run pruefen. Details: DEVLOG.md (2026-07-15).
