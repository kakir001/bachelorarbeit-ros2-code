#!/usr/bin/env bash
# Offline-Demo unter Linux/macOS: virtuelle Umgebung anlegen, Pakete installieren, beide Demos starten.
set -e
cd "$(dirname "$0")"
PY="${PYTHON:-python3}"
if [ ! -x .venv/bin/python ]; then "$PY" -m venv .venv; fi
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements-offline.txt
.venv/bin/python wellenerkennung_offline.py
.venv/bin/python yolo_offline.py
echo
echo "Fertig. Ergebnisbilder: $(pwd)/ausgabe"
