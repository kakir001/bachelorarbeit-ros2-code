@echo off
rem Offline-Demo unter Windows: virtuelle Umgebung anlegen, Pakete installieren, beide Demos starten.
rem Voraussetzung: Python 3.9 bis 3.12 von https://www.python.org (bei der Installation "Add python.exe to PATH" ankreuzen).
setlocal
cd /d "%~dp0"
chcp 65001 >nul
set PYTHONUTF8=1

set "PY=python"
where py >nul 2>nul && set "PY=py -3"
%PY% --version >nul 2>nul
if errorlevel 1 (
    echo Python wurde nicht gefunden. Bitte Python 3.9 bis 3.12 von https://www.python.org installieren
    echo und dabei "Add python.exe to PATH" ankreuzen. Danach diese Datei erneut starten.
    goto ende
)

if not exist ".venv\Scripts\python.exe" (
    echo [1/4] Lege virtuelle Umgebung an ...
    %PY% -m venv .venv
    if errorlevel 1 goto fehler
)
echo [2/4] Installiere Pakete (beim ersten Mal einige Minuten, ca. 1 GB) ...
".venv\Scripts\python.exe" -m pip install --upgrade pip
".venv\Scripts\python.exe" -m pip install -r requirements-offline.txt
if errorlevel 1 goto fehler

echo.
echo [3/4] Demo 1: Wellen- und Kopfseitenerkennung (klassische Bildverarbeitung, ohne KI) ...
".venv\Scripts\python.exe" wellenerkennung_offline.py
if errorlevel 1 goto fehler

echo.
echo [4/4] Demo 2: YOLO11l-seg-Instanzsegmentierung ...
".venv\Scripts\python.exe" yolo_offline.py
if errorlevel 1 goto fehler

echo.
echo Fertig. Die Ergebnisbilder liegen im Ordner "ausgabe" und werden jetzt geoeffnet.
start "" "ausgabe"
goto ende

:fehler
echo.
echo FEHLER - siehe Meldung oben. Hinweise stehen in README.md in diesem Ordner.

:ende
echo.
pause
endlocal
