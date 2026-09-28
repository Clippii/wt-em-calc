@echo off
setlocal
cd /d "%~dp0"
set PYTHONUTF8=1
set PYTHONHOME=
set PYTHONPATH=
set PYTHONNOUSERSITE=1
set "APP_PYTHON=.venv\Scripts\python.exe"
if exist "runtime\python.exe" set "APP_PYTHON=runtime\python.exe"
set OPENBLAS_NUM_THREADS=1
set OMP_NUM_THREADS=1
if not exist "%APP_PYTHON%" (
    echo Run Setup Windows.cmd first.
    pause
    exit /b 1
)
"%APP_PYTHON%" scripts\sync_game_data.py --offline-ok
if errorlevel 1 (
    pause
    exit /b 1
)
"%APP_PYTHON%" scripts\altitude_launch.py --open
if errorlevel 1 pause
