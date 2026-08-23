@echo off
REM ---- 5-Bar Digital Twin launcher (Windows) ----
where python >nul 2>nul
if errorlevel 1 (
  echo Python not found on PATH. Install Python 3.10+ from python.org and re-run.
  pause & exit /b 1
)
python app.py
if errorlevel 1 pause
