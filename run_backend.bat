@echo off
echo ========================================================
echo   NAVIS - Starting Backend Simulation Server (FastAPI)
echo ========================================================
cd /d "%~dp0"
set "PYTHONPATH=%~dp0;%PYTHONPATH%"
python -m uvicorn backend.main:app --host 0.0.0.0 --port 8000 --reload
pause
