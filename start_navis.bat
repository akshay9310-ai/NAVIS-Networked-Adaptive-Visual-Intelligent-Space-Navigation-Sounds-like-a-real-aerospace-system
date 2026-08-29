@echo off
title NAVIS Mission Control
echo =========================================================================
echo    NAVIS - Networked Adaptive Visual & Intelligent Space Navigation
echo            Adaptive Intelligence for Navigation Beyond Earth
echo =========================================================================
echo.
echo [1/2] Starting NAVIS FastAPI Backend & Physics Engine on port 8000...
cd /d "%~dp0"
set "PYTHONPATH=%~dp0;%PYTHONPATH%"
start "NAVIS Backend Server" cmd /k "cd /d ""%~dp0"" && set PYTHONPATH=. && python -m uvicorn backend.main:app --host 0.0.0.0 --port 8000"

timeout /t 3 /nobreak >nul

echo [2/2] Launching NAVIS Mission Control Dashboard in your browser...
start http://localhost:8000/

echo.
echo =========================================================================
echo   NAVIS is now LIVE at http://localhost:8000/
echo   API Docs: http://localhost:8000/docs
echo =========================================================================
pause
