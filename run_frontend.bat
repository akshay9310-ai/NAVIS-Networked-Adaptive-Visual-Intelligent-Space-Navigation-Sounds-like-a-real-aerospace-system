@echo off
echo ========================================================
echo   NAVIS - Starting React/Vite Frontend Dev Server
echo ========================================================
cd /d "%~dp0frontend"
if exist "C:\Program Files\nodejs\npm.cmd" (
    call "C:\Program Files\nodejs\npm.cmd" run dev
) else (
    call npm run dev
)
pause
