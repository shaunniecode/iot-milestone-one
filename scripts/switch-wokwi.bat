@echo off
setlocal

set MODE=%1
if /I "%MODE%"=="rack" goto :run
if /I "%MODE%"=="room" goto :run

echo Usage: switch-wokwi.bat [rack^|room]
exit /b 1

:run
powershell -ExecutionPolicy Bypass -File "%~dp0\switch-wokwi.ps1" -Mode %MODE%
