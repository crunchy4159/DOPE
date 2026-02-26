@echo off
setlocal
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\run_native_gui.ps1" %*
exit /b %ERRORLEVEL%
