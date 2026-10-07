@echo off
setlocal EnableExtensions
pushd "%~dp0"

REM Admin check (UAC)
net session >nul 2>&1
if errorlevel 1 (
  echo [!] Admin privileges required. Relaunching...
  powershell -NoProfile -Command "Start-Process '%~f0' -Verb RunAs"
  exit /b
)

REM Locate RegAsm
set "REGASM="
for %%P in (
  "%SystemRoot%\Microsoft.NET\Framework64\v4.0.30319\RegAsm.exe"
  "%SystemRoot%\Microsoft.NET\Framework\v4.0.30319\RegAsm.exe"
  "%SystemRoot%\Microsoft.NET\Framework64\v2.0.50727\RegAsm.exe"
  "%SystemRoot%\Microsoft.NET\Framework\v2.0.50727\RegAsm.exe"
) do (
  if not defined REGASM if exist "%%~P" set "REGASM=%%~P"
)
if not defined REGASM (
  echo [ERROR] RegAsm.exe not found.
  pause
  popd
  exit /b 1
)

REM Locate target DLL
set "DLL="
if exist "%~dp0SWAnkleInterference.dll" set "DLL=%~dp0SWAnkleInterference.dll"
if not defined DLL if exist "%~dp0bin\Release\SWAnkleInterference.dll" set "DLL=%~dp0bin\Release\SWAnkleInterference.dll"
if not defined DLL if exist "%~dp0bin\Debug\SWAnkleInterference.dll" set "DLL=%~dp0bin\Debug\SWAnkleInterference.dll"
if not defined DLL (
  echo [ERROR] SWAnkleInterference.dll not found. Build the project first.
  pause
  popd
  exit /b 1
)

REM Freshness check: block registration if DLL is older than source files
echo [CHECK] Verifying DLL freshness...
powershell -NoProfile -Command "$dll = Get-Item -LiteralPath '%DLL%'; $root = Resolve-Path '%~dp0'; $src = Get-ChildItem -LiteralPath $root -Recurse -File | Where-Object { $_.Extension -in '.cs','.csproj' }; if(-not $src){ exit 0 }; $newest = $src | Sort-Object LastWriteTime -Descending | Select-Object -First 1; if($dll.LastWriteTime -lt $newest.LastWriteTime){ Write-Host ('[STALE] DLL time:  ' + $dll.LastWriteTime.ToString('yyyy-MM-dd HH:mm:ss')); Write-Host ('[STALE] Source time:' + $newest.LastWriteTime.ToString('yyyy-MM-dd HH:mm:ss')); Write-Host ('[STALE] Newest source: ' + $newest.FullName); exit 2 } else { Write-Host ('[OK] DLL is up-to-date: ' + $dll.LastWriteTime.ToString('yyyy-MM-dd HH:mm:ss')); exit 0 }"
if errorlevel 2 (
  echo [ERROR] DLL is older than source files. Please rebuild before registering.
  pause
  popd
  exit /b 1
)

echo [INFO] RegAsm: "%REGASM%"
echo [INFO] Target: "%DLL%"
echo.

echo [1/2] Unregister...
"%REGASM%" "%DLL%" /unregister /codebase >nul 2>&1

echo [2/2] Register...
echo [cmd] "%REGASM%" "%DLL%" /codebase /tlb
"%REGASM%" "%DLL%" /codebase /tlb
if errorlevel 1 (
  echo [ERROR] Registration failed.
  pause
  popd
  exit /b 1
)

echo [OK] Registration succeeded. Restart SolidWorks.
pause
popd
