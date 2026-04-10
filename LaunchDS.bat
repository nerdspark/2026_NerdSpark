@echo off
cd /d "%~dp0"

start "" "C:\Program Files (x86)\FRC Driver Station\DriverStation.exe"

if not exist "%~dp0PassTargetPicker.jar" (
    echo ERROR: PassTargetPicker.jar not found in %~dp0
    pause
    exit /b 1
)

if not exist "C:\Users\Public\wpilib\2026\jdk\bin\java.exe" (
    echo ERROR: Java not found at C:\Users\Public\wpilib\2026\jdk\bin\java.exe
    pause
    exit /b 1
)

set PATH=%~dp0;%PATH%
"C:\Users\Public\wpilib\2026\jdk\bin\java.exe" -Djava.library.path=%~dp0 -jar "%~dp0PassTargetPicker.jar"
pause
