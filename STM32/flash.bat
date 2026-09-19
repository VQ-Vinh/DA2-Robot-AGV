@echo off
setlocal EnableDelayedExpansion

rem ============================================================
rem  flash.bat - Flash the STM32F407 over ST-Link
rem
rem  Usage:
rem    flash.bat                  Flash the Debug build
rem    flash.bat Release          Flash the Release build
rem    flash.bat Debug build      Run build.bat first, then flash
rem
rem  Programmer: OpenOCD (preferred), ST-LINK_CLI as fallback.
rem  Override with:  set PROGRAMMER=openocd   or   set PROGRAMMER=stlink
rem ============================================================

cd /d "%~dp0"

rem ---- Parse arguments ---------------------------------------
set "PRESET=Debug"
set "DO_BUILD=0"

for %%A in (%*) do (
    if /i "%%~A"=="build"   ( set "DO_BUILD=1" ) else (
    if /i "%%~A"=="Debug"   ( set "PRESET=Debug" ) else (
    if /i "%%~A"=="Release" ( set "PRESET=Release" ) else (
        echo [ERROR] Unknown argument: %%~A
        echo         Usage: flash.bat [Debug^|Release] [build]
        exit /b 1
    )))
)

set "BUILD_DIR=%~dp0build\%PRESET%"
set "ELF=%BUILD_DIR%\STM32.elf"
set "HEX=%BUILD_DIR%\STM32.hex"

rem ---- Optionally build first --------------------------------
if "%DO_BUILD%"=="1" (
    call "%~dp0build.bat" %PRESET%
    if errorlevel 1 exit /b 1
)

if not exist "%ELF%" (
    echo [ERROR] %ELF% not found. Run:  build.bat %PRESET%
    exit /b 1
)

rem ---- Locate OpenOCD ----------------------------------------
if not defined OPENOCD (
    where openocd >nul 2>&1 && set "OPENOCD=openocd"
)
if not defined OPENOCD (
    for /d %%D in ("%LOCALAPPDATA%\Microsoft\WinGet\Packages\xpack-dev-tools.openocd-xpack*") do (
        for /d %%E in ("%%~D\xpack-openocd-*") do (
            if exist "%%~E\bin\openocd.exe" set "OPENOCD=%%~E\bin\openocd.exe"
        )
    )
)

rem ---- Locate ST-LINK_CLI (fallback) -------------------------
if not defined STLINK_CLI (
    set "STLINK_CLI=%ProgramFiles(x86)%\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe"
)

rem ---- Pick a programmer -------------------------------------
if /i "%PROGRAMMER%"=="stlink" goto :use_stlink
if /i "%PROGRAMMER%"=="openocd" (
    if not defined OPENOCD (
        echo [ERROR] PROGRAMMER=openocd but openocd was not found.
        exit /b 1
    )
    goto :use_openocd
)
if defined OPENOCD goto :use_openocd
goto :use_stlink

rem ------------------------------------------------------------
:use_openocd
echo [INFO ] Flashing with OpenOCD: "%OPENOCD%"
echo [INFO ] Image: %ELF%
"%OPENOCD%" -f interface/stlink.cfg -f target/stm32f4x.cfg ^
    -c "program \"%ELF:\=/%\" verify reset exit"
if errorlevel 1 (
    echo [ERROR] OpenOCD flashing failed.
    echo         Check the ST-Link cable, board power and the ST-Link driver.
    exit /b 1
)
goto :ok

rem ------------------------------------------------------------
:use_stlink
if not exist "%STLINK_CLI%" (
    echo [ERROR] No programmer found.
    echo         Install OpenOCD ^(on PATH^) or the STM32 ST-LINK Utility.
    exit /b 1
)
if not exist "%HEX%" (
    echo [ERROR] %HEX% not found. Run:  build.bat %PRESET%
    exit /b 1
)
echo [INFO ] Flashing with ST-LINK_CLI
echo [INFO ] Image: %HEX%
"%STLINK_CLI%" -c SWD UR -P "%HEX%" -V -Rst
if errorlevel 1 (
    echo [ERROR] ST-LINK_CLI flashing failed.
    exit /b 1
)
goto :ok

rem ------------------------------------------------------------
:ok
echo [ OK  ] Flash finished, target reset.
exit /b 0
