@echo off
setlocal EnableDelayedExpansion

rem ============================================================
rem  build.bat - Build STM32 project with CMake + Ninja
rem
rem  Usage:
rem    build.bat                 Build Debug preset
rem    build.bat Release         Build Release preset
rem    build.bat Debug clean     Delete build dir, then rebuild
rem    build.bat clean           Same, for Debug
rem ============================================================

cd /d "%~dp0"

rem ---- Parse arguments ---------------------------------------
set "PRESET=Debug"
set "DO_CLEAN=0"

for %%A in (%*) do (
    if /i "%%~A"=="clean"   ( set "DO_CLEAN=1" ) else (
    if /i "%%~A"=="Debug"   ( set "PRESET=Debug" ) else (
    if /i "%%~A"=="Release" ( set "PRESET=Release" ) else (
        echo [ERROR] Unknown argument: %%~A
        echo         Usage: build.bat [Debug^|Release] [clean]
        exit /b 1
    )))
)

set "BUILD_DIR=%~dp0build\%PRESET%"

rem ---- Locate the ARM toolchain ------------------------------
where arm-none-eabi-gcc >nul 2>&1
if errorlevel 1 (
    set "ARM_BIN="
    for /d %%D in ("%ProgramFiles(x86)%\Arm GNU Toolchain arm-none-eabi\*") do (
        if exist "%%~D\bin\arm-none-eabi-gcc.exe" set "ARM_BIN=%%~D\bin"
    )
    for /d %%D in ("%ProgramFiles%\Arm GNU Toolchain arm-none-eabi\*") do (
        if exist "%%~D\bin\arm-none-eabi-gcc.exe" set "ARM_BIN=%%~D\bin"
    )
    if not defined ARM_BIN (
        echo [ERROR] arm-none-eabi-gcc not found on PATH or in the default install dir.
        echo         Install the Arm GNU Toolchain or add its bin folder to PATH.
        exit /b 1
    )
    set "PATH=!ARM_BIN!;%PATH%"
    echo [INFO ] Using toolchain: !ARM_BIN!
)

rem ---- Check CMake and Ninja ---------------------------------
where cmake >nul 2>&1 || ( echo [ERROR] cmake not found on PATH. & exit /b 1 )
where ninja >nul 2>&1 || ( echo [ERROR] ninja not found on PATH. & exit /b 1 )

rem ---- Clean -------------------------------------------------
if "%DO_CLEAN%"=="1" (
    if exist "%BUILD_DIR%" (
        echo [INFO ] Removing "%BUILD_DIR%"
        rmdir /s /q "%BUILD_DIR%"
    )
)

rem ---- Configure (only if the cache is missing) --------------
if not exist "%BUILD_DIR%\CMakeCache.txt" (
    echo [INFO ] Configuring preset "%PRESET%" ...
    cmake --preset %PRESET%
    if errorlevel 1 (
        echo [ERROR] CMake configure failed.
        exit /b 1
    )
)

rem ---- Build -------------------------------------------------
echo [INFO ] Building preset "%PRESET%" ...
cmake --build --preset %PRESET%
if errorlevel 1 (
    echo [ERROR] Build failed.
    exit /b 1
)

rem ---- Post-build: hex + bin + size --------------------------
set "ELF=%BUILD_DIR%\STM32.elf"
if not exist "%ELF%" (
    echo [WARN ] %ELF% not found; skipping hex/bin generation.
    goto :done
)

arm-none-eabi-objcopy -O ihex   "%ELF%" "%BUILD_DIR%\STM32.hex"
arm-none-eabi-objcopy -O binary "%ELF%" "%BUILD_DIR%\STM32.bin"

echo.
arm-none-eabi-size "%ELF%"
echo.
echo [INFO ] Output: %BUILD_DIR%\STM32.elf ^| .hex ^| .bin

:done
echo [ OK  ] Build finished.
exit /b 0
