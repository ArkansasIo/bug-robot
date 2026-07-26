@echo off
REM ============================================================================
REM centipede_robot/build/build.bat
REM Windows Build Script for Centipede Robot Firmware
REM ============================================================================

setlocal enabledelayedexpansion

REM Check for NASM
where nasm >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo ERROR: NASM not found! Please install NASM from https://www.nasm.us/
    echo Alternatively, run: winget install NASM.NASM
    pause
    exit /b 1
)

REM Check for MinGW or other GCC/LD
where ld >nul 2>&1
if %ERRORLEVEL% neq 0 (
    where i686-elf-ld >nul 2>&1
    if !ERRORLEVEL! neq 0 (
        echo WARNING: Standard ld not found. Trying MinGW...
        where gcc >nul 2>&1
        if !ERRORLEVEL! neq 0 (
            echo ERROR: No linker found! Install MinGW or a cross-compiler.
            pause
            exit /b 1
        )
    )
)

REM Set toolchain
set ASM=nasm
if not "%1"=="" (
    if "%1"=="-m" (
        set LD=i686-elf-ld
    ) else (
        set LD=ld
    )
) else (
    set LD=ld
)

REM Directories
set SRC_DIR=..\src
set INC_DIRS=-I..\inc -I..\config
set BUILD_DIR=.
set LINKER_DIR=..\linker

echo.
echo ========================================
echo Centipede Robot Firmware Builder
echo ========================================
echo.

echo Assembling source files...

REM Assemble all .asm files
set OBJS=
for %%F in (%SRC_DIR%\*.asm) do (
    echo   Assembling %%~nxF...
    %ASM% -f elf32 -g %INC_DIRS% -o "%BUILD_DIR%\%%~nF.o" "%%F"
    if !ERRORLEVEL! neq 0 (
        echo ERROR: Failed to assemble %%~nxF!
        pause
        exit /b 1
    )
    set OBJS=!OBJS! "%BUILD_DIR%\%%~nF.o"
)

echo.
echo Linking...
%LD% -T %LINKER_DIR%\link.ld -m elf_i386 -o centipede_robot.elf %OBJS%
if %ERRORLEVEL% neq 0 (
    echo ERROR: Linking failed!
    pause
    exit /b 1
)

echo.
echo Generating binary...
where objcopy >nul 2>&1
if %ERRORLEVEL% equ 0 (
    objcopy -O binary centipede_robot.elf centipede_robot.bin
    echo Binary: centipede_robot.bin
) else (
    echo WARNING: objcopy not found, skipping binary generation.
)

echo.
echo ========================================
echo Build SUCCESSFUL!
echo ========================================

REM Show sizes
echo.
echo File sizes:
if exist centipede_robot.elf (
    where size >nul 2>&1
    if !ERRORLEVEL! equ 0 (
        size -A -x centipede_robot.elf
    ) else (
        for %%F in (centipede_robot.elf) do echo   ELF: %%~zF bytes
    )
)
if exist centipede_robot.bin (
    for %%F in (centipede_robot.bin) do echo   BIN: %%~zF bytes
)

echo.
pause

