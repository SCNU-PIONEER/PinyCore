@echo off
set curpath=D:\
:: change the Installation path 
cd /d %curpath%
 
set exename=openocd.zip

set downurl=https://gitee.com/liudeyu233/download-openocd/raw/master/openocd.zip

:: use powershell curl cmd
powershell curl -o "%exename%" "%downurl%"

:: Set the decompression destination path
set targetdir=%curpath%openocd


:: Make sure the destination path folder exists
if not exist "%targetdir%" (
    mkdir "%targetdir%"
)

echo 正在解压 %exename% 到 %targetdir%...
tar -xf "%exename%" -C "%targetdir%"

:: detect decompression result
if exist "%targetdir%" (
    echo 解压完成。
) else (
    echo ❌ 解压失败，请检查文件路径。
)

:: environment variable path
set "openocd_bin=%curpath%openocd\OpenOCD-20240916-0.12.0\bin"

:: Check if the path is already in the PATH
echo %PATH% | findstr /i "%openocd_bin%" >nul
if %ERRORLEVEL%==0 (
    echo PATH already includes openocd.
) else (
    echo Adding openocd to system PATH...
    setx PATH "%PATH%;%openocd_bin%" /M
)

set exename=gcc-14.2.zip

set downurl=https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-mingw-w64-x86_64-arm-none-eabi.zip
:: use powershell curl cmd
powershell curl -o "%exename%" "%downurl%"

set targetdir=%curpath%gcc-14.2
:: Make sure the destination path folder exists
if not exist "%targetdir%" (
    mkdir "%targetdir%"
)

echo 正在解压 %exename% 到 %targetdir%...
tar -xf "%exename%" -C "%targetdir%"
:: detect decompression result
if exist "%targetdir%" (
    echo 解压完成。
) else (
    echo ❌ 解压失败，请检查文件路径。
)
:: environment variable path
set "gcc_bin=%curpath%gcc-14.2\bin"

echo %PATH% | findstr /i "%gcc_bin%" >nul
if %ERRORLEVEL%==0 (
    echo PATH already includes openocd.
) else (
    echo Adding openocd to system PATH...
    setx PATH "%PATH%;%gcc_bin%" /M
)

pause