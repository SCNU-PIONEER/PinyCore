@echo off
set curpath=D:\
:: 切换到当前脚本路径，后面下载文件默认到当前路径。
cd /d %curpath%
 
set exename=openocd.zip

set downurl=https://gitee.com/liudeyu233/download-openocd/raw/master/openocd.zip

:: 使用powershell附带的curl命令
powershell curl -o "%exename%" "%downurl%"

:: 设置解压目标路径
set targetdir=%curpath%openocd


:: 确保目标文件夹存在
if not exist "%targetdir%" (
    mkdir "%targetdir%"
)

echo 正在解压 %exename% 到 %targetdir%...
tar -xf "%exename%" -C "%targetdir%"

:: 检查解压结果
if exist "%targetdir%" (
    echo 解压完成。
) else (
    echo ❌ 解压失败，请检查文件路径。
)

:: 假设 openocd 可执行文件位于 bin 子目录中
set "openocd_bin=%curpath%openocd\OpenOCD-20240916-0.12.0\bin"

:: 检查路径是否已在 PATH 中
echo %PATH% | findstr /i "%openocd_bin%" >nul
if %ERRORLEVEL%==0 (
    echo PATH already includes openocd.
) else (
    echo Adding openocd to system PATH...
    setx PATH "%PATH%;%openocd_bin%" /M
)

pause