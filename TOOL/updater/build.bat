@echo off
setlocal
py -m pip install -r requirements.txt
py -m pip install pyinstaller
py -m pyinstaller --onefile --noconsole --name SYUBEA_Update --collect-all esptool update.pyw

echo.
echo Build complete.
echo EXE: .\dist\SYUBEA_Update.exe
pause
