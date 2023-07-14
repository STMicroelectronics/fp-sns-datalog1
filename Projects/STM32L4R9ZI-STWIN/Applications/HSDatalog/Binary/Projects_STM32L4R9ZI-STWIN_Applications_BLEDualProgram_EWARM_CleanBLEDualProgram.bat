@echo off
set STLINK_PATH="C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\"
set BINARY_NAME=".\HSDatalog"
color 0F
echo                /******************************************/
echo                            Full Chip Erase
echo                /******************************************/
%STLINK_PATH%ST-LINK_CLI.exe -c UR -HardRst -ME
echo                /******************************************/
echo                          Set Boot from Bank 1
echo                /******************************************/
%STLINK_PATH%ST-LINK_CLI.exe -oB BFB2=0
echo                /******************************************/
echo                          Install BLEDualProgram
echo                /******************************************/
%STLINK_PATH%ST-LINK_CLI.exe -P %BINARY_NAME%.bin 0x08000000 -V "after_programming"
echo                /******************************************/
echo                                 Reset STM32
echo                /******************************************/
%STLINK_PATH%ST-LINK_CLI.exe -c UR -Rst
if NOT "%1" == "SILENT" pause