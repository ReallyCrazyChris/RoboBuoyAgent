@echo off
call ../env.bat

rem firmware for standard ESP32
esptool.py --chip esp32 --port %AMPY_PORT%  write_flash -z 0x1000 ./esp32-20220708-unstable-v1.19.1-113-g9af6a275d.bin

rem firmware for ESP32 with SPIRAM
rem esptool.py --chip esp32 --port %AMPY_PORT%  write_flash -z 0x1000 ./esp32spiram-20210902-v1.17.bin