echo on
call ./env.bat

echo removing main, lib
ampy rm /main.py
ampy rmdir /lib

echo uploading lib
ampy put ./src/lib

echo uploading config main 
ampy put ./src/config.py /config.py
ampy put ./src/main.py /main.py

echo starting serial console
call ./console.bat



