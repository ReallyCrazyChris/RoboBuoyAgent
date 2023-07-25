echo on
call ./env.bat

echo removing main 
ampy rm /main.py

echo main
ampy put ./src/main.py /main.py

echo starting serial console
call ./console.bat