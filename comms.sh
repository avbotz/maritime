# Establish serial communication between pc and microcontroller
sudo stty -F /dev/ttyACM* 9600 raw -clocal -echo