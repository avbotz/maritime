# Establish serial communication between pc and microcontroller
sudo stty -F /dev/ttyACM* 115200 raw -clocal -echo
sudo chmod 666 /dev/ttyACM*