sudo mkdir -p /mnt/rpi_pico
sudo mount -o rw,uid=$(id -u),gid=$(id -g) -L RPI-RP2 /mnt/rpi_pico
