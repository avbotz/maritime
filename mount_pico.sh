sudo mkdir -p /mnt/rpi_pico
sudo mount -o rw,uid=$(id -u),gid=$(id -g) /dev/sd* /mnt/rpi_pico
