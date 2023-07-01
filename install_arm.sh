#!/bin/bash

# For installing dependencies (https://docs.zephyrproject.org/latest/develop/getting_started/index.html)

maritime_dir=$(pwd)

sudo apt update
sudo apt upgrade

wget https://apt.kitware.com/kitware-archive.sh
sudo bash kitware-archive.sh

sudo apt install --no-install-recommends git cmake ninja-build gperf \
  ccache dfu-util device-tree-compiler wget \
  python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
  make gcc libsdl2-dev libmagic1 stlink-tools

# Add the maritime_ec library
git clone https://github.com/avbotz/maritime_ec.git

pip3 install --user -U west
echo 'export PATH=~/.local/bin:"$PATH"' >> ~/.bashrc
source ~/.bashrc

west init ~/zephyrproject
cd ~/zephyrproject
west update

west zephyr-export

pip3 install --user -r ~/zephyrproject/zephyr/scripts/requirements.txt

cd ~
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.1/zephyr-sdk-0.16.1_linux-aarch64.tar.xz
wget -O - https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.1/sha256.sum | shasum --check --ignore-missing
tar xvf zephyr-sdk-0.16.1_linux-aarch64.tar.xz

cd zephyr-sdk-0.16.1
./setup.sh

sudo cp ~/zephyr-sdk-0.16.1/sysroots/aarch64-pokysdk-linux/usr/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d
sudo udevadm control --reload

west init
west update
