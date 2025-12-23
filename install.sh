#!/bin/bash

# For installing dependencies (https://docs.zephyrproject.org/latest/develop/getting_started/index.html)

sudo apt-get update
sudo apt-get -y upgrade

sudo apt-get install --no-install-recommends -y git cmake ninja-build gperf \
    ccache dfu-util device-tree-compiler wget python3-dev python3-venv python3-tk \
    xz-utils file make gcc libsdl2-dev libmagic1

if [ "$(uname -m)" != "aarch64" ]; then
    sudo apt-get install --no-install-recommends -y gcc-multilib g++-multilib
fi

echo 'export PATH=~/.local/bin:"$PATH"' >> ~/.bashrc
source ~/.bashrc

python3 -m venv ~/zephyrproject/.venv

source ~/zephyrproject/.venv/bin/activate

pip install west

west init ~/zephyrproject
cd ~/zephyrproject
west update

west zephyr-export

west packages pip --install

cd ~/zephyrproject/zephyr
west sdk install
