#!/bin/bash

set -euo pipefail

sudo apt-get update

sudo apt-get install --no-install-recommends -y git cmake ninja-build gperf \
    ccache dfu-util device-tree-compiler wget python3-dev python3-venv \
    python3-tk xz-utils file make gcc libsdl2-dev libmagic1

if [ "$(uname -m)" != "aarch64" ]; then
    sudo apt-get install --no-install-recommends -y gcc-multilib g++-multilib
fi

mkdir -p maritime-workspace
cd maritime-workspace

python3 -m venv .venv
source .venv/bin/activate

git clone https://github.com/avbotz/maritime
cd maritime

pip install --upgrade pip
pip install west

# Setup maritime with T2 topology
# https://docs.zephyrproject.org/latest/develop/west/workspaces.html
west init -l .
west update

west packages pip --install

# Only the Arm toolchain, rather than one for every architecture
west sdk install -t arm-zephyr-eabi
