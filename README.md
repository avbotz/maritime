# Maritime
Firmware written for a Raspberry Pi Pico running Zephyr RTOS. Drives the thrusters and servos, reads the depth sensor and the kill switch, and communicates with the high-level stack, [thalassic](https://github.com/avbotz/thalassic), over USB CDC-ACM.

Documentation is hosted at https://sw.avbotz.com/maritime. Its source is in [avbotz/software-docs](https://github.com/avbotz/software-docs).

## Quick start

In a terminal (do not clone the repo, the installer will do that):
```sh
curl -sSL https://raw.githubusercontent.com/avbotz/maritime/refs/heads/main/install.sh | bash
```

Then, in `maritime-workspace/maritime`:
```sh
source ./setup.sh          (init terminal)
west build -b rpi_pico     (compile)
west flash -r uf2          (flash a board in its UF2 bootloader)
./maritime.py              (talk to the board; type help)
```

| Command | Does |
| --- | --- |
| `west build -b rpi_pico -- -DEXTRA_CONF_FILE=debug.conf` | Debug build, with logs over USB |
| `./maritime.py monitor` | Print the board's state at 5 Hz |
| `./maritime.py bootloader` | Reboot the board into its UF2 bootloader for flashing |
| `./format.sh` | Format C and devicetree, report checkpatch issues |
| `./format.sh --check` | Change nothing, fail if anything needs fixing |
