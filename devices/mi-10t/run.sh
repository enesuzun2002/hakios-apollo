#!/bin/bash
# A session on the Mi 10T (apollo): the phone into the loader with this phone's UEFI (DEVICE=mi-10t: mkxnuimg.py takes
# devices/mi-10t/prebuilt and device.conf), the system image uploaded, the kernel run with the phone's drivers on.
# Nothing is flashed: everything lives in RAM, and './xl reboot' (or a power-key reset) brings back Android.
#
# usage: devices/mi-10t/run.sh [SECS]          (default 900)
#   ARGS="..."   the drivers' boot-args instead of the default below (each driver is off unless its own is given);
#                wifi_mac= is added from fw-wifi-apollo/mac when that is there
#   XL_CMDLINE   the whole command line instead (then ARGS and the base are not used)
set -e
cd "$(dirname "$(readlink -f "$0")")/../.."
SECS=${1:-900}
# the base: the kernel on the fast core (bootcpu=7; the loader keeps core 0 for the battery, the log and the watchdog)
BASE="cpus=8 bootcpu=7"
# every driver of the phone, and the radio's address (boot-args.sh)
. devices/mi-10t/boot-args.sh
[ -f userland/system.apfs ] || { echo "run.sh: no userland/system.apfs: build the images first (devices/mi-10t/README.md)" >&2; exit 1; }
export DEVICE=mi-10t
./xl enter
./xl ramdisk userland/system.apfs
XL_SECS=$SECS XL_CMDLINE="${XL_CMDLINE-$BASE $ARGS}" ./xl run
echo "run.sh: './xl screenshot shot.png' for the screen, './xl reboot' to go back to Android"
