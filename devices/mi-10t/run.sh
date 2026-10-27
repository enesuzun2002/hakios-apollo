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
# every driver of the phone (devices/mi-10t/xnu, devices/mi-10t/README.md says what each does)
ARGS=${ARGS-"apollo_touch=1 apollo_buttons=1 apollo_battery=1 apollo_cpufreq=1 apollo_audio=5 apollo_gpu=21 \
apollo_gpu_use=1 gpu_recover=1 apollo_backlight=50 apollo_gpu_speed=1 apollo_display=1 apollo_display_hz=60 \
apollo_haptic=5 apollo_wifi=5 apollo_wifi_rddm=0 wifi_bdf_phone=1 wifi_station=1 apollo_flip=1 \
apollo_gpu_present=1 apollo_charger=3"}
# the radio's address, kept beside the phone's Wi-Fi firmware (one line, AA:BB:CC:DD:EE:FF)
if [ -f fw-wifi-apollo/mac ]; then ARGS="$ARGS wifi_mac=$(head -1 fw-wifi-apollo/mac)"; fi
[ -f userland/system.apfs ] || { echo "run.sh: no userland/system.apfs: build the images first (devices/mi-10t/README.md)" >&2; exit 1; }
export DEVICE=mi-10t
./xl enter
./xl ramdisk userland/system.apfs
XL_SECS=$SECS XL_CMDLINE="${XL_CMDLINE-$BASE $ARGS}" ./xl run
echo "run.sh: './xl screenshot shot.png' for the screen, './xl reboot' to go back to Android"
