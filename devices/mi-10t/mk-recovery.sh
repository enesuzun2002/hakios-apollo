#!/bin/bash
# The Mi 10T's recovery image with everything in it: the loader, the kernel, the small image and the boot image of a
# boot off the flash (userland/root.apfs and userland/flashboot.apfs, built with FLASHBOOT=1 CRYPTEXES=flash) and the
# boot-args (boot-args.sh, plus a root on darwin). Flashed to `recovery`, power + volume up starts its menu and iOS
# starts from the phone, no PC: the system from darwin (tools/darwin-write.py put it there). A PC still gets the log
# (xl), and `xl enter` (fastboot boot of the bare loader) works as before for runs from the PC.
#
# usage: devices/mi-10t/mk-recovery.sh [OUT]     (default xnu-apollo-recovery.img; ARGS as for run.sh)
#   then, with the phone in fastboot (the bootloader's, not the loader's):  fastboot flash recovery OUT
#
# The images must fit in the payload area (63 MB, xnuloader/loader.c PAYLOAD_MAX_SIZE less the panic log). The
# small image holds this phone's Wi-Fi network file when it was built without SHARE=1: the recovery image then too.
set -e
cd "$(dirname "$(readlink -f "$0")")/../.."
OUT=${1:-xnu-apollo-recovery.img}
K=xnu-src/xnu-8796.121.2/BUILD/obj/DEVELOPMENT_ARM64_BCM2837/mach.development.bcm2837
L=xnuloader/loader.efi
for f in $K $L userland/root.apfs userland/flashboot.apfs; do
	[ -f $f ] || { echo "mk-recovery.sh: no $f (make kernel, make loader, the images with FLASHBOOT=1)" >&2; exit 1; }
done
. devices/mi-10t/boot-args.sh
xnuloader/mkpayload.py -o recovery-payload.bin loader.efi=$L kernel=$K ramdisk=userland/root.apfs \
	resident=userland/flashboot.apfs --usb --cmdline "cpus=8 bootcpu=7 ufs=2 root_apfs=ufs_darwin $ARGS" >/dev/null
size=$(stat -c %s recovery-payload.bin)
[ $size -le $((0x3F00000)) ] || { echo "mk-recovery.sh: the payload is $((size >> 20)) MB, more than 63 MB" >&2; exit 1; }
DEVICE=mi-10t ./mkxnuimg.py -p recovery-payload.bin -o "$OUT" >/dev/null
rm -f recovery-payload.bin
echo "mk-recovery.sh: $OUT ($(( $(stat -c %s "$OUT") >> 20 )) MB, payload $((size >> 20)) MB)"
