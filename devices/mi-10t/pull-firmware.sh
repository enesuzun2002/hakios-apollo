#!/bin/bash
# Pull the Mi 10T's (apollo) own firmware out of its Android into the fw-*-apollo/ directories at the top of the
# repository (never committed: it is the vendor's), kept apart from the POCO's fw-*/. The phone is on adb, in Android.
# devices/mi-10t/userland/firmware.sh puts it into the small image. What the phone does not have is said, not fatal.
set -e
cd "$(dirname "$0")/../.."
pull() { # DIR SOURCE...  (missing files are told, not fatal)
	local dir=$1; shift
	mkdir -p "$dir"
	for f in "$@"; do adb pull "$f" "$dir/" >/dev/null 2>&1 || echo "pull-firmware.sh: $f: not on the phone"; done
	echo "pull-firmware.sh: $dir: $(ls -1 "$dir" | wc -l | tr -d ' ') files"
}
first() { # DIR NAME PLACE...: NAME from the first PLACE that has it
	local dir=$1 name=$2; shift 2
	mkdir -p "$dir"
	for p in "$@"; do adb pull "$p/$name" "$dir/" >/dev/null 2>&1 && return 0; done
	echo "pull-firmware.sh: $name: in none of $*"
}
on_phone() { adb shell ls "$@" 2>/dev/null | tr -d '\r'; }
adb get-state >/dev/null
# the GPU's: the Adreno 650's microcode and its zap shader (hakios.apollo_adreno.c)
pull fw-gpu-apollo $(on_phone /vendor/firmware/a650_gmu.bin /vendor/firmware/a650_sqe.fw /vendor/firmware/a650_zap.*)
# the audio DSP's image (hakios.apollo_adsp.c) and the CS35L41 amplifiers' protection firmware (hakios.apollo_cs35l41.c)
pull fw-audio-apollo $(on_phone /vendor/firmware_mnt/image/adsp.*) /vendor/firmware/cs35l41-dsp1-spk-prot-revb2.wmfw \
	/vendor/firmware/cs35l41-dsp1-spk-prot-revb2.bin /vendor/firmware/cs35l41-dsp1-spk-prot-RCV-revb2.bin
# the main microphone's path 1 (hakios.apollo_mic.c): Xiaomi's ACDB (world-readable, no root needed), and out of it
# the device path's topology and calibration (devices/mi-10t/tools/acdb-mic.py: mic_topology_v2/v1/legacy.bin,
# mic_audproc.bin)
pull fw-audio-apollo/acdb /vendor/etc/acdbdata/Tutu/Tutu_Handset_cal.acdb /vendor/etc/acdbdata/Tutu/Tutu_Global_cal.acdb
if [ -f fw-audio-apollo/acdb/Tutu_Handset_cal.acdb ] && [ -f fw-audio-apollo/acdb/Tutu_Global_cal.acdb ]; then
	python3 devices/mi-10t/tools/acdb-mic.py fw-audio-apollo/acdb/Tutu_Handset_cal.acdb \
		fw-audio-apollo/acdb/Tutu_Global_cal.acdb fw-audio-apollo ||
		echo "pull-firmware.sh: acdb-mic.py failed: the microphone keeps to path 0"
fi
# the vibration motor's waveforms (hakios.apollo_haptic.c; Android's ueventd loads it from /vendor/firmware)
pull fw-haptic-apollo /vendor/firmware/aw8697_haptic.bin
# the touch chip's (hakios.apollo_touch.c): Android's kernel has it built in, so it is usually not a file on the phone
first fw-touch-apollo novatek_nt36675_j3s_fw01.bin /vendor/firmware /vendor/etc/firmware
[ -f fw-touch-apollo/novatek_nt36675_j3s_fw01.bin ] ||
	echo "pull-firmware.sh: take novatek_nt36675_j3s_fw01.bin out of the phone's boot image (devices/mi-10t/README.md)"
# the Wi-Fi's: the phone's board file and regulatory database (the place to be checked on another build of Android)
first fw-wifi-apollo bd_j3s.elf /vendor/firmware/qca6390 /vendor/firmware /vendor/firmware/wlan/qca_cld
first fw-wifi-apollo regdb.bin /vendor/firmware/qca6390 /vendor/firmware /vendor/firmware/wlan/qca_cld
# ... and the QCA6390's own firmware as ath11k wants it, which Android does not have: from linux-firmware
LF=https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/plain/ath11k/QCA6390/hw2.0
for f in amss.bin m3.bin board-2.bin; do
	[ -f fw-wifi-apollo/$f ] || curl -sfL -o fw-wifi-apollo/$f "$LF/$f" || echo "pull-firmware.sh: $f: not fetched from linux-firmware"
done
echo "pull-firmware.sh: fw-wifi-apollo: $(ls -1 fw-wifi-apollo | wc -l | tr -d ' ') files"
echo "pull-firmware.sh: done; the Wi-Fi also wants fw-wifi-apollo/network (the network's name and passphrase, two lines)"
