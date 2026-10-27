#!/bin/bash
# iOS 26.6.2 (23G90, iPhone12,3) system image: the same recipe as ../../sysimage/fetch.sh + ../../userland/build.sh
# ("the system image" section), as one script of its own. Produces ios26/system.apfs (in the repo root), the resident RAM disk
# (/dev/md0): the root filesystem chosen by select-system.py, the SystemOS cryptex at /System/Cryptexes/OS
# with the dyld shared cache in it, the AppOS cryptex at /System/Cryptexes/App, and (optionally) stage0 as
# /sbin/launchd. Nothing of Apple's is rewritten: files are copied as they are.
#
# Inputs (see fetch.sh): dec/rootfs.dmg, dec/cryptex-os.dmg, cryptex-app.dmg, 23G90__iPhone12,3_arm64e/
# (the extracted dyld shared cache, all 87 subcache files), rootfs.slt (7z l -slt dec/rootfs.dmg).
# Needs: python3, 7z (7-Zip 23 or newer), ../../xnu-deps/host/bin/unzbm (made by ../../sysimage/fetch.sh), about
# 8 GB of disk while it runs (rootfs/ and cryptex*/ stay, as in sysimage/).
#
#   usage: tools/ios26/build-system.sh [extract|image|all]      (default all)
#   env:   IOS26_DATA=DIR  the data directory (default: ios26/ at the repo root)
#          STAGE0=FILE  the launchd wrapper to install as /sbin/launchd (default ../../userland/obj/stage0;
#                       NO_STAGE0=1 leaves Apple's launchd where it is: an image that cannot boot as is,
#                       for sizing)
#          OVERLAY=0    leave out the inert link into /redwood/lib (the audio HAL)
#          CHECK=1      run tools/apfs/check-image.sh on the result
#          OUT=FILE     image name (default system.apfs)
#
# userland/build.sh could call this with IOS=26 later; it is not touched by this script.
set -e
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
DATA=${IOS26_DATA:-$ROOT/ios26}
cd "$DATA"
STEP=${1:-all}
OUT=${OUT:-system.apfs}
STAGE0=${STAGE0-$ROOT/userland/obj/stage0}
CACHE=23G90__iPhone12,3_arm64e
UNZBM=$ROOT/xnu-deps/host/bin/unzbm

for tool in python3 7z; do
	command -v $tool >/dev/null || { echo "build-system.sh: $tool is not installed"; exit 1; }
done
[ -x $UNZBM ] || { echo "build-system.sh: $UNZBM missing: run ../../sysimage/fetch.sh once, or build it from xnu-deps/build/lzbitmap"; exit 1; }

unzbm_all() { # unzbm_all DIR: expand every LZBITMAP file below DIR (7z leaves them empty, with an attribute file)
	find "$1" -type f -name '*:com.apple.decmpfs' | sed 's/:com\.apple\.decmpfs$//' | tr '\n' '\0' | xargs -0 -r -n 200 $UNZBM
	local left
	left=$(find "$1" -name '*:com.apple.*' | wc -l)
	[ "$left" = 0 ] || { echo "build-system.sh: $left files in $1 are still compressed"; exit 1; }
}

if [ $STEP = extract ] || [ $STEP = all ]; then
	# --- the root filesystem: what select-system.py chooses ---------------------------------------
	if [ ! -f rootfs.slt ]; then
		echo "build-system.sh: listing dec/rootfs.dmg"
		7z l -slt dec/rootfs.dmg > rootfs.slt
	fi
	python3 -I "$HERE/select-system.py" rootfs.slt system.files system.manifest
	if [ ! -f .done-rootfs ] || [ "$HERE/select-system.py" -nt .done-rootfs ]; then
		echo "build-system.sh: extracting $(wc -l < system.files) files of the root filesystem"
		rm -rf rootfs
		7z x -y -orootfs dec/rootfs.dmg @system.files > extract-system.log 2>&1 || true
		unzbm_all rootfs
		touch .done-rootfs
	fi
	# --- the SystemOS cryptex, without the dyld shared cache (23G90__iPhone12,3_arm64e/ has that) ---
	if [ ! -f .done-cryptex ]; then
		echo "build-system.sh: extracting the SystemOS cryptex"
		rm -rf cryptex
		# 7z reports every LZBITMAP file as "Unsupported method": expected, unzbm_all checks the result
		7z x -y -ocryptex dec/cryptex-os.dmg '-xr!com.apple.dyld' '-x!System/DriverKit' > extract-cryptex.log 2>&1 || true
		unzbm_all cryptex
		[ -d cryptex/System/Library/PrivateFrameworks ] || { echo "build-system.sh: the SystemOS cryptex is incomplete"; exit 1; }
		touch .done-cryptex
	fi
	# --- the AppOS cryptex, plain APFS ---------------------------------------------------------------
	if [ ! -f .done-cryptex-app ]; then
		echo "build-system.sh: extracting the AppOS cryptex"
		rm -rf cryptex-app
		7z x -y -ocryptex-app cryptex-app.dmg > extract-cryptex-app.log 2>&1 || true
		unzbm_all cryptex-app
		[ -n "$(ls cryptex-app)" ] || { echo "build-system.sh: the AppOS cryptex is empty"; exit 1; }
		touch .done-cryptex-app
	fi
fi

if [ $STEP = image ] || [ $STEP = all ]; then
	[ "$(ls $CACHE | grep -vc symbols)" -ge 80 ] || { echo "build-system.sh: the dyld shared cache in $CACHE is incomplete"; exit 1; }
	for d in rootfs cryptex cryptex-app; do
		[ -L $d ] && { echo "build-system.sh: $d is a symlink; make it a directory of this copy's own (cp -al), the build edits it via hard links"; exit 1; }
	done
	rm -rf sysroot
	cp -al rootfs sysroot                      # hard links: no copies
	# launchd must load our choice of jobs (mkjobs.py puts them in /redwood/LaunchDaemons), not all of iOS's
	rm -rf sysroot/System/Library/LaunchDaemons sysroot/System/Library/NanoLaunchDaemons \
		sysroot/System/Library/LaunchAngels sysroot/System/Library/xpc/launchd.plist
	ln -s /redwood/LaunchDaemons sysroot/System/Library/LaunchDaemons
	mkdir -p sysroot/redwood sysroot/sbin sysroot/System/Volumes/iOS
	# the applications iOS installs on its first boot are system applications here (see select-system.py)
	if [ -d sysroot/private/var/staged_system_apps ]; then
		mv sysroot/private/var/staged_system_apps/* sysroot/Applications/
		rm -rf sysroot/private/var
	fi
	if [ -z "$NO_STAGE0" ]; then
		[ -f "$STAGE0" ] || { echo "build-system.sh: $STAGE0 not found (build userland first, or NO_STAGE0=1)"; exit 1; }
		mv sysroot/sbin/launchd sysroot/sbin/launchd.ios
		cp "$STAGE0" sysroot/sbin/launchd
	else
		echo "build-system.sh: NO_STAGE0: /sbin/launchd is Apple's"
	fi
	if [ "${OVERLAY-1}" != 0 ]; then
		mkdir -p sysroot/System/Library/Audio/Plug-Ins/HAL
		ln -s /redwood/lib/HakiosAudio.driver sysroot/System/Library/Audio/Plug-Ins/HAL/HakiosAudio.driver
	fi
	# the cryptexes at their real paths (on a device two volumes grafted there; the root's own
	# System/Cryptexes/* are links into /private/preboot, which select-system.py leaves out)
	mkdir -p sysroot/System/Cryptexes
	cp -al cryptex sysroot/System/Cryptexes/OS
	cp -al cryptex-app sysroot/System/Cryptexes/App
	D=sysroot/System/Cryptexes/OS/System/Library/Caches/com.apple.dyld
	mkdir -p $D
	for f in $CACHE/dyld_shared_cache_arm64e*; do
		case "$f" in *.symbols) continue ;; esac     # symbolication data, never mapped
		ln -f "$f" $D/
	done
	echo "build-system.sh: tree $(du -sh sysroot | cut -f1) (apparent $(du -sh --apparent-size sysroot | cut -f1))"
	python3 "$ROOT/tools/apfs/mkapfs.py" -z -o $OUT System=sysroot,manifest=system.manifest | tail -1
	[ -z "$CHECK" ] || "$ROOT/tools/apfs/check-image.sh" $OUT System=sysroot,manifest=system.manifest
	rm -rf sysroot
	echo "build-system.sh: $OUT $(stat -c %s $OUT) bytes ($(( $(stat -c %s $OUT) / 1048576 )) MB)"
fi
