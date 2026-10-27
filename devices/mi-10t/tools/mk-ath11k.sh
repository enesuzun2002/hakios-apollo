#!/bin/bash
# The Mi 10T's (apollo) Wi-Fi driver for our kernel: the same ath11k as tools/mk-ath11k.sh builds for the POCO
# (Linux's, on FreeBSD's LinuxKPI headers and wifi/lx), for the QCA6390 on PCIe, tied into an object of its own.
# It uses what that script has fetched and prepared (xnu-deps/build/ath11k: src/, freebsd/, gen/; the script is
# run when they are not there) and compiles the driver's files, wifi/lx/src's shared ones (lx_base.c, lx_80211.c,
# lx_wpa.c) and this phone's bus (devices/mi-10t/wifi/lx/src/lx_bus_pci.c, in place of lx_bus.c) again, with
# devices/mi-10t/wifi/lx/include/apollo_prefix.h in front of everything: what the object asks of the kernel is
# then apollo_lx_os_* (devices/mi-10t/xnu: hakios.apollo_lx.c) and what it shows is apollo_lx_prepare/start/stop/
# transmit, so that the POCO's object and bsd/dev/hakios.lx.c stay linked into the same kernel without a name in common.
#
# What comes out, in xnu-deps/build/ath11k-apollo:
#   out/ath11k-apollo.o    the object for the kernel (xnu-build.sh links it when it is there)
#   out/stubs.txt          what of the driver is not built (as for the POCO)
# usage: devices/mi-10t/tools/mk-ath11k.sh
set -e
ROOT=$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)
B=$ROOT/xnu-deps/build/ath11k
A=$ROOT/xnu-deps/build/ath11k-apollo
W=$ROOT/wifi/lx
WA=$ROOT/devices/mi-10t/wifi/lx            # this phone's own: the PCIe bus, the name prefix, the QMI patches
if [ ! -f "$B/src/.release" ] || [ ! -f "$B/freebsd/.release" ] || [ ! -d "$B/gen" ]; then
	"$ROOT/tools/mk-ath11k.sh" || true
fi
[ -f "$B/src/.release" ] && [ -f "$B/freebsd/.release" ] && [ -d "$B/gen" ] || { echo "mk-ath11k.sh (mi-10t): tools/mk-ath11k.sh has not prepared $B" >&2; exit 1; }
S=$B/freebsd/sys
mkdir -p "$A"
cd "$A"

# the QCA6390 needs no ahb.c, pci.c, pcic.c or mhi.c: the bus is lx_bus_pci.c, the MHI side is the kernel's
DRIVER="core hal qmi ce hw fw htc wmi mac reg peer dp dp_rx dp_tx hal_rx hal_tx dbring p2p"
SHARED="lx_base lx_80211 lx_wpa"
CC="clang -target arm64-apple-ios16.5"
LD=$ROOT/xnu-deps/host/bin/ld64
CFLAGS="-mkernel -mgeneral-regs-only -fno-builtin -fno-common -ffreestanding -fno-strict-aliasing
	-funsigned-char -nostdinc -std=gnu17 -O2 -g -w
	-D__FreeBSD__=15 -D_KERNEL -DKLD_MODULE -DHAVE_KERNEL_OPTION_HEADERS -include opt_global.h
	-include $WA/include/apollo_prefix.h
	-I$W/include -I$B/gen -I$S -I$S/contrib/ck/include
	-I$S/compat/linuxkpi/common/include -I$S/compat/linuxkpi/dummy/include
	-include $S/compat/linuxkpi/common/include/linux/kconfig.h -DLINUXKPI_VERSION=70000"
DFLAGS="-I$A/src/ath11k -I$B/src -include $W/include/lx_prelude.h -DCONFIG_ATH11K_PCI -DCONFIG_ATH11K_DEBUG
	-DKBUILD_MODNAME=\"ath11k\""
source "$ROOT/xnu-env.sh"
for h in lx qmi; do
	[ -f "$XNU_SRC/bsd/sys/hakios.$h.h" ] || { echo "mk-ath11k.sh (mi-10t): the kernel tree has no bsd/sys/hakios.$h.h" >&2; exit 1; }
done
DFLAGS="$DFLAGS -DLX_HAKIOS_LX_H=\"$XNU_SRC/bsd/sys/hakios.lx.h\" -DLX_HAKIOS_QMI_H=\"$XNU_SRC/bsd/sys/hakios.qmi.h\""
mkdir -p obj out src/ath11k
# a private copy of the driver's directory (the shared sources stay as they are) with devices/mi-10t/wifi/lx/patches/*.patch applied
# to it (the patches are against the files by their names in it): cnss2's host capability (ddr_range) in qmi.c/qmi.h, and the firmware's memory on round boundaries (qmi-align)
cp -pru "$B"/src/. src/
rm -rf patched && mkdir patched
cp "$B/src/ath11k/qmi.c" "$B/src/ath11k/qmi.h" patched/
for p in qmi-ddr-range qmi-align; do	# (in this order: each is against the file as the one before left it)
	patch -s -d patched -p0 < "$WA/patches/$p.patch" || { echo "mk-ath11k.sh (mi-10t): patches/$p.patch does not apply" >&2; exit 1; }
done
for f in qmi.c qmi.h; do cmp -s patched/$f src/ath11k/$f || cp patched/$f src/ath11k/$f; done
rm -f obj/*.err
compile() {		# compile SOURCE OBJECT [FLAGS]: only when the object is older than anything it was made of
	local src=$1 obj=$2
	shift 2
	if [ -f "$obj" ] && [ -f "$obj.d" ] && [ -z "$(find $(sed -e 's/^[^:]*://' -e 's/\\$//' "$obj.d") "$0" -newer "$obj" 2>/dev/null | head -1)" ]; then
		return
	fi
	$CC $CFLAGS "$@" -MMD -MF "$obj.d" -c "$src" -o "$obj" 2> "$obj.err" || rm -f "$obj"
}
SOURCES="$(for f in $SHARED lx_bus_pci; do echo $f; done)"
for f in $DRIVER; do compile "$A/src/ath11k/$f.c" obj/ath11k_$f.o $DFLAGS & done
# lx_base.c's DMA pool, larger for the QCA6390: the firmware's own memory (6 MB of HOST_DDR, 864 KB more, M3) comes out of
# its rings part on round boundaries (patches/qmi-align.patch), and 12 MB of rings left the data path 2 MB short.
# A private copy with POOL_SIZE 64 MB and POOL_RINGS 28 MB; hakios.apollo_pcie.c reserves the 64 MB at boot.
mkdir -p src/lx
sed -e 's/^#define POOL_SIZE\t.*/#define POOL_SIZE\t(64u << 20)/' -e 's/^#define POOL_RINGS\t.*/#define POOL_RINGS\t(28u << 20)/' \
    "$W/src/lx_base.c" > src/lx/lx_base.c.new
cmp -s src/lx/lx_base.c.new src/lx/lx_base.c && rm src/lx/lx_base.c.new || mv src/lx/lx_base.c.new src/lx/lx_base.c
grep -q '^#define POOL_SIZE.(64u << 20)' src/lx/lx_base.c || { echo "mk-ath11k.sh (mi-10t): lx_base.c's POOL_SIZE was not changed" >&2; exit 1; }
for f in $SOURCES; do
	s="$W/src/$f.c"; [ $f = lx_base ] && s=src/lx/lx_base.c; [ $f = lx_bus_pci ] && s="$WA/src/$f.c"
	compile "$s" obj/lx_$f.o $DFLAGS -I$W/src &
done
wait
bad=
for f in obj/*.err; do
	[ -s "$f" ] && [ ! -f "${f%.err}" ] && { echo "--- $(basename "$f" .o.err)"; grep -m6 -A3 "error:" "$f"; bad=1; }
done
[ -z "$bad" ] || exit 1
OBJECTS=$(for f in $DRIVER; do echo obj/ath11k_$f.o; done; for f in $SOURCES; do echo obj/lx_$f.o; done)

# --- tie it into one object, as mk-ath11k.sh does: what is still wanted is a call of the kernel's, or a function of
# a part of the driver that is not built (a stub that says its name), or a mistake
KERNEL="memcmp memcpy memmove memset strcmp strncmp strlen strchr strrchr strlcpy strlcat bzero
	snprintf vsnprintf __stack_chk_fail __stack_chk_guard"
$LD -r -arch arm64 $OBJECTS -o out/all.o 2> out/ld.err || { cat out/ld.err; exit 1; }
llvm-nm -u --format=just-symbols out/all.o | sed 's/^_//' | sort > out/undefined.txt
: > out/stubs.txt
: > out/stubs.s
missing=
for s in $(cat out/undefined.txt); do
	case " $(echo $KERNEL) " in *" $s "*) continue ;; esac
	case $s in
	apollo_lx_os_*|qmi_*|hakios_qmi_*|apollo_wifi_*|section\$*) continue ;;
	ath11k_*|__ath11k_*) ;;
	*) echo "mk-ath11k.sh (mi-10t): nothing defines $s" >&2; missing=1; continue ;;
	esac
	echo $s >> out/stubs.txt
	printf '\t.globl _%s\n_%s:\n\tadrp x0, 1f@PAGE\n\tadd x0, x0, 1f@PAGEOFF\n\tb _lx_stub\n\t.cstring\n1:\t.asciz "%s"\n\t.text\n\t.p2align 2\n' \
		$s $s $s >> out/stubs.s
done
[ -z "$missing" ] || exit 1
$CC -c out/stubs.s -o out/stubs.o
printf '_apollo_lx_prepare\n_apollo_lx_start\n_apollo_lx_stop\n_apollo_lx_transmit\n' > out/exported.txt
$LD -r -arch arm64 -exported_symbols_list out/exported.txt out/all.o out/stubs.o -o out/ath11k-apollo.o 2> out/ld.err ||
	{ cat out/ld.err; exit 1; }
rm -f out/all.o

# --- nothing but the four names may be defined with global scope (private externs included): the POCO's object is
# linked into the same kernel
llvm-nm -g --defined-only --format=just-symbols out/ath11k-apollo.o | sort > out/defined-global.txt
if [ "$(cat out/defined-global.txt | tr '\n' ' ')" != "$(sort out/exported.txt | tr '\n' ' ')" ]; then
	echo "mk-ath11k.sh (mi-10t): the object defines global names besides the four:" >&2
	grep -vxFf out/exported.txt out/defined-global.txt >&2
	exit 1
fi
echo "mk-ath11k.sh (mi-10t): out/ath11k-apollo.o: $(llvm-size out/ath11k-apollo.o | awk 'NR == 2 { print $1 + $2 }') bytes of code and data;" \
	"$(wc -l < out/stubs.txt) functions of the driver's other files are stubs (out/stubs.txt);" \
	"asks the kernel for $(llvm-nm -u --format=just-symbols out/ath11k-apollo.o | grep -vc 'section\$') names"
