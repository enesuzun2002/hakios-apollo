#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The Mi 10T's touch firmware (Novatek NT36675, novatek_nt36675_j3s_fw01.bin), taken out of Android's boot image.

usage: touch-fw-from-boot.py BOOT_IMG OUT_FILE

Android's kernel on this phone has the touch chip's firmware built in, so it is not a file on the phone. This reads
the boot image (an Android boot image, header v0-2, with an uncompressed arm64 Image as on the LineageOS builds it was
written for; a gzip'd one is unpacked too), and looks in the kernel for the firmware by its shape -- nothing of the
firmware itself is in this file: 139264 bytes (0x22000) starting with an info table that ends at 0x100, the version
byte at 0x1f000 and its complement at 0x1f001 (the check bsd/dev/hakios.apollo_touch.c makes), and section entries
(the first two: offset, address, size; 12 bytes each) that lie inside the file. The kernel holds the same firmware more than once;
every candidate must be byte for byte the same, or nothing is written.
"""
import gzip
import struct
import sys
import zlib

SIZE = 0x22000
NAME = b'novatek_nt36675_j3s_fw01.bin'     # the name the driver asks for (Xiaomi's nt36xxx driver, and apollo_touch.c)


def kernel_of(boot):
    if boot[:8] != b'ANDROID!':
        sys.exit('touch-fw-from-boot.py: not an Android boot image')
    kernel_size = struct.unpack_from('<I', boot, 8)[0]
    page = struct.unpack_from('<I', boot, 36)[0]
    version = struct.unpack_from('<I', boot, 40)[0]
    if version > 2:
        page = 4096                     # (v3/v4: fixed 4 KiB pages)
        kernel_size = struct.unpack_from('<I', boot, 8)[0]
    kernel = boot[page:page + kernel_size]
    if kernel[:2] == b'\x1f\x8b':
        kernel = gzip.decompress(kernel)
    return kernel


def plausible(blob):
    if len(blob) != SIZE or struct.unpack_from('<I', blob, 0)[0] != 0x100:
        return False
    if (blob[0x1f000] + blob[0x1f001]) & 0xff != 0xff or blob[0x1f000] in (0, 0xff):
        return False
    # the two sections the driver loads first (ILM, DLM: offset, address, size), then their CRCs from 0x18
    for i in range(2):
        offset, address, size = struct.unpack_from('<III', blob, 12 * i)
        if size == 0 or offset < 0x100 or offset + size > SIZE:
            return False
    return True


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    kernel = kernel_of(open(sys.argv[1], 'rb').read())
    found = {}
    at = 0
    head = b'\x00\x01\x00\x00'
    while True:
        at = kernel.find(head, at)
        if at < 0:
            break
        if at % 4 == 0 and plausible(kernel[at:at + SIZE]):
            blob = kernel[at:at + SIZE]
            found.setdefault(zlib.crc32(blob), (at, blob))
        at += 4
    if not found:
        sys.exit('touch-fw-from-boot.py: no touch firmware in this kernel')
    if len(found) > 1:
        # The kernel has others of the same shape (novatek_nt36675_j3s_mp01.bin, the factory's test firmware). The one
        # asked for by name is found through the kernel's table of built-in firmware (CONFIG_EXTRA_FIRMWARE:
        # { const char *name; const u8 *data; size_t size; }, pointers being addresses in the running kernel): an entry
        # of that size whose data and name lie as far apart in the Image as its two pointers do.
        name_at = kernel.find(NAME + b'\0')
        chosen = None
        if name_at >= 0:
            p = kernel.find(struct.pack('<Q', SIZE))
            while p >= 0 and chosen is None:
                if p % 8 == 0 and p >= 16:
                    name_va, data_va = struct.unpack_from('<QQ', kernel, p - 16)
                    data_at = name_at + (data_va - name_va)
                    if 0 <= data_at <= len(kernel) - SIZE:
                        for at, blob in found.values():
                            if kernel[data_at:data_at + SIZE] == blob:
                                chosen = (data_at, blob)
                p = kernel.find(struct.pack('<Q', SIZE), p + 8)
        if chosen is not None:
            found = {0: chosen}
    if len(found) > 1:
        sys.exit('touch-fw-from-boot.py: %d different candidates (at %s): not sure which, nothing written' %
                 (len(found), ', '.join('%#x' % a for a, _ in found.values())))
    at, blob = next(iter(found.values()))
    open(sys.argv[2], 'wb').write(blob)
    print('touch-fw-from-boot.py: %s: %d bytes from the kernel at %#x, firmware version %#x' %
          (sys.argv[2], SIZE, at, blob[0x1f000]))


main()
