#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The Mi 10T's (apollo) partition table with a partition for HakiOS: userdata made 20 GiB shorter, and the
blocks it gives up made "darwin" (an APFS container, as on the POCO X5 Pro: tools/userdata-carve.py).

usage: gpt-carve-darwin.py BACKUP_DIR OUT_DIR [--size-gib N]
  BACKUP_DIR  holds gpt-primary.bin (logical unit 0's blocks 0..5) and gpt-backup.bin (its last 8 blocks), as
              read from the phone with root (dd if=/dev/block/sda bs=4096 count=6, and skip=30935032 count=8)
  OUT_DIR     gets gpt-primary.bin and gpt-backup.bin: the same blocks with the new table, to be written back
              to the same places; and darwin.txt, the new partition's identity (for the tools that write it)

Nothing is read from or written to the phone here. The userdata partition on this phone is encrypted (metadata
and file encryption), so its filesystem cannot be made smaller: it is to be formatted after the new table is
written (fastboot -w), and Android makes it again, encrypted, on the smaller partition at its next boot.

Checked before anything is written out, and the new table checked again after it is made:
  - both headers and both tables as read: signatures, header and table CRCs, each header pointing at the other,
    the two tables byte for byte the same;
  - the layout this was written for: 4096-byte blocks, 30935040 of them, 64 entries of 128 bytes, userdata in
    entry 33 from block 2883584 to the last usable block (30935034), entry 34 empty;
  - in the new table: every entry other than 33 and 34 byte for byte as it was; userdata only shorter (same start,
    type, GUID, name, attributes); darwin in entry 34, inside the usable blocks, overlapping nothing, its start on a
    1 MiB boundary; the headers changed in their CRCs only.
"""
import os
import struct
import sys
import uuid
import zlib

BS = 4096
BLOCKS = 30935040
ENTRIES, ENTRY_SIZE = 64, 128
USERDATA, DARWIN = 33, 34
USERDATA_FIRST, LAST_USABLE = 2883584, 30935034
ALIGN = (1 << 20) // BS
APFS_CONTAINER = uuid.UUID('7c3457ef-0000-11aa-aa11-00306543ecac')


def die(why):
    sys.exit(f'gpt-carve-darwin.py: {why}: nothing written')


def header(block, where):
    fields = struct.unpack('<8sIIIIQQQQ16sQIII', block[:92])
    sig, rev, size, crc, _, current, other, first, last, disk, entries_at, n, esize, table_crc = fields
    if sig != b'EFI PART':
        die(f'the {where} header has no signature')
    raw = bytearray(block[:size])
    raw[16:20] = b'\0\0\0\0'
    if zlib.crc32(raw) != crc:
        die(f'the {where} header CRC is wrong')
    return dict(rev=rev, size=size, current=current, other=other, first=first, last=last, disk=disk,
                entries_at=entries_at, n=n, esize=esize, table_crc=table_crc)


def entry(table, i):
    e = table[i * ENTRY_SIZE:(i + 1) * ENTRY_SIZE]
    first, last, attributes = struct.unpack('<QQQ', e[32:56])
    return dict(type=uuid.UUID(bytes_le=bytes(e[:16])), guid=uuid.UUID(bytes_le=bytes(e[16:32])), first=first,
                last=last, attributes=attributes, name=bytes(e[56:128]).decode('utf-16le').rstrip('\0'), raw=bytes(e))


def seal(block, table):
    """the header in `block` with the table's CRC and its own CRC made again"""
    h = bytearray(block)
    struct.pack_into('<I', h, 88, zlib.crc32(table))
    struct.pack_into('<I', h, 16, 0)
    size = struct.unpack_from('<I', h, 12)[0]
    struct.pack_into('<I', h, 16, zlib.crc32(bytes(h[:size])))
    return bytes(h)


def main():
    args = sys.argv[1:]
    size_gib = 20
    if '--size-gib' in args:
        i = args.index('--size-gib')
        size_gib = int(args[i + 1])
        del args[i:i + 2]
    if len(args) != 2:
        sys.exit(__doc__)
    src, out = args
    primary = bytearray(open(os.path.join(src, 'gpt-primary.bin'), 'rb').read())
    backup = bytearray(open(os.path.join(src, 'gpt-backup.bin'), 'rb').read())
    if len(primary) != 6 * BS or len(backup) != 8 * BS:
        die('gpt-primary.bin is not 6 blocks or gpt-backup.bin not 8')

    # as read
    ph = header(primary[BS:2 * BS], 'primary')
    bh = header(backup[7 * BS:8 * BS], 'backup')
    if (ph['current'], ph['other'], bh['current'], bh['other']) != (1, BLOCKS - 1, BLOCKS - 1, 1):
        die('the headers do not point at each other at blocks 1 and %d' % (BLOCKS - 1))
    if (ph['n'], ph['esize'], ph['entries_at'], ph['first'], ph['last']) != (ENTRIES, ENTRY_SIZE, 2, 6, LAST_USABLE):
        die('the primary header is not the layout this was written for')
    if (bh['n'], bh['esize'], bh['entries_at'], bh['first'], bh['last']) != (ENTRIES, ENTRY_SIZE, BLOCKS - 5, 6,
                                                                          LAST_USABLE):
        die('the backup header is not the layout this was written for')
    ptable = bytes(primary[2 * BS:2 * BS + ENTRIES * ENTRY_SIZE])
    b_at = (bh['entries_at'] - (BLOCKS - 8)) * BS
    btable = bytes(backup[b_at:b_at + ENTRIES * ENTRY_SIZE])
    if zlib.crc32(ptable) != ph['table_crc'] or zlib.crc32(btable) != bh['table_crc']:
        die('a table CRC is wrong')
    if ptable != btable:
        die('the primary and backup tables differ')
    old = [entry(ptable, i) for i in range(ENTRIES)]
    u = old[USERDATA]
    if (u['name'], u['first'], u['last']) != ('userdata', USERDATA_FIRST, LAST_USABLE):
        die(f'entry {USERDATA} is not userdata at {USERDATA_FIRST}..{LAST_USABLE}')
    if old[DARWIN]['raw'] != b'\0' * ENTRY_SIZE:
        die(f'entry {DARWIN} is not empty')
    if any(e['name'] == 'darwin' for e in old):
        die('there is a darwin partition already')

    # the new table: darwin the last size_gib GiB, starting on a 1 MiB boundary; userdata up to the block before
    darwin_first = (LAST_USABLE + 1 - size_gib * (1 << 30) // BS) // ALIGN * ALIGN
    userdata_last = darwin_first - 1
    darwin_guid = uuid.uuid4()
    table = bytearray(ptable)
    struct.pack_into('<Q', table, USERDATA * ENTRY_SIZE + 40, userdata_last)
    d = bytearray(ENTRY_SIZE)
    d[:16] = APFS_CONTAINER.bytes_le
    d[16:32] = darwin_guid.bytes_le
    struct.pack_into('<QQQ', d, 32, darwin_first, LAST_USABLE, 0)
    d[56:56 + 12] = 'darwin'.encode('utf-16le')
    table[DARWIN * ENTRY_SIZE:(DARWIN + 1) * ENTRY_SIZE] = d
    table = bytes(table)

    # checked again
    new = [entry(table, i) for i in range(ENTRIES)]
    for i in range(ENTRIES):
        if i not in (USERDATA, DARWIN) and new[i]['raw'] != old[i]['raw']:
            die(f'entry {i} would change')
    nu = new[USERDATA]
    if (nu['type'], nu['guid'], nu['name'], nu['first'], nu['attributes']) != \
            (u['type'], u['guid'], u['name'], u['first'], u['attributes']) or not nu['last'] < u['last']:
        die('userdata would change other than by getting shorter')
    nd = new[DARWIN]
    if not (6 <= nd['first'] <= nd['last'] <= LAST_USABLE) or nd['first'] % ALIGN:
        die('darwin is not inside the usable blocks on a 1 MiB boundary')
    # (unit 0's table lists only unit 0's partitions; overlaps are looked for among them)
    unit0 = [(e['first'], e['last'], e['name']) for e in new if e['type'].int]
    unit0.sort()
    for (f1, l1, n1), (f2, l2, n2) in zip(unit0, unit0[1:]):
        if f2 <= l1:
            die(f'{n1} ({f1}..{l1}) and {n2} ({f2}..{l2}) would overlap')

    new_primary = bytearray(primary)
    new_primary[2 * BS:2 * BS + ENTRIES * ENTRY_SIZE] = table
    new_primary[BS:2 * BS] = seal(primary[BS:2 * BS], table)
    new_backup = bytearray(backup)
    new_backup[b_at:b_at + ENTRIES * ENTRY_SIZE] = table
    new_backup[7 * BS:8 * BS] = seal(backup[7 * BS:8 * BS], table)
    # the headers: only their CRCs may have changed
    for was, now in ((primary[BS:2 * BS], new_primary[BS:2 * BS]), (backup[7 * BS:8 * BS], new_backup[7 * BS:8 * BS])):
        if bytes(was[:16]) + bytes(was[20:88]) + bytes(was[92:]) != bytes(now[:16]) + bytes(now[20:88]) + bytes(now[92:]):
            die('a header would change other than in its CRCs')
    header(new_primary[BS:2 * BS], 'new primary')
    header(new_backup[7 * BS:8 * BS], 'new backup')
    # everything outside the headers and tables as it was (the protective MBR, the padding blocks)
    for was, now, keep in ((primary, new_primary, [(0, BS), (2 * BS + ENTRIES * ENTRY_SIZE, 6 * BS)]),
                           (backup, new_backup, [(0, b_at), (b_at + ENTRIES * ENTRY_SIZE, 7 * BS)])):
        for a, b in keep:
            if was[a:b] != now[a:b]:
                die('a byte outside the headers and tables would change')

    os.makedirs(out, exist_ok=True)
    open(os.path.join(out, 'gpt-primary.bin'), 'wb').write(new_primary)
    open(os.path.join(out, 'gpt-backup.bin'), 'wb').write(new_backup)
    blocks = LAST_USABLE - darwin_first + 1
    info = (f'darwin: logical unit 0, entry {DARWIN} (sda{DARWIN + 1} under Android), blocks {darwin_first}..'
            f'{LAST_USABLE} ({blocks} blocks of {BS}, {blocks * BS / 2 ** 30:.3f} GiB), type {APFS_CONTAINER}, '
            f'unique GUID {darwin_guid}\n'
            f'userdata: entry {USERDATA}, blocks {USERDATA_FIRST}..{userdata_last} '
            f'({(userdata_last - USERDATA_FIRST + 1) * BS / 2 ** 30:.3f} GiB, was {(LAST_USABLE - USERDATA_FIRST + 1) * BS / 2 ** 30:.3f})\n')
    open(os.path.join(out, 'darwin.txt'), 'w').write(info)
    sys.stdout.write(info)
    print(f'written: {out}/gpt-primary.bin (blocks 0..5 of unit 0), {out}/gpt-backup.bin (blocks {BLOCKS - 8}..{BLOCKS - 1})')


main()
