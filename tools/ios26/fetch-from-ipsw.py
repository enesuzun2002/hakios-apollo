#!/usr/bin/env python3
"""Fetch one member of an iOS firmware archive (set IPSW_URL and IPSW_SIZE; or MEMBER=--list to list) without downloading the rest.

usage: fetch-from-ipsw.py MEMBER OUTPUT

The IPSW is a ZIP64 archive on Apple's CDN; its directory is read with HTTP range requests and
the member is streamed (and inflated, if it is compressed) straight to OUTPUT. A download that
breaks off is taken up again: a member stored without compression (the big disk images are)
continues where it stopped, also when OUTPUT is the partial file of an earlier run (OUTPUT.member says
which member it is a part of: one of another member, or of another archive, is started over); a compressed
one starts over. Every member is checked against the CRC-32 of the archive's directory once it is whole,
and one that does not match is fetched again from the start.
"""
import http.client
import os
import struct
import sys
import time
import urllib.request
import zlib

ATTEMPTS = 20

URL = os.environ['IPSW_URL']
SIZE = int(os.environ['IPSW_SIZE'])


def fetch(start, end):
    request = urllib.request.Request(URL, headers={'Range': f'bytes={start}-{end}'})
    return urllib.request.urlopen(request, timeout=120)


def directory():
    tail = fetch(SIZE - 65536, SIZE - 1).read()
    at = tail.rfind(b'PK\x06\x06')
    cd_size, cd_offset = struct.unpack('<QQ', tail[at + 40:at + 56])
    cd = fetch(cd_offset, cd_offset + cd_size - 1).read()
    p = 0
    while cd[p:p + 4] == b'PK\x01\x02':
        method, = struct.unpack('<H', cd[p + 10:p + 12])
        crc, csize, usize, nl, el, cl = struct.unpack('<IIIHHH', cd[p + 16:p + 34])
        offset, = struct.unpack('<I', cd[p + 42:p + 46])
        name = cd[p + 46:p + 46 + nl].decode()
        extra = cd[p + 46 + nl:p + 46 + nl + el]
        q = 0
        while q + 4 <= len(extra):
            tag, length = struct.unpack('<HH', extra[q:q + 4])
            if tag == 1:
                values = list(struct.unpack('<' + 'Q' * (length // 8), extra[q + 4:q + 4 + length // 8 * 8]))
                if usize == 0xFFFFFFFF:
                    usize = values.pop(0)
                if csize == 0xFFFFFFFF:
                    csize = values.pop(0)
                if offset == 0xFFFFFFFF:
                    offset = values.pop(0)
            q += 4 + length
        yield name, method, csize, usize, offset, crc
        p += 46 + nl + el + cl


def file_crc(path):
    crc = 0
    with open(path, 'rb') as f:
        while chunk := f.read(1 << 20):
            crc = zlib.crc32(chunk, crc)
    return crc


def download(data, method, csize, output, ident):
    """One attempt at the member whose bytes start at DATA in the archive; returns the size of OUTPUT."""
    if method == 0:
        marker = output + '.member'
        have = os.path.getsize(output) if os.path.exists(output) else 0
        if have > csize:
            have = 0
        if have and os.path.exists(marker):
            with open(marker) as f:
                if f.read() != ident:
                    print(f'{output}: the partial file is of another member; starting over', file=sys.stderr)
                    have = 0
        with open(marker, 'w') as f:
            f.write(ident)
        if have < csize:
            with fetch(data + have, data + csize - 1) as stream, open(output, 'ab' if have else 'wb') as out:
                while chunk := stream.read(1 << 20):
                    out.write(chunk)
        return os.path.getsize(output)
    inflater = zlib.decompressobj(-15)
    written = 0
    with fetch(data, data + csize - 1) as stream, open(output, 'wb') as out:
        while chunk := stream.read(1 << 20):
            chunk = inflater.decompress(chunk)
            out.write(chunk)
            written += len(chunk)
        tail = inflater.flush()
        out.write(tail)
    return written + len(tail)


def main():
    member, output = sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else ''
    if member == '--list':
        for n, m, c, u, o, crc in directory():
            print(u, c, m, n)
        return
    for name, method, csize, usize, offset, crc in directory():
        if name != member:
            continue
        if method not in (0, 8):
            sys.exit(f'{member}: compression method {method} is not supported')
        header = fetch(offset, offset + 29).read()
        nl, el = struct.unpack('<HH', header[26:30])
        data = offset + 30 + nl + el
        ident = f'{URL} {member} {offset} {csize} {crc:08x}\n'
        written, good = -1, False
        for attempt in range(1, ATTEMPTS + 1):
            written = -1
            try:
                written = download(data, method, csize, output, ident)
            except (OSError, http.client.HTTPException, zlib.error) as error:
                print(f'{member}: {error}', file=sys.stderr)
            if written == usize:
                if file_crc(output) == crc:
                    good = True
                    break
                # (a partial file of an earlier run that was not this member's, or bytes that came wrong)
                print(f'{member}: CRC-32 does not match the archive\'s {crc:08x}; starting over', file=sys.stderr)
                os.remove(output)
            else:
                print(f'{member}: the download broke off, trying again ({attempt} of {ATTEMPTS})', file=sys.stderr)
            time.sleep(5)
        if good and os.path.exists(output + '.member'):
            os.remove(output + '.member')
        print(f'{member}: {written} bytes (expected {usize}){"" if good else ", CRC-32 not verified"}')
        sys.exit(0 if good else 1)
    sys.exit(f'{member}: not in the archive')


if __name__ == '__main__':
    main()
