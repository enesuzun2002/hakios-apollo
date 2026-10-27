#!/usr/bin/env python3 -I
"""dscx.py CACHEDIR_OR_MACHO... OUT_PREFIX : write OUT.blob (concatenated executable sections) and OUT.tsv (name start len)"""
import sys, os, struct, mmap, glob
def sections_of(get, hdr_off, translate=None):
    magic, = struct.unpack_from('<I', get(hdr_off,4))
    ncmds, = struct.unpack_from('<I', get(hdr_off+16,4))
    p = hdr_off+32; out=[]
    for _ in range(ncmds):
        cmd, size = struct.unpack_from('<II', get(p,8))
        if cmd == 0x19:
            nsects, = struct.unpack_from('<I', get(p+64,4))
            for i in range(nsects):
                s = p+72+i*80
                d = get(s,80)
                sect = d[:16].rstrip(b'\0').decode(); seg=d[16:32].rstrip(b'\0').decode()
                addr,sz = struct.unpack_from('<QQ', d, 32); off, = struct.unpack_from('<I', d, 48); flags, = struct.unpack_from('<I', d, 64)
                if (flags & 0x80000000) or (flags & 0x400):
                    out.append((seg,sect,addr,sz,off))
        p += size
    return out
def main():
    *inputs, outp = sys.argv[1:]
    blob = open(outp+'.blob','wb'); tsv = open(outp+'.tsv','w'); pos = 0
    def emit(name, data):
        nonlocal pos
        data = data[:len(data)//4*4]
        blob.write(data); tsv.write(f'{name}\t{pos}\t{len(data)}\n'); pos += len(data)
    for inp in inputs:
        if os.path.isdir(inp):
            files = sorted(f for f in glob.glob(inp+'/*') if not f.endswith(('.symbols','.map','.atlas','.json')) and not f.endswith('.dylddata'))
            maps = []; mm = {}
            for f in files:
                if os.path.getsize(f)==0: continue
                fh = open(f,'rb'); m = mmap.mmap(fh.fileno(),0,access=mmap.ACCESS_READ); 
                if m[:4] != b'dyld': continue
                mm[f]=m
                mo, mc = struct.unpack_from('<II', m, 16)
                for i in range(mc):
                    a,s,fo,mx,ini = struct.unpack_from('<QQQII', m, mo+i*32)
                    maps.append((a,s,fo,f))
            main = [f for f in files if f in mm][0]
            m0 = mm[main]; print(main, m0[:16], 'subcaches files', len(mm), 'mappings', len(maps), file=sys.stderr)
            def tr(addr, n):
                for a,s,fo,f in maps:
                    if a <= addr < a+s:
                        return mm[f], fo+addr-a
                raise KeyError(hex(addr))
            def get_addr(addr,n):
                mp, off = tr(addr,n); return mp[off:off+n]
            mo, = struct.unpack_from('<I', m0, 16)
            if mo >= 0x1c8: io, ic = struct.unpack_from('<II', m0, 0x1c0)
            else: io, ic = struct.unpack_from('<II', m0, 0x18)
            print('images', ic, file=sys.stderr)
            for i in range(ic):
                a, mt, ino, po = struct.unpack_from('<QQQI', m0, io+i*32)
                path = m0[po:m0.find(b'\0',po)].decode()
                for seg,sect,addr,sz,_ in sections_of(get_addr, None) if False else sections_of(lambda x,n: get_addr(x,n), a):
                    mp, off = tr(addr, sz)
                    emit(f'{path}', mp[off:off+sz])
        else:
            f = open(inp,'rb'); m = mmap.mmap(f.fileno(),0,access=mmap.ACCESS_READ)
            base = 0
            if m[:4] == b'\xca\xfe\xba\xbe':
                n, = struct.unpack_from('>I', m, 4); best=None
                for i in range(n):
                    ct, cs, off, sz, al = struct.unpack_from('>iiIII', m, 8+i*20)
                    if ct == 0x0100000c and (best is None or cs == 2): best = off  # cpusubtype 2 = arm64e
                    # prefer arm64e (subtype 2 w/ caps masked)
                base = best
            for seg,sect,addr,sz,off in sections_of(lambda x,n: m[x:x+n], base):
                emit(os.path.basename(inp), m[base+off:base+off+sz])
    blob.close(); tsv.close()
main()
