#!/bin/bash
# hist.sh chunk : mnemonic histogram
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DATA=${IOS26_DATA:-$HERE/../../../ios26}
f=$1; b=$(basename $f)
llvm-objcopy -I binary -O elf64-littleaarch64 --rename-section .data=.text,alloc,load,readonly,code $f ${f}.o
llvm-objdump -d --no-show-raw-insn --no-leading-addr --mattr=+all ${f}.o | awk 'NF>0 && $1 !~ /:$/ && $1 !~ /^</ && $1 !~ /^Disassembly/ {print $1}' | sort | uniq -c > "$DATA/h/$b.hist"
rm -f ${f}.o
