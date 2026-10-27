#!/bin/bash
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DATA=${IOS26_DATA:-$HERE/../../../ios26}
f=$1; b=$(basename $f)
llvm-objcopy -I binary -O elf64-littleaarch64 --rename-section .data=.text,alloc,load,readonly,code $f ${f}.o
llvm-objdump -d --no-show-raw-insn --mattr=+all ${f}.o | awk -v re='^(ptrue|pfalse|ptest|whilelt|whilegt|whilelo|whilehi|ld1[bhwd]|st1[bhwd]|st2w|ld1r[bhwd]|prf[bhwd]|smstart|smstop|zero|fmopa|fmops|bfmopa|smopa|umopa|addvl|rdvl|addsvl|rdsvl|movprfx|sel|revw|revd|splice|insr|pext|cntp|cnt[bhwd]|inc[bhwdp]|dec[bhwd]|uqincp|brkb|movaz|luti[24]|fdivr|zip|bfmax|bfmin|bfmul|fcadd|fcmla|bfcvt|bfcvtn|bfcvtn2|bfdot|bfmlal|usdot|usmmla|smmla|ummla|sudot|irg|stg|st2g|stzg|stz2g|ldg|gmi|stgp|addg|subg|ldapr[bh]?|ldapur.*|stlur.*|fjcvtzs|pacga|bti|sb|fmlal|fmlal2|crc32.*|sqrshru|dc|bfmla|bfcvtnt)$' '$2 ~ re {print $1, $2}' > "$DATA/h/$b.sel"
rm -f ${f}.o
