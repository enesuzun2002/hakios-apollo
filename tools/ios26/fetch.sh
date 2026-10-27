#!/bin/bash
# iOS 26.6.2 (23G90) for iPhone12,3 (iPhone 11 Pro): members fetched over HTTP range requests, AEA-decrypted with
# blacktop/ipsw (v3.1.739 linux binary; key comes from Apple's public wkms fcs-key service). Phase 0 notes only.
set -e
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DATA=${IOS26_DATA:-$HERE/../../ios26}
cd "$DATA"
export IPSW_URL='https://updates.cdn-apple.com/2026SummerFCS/c377a23f-a248-44fb-b9ee-121a408c4382/iPhone12,3,iPhone12,5_26.6.2_23G90_Restore.ipsw' IPSW_SIZE=10008586310
python3 "$HERE/fetch-from-ipsw.py" 094-95981-090.dmg.aea rootfs.dmg.aea        # OS (system volume), AEA
python3 "$HERE/fetch-from-ipsw.py" 094-95834-092.dmg.aea cryptex-os.dmg.aea    # Cryptex1,SystemOS (dyld cache), AEA
python3 "$HERE/fetch-from-ipsw.py" 094-96441-094.dmg cryptex-app.dmg           # Cryptex1,AppOS, plain APFS
# ipsw fw aea --key X.aea  -> base64 key;  ipsw fw aea X.aea -b KEY -o dec   (then 7z x dec/*.dmg; 7z 26 reads APFS)
