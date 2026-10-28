# Third-party licenses

Every third-party component HakiOS redistributes or fetches at build time, with its origin,
license, and status. Where "origin uncertain" or "to verify" is written, the item is **not**
asserted to be cleared for redistribution; treat it as pending review (see `COMPLIANCE.md`).

This supersedes the summary in `NOTICE`, which stays as the short, canonical list.

## Redistributed in this repository

| Component | Path | Origin | License | Notes |
|---|---|---|---|---|
| XNU modifications | `xnu/hakios.patch`, `xnu/files/` | derivative of Apple XNU | APSL 2.0 (`LICENSE.APSL`) | base tag `xnu-8796.121.2`, fetched by `xnu/apply.sh` |
| libdeflate | `xnu/files/bsd/miscfs/apfs/libdeflate/` | Eric Biggers | MIT | see its `COPYING` |
| libzbitmap | `xnu/files/bsd/miscfs/apfs/libzbitmap/`, `xnu-deps/build/lzbitmap/` | Corellium LLC (port of eafer/libzbitmap) | GPL-2.0+ OR MIT | used under MIT |
| LinuxKPI / Linux headers | `wifi/lx/include/linux/`, `wifi/lx/include/net/` | Linux / FreeBSD LinuxKPI | GPL-2.0 (Linux) / BSD-2-Clause (FreeBSD) | small compatibility headers; see each file's notice |
| Linux DRM shim | `tools/mesa/hakios-drm/`, `xf86drm.h` | Linux / X.Org | MIT | build-time shim for Mesa |
| AIR / Metal-library tooling | `tools/mesa/air/` | HakiOS (Darwin on redwood contributors) | MIT | our own; LLVM bitcode layout is a published format |
| mkbootimg.py | `devices/poco-x5-pro/prebuilt/mkbootimg.py` | Android Open Source Project | Apache-2.0 | |
| Mu-Silicium / Project Mu derived files | `devices/poco-x5-pro/uefi/` | Project Silicium, Microsoft, contributors | BSD-2-Clause-Patent; some GPL-2.0+ OR MIT | derived parts keep upstream license |
| Qualcomm/Xiaomi UEFI driver binaries | `devices/poco-x5-pro/uefi/files/Binaries/` | Qualcomm / Xiaomi (from device `xbl.img`) | vendor's — **to verify** | needed only to rebuild UEFI; removable, re-extract with UEFIReader |
| FocalTech touch firmware header | `xnu/files/bsd/dev/hakios.focaltech_fw.h` | FocalTech (device firmware blob) | vendor's — **to verify** | candidate to move to pull-from-device |
| Apple host headers (build only) | `xnu-deps/hostinc/mach-o/`, `.../mach/` | Apple (open-source cctools/xnu headers) | APSL 2.0 | host-side Mach-O headers for the cross build |
| Montserrat (bitmap glyphs of the Mi 10T's boot menu) | `xnuloader/menu_font.h` (made by `xnuloader/mkmenufont.py`) | The Montserrat Project Authors (github.com/JulietaUla/Montserrat), v9.000 | SIL OFL 1.1 (`xnuloader/fonts/OFL-Montserrat.txt`) | ASCII glyphs rasterised at four sizes; no Reserved Font Name |
| Pre-built kernel & user space | `prebuilt/` | built from this repo | APSL 2.0 / MIT (as source) | reproducible from source here |

## Fetched at build time, never committed

| Component | Fetched by | Origin | License |
|---|---|---|---|
| iOS 16.5 IPSW (user space of the system image) | `sysimage/fetch.sh` | Apple | Apple's — user supplies, user's own copy |
| iPhoneOS 16.5 SDK | `xnu-deps/setup.sh` (github.com/theos/sdks) | Apple (headers) via theos | Apple's / theos |
| Apple open sources (AvailabilityVersions, libdispatch, libpthread, cctools, bootstrap_cmds) | `xnu-deps/setup.sh` | Apple | APSL 2.0 |
| MacOSX13.3 SDK (Kernel.framework DriverKit headers only) | `xnu-deps/setup.sh` | Apple via joseluisq/macosx-sdks | Apple's |
| lzfse | `xnuloader/mkidentity.sh` (when no decoder present) | Apple, github.com/lzfse/lzfse | BSD-3-Clause |
| ath11k (Wi-Fi driver) | `tools/mk-ath11k.sh` | Linux | BSD-3-Clause-Clear |
| Mesa (GL=1 images) | `tools/mk-mesa.sh` | Mesa project | MIT and others |
| Procursus / Sileo packages | `tools/mk-jb.sh` | Procursus team | their own licenses |
| Phone firmware (GPU, Wi-Fi, DSP, modem, haptics, cameras) | `devices/poco-x5-pro/pull-firmware.sh` | Qualcomm / Xiaomi | vendor's — user's own device |

## How to rebuild the inventory

- Tracked non-text/binary files: `git ls-files -z | xargs -0 file --mime-type | grep -v text/`
- License mentions in tracked files: `git grep -li "SPDX-License-Identifier\|GPL\|BSD\|MIT\|APSL\|copyright"`

If a component's origin is uncertain, it is marked **to verify** above and must be treated as not
yet cleared for redistribution until confirmed with its owner.
