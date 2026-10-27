# Device: Xiaomi Mi 10T (apollo)

The second phone. Everything of HakiOS that belongs to it is here, apart from small hooks in the
shared code that call into it. Its drivers start only when the loader has recognised the phone
(`device=apollo` on the kernel command line), and each one also needs its own boot-arg. The POCO X5
Pro's drivers stay off on this phone, and nothing here runs on the POCO.

| What | Where |
|---|---|
| SoC | Qualcomm Snapdragon 865 (SM8250, "kona"); Adreno 650; QCA6390 Wi-Fi on PCIe |
| UEFI for the phone | `uefi/`: a port of Mu-Silicium after `alioth` (`KonaPkg`), built in its own `Mu-Silicium-apollo/` |
| Prebuilt UEFI images | `prebuilt/`: `SILICIUM_UEFI.fd` (0x300000 bytes), `BootShim.bin` (FD 0x9FC00000, payload 0xF0000000), the phone's `device-tree.dtb`, `ramdisk`, `mkbootimg.py` |
| What the scripts need to know | `device.conf`: `UEFI_TARGET`, `MU`, `FD_BASE`, `FD_SIZE`, `PAYLOAD_BASE` (read by `mkxnuimg.py` with `DEVICE=mi-10t`) |
| The loader's description | `xnuloader/platform.h` (`platform_apollo`, included by `xnuloader/loader.c`) |
| The kernel's drivers | `xnu/`: `files/` holds the drivers (`bsd/dev/hakios.apollo_*.c`, `iokit/Kernel/hakios.apollo_gpu.cpp`); `mi-10t.patch` adds them to the build and hooks `bsd_init`, `MiniIOMobileFramebuffer.cpp` and `loader_return.c`. `xnu/apply.sh` puts both on top of the common work, and `xnu/export.sh` saves them back here |
| Wi-Fi driver object | `tools/mk-ath11k.sh` builds ath11k for the QCA6390 from `wifi/lx/` (PCIe bus, name prefix, QMI patches) into `xnu-deps/build/ath11k-apollo`, which `xnu-build.sh` links |
| User space | `userland/firmware.sh` (the firmware into the small image), `userland/rc.jobs` (the job list) |
| The phone's firmware | **not in the repository**: `pull-firmware.sh` pulls it into `fw-*-apollo/` at the top |
| Running | `run.sh` |

## Firmware

```sh
devices/mi-10t/pull-firmware.sh      # the phone in Android, on adb
```

It writes the following directories:

- `fw-gpu-apollo/`: `a650_gmu.bin`, `a650_sqe.fw`, `a650_zap.*`.
- `fw-audio-apollo/`: `adsp.mdt` and `adsp.bNN`, plus the CS35L41 `cs35l41-dsp1-spk-prot-*revb2.*`.
- `fw-haptic-apollo/`: `aw8697_haptic.bin`.
- `fw-wifi-apollo/`: the phone's `bd_j3s.elf` and `regdb.bin`, and `amss.bin`, `m3.bin` and `board-2.bin` from linux-firmware (`ath11k/QCA6390/hw2.0`).
- `fw-touch-apollo/`: the touch firmware, `novatek_nt36675_j3s_fw01.bin`.

Android's kernel has the touch firmware built in, so it is usually not a file on the phone. Take it out of
the phone's boot image instead. The places the script looks for `bd_j3s.elf` and `regdb.bin` are the
likely ones; check them on another build of Android.

Two more files are made by hand and stay out of git:

- `fw-wifi-apollo/network`: the network's name and its passphrase, one per line.
- `fw-wifi-apollo/mac`: the radio's address.

## Building and running

```sh
xnu/apply.sh                                 # the kernel tree, with devices/*/xnu on top
devices/mi-10t/tools/mk-ath11k.sh            # the Wi-Fi driver object (runs tools/mk-ath11k.sh first if needed)
make kernel                                  # links it when it is there
make loader
DEVICE=mi-10t RC_JOBS=../devices/mi-10t/userland/rc.jobs GL=1 make images
devices/mi-10t/run.sh 900                    # the phone in Android or fastboot, on USB
```

`run.sh` does `DEVICE=mi-10t ./xl enter`, `./xl ramdisk userland/system.apfs` and `./xl run` with the
boot-args below (`ARGS=` replaces them).

## The boot-args

Each driver is off unless its own boot-arg is given. The head of each file says more.

| Boot-arg | Driver | What |
|---|---|---|
| `apollo_touch=1` | `hakios.apollo_touch.c` | Novatek NT36675 touch screen (SPI) |
| `apollo_buttons=1` | `hakios.apollo_buttons.c` | power and volume keys (PM8150 PON, GPIO6) |
| `apollo_battery=1` | `hakios.apollo_battery.c` | the battery, from the loader's page (UEFI charger drivers); `apollo_charger=N` |
| `apollo_charge=1` | loader | the loader takes Qualcomm's charging actions 7, 1 and 0 |
| `apollo_cpufreq=1` | `hakios.apollo_cpufreq.c` | every cluster at top speed |
| `apollo_display=1`, `apollo_display_hz=N` | `hakios.apollo_display.c` | the DPU's timing and refresh rate |
| `apollo_flip=1` | `hakios.apollo_display.c` | three scanout frames, page-flipped |
| `apollo_backlight=P` | `hakios.apollo_backlight.c` | PM8150L WLED; iOS's brightness slider drives it (`kern.apollo.backlight`) |
| `apollo_gpu=N`, `apollo_gpu_use=1` | `hakios.apollo_adreno.c`, `hakios.apollo_gpu.cpp` | Adreno 650 steps 1 to N; the `ApolloGPU` service for Mesa |
| `apollo_gpu_present=1` | `userland/src/eagl.c` | the GPU scales each frame into a scanout frame (needs `apollo_flip=1`) |
| `apollo_frames=1` | `hakios.apollo_frames.c` | frame pacing statistics |
| `apollo_audio=N` | `hakios.apollo_adsp.c`, `hakios.apollo_sound.c` | the audio DSP, and from it the speakers; `/dev/apollo-audio` |
| `apollo_amps_dsp=N` | `hakios.apollo_cs35l41.c` | the CS35L41 amplifiers' protection firmware |
| `apollo_haptic=N` | `hakios.apollo_haptic.c` | the AW8697 vibration motor; `/dev/apollo-haptic` |
| `apollo_wifi=N` | `hakios.apollo_pcie.c`, `hakios.apollo_qrtr.c`, `hakios.apollo_lx.c` | QCA6390 over PCIe0 and MHI; `wifi_bdf_phone=1` for `bd_j3s.elf`, `wifi_station=1` to join `fw/network` |

## What is still provisional

- The loader recognises the phone by its 1080x2400 screen at 0x9C000000, the display region
  `MemoryMapLib.c` reserves (`xnuloader/platform.h`). That this is the GOP's base on the phone is
  still to be seen in a loader log ("display 1080 x 2400 at 9c000000"); if it is not, the loader
  says "platform qemu-virt" and the base in platform.h wants changing.
- The modem and the IPA are not brought up on this phone. `soc.args` and `pull-soc.sh` are not
  here: the `soc_*` arguments feed the POCO's modem and IPA drivers, which are not ported to
  this phone.
