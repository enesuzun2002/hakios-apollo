# HakiOS on the Xiaomi Mi 10T (apollo)

HakiOS runs iOS 16.5's own user space (SpringBoard, the stock apps) on the Xiaomi Mi 10T ("apollo",
Snapdragon 865), on Apple's open-source XNU kernel with drivers written for this phone. The Mi 10T Pro and
Redmi K30S Ultra are the same board, but only the Mi 10T has been tried. Nothing replaces Android: the phone
keeps booting Android normally, and iOS runs in a session started from a Linux PC over USB. A reboot brings
Android back.

This is a research project, not a daily driver. It is not affiliated with Apple, and no Apple or Xiaomi
files are in this repository: you fetch iOS from Apple's public servers and copy the firmware from your
own phone (see [LEGAL.md](../../LEGAL.md)).

## What works

| Works | Not yet |
|---|---|
| Lock screen, home screen, Control Center, the stock apps | Cellular (calls, SIM, mobile data) |
| Touch screen, power and volume keys | Bluetooth |
| GPU-accelerated UI (Adreno 650, OpenGL ES through Mesa) | Metal |
| Speakers, both microphones (Voice Memos records) | Cameras |
| Vibration | Hardware video decoding |
| Wi-Fi: Settings lists networks and joins them | Booting iOS without a PC (in progress) |
| Battery level and charging | Keeping iOS's data between sessions (in progress) |
| Screen brightness from the slider | |

Some animations (home screen swipes, the app switcher) still stutter now and then; that is being worked on.

## What you need

- A **Xiaomi Mi 10T** (apollo) with an **unlocked bootloader**, running **LineageOS** (the port was
  brought up on LineageOS; other ROMs may keep the firmware elsewhere).
- In LineageOS: **Settings → System → Developer options**, turn on **USB debugging** and **Rooted
  debugging** (the firmware is read as root).
- A **Linux PC** (x86-64) with about **40 GB** free, a USB cable, and these installed: `adb` and `fastboot`
  (`android-tools`), `git`, `curl`, `python3` (with `venv`), `clang`, `lld`, `make`, `cc`, `flex`, `bison`, `m4`,
  `perl`, `autoconf`, `automake`, `meson`, `ninja`, `xz`, `lz4`, and `7z` (7-Zip 23 or newer).
- **theos** with its Linux toolchain and the **iPhoneOS16.5.sdk** (see [xnu-deps/setup.sh](../../xnu-deps/setup.sh));
  the rest of the build tools are fetched for you.

## Step by step

All commands are run from the top of the repository.

**1. Get the code**

```sh
git clone -b apollo https://github.com/enesuzun2002/hakios-apollo.git hakios
cd hakios
```

**2. Fetch iOS 16.5 from Apple** (once; about 7.4 GB downloaded, unpacked into `sysimage/`)

```sh
sysimage/fetch.sh
```

**3. Copy the phone's firmware** (once; the phone in Android, on USB, Rooted debugging on)

```sh
devices/mi-10t/pull-firmware.sh
```

It fills `fw-gpu-apollo/`, `fw-audio-apollo/`, `fw-haptic-apollo/`, `fw-touch-apollo/` and `fw-wifi-apollo/`
at the top of the repository (they stay out of git). It says if anything is missing.

**4. Wi-Fi details** (optional: without them Wi-Fi is off)

```sh
adb shell dumpsys wifi | grep wifi_sta_factory_mac_address     # the phone's own Wi-Fi address
echo 'AA:BB:CC:DD:EE:FF' > fw-wifi-apollo/mac                    # ... written here
printf '%s\n%s\n' 'MyNetwork' 'my passphrase' > fw-wifi-apollo/network   # optional: joined at start
```

Without `network` you can still join a network from Settings → Wi-Fi in iOS.

**5. Build** (the first time takes a while: the toolchain, the kernel, Mesa)

```sh
xnu/apply.sh                                   # XNU from Apple's GitHub, with HakiOS's changes on top
devices/mi-10t/tools/mk-ath11k.sh              # the Wi-Fi driver
make kernel
make loader
DEVICE=mi-10t RC_JOBS=../devices/mi-10t/userland/rc.jobs GL=1 make images
```

**6. Run**

```sh
devices/mi-10t/run.sh 900                      # a 15-minute session
```

The phone can be in Android or in fastboot. The script puts it into the HakiOS loader, uploads the
system image (about 90 s), and starts iOS; the lock screen appears about a minute later. The session
ends by itself after the given number of seconds, and the log is saved in `logs/`.

**7. Back to Android**

```sh
./xl reboot
```

or hold the power key until the phone restarts.

## If something goes wrong

- **The loader stops answering** (the phone shows the loader's text and nothing happens, or `xl` says
  "upload failed" or "loader did not come up"): the phone's USB firmware sometimes stalls during the big
  upload. Hold **power + volume down** until the phone restarts into fastboot, then run `run.sh` again.
- **The phone always starts the loader instead of Android**: something asked the bootloader for recovery.
  Flash your own recovery back (`fastboot flash recovery recovery.img`), choose "Reboot system now" in it.
- **No touch, no Wi-Fi, no sound**: run `pull-firmware.sh` again and look at what it says is missing, then
  rebuild the images (`make images` as in step 5).
- Send the newest file in `logs/` along with a report: it says what every driver did.

## Optional: the loader on the phone

The loader can live in the phone's `recovery` partition, so that **power + volume up** starts it without
`fastboot boot` from the PC (Android still boots normally). Back up your recovery first:

```sh
adb root && adb shell dd if=/dev/block/by-name/recovery of=/data/local/tmp/recovery.img
adb pull /data/local/tmp/recovery.img
adb reboot bootloader
fastboot flash recovery xnu-usb.img            # the image xl makes (see ./xl enter)
```

Start it only with the key combination, not with `fastboot reboot recovery` (that leaves the phone asking
for recovery at every boot). To undo: `fastboot flash recovery recovery.img`.

---

# For developers

The rest of this page is the port's reference: where everything is, the firmware, and every driver's
boot-arg.

Everything of HakiOS that belongs to this phone is in this directory, apart from small hooks in the
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
| User space | `userland/firmware.sh` (the firmware into the small image), `userland/rc.jobs` (the job list), `userland/env.sh` (read by `userland/build.sh` with `DEVICE=mi-10t`: `DENY_MORE`, the launchd jobs left out on this phone only, e.g. Visual Voicemail's `vmd`, which spins on a core looking for CommCenter) |
| The phone's firmware | **not in the repository**: `pull-firmware.sh` pulls it into `fw-*-apollo/` at the top |
| Running | `run.sh` |

## Firmware

`pull-firmware.sh` (the phone in Android, on adb as root) writes:

- `fw-gpu-apollo/`: `a650_gmu.bin`, `a650_sqe.fw`, `a650_zap.*`.
- `fw-audio-apollo/`: `adsp.mdt` and `adsp.bNN`; the CS35L41 `cs35l41-dsp1-spk-prot-*revb2.*`; and the
  microphones' `mic_topology_v2.bin`, `mic_topology_v1.bin`, `mic_topology_legacy.bin` and `mic_audproc.bin`, which
  `tools/acdb-mic.py` writes out of the phone's ACDB (`acdb/Tutu_Handset_cal.acdb`, `acdb/Tutu_Global_cal.acdb`).
- `fw-haptic-apollo/`: `aw8697_haptic.bin`.
- `fw-wifi-apollo/`: from the phone's `/vendor/firmware_mnt/image/qca6390`: `bd_j3s.elf`, `regdb.bin`, `m3.bin`, and
  `amss20.bin` as `amss.bin`; `board-2.bin` from linux-firmware (`ath11k/QCA6390/hw2.0`).
- `fw-touch-apollo/`: `novatek_nt36675_j3s_fw01.bin`, which Android's kernel has built in: `tools/touch-fw-from-boot.py`
  takes it out of the `boot` partition.

Two more files are made by hand and stay out of git: `fw-wifi-apollo/network` (the network's name and its
passphrase, one per line) and `fw-wifi-apollo/mac` (the radio's address).

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
| `apollo_battery=1` | `hakios.apollo_battery.c` | the battery, read from the PM8150B's fuel gauge over SPMI; `apollo_battery_source=loader` reads the loader's page instead (UEFI charger drivers, polled by the loader during the run); `apollo_charger=N` |
| `apollo_charge=1` | loader | the loader takes Qualcomm's charging actions 7, 1 and 0 |
| `apollo_cpufreq=1` | `hakios.apollo_cpufreq.c` | every cluster at top speed; `apollo_ddr=1` votes the memory bus to its top too |
| `apollo_display=1`, `apollo_display_hz=N` | `hakios.apollo_display.c` | the DPU's timing and refresh rate |
| `apollo_flip=1` | `hakios.apollo_display.c` | three scanout frames, page-flipped without waiting for the last flip; `apollo_flip_latest=1` lets a newer frame replace a pending one |
| `apollo_backlight=P` | `hakios.apollo_backlight.c` | PM8150L WLED; iOS's brightness slider drives it (`kern.apollo.backlight`) |
| `apollo_gpu=N`, `apollo_gpu_use=1` | `hakios.apollo_adreno.c`, `hakios.apollo_gpu.cpp` | Adreno 650 steps 1 to N; the `ApolloGPU` service for Mesa |
| `apollo_gpu_present=1` | `userland/src/eagl.c` | the GPU scales each frame into a scanout frame (needs `apollo_flip=1`) |
| `apollo_frames=1` | `hakios.apollo_frames.c` | frame pacing statistics every 2 s: frames presented and their intervals, the present path's waits, the flips |
| `apollo_hitch=1` | `hakios.apollo_hitch.c`, `osfmk/arm64/hakios.apollo_hitch_cpu.c` | with `apollo_frames=1`: samples what each CPU runs every `apollo_hitch_us` (2000), and for every gap of more than `apollo_hitch_ms` (50) between iOS's frames *while a finger moves* (touch reports that moved in both halves of the gap; other gaps are only counted, as idle gaps, in the CPU-use line) says which processes ran on which CPU, the busiest threads of SpringBoard, backboardd and any process over 10%, the touch reports in the gap, the CPU and GPU speeds and what the GPU was doing; `apollo_hitch_rate` (10 a minute), `apollo_hitch_windows` (5: a CPU-use line every N windows, 0 none), `apollo_hitch_cpu` (0: the sampler's CPU), `apollo_hitch_threads=0` (no per-thread lines) |
| `apollo_audio=N` | `hakios.apollo_adsp.c`, `hakios.apollo_sound.c` | the audio DSP, and from it the speakers; `/dev/apollo-audio` |
| `apollo_mic=N` | `hakios.apollo_mic.c` | the microphones (WCD938x ADC1 and ADC4 over SoundWire, needs `apollo_audio=N`): 1 a look, 2 the codec on the bus, 3 `apollo_mic_secs` of listening, 4 `/dev/hakios-mic` for iOS; `apollo_mic_path=1` (default: both microphones and Xiaomi's ACDB processing, `apollo_mic_mix=0/1/2` average/main/top) or `0` (main microphone alone, no processing); `apollo_mic_ns=1` turns on the noise suppressor in Xiaomi's chain (off in their ordinary-recording calibration), `apollo_mic_drc=0` turns its multiband DRC off (its make-up gain brings the hiss back in pauses), `apollo_mic_pan=N` the IIR's pan (0, both channels; Xiaomi's is 1) |
| `apollo_amps_dsp=N` | `hakios.apollo_cs35l41.c` | the CS35L41 amplifiers' protection firmware |
| `apollo_haptic=N` | `hakios.apollo_haptic.c` | the AW8697 vibration motor; `/dev/apollo-haptic` |
| `apollo_wifi=N` | `hakios.apollo_pcie.c`, `hakios.apollo_qrtr.c`, `hakios.apollo_lx.c` | QCA6390 over PCIe0 and MHI; `wifi_bdf_phone=1` for `bd_j3s.elf`, `wifi_station=1` to join `fw/network` |

## The phone's storage

`tools/gpt-carve-darwin.py` made userdata 20 GiB shorter and put a partition `darwin` (an APFS container, as on
the POCO) in the blocks it gave up: logical unit 0, entry 34, `sda35` under Android. That wipes Android's data
(userdata is encrypted and cannot be shrunk in place); Android makes it again, encrypted, at its next boot. It is
for iOS on the phone's own storage, which is not done yet. `ufs=1` among the boot-args makes the loader list every
partition (read only); `tools/ufs-super.py` reads `super`'s layout out of that log.

## What is still provisional

- The modem and the IPA are not brought up on this phone.
- The boot menu (`xnuloader/menu.c`), shown when the loader starts from `recovery` with no PC talking to it, has
  not run on the phone yet.
- The loader sometimes stops answering during a long upload; this is inside the phone's UEFI USB driver.
