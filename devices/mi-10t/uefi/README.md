# UEFI for apollo

A port of [Project Silicium / Mu-Silicium](https://github.com/Project-Silicium/Mu-Silicium) to the
Xiaomi Mi 10T (SM8250), modelled on `alioth` (the same `KonaPkg` SoC package and XBL firmware
family). You do not need to rebuild it to run HakiOS: the built image is in `devices/mi-10t/prebuilt`,
and `DEVICE=mi-10t ./mkxnuimg.py` (which `./xl enter` calls) takes it.

What is here:

- `BASE`: the Mu-Silicium commit the port is made on (the same as the POCO's);
- `mu-silicium.patch`: changes to existing files: `BootShim/BootShim.S` (copies our "XNUP"
  container behind the UEFI image to 0xF0000000) and `BootManagerLib` (starts the XNU loader from
  the container). Unlike the POCO's, `BootManager.c` calls `XnuLoaderMain()` right after the GOP is set
  up, before the boot logo and the normal/recovery split, so `fastboot boot` always reaches the payload;
- `files/`: new files:
  - `Platforms/Xiaomi/apolloPkg`: `apollo.dsc` (`KonaPkg`, SMBIOS "Xiaomi Mi 10T"), the memory map
    (`MemoryMapLib.c`, from the phone's XBL; "XNU Payload" at 0xF0000000), the driver list;
  - `Silicon/.../BootManagerLib/XnuLoader.[ch]`: as the POCO's, plus a coloured stripe on the screen
    for each stage and a `[handoff]` trace in the log around LoadImage/StartImage, which is how the
    bring-up was followed before the loader's own log worked;
  - `Resources/Configs/apollo.toml`: FD 0x9FC00000, 0x300000 bytes (XBL's UEFI window in the
    phone's `uefiplatLA.cfg`);
  - `Resources/DTBs/apollo.dtb`: the device tree of the phone's own Android (`/sys/firmware/fdt`);
  - `Binaries/apollo`: the UEFI drivers of the phone's own firmware, extracted from XBL
    `BOOT.XF.3.2-00304-SM8250-2` (slot 4) with [UEFIReader](https://github.com/WOA-Project/UEFIReader),
    the way Project Silicium ships them for other phones. Six drivers carry alioth's workarounds for
    the same firmware build; see `files/Binaries/apollo/README.md`.

## Build

The same as the POCO's (`devices/poco-x5-pro/uefi/README.md`, including the `mu_nasm` note), in a
tree of its own, `Mu-Silicium-apollo/` (the BootManager change differs, so the two ports do not share one):

```sh
devices/mi-10t/uefi/apply.sh          # clones Mu-Silicium into Mu-Silicium-apollo/ and applies the port
python3 -m venv venv && venv/bin/pip install -r Mu-Silicium-apollo/pip-requirements.txt
cd Mu-Silicium-apollo
source ../venv/bin/activate
python3 build_uefi.py -d apollo -r RELEASE     # -> Build/apolloPkg/RELEASE_CLANGPDB/FV/SILICIUM_UEFI.fd
```

After a build, `DEVICE=mi-10t ./mkxnuimg.py` takes the UEFI from `Mu-Silicium-apollo/` (`MU` in
`devices/mi-10t/device.conf`), building the boot shim there with this phone's FD base and size.
Copy `SILICIUM_UEFI.fd` and `BootShim/BootShim.bin` into `devices/mi-10t/prebuilt/` to refresh the
prebuilt image. `devices/mi-10t/uefi/export.sh` is the reverse of `apply.sh`: it saves the changes
from `Mu-Silicium-apollo/` here.
