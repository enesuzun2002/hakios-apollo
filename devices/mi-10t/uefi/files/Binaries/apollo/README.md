# Xiaomi Mi 10T (apollo) UEFI binaries

Extracted from the device XBL firmware `BOOT.XF.3.2-00304-SM8250-2` (XBL slot 4) with UEFIReader. The local unmodified XBL images are kept outside the repository under `.device-local/apollo/`.

Six EFI driver workarounds are carried over from Project-Silicium's alioth binaries for the same firmware build. Each patch was transferred as a byte-level delta onto the matching driver extracted from this phone, after verifying that every patched byte had the expected original value. The INF files point at the resulting `*.patched.efi` binaries.
