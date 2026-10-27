# tools/ios26: iOS 26.6.2 (23G90, iPhone12,3) image scripts

The data lives in `ios26/` at the repo root (git-ignored). Scripts there resolve it from their own
location; set `IOS26_DATA=DIR` to point them elsewhere.

- `fetch.sh`: downloads the three DMG members of the 26.6.2 IPSW into `ios26/` (AEA-decrypted via blacktop/ipsw).
- `fetch-from-ipsw.py`: fetches one member of a remote IPSW over HTTP range requests (`MEMBER OUTPUT`).
- `select-system.py`: picks the rootfs files to keep from `7z l -slt` output; writes `system.files` and `system.manifest`.
- `build-system.sh`: extracts the rootfs and cryptexes into `ios26/` and builds `ios26/system.apfs` (`extract|image|all`).
- `census/`: instruction-census helpers (`census.c`, `dscx.py`, `hist.sh`, `sel.sh`); their data stays in `ios26/census/` and `ios26/h/`.
