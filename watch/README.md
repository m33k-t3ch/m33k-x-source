# M33K X Watch — development source

Firmware project for the LilyGO T-Watch Ultra side of M33K X.

## Current baseline

- M33K X v0.6.0g beta/development branch
- Full embedded M33K X UI/artwork
- Wi-Fi, BLE, Recon, GPS, Logs, Settings, Wardrive and C5-assisted 5 GHz functionality
- NFC — in progress / not yet reliable
- LoRa / Radio — in progress / not yet reliable
- Phone/Web Dashboard integration — planned / in progress
- Fresh-install timezone is not set until the user chooses one
- Wi-Fi credentials are not baked into source; persistence occurs only when Stay Connected is explicitly enabled

## Watch ↔ C5 enrollment and authentication

Public firmware no longer requires a compiled-in development key.

On a fresh Watch/C5 pair:

1. The C5 starts with no application link key and opens a 120-second enrollment window after boot.
2. The Watch discovers the C5 over BLE.
3. The Watch creates a random 256-bit per-device key and stores it in Watch NVS.
4. The key is sent to the C5 only over a BLE characteristic that requires encryption.
5. The C5 stores the key in its own NVS.
6. Normal connections use a one-use random challenge plus HMAC-SHA256 authentication.

The BLE pairing mode is Secure Connections "Just Works" because the C5 has no display or keyboard. This protects the enrollment key from passive sniffing, but it does not provide authenticated protection against an active man-in-the-middle during first enrollment. Pair the devices physically nearby and only during the intended enrollment window.

The application link key is generated at runtime and is not present in the public firmware binary.

## Dependencies

LVGL is pinned to 9.4.0. Known-working LilyGoLib/SensorLib copies are vendored in `lib/` to avoid upstream version drift while this branch is stabilized. Third-party notices/licenses remain with those libraries.

## Public release

Before a public release, build fresh Watch and C5 binaries from reviewed source, scan the exact binaries, test a clean first-time enrollment, and complete the repository release checklist.
