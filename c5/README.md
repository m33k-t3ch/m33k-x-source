# M33K X C5 v0.1.3-beta

Development source for the XIAO ESP32-C5 companion used by M33K X.

## Current behavior

- BLE advertised name: `M33K X C5`
- Provides passive 5 GHz Wi-Fi scan results to the matching M33K X Watch
- Fresh C5 devices open a 120-second first-time enrollment window after boot
- Enrollment uses BLE Secure Connections encryption plus a random Watch-generated 256-bit application key
- The application key is stored in C5 NVS and is not compiled into public firmware
- `SCAN5`, `PING`, `STATUS`, and other control commands require HMAC-SHA256 challenge/response authentication
- Authentication resets on BLE disconnect
- Three failed authentication attempts lock commands until reconnect

## First-time enrollment

Flash clean Watch and C5 builds, power the C5, then allow the Watch to discover it within the C5's 120-second enrollment window.

The C5 has no display/input, so BLE uses Secure Connections "Just Works." This encrypts the BLE link against passive sniffing but does not authenticate the first pairing against an active MITM. Keep both devices physically nearby during initial enrollment.

If the enrollment window expires before pairing, reboot the still-unenrolled C5 to open a new 120-second window.

## Public-release direction

No universal Watch/C5 secret is stored in source or release binaries. The per-device key is created at runtime and persisted only on the paired devices.
