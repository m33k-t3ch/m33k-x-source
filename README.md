# M33K X Public Source

This repository contains the public source snapshot corresponding to the M33K X public beta firmware distributed through the M33K X Web Flasher.

## Hardware

- LilyGO T-Watch Ultra — ESP32-S3
- Seeed XIAO ESP32-C5 companion

## Source layout

- `watch/` — M33K X Watch firmware
- `c5/` — ESP32-C5 companion firmware

Both projects are built with PlatformIO.

## Features

- **Watch faces** — multiple animated faces, including a live **weather face** (pulls current conditions over Wi-Fi and animates to match: sun, clouds, rain, lightning, snow, fog, with day/night) and a neon **radar face**.
- **Wi-Fi / BLE recon** — scanning and logging, plus a **Signal Hunter** proximity mode.
- **Wardrive logging** — Wi-Fi / BLE session logging (C5-assisted 5 GHz discovery).
- **LoRa Mesh Monitor** — receive-only MeshCore monitor that decodes the public channel and shows live packets (signal, spreading factor, hex). It never transmits.
- **M33K Arcade** — built-in mini-games: an endless **Runner** and a top-down **Wardrive**, with arcade sound effects.
- **GPS**.
- **NFC** — experimental and in active development; disabled in the public build.

## Public beta

Current public firmware:

- M33K X Watch — v0.6.1 public beta
- M33K X C5 — v0.1.3 public beta

The C5 companion extends the Watch with 5 GHz Wi-Fi scanning and uses per-device Watch-to-C5 enrollment and authenticated reconnect.

NFC remains experimental and is disabled in the public build.

## Web Flasher

https://m33k-t3ch.github.io/m33k-x-flasher/

## License

M33K X source code is licensed under the GNU General Public License v3.0 (GPLv3).

M33K T3CH artwork, character designs, logos, graphics, icons, visual assets, and branding are All Rights Reserved and are not included in the GPLv3 license for the source code.

See `LICENSE.md` and `THIRD_PARTY_NOTICES.md` for details.

## Disclaimer

M33K X is an experimental learning and wireless-research project. Use wireless scanning and logging features only where you are authorized to do so.
