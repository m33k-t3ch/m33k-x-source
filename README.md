# M33K X Public Source

This repository contains the public source snapshot corresponding to the M33K X public beta firmware distributed through the M33K X Web Flasher.

## Hardware

- LilyGO T-Watch Ultra — ESP32-S3
- Seeed XIAO ESP32-C5 companion

## Source layout

- `watch/` — M33K X Watch firmware
- `c5/` — ESP32-C5 companion firmware

Both projects are built with PlatformIO.

## Public beta

Current public firmware:

- M33K X Watch — v0.6.0h public beta
- M33K X C5 — v0.1.3 public beta

The C5 companion extends the Watch with 5 GHz Wi-Fi scanning and uses per-device Watch?C5 enrollment and authenticated reconnect.

NFC and Radio / LoRa remain experimental and in active development.

## Web Flasher

https://m33k-t3ch.github.io/m33k-x-flasher/

## License

M33K X source code is licensed under the GNU General Public License v3.0 (GPLv3).

M33K T3CH artwork, character designs, logos, graphics, icons, visual assets, and branding are All Rights Reserved and are not included in the GPLv3 license for the source code.

See `LICENSE.md` and `THIRD_PARTY_NOTICES.md` for details.

## Disclaimer

M33K X is an experimental learning and wireless-research project. Use wireless scanning and logging features only where you are authorized to do so.
