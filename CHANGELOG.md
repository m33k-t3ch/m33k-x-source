# Changelog

All notable changes to the M33K X public beta are documented here.
The Watch firmware is distributed through the [M33K X Web Flasher](https://m33k-t3ch.github.io/m33k-x-flasher/).

## [0.6.1] — 2026-10-30

### Added
- **M33K Arcade** — built-in mini-games: an endless **Runner** and a top-down **Wardrive**, with per-character projectiles and arcade sound effects.
- **LoRa Mesh Monitor** — receive-only MeshCore decode of the public channel (SX1262), with a live packet log. It never transmits.
- **Live weather face** — condition-driven animated backdrop (clear / clouds / fog / rain / snow / storm), day/night, and wind-driven motion. Weather data over Wi-Fi; location from GPS or IP geolocation.
- **Radar watch face** — animated neon eye-sweeps with a sideways clock/date.
- Animated Wi-Fi scanner mascot.

### Changed
- Denser, faster obstacle pacing in the Runner.

### Removed
- Map dev-test location fallback (a hardcoded placeholder coordinate) — the offline map now uses only a real GPS fix.

### Notes
- NFC remains experimental and is disabled in the public build.

## [0.6.0i] — 2026-10

### Changed
- Wardrive Wi-Fi session totals can exceed 56 observed networks.
- Improved Wardrive scrolling and target-detail overlay recovery.
- Retains C5-assisted 5 GHz discovery and reconnect.
