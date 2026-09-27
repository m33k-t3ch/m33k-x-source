# Third-Party Notices

M33K X depends on third-party libraries and vendor components. Those components remain subject to their own licenses and copyright notices.

## Main firmware dependencies

| Component | License |
| --- | --- |
| LVGL | MIT |
| RadioLib | MIT |
| XPowersLib | MIT |
| TinyGPSPlus | GNU LGPL v2.1 or later |
| NimBLE-Arduino | Apache License 2.0 |
| LilyGoLib | MIT |
| SensorLib | MIT |

Vendored third-party source must retain its original license and copyright notices. The copies of LilyGoLib and SensorLib in this repository include their own license files.

Additional transitive dependencies may carry their own terms. Review the exact dependency set used by a release build before publication.

M33K T3CH artwork and branding are governed separately by the artwork terms in [LICENSE.md](LICENSE.md).


## NFC status

NFC support is currently in progress and intentionally disabled in the public beta build. The ST25R3916-fork and NFC-RFAL-fork packages are not included in the public Watch build dependency set.
