#include <Arduino.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <esp_system.h>
#include "mbedtls/md.h"

static constexpr char DEVICE_NAME[] = "M33K X C5";
static constexpr char SERVICE_UUID[] = "7d8a1000-6d33-4b33-a33c-6d33336b4335";
static constexpr char COMMAND_UUID[] = "7d8a1001-6d33-4b33-a33c-6d33336b4335";
static constexpr char DATA_UUID[] = "7d8a1002-6d33-4b33-a33c-6d33336b4335";
static constexpr char STATUS_UUID[] = "7d8a1003-6d33-4b33-a33c-6d33336b4335";

// Per-device Watch <-> C5 link key.
// A fresh C5 has no key. During first-time enrollment the Watch creates a
// random 256-bit key, sends it over an encrypted BLE link, and the C5 stores it
// in NVS. Public firmware therefore contains no universal shared secret.
static Preferences gPrefs;
static uint8_t gLinkKey[32] = {};
static bool gLinkKeyLoaded = false;
static constexpr uint32_t ENROLLMENT_WINDOW_MS = 120000;
static uint32_t gEnrollmentDeadlineMs = 0;

static NimBLECharacteristic *gDataChar = nullptr;
static NimBLECharacteristic *gStatusChar = nullptr;
static volatile bool gScanRequested = false;
static volatile bool gBleConnected = false;
static bool gAuthenticated = false;
static bool gNonceValid = false;
static uint8_t gAuthNonce[16] = {};
static uint32_t gNonceIssuedMs = 0;
static uint8_t gAuthFailures = 0;
static uint32_t gScanNumber = 0;
static uint32_t gLastScanMs = 0;
static int gLastScanCount = 0;
static constexpr size_t NOTIFY_CHUNK = 150;
static constexpr uint32_t AUTH_CHALLENGE_TTL_MS = 6000;

bool loadLinkKey() {
    if (gPrefs.getBytesLength("linkkey") != sizeof(gLinkKey)) {
        memset(gLinkKey, 0, sizeof(gLinkKey));
        gLinkKeyLoaded = false;
        return false;
    }
    const size_t read = gPrefs.getBytes("linkkey", gLinkKey, sizeof(gLinkKey));
    gLinkKeyLoaded = read == sizeof(gLinkKey);
    return gLinkKeyLoaded;
}

bool saveLinkKey(const uint8_t key[32]) {
    if (gPrefs.putBytes("linkkey", key, 32) != 32) return false;
    memcpy(gLinkKey, key, 32);
    gLinkKeyLoaded = true;
    return true;
}

bool enrollmentWindowOpen() {
    return !gLinkKeyLoaded && millis() <= gEnrollmentDeadlineMs;
}

String authToString(wifi_auth_mode_t mode) {
    switch (mode) {
        case WIFI_AUTH_OPEN: return "OPEN";
        case WIFI_AUTH_WEP: return "WEP";
        case WIFI_AUTH_WPA_PSK: return "WPA";
        case WIFI_AUTH_WPA2_PSK: return "WPA2";
        case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
        case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-E";
        case WIFI_AUTH_WPA3_PSK: return "WPA3";
        case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
        case WIFI_AUTH_WAPI_PSK: return "WAPI";
        default: return "UNKNOWN";
    }
}

String bytesToHex(const uint8_t *data, size_t len) {
    static constexpr char HEX_CHARS[] = "0123456789abcdef";
    String out;
    out.reserve(len * 2);
    for (size_t i = 0; i < len; ++i) {
        out += HEX_CHARS[(data[i] >> 4) & 0x0F];
        out += HEX_CHARS[data[i] & 0x0F];
    }
    return out;
}

int hexNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}

bool hexToBytes(const String &hex, uint8_t *out, size_t outLen) {
    if (hex.length() != outLen * 2) return false;
    for (size_t i = 0; i < outLen; ++i) {
        const int hi = hexNibble(hex[i * 2]);
        const int lo = hexNibble(hex[i * 2 + 1]);
        if (hi < 0 || lo < 0) return false;
        out[i] = static_cast<uint8_t>((hi << 4) | lo);
    }
    return true;
}

bool computeLinkHmac(const uint8_t *data, size_t len, uint8_t out[32]) {
    const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (!info) return false;
    return mbedtls_md_hmac(
        info,
        gLinkKey,
        sizeof(gLinkKey),
        data,
        len,
        out
    ) == 0;
}

bool constantTimeEqual(const uint8_t *a, const uint8_t *b, size_t len) {
    uint8_t diff = 0;
    for (size_t i = 0; i < len; ++i) diff |= static_cast<uint8_t>(a[i] ^ b[i]);
    return diff == 0;
}

void notifyText(const String &text) {
    // Do not print authentication payloads or link material to Serial.
    if (!text.startsWith("CHALLENGE|") && !text.startsWith("AUTH_")) {
        Serial.println(text);
    }
    if (!gBleConnected || !gDataChar) return;
    size_t offset = 0;
    while (offset < text.length()) {
        const size_t remaining = text.length() - offset;
        const size_t take = remaining > NOTIFY_CHUNK ? NOTIFY_CHUNK : remaining;
        std::string chunk(text.c_str() + offset, take);
        gDataChar->setValue(chunk);
        gDataChar->notify();
        offset += take;
        delay(8);
    }
}

void updateStatus(const String &status) {
    Serial.print("[STATUS] ");
    Serial.println(status);
    if (!gStatusChar) return;
    gStatusChar->setValue(status.c_str());
    if (gBleConnected) gStatusChar->notify();
}

void issueChallenge() {
    esp_fill_random(gAuthNonce, sizeof(gAuthNonce));
    gNonceValid = true;
    gAuthenticated = false;
    gNonceIssuedMs = millis();
    updateStatus("AUTH_REQUIRED");
    notifyText("CHALLENGE|" + bytesToHex(gAuthNonce, sizeof(gAuthNonce)) + "\n");
}

bool verifyAuthResponse(const String &hexDigest) {
    if (!gLinkKeyLoaded || !gNonceValid) return false;
    if (millis() - gNonceIssuedMs > AUTH_CHALLENGE_TTL_MS) {
        gNonceValid = false;
        return false;
    }

    uint8_t received[32] = {};
    uint8_t expected[32] = {};
    if (!hexToBytes(hexDigest, received, sizeof(received))) {
        gNonceValid = false;
        return false;
    }
    if (!computeLinkHmac(gAuthNonce, sizeof(gAuthNonce), expected)) {
        gNonceValid = false;
        return false;
    }

    const bool ok = constantTimeEqual(received, expected, sizeof(expected));
    gNonceValid = false; // challenges are strictly one-use
    return ok;
}

void sendStatus() {
    String status = "STATUS|name=" + String(DEVICE_NAME) +
        "|ble=" + String(gBleConnected ? 1 : 0) +
        "|auth=" + String(gAuthenticated ? 1 : 0) +
        "|scan=" + String(gScanNumber) +
        "|aps=" + String(gLastScanCount) +
        "|last_ms=" + String(gLastScanMs) + "\n";
    notifyText(status);
}

void perform5GHzScan() {
    if (!gAuthenticated) {
        updateStatus("AUTH_REQUIRED");
        return;
    }

    ++gScanNumber;
    updateStatus("SCANNING_5G");
#if CONFIG_SOC_WIFI_SUPPORT_5G
    WiFi.setBandMode(WIFI_BAND_MODE_5G_ONLY);
#endif
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, false);
    delay(80);
    WiFi.scanDelete();
    const uint32_t started = millis();
    const int count = WiFi.scanNetworks(false, true, true, 180, 0, nullptr, nullptr);
    const uint32_t elapsed = millis() - started;
    gLastScanMs = millis();
    if (count < 0) {
        gLastScanCount = 0;
        notifyText("SCAN_BEGIN|band=5G|id=" + String(gScanNumber) + "\n");
        notifyText("ERROR|scan_failed|code=" + String(count) + "\n");
        notifyText("SCAN_END|id=" + String(gScanNumber) + "|aps=0|ms=" + String(elapsed) + "\n");
        updateStatus("SCAN_FAILED");
        WiFi.scanDelete();
        return;
    }
    gLastScanCount = count;
    notifyText("SCAN_BEGIN|band=5G|id=" + String(gScanNumber) + "\n");
    int reported = 0;
    for (int i = 0; i < count; ++i) {
        String ssid = WiFi.SSID(i);
        String bssid = WiFi.BSSIDstr(i);
        ssid.replace("|", "_"); ssid.replace("\n", "_"); ssid.replace("\r", "_");
        if (ssid.length() == 0) ssid = "<hidden>";
        const int32_t rssi = WiFi.RSSI(i);
        const int32_t channel = WiFi.channel(i);
        if (channel <= 14) continue;
        const wifi_auth_mode_t auth = WiFi.encryptionType(i);
        String row = "AP|ssid=" + ssid + "|bssid=" + bssid + "|ch=" + String(channel) +
            "|rssi=" + String(rssi) + "|sec=" + authToString(auth) + "\n";
        notifyText(row);
        ++reported;
    }
    gLastScanCount = reported;
    notifyText("SCAN_END|id=" + String(gScanNumber) + "|aps=" + String(reported) + "|ms=" + String(elapsed) + "\n");
    WiFi.scanDelete();
    updateStatus("READY");
}

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
        (void)server;
        gBleConnected = true;
        gAuthenticated = false;
        gNonceValid = false;
        gAuthFailures = 0;
        Serial.print("[BLE] connected: ");
        Serial.println(connInfo.getAddress().toString().c_str());
        if (gLinkKeyLoaded) {
            updateStatus("AUTH_REQUIRED");
        } else if (enrollmentWindowOpen()) {
            updateStatus("ENROLL_REQUIRED");
            notifyText("ENROLL_REQUIRED|window=120\n");
        } else {
            updateStatus("ENROLL_REBOOT_REQUIRED");
            notifyText("ERROR|enroll_window_closed_reboot\n");
        }
    }
    void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override {
        (void)server; (void)connInfo;
        gBleConnected = false;
        gAuthenticated = false;
        gNonceValid = false;
        gAuthFailures = 0;
        Serial.print("[BLE] disconnected, reason="); Serial.println(reason);
        updateStatus("READY");
        NimBLEDevice::startAdvertising();
    }
};

class CommandCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &connInfo) override {
        (void)connInfo;
        std::string raw = characteristic->getValue();
        String command(raw.c_str());
        command.trim();

        if (command.startsWith("ENROLL|") || command.startsWith("enroll|")) {
            if (gLinkKeyLoaded) {
                notifyText("ERROR|already_enrolled\n");
                return;
            }
            if (!enrollmentWindowOpen()) {
                notifyText("ERROR|enroll_window_closed_reboot\n");
                updateStatus("ENROLL_REBOOT_REQUIRED");
                return;
            }

            const String keyHex = command.substring(7);
            uint8_t candidate[32] = {};
            if (!hexToBytes(keyHex, candidate, sizeof(candidate))) {
                notifyText("ERROR|bad_enroll_key\n");
                return;
            }
            if (!saveLinkKey(candidate)) {
                memset(candidate, 0, sizeof(candidate));
                notifyText("ERROR|enroll_store_failed\n");
                updateStatus("ENROLL_FAILED");
                return;
            }
            memset(candidate, 0, sizeof(candidate));
            Serial.println("[CMD] ENROLL|<redacted>");
            notifyText("ENROLL_OK|M33K X C5\n");
            issueChallenge();
            return;
        }

        if (command.equalsIgnoreCase("CHALLENGE")) {
            if (!gLinkKeyLoaded) {
                if (enrollmentWindowOpen()) {
                    updateStatus("ENROLL_REQUIRED");
                    notifyText("ENROLL_REQUIRED|window=120\n");
                } else {
                    updateStatus("ENROLL_REBOOT_REQUIRED");
                    notifyText("ERROR|enroll_window_closed_reboot\n");
                }
                return;
            }
            if (gAuthFailures >= 3) {
                notifyText("ERROR|auth_locked_reconnect\n");
                return;
            }
            issueChallenge();
            return;
        }

        if (command.startsWith("AUTH|") || command.startsWith("auth|")) {
            if (gAuthFailures >= 3) {
                notifyText("ERROR|auth_locked_reconnect\n");
                return;
            }
            const String digest = command.substring(5);
            Serial.println("[CMD] AUTH|<redacted>");
            if (verifyAuthResponse(digest)) {
                gAuthenticated = true;
                gAuthFailures = 0;
                updateStatus("AUTHENTICATED");
                notifyText("AUTH_OK|M33K X C5\n");
            } else {
                ++gAuthFailures;
                gAuthenticated = false;
                updateStatus("AUTH_FAILED");
                notifyText("ERROR|auth_failed\n");
            }
            return;
        }

        String upper = command;
        upper.toUpperCase();
        Serial.print("[CMD] "); Serial.println(upper);

        if (!gAuthenticated) {
            notifyText("ERROR|auth_required\n");
            updateStatus("AUTH_REQUIRED");
            return;
        }

        if (upper == "PING") { notifyText("PONG|M33K X C5\n"); return; }
        if (upper == "STATUS") { sendStatus(); return; }
        if (upper == "HELP") { notifyText("HELP|PING|STATUS|SCAN5\n"); return; }
        if (upper == "SCAN5") {
            if (!gScanRequested) { gScanRequested = true; updateStatus("SCAN_QUEUED"); }
            else notifyText("BUSY|scan_already_queued\n");
            return;
        }
        notifyText("ERROR|unknown_command=" + upper + "\n");
    }
};

void setupBle() {
    NimBLEDevice::init(DEVICE_NAME);
    NimBLEDevice::setMTU(247);
    // Use encrypted Secure Connections without BLE bonding. Persistent trust is
    // provided by the per-device M33K X link key stored in Preferences and the
    // HMAC challenge/response after every reconnect. Clearing stale BLE bonds
    // prevents a reboot from failing because one side retained different BLE
    // transport keys. Pairing remains Just Works because the C5 has no I/O.
    NimBLEDevice::deleteAllBonds();
    NimBLEDevice::setSecurityAuth(false, false, true);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
    NimBLEServer *server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());
    NimBLEService *service = server->createService(SERVICE_UUID);
    NimBLECharacteristic *commandChar = service->createCharacteristic(
        COMMAND_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::WRITE_ENC
    );
    gDataChar = service->createCharacteristic(
        DATA_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ_ENC
    );
    gStatusChar = service->createCharacteristic(
        STATUS_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ_ENC
    );
    commandChar->setCallbacks(new CommandCallbacks());
    gDataChar->setValue(gLinkKeyLoaded ? "M33K X C5|auth_required\n" : "M33K X C5|enroll_required\n");
    gStatusChar->setValue(gLinkKeyLoaded ? "READY" : "ENROLL_REQUIRED");
    service->start();
    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setName(DEVICE_NAME);
    advertising->enableScanResponse(true);
    NimBLEDevice::startAdvertising();
    Serial.println("[BLE] advertising as M33K X C5");
}

void setup() {
    Serial.begin(115200);
    delay(900);
    Serial.println("\n================================");
    Serial.println(" M33K X C5 v0.1.3-beta");
    Serial.println(" First-time enrollment + authenticated BLE + passive 5 GHz scan");
    Serial.println("================================");

    gPrefs.begin("m33kc5", false);
    loadLinkKey();
    if (!gLinkKeyLoaded) {
        gEnrollmentDeadlineMs = millis() + ENROLLMENT_WINDOW_MS;
        Serial.println("[PAIR] no saved Watch key; enrollment open for 120 seconds");
    } else {
        Serial.println("[PAIR] saved Watch key loaded");
    }

    WiFi.STA.begin();
#if CONFIG_SOC_WIFI_SUPPORT_5G
    WiFi.setBandMode(WIFI_BAND_MODE_5G_ONLY);
    Serial.println("[WIFI] 5 GHz hardware support: YES");
#else
    Serial.println("[WIFI] 5 GHz hardware support: NO");
#endif
    setupBle();
    updateStatus(gLinkKeyLoaded ? "READY" : "ENROLL_REQUIRED");
    Serial.println(gLinkKeyLoaded
        ? "[READY] enrolled Watch <-> C5 link enabled"
        : "[READY] waiting for first-time Watch enrollment");
}

void loop() {
    if (gScanRequested) {
        gScanRequested = false;
        perform5GHzScan();
    }
    delay(10);
}
