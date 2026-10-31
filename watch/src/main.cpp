#include <Arduino.h>

// Recon/Pulse/Radar process Wi-Fi, BLE and LVGL results on Arduino's loopTask.
// The ESP32 Arduino default loop stack is 8 KB; measured crashes on hardware
// hit the loopTask stack canary after the first Recon Pulse/Radar result pass.
// Give the main task more headroom for those combined scan/render paths.
SET_LOOP_TASK_STACK_SIZE(16 * 1024);

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <esp_heap_caps.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <SD.h>
#include <time.h>
#include <TinyGPSPlus.h>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "mbedtls/md.h"
#include "mbedtls/aes.h"
#include "mbedtls/sha256.h"
#include <LilyGoLib.h>
#include <LV_Helper.h>
#include <lvgl.h>

#include "m33k_peek_art.h"
#include "bunny_skull_small.h"
#include "bunny_skull_graph.h"
#include "wifi_bunny_bg.h"
#include "m33k_ble_anim.h"
#include "m33k_faces.h"
#include "m33k_bubbles.h"
#include "m33k_wx.h"
#include "m33k_radarface.h"
#include "m33k_game.h"
#include "hunter_choose_bg.h"
#include "wardrive_banner.h"
#include "recon_dashboard_art.h"


namespace {

constexpr uint8_t M33K_ROTATION = 0;
constexpr uint8_t M33K_DEFAULT_BRIGHTNESS_PERCENT = 82;
constexpr uint8_t M33K_MIN_BRIGHTNESS_PERCENT = 10;
uint8_t displayBrightnessPercent = M33K_DEFAULT_BRIGHTNESS_PERCENT;

uint8_t displayBrightnessRaw()
{
    const uint16_t pct = constrain(
        static_cast<uint16_t>(displayBrightnessPercent),
        static_cast<uint16_t>(M33K_MIN_BRIGHTNESS_PERCENT),
        static_cast<uint16_t>(100)
    );
    return static_cast<uint8_t>((pct * 255u) / 100u);
}

constexpr uint32_t SCREEN_TIMEOUT_MS = 25000;

// After the screen has been idle this long, shed the GNSS rail to save
// battery during deep idle (pocket/overnight). GPS stays powered during
// normal use and brief screen-sleeps so satellite lock stays warm; it is
// only cut once the watch has clearly been set down, and is restored the
// instant the screen wakes. See sleepScreen()/wakeScreen() and loop().
constexpr uint32_t GPS_DEEP_IDLE_MS = 180000;  // 3 minutes

constexpr int16_t SCREEN_W = 410;
constexpr int16_t SCREEN_H = 502;
constexpr int16_t DOCK_Y   = 426;

enum class Page : uint8_t {
    Home,
    WiFi,
    BLE,
    Recon,
    ReconPulse,
    Radar,
    SignalHunter,
    WatchMode,
    Wardrive,
    Radio,
    GPS,
    Map,
    NFC,
    Logs,
    Settings,
    Stopwatch,
    Game
};

struct DockItem {
    const char *label;
    const char *icon;
    uint32_t iconColorHex;
    Page page;
};

struct MatrixEntity {
    bool isSkull = false;
    lv_obj_t *glow = nullptr;
    lv_obj_t *obj = nullptr;
    int16_t x = 0;
    int16_t y = 0;
    int16_t speed = 4;
    uint32_t colorHex = 0x3AEFFF;
    uint8_t lines = 14;
};

struct TimeZoneEntry {
    const char *label;
    const char *posix;
};

struct WifiResultDetail {
    String ssid;
    String bssid;
    int32_t rssi = 0;
    int32_t channel = 0;
    wifi_auth_mode_t auth = WIFI_AUTH_OPEN;
    bool fromC5 = false;
};

struct BleResultDetail {
    String name;
    String address;
    int32_t rssi = 0;
    int8_t txPower = 0;
    uint8_t addressType = 0;
    uint8_t serviceCount = 0;
    uint8_t manufacturerCount = 0;
    bool connectable = false;
    bool scannable = false;
    bool haveTxPower = false;
};

struct WardriveRecord {
    bool active = false;
    bool wifi = false;
    bool fromC5 = false;
    bool trackerLike = false;
    String name;
    String id;
    int32_t rssi = -127;
    int32_t channel = 0;
    wifi_auth_mode_t auth = WIFI_AUTH_OPEN;
    uint16_t seenCount = 0;
    uint32_t firstSeenMs = 0;
    uint32_t lastSeenMs = 0;
    bool haveLocation = false;
    double lat = 0.0;
    double lon = 0.0;
    uint8_t sats = 0;
    int8_t txPower = 0;
    bool haveTxPower = false;
    uint8_t serviceCount = 0;
    uint8_t manufacturerCount = 0;
    bool connectable = false;
    bool scannable = false;
    lv_obj_t *row = nullptr;
    lv_obj_t *label = nullptr;
};


struct RadarBlip {
    bool active = false;
    bool seenThisSweep = false;
    bool missing = false;
    bool wifi = false;
    bool fromC5 = false;
    String name;
    String id;
    int rssi = -127;
    uint16_t angleDeg = 0;
    uint32_t lastSeenMs = 0;
    lv_obj_t *dot = nullptr;
};

struct PulseAp {
    bool active = false;
    bool seenThisSweep = false;
    bool missing = false;
    bool fromC5 = false;
    String ssid;
    String bssid;
    int rssi = -127;
    int prevRssi = -127;
    int deltaRssi = 0;
    int channel = 0;
    uint16_t angleDeg = 0;
    uint32_t firstSeenMs = 0;
    uint32_t lastSeenMs = 0;
    int trailRssi[4] = {-127, -127, -127, -127};
    uint8_t trailCount = 0;
    lv_obj_t *dot = nullptr;
    lv_obj_t *trail[4] = {};
};

enum class PulsePhase : uint8_t {
    Idle,
    WifiScanning,
    Failed
};

enum class RadarPhase : uint8_t {
    Idle,
    WifiScanning,
    BleScanning,
    Done,
    Failed
};

enum class HunterKind : uint8_t {
    None,
    WiFi,
    BLE
};

enum class HunterPhase : uint8_t {
    Idle,
    WifiSelecting,
    BleSelecting,
    Tracking
};

enum class HunterFeedbackMode : uint8_t {
    Off,
    Beep,
    Haptic,
    Both
};

constexpr TimeZoneEntry TIMEZONES[] = {
    {"Pacific (DST)",  "PST8PDT,M3.2.0,M11.1.0"},
    {"Mountain (DST)", "MST7MDT,M3.2.0,M11.1.0"},
    {"Central (DST)",  "CST6CDT,M3.2.0,M11.1.0"},
    {"Eastern (DST)",  "EST5EDT,M3.2.0,M11.1.0"},
    {"Arizona",        "MST7"},
    {"Alaska (DST)",   "AKST9AKDT,M3.2.0,M11.1.0"},
    {"Hawaii",         "HST10"},
    {"UTC",            "UTC0"},
};

constexpr uint8_t TIMEZONE_COUNT = sizeof(TIMEZONES) / sizeof(TIMEZONES[0]);
constexpr uint8_t TIMEZONE_UNSET = 0xFF;

Preferences prefs;
// Fresh/public installs intentionally start with no timezone selected.
// Existing watches keep any valid timezone already saved in NVS.
uint8_t timezoneIndex = TIMEZONE_UNSET;
bool use24Hour = false;
bool wifiStayConnected = false;
String savedWifiSsid;
String savedWifiPassword;

bool wifiConnectPending = false;
uint32_t wifiConnectStartedMs = 0;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;

constexpr DockItem dock[] = {
    {"Wi-Fi",    LV_SYMBOL_WIFI,      0x2F6BFF, Page::WiFi},
    {"BLE",      LV_SYMBOL_BLUETOOTH, 0x6D86FF, Page::BLE},
    {"Recon",    "R",                 0xFF52C8, Page::Recon},
    {"GPS",      LV_SYMBOL_GPS,       0xF5FF3B, Page::GPS},
    {"NFC",      "N",                 0xB37BFF, Page::NFC},
    {"Logs",     LV_SYMBOL_FILE,      0x6AFF78, Page::Logs},
    {"Game",     "G",                 0xFF2A2A, Page::Game},
    {"Settings", LV_SYMBOL_SETTINGS,  0xF4F6FF, Page::Settings},
};

constexpr uint8_t MATRIX_ENTITY_COUNT = 14;
MatrixEntity gMatrix[MATRIX_ENTITY_COUNT];

Page currentPage = Page::Home;
lv_obj_t *screen = nullptr;

lv_timer_t *matrixTimer = nullptr;
lv_timer_t *homeLiveTimer = nullptr;
lv_timer_t *wifiTimer = nullptr;
// Animated m33k running across the Wi-Fi scan page (replaces the static bunny).
lv_obj_t *wifiMascot = nullptr;
lv_timer_t *wifiMascotTimer = nullptr;
int wifiMascotX = 20, wifiMascotFrame = 0, wifiMascotDir = 1;
uint32_t wifiMascotAnimMs = 0;
void wifiMascotCb(lv_timer_t *timer);
lv_timer_t *gpsTimer = nullptr;
lv_timer_t *mapTimer = nullptr;
lv_timer_t *reconTimer = nullptr;
lv_timer_t *watchTimer = nullptr;
lv_timer_t *radioTimer = nullptr;
lv_timer_t *nfcTimer = nullptr;

// Stopwatch
lv_timer_t *stopwatchTimer = nullptr;
lv_obj_t *stopwatchTimeLabel = nullptr;
lv_obj_t *stopwatchStartLabel = nullptr;
lv_obj_t *stopwatchStatusLabel = nullptr;

bool stopwatchRunning = false;
uint32_t stopwatchStartedMs = 0;
uint32_t stopwatchAccumulatedMs = 0;

enum class ReconPhase : uint8_t {
    Idle,
    WifiScanning,
    BleScanning,
    Done,
    Failed
};

enum class WatchPhase : uint8_t {
    Idle,
    WifiScanning,
    BleScanning,
    Failed
};

enum class WardrivePhase : uint8_t {
    Stopped,
    Idle,
    WifiScanning,
    BleScanning,
    Failed
};

ReconPhase reconPhase = ReconPhase::Idle;
lv_obj_t *reconStatusLabel = nullptr;
lv_obj_t *reconWifiCountLabel = nullptr;
lv_obj_t *reconWifiStrongLabel = nullptr;
lv_obj_t *reconChannelLabel = nullptr;
lv_obj_t *reconBleCountLabel = nullptr;
lv_obj_t *reconBleStrongLabel = nullptr;
lv_obj_t *reconGpsLabel = nullptr;
lv_obj_t *reconSatLabel = nullptr;

int reconWifiCount = 0;
int reconWifi24Count = 0;
int reconWifi5Count = 0;
int reconStrongestWifi = -127;
int reconBusiestChannel = 0;
int reconBusiestChannelCount = 0;
int reconBleCount = 0;
int reconStrongestBle = -127;

// Watch Mode / Rogue AP Watch
constexpr uint8_t WATCH_MAX_WIFI = 56;
constexpr uint8_t WATCH_MAX_BLE = 28;
constexpr uint8_t WATCH_MAX_ALERT_ROWS = 8;

WatchPhase watchPhase = WatchPhase::Idle;
lv_obj_t *watchStatusLabel = nullptr;
lv_obj_t *watchWifiLabel = nullptr;
lv_obj_t *watchBleLabel = nullptr;
lv_obj_t *watchAlertCountLabel = nullptr;
lv_obj_t *watchAlertList = nullptr;

String watchPrevWifiKeys[WATCH_MAX_WIFI];
String watchPrevWifiSsids[WATCH_MAX_WIFI];
uint8_t watchPrevWifiAuth[WATCH_MAX_WIFI] = {};
int16_t watchPrevWifiChannels[WATCH_MAX_WIFI] = {};
uint8_t watchPrevWifiCount = 0;
bool watchWifiBaselineReady = false;

String watchPrevBleAddrs[WATCH_MAX_BLE];
bool watchPrevBleTrackerLike[WATCH_MAX_BLE] = {};
uint8_t watchPrevBleCount = 0;
bool watchBleBaselineReady = false;

uint16_t watchAlertTotal = 0;
int watchCurrentWifiCount = 0;
int watchCurrentWifi24Count = 0;
int watchCurrentWifi5Count = 0;
int watchCurrentBleCount = 0;
uint32_t watchNextSweepMs = 0;
bool watchAlertsArmed = false;
uint32_t watchLastBeepMs = 0;

// Wardrive session
constexpr uint8_t WARD_MAX_WIFI = 56;
constexpr uint8_t WARD_MAX_BLE = 64;
WardrivePhase wardrivePhase = WardrivePhase::Stopped;
lv_timer_t *wardriveTimer = nullptr;
lv_obj_t *wardriveStatusLabel = nullptr;
lv_obj_t *wardriveWifiLabel = nullptr;
lv_obj_t *wardriveBleLabel = nullptr;
lv_obj_t *wardriveGpsLabel = nullptr;
lv_obj_t *wardriveElapsedLabel = nullptr;
lv_obj_t *wardriveLocationLabel = nullptr;
lv_obj_t *wardriveFeedScroll = nullptr;
lv_obj_t *wardriveFeedLabel = nullptr;
lv_obj_t *wardriveFeedStateLabel = nullptr;
lv_obj_t *wardriveStartLabel = nullptr;
lv_obj_t *wardriveSdLabel = nullptr;
bool wardriveRunning = false;
uint32_t wardriveLastStartStopMs = 0;
bool wardriveStopCleanupPending = false;
bool wardriveExitToReconPending = false;
bool wardriveOpenFromReconPending = false;
bool reconExitHomePending = false;

// SD logging is intentionally optional. A missing/unreadable card must never
// prevent the Watch, Recon, GPS, Wi-Fi, BLE, or Wardrive tools from running.
bool m33kSdReady = false;
bool wardriveSdLogging = false;
String wardriveLogPath;
uint32_t wardriveSdRowsWritten = 0;
uint32_t wardriveSessionStartedMs = 0;
uint32_t wardriveNextSweepMs = 0;
uint32_t wardriveLastElapsedSeconds = 0;
String wardriveWifiKeys[WARD_MAX_WIFI];
uint8_t wardriveWifiCount = 0;

// Independent fixed-memory session-total tracker. A Bloom filter avoids the
// old 192-device ceiling without growing heap usage during long drives.
constexpr uint16_t WARD_TOTAL_WIFI_BLOOM_BITS = 16384;
constexpr uint16_t WARD_TOTAL_WIFI_BLOOM_BYTES = WARD_TOTAL_WIFI_BLOOM_BITS / 8;
uint8_t wardriveTotalWifiBloom[WARD_TOTAL_WIFI_BLOOM_BYTES] = {};
uint32_t wardriveTotalWifiCount = 0;
bool wardriveMemoryPaused = false;
bool wardrivePageActive = false;
// Wardrive heap guard.
//
// Current post-C5 steady-state heap is commonly around 28-30 KB.
// The previous 28K pause / 32K resume thresholds could latch Wardrive
// permanently after a healthy C5 transaction or page re-entry.
//
// Keep a safety floor, but allow normal post-C5 recovery to resume scans.
constexpr uint32_t WARD_HEAP_PAUSE_BYTES = 24000U;
constexpr uint32_t WARD_HEAP_RESUME_BYTES = 27000U;
constexpr uint32_t WARD_LARGEST_BLOCK_PAUSE_BYTES = 24000U;
constexpr uint32_t WARD_LARGEST_BLOCK_RESUME_BYTES = 36000U;
bool wardriveC5CacheApplied = false;
uint32_t wardriveAppliedC5Generation = 0;
String wardriveBleAddrs[WARD_MAX_BLE];
uint8_t wardriveBleCount = 0;

// Full-session BLE unique tracker.
// wardriveBleAddrs[] remains only a bounded detail cache.
constexpr uint16_t WARD_TOTAL_BLE_BLOOM_BITS = 16384;
constexpr uint16_t WARD_TOTAL_BLE_BLOOM_BYTES =
    WARD_TOTAL_BLE_BLOOM_BITS / 8;

uint8_t wardriveTotalBleBloom[WARD_TOTAL_BLE_BLOOM_BYTES] = {};
uint32_t wardriveTotalBleCount = 0;
// Fixed six-row LVGL UI with a larger temporary Wardrive cache.
// Long-session history will later move to SD-backed storage.
constexpr uint16_t WARD_MAX_FEED_LINES = 120;
String wardriveSweepLines[WARD_MAX_FEED_LINES];
uint16_t wardriveSweepLineCount = 0;
bool wardriveFeedDirty = true;

constexpr uint8_t WARD_FEED_PAGE_ROWS = 6;
lv_obj_t *wardriveFeedRows[WARD_FEED_PAGE_ROWS] = {};
lv_obj_t *wardriveFeedPageLabel = nullptr;

// Five lightweight bunny-skull progress markers beside the
// Wardrive feed paging controls.
//
// Each marker is made from one head + two ears.
// No images, canvases, timers, or dynamic buffers are used.
constexpr uint8_t WARD_BUNNY_PROGRESS_COUNT = 5;
lv_obj_t *wardriveBunnyHeads[WARD_BUNNY_PROGRESS_COUNT] = {};
lv_obj_t *wardriveBunnyEarL[WARD_BUNNY_PROGRESS_COUNT] = {};
lv_obj_t *wardriveBunnyEarR[WARD_BUNNY_PROGRESS_COUNT] = {};
uint32_t wardriveFeedPageStart = 0;
bool wardriveFeedFollowLatest = true;

// SD-backed Wardrive feed.
// Six LVGL rows stay fixed; older entries can be read directly from SD.
constexpr size_t WARD_FEED_RECORD_BYTES = 32;
String wardriveFeedPath;
uint32_t wardriveFeedStoredCount = 0;

// Fixed-memory unique-device tracker for the SD feed.
constexpr uint16_t WARD_FEED_BLOOM_BITS = 16384;
constexpr uint16_t WARD_FEED_BLOOM_BYTES =
    WARD_FEED_BLOOM_BITS / 8;
uint8_t wardriveFeedBloom[WARD_FEED_BLOOM_BYTES] = {};

// Keep this moderate: WardriveRecord contains Strings.
constexpr uint16_t WARD_MAX_RECORDS = 96;
WardriveRecord wardriveRecords[WARD_MAX_RECORDS];
uint16_t wardriveRecordCount = 0;
bool wardriveFeedScrolling = false;
bool wardriveUiSyncPending = false;
uint32_t wardriveDetailIgnoreUntilMs = 0;
bool wardriveDetailOpen = false;
uint8_t wardriveUiSyncCursor = 0;
uint32_t wardriveLastDiagMs = 0;

// Hold display updates briefly after NimBLE work completes.
uint32_t wardriveDisplayResumeMs = 0;

lv_obj_t *wardriveDetailOverlay = nullptr;
lv_obj_t *wardriveDetailTitle = nullptr;
lv_obj_t *wardriveDetailBody = nullptr;

// SX1262 / 915 MHz radio monitor v2 (receive-only)
constexpr float M33K_RADIO_FREQ_MHZ = 915.0f;
constexpr uint8_t M33K_RADIO_SF_VALUES[] = {7, 8, 9, 10, 11, 12};
constexpr float M33K_RADIO_BW_VALUES[] = {125.0f, 250.0f, 500.0f};
constexpr uint8_t M33K_RADIO_SF_COUNT =
    sizeof(M33K_RADIO_SF_VALUES) / sizeof(M33K_RADIO_SF_VALUES[0]);
constexpr uint8_t M33K_RADIO_BW_COUNT =
    sizeof(M33K_RADIO_BW_VALUES) / sizeof(M33K_RADIO_BW_VALUES[0]);
constexpr uint32_t M33K_RADIO_AUTO_DWELL_MS = 3500;
constexpr uint8_t M33K_RADIO_HISTORY_MAX = 10;

volatile bool m33kRadioPacketFlag = false;
bool m33kRadioInitialized = false;
bool m33kRadioListening = false;
bool m33kRadioAutoScan = false;
uint8_t m33kRadioSfIndex = 2;
uint8_t m33kRadioBwIndex = 0;
// MeshCore "Listen" preset: tunes the receiver to the MeshCore US/Canada LoRa
// radio parameters so it actually hears MeshCore traffic. Still receive-only;
// this does NOT decode/decrypt or join the mesh (that is a separate project).
bool m33kRadioMeshCore = false;
constexpr float   M33K_MESHCORE_FREQ_MHZ = 910.525f;  // US/Canada preset
constexpr float   M33K_MESHCORE_BW_KHZ   = 62.5f;
constexpr uint8_t M33K_MESHCORE_SF       = 7;
constexpr uint8_t M33K_MESHCORE_CR       = 5;          // 4/5
constexpr uint8_t M33K_MESHCORE_SYNC     = 0x12;       // RadioLib private default
uint32_t m33kRadioNextProfileMs = 0;
uint32_t m33kRadioPacketCount = 0;
uint32_t m33kRadioLastPacketMs = 0;
float m33kRadioLastRssi = -127.0f;
float m33kRadioLastSnr = 0.0f;
float m33kRadioLastFreqError = 0.0f;
size_t m33kRadioLastLength = 0;

String m33kRadioHistory[M33K_RADIO_HISTORY_MAX];
uint8_t m33kRadioHistoryCount = 0;
bool m33kRadioHistoryDirty = true;

lv_obj_t *radioStatusLabel = nullptr;
lv_obj_t *radioChipLabel = nullptr;
lv_obj_t *radioFreqLabel = nullptr;
lv_obj_t *radioModeLabel = nullptr;
lv_obj_t *radioPacketLabel = nullptr;
lv_obj_t *radioSignalLabel = nullptr;
lv_obj_t *radioLastLabel = nullptr;
lv_obj_t *radioProfileLabel = nullptr;
lv_obj_t *radioPayloadLabel = nullptr;
lv_obj_t *radioListenLabel = nullptr;
lv_obj_t *radioSfButtonLabel = nullptr;
lv_obj_t *radioBwButtonLabel = nullptr;
lv_obj_t *radioAutoButtonLabel = nullptr;
lv_obj_t *radioMeshButtonLabel = nullptr;

// Recon Radar
constexpr uint8_t MAX_RADAR_BLIPS = 12;
RadarBlip radarBlips[MAX_RADAR_BLIPS];
uint8_t radarBlipCount = 0;
RadarPhase radarPhase = RadarPhase::Idle;
lv_timer_t *radarTimer = nullptr;
lv_obj_t *radarStatusLabel = nullptr;
lv_obj_t *radarInfoLabel = nullptr;
lv_obj_t *radarSweepDots[9] = {};
uint16_t radarSweepAngle = 0;
constexpr uint32_t RADAR_RESCAN_MS = 4200;
// Keep a signal visible across refreshes. If it misses a sweep it fades;
// remove it only after it has not been observed for a while.
constexpr uint32_t RADAR_STALE_MS = 15000;
uint32_t radarNextScanMs = 0;

// Recon Pulse: passive Wi-Fi change radar
constexpr uint8_t PULSE_MAX_APS = 10;
constexpr uint8_t PULSE_HISTORY_COUNT = 24;
constexpr uint32_t PULSE_SWEEP_INTERVAL_MS = 2600;
constexpr uint32_t PULSE_FADE_MS = 8000;

PulseAp pulseAps[PULSE_MAX_APS];
PulsePhase pulsePhase = PulsePhase::Idle;
lv_timer_t *pulseTimer = nullptr;
lv_obj_t *pulseStatusLabel = nullptr;
lv_obj_t *pulseInfoLabel = nullptr;
lv_obj_t *pulseNewLabel = nullptr;
lv_obj_t *pulseLostLabel = nullptr;
lv_obj_t *pulseMoveLabel = nullptr;
lv_obj_t *pulseGraphNewBars[PULSE_HISTORY_COUNT] = {};
lv_obj_t *pulseGraphLostBars[PULSE_HISTORY_COUNT] = {};
lv_obj_t *pulseGraphMoveBars[PULSE_HISTORY_COUNT] = {};
lv_obj_t *pulseSweepDots[7] = {};
uint8_t pulseHistoryNew[PULSE_HISTORY_COUNT] = {};
uint8_t pulseHistoryLost[PULSE_HISTORY_COUNT] = {};
uint8_t pulseHistoryMove[PULSE_HISTORY_COUNT] = {};
uint8_t pulseLastNew = 0;
uint8_t pulseLastLost = 0;
uint8_t pulseLastMoved = 0;
uint16_t pulseSweepAngle = 0;
uint32_t pulseNextSweepMs = 0;

// Signal Hunter
constexpr uint8_t HUNTER_HISTORY_COUNT = 24;
HunterKind hunterKind = HunterKind::None;
HunterPhase hunterPhase = HunterPhase::Idle;
HunterFeedbackMode hunterFeedbackMode = HunterFeedbackMode::Off;
lv_timer_t *hunterTimer = nullptr;
lv_obj_t *hunterStatusLabel = nullptr;
lv_obj_t *hunterList = nullptr;
lv_obj_t *hunterTrackingPanel = nullptr;
lv_obj_t *hunterTargetLabel = nullptr;
lv_obj_t *hunterRssiLabel = nullptr;
lv_obj_t *hunterStrengthLabel = nullptr;
lv_obj_t *hunterSignalBar = nullptr;
lv_obj_t *hunterModeLabel = nullptr;
lv_obj_t *hunterHistoryBars[HUNTER_HISTORY_COUNT] = {};
String hunterTargetName;
String hunterTargetId;
int hunterCurrentRssi = -127;
int hunterBestRssi = -127;
int hunterWeakestRssi = 0;
int hunterHistory[HUNTER_HISTORY_COUNT] = {};
uint8_t hunterHistoryUsed = 0;
bool hunterWifiScanInFlight = false;
bool hunterBleScanInFlight = false;
bool hunterTrackerSelectionMode = false;
bool hunterTargetFromC5 = false;
uint32_t hunterAppliedC5Generation = 0;
uint32_t hunterNextScanMs = 0;
uint32_t hunterLastFeedbackMs = 0;
uint32_t hunterLastSeenMs = 0;

// Speaker tone for Signal Hunter
// Confirmed working on this T-Watch Ultra by raw I2S hardware test:
// 48 kHz / 16-bit / stereo works; 160 kHz is silent.
constexpr int HUNTER_AUDIO_RATE = 48000;
constexpr int HUNTER_BEEP_MS = 90;
constexpr int HUNTER_BEEP_FRAMES =
    HUNTER_AUDIO_RATE * HUNTER_BEEP_MS / 1000;
constexpr int HUNTER_BEEP_SAMPLES =
    HUNTER_BEEP_FRAMES * 2; // interleaved stereo L/R
constexpr int HUNTER_SPK_BCLK = 9;
constexpr int HUNTER_SPK_WCLK = 10;
constexpr int HUNTER_SPK_DOUT = 11;
int16_t hunterBeepBuffer[HUNTER_BEEP_SAMPLES];
bool hunterBeepReady = false;
bool hunterAudioConfigured = false;

// GPS / GNSS page
TinyGPSPlus gpsParser;
bool gpsToolActive = true;
// True while the GNSS rail has been powered down for deep-idle battery
// saving, so wakeScreen() knows to turn it back on.
bool gpsDeepIdleOff = false;
bool gpsSessionLocationSeen = false;
uint32_t gpsSessionBytes = 0;
uint32_t gpsPageOpenedMs = 0;
uint32_t gpsLastByteMs = 0;
bool gpsRecoveryAttempted = false;

lv_obj_t *gpsStatusLabel = nullptr;
lv_obj_t *gpsSatLabel = nullptr;
lv_obj_t *gpsLatLabel = nullptr;
lv_obj_t *gpsLonLabel = nullptr;
lv_obj_t *gpsAltLabel = nullptr;
lv_obj_t *gpsSpeedLabel = nullptr;
lv_obj_t *gpsHdopLabel = nullptr;
lv_obj_t *gpsCourseLabel = nullptr;
lv_obj_t *gpsUtcLabel = nullptr;
lv_obj_t *gpsDataLabel = nullptr;

// Offline map page shell. The GNSS parser remains shared with the existing
// GPS page; the map never opens a second UART/GPS session.
lv_obj_t *mapStatusLabel = nullptr;
lv_obj_t *mapCoordLabel = nullptr;
lv_obj_t *mapZoomLabel = nullptr;
lv_obj_t *mapSdLabel = nullptr;
lv_obj_t *mapMarkerLabel = nullptr;
lv_obj_t *mapMarkerDot = nullptr;
lv_obj_t *mapViewport = nullptr;
lv_obj_t *mapTileCanvas = nullptr;
lv_obj_t *mapTilePlaceholder = nullptr;
constexpr uint8_t MAP_VECTOR_LABEL_COUNT = 10;
lv_obj_t *mapVectorLabels[MAP_VECTOR_LABEL_COUNT] = {};
String mapVectorLabelText[MAP_VECTOR_LABEL_COUNT];
int16_t mapVectorLabelX[MAP_VECTOR_LABEL_COUNT] = {};
int16_t mapVectorLabelY[MAP_VECTOR_LABEL_COUNT] = {};
uint8_t mapVectorLabelPriority[MAP_VECTOR_LABEL_COUNT] = {};
lv_obj_t *mapGridVertical[7] = {};
lv_obj_t *mapGridHorizontal[7] = {};

constexpr uint16_t MAP_TILE_SIZE = 256;
constexpr size_t MAP_TILE_BYTES =
    static_cast<size_t>(MAP_TILE_SIZE) * MAP_TILE_SIZE * 2U;
constexpr uint16_t MAP_VIEW_WIDTH = 350;
constexpr uint16_t MAP_VIEW_HEIGHT = 292;
constexpr size_t MAP_VIEW_BYTES =
    static_cast<size_t>(MAP_VIEW_WIDTH) * MAP_VIEW_HEIGHT * 2U;
constexpr int MAP_RELOAD_PIXEL_THRESHOLD = 16;
constexpr const char *MAP_VECTOR_PROTOTYPE_PATH =
    "/M33KX/VECTOR/prototype.m33kv";
constexpr uint8_t MAP_VECTOR_V2_CHUNK_ZOOM = 12;
constexpr const char *MAP_VECTOR_V2_ROOT =
    "/M33KX/VECTOR2";

uint8_t *mapTileBuffer = nullptr;
int32_t mapLoadedTileX = -1;
int32_t mapLoadedTileY = -1;
int8_t mapLoadedTileZ = -1;
int32_t mapRequestedTileX = -1;
int32_t mapRequestedTileY = -1;
int8_t mapRequestedTileZ = -1;
int32_t mapLoadedWorldX = -1;
int32_t mapLoadedWorldY = -1;
int32_t mapRequestedWorldX = -1;
int32_t mapRequestedWorldY = -1;
double mapRequestedLat = 0.0;
double mapRequestedLon = 0.0;
bool mapTileReloadPending = false;
bool mapTileLoaded = false;


uint8_t mapZoomLevel = 16;
bool mapOpenPending = false;
bool mapBackToGpsPending = false;

// ST25R3916 NFC-A reader. The RF field is powered only while the NFC page is
// open; detections are serviced by the LVGL timer on the main UI thread.
lv_obj_t *nfcStatusLabel = nullptr;
lv_obj_t *nfcTypeLabel = nullptr;
lv_obj_t *nfcUidLabel = nullptr;
lv_obj_t *nfcTechLabel = nullptr;
lv_obj_t *nfcCountLabel = nullptr;
bool nfcReaderReady = false;
bool nfcTagPending = false;
bool nfcDiscoveryActive = false;
uint32_t nfcDiscoveryStartedMs = 0;
uint32_t nfcNextDiscoveryMs = 0;
uint32_t nfcLastRawDetectionMs = 0;
uint32_t nfcLastDiagUpdateMs = 0;
uint32_t nfcWorkerCalls = 0;
int nfcInitCode = 0;
int nfcDiscoverCode = 0;
uint16_t nfcTagCount = 0;
String nfcDetectedUid;
String nfcDetectedType;
String nfcDetectedTech;
#ifdef USING_ST25R3916
rfalNfcDiscoverParam nfcDiscoverParams = {};
#endif

// Live home UI
lv_obj_t *batteryLabel = nullptr;

// Compact battery indicator used on tool pages.
lv_obj_t *pageBatteryLabel = nullptr;
uint32_t lastPageBatteryUpdateMs = 0;

lv_obj_t *homeWifiIcon = nullptr;
lv_obj_t *homeWifiDot = nullptr;
lv_obj_t *homeC5Label = nullptr;
lv_obj_t *homeC5Dot = nullptr;
lv_obj_t *hourLabel = nullptr;
lv_obj_t *colonLabel = nullptr;
lv_obj_t *minuteLabel = nullptr;
lv_obj_t *dateTopLabel = nullptr;
lv_obj_t *dateBottomLabel = nullptr;

lv_obj_t *hourGlow[16] = {};
lv_obj_t *colonGlow[16] = {};
lv_obj_t *minuteGlow[16] = {};
lv_obj_t *dateTopGlow[4] = {};
lv_obj_t *dateBottomGlow[4] = {};

// Wi-Fi page
lv_obj_t *wifiStatusLabel = nullptr;
lv_obj_t *wifiList = nullptr;
lv_obj_t *wifiGraphContainer = nullptr;
lv_obj_t *wifiGraphLegend = nullptr;
bool wifiGraphView = false;
bool wifiSubpageTransition = false;

enum class WifiPendingNav : uint8_t {
    None,
    OpenGraph,
    BackToScanner,
    ExitHome
};

WifiPendingNav wifiPendingNav = WifiPendingNav::None;
bool wifiScanning = false;
uint32_t wifiLastCompleteMs = 0;

// Live Wi-Fi channel graph.
constexpr uint8_t WIFI_GRAPH_MAX_DROPS = 56;

// Near-continuous graph scanning. A new scan begins shortly after the
// previous asynchronous sweep completes; scans themselves cannot overlap.
constexpr uint32_t WIFI_GRAPH_RESCAN_MS = 120;
constexpr uint16_t WIFI_GRAPH_SCAN_DWELL_MS = 150;

constexpr uint32_t WIFI_DROP_FALL_MS = 560;
constexpr uint32_t WIFI_DROP_HOLD_MS = 140;
constexpr uint32_t WIFI_DROP_FADE_MS = 360;

struct WifiGraphDrop {
    bool active = false;
    lv_obj_t *obj = nullptr;
    lv_obj_t *silhouette = nullptr;
    int16_t startY = 22;
    int16_t targetY = 162;
    uint32_t bornMs = 0;
    uint32_t delayMs = 0;
};

WifiGraphDrop wifiGraphDrops[WIFI_GRAPH_MAX_DROPS];
bool wifiGraphLive = false;
uint32_t wifiGraphNextScanMs = 0;
uint32_t wifiGraphRenderedSignature = 0;
bool wifiGraphHasRender = false;

constexpr uint8_t MAX_WIFI_DETAILS = 56;
WifiResultDetail wifiDetails[MAX_WIFI_DETAILS];
uint8_t wifiDetailCount = 0;
uint8_t wifiLocalDetailCount = 0;

// XIAO ESP32-C5 dual-band companion. The watch remains responsible for
// 2.4 GHz scanning; M33K X C5 supplies 5 GHz scan rows over this private BLE
// service. Discovery uses the service UUID; trust is established only after the authenticated challenge/response handshake.
static constexpr char C5_SERVICE_UUID[] = "7d8a1000-6d33-4b33-a33c-6d33336b4335";
static constexpr char C5_COMMAND_UUID[] = "7d8a1001-6d33-4b33-a33c-6d33336b4335";
static constexpr char C5_DATA_UUID[] = "7d8a1002-6d33-4b33-a33c-6d33336b4335";
static constexpr char C5_STATUS_UUID[] = "7d8a1003-6d33-4b33-a33c-6d33336b4335";
// Per-device C5 link key. Fresh public firmware contains no shared secret.
// The Watch creates a random 256-bit key on first enrollment and stores it in
// NVS; the C5 stores the same key in its own NVS.
uint8_t c5LinkKey[32] = {};
bool c5LinkKeyLoaded = false;
constexpr uint8_t MAX_C5_WIFI_DETAILS = 28;
WifiResultDetail c5WifiDetails[MAX_C5_WIFI_DETAILS];
volatile uint8_t c5WifiDetailCount = 0;
WifiResultDetail c5WifiPendingDetails[MAX_C5_WIFI_DETAILS];
volatile uint8_t c5WifiPendingDetailCount = 0;
volatile bool c5ScanInFlight = false;
volatile bool c5ScanComplete = false;
volatile uint32_t c5ScanGeneration = 0;
uint32_t wifiAppliedC5Generation = 0;
uint32_t c5ScanRequestedMs = 0;
uint32_t c5NextConnectAttemptMs = 0;
bool c5DiscoveryInFlight = false;
bool c5DiscoveryRequestedByWifi = false;
bool wifiInitialC5LinkPending = false;
uint32_t c5DiscoveryStartedMs = 0;
constexpr uint32_t C5_DISCOVERY_DURATION_MS = 3000;
constexpr uint32_t C5_DISCOVERY_RETRY_MS = 12000;
String c5StatusText = "OFFLINE";
NimBLEClient *c5Client = nullptr;
NimBLERemoteCharacteristic *c5CommandChar = nullptr;
NimBLERemoteCharacteristic *c5DataChar = nullptr;
NimBLERemoteCharacteristic *c5StatusChar = nullptr;
volatile bool c5Authenticated = false;
String c5AuthChallengeHex = "";
bool c5AuthResponseSent = false;
uint32_t c5AuthStartedMs = 0;
constexpr uint32_t C5_AUTH_TIMEOUT_MS = 10000;

// NimBLE callbacks execute on the BLE host task, not the Arduino/LVGL task.
// Never mutate Arduino String objects, Wi-Fi/C5 caches, or Preferences from
// those callbacks. Queue compact events and apply them from loop() instead.
enum class C5EventType : uint8_t {
    Connect,
    Disconnect,
    Data,
    Status
};

struct C5Event {
    C5EventType type = C5EventType::Data;
    int reason = 0;
    char text[192] = {};
};

QueueHandle_t c5EventQueue = nullptr;

lv_obj_t *wifiDetailOverlay = nullptr;
lv_obj_t *wifiDetailTitle = nullptr;
lv_obj_t *wifiDetailBody = nullptr;

// BLE page
constexpr uint8_t MAX_BLE_DETAILS = 40;
BleResultDetail bleDetails[MAX_BLE_DETAILS];
uint8_t bleDetailCount = 0;

lv_obj_t *bleStatusLabel = nullptr;
lv_obj_t *bleList = nullptr;
lv_obj_t *bleDetailOverlay = nullptr;
lv_obj_t *bleDetailTitle = nullptr;
lv_obj_t *bleDetailBody = nullptr;

// Animated m33k t3ch character on the BLE page (behind the device list).
lv_obj_t *bleCat = nullptr;
lv_obj_t *blePingRing = nullptr;
lv_timer_t *bleAnimTimer = nullptr;
int bleAnimStep = 0;
uint32_t bleAnimStepStart = 0;
int bleWalkSub = 0;
uint32_t bleWalkSubStart = 0;
int16_t blePingCx = 205;
int16_t blePingCy = 300;

volatile bool bleScanFinished = false;
volatile int bleScanEndReason = 0;
bool bleScannerInitialized = false;

// Phone/web Companion is intentionally disabled in this development build.
// Future direction: M33K X Dashboard / app integration.

// Settings / time UI
lv_obj_t *settingsStatusLabel = nullptr;
lv_obj_t *settingsTzDropdown = nullptr;
lv_obj_t *settingsFormatDropdown = nullptr;
lv_obj_t *settingsSsidInput = nullptr;
lv_obj_t *settingsWifiStateLabel = nullptr;
lv_obj_t *settingsStaySwitch = nullptr;
lv_obj_t *settingsPasswordInput = nullptr;
lv_obj_t *settingsPasswordCountLabel = nullptr;
lv_obj_t *settingsKeyboard = nullptr;
lv_obj_t *settingsKeyboardShade = nullptr;
lv_obj_t *settingsContent = nullptr;
lv_obj_t *settingsAvailableDropdown = nullptr;
lv_obj_t *settingsShowPasswordSwitch = nullptr;
lv_obj_t *settingsBrightnessSlider = nullptr;
lv_obj_t *settingsBrightnessValueLabel = nullptr;

constexpr uint8_t MAX_CONNECT_SSIDS = 20;
String availableSsids[MAX_CONNECT_SSIDS];
uint8_t availableSsidCount = 0;

lv_obj_t *manualStatusLabel = nullptr;
lv_obj_t *manualYearSpin = nullptr;
lv_obj_t *manualMonthSpin = nullptr;
lv_obj_t *manualDaySpin = nullptr;
lv_obj_t *manualHourSpin = nullptr;
lv_obj_t *manualMinuteSpin = nullptr;

// Display inactivity
uint32_t lastActivityMs = 0;
uint32_t ignoreClicksUntil = 0;
bool screenSleeping = false;

// Screen lock. When locked, all dock/tool taps are suppressed (via
// clickAllowed) so a stray touch can't launch a tool. A long press toggles
// the lock; the screen also auto-locks whenever it sleeps.
bool screenLocked = false;
constexpr uint32_t LOCK_LONGPRESS_MS = 650;
uint32_t touchDownMs = 0;
bool touchWasDown = false;
bool lockPressHandled = false;
lv_obj_t *lockIndicator = nullptr;
uint32_t lastWifiUiUpdateMs = 0;

// Bottom custom side button on the T-Watch Ultra is GPIO0.
constexpr uint8_t M33K_BACK_BUTTON_PIN = 0;
bool backButtonWasDown = false;
uint32_t backButtonLastChangeMs = 0;
constexpr uint32_t BACK_BUTTON_DEBOUNCE_MS = 45;

class M33KBleScanCallbacks : public NimBLEScanCallbacks {
public:
    void onScanEnd(const NimBLEScanResults &results, int reason) override
    {
        LV_UNUSED(results);
        bleScanEndReason = reason;
        bleScanFinished = true;
    }
};

M33KBleScanCallbacks bleScanCallbacks;

// Early forward declarations used by Recon sub-tool callbacks.
void clearScreen();
lv_obj_t *createSafeHeaderBack(lv_event_cb_t callback);
lv_obj_t *createReconTile(
    int x,
    int y,
    int w,
    int h,
    uint32_t borderColor,
    lv_obj_t **labelOut,
    const char *text,
    uint32_t textColor,
    uint8_t bgOpacity = 230
);

void showReconPage();
void showReconPulsePage();
void showRadarPage();
void showSignalHunterPage();
void showWatchModePage();
void showWardrivePage();
void showLogsPage();
void showRadioPage();
void showNfcPage();
void stopWatchTool();

// ------------------------------------------------------------
// Wardrive radio time-sharing.
//
// Keep GPS + local 2.4 GHz running normally.
//
// C5:
//   connect -> one 5 GHz sweep -> consume -> disconnect
//
// Watch BLE:
//   short sample only while C5 transport is idle
//
// This prevents the live C5 NimBLE connection from starving the
// display's internal DMA-capable memory.
// ------------------------------------------------------------
bool wardriveC5CycleActive = false;
uint32_t wardriveC5CycleStartedMs = 0;
uint32_t wardriveC5RequestGeneration = 0;
uint32_t wardriveNextC5CycleMs = 0;
uint32_t wardriveNextBleSampleMs = 0;

constexpr uint32_t WARD_C5_INTERVAL_MS = 30000UL;
constexpr uint32_t WARD_C5_TIMEOUT_MS = 18000UL;
constexpr uint32_t WARD_BLE_INTERVAL_MS = 18000UL;

void stopWardriveTool();
void stopRadioTool();
void showGamePage();
void stopGameTool();
void stopNfcTool();
void stopReconPulseTool();
void resetWifiGraphDrops();
void captureLocalWifiResults(int16_t count);
void mergeC5WifiResults();
void finishWatchBleScan();
void finishWardriveBleScan();
void playHunterBeep();
void ensureGpsReady();
const char *authLabel(wifi_auth_mode_t auth);
const char *signalLabel(int32_t rssi);
void wardriveDetailEvent(lv_event_t *e);
void wardriveDetailCloseEvent(lv_event_t *e);
void wardriveFeedScrollEvent(lv_event_t *e);
void syncWardriveRecordRows();
void initWardriveRecordRows();

// Used by Watch Mode before the detector function body appears later.
bool isAppleFindMyTrackerLike(const NimBLEAdvertisedDevice *dev);

// XIAO C5 link helper.
bool c5IsLinked();

// Used by both the normal BLE target list and the tracker-like BLE list.
void hunterBleTargetEvent(lv_event_t *e);

const char *MONTHS[] = {
    "JAN", "FEB", "MAR", "APR", "MAY", "JUN",
    "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"
};

const char *WEEKDAYS[] = {
    "SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY",
    "THURSDAY", "FRIDAY", "SATURDAY"
};

// ----------------------------------------------------------------------------
// Home watch faces: 0 = Original (unchanged), 1 = Terminal, 2 = M33K Crew,
// 3 = Video. Swipe left/right on Home to cycle; the choice persists in NVS.
// ----------------------------------------------------------------------------
uint8_t homeFaceIndex = 0;
constexpr uint8_t HOME_FACE_COUNT = 7;
lv_timer_t *faceAnimTimer = nullptr;
lv_obj_t *faceClockHour = nullptr, *faceClockColon = nullptr,
         *faceClockMin = nullptr, *faceClockDate = nullptr;
lv_obj_t *termTime = nullptr, *termDate = nullptr, *termBatt = nullptr,
         *termCursor = nullptr;
lv_obj_t *crewMain = nullptr, *crewCatBody = nullptr, *crewCatTail = nullptr,
         *crewMonBody = nullptr, *crewMonTail = nullptr;
lv_obj_t *videoImg = nullptr;
int videoFrameIdx = 0;
int crewCatBaseX = 0, crewCatBaseY = 0, crewMonBaseX = 0, crewMonBaseY = 0;
uint32_t videoFrameMs = 0, termCursorMs = 0, faceAnimStartMs = 0,
         lastFaceSwitchMs = 0;
bool termCursorOn = true;

// Radar face (face 6): a settled m33k t3ch frame with live overlays - the two
// scope eyes get a sweep hand plus spinning rings, a cycling readout animates
// between the eyes, and the time shows as sideways neon-green 7-segment digits.
lv_obj_t *radarFaceImg = nullptr, *radarEyeL = nullptr, *radarEyeR = nullptr;
lv_obj_t *radarEyeLGlow = nullptr, *radarEyeRGlow = nullptr;   // soft pink halo under each sweep
lv_obj_t *radarRing[4] = {};           // spinning arcs: L-outer,L-inner,R-outer,R-inner
lv_obj_t *radarRingGlow[4] = {};       // soft wide halo under each ring
int       radarRingAng[4] = {};
lv_obj_t *radarCenterTxt = nullptr;    // cycling HUD readout between the eyes
lv_obj_t *radarVTime = nullptr, *radarVMeta = nullptr;
lv_obj_t *radarSeg[4][7] = {};         // 7-seg digits: H-tens,H-ones,M-tens,M-ones
lv_obj_t *radarMetaDay = nullptr, *radarMetaDate = nullptr;
uint32_t radarClockMs = 0, radarRingMs = 0, radarTxtMs = 0;
int radarSweepAng = 0;                 // 0.1-degree units for the eye sweeps
static lv_point_precise_t radarEyeLPts[2];
static lv_point_precise_t radarEyeRPts[2];
// Neon green used across the radar face clock/date.
constexpr uint32_t RADAR_NEON = 0x39FF14;
// Eye centers (measured on the settled frame) and sweep radius.
constexpr int RADAR_LX = 63, RADAR_LY = 250, RADAR_RX = 342, RADAR_RY = 250, RADAR_ER = 72;

// ---- M33K Runner game (Page::Game) ----
lv_timer_t *gameTimer = nullptr;
enum class GameState : uint8_t { Pick, Play, Dead };
GameState gameState = GameState::Pick;
lv_obj_t *gameBg = nullptr, *gameGroundLine = nullptr, *gamePlayer = nullptr;
lv_obj_t *gameScoreLbl = nullptr, *gameBestLbl = nullptr;
lv_obj_t *gamePickPanel = nullptr, *gamePickTitle = nullptr, *gamePickHint = nullptr, *gameOverLbl = nullptr;
lv_obj_t *gamePickBtn[4] = {};
constexpr int GAME_OBS = 10;
struct GameObs { lv_obj_t *img; bool active; bool air; float x, w, h; int bob, oy; };
GameObs gameObs[GAME_OBS] = {};
int   gameSel = 0, gameBest = 0, gameShake = 0, gameFrame = 0;
float gamePy = 0, gameVy = 0, gameSpd = 280, gameDist = 0, gameSpawnIn = 0.9f, gamePw = 40, gamePh = 58;
bool  gameOnGround = true, gameLastAir = false;
uint32_t gameAnimMs = 0;
constexpr int   GAME_GROUND_Y = 430;
constexpr float GAME_GRAV = 2600.0f, GAME_JUMP = 860.0f, GAME_FLAP = 480.0f;
// Parallax skyline + stars to fill the sky above the runner.
constexpr int GAME_BLD = 7, GAME_STAR = 9;
lv_obj_t *gameBld[GAME_BLD] = {}, *gameStar[GAME_STAR] = {};
float gameBldX[GAME_BLD] = {};
// Wi-Fi / BLE signal bursts popping in the sky.
constexpr int GAME_POP = 3;
lv_obj_t *gamePop[GAME_POP] = {};
float gamePopT[GAME_POP] = { -1.0f, -1.0f, -1.0f };
int gamePopX[GAME_POP] = {}, gamePopY[GAME_POP] = {};
uint32_t gamePopNextMs = 0;
// Blasts the runner fires while airborne, to pop the flying bubbles.
constexpr int GAME_BLAST = 4;
lv_obj_t *gameBlast[GAME_BLAST] = {};
float gameBlastX[GAME_BLAST] = {}, gameBlastY[GAME_BLAST] = {};
bool  gameBlastActive[GAME_BLAST] = {};
// Explosions when a bubble is popped (red / white / black flash).
constexpr int GAME_EXPL = 4;
lv_obj_t *gameExpl[GAME_EXPL] = {};
float gameExplT[GAME_EXPL] = { -1.0f, -1.0f, -1.0f, -1.0f };
int gameExplX[GAME_EXPL] = {}, gameExplY[GAME_EXPL] = {};

// ---- Wardrive mini-game (second mode on the Game page) ----
// Top-down: drive a character car around collecting Wi-Fi / BLE signals while
// dodging roaming "malware". gameMode: 0 = M33K Runner, 1 = Wardrive.
int gameMode = 0;
lv_obj_t *gameModeBtn[2] = {};
lv_obj_t *driveCar = nullptr, *driveBg = nullptr, *driveHead = nullptr;
float carX = 205, carY = 250, carHeading = 0, carSpeed = 150;
float carTargetX = 205, carTargetY = 120;
bool  carSteer = false;
constexpr int DRV_PICK = 6;                 // signal pickups on the map
lv_obj_t *drvPick[DRV_PICK] = {};
float drvPickX[DRV_PICK] = {}, drvPickY[DRV_PICK] = {};
bool  drvPickBle[DRV_PICK] = {};
constexpr int DRV_ENE = 6;                  // malware enemies
lv_obj_t *drvEne[DRV_ENE] = {};
float drvEneX[DRV_ENE] = {}, drvEneY[DRV_ENE] = {}, drvEneVx[DRV_ENE] = {}, drvEneVy[DRV_ENE] = {};
bool  drvEneActive[DRV_ENE] = {};
int   driveScore = 0, driveBest = 0, driveActiveEne = 2, driveSignals = 0;
float driveTime = 0;
uint32_t drvPulseMs = 0;
// Per-character car trim colors (body stays dark).
constexpr uint32_t DRV_TRIM[4] = { 0x3AEFFF, 0x39FF14, 0xFFB83A, 0xFF4AD5 };
void driveStart(int sel);
void driveDie();

// Matrix-rain tuning the faces override before createMatrixBackground():
// per-frame horizontal drift (diagonal rain) and the two glyph colors.
int matrixDriftX = 0;
uint32_t matrixColA = 0x3AEFFF, matrixColB = 0xFF4AD5;
// Per-face overrides (0 = use defaults): recolor the matrix bunny skulls, and
// tint the dock icons. Faces set these before createMatrixBackground/createDock.
uint32_t matrixSkullRecolor   = 0;
uint32_t dockIconColorOverride = 0;

// Bubbles face: character portraits float UP in big neon bubbles, while a
// field of small faint bubbles drifts DOWN behind them.
struct BubbleEntity {
    lv_obj_t *obj;
    lv_obj_t *glowL, *glowR;   // pulsing eye-glow overlays (null if none)
    float x, y;        // top-left position (float for smooth motion)
    float speed;       // pixels per tick (up for chars, down for background)
    float phase;       // horizontal-sway phase
    float swayAmp;     // horizontal-sway amplitude
    int size;
};
constexpr int BUBBLE_COUNT    = 5;    // character bubbles (rise), all unique
constexpr int BG_BUBBLE_COUNT = 10;   // small background bubbles (fall)
BubbleEntity gBubbles[BUBBLE_COUNT] = {};
BubbleEntity gBgBubbles[BG_BUBBLE_COUNT] = {};
void bubbleRespawn(int i, bool initial);
// Swipe-to-switch tracking (manual, more reliable than LVGL gestures).
int16_t swipeStartX = 0, swipeStartY = 0;
int16_t swipeLastX = 0, swipeLastY = 0;
uint32_t swipeStartMs = 0;
uint32_t swipeLostMs = 0;   // when touch first went absent mid-drag (0 = present)
bool swipeTracking = false;

// ----------------------------------------------------------------------------
// Weather face (home face). Pulls live weather from Open-Meteo over the saved
// Wi-Fi link. Location comes from GPS when there is a fix, otherwise from a
// runtime IP-geolocation lookup -- NO coordinates or personal data live in this
// source, so the public firmware ships clean.
// ----------------------------------------------------------------------------
enum class WeatherState : uint8_t { Idle, Updating, Ready, NoWifi, Error };
WeatherState weatherState = WeatherState::Idle;
float    weatherTempF = 0, weatherHiF = 0, weatherLoF = 0;
int      weatherCode = -1;
bool     weatherIsDay = true;
String   weatherPlace = "";
String   weatherUpdatedStr = "";   // "MM/DD HH:MM" of last successful fetch
bool     weatherHaveData = false;  // true once we have any reading (RAM or NVS)
uint32_t weatherLastFetchMs = 0;
bool     weatherFetchPending = false;
lv_obj_t *weatherTempLabel = nullptr, *weatherCondLabel = nullptr,
         *weatherHiLoLabel = nullptr, *weatherPlaceLabel = nullptr,
         *weatherStatusLabel = nullptr, *weatherIcon = nullptr;
constexpr uint32_t WEATHER_REFRESH_MS = 15UL * 60UL * 1000UL;  // 15 minutes
// Animated weather backdrop: cycles storm/lightning -> sunshine and back.
lv_obj_t *wxBg = nullptr, *wxFlash = nullptr, *wxSun = nullptr, *wxBolt = nullptr;
lv_obj_t *wxMtn = nullptr, *wxCity = nullptr;   // mountains (sun) / skyline (storm)
lv_obj_t *wxCloud1 = nullptr, *wxCloud2 = nullptr, *wxBird = nullptr;
float wxCloud1X = 0, wxCloud2X = 0, wxBirdX = 0;
int wxBirdY = 128;
uint32_t wxBirdFlapMs = 0;
bool wxBirdWingsUp = false;
lv_obj_t *weekCells[7] = {};                    // day-of-week strip
lv_obj_t *weekDateLabel = nullptr;
uint32_t wxNextFlashMs = 0, wxFlashUntil = 0;
// Wind (mph) from the fetch; speeds up clouds/rain and tilts rain when gusty.
float    weatherWindMph = 0;
// Condition-driven backdrop extras. The scene is chosen from weather_code +
// day/night each tick, and these elements fade in only for their condition.
constexpr int WX_RAIN_N = 14, WX_SNOW_N = 14, WX_STAR_N = 10;
lv_obj_t *wxMoon = nullptr, *wxFog1 = nullptr, *wxFog2 = nullptr;
lv_obj_t *wxStars[WX_STAR_N] = {};
lv_obj_t *wxRain[WX_RAIN_N]  = {};
lv_obj_t *wxSnow[WX_SNOW_N]  = {};
float wxRainX[WX_RAIN_N] = {}, wxRainY[WX_RAIN_N] = {}, wxRainSpd[WX_RAIN_N] = {};
float wxSnowX[WX_SNOW_N] = {}, wxSnowY[WX_SNOW_N] = {}, wxSnowSpd[WX_SNOW_N] = {}, wxSnowPh[WX_SNOW_N] = {};
float wxFog1X = 0, wxFog2X = 0;

void updateFaceClock();  // defined with the face builders below

void saveHomeFace()
{
    prefs.putUChar("face", homeFaceIndex);
}

void loadPersistentSettings()
{
    timezoneIndex = prefs.getUChar("tz", TIMEZONE_UNSET);
    if (timezoneIndex >= TIMEZONE_COUNT) timezoneIndex = TIMEZONE_UNSET;

    use24Hour = prefs.getBool("24h", false);
    wifiStayConnected = prefs.getBool("wstay", false);

    displayBrightnessPercent =
        prefs.getUChar("bright", M33K_DEFAULT_BRIGHTNESS_PERCENT);
    if (displayBrightnessPercent < M33K_MIN_BRIGHTNESS_PERCENT ||
        displayBrightnessPercent > 100) {
        displayBrightnessPercent = M33K_DEFAULT_BRIGHTNESS_PERCENT;
    }

    savedWifiSsid = prefs.getString("ssid", "");
    savedWifiPassword = prefs.getString("pass", "");

    homeFaceIndex = prefs.getUChar("face", 0);
    if (homeFaceIndex >= HOME_FACE_COUNT) homeFaceIndex = 0;
}

void saveTimezone()
{
    if (timezoneIndex < TIMEZONE_COUNT) {
        prefs.putUChar("tz", timezoneIndex);
    } else {
        prefs.remove("tz");
    }
}

void saveTimeFormat()
{
    prefs.putBool("24h", use24Hour);
}

void saveStayConnected()
{
    prefs.putBool("wstay", wifiStayConnected);
}

void saveDisplayBrightness()
{
    prefs.putUChar("bright", displayBrightnessPercent);
}

void saveWifiCredentials(const String &ssid, const String &password)
{
    // Always keep the active credentials in RAM for the current session.
    savedWifiSsid = ssid;
    savedWifiPassword = password;

    // Tester privacy default: only persist credentials when the owner has
    // explicitly enabled Stay Connected. Otherwise remove any older saved
    // credentials so a temporary NTP/Wi-Fi session is not left in NVS.
    if (wifiStayConnected) {
        prefs.putString("ssid", savedWifiSsid);
        prefs.putString("pass", savedWifiPassword);
    } else {
        prefs.remove("ssid");
        prefs.remove("pass");
    }
}

// -----------------------------------------------------------------------------
// Activity / screen timeout
// -----------------------------------------------------------------------------

void noteActivity()
{
    lastActivityMs = millis();
}

bool clickAllowed()
{
    return !screenSleeping && millis() >= ignoreClicksUntil;
}

// Show/hide a small "LOCKED" badge on the top layer so it floats over any
// page. Must be called from the LVGL/loop context (it creates LVGL objects).
void updateLockIndicator()
{
    if (screenLocked) {
        if (!lockIndicator) {
            lockIndicator = lv_label_create(lv_layer_top());
            lv_label_set_text(lockIndicator, "LOCKED  -  HOLD TO UNLOCK");
            lv_obj_set_style_text_color(lockIndicator, lv_color_hex(0xFF4FC3), 0);
            lv_obj_set_style_bg_color(lockIndicator, lv_color_hex(0x000000), 0);
            lv_obj_set_style_bg_opa(lockIndicator, LV_OPA_60, 0);
            lv_obj_set_style_pad_all(lockIndicator, 5, 0);
            lv_obj_set_style_radius(lockIndicator, 8, 0);
            lv_obj_add_flag(lockIndicator, LV_OBJ_FLAG_FLOATING);
            lv_obj_clear_flag(lockIndicator, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_align(lockIndicator, LV_ALIGN_TOP_MID, 0, 6);
        }
        lv_obj_move_foreground(lockIndicator);
        lv_obj_clear_flag(lockIndicator, LV_OBJ_FLAG_HIDDEN);
    } else if (lockIndicator) {
        lv_obj_add_flag(lockIndicator, LV_OBJ_FLAG_HIDDEN);
    }
}

void setScreenLocked(bool locked)
{
    screenLocked = locked;
    // Swallow the release of the gesture that toggled the lock so it does not
    // also register as a tap once unlocked.
    ignoreClicksUntil = millis() + 500;
    updateLockIndicator();
    Serial.printf("[M33K] Screen %s\n", locked ? "LOCKED" : "unlocked");
}

void sleepScreen()
{
    // --------------------------------------------------------
    // Wardrive background capture mode
    //
    // A running Wardrive session must survive the display
    // timeout. Blank only the backlight and keep the MCU,
    // Wi-Fi, BLE, GPS, C5 scheduler, SD logger, and Wardrive
    // timer alive.
    // --------------------------------------------------------
    if (currentPage == Page::Wardrive &&
        wardrivePageActive &&
        wardriveRunning) {

        screenSleeping = true;

        // No visible rendering is needed while the backlight
        // is off. Other non-Wardrive home animation timers may
        // remain paused normally.
        if (matrixTimer) {
            lv_timer_pause(matrixTimer);
        }

        if (homeLiveTimer) {
            lv_timer_pause(homeLiveTimer);
        }

        // Backlight off only. Do NOT enter board/system sleep.
        instance.setBrightness(0);

        wardriveUiSyncPending = true;

        Serial.println(
            "[M33K WARD] display off; background capture remains active"
        );

        return;
    }

    if (screenSleeping) return;

    screenSleeping = true;
    if (matrixTimer) lv_timer_pause(matrixTimer);
    if (homeLiveTimer) lv_timer_pause(homeLiveTimer);

    if (currentPage == Page::Home) {
        // Keep GNSS powered continuously. Power-cycling the GNSS rail on
        // screen sleep caused fresh-acquisition delays and loss of satellite
        // lock in otherwise stable locations.
        gpsToolActive = true;

        if (!wifiStayConnected) {
            WiFi.disconnect(false, false);
            WiFi.mode(WIFI_OFF);
        }

        Serial.println("[M33K POWER] Home idle: GNSS kept on, unused Wi-Fi off");
    }

    instance.sleepDisplay();
    Serial.println("[M33K] Display timeout");
}

void wakeScreen()
{
    if (!screenSleeping) return;

    instance.wakeupDisplay();
    instance.setBrightness(displayBrightnessRaw());

    screenSleeping = false;
    lastActivityMs = millis();

    // First touch only wakes the watch instead of also opening a tool.
    ignoreClicksUntil = millis() + 350;

    // GNSS stays powered through brief screen-sleeps, so no restart is needed
    // on a normal wake. Only if deep idle had shed the GNSS rail do we restore
    // it here; a cold re-acquire after a long idle is an acceptable trade.
    if (gpsDeepIdleOff) {
        instance.powerControl(POWER_GPS, true);
        gpsToolActive = true;
        gpsDeepIdleOff = false;
        Serial.println("[M33K POWER] Wake: GNSS rail restored after deep idle");
    }

    if (matrixTimer) lv_timer_resume(matrixTimer);
    if (homeLiveTimer) {
        lv_timer_resume(homeLiveTimer);
        lv_timer_ready(homeLiveTimer);
    }

    Serial.println("[M33K] Display wake");
}

void hapticTap()
{
    instance.vibrator();
}

// -----------------------------------------------------------------------------
// RTC / battery
// -----------------------------------------------------------------------------

uint8_t buildMonthNumber(const char *monthText)
{
    static const char *months[] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };

    for (uint8_t i = 0; i < 12; ++i) {
        if (std::strncmp(monthText, months[i], 3) == 0) {
            return i + 1;
        }
    }
    return 1;
}

void seedRTCIfNeeded()
{
    RTC_DateTime now = instance.rtc.getDateTime();

    const bool valid =
        now.getYear() >= 2025 && now.getYear() <= 2099 &&
        now.getMonth() >= 1 && now.getMonth() <= 12 &&
        now.getDay() >= 1 && now.getDay() <= 31;

    if (valid) {
        return;
    }

    char monthText[4] = {};
    int day = 1;
    int year = 2026;
    int hour = 0;
    int minute = 0;
    int second = 0;

    sscanf(__DATE__, "%3s %d %d", monthText, &day, &year);
    sscanf(__TIME__, "%d:%d:%d", &hour, &minute, &second);

    const uint8_t month = buildMonthNumber(monthText);

    instance.rtc.setDateTime(
        static_cast<uint16_t>(year),
        month,
        static_cast<uint8_t>(day),
        static_cast<uint8_t>(hour),
        static_cast<uint8_t>(minute),
        static_cast<uint8_t>(second)
    );

    Serial.printf("[M33K] RTC seeded from build time: %04d-%02u-%02d %02d:%02d:%02d\n",
                  year, month, day, hour, minute, second);
}

void setGlowText(lv_obj_t **items, size_t count, const char *txt)
{
    for (size_t i = 0; i < count; ++i) {
        if (items[i]) {
            lv_label_set_text(items[i], txt);
        }
    }
}

void updatePageBatteryIndicator()
{
    if (!pageBatteryLabel) return;

    if (instance.pmu.isBatteryConnect()) {
        int pct = instance.pmu.getBatteryPercent();
        pct = constrain(pct, 0, 100);
        lv_label_set_text_fmt(
            pageBatteryLabel,
            LV_SYMBOL_BATTERY_FULL " %d%%",
            pct
        );
    } else {
        lv_label_set_text(
            pageBatteryLabel,
            LV_SYMBOL_BATTERY_FULL " --"
        );
    }
}

void createPageBatteryIndicator()
{
    if (!screen) screen = lv_screen_active();

    pageBatteryLabel = lv_label_create(screen);
    lv_label_set_text(
        pageBatteryLabel,
        LV_SYMBOL_BATTERY_FULL " --"
    );
    lv_obj_set_width(pageBatteryLabel, 82);
    lv_obj_set_pos(pageBatteryLabel, 164, 3);
    lv_obj_set_style_text_align(
        pageBatteryLabel,
        LV_TEXT_ALIGN_CENTER,
        0
    );
    lv_obj_set_style_text_color(
        pageBatteryLabel,
        lv_color_hex(0x7CFF45),
        0
    );
    lv_obj_set_style_text_font(
        pageBatteryLabel,
        &lv_font_montserrat_12,
        0
    );

    updatePageBatteryIndicator();
}

void updateHomeLiveData()
{
    if (currentPage != Page::Home) return;

    // Battery
    if (batteryLabel) {
        if (instance.pmu.isBatteryConnect()) {
            int pct = instance.pmu.getBatteryPercent();
            pct = constrain(pct, 0, 100);
            lv_label_set_text_fmt(batteryLabel, LV_SYMBOL_BATTERY_FULL " %d%%", pct);
        } else {
            lv_label_set_text(batteryLabel, LV_SYMBOL_BATTERY_FULL " --");
        }
    }

    // Home Wi-Fi state indicator:
    // green dot + bright cyan = connected
    // yellow dot = connection attempt in progress
    // dim gray-blue = disconnected
    if (homeWifiIcon && homeWifiDot) {
        if (WiFi.status() == WL_CONNECTED) {
            lv_obj_set_style_text_color(
                homeWifiIcon,
                lv_color_hex(0x3AEFFF),
                0
            );
            lv_obj_set_style_bg_color(
                homeWifiDot,
                lv_color_hex(0x7CFF45),
                0
            );
        } else if (wifiConnectPending) {
            lv_obj_set_style_text_color(
                homeWifiIcon,
                lv_color_hex(0xF5FF3B),
                0
            );
            lv_obj_set_style_bg_color(
                homeWifiDot,
                lv_color_hex(0xF5FF3B),
                0
            );
        } else {
            lv_obj_set_style_text_color(
                homeWifiIcon,
                lv_color_hex(0x526A82),
                0
            );
            lv_obj_set_style_bg_color(
                homeWifiDot,
                lv_color_hex(0x526A82),
                0
            );
        }
    }

    // Home C5 state indicator:
    // cyan + green dot = authenticated link
    // yellow = discovery/transport/authentication in progress
    // dim gray-blue = offline
    if (homeC5Label && homeC5Dot) {
        if (c5IsLinked()) {
            lv_obj_set_style_text_color(homeC5Label, lv_color_hex(0x3AEFFF), 0);
            lv_obj_set_style_bg_color(homeC5Dot, lv_color_hex(0x7CFF45), 0);
        } else if (c5DiscoveryInFlight ||
                   (c5Client && c5Client->isConnected())) {
            lv_obj_set_style_text_color(homeC5Label, lv_color_hex(0xF5FF3B), 0);
            lv_obj_set_style_bg_color(homeC5Dot, lv_color_hex(0xF5FF3B), 0);
        } else {
            lv_obj_set_style_text_color(homeC5Label, lv_color_hex(0x526A82), 0);
            lv_obj_set_style_bg_color(homeC5Dot, lv_color_hex(0x526A82), 0);
        }
    }

    // Hardware RTC
    RTC_DateTime dt = instance.rtc.getDateTime();

    uint8_t h24 = dt.getHour();
    uint8_t displayHour = h24;
    if (!use24Hour) {
        displayHour = h24 % 12;
        if (displayHour == 0) displayHour = 12;
    }

    char hourBuf[4];
    char minuteBuf[4];
    char dateTopBuf[16];

    // In 12-hour mode, do not pad single-digit hours with a leading zero.
    // The extra glyph can wrap in the fixed-width hour box (for example 08),
    // making the 8 appear below the 0. Keep zero-padding only in 24-hour mode.
    if (use24Hour) {
        snprintf(hourBuf, sizeof(hourBuf), "%02u", displayHour);
    } else {
        snprintf(hourBuf, sizeof(hourBuf), "%u", displayHour);
    }
    snprintf(minuteBuf, sizeof(minuteBuf), "%02u", dt.getMinute());

    uint8_t month = dt.getMonth();
    if (month < 1 || month > 12) month = 1;
    snprintf(dateTopBuf, sizeof(dateTopBuf), "%s %u", MONTHS[month - 1], dt.getDay());

    struct tm rtcTm = dt.toUnixTime();
    uint8_t week = static_cast<uint8_t>(rtcTm.tm_wday);
    if (week > 6) week = 0;

    if (hourLabel) lv_label_set_text(hourLabel, hourBuf);
    if (minuteLabel) lv_label_set_text(minuteLabel, minuteBuf);
    if (dateTopLabel) lv_label_set_text(dateTopLabel, dateTopBuf);
    if (dateBottomLabel) lv_label_set_text(dateBottomLabel, WEEKDAYS[week]);

    setGlowText(hourGlow, 16, hourBuf);
    setGlowText(minuteGlow, 16, minuteBuf);
    setGlowText(dateTopGlow, 4, dateTopBuf);
    setGlowText(dateBottomGlow, 4, WEEKDAYS[week]);
}

void homeLiveTimerCb(lv_timer_t *timer)
{
    LV_UNUSED(timer);
    updateHomeLiveData();
    updateFaceClock();
}

// -----------------------------------------------------------------------------
// Matrix rain
// -----------------------------------------------------------------------------

void styleMatrixGlyph(lv_obj_t *label, uint32_t colorHex)
{
    lv_obj_set_style_text_font(label, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(colorHex), 0);
    lv_obj_set_style_text_opa(label, 255, 0);
}

void buildMatrixStream(MatrixEntity &ent)
{
    lv_obj_clean(ent.obj);
    lv_obj_set_size(ent.obj, 18, ent.lines * 18);
    lv_obj_clear_flag(ent.obj, LV_OBJ_FLAG_SCROLLABLE);

    for (uint8_t i = 0; i < ent.lines; ++i) {
        const uint32_t colorHex = random(0, 2) ? matrixColA : matrixColB;
        const bool makeZero = random(0, 100) < 56;

        lv_obj_t *row = lv_obj_create(ent.obj);
        lv_obj_remove_style_all(row);
        lv_obj_set_size(row, 18, 18);
        lv_obj_set_pos(row, 0, i * 18);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        if (!makeZero) {
            lv_obj_t *glyph = lv_label_create(row);
            lv_label_set_text(glyph, "X");
            styleMatrixGlyph(glyph, colorHex);
            lv_obj_center(glyph);
            continue;
        }

        // True visual slashed-zero: the slash is a separate label placed
        // directly ON TOP of the zero, not beside it as "0/".
        lv_obj_t *zero = lv_label_create(row);
        lv_label_set_text(zero, "0");
        styleMatrixGlyph(zero, colorHex);
        lv_obj_align(zero, LV_ALIGN_CENTER, 0, 0);

        lv_obj_t *slash = lv_label_create(row);
        lv_label_set_text(slash, "/");
        styleMatrixGlyph(slash, colorHex);
        lv_obj_align(slash, LV_ALIGN_CENTER, 0, -1);
    }
}

void resetEntity(MatrixEntity &ent, bool fullyRandomY)
{
    ent.x = random(8, SCREEN_W - 30);
    ent.speed = ent.isSkull ? random(2, 4) : random(3, 7);
    ent.colorHex = random(0, 2) ? matrixColA : matrixColB;
    ent.y = fullyRandomY ? random(-360, DOCK_Y - 20) : random(-280, -40);

    if (ent.isSkull) {
        lv_obj_set_size(ent.glow, 20, 20);
        lv_obj_set_pos(ent.glow, ent.x + 2, ent.y + 4);
        lv_obj_set_style_radius(ent.glow, 10, 0);
        lv_obj_set_style_bg_color(ent.glow, lv_color_hex(ent.colorHex), 0);
        lv_obj_set_style_bg_opa(ent.glow, 0, 0);
        lv_obj_set_style_border_width(ent.glow, 0, 0);

        lv_image_set_src(ent.obj, &bunny_skull_small_img);
        lv_obj_set_pos(ent.obj, ent.x, ent.y);
        lv_obj_set_style_opa(ent.obj, 255, 0);
        if (matrixSkullRecolor != 0) {
            lv_obj_set_style_img_recolor(ent.obj, lv_color_hex(matrixSkullRecolor), 0);
            lv_obj_set_style_img_recolor_opa(ent.obj, 255, 0);
        } else {
            lv_obj_set_style_img_recolor_opa(ent.obj, 0, 0);
        }
    } else {
        ent.lines = random(10, 19);
        buildMatrixStream(ent);
        lv_obj_set_pos(ent.obj, ent.x, ent.y);

        lv_obj_set_size(ent.glow, 8, ent.lines * 18);
        lv_obj_set_pos(ent.glow, ent.x + 4, ent.y + 2);
        lv_obj_set_style_radius(ent.glow, 4, 0);
        lv_obj_set_style_bg_color(ent.glow, lv_color_hex(ent.colorHex), 0);
        lv_obj_set_style_bg_opa(ent.glow, 0, 0);
        lv_obj_set_style_border_width(ent.glow, 0, 0);
    }
}

void matrixTimerCb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    for (uint8_t i = 0; i < MATRIX_ENTITY_COUNT; ++i) {
        gMatrix[i].y += gMatrix[i].speed;

        if (gMatrix[i].y > DOCK_Y - 10 ||
            gMatrix[i].x < -30 || gMatrix[i].x > SCREEN_W + 10) {
            resetEntity(gMatrix[i], false);
        } else {
            if (matrixDriftX != 0) {
                gMatrix[i].x += matrixDriftX;
                lv_obj_set_x(gMatrix[i].obj, gMatrix[i].x);
                lv_obj_set_x(gMatrix[i].glow,
                             gMatrix[i].x + (gMatrix[i].isSkull ? 2 : 4));
            }
            lv_obj_set_y(gMatrix[i].obj, gMatrix[i].y);
            lv_obj_set_y(gMatrix[i].glow,
                         gMatrix[i].isSkull ? gMatrix[i].y + 4 : gMatrix[i].y + 2);
        }
    }
}

void createMatrixBackground()
{
    for (uint8_t i = 0; i < MATRIX_ENTITY_COUNT; ++i) {
        gMatrix[i].isSkull = (i % 5 == 0);

        gMatrix[i].glow = lv_obj_create(screen);
        lv_obj_remove_style_all(gMatrix[i].glow);

        if (gMatrix[i].isSkull) {
            gMatrix[i].obj = lv_image_create(screen);
        } else {
            gMatrix[i].obj = lv_obj_create(screen);
            lv_obj_remove_style_all(gMatrix[i].obj);
            lv_obj_clear_flag(gMatrix[i].obj, LV_OBJ_FLAG_SCROLLABLE);
        }

        resetEntity(gMatrix[i], true);
    }

    matrixTimer = lv_timer_create(matrixTimerCb, 90, nullptr);
}

// -----------------------------------------------------------------------------
// BLE scanner core
// -----------------------------------------------------------------------------

void initBleScanner()
{
    if (bleScannerInitialized) return;

    if (!NimBLEDevice::isInitialized()) {
        NimBLEDevice::init("M33K X");
    }

    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(&bleScanCallbacks, false);
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(80);
    scan->setMaxResults(MAX_BLE_DETAILS);

    bleScannerInitialized = true;

    // GPS-anchor coexistence guard: v0.6.0b3 is the last hardware-confirmed
    // GPS-working startup. Keep that boot sequence untouched, and only after
    // the BLE scanner is configured reassert the Ultra GNSS rail/UART.
    // LilyGoLib powerControl(POWER_GPS, true) reopens Serial1 at 38400 on the
    // Ultra GPS pins without power-cycling the GNSS module.
    instance.powerControl(POWER_GPS, true);
    gpsToolActive = true;

    Serial.println("[M33K] NimBLE scanner initialized; GNSS UART reasserted");
}

void stopBleTool()
{
    if (!bleScannerInitialized) {
        bleScanFinished = false;
        return;
    }

    NimBLEScan *scan = NimBLEDevice::getScan();

    if (scan->isScanning()) {
        scan->stop();
    }

    scan->clearResults();
    bleScanFinished = false;
}


// -----------------------------------------------------------------------------
// Phone/web Companion intentionally omitted from this development build.
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// M33K X C5 / XIAO ESP32-C5 BLE client
// -----------------------------------------------------------------------------

wifi_auth_mode_t c5AuthFromText(const String &sec)
{
    if (sec == "OPEN") return WIFI_AUTH_OPEN;
    if (sec == "WEP") return WIFI_AUTH_WEP;
    if (sec == "WPA") return WIFI_AUTH_WPA_PSK;
    if (sec == "WPA2") return WIFI_AUTH_WPA2_PSK;
    if (sec == "WPA/WPA2") return WIFI_AUTH_WPA_WPA2_PSK;
    if (sec == "WPA2-E") return WIFI_AUTH_WPA2_ENTERPRISE;
    if (sec == "WPA3") return WIFI_AUTH_WPA3_PSK;
    if (sec == "WPA2/WPA3") return WIFI_AUTH_WPA2_WPA3_PSK;
    if (sec == "WAPI") return WIFI_AUTH_WAPI_PSK;
    return WIFI_AUTH_OPEN;
}

String c5FieldValue(const String &line, const char *field)
{
    const String needle = String("|") + field + "=";
    int start = line.indexOf(needle);
    if (start < 0) return "";
    start += needle.length();
    int end = line.indexOf('|', start);
    if (end < 0) end = line.length();
    return line.substring(start, end);
}

int c5HexNibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}

bool c5HexToBytes(const String &hex, uint8_t *out, size_t outLen)
{
    if (hex.length() != outLen * 2) return false;
    for (size_t i = 0; i < outLen; ++i) {
        const int hi = c5HexNibble(hex[i * 2]);
        const int lo = c5HexNibble(hex[i * 2 + 1]);
        if (hi < 0 || lo < 0) return false;
        out[i] = static_cast<uint8_t>((hi << 4) | lo);
    }
    return true;
}

String c5BytesToHex(const uint8_t *data, size_t len)
{
    static constexpr char HEX_CHARS[] = "0123456789abcdef";
    String out;
    out.reserve(len * 2);
    for (size_t i = 0; i < len; ++i) {
        out += HEX_CHARS[(data[i] >> 4) & 0x0F];
        out += HEX_CHARS[data[i] & 0x0F];
    }
    return out;
}

bool c5LoadLinkKey()
{
    if (prefs.getBytesLength("c5key") != sizeof(c5LinkKey)) {
        memset(c5LinkKey, 0, sizeof(c5LinkKey));
        c5LinkKeyLoaded = false;
        return false;
    }

    const size_t read = prefs.getBytes("c5key", c5LinkKey, sizeof(c5LinkKey));
    c5LinkKeyLoaded = read == sizeof(c5LinkKey);
    return c5LinkKeyLoaded;
}

bool c5CreateAndStoreLinkKey()
{
    esp_fill_random(c5LinkKey, sizeof(c5LinkKey));
    if (prefs.putBytes("c5key", c5LinkKey, sizeof(c5LinkKey)) != sizeof(c5LinkKey)) {
        memset(c5LinkKey, 0, sizeof(c5LinkKey));
        c5LinkKeyLoaded = false;
        return false;
    }

    c5LinkKeyLoaded = true;
    return true;
}

bool c5ComputeAuthResponse(const String &challengeHex, String &responseHex)
{
    if (!c5LinkKeyLoaded) return false;

    uint8_t challenge[16] = {};
    uint8_t digest[32] = {};
    if (!c5HexToBytes(challengeHex, challenge, sizeof(challenge))) return false;

    const mbedtls_md_info_t *info =
        mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (!info) return false;

    if (mbedtls_md_hmac(
            info,
            c5LinkKey,
            sizeof(c5LinkKey),
            challenge,
            sizeof(challenge),
            digest
        ) != 0) {
        return false;
    }

    responseHex = c5BytesToHex(digest, sizeof(digest));
    return true;
}

void c5HandleDataLine(String line)
{
    line.trim();
    if (line.length() == 0) return;

    if (line.startsWith("ENROLL_REQUIRED|")) {
        if (!c5LinkKeyLoaded && !c5CreateAndStoreLinkKey()) {
            c5StatusText = "C5 ENROLL KEY ERROR";
            if (c5Client) c5Client->disconnect();
            return;
        }

        if (!c5CommandChar) return;
        const String enrollCommand = "ENROLL|" + c5BytesToHex(c5LinkKey, sizeof(c5LinkKey));
        c5StatusText = "PAIRING C5";
        c5AuthStartedMs = millis();

        // Never print the enrollment key. The characteristic requires an
        // encrypted BLE link before this write is accepted.
        if (!c5CommandChar->writeValue(enrollCommand.c_str())) {
            c5StatusText = "C5 ENROLL WRITE FAIL";
            if (c5Client) c5Client->disconnect();
        }
        return;
    }

    if (line.startsWith("ENROLL_OK|")) {
        c5StatusText = "AUTHENTICATING";
        c5AuthStartedMs = millis();
        if (!c5CommandChar || !c5CommandChar->writeValue("CHALLENGE")) {
            c5StatusText = "C5 AUTH START FAIL";
            if (c5Client) c5Client->disconnect();
        }
        return;
    }

    if (line.startsWith("CHALLENGE|")) {
        c5AuthChallengeHex = line.substring(10);
        c5AuthResponseSent = false;
        c5StatusText = "AUTHENTICATING";
        return;
    }

    if (line.startsWith("AUTH_OK|")) {
        c5Authenticated = true;
        c5AuthChallengeHex = "";
        c5AuthResponseSent = false;
        c5StatusText = "READY";
        if (c5CommandChar) c5CommandChar->writeValue("PING");
        Serial.println("[M33K C5] authenticated");
        return;
    }

    if (line.startsWith("ERROR|auth_")) {
        c5Authenticated = false;
        c5ScanInFlight = false;
        c5StatusText = "C5 AUTH FAILED";
        return;
    }

    if (line.startsWith("ERROR|enroll_") ||
        line.startsWith("ERROR|already_enrolled") ||
        line.startsWith("ERROR|bad_enroll_key")) {
        c5Authenticated = false;
        c5ScanInFlight = false;
        c5StatusText = "C5 PAIRING FAILED";
        return;
    }

    if (line.startsWith("SCAN_BEGIN|")) {
        // Fill a staging buffer while the new C5 sweep runs.  The published
        // results stay untouched so the 5 GHz graph never disappears between
        // scans.
        c5WifiPendingDetailCount = 0;
        c5ScanComplete = false;
        c5ScanInFlight = true;
        c5StatusText = "SCANNING 5G";
        return;
    }

    if (line.startsWith("AP|")) {
        const uint8_t index = c5WifiPendingDetailCount;
        if (index >= MAX_C5_WIFI_DETAILS) return;

        WifiResultDetail &detail = c5WifiPendingDetails[index];
        detail.ssid = c5FieldValue(line, "ssid");
        if (detail.ssid.length() == 0) detail.ssid = "<hidden>";
        detail.bssid = c5FieldValue(line, "bssid");
        detail.channel = c5FieldValue(line, "ch").toInt();
        detail.rssi = c5FieldValue(line, "rssi").toInt();
        detail.auth = c5AuthFromText(c5FieldValue(line, "sec"));
        detail.fromC5 = true;
        c5WifiPendingDetailCount = index + 1;
        return;
    }

    if (line.startsWith("SCAN_END|")) {
        // Publish the completed sweep atomically from the UI's point of view.
        // Until this point, the previous complete 5 GHz result set remains on
        // screen as the faint blue occupancy layer.
        const uint8_t completedCount = c5WifiPendingDetailCount;
        for (uint8_t i = 0; i < completedCount; ++i) {
            c5WifiDetails[i] = c5WifiPendingDetails[i];
        }
        c5WifiDetailCount = completedCount;
        c5ScanInFlight = false;
        c5ScanComplete = true;
        ++c5ScanGeneration;
        c5StatusText = "READY";
        return;
    }

    if (line.startsWith("ERROR|")) {
        c5ScanInFlight = false;
        c5StatusText = "SCAN ERROR";
        return;
    }

    if (line.startsWith("PONG|")) {
        c5StatusText = "READY";
    }
}

bool queueC5Event(C5EventType type, const uint8_t *data = nullptr,
                  size_t length = 0, int reason = 0)
{
    if (!c5EventQueue) return false;

    C5Event event;
    event.type = type;
    event.reason = reason;

    if (data && length > 0) {
        const size_t copyLen = min(length, sizeof(event.text) - 1);
        memcpy(event.text, data, copyLen);
        event.text[copyLen] = '\0';
    }

    return xQueueSend(c5EventQueue, &event, 0) == pdTRUE;
}

void c5DataNotifyCB(
    NimBLERemoteCharacteristic *characteristic,
    uint8_t *data,
    size_t length,
    bool isNotify)
{
    LV_UNUSED(characteristic);
    LV_UNUSED(isNotify);
    queueC5Event(C5EventType::Data, data, length);
}

void c5StatusNotifyCB(
    NimBLERemoteCharacteristic *characteristic,
    uint8_t *data,
    size_t length,
    bool isNotify)
{
    LV_UNUSED(characteristic);
    LV_UNUSED(isNotify);
    queueC5Event(C5EventType::Status, data, length);
}

class M33KC5ClientCallbacks : public NimBLEClientCallbacks {
public:
    void onConnect(NimBLEClient *client) override
    {
        LV_UNUSED(client);
        queueC5Event(C5EventType::Connect);
    }

    void onDisconnect(NimBLEClient *client, int reason) override
    {
        LV_UNUSED(client);
        queueC5Event(C5EventType::Disconnect, nullptr, 0, reason);
    }
};

M33KC5ClientCallbacks c5ClientCallbacks;

void serviceC5Events()
{
    if (!c5EventQueue) return;

    C5Event event;
    while (xQueueReceive(c5EventQueue, &event, 0) == pdTRUE) {
        switch (event.type) {
            case C5EventType::Connect:
                c5StatusText = "LINKED";
                Serial.println("[M33K C5] M33K X C5 connected");
                break;

            case C5EventType::Disconnect: {
                Serial.printf("[M33K C5] disconnected reason=%d\n", event.reason);
                c5CommandChar = nullptr;
                c5DataChar = nullptr;
                c5StatusChar = nullptr;
                c5Authenticated = false;
                c5AuthChallengeHex = "";
                c5AuthResponseSent = false;
                c5AuthStartedMs = 0;
                c5ScanInFlight = false;
                c5ScanComplete = false;
                c5DiscoveryInFlight = false;

                // If Wardrive froze visual updates during this C5
                // transaction, redraw once the BLE transport is gone.
                if (currentPage == Page::Wardrive) {
                    wardriveUiSyncPending = true;
                }

                const bool hadRemoteData =
                    c5WifiDetailCount > 0 || c5WifiPendingDetailCount > 0;
                c5WifiDetailCount = 0;
                c5WifiPendingDetailCount = 0;
                if (hadRemoteData) ++c5ScanGeneration;

                // Fully release the disconnected C5 GATT client.
                if (c5Client && !c5Client->isConnected()) {
                    NimBLEDevice::deleteClient(c5Client);
                    c5Client = nullptr;

                    Serial.println(
                        "[M33K C5] hard client cleanup complete"
                    );
                }

                // Let BLE teardown settle before Wardrive redraws.
                if (currentPage == Page::Wardrive) {
                    wardriveDisplayResumeMs =
                        millis() + 1500UL;
                    wardriveUiSyncPending = true;
                }


                c5StatusText = "OFFLINE";
                c5NextConnectAttemptMs = millis() + 2500;
                break;
            }

            case C5EventType::Data:
                c5HandleDataLine(String(event.text));
                break;

            case C5EventType::Status: {
                String text(event.text);
                text.trim();
                if (text.length() > 0) c5StatusText = text;
                break;
            }
        }
    }
}

bool c5TransportConnected()
{
    return c5Client &&
           c5Client->isConnected() &&
           c5CommandChar &&
           c5DataChar;
}

bool c5IsLinked()
{
    return c5TransportConnected() && c5Authenticated;
}

bool startC5DiscoveryAsync()
{
    if (c5IsLinked() || c5DiscoveryInFlight) return c5IsLinked();
    if (c5TransportConnected()) return false; // authentication is still running

    const uint32_t now = millis();
    if (now < c5NextConnectAttemptMs) return false;

    initBleScanner();
    NimBLEScan *scan = NimBLEDevice::getScan();

    // Never steal the scanner from another BLE tool/recon phase.
    if (scan->isScanning()) return false;

    scan->clearResults();
    scan->setActiveScan(true);
    // Keep discovery bounded.  A large BLE result cache increases heap use
    // and stack pressure when finishC5Discovery() walks the results.
    scan->setMaxResults(24);
    bleScanFinished = false;

    c5StatusText = "FINDING M33K X C5";
    c5DiscoveryInFlight = true;
    c5DiscoveryStartedMs = now;
    c5NextConnectAttemptMs = now + C5_DISCOVERY_RETRY_MS;

    Serial.println("[M33K C5] async search for C5 service...");

    if (!scan->start(C5_DISCOVERY_DURATION_MS, false, true)) {
        c5DiscoveryInFlight = false;
        c5StatusText = "C5 SEARCH BUSY";
        scan->setMaxResults(MAX_BLE_DETAILS);
        return false;
    }

    return true;
}

bool finishC5Discovery()
{
    if (!c5DiscoveryInFlight) return false;
    c5DiscoveryInFlight = false;

    NimBLEScan *scan = NimBLEDevice::getScan();
    NimBLEScanResults results = scan->getResults();

    const NimBLEUUID targetService(C5_SERVICE_UUID);
    const NimBLEAdvertisedDevice *target = nullptr;

    for (int i = 0; i < results.getCount(); ++i) {
        const NimBLEAdvertisedDevice *dev = results.getDevice(i);
        if (!dev) continue;

        if (dev->haveServiceUUID() &&
            dev->isAdvertisingService(targetService)) {
            target = dev;
            break;
        }

        const std::string rawName = dev->getName();
        if (!rawName.empty() && rawName == "M33K X C5") {
            target = dev;
            break;
        }
    }

    if (!target) {
        scan->clearResults();
        scan->setMaxResults(MAX_BLE_DETAILS);
        c5StatusText = "C5 NOT FOUND";

        // C5 is optional in Wardrive.
        //
        // A failed C5 discovery must not immediately monopolize NimBLE
        // again. Give the watch-side BLE scanner the next quiet radio
        // window, then retry the optional C5 later.
        c5NextConnectAttemptMs =
            millis() + 15000UL;

        if (currentPage == Page::Wardrive &&
            wardriveRunning) {

            wardriveNextC5CycleMs =
                millis() + 15000UL;

            wardriveNextBleSampleMs =
                millis() + 300UL;
        }

        Serial.println(
            "[M33K C5] M33K X C5 not found (async)"
        );

        if (currentPage == Page::Wardrive &&
            wardriveRunning) {
            Serial.println(
                "[M33K WARD] C5 offline; BLE gets next radio window"
            );
        }

        return false;
    }

    if (c5Client) {
        if (c5Client->isConnected()) c5Client->disconnect();
        NimBLEDevice::deleteClient(c5Client);
        c5Client = nullptr;
    }

    c5Client = NimBLEDevice::createClient();
    if (!c5Client) {
        scan->clearResults();
        scan->setMaxResults(MAX_BLE_DETAILS);
        c5StatusText = "CLIENT ERROR";
        return false;
    }

    c5Client->setClientCallbacks(&c5ClientCallbacks, false);
    c5Client->setConnectionParams(12, 24, 0, 180);
    // The companion is physically nearby; do not freeze the watch for 4 s.
    c5Client->setConnectTimeout(1800);

    if (!c5Client->connect(target)) {
        Serial.println("[M33K C5] async connection failed");
        NimBLEDevice::deleteClient(c5Client);
        c5Client = nullptr;
        scan->clearResults();
        scan->setMaxResults(MAX_BLE_DETAILS);
        c5StatusText = "LINK FAILED";
        return false;
    }

    // Re-establish BLE link encryption explicitly on every connection.
    // The C5 command/data characteristics require encrypted access. Relying
    // only on an encrypted characteristic write can work on first enrollment
    // but is not reliable enough for reconnects after either device reboots.
    c5StatusText = "SECURING C5 LINK";
    if (!c5Client->secureConnection()) {
        Serial.println("[M33K C5] BLE security failed");
        c5StatusText = "SECURITY FAILED";
        c5Client->disconnect();
        scan->clearResults();
        scan->setMaxResults(MAX_BLE_DETAILS);
        return false;
    }
    Serial.println("[M33K C5] encrypted BLE link ready");

    NimBLERemoteService *service =
        c5Client->getService(C5_SERVICE_UUID);

    if (!service) {
        Serial.println("[M33K C5] service not found after connect");
        c5Client->disconnect();
        scan->clearResults();
        scan->setMaxResults(MAX_BLE_DETAILS);
        c5StatusText = "SERVICE ERROR";
        return false;
    }

    c5CommandChar = service->getCharacteristic(C5_COMMAND_UUID);
    c5DataChar = service->getCharacteristic(C5_DATA_UUID);
    c5StatusChar = service->getCharacteristic(C5_STATUS_UUID);

    if (!c5CommandChar || !c5DataChar) {
        Serial.println("[M33K C5] required characteristics missing");
        c5Client->disconnect();
        scan->clearResults();
        scan->setMaxResults(MAX_BLE_DETAILS);
        c5StatusText = "CHAR ERROR";
        return false;
    }

    if (c5DataChar->canNotify()) {
        if (!c5DataChar->subscribe(true, c5DataNotifyCB)) {
            Serial.println("[M33K C5] data subscribe failed");
            c5Client->disconnect();
            scan->clearResults();
            scan->setMaxResults(MAX_BLE_DETAILS);
            c5StatusText = "SUBSCRIBE ERROR";
            return false;
        }
    }

    if (c5StatusChar && c5StatusChar->canNotify()) {
        c5StatusChar->subscribe(true, c5StatusNotifyCB);
    }

    scan->clearResults();
    scan->setMaxResults(MAX_BLE_DETAILS);
    c5Authenticated = false;
    c5AuthChallengeHex = "";
    c5AuthResponseSent = false;
    c5AuthStartedMs = millis();
    c5StatusText = "AUTHENTICATING";
    c5NextConnectAttemptMs = 0;

    if (!c5CommandChar->writeValue("CHALLENGE")) {
        c5StatusText = "AUTH START FAILED";
        c5Client->disconnect();
        return false;
    }

    Serial.println("[M33K C5] transport linked; authenticating...");
    return true;
}

bool startToolC5Discovery()
{
    if (c5IsLinked() || c5TransportConnected() || c5DiscoveryInFlight) {
        return c5DiscoveryInFlight;
    }

    // A tool page is an explicit request to use dual-band features, so do not
    // inherit a previous "not found" retry delay from another page.
    c5NextConnectAttemptMs = 0;
    return startC5DiscoveryAsync();
}

bool requestC5Scan()
{
    if (c5ScanInFlight) return true;

    // Recon/Pulse/Radar timers must never start C5 BLE discovery themselves.
    // Discovery allocates/iterates a comparatively large NimBLE result set on
    // loopTask and can exhaust the Arduino loop stack while the recon tools
    // are also processing Wi-Fi/LVGL work.  If the companion is not already
    // authenticated, continue safely in local 2.4 GHz-only mode.  C5
    // discovery remains available from the Wi-Fi page, where it is started
    // outside the recon scan/timer path.
    if (!c5IsLinked()) {
        return false;
    }

    c5WifiPendingDetailCount = 0;
    c5ScanComplete = false;
    c5ScanInFlight = true;
    c5ScanRequestedMs = millis();
    c5StatusText = "SCAN QUEUED";

    if (!c5CommandChar->writeValue("SCAN5")) {
        c5ScanInFlight = false;
        c5StatusText = "COMMAND FAILED";
        return false;
    }

    return true;
}

void serviceC5Link()
{
    if (c5TransportConnected() && !c5Authenticated) {
        if (!c5AuthResponseSent && c5AuthChallengeHex.length() > 0) {
            String responseHex;
            if (!c5ComputeAuthResponse(c5AuthChallengeHex, responseHex)) {
                c5StatusText = "C5 AUTH ERROR";
                c5Client->disconnect();
                return;
            }

            const String authCommand = "AUTH|" + responseHex;
            c5AuthResponseSent = true;
            c5AuthStartedMs = millis();
            // Never print the response digest; it is proof material.
            if (!c5CommandChar->writeValue(authCommand.c_str())) {
                c5StatusText = "C5 AUTH WRITE FAIL";
                c5Client->disconnect();
                return;
            }
        }

        if (c5AuthStartedMs != 0 &&
            millis() - c5AuthStartedMs > C5_AUTH_TIMEOUT_MS) {
            c5StatusText = "C5 AUTH TIMEOUT";
            c5Client->disconnect();
            return;
        }
    }

    if (c5ScanInFlight &&
        millis() - c5ScanRequestedMs > 12000) {
        c5ScanInFlight = false;
        c5StatusText = "SCAN TIMEOUT";
    }

    // Safety net: if the BLE disconnect callback was missed for any reason,
    // never merge stale C5 results while the companion is offline.
    if (!c5TransportConnected() && !c5DiscoveryInFlight && c5WifiDetailCount > 0) {
        c5WifiDetailCount = 0;
        c5WifiPendingDetailCount = 0;
        ++c5ScanGeneration;
    }

    if (c5DiscoveryInFlight &&
        millis() - c5DiscoveryStartedMs > C5_DISCOVERY_DURATION_MS + 2500) {
        c5DiscoveryInFlight = false;
        c5StatusText = "C5 SEARCH TIMEOUT";
    }
}

// -----------------------------------------------------------------------------
// Recon dashboard core
// -----------------------------------------------------------------------------

void updateReconGpsLabels()
{
    if (currentPage != Page::Recon) return;

    const bool haveFix =
        gpsParser.location.isValid() &&
        gpsParser.location.age() < 5000;

    if (reconGpsLabel) {
        const unsigned long sats = gpsParser.satellites.isValid()
            ? (unsigned long)gpsParser.satellites.value()
            : 0UL;

        if (haveFix) {
            lv_label_set_text_fmt(
                reconGpsLabel,
                "GPS FIX\nSATS %lu",
                sats
            );
            lv_obj_set_style_text_color(
                reconGpsLabel,
                lv_color_hex(0x7CFF45),
                0
            );
        } else {
            lv_label_set_text_fmt(
                reconGpsLabel,
                "GPS SEARCH\nSATS %lu",
                sats
            );
            lv_obj_set_style_text_color(
                reconGpsLabel,
                lv_color_hex(0xF5FF3B),
                0
            );
        }
    }
}

void updateReconFromMergedWifiCache()
{
    reconWifiCount = wifiDetailCount;
    reconWifi24Count = 0;
    reconWifi5Count = 0;
    reconStrongestWifi = -127;
    reconBusiestChannel = 0;
    reconBusiestChannelCount = 0;

    uint8_t channelCounts[178] = {};

    for (uint8_t i = 0; i < wifiDetailCount; ++i) {
        const WifiResultDetail &detail = wifiDetails[i];
        const bool band5 = detail.fromC5 || detail.channel > 14;

        if (band5) ++reconWifi5Count;
        else ++reconWifi24Count;

        if (detail.rssi > reconStrongestWifi) {
            reconStrongestWifi = detail.rssi;
        }

        if (detail.channel >= 1 && detail.channel <= 177) {
            ++channelCounts[detail.channel];
        }
    }

    for (int channel = 1; channel <= 177; ++channel) {
        if (channelCounts[channel] > reconBusiestChannelCount) {
            reconBusiestChannelCount = channelCounts[channel];
            reconBusiestChannel = channel;
        }
    }
}

void updateReconSummaryLabels()
{
    if (reconWifiCountLabel) {
        if (reconWifiCount > 0) {
            lv_label_set_text_fmt(
                reconWifiCountLabel,
                "WI-FI\n%d APs\n2.4:%d  5G:%d",
                reconWifiCount,
                reconWifi24Count,
                reconWifi5Count
            );
        } else {
            lv_label_set_text(
                reconWifiCountLabel,
                "WI-FI\n0 APs\nBEST --"
            );
        }
    }

    if (reconChannelLabel) {
        if (reconBusiestChannel > 0) {
            lv_label_set_text_fmt(
                reconChannelLabel,
                "BUSIEST\nCH %d\n%d APs",
                reconBusiestChannel,
                reconBusiestChannelCount
            );
        } else {
            lv_label_set_text(
                reconChannelLabel,
                "BUSIEST\nCH --"
            );
        }
    }

    if (reconBleCountLabel) {
        if (reconBleCount > 0) {
            lv_label_set_text_fmt(
                reconBleCountLabel,
                "BLE\n%d DEV\nBEST %d",
                reconBleCount,
                reconStrongestBle
            );
        } else {
            lv_label_set_text(
                reconBleCountLabel,
                "BLE\n0 DEV\nBEST --"
            );
        }
    }

    updateReconGpsLabels();
}

bool startReconBleScan()
{
    initBleScanner();

    NimBLEScan *scan = NimBLEDevice::getScan();

    if (scan->isScanning()) {
        scan->stop();
    }

    scan->clearResults();
    bleScanFinished = false;
    bleScanEndReason = 0;

    reconPhase = ReconPhase::BleScanning;

    if (reconStatusLabel) {
        lv_label_set_text(
            reconStatusLabel,
            "DUAL WI-FI DONE  |  SCANNING BLE..."
        );
    }

    return scan->start(3500, false, true);
}

void finishReconBleScan()
{
    if (!bleScannerInitialized) {
        reconPhase = ReconPhase::Failed;
        if (reconStatusLabel) {
            lv_label_set_text(
                reconStatusLabel,
                "BLE SCAN FAILED"
            );
        }
        return;
    }

    NimBLEScan *scan = NimBLEDevice::getScan();
    NimBLEScanResults results = scan->getResults();

    reconBleCount = results.getCount();
    reconStrongestBle = -127;

    for (int i = 0; i < reconBleCount; ++i) {
        const NimBLEAdvertisedDevice *dev =
            results.getDevice(i);

        if (!dev) continue;

        const int rssi = dev->getRSSI();

        if (rssi > reconStrongestBle) {
            reconStrongestBle = rssi;
        }
    }

    scan->clearResults();

    serviceC5Link();
    mergeC5WifiResults();
    updateReconFromMergedWifiCache();

    reconPhase = ReconPhase::Done;

    if (reconStatusLabel) {
        lv_label_set_text(
            reconStatusLabel,
            "RECON COMPLETE  |  TAP REFRESH"
        );
    }

    updateReconSummaryLabels();
}

void beginReconScan()
{
    if (currentPage != Page::Recon) return;

    noteActivity();

    reconWifiCount = 0;
    reconWifi24Count = 0;
    reconWifi5Count = 0;
    reconStrongestWifi = -127;
    reconBusiestChannel = 0;
    reconBusiestChannelCount = 0;
    reconBleCount = 0;
    reconStrongestBle = -127;

    updateReconSummaryLabels();

    if (reconStatusLabel) {
        lv_label_set_text(
            reconStatusLabel,
            "SCANNING 2.4 + 5 GHz WI-FI..."
        );
    }

    requestC5Scan();

    // Preserve the working v0.3.23 Wi-Fi recovery pattern.
    esp_wifi_scan_stop();
    WiFi.scanDelete();

    const bool alreadyConnected =
        WiFi.status() == WL_CONNECTED;

    if (!alreadyConnected) {
        WiFi.disconnect(false, false);
        delay(120);
    }

    WiFi.mode(WIFI_STA);
    delay(220);

    esp_wifi_scan_stop();
    WiFi.scanDelete();
    delay(50);

    int16_t result =
        WiFi.scanNetworks(
            true,
            true,
            false,
            300
        );

    if (result == WIFI_SCAN_FAILED &&
        !alreadyConnected) {

        if (reconStatusLabel) {
            lv_label_set_text(
                reconStatusLabel,
                "RESETTING WI-FI RADIO..."
            );
        }

        WiFi.disconnect(true, false);
        delay(120);

        WiFi.mode(WIFI_STA);
        delay(350);

        WiFi.disconnect(false, false);
        delay(100);

        result =
            WiFi.scanNetworks(true, true, false, 350);
    }

    if (result == WIFI_SCAN_FAILED) {
        reconPhase = ReconPhase::Failed;

        if (reconStatusLabel) {
            lv_label_set_text(
                reconStatusLabel,
                "WI-FI SCAN FAILED"
            );
        }

        return;
    }

    reconPhase = ReconPhase::WifiScanning;

    if (result >= 0) {
        // Synchronous/immediate completion is uncommon, but the timer
        // will process this same completed result on its next tick.
    }
}

void reconRefreshEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    hapticTap();
    beginReconScan();
}

void reconTimerCb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    if (currentPage != Page::Recon) return;

    serviceC5Link();

    updateReconGpsLabels();

    if (reconPhase != ReconPhase::WifiScanning) {
        return;
    }

    const int16_t result =
        WiFi.scanComplete();

    if (result == WIFI_SCAN_RUNNING) {
        return;
    }

    if (result == WIFI_SCAN_FAILED) {
        reconPhase = ReconPhase::Failed;

        if (reconStatusLabel) {
            lv_label_set_text(
                reconStatusLabel,
                "WI-FI SCAN FAILED"
            );
        }

        WiFi.scanDelete();
        return;
    }

    captureLocalWifiResults(result);
    mergeC5WifiResults();
    updateReconFromMergedWifiCache();

    WiFi.scanDelete();
    updateReconSummaryLabels();

    const bool bleStarted =
        startReconBleScan();

    if (!bleStarted) {
        reconPhase = ReconPhase::Failed;

        if (reconStatusLabel) {
            lv_label_set_text(
                reconStatusLabel,
                "BLE SCAN COULD NOT START"
            );
        }
    }
}

uint64_t wardriveTotalWifiHash(const String &bssid)
{
    uint64_t hash = 1469598103934665603ULL;

    for (size_t i = 0; i < bssid.length(); ++i) {
        char c = bssid[i];

        if (c >= 'a' && c <= 'f') {
            c = static_cast<char>(c - ('a' - 'A'));
        }

        hash ^= static_cast<uint8_t>(c);
        hash *= 1099511628211ULL;
    }

    return hash;
}

void wardriveTrackWifiTotal(const String &bssid)
{
    const uint64_t hash = wardriveTotalWifiHash(bssid);
    constexpr uint16_t mask = WARD_TOTAL_WIFI_BLOOM_BITS - 1;

    const uint16_t bit1 = static_cast<uint16_t>(hash) & mask;
    const uint16_t bit2 = static_cast<uint16_t>((hash >> 21) ^ hash) & mask;
    const uint16_t bit3 = static_cast<uint16_t>((hash >> 42) ^ (hash >> 7)) & mask;

    auto bitIsSet = [](uint16_t bit) {
        return (wardriveTotalWifiBloom[bit >> 3] & (1U << (bit & 7))) != 0;
    };
    auto setBit = [](uint16_t bit) {
        wardriveTotalWifiBloom[bit >> 3] |= static_cast<uint8_t>(1U << (bit & 7));
    };

    const bool probablySeen =
        bitIsSet(bit1) && bitIsSet(bit2) && bitIsSet(bit3);

    setBit(bit1);
    setBit(bit2);
    setBit(bit3);

    if (!probablySeen && wardriveTotalWifiCount < UINT32_MAX) {
        ++wardriveTotalWifiCount;
    }
}


void wardriveTrackBleTotal(const String &addr)
{
    if (addr.length() == 0) return;

    const uint64_t hash =
        wardriveTotalWifiHash(addr);

    constexpr uint16_t mask =
        WARD_TOTAL_BLE_BLOOM_BITS - 1;

    const uint16_t bit1 =
        static_cast<uint16_t>(hash) & mask;

    const uint16_t bit2 =
        static_cast<uint16_t>(
            (hash >> 21) ^ hash
        ) & mask;

    const uint16_t bit3 =
        static_cast<uint16_t>(
            (hash >> 42) ^ (hash >> 7)
        ) & mask;

    auto bitIsSet = [](uint16_t bit) {
        return (
            wardriveTotalBleBloom[bit >> 3] &
            (1U << (bit & 7))
        ) != 0;
    };

    auto setBit = [](uint16_t bit) {
        wardriveTotalBleBloom[bit >> 3] |=
            static_cast<uint8_t>(
                1U << (bit & 7)
            );
    };

    const bool probablySeen =
        bitIsSet(bit1) &&
        bitIsSet(bit2) &&
        bitIsSet(bit3);

    setBit(bit1);
    setBit(bit2);
    setBit(bit3);

    if (!probablySeen &&
        wardriveTotalBleCount < UINT32_MAX) {

        ++wardriveTotalBleCount;
    }
}

bool wardriveHasWifi(const String &key)
{
    for (uint8_t i = 0; i < wardriveWifiCount; ++i) {
        if (wardriveWifiKeys[i] == key) return true;
    }
    return false;
}

bool wardriveHasBle(const String &addr)
{
    for (uint8_t i = 0; i < wardriveBleCount; ++i) {
        if (wardriveBleAddrs[i] == addr) return true;
    }
    return false;
}

WardriveRecord *findWardriveRecord(bool wifi, const String &id)
{
    for (uint16_t i = 0; i < wardriveRecordCount; ++i) {
        WardriveRecord &rec = wardriveRecords[i];
        if (rec.active && rec.wifi == wifi && rec.id == id) return &rec;
    }
    return nullptr;
}

void snapshotWardriveLocation(WardriveRecord &rec)
{
    const bool fix = gpsParser.location.isValid() && gpsParser.location.age() < 5000;
    rec.haveLocation = fix;
    if (fix) {
        rec.lat = gpsParser.location.lat();
        rec.lon = gpsParser.location.lng();
        rec.sats = gpsParser.satellites.isValid() ? (uint8_t)gpsParser.satellites.value() : 0;
    }
}

void updateWardriveRecordLabel(WardriveRecord &rec)
{
    if (!wardrivePageActive || currentPage != Page::Wardrive) return;
    if (wardriveFeedScrolling || wardriveDetailOpen) {
        wardriveUiSyncPending = true;
        wardriveUiSyncCursor = 0;
        return;
    }
    if (!rec.label) return;

    if (rec.row) {
        lv_obj_set_style_bg_color(
            rec.row,
            lv_color_hex(rec.wifi ? (rec.fromC5 ? 0x08112A : 0x071C29) : 0x1A0B1B),
            0
        );
        lv_obj_set_style_border_color(
            rec.row,
            lv_color_hex(rec.wifi ? (rec.fromC5 ? 0x2F6BFF : 0x55EEFF) : 0xFF52C8),
            0
        );
    }

    String displayName = rec.name.length() ? rec.name : rec.id;
    if (displayName.length() > 17) displayName = displayName.substring(0, 14) + "...";

    char rowText[190];
    if (rec.wifi) {
        snprintf(rowText, sizeof(rowText),
                 "%s  %s\n%ld dBm  CH%ld  %s  x%u",
                 rec.fromC5 ? "5G" : "2.4", displayName.c_str(),
                 (long)rec.rssi, (long)rec.channel,
                 authLabel(rec.auth), (unsigned)rec.seenCount);
    } else {
        snprintf(rowText, sizeof(rowText),
                 "%s  %s\n%ld dBm  %s  x%u",
                 rec.trackerLike ? "TRACKER" : "BLE", displayName.c_str(),
                 (long)rec.rssi, rec.connectable ? "CONN" : "ADV",
                 (unsigned)rec.seenCount);
    }
    lv_label_set_text(rec.label, rowText);
}

void createWardriveRecordRow(WardriveRecord &rec)
{
    LV_UNUSED(rec);
}

void initWardriveRecordRows()
{
    // Intentionally disabled. Wardrive uses one bounded scrolling text label
    // for the live feed. This avoids dynamic/reflowing LVGL child rows during
    // scans and touch scrolling.
}

void syncWardriveRecordRows()
{
    wardriveUiSyncPending = false;
    wardriveUiSyncCursor = 0;
}

void wardriveFeedScrollEvent(lv_event_t *e)
{
    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_SCROLL_BEGIN) {
        wardriveFeedScrolling = true;
        wardriveDetailIgnoreUntilMs = millis() + 900;
        noteActivity();
        return;
    }
    if (code == LV_EVENT_SCROLL_END) {
        wardriveFeedScrolling = false;
        wardriveDetailIgnoreUntilMs = millis() + 900;
        // Defer row creation/update until the Wardrive timer tick.
        noteActivity();
    }
}

WardriveRecord *newWardriveRecord(
    bool wifi,
    const String &name,
    const String &id,
    bool fromC5 = false)
{
    if (!wardrivePageActive || currentPage != Page::Wardrive) return nullptr;
    if (wardriveRecordCount >= WARD_MAX_RECORDS) return nullptr;

    if (ESP.getFreeHeap() < WARD_HEAP_PAUSE_BYTES) {
        wardriveMemoryPaused = true;
        return nullptr;
    }

    WardriveRecord &rec = wardriveRecords[wardriveRecordCount++];
    rec = WardriveRecord{};
    rec.active = true;
    rec.wifi = wifi;
    rec.fromC5 = fromC5;
    rec.name = name;
    rec.id = id;
    rec.seenCount = 1;
    rec.firstSeenMs = millis();
    rec.lastSeenMs = rec.firstSeenMs;
    snapshotWardriveLocation(rec);
    return &rec;
}

void wardriveDetailCloseEvent(lv_event_t *e)
{
    if (!wardrivePageActive || currentPage != Page::Wardrive) return;
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    wardriveDetailOpen = false;

    if (wardriveDetailOverlay) {
        lv_obj_add_flag(
            wardriveDetailOverlay,
            LV_OBJ_FLAG_HIDDEN
        );
    }

    // Background records may have changed while the overlay was open.
    // Resume them gradually through the existing batched UI sync.
    wardriveUiSyncPending = true;
    wardriveUiSyncCursor = 0;

    if (wardriveFeedScroll) {
        lv_obj_update_layout(wardriveFeedScroll);
        lv_obj_invalidate(wardriveFeedScroll);
    }

    if (screen) {
        lv_obj_invalidate(screen);
    }

    noteActivity();
}

void wardriveDetailEvent(lv_event_t *e)
{
    if (!wardrivePageActive || currentPage != Page::Wardrive) return;
    if (lv_event_get_code(e) != LV_EVENT_SHORT_CLICKED || !clickAllowed()) return;
    if (wardriveFeedScrolling ||
        (int32_t)(millis() - wardriveDetailIgnoreUntilMs) < 0) {
        return;
    }
    auto *rec = static_cast<WardriveRecord *>(lv_event_get_user_data(e));
    if (!rec || !wardriveDetailOverlay || !wardriveDetailTitle || !wardriveDetailBody) return;

    noteActivity();
    hapticTap();

    String title = rec->wifi
        ? (rec->fromC5 ? "WI-FI 5G  " : "WI-FI 2.4  ")
        : (rec->trackerLike ? "TRACKER-LIKE  " : "BLE  ");
    title += rec->name.length() ? rec->name : rec->id;
    if (title.length() > 29) title = title.substring(0, 26) + "...";
    lv_label_set_text(wardriveDetailTitle, title.c_str());

    const uint32_t firstSec = rec->firstSeenMs >= wardriveSessionStartedMs ? (rec->firstSeenMs - wardriveSessionStartedMs) / 1000 : 0;
    const uint32_t lastSec = rec->lastSeenMs >= wardriveSessionStartedMs ? (rec->lastSeenMs - wardriveSessionStartedMs) / 1000 : 0;

    String body;
    body.reserve(430);
    body += "ID\n";
    body += rec->id;
    body += "\n\nRSSI   ";
    body += String(rec->rssi);
    body += " dBm   ";
    body += signalLabel(rec->rssi);
    body += "\nSEEN   " + String(rec->seenCount) + "x";
    body += "\nFIRST   " + String(firstSec / 60) + ":";
    if ((firstSec % 60) < 10) body += "0";
    body += String(firstSec % 60);
    body += "   LAST   " + String(lastSec / 60) + ":";
    if ((lastSec % 60) < 10) body += "0";
    body += String(lastSec % 60);

    if (rec->wifi) {
        body += rec->fromC5
            ? "\nBAND   5 GHz\nSOURCE   M33K X C5"
            : "\nBAND   2.4 GHz\nSOURCE   T-WATCH";
        body += "\nCHANNEL   " + String(rec->channel);
        body += "\nSECURITY   ";
        body += authLabel(rec->auth);
        if (rec->id.length() >= 8) body += "\nOUI PREFIX   " + rec->id.substring(0, 8);
    } else {
        body += "\nRADIO   BLE";
        if (rec->trackerLike) body += "  |  TRACKER-LIKE";
        body += "\nCONNECTABLE   ";
        body += rec->connectable ? "YES" : "NO";
        body += "\nSCANNABLE   ";
        body += rec->scannable ? "YES" : "NO";
        body += "\nSERVICE UUIDS   " + String(rec->serviceCount);
        body += "\nMFR DATA BLOCKS   " + String(rec->manufacturerCount);
        body += "\nTX POWER   ";
        body += rec->haveTxPower ? String(rec->txPower) + " dBm" : String("--");
    }

    body += "\nGPS   ";
    if (rec->haveLocation) {
        body += String(rec->lat, 5) + ", " + String(rec->lon, 5);
        body += "\nSATS   " + String(rec->sats);
    } else {
        body += "NO FIX AT LAST SIGHTING";
    }

    lv_label_set_text(wardriveDetailBody, body.c_str());
    wardriveDetailOpen = true;
    lv_obj_remove_flag(wardriveDetailOverlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(wardriveDetailOverlay);
}

void resetWardriveRecords()
{
    wardriveFeedScrolling = false;
    wardriveDetailOpen = false;
    wardriveUiSyncPending = false;
    wardriveUiSyncCursor = 0;
    wardriveRecordCount = 0;

    for (auto &rec : wardriveRecords) {
        rec = WardriveRecord{};
    }

    wardriveFeedPageStart = 0;
    wardriveFeedFollowLatest = true;
}

uint32_t wardriveFeedHash(
    const String &value,
    uint32_t seed)
{
    uint32_t hash = seed;

    for (size_t i = 0; i < value.length(); ++i) {
        hash ^= static_cast<uint8_t>(value[i]);
        hash *= 16777619UL;
    }

    hash ^= hash >> 16;
    hash *= 0x7FEB352DUL;
    hash ^= hash >> 15;

    return hash;
}

bool wardriveFeedMarkUnique(
    bool wifi,
    const String &id)
{
    if (id.length() == 0) return false;

    String key;
    key.reserve(id.length() + 2);
    key += wifi ? "W|" : "B|";
    key += id;

    const uint32_t h1 =
        wardriveFeedHash(
            key,
            2166136261UL
        );

    const uint32_t h2 =
        wardriveFeedHash(
            key,
            0x9E3779B9UL
        );

    const uint16_t bit1 =
        static_cast<uint16_t>(
            h1 % WARD_FEED_BLOOM_BITS
        );

    const uint16_t bit2 =
        static_cast<uint16_t>(
            h2 % WARD_FEED_BLOOM_BITS
        );

    const uint16_t byte1 = bit1 >> 3;
    const uint16_t byte2 = bit2 >> 3;

    const uint8_t mask1 =
        static_cast<uint8_t>(
            1U << (bit1 & 7U)
        );

    const uint8_t mask2 =
        static_cast<uint8_t>(
            1U << (bit2 & 7U)
        );

    const bool alreadySeen =
        (wardriveFeedBloom[byte1] & mask1) &&
        (wardriveFeedBloom[byte2] & mask2);

    wardriveFeedBloom[byte1] |= mask1;
    wardriveFeedBloom[byte2] |= mask2;

    return !alreadySeen;
}

String wardriveMakeFeedPath()
{
    if (wardriveLogPath.length() == 0) {
        return "";
    }

    String path = wardriveLogPath;

    if (path.endsWith(".csv")) {
        path.remove(path.length() - 4);
    }

    path += ".feed";
    return path;
}

bool wardriveAppendFeedRecord(
    const String &line)
{
    if (!wardriveSdLogging ||
        !m33kSdReady ||
        wardriveLogPath.length() == 0) {
        return false;
    }

    if (wardriveFeedPath.length() == 0) {
        wardriveFeedPath =
            wardriveMakeFeedPath();
    }

    if (wardriveFeedPath.length() == 0) {
        return false;
    }

    if (!instance.lockSPI(
            pdMS_TO_TICKS(100))) {
        return false;
    }

    File file = SD.open(
        wardriveFeedPath.c_str(),
        FILE_APPEND
    );

    if (!file) {
        instance.unlockSPI();
        return false;
    }

    char record[WARD_FEED_RECORD_BYTES] = {};

    String clipped = line;

    if (clipped.length() >=
        WARD_FEED_RECORD_BYTES) {

        clipped = clipped.substring(
            0,
            WARD_FEED_RECORD_BYTES - 1
        );
    }

    memcpy(
        record,
        clipped.c_str(),
        clipped.length()
    );

    const size_t written =
        file.write(
            reinterpret_cast<const uint8_t *>(record),
            sizeof(record)
        );

    file.close();
    instance.unlockSPI();

    if (written != sizeof(record)) {
        return false;
    }

    ++wardriveFeedStoredCount;
    return true;
}

bool wardriveReadFeedRecord(
    uint32_t index,
    String &out)
{
    out = "";

    if (!m33kSdReady ||
        wardriveFeedPath.length() == 0 ||
        index >= wardriveFeedStoredCount) {
        return false;
    }

    if (!instance.lockSPI(
            pdMS_TO_TICKS(100))) {
        return false;
    }

    File file = SD.open(
        wardriveFeedPath.c_str(),
        FILE_READ
    );

    if (!file) {
        instance.unlockSPI();
        return false;
    }

    const uint32_t offset =
        index *
        static_cast<uint32_t>(
            WARD_FEED_RECORD_BYTES
        );

    if (!file.seek(offset)) {
        file.close();
        instance.unlockSPI();
        return false;
    }

    char record[
        WARD_FEED_RECORD_BYTES + 1
    ] = {};

    const size_t got =
        file.read(
            reinterpret_cast<uint8_t *>(
                record
            ),
            WARD_FEED_RECORD_BYTES
        );

    file.close();
    instance.unlockSPI();

    if (got != WARD_FEED_RECORD_BYTES) {
        return false;
    }

    record[WARD_FEED_RECORD_BYTES] = '\0';
    out = String(record);

    return true;
}


void wardriveResetSweepFeed()
{
    wardriveSweepLineCount = 0;
    wardriveFeedPageStart = 0;
    wardriveFeedFollowLatest = true;

    wardriveFeedStoredCount = 0;
    wardriveFeedPath = "";

    memset(
        wardriveFeedBloom,
        0,
        sizeof(wardriveFeedBloom)
    );

    for (auto &line : wardriveSweepLines) {
        line = "";
    }
}

void wardriveAddSweepLine(const String &line)
{
    if (line.length() == 0) return;

    // Never allow scan-controlled SSID/BLE text to alter LVGL row geometry.
    // Replace newlines, tabs and control bytes with spaces.
    String displayLine;
    displayLine.reserve(32);

    for (size_t i = 0;
         i < line.length() && displayLine.length() < 30;
         ++i) {

        const uint8_t c =
            static_cast<uint8_t>(line[i]);

        if (c >= 32 && c <= 126) {
            displayLine += static_cast<char>(c);
        } else {
            displayLine += ' ';
        }
    }

    // Trim repeated trailing spaces.
    while (displayLine.endsWith(" ")) {
        displayLine.remove(
            displayLine.length() - 1
        );
    }

    if (displayLine.length() == 0) {
        displayLine = "<device>";
    }

    // The live feed is intentionally RAM-only.
    //
    // It refreshes frequently and keeping each visual row on SD
    // creates unnecessary SPI/file contention with the structured
    // Wardrive CSV logger.
    //
    // CSV logging remains the authoritative long-session record.

    // RAM is only the bounded LIVE display window.
    if (wardriveSweepLineCount >= WARD_MAX_FEED_LINES) {
        for (uint16_t i = 1;
             i < WARD_MAX_FEED_LINES;
             ++i) {

            wardriveSweepLines[i - 1] =
                wardriveSweepLines[i];
        }

        wardriveSweepLines[
            WARD_MAX_FEED_LINES - 1
        ] = displayLine;

    } else {
        wardriveSweepLines[
            wardriveSweepLineCount++
        ] = displayLine;
    }

    wardriveFeedDirty = true;
}

void updateWardriveBunnyProgress()
{
    const uint16_t total =
        wardriveSweepLineCount;

    const uint16_t maxStart =
        total > WARD_FEED_PAGE_ROWS
            ? total - WARD_FEED_PAGE_ROWS
            : 0;

    uint8_t level = 0;

    if (maxStart > 0) {
        // Convert current page position to five visual steps:
        // 0 = first bunny only
        // 4 = all five bunnies
        const uint32_t scaled =
            static_cast<uint32_t>(
                wardriveFeedPageStart
            ) * 4UL;

        level = static_cast<uint8_t>(
            min<uint32_t>(
                4UL,
                (scaled + (maxStart / 2UL)) /
                    maxStart
            )
        );
    }

    for (uint8_t i = 0;
         i < WARD_BUNNY_PROGRESS_COUNT;
         ++i) {

        const bool filled =
            total > 0 && i <= level;

        const lv_color_t color =
            filled
                ? lv_color_hex(0xFF52C8)
                : lv_color_hex(0x55EEFF);

        const lv_opa_t fillOpacity =
            filled
                ? LV_OPA_COVER
                : LV_OPA_TRANSP;

        lv_obj_t *parts[] = {
            wardriveBunnyHeads[i],
            wardriveBunnyEarL[i],
            wardriveBunnyEarR[i]
        };

        for (lv_obj_t *part : parts) {
            if (!part) continue;

            lv_obj_set_style_border_color(
                part,
                color,
                0
            );

            lv_obj_set_style_bg_color(
                part,
                color,
                0
            );

            lv_obj_set_style_bg_opa(
                part,
                fillOpacity,
                0
            );
        }
    }
}

void updateWardriveFeed()
{
    if (!wardrivePageActive ||
        currentPage != Page::Wardrive ||
        wardriveExitToReconPending ||
        !wardriveFeedPageLabel) {
        return;
    }

    const uint16_t total =
        wardriveSweepLineCount;

    if (total == 0) {
        for (uint8_t i = 0;
             i < WARD_FEED_PAGE_ROWS;
             ++i) {

            if (!wardriveFeedRows[i]) continue;

            lv_label_set_text(
                wardriveFeedRows[i],
                i == 0
                    ? (
                        wardriveRunning
                            ? "Scanning... waiting"
                            : "Tap START"
                      )
                    : ""
            );
        }

        lv_label_set_text(
            wardriveFeedPageLabel,
            "0 / 0"
        );

        updateWardriveBunnyProgress();
        wardriveFeedDirty = false;
        return;
    }

    const uint16_t maxStart =
        total > WARD_FEED_PAGE_ROWS
            ? total - WARD_FEED_PAGE_ROWS
            : 0;

    if (wardriveFeedFollowLatest) {
        wardriveFeedPageStart = maxStart;

    } else if (
        wardriveFeedPageStart > maxStart) {

        wardriveFeedPageStart = maxStart;
    }

    for (uint8_t row = 0;
         row < WARD_FEED_PAGE_ROWS;
         ++row) {

        lv_obj_t *label =
            wardriveFeedRows[row];

        if (!label) continue;

        const uint16_t index =
            static_cast<uint16_t>(
                wardriveFeedPageStart + row
            );

        if (index < total) {
            lv_label_set_text(
                label,
                wardriveSweepLines[index].c_str()
            );
        } else {
            lv_label_set_text(
                label,
                ""
            );
        }
    }

    const uint16_t firstShown =
        static_cast<uint16_t>(
            wardriveFeedPageStart + 1
        );

    const uint16_t lastShown =
        min<uint16_t>(
            total,
            static_cast<uint16_t>(
                wardriveFeedPageStart +
                WARD_FEED_PAGE_ROWS
            )
        );

    // Show RAM-window position. Full SD history count remains separate.
    lv_label_set_text_fmt(
        wardriveFeedPageLabel,
        "%u-%u/%u",
        static_cast<unsigned>(firstShown),
        static_cast<unsigned>(lastShown),
        static_cast<unsigned>(total)
    );

    updateWardriveBunnyProgress();

    wardriveFeedDirty = false;
}

void wardriveFeedPageEvent(lv_event_t *e)
{
    if (!wardrivePageActive ||
        currentPage != Page::Wardrive ||
        wardriveExitToReconPending) {
        return;
    }

    if (lv_event_get_code(e) != LV_EVENT_CLICKED ||
        !clickAllowed()) {
        return;
    }

    const uint16_t total =
        wardriveSweepLineCount;

    if (total == 0) return;

    const intptr_t direction =
        reinterpret_cast<intptr_t>(
            lv_event_get_user_data(e)
        );

    const uint16_t maxStart =
        total > WARD_FEED_PAGE_ROWS
            ? total - WARD_FEED_PAGE_ROWS
            : 0;

    if (direction < 0) {
        wardriveFeedFollowLatest = false;

        if (wardriveFeedPageStart >=
            WARD_FEED_PAGE_ROWS) {

            wardriveFeedPageStart -=
                WARD_FEED_PAGE_ROWS;
        } else {
            wardriveFeedPageStart = 0;
        }

    } else {
        const uint32_t nextStart =
            wardriveFeedPageStart +
            WARD_FEED_PAGE_ROWS;

        if (nextStart < maxStart) {
            wardriveFeedPageStart =
                nextStart;

            wardriveFeedFollowLatest =
                false;

        } else {
            wardriveFeedPageStart =
                maxStart;

            wardriveFeedFollowLatest =
                true;
        }
    }

    wardriveFeedDirty = true;
    updateWardriveFeed();

    noteActivity();
    hapticTap();
}

bool ensureM33KSdDirectories()
{
    if (!instance.lockSPI(pdMS_TO_TICKS(250))) {
        return false;
    }

    bool ok = true;
    if (!SD.exists("/M33KX")) {
        ok = SD.mkdir("/M33KX");
    }
    if (ok && !SD.exists("/M33KX/WARDRIVE")) {
        ok = SD.mkdir("/M33KX/WARDRIVE");
    }

    instance.unlockSPI();
    return ok;
}

bool ensureM33KSdReady()
{
    if (instance.isCardReady()) {
        m33kSdReady = ensureM33KSdDirectories();
        return m33kSdReady;
    }

    // Support inserting a card after boot as well as cards present at startup.
    instance.powerControl(POWER_SD_CARD, true);
    delay(25);

    if (!instance.installSD()) {
        m33kSdReady = false;
        return false;
    }

    m33kSdReady = ensureM33KSdDirectories();
    return m33kSdReady;
}

String m33kRtcTimestamp()
{
    const RTC_DateTime dt = instance.rtc.getDateTime();
    char buf[24] = {};
    snprintf(
        buf,
        sizeof(buf),
        "%04u-%02u-%02u %02u:%02u:%02u",
        (unsigned)dt.getYear(),
        (unsigned)dt.getMonth(),
        (unsigned)dt.getDay(),
        (unsigned)dt.getHour(),
        (unsigned)dt.getMinute(),
        (unsigned)dt.getSecond()
    );
    return String(buf);
}

String m33kNewWardriveLogPath()
{
    const RTC_DateTime dt = instance.rtc.getDateTime();
    char buf[72] = {};
    snprintf(
        buf,
        sizeof(buf),
        "/M33KX/WARDRIVE/%04u%02u%02u_%02u%02u%02u_%04lu.csv",
        (unsigned)dt.getYear(),
        (unsigned)dt.getMonth(),
        (unsigned)dt.getDay(),
        (unsigned)dt.getHour(),
        (unsigned)dt.getMinute(),
        (unsigned)dt.getSecond(),
        (unsigned long)(millis() % 10000UL)
    );
    return String(buf);
}

void m33kRtcTimestampBuf(char *buf, size_t len)
{
    if (!buf || len == 0) return;

    const RTC_DateTime dt = instance.rtc.getDateTime();
    snprintf(
        buf,
        len,
        "%04u-%02u-%02u %02u:%02u:%02u",
        (unsigned)dt.getYear(),
        (unsigned)dt.getMonth(),
        (unsigned)dt.getDay(),
        (unsigned)dt.getHour(),
        (unsigned)dt.getMinute(),
        (unsigned)dt.getSecond()
    );
}

void m33kCsvWriteQuoted(File &file, const String &value)
{
    file.write('"');

    for (size_t i = 0; i < value.length(); ++i) {
        const char ch = value[i];

        if (ch == '"') {
            file.write('"');
            file.write('"');
        } else if (ch == '\r' || ch == '\n') {
            file.write(' ');
        } else {
            file.write(static_cast<uint8_t>(ch));
        }
    }

    file.write('"');
}

void writeWardriveCsvRecord(
    File &file,
    const WardriveRecord &rec,
    const char *timestamp)
{
    file.write('"');
    file.print(timestamp ? timestamp : "");
    file.write('"');
    file.write(',');

    file.print(rec.wifi ? "WIFI" : "BLE");
    file.write(',');
    file.print(rec.fromC5 ? "C5" : "WATCH");
    file.write(',');
    file.print(
        rec.wifi
            ? (rec.fromC5 || rec.channel > 14 ? "5GHz" : "2.4GHz")
            : "BLE"
    );
    file.write(',');

    m33kCsvWriteQuoted(file, rec.name);
    file.write(',');
    m33kCsvWriteQuoted(file, rec.id);
    file.write(',');

    file.print(rec.rssi);
    file.write(',');

    if (rec.wifi) file.print(rec.channel);
    file.write(',');

    if (rec.wifi) {
        m33kCsvWriteQuoted(file, String(authLabel(rec.auth)));
    } else {
        file.print("\"\"");
    }
    file.write(',');

    if (rec.haveLocation) file.print(rec.lat, 6);
    file.write(',');
    if (rec.haveLocation) file.print(rec.lon, 6);
    file.write(',');
    if (rec.haveLocation) file.print(rec.sats);
    file.write(',');

    file.print(rec.seenCount);
    file.write(',');
    file.print(rec.trackerLike ? 1 : 0);
    file.write(',');

    if (rec.haveTxPower) file.print(rec.txPower);
    file.write(',');

    file.print(rec.serviceCount);
    file.write(',');
    file.print(rec.manufacturerCount);
    file.write(',');
    file.print(rec.connectable ? 1 : 0);
    file.write(',');
    file.print(rec.scannable ? 1 : 0);
    file.write('\n');
}

bool appendWardriveRecordGroupToSd(bool wifiRecords, uint32_t seenSinceMs)
{
    if (!wardriveSdLogging || !m33kSdReady || wardriveLogPath.length() == 0) {
        return false;
    }

    if (!instance.lockSPI(pdMS_TO_TICKS(350))) {
        return false;
    }

    File file = SD.open(wardriveLogPath.c_str(), FILE_APPEND);
    if (!file) {
        instance.unlockSPI();

        // Preserve physical card-ready state. This may be a transient
        // SPI/file contention failure rather than card removal.
        wardriveSdLogging = false;

        Serial.printf(
            "[M33K][SD] Wardrive CSV append failed after %lu rows; logging paused\n",
            (unsigned long)wardriveSdRowsWritten
        );

        return false;
    }

    char timestamp[24] = {};
    m33kRtcTimestampBuf(timestamp, sizeof(timestamp));

    uint32_t rows = 0;

    for (uint16_t i = 0; i < wardriveRecordCount; ++i) {
        WardriveRecord &rec = wardriveRecords[i];

        if (!rec.active ||
            rec.wifi != wifiRecords ||
            rec.lastSeenMs < seenSinceMs) {
            continue;
        }

        writeWardriveCsvRecord(file, rec, timestamp);
        ++rows;
    }

    file.close();
    instance.unlockSPI();

    wardriveSdRowsWritten += rows;
    return rows > 0;
}

bool startWardriveSdLog()
{
    wardriveSdLogging = false;
    wardriveSdRowsWritten = 0;
    wardriveLogPath = "";

    if (!ensureM33KSdReady()) {
        Serial.println("[M33K][SD] No usable SD card; Wardrive will run without logging");
        return false;
    }

    wardriveLogPath = m33kNewWardriveLogPath();

    if (!instance.lockSPI(pdMS_TO_TICKS(350))) {
        wardriveLogPath = "";
        return false;
    }

    File file = SD.open(wardriveLogPath.c_str(), FILE_WRITE);
    bool ok = static_cast<bool>(file);
    if (ok) {
        const char *header =
            "timestamp,type,source,band,name,id,rssi,channel,security,"
            "latitude,longitude,satellites,seen_count,tracker_like,tx_power,"
            "service_count,manufacturer_count,connectable,scannable\n";
        const size_t written = file.print(header);
        file.flush();
        ok = written == strlen(header);
        file.close();
    }

    instance.unlockSPI();

    if (!ok) {
        Serial.println("[M33K][SD] Could not create Wardrive CSV");
        wardriveLogPath = "";
        m33kSdReady = false;
        return false;
    }

    wardriveSdLogging = true;
    Serial.printf("[M33K][SD] Wardrive logging: %s\n", wardriveLogPath.c_str());
    return true;
}

void stopWardriveSdLog()
{
    wardriveSdLogging = false;
}

void updateWardriveUi()
{
    // Wardrive data collection continues while the display is
    // dark. Defer LVGL visual synchronization until wake.
    if (screenSleeping) {
        wardriveUiSyncPending = true;
        return;
    }


    // Time-share NimBLE and display SPI.
    // Data collection continues; only LVGL drawing pauses.
    const bool wardriveRadioUiHold =
        c5DiscoveryInFlight ||
        c5ScanInFlight ||
        c5TransportConnected() ||
        wardrivePhase == WardrivePhase::BleScanning ||
        (int32_t)(millis() - wardriveDisplayResumeMs) < 0;

    if (wardriveRadioUiHold) {
        wardriveUiSyncPending = true;
        return;
    }


    if (!wardrivePageActive ||
        currentPage != Page::Wardrive ||
        wardriveExitToReconPending) {
        return;
    }

    if (wardriveWifiLabel) {
        lv_label_set_text_fmt(wardriveWifiLabel, "WI-FI\n%u", (unsigned)wardriveTotalWifiCount);
    }
    if (wardriveBleLabel) {
        lv_label_set_text_fmt(
            wardriveBleLabel,
            "BLE\n%lu",
            (unsigned long)wardriveTotalBleCount
        );
    }

    const bool fix = gpsParser.location.isValid() && gpsParser.location.age() < 5000;
    if (wardriveGpsLabel) {
        if (fix) {
            unsigned sats = gpsParser.satellites.isValid() ? (unsigned)gpsParser.satellites.value() : 0U;
            lv_label_set_text_fmt(wardriveGpsLabel, "GPS FIX\n%u SAT", sats);
            lv_obj_set_style_text_color(wardriveGpsLabel, lv_color_hex(0x7CFF45), 0);
        } else {
            lv_label_set_text(wardriveGpsLabel, "GPS\nSEARCHING");
            lv_obj_set_style_text_color(wardriveGpsLabel, lv_color_hex(0xF5FF3B), 0);
        }
    }

    if (wardriveElapsedLabel) {
        uint32_t elapsed = wardriveLastElapsedSeconds;
        if (wardriveRunning) {
            elapsed = (millis() - wardriveSessionStartedMs) / 1000;
            wardriveLastElapsedSeconds = elapsed;
        }
        lv_label_set_text_fmt(
            wardriveElapsedLabel,
            "SESSION  %02lu:%02lu",
            (unsigned long)(elapsed / 60),
            (unsigned long)(elapsed % 60)
        );
    }

    if (wardriveLocationLabel) {
        if (fix) {
            const unsigned sats = gpsParser.satellites.isValid()
                ? (unsigned)gpsParser.satellites.value() : 0U;
            lv_label_set_text_fmt(
                wardriveLocationLabel,
                "GPS FIX  %u SAT\n%.4f\n%.4f",
                sats,
                gpsParser.location.lat(),
                gpsParser.location.lng()
            );
        } else {
            lv_label_set_text(wardriveLocationLabel, "GPS SEARCHING\n--.----\n--.----");
        }
    }

    if (wardriveSdLabel) {
        if (wardriveSdLogging) {
            lv_label_set_text_fmt(
                wardriveSdLabel,
                "SD LOGGING\n%lu ROWS",
                (unsigned long)wardriveSdRowsWritten
            );
            lv_obj_set_style_text_color(wardriveSdLabel, lv_color_hex(0x7CFF45), 0);
        } else if (m33kSdReady) {
            lv_label_set_text(wardriveSdLabel, "SD READY");
            lv_obj_set_style_text_color(wardriveSdLabel, lv_color_hex(0x55EEFF), 0);
        } else {
            lv_label_set_text(wardriveSdLabel, "SD OFFLINE");
            lv_obj_set_style_text_color(wardriveSdLabel, lv_color_hex(0xF5FF3B), 0);
        }
    }

    if (wardriveFeedStateLabel) {
        lv_label_set_text(
            wardriveFeedStateLabel,
            wardriveRunning ? "LIVE SCAN FEED" : "SCAN FEED"
        );
    }

    // Reassert the visible button state from the authoritative session flag.
    // This also recovers the label after a delayed scan callback/UI refresh.
    if (wardriveStartLabel) {
        lv_label_set_text(
            wardriveStartLabel,
            wardriveRunning ? "STOP" : "START"
        );
    }

    if (wardriveFeedDirty) {
        updateWardriveFeed();
    }
}

void stopWardriveTool()
{
    wardrivePageActive = false;
    wardriveUiSyncPending = false;
    wardriveFeedScrolling = false;
    wardriveDetailOpen = false;

    if (wardriveTimer) {
        lv_timer_del(wardriveTimer);
        wardriveTimer = nullptr;
    }

    // Page navigation must not synchronously stop an active NimBLE scan.
    // Let asynchronous radio work finish on its own so LVGL/back navigation
    // can never be held hostage by the BLE stack.
    const int16_t wifiState = WiFi.scanComplete();
    if (wifiState != WIFI_SCAN_RUNNING) {
        WiFi.scanDelete();
    }

    if (bleScannerInitialized) {
        NimBLEScan *scan = NimBLEDevice::getScan();
        if (!scan->isScanning()) {
            scan->clearResults();
            scan->setMaxResults(MAX_BLE_DETAILS);
        }
    }

    // Preserve/arm deferred cleanup when leaving Wardrive.
    // Back navigation can occur while C5 discovery, authentication,
    // or a 5 GHz scan is still active.
    //
    // CRITICAL: stopWardriveTool() runs on EVERY page teardown, not just when
    // leaving Wardrive. The Wi-Fi scanner and live graph also use the shared
    // C5 link, so we must NOT arm a C5 teardown merely because C5 is connected
    // (c5TransportConnected()) or a C5 scan is in flight (c5ScanInFlight) -
    // those are usually owned by the Wi-Fi tool. Doing so was repeatedly
    // disconnecting C5 on the graph: it would reconnect, then this cleanup
    // would tear it right back down, in a loop. The explicit Wardrive STOP
    // path sets wardriveStopCleanupPending itself, so real Wardrive cleanup is
    // still honored; here we only arm it when a Wardrive session was actually
    // running (or a C5 cycle it owns is still active).
    const bool wasWardriveRunning = wardriveRunning;
    const bool wardriveRadioCleanupNeeded =
        wardriveStopCleanupPending ||
        wardriveC5CycleActive ||
        (wasWardriveRunning &&
         (c5DiscoveryInFlight ||
          c5ScanInFlight ||
          c5TransportConnected()));

    wardriveRunning = false;
    wardriveStopCleanupPending =
        wardriveRadioCleanupNeeded;

    if (wardriveStopCleanupPending &&
        c5Client &&
        c5Client->isConnected()) {

        c5Client->disconnect();

        Serial.println(
            "[M33K WARD] exit requested C5 disconnect"
        );
    }

    stopWardriveSdLog();
    wardrivePhase = WardrivePhase::Stopped;
}

bool startWardriveBleScan()
{
    initBleScanner();
    NimBLEScan *scan = NimBLEDevice::getScan();

    // Never steal NimBLE from C5 discovery/GATT activity.
    if (scan->isScanning()) {
        Serial.println(
            "[M33K WARD] BLE sample deferred; scanner busy"
        );
        return false;
    }

    // C5 client must be completely gone before target BLE starts.
    if (c5Client ||
        c5DiscoveryInFlight ||
        c5ScanInFlight ||
        c5TransportConnected()) {

        Serial.println(
            "[M33K WARD] BLE sample deferred; C5 cleanup pending"
        );
        return false;
    }

    scan->clearResults();
    scan->setMaxResults(12);
    bleScanFinished = false;
    bleScanEndReason = 0;
    wardrivePhase = WardrivePhase::BleScanning;
    if (wardriveStatusLabel) lv_label_set_text(wardriveStatusLabel, "SESSION LIVE  |  SCANNING BLE...");
    const bool started = scan->start(900, false, true);
    if (!started) scan->setMaxResults(MAX_BLE_DETAILS);
    return started;
}

void finishWardriveWifiScan()
{
    if (!wardrivePageActive || !wardriveRunning) {
        WiFi.scanDelete();
        return;
    }

    const int16_t result = WiFi.scanComplete();
    if (result == WIFI_SCAN_FAILED) {
        wardrivePhase = WardrivePhase::Failed;
        if (wardriveStatusLabel) lv_label_set_text(wardriveStatusLabel, "WARDRIVE WI-FI SCAN FAILED");
        WiFi.scanDelete();
        return;
    }

    // Count every raw watch-side Wi-Fi result before the shared
    // UI/detail cache truncates the list. The cache stays bounded,
    // but the Wardrive session total does not.
    if (result > 0) {
        for (int16_t i = 0; i < result; ++i) {
            const String rawBssid = WiFi.BSSIDstr(i);

            if (rawBssid.length() > 0) {
                wardriveTrackWifiTotal(rawBssid);
            }
        }
    }

    captureLocalWifiResults(result);
    mergeC5WifiResults();

    const bool applyFreshC5 =
        !wardriveC5CacheApplied ||
        wardriveAppliedC5Generation != c5ScanGeneration;

    const uint32_t sdSeenSinceMs = millis();

    for (uint8_t i = 0; i < wifiDetailCount; ++i) {
        const WifiResultDetail &detail = wifiDetails[i];
        const bool band5 = detail.fromC5 || detail.channel > 14;

        // Do not count the same cached C5 sweep repeatedly while a new remote
        // scan is still running. Local watch results are fresh every sweep.
        if (band5 && !applyFreshC5) continue;

        const String &ssid = detail.ssid;
        const String &bssid = detail.bssid;

        // Session total remains unique for the entire drive.
        wardriveTrackWifiTotal(bssid);

        String key =
            ssid + "|" + bssid;
        if (!wardriveHasWifi(key) && wardriveWifiCount < WARD_MAX_WIFI) {
            wardriveWifiKeys[wardriveWifiCount++] = key;
        }

        WardriveRecord *rec = findWardriveRecord(true, bssid);
        const bool existed = rec != nullptr;
        if (!rec) rec = newWardriveRecord(true, ssid, bssid, band5);
        if (rec) {
            rec->name = ssid;
            rec->fromC5 = band5;
            rec->rssi = detail.rssi;
            rec->channel = detail.channel;
            rec->auth = detail.auth;
            if (existed && rec->seenCount < 65535) ++rec->seenCount;
            rec->lastSeenMs = millis();
            snapshotWardriveLocation(*rec);

            // Feed history is handled independently below.
            // This RAM record can still update details when available.
        }

        // Live feed is intentionally NOT session-unique.
        // Each completed Wi-Fi scan can refresh currently visible APs.
        // wardriveAddSweepLine() maintains the bounded rolling window
        // and automatically drops the oldest rows when it reaches 120.
        String line =
            band5
                ? "5G  "
                : "2.4  ";

        line +=
            ssid.length()
                ? ssid
                : String("<hidden>");

        line += "  ";
        line += String(detail.rssi);
        line += "dBm CH";
        line += String(detail.channel);

        wardriveAddSweepLine(line);
    }

    appendWardriveRecordGroupToSd(true, sdSeenSinceMs);

    if (applyFreshC5) {
        wardriveC5CacheApplied = true;
        wardriveAppliedC5Generation = c5ScanGeneration;
    }

    // If the C5 completed the sweep we requested, its newly
    // published cache has now been merged into Wardrive.
    // Release the BLE client immediately to recover DMA RAM.
    if (wardriveC5CycleActive &&
        !c5ScanInFlight &&
        c5ScanGeneration != wardriveC5RequestGeneration) {

        Serial.printf(
            "[M33K WARD] C5 generation %lu consumed; releasing transport\n",
            (unsigned long)c5ScanGeneration
        );

        wardriveC5CycleActive = false;
        wardriveNextC5CycleMs =
            millis() + WARD_C5_INTERVAL_MS;

        // Give NimBLE/display DMA allocations time to settle before
        // launching the watch-side BLE scanner.
        wardriveNextBleSampleMs =
            millis() + 2500UL;

        if (c5Client && c5Client->isConnected()) {
            c5Client->disconnect();
        }
    }

    WiFi.scanDelete();
    updateWardriveUi();

    // Return to idle. wardriveTimerCb() independently schedules
    // watch BLE, Wi-Fi, and the optional C5 without coupling them.
    wardrivePhase = WardrivePhase::Idle;
    wardriveNextSweepMs = millis() + 1800;

    if (wardriveStatusLabel) {
        lv_label_set_text(
            wardriveStatusLabel,
            "SESSION LIVE"
        );
    }
}

void finishWardriveBleScan()
{
    if (!wardrivePageActive || !wardriveRunning) {
        if (bleScannerInitialized) {
            NimBLEDevice::getScan()->clearResults();
        }
        return;
    }

    if (!bleScannerInitialized) {
        wardrivePhase = WardrivePhase::Failed;
        return;
    }
    NimBLEScan *scan = NimBLEDevice::getScan();
    NimBLEScanResults results = scan->getResults();

    const uint32_t sdSeenSinceMs = millis();

    for (int i = 0; i < results.getCount(); ++i) {
        const NimBLEAdvertisedDevice *dev = results.getDevice(i);
        if (!dev) continue;
        String addr = String(dev->getAddress().toString().c_str());

        wardriveTrackBleTotal(addr);

        // Live BLE feed is allowed to refresh on each scan cycle.
        // Session uniqueness is tracked independently.

        if (!wardriveHasBle(addr) &&
            wardriveBleCount < WARD_MAX_BLE) {
            wardriveBleAddrs[wardriveBleCount++] = addr;
        }

        const std::string rawName = dev->getName();
        String name = rawName.empty() ? addr : String(rawName.c_str());
        WardriveRecord *rec = findWardriveRecord(false, addr);
        const bool existed = rec != nullptr;
        if (!rec) rec = newWardriveRecord(false, name, addr);
        if (rec) {
            rec->name = name;
            rec->rssi = dev->getRSSI();
            rec->trackerLike = isAppleFindMyTrackerLike(dev);
            rec->connectable = dev->isConnectable();
            rec->scannable = dev->isScannable();
            rec->serviceCount = dev->getServiceUUIDCount();
            rec->manufacturerCount = dev->getManufacturerDataCount();
            rec->haveTxPower = dev->haveTXPower();
            rec->txPower = rec->haveTxPower ? dev->getTXPower() : 0;
            if (existed && rec->seenCount < 65535) ++rec->seenCount;
            rec->lastSeenMs = millis();
            snapshotWardriveLocation(*rec);

            // Feed history is handled independently below.
            // RAM record details still update when this cache entry exists.
        }

        // Show devices that are actually present in the current BLE
        // sample. Repeated sightings on later scans are intentional:
        // this is a real-time rolling feed, not session history.
        const bool trackerLike =
            isAppleFindMyTrackerLike(dev);

        String line =
            trackerLike
                ? "TRACKER  "
                : "BLE  ";

        line +=
            name.length()
                ? name
                : addr;

        line += "  ";
        line += String(dev->getRSSI());
        line += "dBm";

        wardriveAddSweepLine(line);
    }

    appendWardriveRecordGroupToSd(false, sdSeenSinceMs);

    // All advertised-device information has been copied into
    // Wardrive-owned records. Release NimBLE result objects now.
    scan->clearResults();
    scan->setMaxResults(MAX_BLE_DETAILS);

    wardrivePhase = WardrivePhase::Idle;
    wardriveNextSweepMs = millis() + 1800;

    // Quiet window before display DMA resumes.
    wardriveDisplayResumeMs =
        millis() + 1500UL;

    wardriveUiSyncPending = true;

    Serial.println(
        "[M33K WARD] BLE results released; display cooldown"
    );

    if (wardriveStatusLabel) {
        lv_label_set_text(
            wardriveStatusLabel,
            "SESSION LIVE  |  GPS + RADIO CAPTURE"
        );
    }
}

void beginWardriveSweep()
{
    if (!wardrivePageActive || !wardriveRunning) return;
    if (wardriveMemoryPaused) return;
    // Never start companion activity from Wardrive unless
    // the C5 is already authenticated and linked.
    // DIAGNOSTIC:
    // Keep an already-linked C5 connected, but do not request
    // fresh 5 GHz results during Wardrive.
    //
    // This isolates the C5 scan/result path from the BLE link itself.
    if (c5IsLinked()) {
        // requestC5Scan();
    }

    esp_wifi_scan_stop();
    WiFi.scanDelete();
    const bool connected = WiFi.status() == WL_CONNECTED;
    if (!connected) {
        WiFi.disconnect(false, false);
        delay(90);
    }
    WiFi.mode(WIFI_STA);
    delay(180);
    int16_t result = WiFi.scanNetworks(true, true, false, 280);
    if (result == WIFI_SCAN_FAILED && !connected) {
        WiFi.disconnect(true, false);
        delay(100);
        WiFi.mode(WIFI_STA);
        delay(300);
        WiFi.disconnect(false, false);
        delay(80);
        result = WiFi.scanNetworks(true, true, false, 330);
    }
    if (result == WIFI_SCAN_FAILED) {
        wardrivePhase = WardrivePhase::Failed;
        if (wardriveStatusLabel) lv_label_set_text(wardriveStatusLabel, "WARDRIVE WI-FI SCAN FAILED");
        return;
    }
    wardrivePhase = WardrivePhase::WifiScanning;
    if (wardriveStatusLabel) lv_label_set_text(wardriveStatusLabel, "SESSION LIVE  |  SCANNING 2.4 + 5 GHz...");
}

void wardriveTimerCb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    if (currentPage != Page::Wardrive ||
        !wardrivePageActive ||
        wardriveExitToReconPending) {
        return;
    }
    // DIAGNOSTIC:
    // Keep the C5 BLE connection established, but do not continuously
    // service the C5 transport while Wardrive is active.
    serviceC5Link();

    const bool wardriveDisplayHold =
        c5DiscoveryInFlight ||
        c5ScanInFlight ||
        c5TransportConnected();

    if (!wardriveDisplayHold) {
        updateWardriveUi();
    }
    if (!wardriveRunning) return;

    const uint32_t wardNow = millis();

    // --------------------------------------------------------
    // C5 5 GHz transaction scheduler.
    // --------------------------------------------------------

    // A linked/authenticated C5 gets exactly one scan request.
    if (!wardriveC5CycleActive &&
        c5IsLinked() &&
        !c5ScanInFlight &&
        wardNow >= wardriveNextC5CycleMs) {

        wardriveC5RequestGeneration =
            c5ScanGeneration;

        if (requestC5Scan()) {
            wardriveC5CycleActive = true;
            wardriveC5CycleStartedMs = wardNow;

            Serial.printf(
                "[M33K WARD] C5 scan requested generation=%lu\n",
                (unsigned long)wardriveC5RequestGeneration
            );
        }
    }

    // If C5 is physically on but disconnected, rediscover it
    // only when the next C5 window arrives.
    // C5 is optional, but Wardrive discovers it itself.
    // Only start discovery during a quiet Idle phase.
    if (wardrivePhase == WardrivePhase::Idle &&
        !wardriveC5CycleActive &&
        !c5TransportConnected() &&
        !c5DiscoveryInFlight &&
        wardNow >= wardriveNextC5CycleMs) {

        // startC5DiscoveryAsync() normally honors its global retry
        // timer. This is an explicit Wardrive transaction window.
        c5NextConnectAttemptMs = 0;

        if (startC5DiscoveryAsync()) {
            wardriveC5CycleStartedMs = wardNow;

            Serial.println(
                "[M33K WARD] scheduled C5 discovery started"
            );
        } else {
            wardriveNextC5CycleMs =
                wardNow + WARD_C5_INTERVAL_MS;
        }
    }

    // Never let an unsuccessful transaction hold the C5 client.
    if (wardriveC5CycleActive &&
        wardNow - wardriveC5CycleStartedMs >
            WARD_C5_TIMEOUT_MS) {

        Serial.println(
            "[M33K WARD] C5 transaction timed out; disconnecting"
        );

        wardriveC5CycleActive = false;
        wardriveNextC5CycleMs =
            wardNow + WARD_C5_INTERVAL_MS;

        if (c5Client && c5Client->isConnected()) {
            c5Client->disconnect();
        }
    }

    // --------------------------------------------------------
    // Watch-side BLE scanner.
    //
    // Never overlap BLE target scanning with C5 discovery,
    // authentication, scan transfer, or a live C5 connection.
    // --------------------------------------------------------

    const bool c5RadioBusy =
        c5DiscoveryInFlight ||
        c5ScanInFlight ||
        c5TransportConnected();

    if (wardrivePhase == WardrivePhase::Idle &&
        !c5RadioBusy &&
        wardNow >= wardriveNextBleSampleMs) {

        wardriveNextBleSampleMs =
            wardNow + WARD_BLE_INTERVAL_MS;

        if (startWardriveBleScan()) {
            Serial.println(
                "[M33K WARD] watch BLE sample started"
            );
            return;
        }

        Serial.println(
            "[M33K WARD] watch BLE sample skipped"
        );
    }


    const uint32_t freeHeap = ESP.getFreeHeap();
    const uint32_t largestBlock =
        heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

    if (millis() - wardriveLastDiagMs >= 5000) {
        wardriveLastDiagMs = millis();

        const uint32_t dmaFree =
            heap_caps_get_free_size(
                MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL
            );

        const uint32_t dmaLargest =
            heap_caps_get_largest_free_block(
                MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL
            );

        const uint32_t dmaMin =
            heap_caps_get_minimum_free_size(
                MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL
            );

        Serial.printf(
            "[M33K DMA] free=%u largest=%u min=%u\n",
            (unsigned)dmaFree,
            (unsigned)dmaLargest,
            (unsigned)dmaMin
        );

        Serial.printf(
            "[M33K WARD] heap=%u minHeap=%u largest=%u stack=%u records=%u wifi=%lu C5=%s gpsAge=%lu gpsBytes=%lu\n",
            (unsigned)freeHeap,
            (unsigned)ESP.getMinFreeHeap(),
            (unsigned)largestBlock,
            (unsigned)uxTaskGetStackHighWaterMark(nullptr),
            (unsigned)wardriveRecordCount,
            (unsigned long)wardriveTotalWifiCount,
            c5IsLinked() ? "LINKED" : "OFFLINE",
            (unsigned long)(millis() - gpsLastByteMs),
            (unsigned long)gpsSessionBytes
        );
    }

    if (wardrivePhase == WardrivePhase::Idle) {
        if (!wardriveMemoryPaused &&
            (freeHeap < WARD_HEAP_PAUSE_BYTES ||
             largestBlock < WARD_LARGEST_BLOCK_PAUSE_BYTES)) {
            wardriveMemoryPaused = true;
            if (wardriveStatusLabel) {
                lv_label_set_text(
                    wardriveStatusLabel,
                    "MEMORY GUARD  |  SCANS PAUSED"
                );
            }
            Serial.println("[M33K WARD] memory guard paused new sweeps");
        } else if (wardriveMemoryPaused &&
                   freeHeap > WARD_HEAP_RESUME_BYTES &&
                   largestBlock > WARD_LARGEST_BLOCK_RESUME_BYTES) {
            wardriveMemoryPaused = false;
            wardriveNextSweepMs = millis() + 750;
            if (wardriveStatusLabel) {
                lv_label_set_text(
                    wardriveStatusLabel,
                    "SESSION LIVE  |  MEMORY RECOVERED"
                );
            }
            Serial.println("[M33K WARD] memory guard resumed sweeps");
        }

        if (wardriveMemoryPaused) return;
    }

    // C5 discovery is handled by the time-share scheduler.
    if (wardrivePhase == WardrivePhase::WifiScanning) {
        int16_t result = WiFi.scanComplete();
        if (result == WIFI_SCAN_RUNNING) return;
        finishWardriveWifiScan();
        return;
    }
    if (wardrivePhase == WardrivePhase::Idle && millis() >= wardriveNextSweepMs) beginWardriveSweep();
}

void startWardriveSession()
{
    wardriveWifiCount = 0;
    wardriveTotalWifiCount = 0;
    memset(wardriveTotalWifiBloom, 0, sizeof(wardriveTotalWifiBloom));
    wardriveMemoryPaused = false;
    wardrivePageActive = true;
    wardriveBleCount = 0;

    wardriveTotalBleCount = 0;

    memset(
        wardriveTotalBleBloom,
        0,
        sizeof(wardriveTotalBleBloom)
    );

    wardriveDetailOpen = false;
    wardriveUiSyncCursor = 0;
    wardriveLastDiagMs = 0;
    wardriveDisplayResumeMs = 0;
    wardriveC5CacheApplied = false;
    wardriveAppliedC5Generation = c5ScanGeneration;

    // --------------------------------------------------------
    // New-session Wardrive radio reset.
    //
    // Clear completed/stale scan state left behind by the
    // previous Wardrive session before starting another.
    // --------------------------------------------------------
    esp_wifi_scan_stop();
    WiFi.scanDelete();

    if (bleScannerInitialized &&
        !c5DiscoveryInFlight &&
        !c5ScanInFlight) {

        NimBLEScan *scan =
            NimBLEDevice::getScan();

        if (scan->isScanning()) {
            scan->stop();
        }

        scan->clearResults();
        scan->setMaxResults(MAX_BLE_DETAILS);
    }

    bleScanFinished = false;
    bleScanEndReason = 0;

    // Every Wardrive START explicitly re-arms C5 discovery.
    c5NextConnectAttemptMs = 0;
    resetWardriveRecords();
    wardriveResetSweepFeed();
    wardriveFeedDirty = true;
    wardriveSessionStartedMs = millis();
    wardriveNextSweepMs = 0;

    wardriveC5CycleActive = false;
    wardriveC5CycleStartedMs = 0;
    wardriveC5RequestGeneration = c5ScanGeneration;

    // Re-arm optional C5 discovery for every new Wardrive session.
    // A previous Wardrive exit can leave a transport/client object
    // behind even though authentication/link state is no longer valid.
    c5NextConnectAttemptMs = 0;

    // Preserve a genuinely authenticated C5 link.
    // Only remove a stale unauthenticated client when no C5 BLE
    // discovery/scan transaction is currently active.
    if (!c5IsLinked() &&
        !c5DiscoveryInFlight &&
        !c5ScanInFlight &&
        c5Client) {

        if (c5Client->isConnected()) {
            c5Client->disconnect();
        }

        NimBLEDevice::deleteClient(c5Client);
        c5Client = nullptr;
        c5Authenticated = false;

        Serial.println(
            "[M33K WARD] stale C5 client cleared for new session"
        );
    }

    // Watch-side BLE is a core Wardrive function.
    // Give it the first NimBLE radio window even when the
    // optional C5 is powered off.
    wardriveNextBleSampleMs = millis() + 1000UL;

    // Wardrive discovers C5 itself; Wi-Fi Scanner is not required.
    wardriveNextC5CycleMs = millis() + 1500UL;

    wardriveLastElapsedSeconds = 0;

    const bool wardriveSdStarted =
        startWardriveSdLog();

    if (wardriveSdLabel) {
        if (wardriveSdStarted) {
            lv_label_set_text(
                wardriveSdLabel,
                "SD LOGGING\n0 ROWS"
            );

            lv_obj_set_style_text_color(
                wardriveSdLabel,
                lv_color_hex(0x7CFF45),
                0
            );
        } else {
            lv_label_set_text(
                wardriveSdLabel,
                "SD OFFLINE"
            );

            lv_obj_set_style_text_color(
                wardriveSdLabel,
                lv_color_hex(0xF5FF3B),
                0
            );
        }
    }

    Serial.printf(
        "[M33K WARD] SD startup=%s\n",
        wardriveSdStarted
            ? "LOGGING"
            : "OFFLINE"
    );

    wardriveRunning = true;
    wardrivePhase = WardrivePhase::Idle;

    // Wardrive owns C5 discovery.
    //
    // Try once immediately before the continuous local Wi-Fi/BLE
    // pipeline starts. This removes the requirement to visit the
    // Wi-Fi Scanner page first.
    if (!c5IsLinked() &&
        !c5TransportConnected() &&
        !c5DiscoveryInFlight) {

        c5NextConnectAttemptMs = 0;

        if (startC5DiscoveryAsync()) {
            wardriveC5CycleStartedMs = millis();

            Serial.println(
                "[M33K WARD] initial C5 discovery started"
            );

            if (wardriveStatusLabel) {
                lv_label_set_text(
                    wardriveStatusLabel,
                    "SESSION STARTING  |  LOOKING FOR C5..."
                );
            }
        } else {
            // Do not block local Wardrive if C5 cannot start.
            wardriveNextC5CycleMs =
                millis() + 15000UL;

            Serial.println(
                "[M33K WARD] initial C5 discovery unavailable; local mode"
            );
        }
    }

    // Wardrive shares the same continuously-running GNSS stream as GPS.
    // Do not tear down/reopen UART1 when entering Wardrive.
    ensureGpsReady();
    gpsToolActive = true;
    gpsRecoveryAttempted = false;
    gpsSessionBytes = 0;
    gpsSessionLocationSeen = false;
    gpsPageOpenedMs = millis();
    gpsLastByteMs = gpsPageOpenedMs;
    if (wardriveStartLabel) lv_label_set_text(wardriveStartLabel, "STOP");
    if (wardriveStatusLabel) lv_label_set_text(wardriveStatusLabel, "SESSION STARTED");
    updateWardriveUi();

    // Do not launch the first local Wi-Fi sweep on top of the
    // initial C5 BLE discovery. Once discovery finishes, the
    // Wardrive timer starts the normal continuous pipeline.
    if (!c5DiscoveryInFlight &&
        !c5TransportConnected()) {

        beginWardriveSweep();
    } else {
        wardriveNextSweepMs =
            millis() + 250UL;
    }
}

void stopWardriveSession()
{
    if (wardriveRunning) {
        wardriveLastElapsedSeconds =
            (millis() - wardriveSessionStartedMs) / 1000;
    }

    // STOP only ends the Wardrive session. Do not touch the Wi-Fi or NimBLE
    // scanner here (or from a cleanup poll) because those stacks can still be
    // completing an asynchronous scan on another task. The in-flight scan is
    // allowed to finish naturally; generic scan-completion handling will
    // release BLE results, and the next Wardrive start/navigation path safely
    // resets Wi-Fi scan state before reuse.
    wardriveRunning = false;
    wardrivePhase = WardrivePhase::Stopped;

    // A C5 transaction may still be active when STOP is pressed.
    // Do not carry that client/transaction into the next session.
    wardriveStopCleanupPending = true;

    wardriveC5CycleActive = false;
    wardriveC5CycleStartedMs = 0;
    c5NextConnectAttemptMs = 0;

    // Ask the active C5 transport to terminate now.
    // Actual client deletion is deferred until callbacks/scans
    // are no longer using it.
    if (c5Client && c5Client->isConnected()) {
        c5Client->disconnect();
    }

    stopWardriveSdLog();

    Serial.printf(
        "[M33K WARD] STOP: heap=%u, minHeap=%u, stack=%u\n",
        (unsigned)ESP.getFreeHeap(),
        (unsigned)ESP.getMinFreeHeap(),
        (unsigned)uxTaskGetStackHighWaterMark(nullptr)
    );

    if (wardriveStartLabel) lv_label_set_text(wardriveStartLabel, "START");

    if (wardriveStatusLabel) {
        lv_label_set_text_fmt(
            wardriveStatusLabel,
            "STOPPED  %02lu:%02lu  |  WI-FI %lu  |  BLE %lu",
            (unsigned long)(wardriveLastElapsedSeconds / 60UL),
            (unsigned long)(wardriveLastElapsedSeconds % 60UL),
            (unsigned long)wardriveTotalWifiCount,
            (unsigned long)wardriveTotalBleCount
        );
    }
}

void wardriveStartEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED || !clickAllowed()) return;

    const uint32_t now = millis();
    if (now - wardriveLastStartStopMs < 700) return;
    wardriveLastStartStopMs = now;

    noteActivity();
    hapticTap();
    if (wardriveRunning) {
        stopWardriveSession();
    } else if (c5DiscoveryInFlight) {
        if (wardriveStatusLabel) {
            lv_label_set_text(
                wardriveStatusLabel,
                "CONNECTING C5  |  TAP START AGAIN"
            );
        }
    } else {
        startWardriveSession();
    }
}

void reconWardriveEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED || !clickAllowed()) return;

    noteActivity();
    hapticTap();

    // Do not destroy/rebuild the Recon LVGL tree from inside its own CLICKED
    // callback. Defer Wardrive creation until the callback has returned.
    wardriveOpenFromReconPending = true;
}

void IRAM_ATTR m33kRadioPacketIsr()
{
    m33kRadioPacketFlag = true;
}

uint8_t currentRadioSf()
{
    return M33K_RADIO_SF_VALUES[m33kRadioSfIndex];
}

float currentRadioBw()
{
    return M33K_RADIO_BW_VALUES[m33kRadioBwIndex];
}

void updateRadioProfileLabels()
{
    if (radioProfileLabel) {
        if (m33kRadioMeshCore)
            lv_label_set_text(radioProfileLabel, "SF 7\nBW 62.5\nCR 4/5");
        else
            lv_label_set_text_fmt(
                radioProfileLabel,
                "SF %u\nBW %.0f\nCR 4/7",
                (unsigned)currentRadioSf(),
                currentRadioBw()
            );
    }

    if (radioFreqLabel) {
        lv_label_set_text_fmt(
            radioFreqLabel,
            "FREQ\n%.3f",
            m33kRadioMeshCore ? M33K_MESHCORE_FREQ_MHZ : M33K_RADIO_FREQ_MHZ
        );
    }

    if (radioSfButtonLabel) {
        lv_label_set_text_fmt(
            radioSfButtonLabel,
            "SF %u",
            (unsigned)(m33kRadioMeshCore ? M33K_MESHCORE_SF : currentRadioSf())
        );
    }

    if (radioBwButtonLabel) {
        lv_label_set_text_fmt(
            radioBwButtonLabel,
            "BW %.0f",
            (double)(m33kRadioMeshCore ? M33K_MESHCORE_BW_KHZ : currentRadioBw())
        );
    }

    if (radioAutoButtonLabel) {
        lv_label_set_text(
            radioAutoButtonLabel,
            m33kRadioAutoScan ? "AUTO ON" : "AUTO OFF"
        );
    }

    if (radioMeshButtonLabel) {
        lv_label_set_text(
            radioMeshButtonLabel,
            m33kRadioMeshCore ? "MESH ON" : "MESH OFF"
        );
    }

    if (radioModeLabel) {
        lv_label_set_text(
            radioModeLabel,
            m33kRadioMeshCore ? "MODE\nMESHCORE"
                              : (m33kRadioAutoScan ? "MODE\nAUTO RX" : "MODE\nRX ONLY")
        );
    }
}

void resetRadioHistory()
{
    m33kRadioHistoryCount = 0;
    for (auto &row : m33kRadioHistory) row = "";
    m33kRadioHistoryDirty = true;
}

void pushRadioHistory(const String &entry)
{
    if (entry.length() == 0) return;

    if (m33kRadioHistoryCount < M33K_RADIO_HISTORY_MAX) {
        m33kRadioHistory[m33kRadioHistoryCount++] = entry;
    } else {
        for (uint8_t i = 1; i < M33K_RADIO_HISTORY_MAX; ++i) {
            m33kRadioHistory[i - 1] = m33kRadioHistory[i];
        }
        m33kRadioHistory[M33K_RADIO_HISTORY_MAX - 1] = entry;
    }

    m33kRadioHistoryDirty = true;
}

void updateRadioHistoryUi()
{
    if (!radioPayloadLabel || !m33kRadioHistoryDirty) return;

    if (m33kRadioHistoryCount == 0) {
        if (m33kRadioMeshCore) {
            lv_label_set_text(
                radioPayloadLabel,
                "Listening on the MeshCore public channel.\n\n"
                "Decoded public messages appear here when a MeshCore "
                "node is in range. Receive-only."
            );
        } else {
            lv_label_set_text(
                radioPayloadLabel,
                "No matching packets yet.\n\n"
                "RX-only. AUTO cycles SF7-SF12 and "
                "125/250/500 kHz receive profiles."
            );
        }
        m33kRadioHistoryDirty = false;
        return;
    }

    String combined;

    for (uint8_t i = 0; i < m33kRadioHistoryCount; ++i) {
        combined += m33kRadioHistory[i];

        if (i + 1 < m33kRadioHistoryCount) {
            combined += "\n----------------\n";
        }
    }

    lv_label_set_text(radioPayloadLabel, combined.c_str());
    m33kRadioHistoryDirty = false;
}

void updateRadioMonitorUi()
{
    if (currentPage != Page::Radio) return;

    if (radioPacketLabel) {
        lv_label_set_text_fmt(
            radioPacketLabel,
            "PACKETS\n%lu",
            (unsigned long)m33kRadioPacketCount
        );
    }

    if (radioSignalLabel) {
        if (m33kRadioPacketCount > 0) {
            lv_label_set_text_fmt(
                radioSignalLabel,
                "LAST SIGNAL\n%.1f dBm\nSNR %.1f dB",
                m33kRadioLastRssi,
                m33kRadioLastSnr
            );
        } else {
            lv_label_set_text(
                radioSignalLabel,
                "LAST SIGNAL\n-- dBm\nSNR --"
            );
        }
    }

    if (radioLastLabel) {
        if (m33kRadioPacketCount > 0) {
            const uint32_t ageSec =
                (millis() - m33kRadioLastPacketMs) / 1000U;

            lv_label_set_text_fmt(
                radioLastLabel,
                "LEN %u B   FERR %.0f Hz   %lus AGO",
                (unsigned)m33kRadioLastLength,
                m33kRadioLastFreqError,
                (unsigned long)ageSec
            );
        } else {
            lv_label_set_text(
                radioLastLabel,
                m33kRadioAutoScan
                    ? "AUTO scanning receive profiles..."
                    : "Waiting for a matching LoRa packet..."
            );
        }
    }

    if (radioListenLabel) {
        lv_label_set_text(
            radioListenLabel,
            m33kRadioListening ? "STOP" : "LISTEN"
        );
    }

    updateRadioProfileLabels();
    updateRadioHistoryUi();
}

bool applyRadioProfile(bool restartReceive)
{
    if (!m33kRadioInitialized) return false;

    int16_t state = -999;

    if (instance.lockSPI(pdMS_TO_TICKS(600))) {
        radio.clearDio1Action();
        radio.standby();

        // MeshCore preset forces the mesh radio parameters; otherwise the
        // generic sniffer uses the user-selected SF/BW at 915 MHz, CR 4/7.
        const float   cfgFreq = m33kRadioMeshCore ? M33K_MESHCORE_FREQ_MHZ : M33K_RADIO_FREQ_MHZ;
        const float   cfgBw   = m33kRadioMeshCore ? M33K_MESHCORE_BW_KHZ   : currentRadioBw();
        const uint8_t cfgSf   = m33kRadioMeshCore ? M33K_MESHCORE_SF       : currentRadioSf();
        const uint8_t cfgCr   = m33kRadioMeshCore ? M33K_MESHCORE_CR       : 7;

        state = radio.setFrequency(cfgFreq);

        if (state == RADIOLIB_ERR_NONE) {
            state = radio.setBandwidth(cfgBw);
        }

        if (state == RADIOLIB_ERR_NONE) {
            state = radio.setSpreadingFactor(cfgSf);
        }

        if (state == RADIOLIB_ERR_NONE) {
            state = radio.setCodingRate(cfgCr);
        }

        if (state == RADIOLIB_ERR_NONE) {
            // MeshCore uses RadioLib's private sync word (0x12).
            state = radio.setSyncWord(M33K_MESHCORE_SYNC);
        }

        if (state == RADIOLIB_ERR_NONE && restartReceive) {
            radio.setDio1Action(m33kRadioPacketIsr);
            m33kRadioPacketFlag = false;
            state = radio.startReceive();
        }

        instance.unlockSPI();
    }

    if (state != RADIOLIB_ERR_NONE) {
        if (radioStatusLabel) {
            lv_label_set_text_fmt(
                radioStatusLabel,
                "PROFILE APPLY FAILED  |  CODE %d",
                state
            );
            lv_obj_set_style_text_color(
                radioStatusLabel,
                lv_color_hex(0xFF5C7A),
                0
            );
        }
        return false;
    }

    if (restartReceive) {
        m33kRadioListening = true;
        m33kRadioNextProfileMs =
            millis() + M33K_RADIO_AUTO_DWELL_MS;
    }

    updateRadioMonitorUi();
    return true;
}

bool startRadioMonitor()
{
    instance.powerControl(POWER_RADIO, true);
    delay(25);

    if (!m33kRadioInitialized) {
        if (radioStatusLabel) {
            lv_label_set_text(
                radioStatusLabel,
                "PROBING SX1262..."
            );
        }

        bool initOk = false;

        if (instance.lockSPI(pdMS_TO_TICKS(600))) {
            initOk = instance.initLoRa();
            instance.unlockSPI();
        }

        if (!initOk) {
            m33kRadioInitialized = false;
            m33kRadioListening = false;

            if (radioStatusLabel) {
                lv_label_set_text(
                    radioStatusLabel,
                    "SX1262 NOT DETECTED / INIT FAILED"
                );
                lv_obj_set_style_text_color(
                    radioStatusLabel,
                    lv_color_hex(0xFF5C7A),
                    0
                );
            }

            return false;
        }

        m33kRadioInitialized = true;
    }

    if (!applyRadioProfile(true)) {
        m33kRadioListening = false;
        return false;
    }

    if (radioStatusLabel) {
        lv_label_set_text_fmt(
            radioStatusLabel,
            "SX1262 ONLINE | SF%u BW%.0f",
            (unsigned)currentRadioSf(),
            currentRadioBw()
        );
        lv_obj_set_style_text_color(
            radioStatusLabel,
            lv_color_hex(0x7CFF45),
            0
        );
    }

    updateRadioMonitorUi();
    return true;
}

void stopRadioMonitorReceive()
{
    if (!m33kRadioInitialized) {
        m33kRadioListening = false;
        return;
    }

    if (instance.lockSPI(pdMS_TO_TICKS(300))) {
        radio.clearDio1Action();
        radio.standby();
        instance.unlockSPI();
    }

    m33kRadioPacketFlag = false;
    m33kRadioListening = false;

    if (radioStatusLabel) {
        lv_label_set_text(
            radioStatusLabel,
            "SX1262 ONLINE  |  RX STOPPED"
        );
        lv_obj_set_style_text_color(
            radioStatusLabel,
            lv_color_hex(0xF5FF3B),
            0
        );
    }

    updateRadioMonitorUi();
}

void stopRadioTool()
{
    if (radioTimer) {
        lv_timer_del(radioTimer);
        radioTimer = nullptr;
    }

    if (m33kRadioInitialized) {
        if (instance.lockSPI(pdMS_TO_TICKS(300))) {
            radio.clearDio1Action();
            radio.standby();
            instance.unlockSPI();
        }
    }

    m33kRadioPacketFlag = false;
    m33kRadioListening = false;
    m33kRadioAutoScan = false;
    m33kRadioInitialized = false;
    instance.powerControl(POWER_RADIO, false);
}

void advanceRadioAutoProfile()
{
    ++m33kRadioSfIndex;

    if (m33kRadioSfIndex >= M33K_RADIO_SF_COUNT) {
        m33kRadioSfIndex = 0;
        ++m33kRadioBwIndex;

        if (m33kRadioBwIndex >= M33K_RADIO_BW_COUNT) {
            m33kRadioBwIndex = 0;
        }
    }

    if (applyRadioProfile(m33kRadioListening)) {
        if (radioStatusLabel) {
            lv_label_set_text_fmt(
                radioStatusLabel,
                "AUTO RX | SF%u BW%.0f",
                (unsigned)currentRadioSf(),
                currentRadioBw()
            );
            lv_obj_set_style_text_color(
                radioStatusLabel,
                lv_color_hex(0x55EEFF),
                0
            );
        }
    }
}

// ---------------------------------------------------------------------------
// MeshCore public-channel decode (Tier 1: receive-only).
//
// MeshCore's "Public" channel is a group channel with a well-known shared key.
// A group-text packet's payload is:  channel_hash(1) | MAC(2) | AES-128-ECB
// ciphertext, where the plaintext is  timestamp(4, LE) | "name: message".
// Crypto here uses the ESP32 SDK's mbedtls (AES-128-ECB, HMAC-SHA256), so no
// extra library is needed. This only READS the public channel - it does not
// transmit, decrypt direct messages, or route/relay.
// ---------------------------------------------------------------------------
// The MeshCore public group key (base64 "izOH6cXN6mrJ5e26oRXNcg=="), 16 bytes.
// This is MeshCore's universal public-channel key, not any personal data.
static const uint8_t M33K_MESH_PUB_KEY[16] = {
    0x8B, 0x33, 0x87, 0xE9, 0xC5, 0xCD, 0xEA, 0x6A,
    0xC9, 0xE5, 0xED, 0xBA, 0xA1, 0x15, 0xCD, 0x72
};
uint8_t  m33kMeshPubChanHash = 0;
bool     m33kMeshPubReady = false;

void meshcoreInitPublic()
{
    if (m33kMeshPubReady) return;
    uint8_t h[32];
    // channel hash = SHA256(key)[0]  (key upper 16 bytes are zero -> hash 16).
    mbedtls_sha256(M33K_MESH_PUB_KEY, sizeof(M33K_MESH_PUB_KEY), h, 0);
    m33kMeshPubChanHash = h[0];
    m33kMeshPubReady = true;
}

// Short human label for a MeshCore payload type (the 4 bits above the route).
static const char *meshcorePayloadName(uint8_t payloadType)
{
    switch (payloadType) {
        case 0x00: return "REQ";
        case 0x01: return "RESP";
        case 0x02: return "DIRECT";    // direct text, encrypted to one node
        case 0x03: return "ACK";
        case 0x04: return "ADVERT";    // node announcing itself
        case 0x05: return "GRP_TXT";   // public/group text message
        case 0x06: return "GRP_DATA";
        case 0x07: return "ANON_REQ";
        case 0x08: return "PATH";
        case 0x09: return "TRACE";
        case 0x0A: return "MPART";
        case 0x0B: return "CONTROL";
        case 0x0F: return "RAW";
        default:   return "?";
    }
}

// Try to decode a received frame as a MeshCore public-channel text message.
// Returns true and fills `out` ("name: message") on success.
bool meshcoreDecodePublic(const uint8_t *frame, size_t len, String &out)
{
    meshcoreInitPublic();
    if (len < 3) return false;

    size_t i = 0;
    const uint8_t header = frame[i++];
    const uint8_t routeType   = header & 0x03;
    const uint8_t payloadType = (header >> 2) & 0x0F;
    if (payloadType != 0x05) return false;            // PAYLOAD_TYPE_GRP_TXT

    if (routeType == 0x00 || routeType == 0x03) i += 4;  // transport codes
    if (i >= len) return false;

    const uint8_t pathLenByte = frame[i++];
    const uint8_t hashSize = ((pathLenByte >> 6) & 0x03) + 1;
    const uint8_t hashCnt  = pathLenByte & 0x3F;
    i += (size_t)hashSize * hashCnt;                  // skip routing path
    if (i >= len) return false;

    // Payload: channel_hash(1) | MAC(2) | ciphertext (AES blocks).
    if (frame[i] != m33kMeshPubChanHash) return false;  // not the public channel
    i++;
    const uint8_t *macAndCipher = &frame[i];
    const int bodyLen = (int)(len - i);
    if (bodyLen <= 2) return false;
    const int cipherLen = bodyLen - 2;                // after 2-byte MAC
    if (cipherLen <= 0 || (cipherLen % 16) != 0) return false;

    // Verify the 2-byte HMAC-SHA256 MAC over the ciphertext (key = 32 bytes:
    // the 16-byte PSK followed by 16 zero bytes, matching MeshCore).
    uint8_t keyFull[32] = {};
    memcpy(keyFull, M33K_MESH_PUB_KEY, 16);
    uint8_t mac[32];
    const mbedtls_md_info_t *md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (!md) return false;
    if (mbedtls_md_hmac(md, keyFull, 32, macAndCipher + 2, cipherLen, mac) != 0)
        return false;
    if (mac[0] != macAndCipher[0] || mac[1] != macAndCipher[1]) return false;

    // AES-128-ECB decrypt the ciphertext.
    uint8_t plain[192];
    if (cipherLen > (int)sizeof(plain)) return false;
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_dec(&aes, M33K_MESH_PUB_KEY, 128);
    for (int off = 0; off < cipherLen; off += 16)
        mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT,
                              macAndCipher + 2 + off, plain + off);
    mbedtls_aes_free(&aes);

    // Plaintext = timestamp(4, LE) + "name: message". Skip the timestamp and
    // take the readable text up to the first NUL / padding.
    if (cipherLen <= 4) return false;
    const char *txt = (const char *)(plain + 4);
    const int txtMax = cipherLen - 4;
    int n = 0;
    while (n < txtMax && txt[n] != '\0') ++n;
    // Trim any non-printable trailing bytes (ECB zero/era padding).
    while (n > 0 && (uint8_t)txt[n - 1] < 0x20) --n;
    if (n <= 0) return false;

    out = "";
    for (int k = 0; k < n; ++k) {
        const uint8_t c = (uint8_t)txt[k];
        out += (c >= 0x20 && c <= 0x7E) ? (char)c : '.';
    }
    return out.length() > 0;
}

void radioTimerCb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    if (currentPage != Page::Radio) return;

    if (m33kRadioAutoScan &&
        m33kRadioListening &&
        millis() >= m33kRadioNextProfileMs) {

        advanceRadioAutoProfile();
        m33kRadioNextProfileMs =
            millis() + M33K_RADIO_AUTO_DWELL_MS;
    }

    if (!m33kRadioListening) {
        updateRadioMonitorUi();
        return;
    }

    if (m33kRadioPacketFlag) {
        m33kRadioPacketFlag = false;

        int16_t state = -999;
        uint8_t data[256] = {};   // large enough for a full MeshCore frame
        size_t packetLen = 0;
        size_t readLen = 0;
        float rssi = -127.0f;
        float snr = 0.0f;
        float freqError = 0.0f;

        if (instance.lockSPI(pdMS_TO_TICKS(500))) {
            packetLen = radio.getPacketLength();
            readLen = packetLen;

            if (readLen > sizeof(data)) {
                readLen = sizeof(data);
            }

            state = radio.readData(data, readLen);
            rssi = radio.getRSSI();
            snr = radio.getSNR();
            freqError = radio.getFrequencyError();

            radio.setDio1Action(m33kRadioPacketIsr);
            radio.startReceive();

            instance.unlockSPI();
        }

        if (state == RADIOLIB_ERR_NONE) {
            ++m33kRadioPacketCount;

            m33kRadioLastPacketMs = millis();
            m33kRadioLastRssi = rssi;
            m33kRadioLastSnr = snr;
            m33kRadioLastFreqError = freqError;
            m33kRadioLastLength = packetLen;

            // MeshCore public-channel decode takes over the entry when the
            // preset is on and the frame decodes as a public group message.
            String meshMsg;
            if (m33kRadioMeshCore && meshcoreDecodePublic(data, readLen, meshMsg)) {
                String e = "MESH #";
                e += String(m33kRadioPacketCount);
                e += "  ";
                e += String(rssi, 0);
                e += " dBm\n";
                e += meshMsg;
                pushRadioHistory(e);
                if (radioStatusLabel) {
                    lv_label_set_text(radioStatusLabel,
                        "MESHCORE | PUBLIC MSG DECODED");
                    lv_obj_set_style_text_color(radioStatusLabel,
                        lv_color_hex(0x7CFF45), 0);
                }
            } else {

            String entry;
            entry += "#";
            entry += String(m33kRadioPacketCount);
            if (m33kRadioMeshCore && readLen >= 1) {
                // Label the packet by its MeshCore type (a GRP_TXT shown here
                // is a public message that failed to decrypt/parse).
                entry += "  ";
                entry += meshcorePayloadName((data[0] >> 2) & 0x0F);
            }
            entry += "  SF";
            entry += String((int)(m33kRadioMeshCore ? M33K_MESHCORE_SF : currentRadioSf()));
            entry += " BW";
            entry += String((int)(m33kRadioMeshCore ? M33K_MESHCORE_BW_KHZ : currentRadioBw()));
            entry += "\n";
            entry += String(rssi, 1);
            entry += " dBm  SNR ";
            entry += String(snr, 1);
            entry += " dB  LEN ";
            entry += String((unsigned)packetLen);
            entry += " B\nHEX ";

            const size_t shown =
                readLen < 18 ? readLen : 18;

            char byteBuf[4];

            for (size_t i = 0; i < shown; ++i) {
                snprintf(
                    byteBuf,
                    sizeof(byteBuf),
                    "%02X",
                    data[i]
                );
                entry += byteBuf;

                if (i + 1 < shown) {
                    entry += ' ';
                }
            }

            if (readLen > shown || packetLen > readLen) {
                entry += " ...";
            }

            entry += "\nASCII ";

            for (size_t i = 0; i < shown; ++i) {
                const uint8_t c = data[i];
                entry +=
                    (c >= 32 && c <= 126)
                        ? char(c)
                        : '.';
            }

            pushRadioHistory(entry);

            if (radioStatusLabel) {
                lv_label_set_text(
                    radioStatusLabel,
                    "SX1262 ONLINE  |  PACKET RECEIVED"
                );
                lv_obj_set_style_text_color(
                    radioStatusLabel,
                    lv_color_hex(0x7CFF45),
                    0
                );
            }
            }  // end else (non-MeshCore or undecoded frame)
        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            if (radioStatusLabel) {
                lv_label_set_text(
                    radioStatusLabel,
                    "PACKET RECEIVED  |  CRC MISMATCH"
                );
                lv_obj_set_style_text_color(
                    radioStatusLabel,
                    lv_color_hex(0xFFB84D),
                    0
                );
            }
        } else {
            if (radioStatusLabel) {
                lv_label_set_text_fmt(
                    radioStatusLabel,
                    "RX READ ERROR  |  CODE %d",
                    state
                );
                lv_obj_set_style_text_color(
                    radioStatusLabel,
                    lv_color_hex(0xFF5C7A),
                    0
                );
            }
        }

        updateRadioMonitorUi();
    } else if (m33kRadioPacketCount > 0) {
        updateRadioMonitorUi();
    }
}

void radioToggleEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED ||
        !clickAllowed()) return;

    noteActivity();
    hapticTap();

    if (m33kRadioListening) {
        stopRadioMonitorReceive();
    } else {
        startRadioMonitor();
    }
}

void radioReprobeEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED ||
        !clickAllowed()) return;

    noteActivity();
    hapticTap();

    stopRadioMonitorReceive();
    m33kRadioInitialized = false;
    startRadioMonitor();
}

void radioSfEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED ||
        !clickAllowed()) return;

    noteActivity();
    hapticTap();

    if (m33kRadioMeshCore) {
        if (radioStatusLabel) lv_label_set_text(radioStatusLabel,
            "MeshCore preset active - turn MESH off to tune SF/BW");
        return;
    }

    m33kRadioAutoScan = false;
    m33kRadioSfIndex =
        (m33kRadioSfIndex + 1) % M33K_RADIO_SF_COUNT;

    if (m33kRadioInitialized) {
        applyRadioProfile(m33kRadioListening);
    }

    if (radioStatusLabel) {
        lv_label_set_text_fmt(
            radioStatusLabel,
            "MANUAL RX | SF%u BW%.0f",
            (unsigned)currentRadioSf(),
            currentRadioBw()
        );
    }

    updateRadioMonitorUi();
}

void radioBwEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED ||
        !clickAllowed()) return;

    noteActivity();
    hapticTap();

    if (m33kRadioMeshCore) {
        if (radioStatusLabel) lv_label_set_text(radioStatusLabel,
            "MeshCore preset active - turn MESH off to tune SF/BW");
        return;
    }

    m33kRadioAutoScan = false;
    m33kRadioBwIndex =
        (m33kRadioBwIndex + 1) % M33K_RADIO_BW_COUNT;

    if (m33kRadioInitialized) {
        applyRadioProfile(m33kRadioListening);
    }

    if (radioStatusLabel) {
        lv_label_set_text_fmt(
            radioStatusLabel,
            "MANUAL RX | SF%u BW%.0f",
            (unsigned)currentRadioSf(),
            currentRadioBw()
        );
    }

    updateRadioMonitorUi();
}

void radioAutoEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED ||
        !clickAllowed()) return;

    noteActivity();
    hapticTap();

    if (m33kRadioMeshCore) {
        if (radioStatusLabel) lv_label_set_text(radioStatusLabel,
            "MeshCore preset active - turn MESH off for AUTO");
        return;
    }

    m33kRadioAutoScan = !m33kRadioAutoScan;

    if (m33kRadioAutoScan) {
        m33kRadioNextProfileMs =
            millis() + M33K_RADIO_AUTO_DWELL_MS;

        if (!m33kRadioListening) {
            startRadioMonitor();
        }
    }

    updateRadioMonitorUi();
}

void radioMeshEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED ||
        !clickAllowed()) return;

    noteActivity();
    hapticTap();

    m33kRadioMeshCore = !m33kRadioMeshCore;
    if (m33kRadioMeshCore) m33kRadioAutoScan = false;   // preset overrides auto

    if (m33kRadioInitialized) {
        applyRadioProfile(m33kRadioListening);
    }

    if (radioStatusLabel) {
        lv_label_set_text(
            radioStatusLabel,
            m33kRadioMeshCore
                ? "MESHCORE LISTEN | 910.525 SF7 BW62.5"
                : "GENERIC RX | 915 MHz"
        );
    }

    updateRadioProfileLabels();
    updateRadioMonitorUi();
}

void reconRadioEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED ||
        !clickAllowed()) return;

    noteActivity();
    hapticTap();
    showRadioPage();
}

void stopWatchTool()
{
    watchPhase = WatchPhase::Idle;
    watchNextSweepMs = 0;

    esp_wifi_scan_stop();
    WiFi.scanDelete();

    if (bleScannerInitialized) {
        NimBLEScan *scan = NimBLEDevice::getScan();

        if (scan->isScanning()) {
            scan->stop();
        }

        scan->clearResults();
    }

    bleScanFinished = false;
}

void updateWatchSummaryLabels()
{
    if (watchWifiLabel) {
        lv_label_set_text_fmt(
            watchWifiLabel,
            "WI-FI %d\n2.4:%d\n5G:%d",
            watchCurrentWifiCount,
            watchCurrentWifi24Count,
            watchCurrentWifi5Count
        );
    }

    if (watchBleLabel) {
        lv_label_set_text_fmt(
            watchBleLabel,
            "BLE\n%d",
            watchCurrentBleCount
        );
    }

    if (watchAlertCountLabel) {
        lv_label_set_text_fmt(
            watchAlertCountLabel,
            "ALERTS\n%u",
            static_cast<unsigned>(watchAlertTotal)
        );
    }
}

String watchShortName(const String &value, size_t maxLen = 22)
{
    if (value.length() <= maxLen) {
        return value;
    }

    return value.substring(0, maxLen - 3) + "...";
}

int findWatchWifiKey(const String &key)
{
    for (uint8_t i = 0; i < watchPrevWifiCount; ++i) {
        if (watchPrevWifiKeys[i] == key) {
            return static_cast<int>(i);
        }
    }

    return -1;
}

bool watchSawSsidBefore(const String &ssid)
{
    for (uint8_t i = 0; i < watchPrevWifiCount; ++i) {
        if (watchPrevWifiSsids[i] == ssid) {
            return true;
        }
    }

    return false;
}

int findWatchBleAddr(const String &addr)
{
    for (uint8_t i = 0; i < watchPrevBleCount; ++i) {
        if (watchPrevBleAddrs[i] == addr) {
            return static_cast<int>(i);
        }
    }

    return -1;
}

void pushWatchAlert(
    const String &text,
    uint32_t colorHex,
    bool audible = true)
{
    ++watchAlertTotal;
    updateWatchSummaryLabels();

    if (audible && watchAlertsArmed) {
        const uint32_t now = millis();
        if (now - watchLastBeepMs >= 650) {
            watchLastBeepMs = now;
            playHunterBeep();
        }
    }

    if (!watchAlertList) return;

    while (lv_obj_get_child_count(watchAlertList) >= WATCH_MAX_ALERT_ROWS) {
        lv_obj_t *first = lv_obj_get_child(watchAlertList, 0);
        if (!first) break;
        lv_obj_delete(first);
    }

    lv_obj_t *row = lv_obj_create(watchAlertList);
    lv_obj_set_width(row, 298);
    lv_obj_set_style_radius(row, 12, 0);
    lv_obj_set_style_bg_color(row, lv_color_hex(0x07101D), 0);
    lv_obj_set_style_bg_opa(row, 235, 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_border_color(row, lv_color_hex(colorHex), 0);
    lv_obj_set_style_pad_all(row, 8, 0);

    lv_obj_t *label = lv_label_create(row);
    lv_label_set_text(label, text.c_str());
    lv_obj_set_width(label, 270);
    lv_obj_set_style_text_color(label, lv_color_hex(colorHex), 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_center(label);
}


bool startWatchBleScan()
{
    initBleScanner();

    NimBLEScan *scan = NimBLEDevice::getScan();

    if (scan->isScanning()) {
        scan->stop();
    }

    scan->clearResults();
    bleScanFinished = false;
    bleScanEndReason = 0;

    watchPhase = WatchPhase::BleScanning;

    if (watchStatusLabel) {
        lv_label_set_text(
            watchStatusLabel,
            "WATCH MODE  |  SCANNING BLE..."
        );
    }

    return scan->start(3000, false, true);
}

void finishWatchWifiScan()
{
    const int16_t result = WiFi.scanComplete();

    if (result == WIFI_SCAN_FAILED) {
        watchPhase = WatchPhase::Failed;

        if (watchStatusLabel) {
            lv_label_set_text(
                watchStatusLabel,
                "WATCH MODE WI-FI SCAN FAILED"
            );
        }

        WiFi.scanDelete();
        return;
    }

    captureLocalWifiResults(result < 0 ? 0 : result);
    mergeC5WifiResults();

    watchCurrentWifiCount = wifiDetailCount;
    watchCurrentWifi24Count = 0;
    watchCurrentWifi5Count = 0;
    for (uint8_t i = 0; i < wifiDetailCount; ++i) {
        if (wifiDetails[i].fromC5 || wifiDetails[i].channel > 14) {
            ++watchCurrentWifi5Count;
        } else {
            ++watchCurrentWifi24Count;
        }
    }
    updateWatchSummaryLabels();

    String nextKeys[WATCH_MAX_WIFI];
    String nextSsids[WATCH_MAX_WIFI];
    uint8_t nextAuth[WATCH_MAX_WIFI] = {};
    int16_t nextChannels[WATCH_MAX_WIFI] = {};
    uint8_t nextCount = 0;

    for (uint8_t i = 0; i < wifiDetailCount && nextCount < WATCH_MAX_WIFI; ++i) {
        const WifiResultDetail &detail = wifiDetails[i];
        const String &ssid = detail.ssid;
        const String &bssid = detail.bssid;
        const String key = ssid + "|" + bssid;
        const uint8_t auth = static_cast<uint8_t>(detail.auth);
        const int16_t channel = static_cast<int16_t>(detail.channel);
        const int rssi = detail.rssi;
        const bool band5 = detail.fromC5 || detail.channel > 14;
        const char *bandLabel = band5 ? "5G" : "2.4";

        if (watchWifiBaselineReady) {
            const int prevIndex = findWatchWifiKey(key);

            if (prevIndex < 0) {
                if (watchSawSsidBefore(ssid)) {
                    pushWatchAlert(
                        String(bandLabel) + " SSID with new BSSID: " + watchShortName(ssid),
                        0xFFB84D
                    );
                } else if (rssi >= -55) {
                    pushWatchAlert(
                        String("New strong ") + bandLabel + " AP: " + watchShortName(ssid) +
                        String("  ") + String(rssi) + " dBm",
                        band5 ? 0x8EA4FF : 0x55EEFF
                    );
                }
            } else {
                if (watchPrevWifiAuth[prevIndex] != auth) {
                    pushWatchAlert(
                        String(bandLabel) + " security changed: " + watchShortName(ssid),
                        0xFF667F
                    );
                }

                if (watchPrevWifiChannels[prevIndex] != channel) {
                    pushWatchAlert(
                        String(bandLabel) + " channel changed: " + watchShortName(ssid) +
                        String(" -> CH ") + String(static_cast<int>(channel)),
                        0xF5FF3B
                    );
                }
            }
        }

        nextKeys[nextCount] = key;
        nextSsids[nextCount] = ssid;
        nextAuth[nextCount] = auth;
        nextChannels[nextCount] = channel;
        ++nextCount;
    }

    for (uint8_t i = 0; i < nextCount; ++i) {
        watchPrevWifiKeys[i] = nextKeys[i];
        watchPrevWifiSsids[i] = nextSsids[i];
        watchPrevWifiAuth[i] = nextAuth[i];
        watchPrevWifiChannels[i] = nextChannels[i];
    }
    watchPrevWifiCount = nextCount;

    WiFi.scanDelete();

    if (!watchWifiBaselineReady) {
        pushWatchAlert(
            "Wi-Fi baseline captured. Future changes will alert here.",
            0x7CFF45,
            false
        );
        watchWifiBaselineReady = true;
    }

    if (!startWatchBleScan()) {
        watchPhase = WatchPhase::Failed;

        if (watchStatusLabel) {
            lv_label_set_text(
                watchStatusLabel,
                "WATCH MODE BLE SCAN COULD NOT START"
            );
        }
    }
}

void finishWatchBleScan()
{
    if (!bleScannerInitialized) {
        watchPhase = WatchPhase::Failed;

        if (watchStatusLabel) {
            lv_label_set_text(
                watchStatusLabel,
                "WATCH MODE BLE SCAN FAILED"
            );
        }

        return;
    }

    NimBLEScan *scan = NimBLEDevice::getScan();
    NimBLEScanResults results = scan->getResults();

    watchCurrentBleCount = results.getCount();
    updateWatchSummaryLabels();

    String nextAddrs[WATCH_MAX_BLE];
    bool nextTracker[WATCH_MAX_BLE] = {};
    uint8_t nextCount = 0;

    for (int i = 0; i < results.getCount() && nextCount < WATCH_MAX_BLE; ++i) {
        const NimBLEAdvertisedDevice *dev = results.getDevice(i);
        if (!dev) continue;

        const String addr = String(dev->getAddress().toString().c_str());
        const bool trackerLike = isAppleFindMyTrackerLike(dev);

        if (watchBleBaselineReady) {
            const int prevIndex = findWatchBleAddr(addr);

            if (prevIndex < 0) {
                pushWatchAlert(
                    trackerLike
                        ? String("Tracker-like BLE appeared: ") + watchShortName(addr, 17)
                        : String("New BLE device: ") + watchShortName(addr, 17),
                    trackerLike ? 0xFFB84D : 0x8EA4FF
                );
            } else if (!watchPrevBleTrackerLike[prevIndex] && trackerLike) {
                pushWatchAlert(
                    String("Known BLE now looks tracker-like: ") + watchShortName(addr, 17),
                    0xFFB84D
                );
            }
        }

        nextAddrs[nextCount] = addr;
        nextTracker[nextCount] = trackerLike;
        ++nextCount;
    }

    for (uint8_t i = 0; i < nextCount; ++i) {
        watchPrevBleAddrs[i] = nextAddrs[i];
        watchPrevBleTrackerLike[i] = nextTracker[i];
    }
    watchPrevBleCount = nextCount;

    scan->clearResults();

    if (!watchBleBaselineReady) {
        pushWatchAlert(
            "BLE baseline captured. New devices will alert here.",
            0x7CFF45,
            false
        );
        watchBleBaselineReady = true;
    }

    if (watchWifiBaselineReady && watchBleBaselineReady && !watchAlertsArmed) {
        watchAlertsArmed = true;
        watchLastBeepMs = millis() - 1000;
    }

    watchPhase = WatchPhase::Idle;
    watchNextSweepMs = millis() + 2500;

    if (watchStatusLabel) {
        lv_label_set_text(
            watchStatusLabel,
            "WATCHING FOR CHANGES  |  BEEP ON"
        );
    }
}

void beginWatchSweep()
{
    if (currentPage != Page::WatchMode) return;

    noteActivity();

    if (watchStatusLabel) {
        lv_label_set_text(
            watchStatusLabel,
            "WATCH MODE  |  SCANNING 2.4 + 5 GHz..."
        );
    }

    requestC5Scan();

    esp_wifi_scan_stop();
    WiFi.scanDelete();

    const bool alreadyConnected =
        WiFi.status() == WL_CONNECTED;

    if (!alreadyConnected) {
        WiFi.disconnect(false, false);
        delay(120);
    }

    WiFi.mode(WIFI_STA);
    delay(220);

    esp_wifi_scan_stop();
    WiFi.scanDelete();
    delay(50);

    int16_t result =
        WiFi.scanNetworks(true, true, false, 300);

    if (result == WIFI_SCAN_FAILED && !alreadyConnected) {
        WiFi.disconnect(true, false);
        delay(120);

        WiFi.mode(WIFI_STA);
        delay(350);

        WiFi.disconnect(false, false);
        delay(100);

        result =
            WiFi.scanNetworks(true, true, false, 350);
    }

    if (result == WIFI_SCAN_FAILED) {
        watchPhase = WatchPhase::Failed;

        if (watchStatusLabel) {
            lv_label_set_text(
                watchStatusLabel,
                "WATCH MODE WI-FI SCAN FAILED"
            );
        }

        return;
    }

    watchPhase = WatchPhase::WifiScanning;
}

void watchTimerCb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    if (currentPage != Page::WatchMode) {
        return;
    }

    serviceC5Link();

    if (watchPhase == WatchPhase::WifiScanning) {
        const int16_t result = WiFi.scanComplete();

        if (result == WIFI_SCAN_RUNNING) {
            return;
        }

        finishWatchWifiScan();
        return;
    }

    if (watchPhase == WatchPhase::Idle && millis() >= watchNextSweepMs) {
        beginWatchSweep();
    }
}

void reconWatchEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();
    showWatchModePage();
}

void watchRefreshEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();
    beginWatchSweep();
}

// -----------------------------------------------------------------------------
// Recon Radar + Signal Hunter
// -----------------------------------------------------------------------------

uint32_t m33kHashString(const String &value)
{
    uint32_t h = 2166136261u;

    for (size_t i = 0; i < value.length(); ++i) {
        h ^= static_cast<uint8_t>(value[i]);
        h *= 16777619u;
    }

    return h;
}

int rssiToRadarRadius(int rssi)
{
    // Stronger signals sit nearer the center.
    const int clamped = constrain(rssi, -95, -35);
    return map(clamped, -35, -95, 24, 122);
}

int rssiToPulseRadius(int rssi)
{
    // Keep every 28 px signal button fully inside the 214 px radar ring.
    // The ring radius is 107 px, so 91 px leaves room for the skull/button
    // itself while still keeping strong APs spread away from the center.
    const int clamped = constrain(rssi, -95, -35);
    return map(clamped, -35, -95, 42, 91);
}

const char *hunterStrengthText(int rssi)
{
    if (rssi >= -45) return "VERY STRONG";
    if (rssi >= -58) return "STRONG";
    if (rssi >= -70) return "MEDIUM";
    if (rssi >= -82) return "WEAK";
    return "VERY WEAK";
}

uint32_t hunterStrengthColor(int rssi)
{
    if (rssi >= -50) return 0x7CFF45;
    if (rssi >= -65) return 0x55EEFF;
    if (rssi >= -78) return 0xF5FF3B;
    return 0xFF667F;
}

bool configureHunterAudio()
{
    if (hunterAudioConfigured) {
        return true;
    }

    // LilyGoLib initializes this bus to a mode that was silent on this unit.
    // The standalone hardware test proved 48 kHz stereo works, so reopen I2S
    // explicitly in that mode for Signal Hunter.
    instance.player.end();
    delay(80);

    instance.player.setPins(
        HUNTER_SPK_BCLK,
        HUNTER_SPK_WCLK,
        HUNTER_SPK_DOUT
    );

    const bool ok =
        instance.player.begin(
            I2S_MODE_STD,
            HUNTER_AUDIO_RATE,
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO,
            I2S_STD_SLOT_BOTH
        );

    if (ok) {
        instance.powerControl(
            POWER_SPEAK,
            true
        );

        delay(80);
        hunterAudioConfigured = true;
    }

    return ok;
}


void prepareHunterBeep()
{
    if (hunterBeepReady) return;

    constexpr float volume = 0.58f;
    constexpr float freq = 1250.0f;

    for (int frame = 0;
         frame < HUNTER_BEEP_FRAMES;
         ++frame) {

        const float sample =
            sinf(
                2.0f * PI * freq *
                static_cast<float>(frame) /
                static_cast<float>(HUNTER_AUDIO_RATE)
            );

        const int16_t pcm =
            static_cast<int16_t>(
                32767.0f * volume * sample
            );

        hunterBeepBuffer[frame * 2] = pcm;
        hunterBeepBuffer[frame * 2 + 1] = pcm;
    }

    hunterBeepReady = true;
}




void playHunterBeep()
{
    prepareHunterBeep();

    if (!configureHunterAudio()) {
        if (hunterStatusLabel) {
            lv_label_set_text(
                hunterStatusLabel,
                "SPEAKER I2S CONFIG FAILED"
            );
        }
        return;
    }

    instance.powerControl(
        POWER_SPEAK,
        true
    );

    delay(8);

    const size_t expected =
        sizeof(hunterBeepBuffer);

    const size_t written =
        instance.player.write(
            reinterpret_cast<uint8_t *>(
                hunterBeepBuffer
            ),
            expected
        );

    if (written != expected &&
        hunterStatusLabel) {

        lv_label_set_text_fmt(
            hunterStatusLabel,
            "AUDIO WRITE %lu/%lu",
            (unsigned long)written,
            (unsigned long)expected
        );
    }
}




// ---- Game sound effects ----------------------------------------------------
// Reuse the Signal Hunter speaker path (48 kHz stereo I2S) and its PCM buffer.
// The games and the Hunter never run at the same time, so sharing the buffer is
// safe; we flag the Hunter beep dirty so it regenerates its own tone next time.
// Tones are kept short so the blocking I2S write barely dents the game tick.
bool gameAudioReady = false;

void gameAudioBegin()
{
    if (configureHunterAudio()) {
        instance.powerControl(POWER_SPEAK, true);
        gameAudioReady = hunterAudioConfigured;
    } else {
        gameAudioReady = false;
    }
}

void gameAudioEnd()
{
    if (!gameAudioReady && !hunterAudioConfigured) return;
    instance.powerControl(POWER_SPEAK, false);
    if (hunterAudioConfigured) { instance.player.end(); hunterAudioConfigured = false; }
    gameAudioReady = false;
}

// Synthesize a short sine sweep (f0 -> f1) with click-free attack/release and
// play it. Capped to the shared buffer length (~90 ms).
void gamePlayTone(float f0, float f1, int ms, float vol)
{
    if (!gameAudioReady) return;
    int frames = HUNTER_AUDIO_RATE * ms / 1000;
    if (frames > HUNTER_BEEP_FRAMES) frames = HUNTER_BEEP_FRAMES;
    if (frames < 1) return;
    const int atk = (frames / 6) > 1 ? frames / 6 : 1;
    const int rel = (frames / 4) > 1 ? frames / 4 : 1;
    float phase = 0.0f;
    for (int i = 0; i < frames; ++i) {
        const float t = (float)i / (float)frames;
        const float f = f0 + (f1 - f0) * t;
        phase += 2.0f * PI * f / (float)HUNTER_AUDIO_RATE;
        float env = 1.0f;
        if (i < atk) env = (float)i / (float)atk;
        else if (i > frames - rel) env = (float)(frames - i) / (float)rel;
        const int16_t pcm = (int16_t)(32767.0f * vol * env * sinf(phase));
        hunterBeepBuffer[i * 2] = pcm;
        hunterBeepBuffer[i * 2 + 1] = pcm;
    }
    hunterBeepReady = false;   // Hunter will rebuild its own beep when next used
    instance.player.write(reinterpret_cast<uint8_t *>(hunterBeepBuffer),
                          frames * 2 * sizeof(int16_t));
}

void sfxJump()    { gamePlayTone(480.0f, 900.0f, 24, 0.50f); }
void sfxFlap()    { gamePlayTone(680.0f, 1040.0f, 14, 0.40f); }
void sfxShoot()   { gamePlayTone(1750.0f, 1050.0f, 12, 0.45f); }
void sfxPop()     { gamePlayTone(320.0f, 90.0f, 22, 0.55f); }
void sfxCollect() { gamePlayTone(760.0f, 1340.0f, 26, 0.50f); }
void sfxDie()     { gamePlayTone(680.0f, 110.0f, 85, 0.60f); }

uint32_t hunterFeedbackInterval(int rssi)
{
    if (rssi >= -45) return 220;
    if (rssi >= -55) return 340;
    if (rssi >= -65) return 520;
    if (rssi >= -75) return 760;
    if (rssi >= -85) return 1050;
    return 1500;
}

void stopHunterTool()
{
    if (hunterTimer) {
        lv_timer_del(hunterTimer);
        hunterTimer = nullptr;
    }

    if (hunterBleScanInFlight &&
        bleScannerInitialized) {

        NimBLEScan *scan =
            NimBLEDevice::getScan();

        if (scan->isScanning()) {
            scan->stop();
        }
    }

    if (hunterWifiScanInFlight) {
        esp_wifi_scan_stop();
        WiFi.scanDelete();
    }

    hunterWifiScanInFlight = false;
    hunterBleScanInFlight = false;
    hunterTargetFromC5 = false;
    hunterPhase = HunterPhase::Idle;
    hunterKind = HunterKind::None;

    // Speaker power is normally off; only the Hunter needs it.
    instance.powerControl(POWER_SPEAK, false);

    if (hunterAudioConfigured) {
        instance.player.end();
        hunterAudioConfigured = false;
    }
}

void stopRadarTool()
{
    if (radarTimer) {
        lv_timer_del(radarTimer);
        radarTimer = nullptr;
    }

    radarPhase = RadarPhase::Idle;

    for (auto &blip : radarBlips) {
        blip.active = false;
        blip.dot = nullptr;
    }

    radarBlipCount = 0;
}

void reconSubBackEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();

    // Wardrive can be inside a live scroll/touch callback when Back is tapped.
    // Rebuilding the whole screen from inside that callback can leave LVGL
    // processing objects that were just deleted. Defer the page transition
    // until the next Arduino loop pass, after LVGL returns from the event.
    if (currentPage == Page::Wardrive) {
        // Freeze every Wardrive UI path immediately.
        // Recon is still created later, outside this LVGL callback.
        wardriveExitToReconPending = true;
        wardrivePageActive = false;
        wardriveRunning = false;
        wardriveUiSyncPending = false;
        wardriveFeedDirty = false;
        wardriveFeedScrolling = false;
        wardriveDetailOpen = false;

        // Do not allow the 150 ms Wardrive timer to touch LVGL objects
        // while the deferred Wardrive -> Recon transition is waiting.
        if (wardriveTimer) {
            lv_timer_del(wardriveTimer);
            wardriveTimer = nullptr;
        }

        // End session-side logging immediately. Radio scans already in
        // progress are still allowed to finish naturally.
        wardriveSdLogging = false;

        return;
    }

    showReconPage();
}

void radarBlipEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed() || currentPage != Page::Radar) return;

    auto *blip =
        static_cast<RadarBlip *>(
            lv_event_get_user_data(e)
        );

    if (!blip || !blip->active || !radarInfoLabel) return;

    noteActivity();
    hapticTap();

    // Repeated Radar taps should not allocate temporary Arduino Strings.
    // Keep the hot click path fixed-buffer only to avoid heap churn while
    // Wi-Fi/BLE/C5 timers are also active.
    char displayName[28] = {};
    const char *sourceName = blip->name.c_str();
    const size_t sourceLen = blip->name.length();

    if (sourceLen > 26) {
        memcpy(displayName, sourceName, 23);
        memcpy(displayName + 23, "...", 4);
    } else {
        strlcpy(displayName, sourceName, sizeof(displayName));
    }

    char info[96] = {};
    snprintf(
        info,
        sizeof(info),
        "%s  %s\n%d dBm",
        blip->wifi
            ? (blip->fromC5 ? "WI-FI 5G" : "WI-FI 2.4")
            : "BLE",
        displayName,
        blip->rssi
    );

    lv_label_set_text(radarInfoLabel, info);
}

void clearRadarBlips()
{
    for (auto &blip : radarBlips) {
        if (blip.dot) {
            lv_obj_delete(blip.dot);
            blip.dot = nullptr;
        }

        blip.active = false;
    }

    radarBlipCount = 0;
}

void addRadarBlip(
    bool wifi,
    const String &name,
    const String &id,
    int rssi,
    bool fromC5 = false)
{
    const uint32_t now = millis();

    // Update an existing signal instead of destroying/recreating it every
    // refresh. Wi-Fi BSSID / BLE address is the stable identity.
    for (auto &blip : radarBlips) {
        if (!blip.active) continue;
        if (blip.wifi != wifi || blip.id != id) continue;

        blip.seenThisSweep = true;
        blip.missing = false;
        blip.fromC5 = fromC5;
        blip.name =
            name.length() > 0
                ? name
                : String("<unnamed>");
        blip.rssi = rssi;
        blip.lastSeenMs = now;
        return;
    }

    RadarBlip *slot = nullptr;
    for (auto &blip : radarBlips) {
        if (!blip.active) {
            slot = &blip;
            break;
        }
    }

    if (!slot) {
        return;
    }

    *slot = RadarBlip{};
    slot->active = true;
    slot->seenThisSweep = true;
    slot->wifi = wifi;
    slot->fromC5 = fromC5;
    slot->name =
        name.length() > 0
            ? name
            : String("<unnamed>");
    slot->id = id;
    slot->rssi = rssi;
    slot->angleDeg =
        static_cast<uint16_t>(
            m33kHashString(id) % 360u
        );
    slot->lastSeenMs = now;
    ++radarBlipCount;
}

void renderRadarBlips()
{
    constexpr int centerX = 205;
    constexpr int centerY = 258;

    for (auto &blip : radarBlips) {
        if (!blip.active) continue;

        const int radius =
            rssiToRadarRadius(blip.rssi);

        const float rad =
            static_cast<float>(
                blip.angleDeg
            ) * PI / 180.0f;

        const int x =
            centerX +
            static_cast<int>(
                cosf(rad) * radius
            );

        const int y =
            centerY +
            static_cast<int>(
                sinf(rad) * radius
            );

        if (!blip.dot) {
            blip.dot =
                lv_button_create(screen);

            lv_obj_remove_style_all(blip.dot);
            lv_obj_set_size(blip.dot, 24, 24);
            lv_obj_set_style_bg_opa(blip.dot, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(blip.dot, 0, 0);
            lv_obj_set_style_shadow_width(blip.dot, 14, 0);
            lv_obj_set_style_shadow_spread(blip.dot, 1, 0);
            lv_obj_set_ext_click_area(blip.dot, 6);

            lv_obj_t *skull = lv_image_create(blip.dot);
            lv_image_set_src(skull, &bunny_skull_graph_img);
            lv_obj_center(skull);

            lv_obj_add_event_cb(
                blip.dot,
                radarBlipEvent,
                LV_EVENT_CLICKED,
                &blip
            );
        }

        lv_obj_set_pos(blip.dot, x - 12, y - 12);
        lv_obj_set_style_shadow_color(
            blip.dot,
            lv_color_hex(
                blip.wifi
                    ? (blip.fromC5 ? 0x2F6BFF : 0x55EEFF)
                    : 0xFF52C8
            ),
            0
        );
        lv_obj_set_style_shadow_opa(
            blip.dot,
            blip.missing ? 70 : 180,
            0
        );
        lv_obj_set_style_opa(
            blip.dot,
            blip.missing ? 95 : 255,
            0
        );
    }
}

void addStrongestMergedWifiRadarBlips(uint8_t limit = 6)
{
    bool selected[MAX_WIFI_DETAILS] = {};
    const uint8_t maxWifi = min(limit, wifiDetailCount);

    for (uint8_t slot = 0; slot < maxWifi; ++slot) {
        int best = -1;
        for (uint8_t i = 0; i < wifiDetailCount; ++i) {
            if (selected[i]) continue;
            if (best < 0 || wifiDetails[i].rssi > wifiDetails[best].rssi) {
                best = i;
            }
        }
        if (best < 0) break;

        selected[best] = true;
        const WifiResultDetail &detail = wifiDetails[best];
        addRadarBlip(
            true,
            detail.ssid,
            detail.bssid,
            detail.rssi,
            detail.fromC5 || detail.channel > 14
        );
    }
}

bool startRadarBleScan()
{
    initBleScanner();

    NimBLEScan *scan =
        NimBLEDevice::getScan();

    if (scan->isScanning()) {
        scan->stop();
    }

    scan->clearResults();
    bleScanFinished = false;
    bleScanEndReason = 0;

    radarPhase =
        RadarPhase::BleScanning;

    if (radarStatusLabel) {
        lv_label_set_text(
            radarStatusLabel,
            "WI-FI DONE  |  SCANNING BLE..."
        );
    }

    return scan->start(
        3000,
        false,
        true
    );
}

void finishRadarBleScan()
{
    if (!bleScannerInitialized) {
        radarPhase = RadarPhase::Failed;
        radarNextScanMs = millis() + RADAR_RESCAN_MS;

        if (radarStatusLabel) {
            lv_label_set_text(
                radarStatusLabel,
                "BLE RADAR SCAN FAILED  |  RETRYING..."
            );
        }

        return;
    }

    NimBLEScan *scan =
        NimBLEDevice::getScan();

    NimBLEScanResults results =
        scan->getResults();

    const int count =
        results.getCount();

    const int maxBle =
        min(count, 6);

    // Refresh Wi-Fi entries after the BLE window so a C5 scan that completed
    // during BLE collection can be included without clearing existing blips.
    mergeC5WifiResults();
    addStrongestMergedWifiRadarBlips();

    for (int i = 0; i < maxBle; ++i) {
        const NimBLEAdvertisedDevice *dev =
            results.getDevice(i);

        if (!dev) continue;

        const std::string rawName =
            dev->getName();

        String name =
            rawName.empty()
                ? String("<BLE>")
                : String(rawName.c_str());

        addRadarBlip(
            false,
            name,
            String(
                dev->getAddress()
                    .toString()
                    .c_str()
            ),
            dev->getRSSI()
        );
    }

    const uint32_t now = millis();
    radarBlipCount = 0;

    for (auto &blip : radarBlips) {
        if (!blip.active) continue;

        if (!blip.seenThisSweep) {
            blip.missing = true;

            if (now - blip.lastSeenMs >= RADAR_STALE_MS) {
                if (blip.dot) {
                    lv_obj_delete(blip.dot);
                    blip.dot = nullptr;
                }
                blip = RadarBlip{};
                continue;
            }
        }

        ++radarBlipCount;
    }

    scan->clearResults();

    renderRadarBlips();

    radarPhase = RadarPhase::Done;
    radarNextScanMs = millis() + RADAR_RESCAN_MS;

    if (radarStatusLabel) {
        lv_label_set_text_fmt(
            radarStatusLabel,
            "%u SIGNALS  |  TAP A BLIP",
            radarBlipCount
        );
    }
}

void beginRadarScan()
{
    if (currentPage != Page::Radar) {
        return;
    }

    noteActivity();

    // Preserve current blips while a new sweep runs. Each one is marked
    // unseen and will either be refreshed by this sweep or faded/expired.
    for (auto &blip : radarBlips) {
        if (blip.active) {
            blip.seenThisSweep = false;
        }
    }

    if (radarInfoLabel) {
        lv_label_set_text(
            radarInfoLabel,
            "Radius = signal strength\nAngle = visual placement"
        );
    }

    if (radarStatusLabel) {
        lv_label_set_text(
            radarStatusLabel,
            "SCANNING 2.4 + 5 GHz WI-FI..."
        );
    }

    requestC5Scan();

    esp_wifi_scan_stop();
    WiFi.scanDelete();

    const bool alreadyConnected =
        WiFi.status() == WL_CONNECTED;

    if (!alreadyConnected) {
        WiFi.disconnect(false, false);
        delay(120);
    }

    WiFi.mode(WIFI_STA);
    delay(220);

    esp_wifi_scan_stop();
    WiFi.scanDelete();
    delay(50);

    int16_t result =
        WiFi.scanNetworks(
            true,
            true,
            false,
            300
        );

    if (result == WIFI_SCAN_FAILED &&
        !alreadyConnected) {

        WiFi.disconnect(true, false);
        delay(120);

        WiFi.mode(WIFI_STA);
        delay(350);

        WiFi.disconnect(false, false);
        delay(100);

        result =
            WiFi.scanNetworks(
                true,
                true,
                false,
                350
            );
    }

    if (result == WIFI_SCAN_FAILED) {
        radarPhase = RadarPhase::Failed;
        radarNextScanMs = millis() + RADAR_RESCAN_MS;

        if (radarStatusLabel) {
            lv_label_set_text(
                radarStatusLabel,
                "WI-FI RADAR SCAN FAILED  |  RETRYING..."
            );
        }

        return;
    }

    radarPhase =
        RadarPhase::WifiScanning;
}

void radarRefreshEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    hapticTap();
    radarNextScanMs = 0;
    beginRadarScan();
}

void radarTimerCb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    if (currentPage != Page::Radar) return;

    serviceC5Link();

    // Animated sweep. It is a visualization only, not direction finding.
    radarSweepAngle =
        static_cast<uint16_t>(
            (radarSweepAngle + 5) % 360
        );

    constexpr int centerX = 205;
    constexpr int centerY = 258;

    const float sweepRad =
        static_cast<float>(
            radarSweepAngle
        ) * PI / 180.0f;

    for (uint8_t i = 0; i < 9; ++i) {
        if (!radarSweepDots[i]) continue;

        const int radius =
            14 + i * 14;

        const int x =
            centerX +
            static_cast<int>(
                cosf(sweepRad) * radius
            );

        const int y =
            centerY +
            static_cast<int>(
                sinf(sweepRad) * radius
            );

        lv_obj_set_pos(
            radarSweepDots[i],
            x - 2,
            y - 2
        );
    }

    if ((radarPhase == RadarPhase::Done ||
         radarPhase == RadarPhase::Failed) &&
        radarNextScanMs != 0 &&
        millis() >= radarNextScanMs) {
        beginRadarScan();
        return;
    }

    if (radarPhase !=
        RadarPhase::WifiScanning) {

        return;
    }

    const int16_t result =
        WiFi.scanComplete();

    if (result == WIFI_SCAN_RUNNING) {
        return;
    }

    if (result == WIFI_SCAN_FAILED) {
        radarPhase = RadarPhase::Failed;

        if (radarStatusLabel) {
            lv_label_set_text(
                radarStatusLabel,
                "WI-FI RADAR SCAN FAILED"
            );
        }

        WiFi.scanDelete();
        return;
    }

    captureLocalWifiResults(result);
    mergeC5WifiResults();

    // Initial merged selection; finishRadarBleScan rebuilds it once more so a
    // C5 sweep that completes during BLE scanning is included immediately.
    addStrongestMergedWifiRadarBlips();

    WiFi.scanDelete();

    if (!startRadarBleScan()) {
        radarPhase = RadarPhase::Failed;
        radarNextScanMs = millis() + RADAR_RESCAN_MS;

        if (radarStatusLabel) {
            lv_label_set_text(
                radarStatusLabel,
                "BLE RADAR SCAN BUSY  |  RETRYING..."
            );
        }
    }
}

void showHunterChooser()
{
    if (hunterTrackingPanel) {
        lv_obj_add_flag(
            hunterTrackingPanel,
            LV_OBJ_FLAG_HIDDEN
        );
    }

    if (hunterList) {
        lv_obj_remove_flag(
            hunterList,
            LV_OBJ_FLAG_HIDDEN
        );
        lv_obj_move_foreground(hunterList);
    }
}

void showHunterTracking()
{
    if (hunterList) {
        lv_obj_add_flag(
            hunterList,
            LV_OBJ_FLAG_HIDDEN
        );
    }

    if (hunterTrackingPanel) {
        lv_obj_remove_flag(
            hunterTrackingPanel,
            LV_OBJ_FLAG_HIDDEN
        );
        lv_obj_move_foreground(
            hunterTrackingPanel
        );
    }
}


bool isAppleFindMyTrackerLike(
    const NimBLEAdvertisedDevice *dev)
{
    if (!dev) return false;

    const uint8_t count =
        dev->getManufacturerDataCount();

    for (uint8_t i = 0; i < count; ++i) {
        const std::string data =
            dev->getManufacturerData(i);

        // Apple Company ID 0x004C is stored little-endian as 4C 00.
        // Offline Finding / Find My frame: type 0x12, length 0x19.
        if (data.size() >= 4) {
            const uint8_t *p =
                reinterpret_cast<const uint8_t *>(
                    data.data()
                );

            if (p[0] == 0x4C &&
                p[1] == 0x00 &&
                p[2] == 0x12 &&
                p[3] == 0x19) {

                return true;
            }
        }
    }

    return false;
}

void renderHunterTrackerTargets()
{
    if (!hunterList ||
        !bleScannerInitialized) {

        return;
    }

    NimBLEScan *scan =
        NimBLEDevice::getScan();

    NimBLEScanResults results =
        scan->getResults();

    lv_obj_clean(hunterList);
    bleDetailCount = 0;

    int trackerCount = 0;

    for (int i = 0;
         i < results.getCount() &&
         bleDetailCount < MAX_BLE_DETAILS &&
         trackerCount < 10;
         ++i) {

        const NimBLEAdvertisedDevice *dev =
            results.getDevice(i);

        if (!dev ||
            !isAppleFindMyTrackerLike(dev)) {

            continue;
        }

        BleResultDetail &detail =
            bleDetails[bleDetailCount];

        detail.name =
            "Find My / AirTag-class";
        detail.address =
            String(
                dev->getAddress()
                    .toString()
                    .c_str()
            );
        detail.rssi = dev->getRSSI();

        lv_obj_t *row =
            lv_button_create(hunterList);

        lv_obj_set_size(row, 316, 58);
        lv_obj_set_style_radius(row, 12, 0);
        lv_obj_set_style_bg_color(
            row,
            lv_color_hex(0x1A1207),
            0
        );
        lv_obj_set_style_border_width(
            row,
            1,
            0
        );
        lv_obj_set_style_border_color(
            row,
            lv_color_hex(0xFFB84D),
            0
        );

        lv_obj_add_event_cb(
            row,
            hunterBleTargetEvent,
            LV_EVENT_CLICKED,
            &detail
        );

        lv_obj_t *label =
            lv_label_create(row);

        lv_label_set_text_fmt(
            label,
            "FIND MY TRACKER-LIKE\n%d dBm",
            detail.rssi
        );

        lv_obj_set_style_text_color(
            label,
            lv_color_hex(0xFFB84D),
            0
        );
        lv_obj_set_style_text_font(
            label,
            &lv_font_montserrat_12,
            0
        );
        lv_obj_center(label);

        ++bleDetailCount;
        ++trackerCount;
    }

    scan->clearResults();

    if (trackerCount == 0) {
        lv_obj_t *none =
            lv_label_create(hunterList);

        lv_label_set_text(
            none,
            "NO TRACKER-LIKE SIGNALS FOUND"
        );

        lv_obj_set_width(none, 300);
        lv_obj_set_style_text_align(
            none,
            LV_TEXT_ALIGN_CENTER,
            0
        );
        lv_obj_set_style_text_color(
            none,
            lv_color_hex(0xE6EDF7),
            0
        );
        lv_obj_set_style_text_font(
            none,
            &lv_font_montserrat_12,
            0
        );
    }

    if (hunterStatusLabel) {
        lv_label_set_text_fmt(
            hunterStatusLabel,
            "%d TRACKER-LIKE SIGNAL%s",
            trackerCount,
            trackerCount == 1 ? "" : "S"
        );
    }
}

void cancelHunterSelectionScans()
{
    // Cancel any stale Wi-Fi selection scan.
    if (hunterWifiScanInFlight) {
        esp_wifi_scan_stop();
        WiFi.scanDelete();
        hunterWifiScanInFlight = false;
    }

    // Cancel any stale BLE/tracker selection scan.
    if (bleScannerInitialized) {
        NimBLEScan *scan =
            NimBLEDevice::getScan();

        if (scan->isScanning()) {
            scan->stop();
            delay(20);
        }

        scan->clearResults();
    }

    hunterBleScanInFlight = false;
    bleScanFinished = false;
}

void startHunterTrackerSelection()
{
    if (currentPage !=
        Page::SignalHunter) {

        return;
    }

    cancelHunterSelectionScans();

    hunterKind = HunterKind::None;
    hunterTrackerSelectionMode = true;
    hunterPhase =
        HunterPhase::BleSelecting;

    showHunterChooser();

    if (hunterList) {
        lv_obj_clean(hunterList);
        lv_obj_remove_flag(
            hunterList,
            LV_OBJ_FLAG_HIDDEN
        );
    }

    if (hunterStatusLabel) {
        lv_label_set_text(
            hunterStatusLabel,
            "SCANNING FIND MY TRACKERS..."
        );
    }

    initBleScanner();

    NimBLEScan *scan =
        NimBLEDevice::getScan();

    scan->clearResults();
    bleScanFinished = false;
    hunterBleScanInFlight = true;

    if (!scan->start(
            4500,
            false,
            true)) {

        hunterBleScanInFlight = false;

        if (hunterStatusLabel) {
            lv_label_set_text(
                hunterStatusLabel,
                "TRACKER SCAN FAILED"
            );
        }
    }
}


void hunterTrackerButtonEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();
    startHunterTrackerSelection();
}


void hunterResetHistory()
{
    hunterHistoryUsed = 0;

    for (uint8_t i = 0;
         i < HUNTER_HISTORY_COUNT;
         ++i) {

        hunterHistory[i] = -100;
    }
}

void hunterPushHistory(int rssi)
{
    if (hunterHistoryUsed <
        HUNTER_HISTORY_COUNT) {

        hunterHistory[
            hunterHistoryUsed++
        ] = rssi;
    } else {
        for (uint8_t i = 1;
             i < HUNTER_HISTORY_COUNT;
             ++i) {

            hunterHistory[i - 1] =
                hunterHistory[i];
        }

        hunterHistory[
            HUNTER_HISTORY_COUNT - 1
        ] = rssi;
    }

    const uint8_t count =
        hunterHistoryUsed;

    for (uint8_t i = 0;
         i < HUNTER_HISTORY_COUNT;
         ++i) {

        if (!hunterHistoryBars[i]) {
            continue;
        }

        int value =
            i < count
                ? hunterHistory[i]
                : -100;

        value =
            constrain(value, -100, -35);

        const int height =
            map(value, -100, -35, 3, 50);

        lv_obj_set_height(
            hunterHistoryBars[i],
            height
        );

        lv_obj_set_y(
            hunterHistoryBars[i],
            54 - height
        );

        lv_obj_set_style_bg_color(
            hunterHistoryBars[i],
            lv_color_hex(
                hunterStrengthColor(value)
            ),
            0
        );
    }
}

void updateHunterMeter(
    bool seen,
    int rssi)
{
    if (!hunterRssiLabel ||
        !hunterSignalBar ||
        !hunterStrengthLabel) {

        return;
    }

    if (!seen) {
        hunterCurrentRssi = -127;

        lv_label_set_text(
            hunterRssiLabel,
            "-- dBm"
        );

        lv_label_set_text(
            hunterStrengthLabel,
            "TARGET NOT SEEN"
        );

        lv_obj_set_style_text_color(
            hunterStrengthLabel,
            lv_color_hex(0xFF667F),
            0
        );

        lv_bar_set_value(
            hunterSignalBar,
            0,
            LV_ANIM_OFF
        );

        return;
    }

    hunterCurrentRssi = rssi;
    hunterLastSeenMs = millis();

    if (hunterBestRssi == -127 ||
        rssi > hunterBestRssi) {

        hunterBestRssi = rssi;
    }

    if (hunterWeakestRssi == 0 ||
        rssi < hunterWeakestRssi) {

        hunterWeakestRssi = rssi;
    }

    lv_label_set_text_fmt(
        hunterRssiLabel,
        "%d dBm",
        rssi
    );

    lv_label_set_text(
        hunterStrengthLabel,
        hunterStrengthText(rssi)
    );

    lv_obj_set_style_text_color(
        hunterStrengthLabel,
        lv_color_hex(
            hunterStrengthColor(rssi)
        ),
        0
    );

    const int percent =
        constrain(
            map(
                constrain(rssi, -100, -35),
                -100,
                -35,
                0,
                100
            ),
            0,
            100
        );

    lv_bar_set_value(
        hunterSignalBar,
        percent,
        LV_ANIM_ON
    );

    hunterPushHistory(rssi);

    if (hunterStatusLabel) {
        lv_label_set_text_fmt(
            hunterStatusLabel,
            "BEST %d  |  WEAKEST %d dBm",
            hunterBestRssi,
            hunterWeakestRssi
        );
    }
}

void hunterDoFeedback()
{
    if (hunterKind == HunterKind::None ||
        hunterCurrentRssi <= -126 ||
        screenSleeping) {

        return;
    }

    if (hunterFeedbackMode ==
        HunterFeedbackMode::Off) {

        return;
    }

    const uint32_t now =
        millis();

    const uint32_t interval =
        hunterFeedbackInterval(
            hunterCurrentRssi
        );

    if (now - hunterLastFeedbackMs <
        interval) {

        return;
    }

    hunterLastFeedbackMs = now;

    const bool useBeep =
        hunterFeedbackMode ==
            HunterFeedbackMode::Beep ||
        hunterFeedbackMode ==
            HunterFeedbackMode::Both;

    const bool useHaptic =
        hunterFeedbackMode ==
            HunterFeedbackMode::Haptic ||
        hunterFeedbackMode ==
            HunterFeedbackMode::Both;

    if (useBeep) {
        playHunterBeep();
    }

    if (useHaptic) {
        instance.vibrator();
    }
}

void updateHunterModeLabel()
{
    if (!hunterModeLabel) return;

    const char *mode = "OFF";

    switch (hunterFeedbackMode) {
        case HunterFeedbackMode::Beep:
            mode = "BEEP";
            break;
        case HunterFeedbackMode::Haptic:
            mode = "HAPTIC";
            break;
        case HunterFeedbackMode::Both:
            mode = "BOTH";
            break;
        default:
            mode = "OFF";
            break;
    }

    lv_label_set_text_fmt(
        hunterModeLabel,
        "FEEDBACK: %s",
        mode
    );
}

void hunterModeEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();

    uint8_t value =
        static_cast<uint8_t>(
            hunterFeedbackMode
        );

    value =
        static_cast<uint8_t>(
            (value + 1) % 4
        );

    hunterFeedbackMode =
        static_cast<HunterFeedbackMode>(
            value
        );

    updateHunterModeLabel();

    if (hunterFeedbackMode ==
            HunterFeedbackMode::Beep ||
        hunterFeedbackMode ==
            HunterFeedbackMode::Both) {

        playHunterBeep();
    }
}

void hunterSoundTestEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();

    if (hunterStatusLabel) {
        lv_label_set_text(
            hunterStatusLabel,
            "SPEAKER TEST 48kHz"
        );
    }

    playHunterBeep();
    delay(160);
    playHunterBeep();
}



void hunterWifiTargetEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    auto *detail =
        static_cast<WifiResultDetail *>(
            lv_event_get_user_data(e)
        );

    if (!detail) return;

    noteActivity();
    hapticTap();

    hunterKind = HunterKind::WiFi;
    hunterTargetName = detail->ssid;
    hunterTargetId = detail->bssid;
    hunterTargetFromC5 = detail->fromC5 || detail->channel > 14;
    hunterAppliedC5Generation = c5ScanGeneration;
    hunterPhase = HunterPhase::Tracking;
    hunterBestRssi = -127;
    hunterWeakestRssi = 0;
    hunterCurrentRssi = detail->rssi;
    hunterWifiScanInFlight = false;
    hunterBleScanInFlight = false;
    hunterNextScanMs = 0;
    hunterLastFeedbackMs = 0;
    hunterResetHistory();

    if (hunterTargetLabel) {
        lv_label_set_text_fmt(
            hunterTargetLabel,
            "%s TARGET\n%s",
            hunterTargetFromC5 ? "5 GHz" : "2.4 GHz",
            hunterTargetName.c_str()
        );
    }

    showHunterTracking();

    updateHunterMeter(
        true,
        detail->rssi
    );
}

void hunterBleTargetEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    auto *detail =
        static_cast<BleResultDetail *>(
            lv_event_get_user_data(e)
        );

    if (!detail) return;

    noteActivity();
    hapticTap();

    hunterKind = HunterKind::BLE;
    hunterTargetName = detail->name;
    hunterTargetId = detail->address;
    hunterPhase = HunterPhase::Tracking;
    hunterBestRssi = -127;
    hunterWeakestRssi = 0;
    hunterCurrentRssi = detail->rssi;
    hunterWifiScanInFlight = false;
    hunterBleScanInFlight = false;
    hunterNextScanMs = 0;
    hunterLastFeedbackMs = 0;
    hunterResetHistory();

    if (hunterTargetLabel) {
        lv_label_set_text_fmt(
            hunterTargetLabel,
            "BLE TARGET\n%s",
            hunterTargetName.c_str()
        );
    }

    showHunterTracking();

    updateHunterMeter(
        true,
        detail->rssi
    );
}

void renderHunterWifiTargets()
{
    if (!hunterList) return;

    const int16_t result =
        WiFi.scanComplete();

    if (result < 0) return;

    lv_obj_clean(hunterList);
    captureLocalWifiResults(result);
    mergeC5WifiResults();

    const int visible = static_cast<int>(wifiDetailCount);

    if (visible <= 0) {
        lv_obj_t *none =
            lv_label_create(hunterList);

        lv_label_set_text(
            none,
            "No Wi-Fi targets found."
        );

        lv_obj_set_style_text_color(
            none,
            lv_color_hex(0xFFFFFF),
            0
        );

        WiFi.scanDelete();
        return;
    }

    for (int i = 0; i < visible; ++i) {
        WifiResultDetail &detail = wifiDetails[i];
        const bool band5 = detail.fromC5 || detail.channel > 14;

        lv_obj_t *row =
            lv_button_create(hunterList);

        lv_obj_set_size(row, 316, 54);
        lv_obj_set_style_radius(row, 12, 0);
        lv_obj_set_style_bg_color(
            row,
            lv_color_hex(0x07101D),
            0
        );
        lv_obj_set_style_border_width(
            row,
            1,
            0
        );
        lv_obj_set_style_border_color(
            row,
            lv_color_hex(band5 ? 0x2F6BFF : 0x55EEFF),
            0
        );

        lv_obj_add_event_cb(
            row,
            hunterWifiTargetEvent,
            LV_EVENT_CLICKED,
            &detail
        );

        lv_obj_t *label =
            lv_label_create(row);

        String shortName =
            detail.ssid;

        if (shortName.length() > 20) {
            shortName =
                shortName.substring(0, 17) +
                "...";
        }

        lv_label_set_text_fmt(
            label,
            "%s\n%s  %d dBm  CH %ld",
            shortName.c_str(),
            band5 ? "5G" : "2.4",
            detail.rssi,
            (long)detail.channel
        );

        lv_obj_set_style_text_color(
            label,
            lv_color_hex(band5 ? 0x8EA4FF : 0x55EEFF),
            0
        );

        lv_obj_set_style_text_font(
            label,
            &lv_font_montserrat_12,
            0
        );

        lv_obj_center(label);

    }

    WiFi.scanDelete();

    if (hunterStatusLabel) {
        lv_label_set_text(
            hunterStatusLabel,
            "SELECT A 2.4 / 5 GHz TARGET"
        );
    }
}

void renderHunterBleTargets()
{
    if (!hunterList ||
        !bleScannerInitialized) {

        return;
    }

    NimBLEScan *scan =
        NimBLEDevice::getScan();

    NimBLEScanResults results =
        scan->getResults();

    lv_obj_clean(hunterList);
    bleDetailCount = 0;

    const int visible =
        min(
            results.getCount(),
            10
        );

    if (visible <= 0) {
        lv_obj_t *none =
            lv_label_create(hunterList);

        lv_label_set_text(
            none,
            "No BLE targets found."
        );

        lv_obj_set_style_text_color(
            none,
            lv_color_hex(0xFFFFFF),
            0
        );

        scan->clearResults();
        return;
    }

    for (int i = 0; i < visible; ++i) {
        const NimBLEAdvertisedDevice *dev =
            results.getDevice(i);

        if (!dev) continue;

        BleResultDetail &detail =
            bleDetails[bleDetailCount];

        const std::string rawName =
            dev->getName();

        detail.name =
            rawName.empty()
                ? String("<unnamed>")
                : String(rawName.c_str());

        detail.address =
            String(
                dev->getAddress()
                    .toString()
                    .c_str()
            );

        detail.rssi = dev->getRSSI();

        lv_obj_t *row =
            lv_button_create(hunterList);

        lv_obj_set_size(row, 316, 54);
        lv_obj_set_style_radius(row, 12, 0);
        lv_obj_set_style_bg_color(
            row,
            lv_color_hex(0x07101D),
            0
        );
        lv_obj_set_style_border_width(
            row,
            1,
            0
        );
        lv_obj_set_style_border_color(
            row,
            lv_color_hex(0x6D86FF),
            0
        );

        lv_obj_add_event_cb(
            row,
            hunterBleTargetEvent,
            LV_EVENT_CLICKED,
            &detail
        );

        lv_obj_t *label =
            lv_label_create(row);

        String shortName =
            detail.name;

        if (shortName.length() > 20) {
            shortName =
                shortName.substring(0, 17) +
                "...";
        }

        lv_label_set_text_fmt(
            label,
            "%s\n%d dBm",
            shortName.c_str(),
            detail.rssi
        );

        lv_obj_set_style_text_color(
            label,
            lv_color_hex(0x8EA4FF),
            0
        );

        lv_obj_set_style_text_font(
            label,
            &lv_font_montserrat_12,
            0
        );

        lv_obj_center(label);

        ++bleDetailCount;
    }

    scan->clearResults();

    if (hunterStatusLabel) {
        lv_label_set_text(
            hunterStatusLabel,
            "SELECT A BLE TARGET"
        );
    }
}

void startHunterWifiSelection()
{
    if (currentPage !=
        Page::SignalHunter) {

        return;
    }

    cancelHunterSelectionScans();

    hunterKind = HunterKind::None;
    hunterTrackerSelectionMode = false;
    hunterPhase =
        HunterPhase::WifiSelecting;

    showHunterChooser();

    if (hunterList) {
        lv_obj_clean(hunterList);
        lv_obj_remove_flag(
            hunterList,
            LV_OBJ_FLAG_HIDDEN
        );
    }

    if (hunterStatusLabel) {
        lv_label_set_text(
            hunterStatusLabel,
            "SCANNING 2.4 + 5 GHz TARGETS..."
        );
    }

    requestC5Scan();

    esp_wifi_scan_stop();
    WiFi.scanDelete();

    const bool connected =
        WiFi.status() == WL_CONNECTED;

    if (!connected) {
        WiFi.disconnect(false, false);
        delay(100);
    }

    WiFi.mode(WIFI_STA);
    delay(200);

    int16_t result =
        WiFi.scanNetworks(
            true,
            true,
            false,
            300
        );

    if (result == WIFI_SCAN_FAILED &&
        !connected) {

        WiFi.disconnect(true, false);
        delay(120);
        WiFi.mode(WIFI_STA);
        delay(320);
        WiFi.disconnect(false, false);
        delay(80);

        result =
            WiFi.scanNetworks(
                true,
                true,
                false,
                350
            );
    }

    hunterWifiScanInFlight =
        result != WIFI_SCAN_FAILED;

    if (!hunterWifiScanInFlight &&
        hunterStatusLabel) {

        lv_label_set_text(
            hunterStatusLabel,
            "WI-FI TARGET SCAN FAILED"
        );
    }
}


void startHunterBleSelection()
{
    if (currentPage !=
        Page::SignalHunter) {

        return;
    }

    cancelHunterSelectionScans();

    hunterKind = HunterKind::None;

    // Critical: normal BLE selection must explicitly clear tracker mode.
    hunterTrackerSelectionMode = false;
    hunterPhase =
        HunterPhase::BleSelecting;

    showHunterChooser();

    if (hunterList) {
        lv_obj_clean(hunterList);
        lv_obj_remove_flag(
            hunterList,
            LV_OBJ_FLAG_HIDDEN
        );
    }

    if (hunterStatusLabel) {
        lv_label_set_text(
            hunterStatusLabel,
            "SCANNING BLE TARGETS..."
        );
    }

    initBleScanner();

    NimBLEScan *scan =
        NimBLEDevice::getScan();

    scan->clearResults();
    bleScanFinished = false;
    hunterBleScanInFlight = true;

    if (!scan->start(
            3000,
            false,
            true)) {

        hunterBleScanInFlight = false;

        if (hunterStatusLabel) {
            lv_label_set_text(
                hunterStatusLabel,
                "BLE TARGET SCAN FAILED"
            );
        }
    }
}


void hunterWifiButtonEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();
    startHunterWifiSelection();
}

void hunterBleButtonEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();
    startHunterBleSelection();
}

void startHunterTrackingWifiScan()
{
    if (hunterKind != HunterKind::WiFi ||
        hunterWifiScanInFlight) {

        return;
    }

    if (hunterTargetFromC5) {
        if (!c5ScanInFlight) {
            requestC5Scan();
        }
        hunterNextScanMs = millis() + 650;
        return;
    }

    esp_wifi_scan_stop();
    WiFi.scanDelete();

    WiFi.mode(WIFI_STA);
    delay(35);

    const int16_t result =
        WiFi.scanNetworks(
            true,
            true,
            false,
            220
        );

    hunterWifiScanInFlight =
        result != WIFI_SCAN_FAILED;

    if (!hunterWifiScanInFlight) {
        hunterNextScanMs =
            millis() + 700;
    }
}

void finishHunterTrackingWifiScan()
{
    const int16_t result =
        WiFi.scanComplete();

    bool seen = false;
    int rssi = -127;

    if (result >= 0) {
        for (int16_t i = 0;
             i < result;
             ++i) {

            if (WiFi.BSSIDstr(i)
                .equalsIgnoreCase(
                    hunterTargetId
                )) {

                rssi = WiFi.RSSI(i);
                seen = true;
                break;
            }
        }
    }

    WiFi.scanDelete();
    hunterWifiScanInFlight = false;
    hunterNextScanMs =
        millis() + 650;

    updateHunterMeter(
        seen,
        rssi
    );
}

void startHunterTrackingBleScan()
{
    if (hunterKind != HunterKind::BLE ||
        hunterBleScanInFlight) {

        return;
    }

    initBleScanner();

    NimBLEScan *scan =
        NimBLEDevice::getScan();

    if (scan->isScanning()) {
        scan->stop();
    }

    scan->clearResults();
    bleScanFinished = false;
    hunterBleScanInFlight = true;

    if (!scan->start(
            1200,
            false,
            true)) {

        hunterBleScanInFlight = false;
        hunterNextScanMs =
            millis() + 650;
    }
}

void finishHunterTrackingBleScan()
{
    if (!bleScannerInitialized) {
        hunterBleScanInFlight = false;
        return;
    }

    NimBLEScan *scan =
        NimBLEDevice::getScan();

    NimBLEScanResults results =
        scan->getResults();

    bool seen = false;
    int rssi = -127;

    for (int i = 0;
         i < results.getCount();
         ++i) {

        const NimBLEAdvertisedDevice *dev =
            results.getDevice(i);

        if (!dev) continue;

        const String address =
            String(
                dev->getAddress()
                    .toString()
                    .c_str()
            );

        if (address.equalsIgnoreCase(
                hunterTargetId)) {

            rssi = dev->getRSSI();
            seen = true;
            break;
        }
    }

    scan->clearResults();
    hunterBleScanInFlight = false;
    hunterNextScanMs =
        millis() + 450;

    updateHunterMeter(
        seen,
        rssi
    );
}

void hunterTimerCb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    if (currentPage !=
        Page::SignalHunter) {

        return;
    }

    serviceC5Link();

    if (hunterPhase ==
            HunterPhase::WifiSelecting &&
        hunterWifiScanInFlight) {

        const int16_t result =
            WiFi.scanComplete();

        if (result == WIFI_SCAN_RUNNING) {
            return;
        }

        hunterWifiScanInFlight = false;

        if (result == WIFI_SCAN_FAILED) {
            if (hunterStatusLabel) {
                lv_label_set_text(
                    hunterStatusLabel,
                    "WI-FI TARGET SCAN FAILED"
                );
            }

            return;
        }

        renderHunterWifiTargets();
        return;
    }

    if (hunterPhase !=
        HunterPhase::Tracking) {

        return;
    }

    const uint32_t now =
        millis();

    if (hunterKind ==
        HunterKind::WiFi) {

        if (hunterTargetFromC5) {
            if (c5ScanGeneration != hunterAppliedC5Generation) {
                hunterAppliedC5Generation = c5ScanGeneration;

                bool seen = false;
                int rssi = -127;
                const uint8_t remoteCount = c5WifiDetailCount;

                for (uint8_t i = 0; i < remoteCount; ++i) {
                    if (c5WifiDetails[i].bssid.equalsIgnoreCase(hunterTargetId)) {
                        rssi = c5WifiDetails[i].rssi;
                        seen = true;
                        break;
                    }
                }

                updateHunterMeter(seen, rssi);
                hunterNextScanMs = now + 650;
            }

            if (!c5ScanInFlight && now >= hunterNextScanMs) {
                requestC5Scan();
                hunterNextScanMs = now + 900;
            }

            hunterDoFeedback();
            return;
        }

        if (hunterWifiScanInFlight) {
            const int16_t result =
                WiFi.scanComplete();

            if (result !=
                WIFI_SCAN_RUNNING) {

                finishHunterTrackingWifiScan();
            }
        } else if (now >=
            hunterNextScanMs) {

            startHunterTrackingWifiScan();
        }
    } else if (
        hunterKind ==
        HunterKind::BLE) {

        if (!hunterBleScanInFlight &&
            now >= hunterNextScanMs) {

            startHunterTrackingBleScan();
        }
    }

    hunterDoFeedback();
}

void reconRadarEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();
    showRadarPage();
}

void reconHunterEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();
    showSignalHunterPage();
}

// -----------------------------------------------------------------------------
// Recon Pulse - passive Wi-Fi change radar
// -----------------------------------------------------------------------------

void deletePulseVisuals()
{
    for (auto &ap : pulseAps) {
        if (ap.dot) {
            lv_obj_delete(ap.dot);
            ap.dot = nullptr;
        }

        for (auto &trailObj : ap.trail) {
            if (trailObj) {
                lv_obj_delete(trailObj);
                trailObj = nullptr;
            }
        }
    }
}

int findPulseAp(const String &bssid)
{
    for (uint8_t i = 0; i < PULSE_MAX_APS; ++i) {
        if (pulseAps[i].active &&
            pulseAps[i].bssid == bssid) {
            return i;
        }
    }

    return -1;
}

int allocPulseAp()
{
    for (uint8_t i = 0; i < PULSE_MAX_APS; ++i) {
        if (!pulseAps[i].active) return i;
    }

    // Prefer replacing the oldest missing entry.
    int oldest = -1;
    uint32_t oldestAge = 0;
    const uint32_t now = millis();

    for (uint8_t i = 0; i < PULSE_MAX_APS; ++i) {
        if (!pulseAps[i].missing) continue;

        const uint32_t age = now - pulseAps[i].lastSeenMs;
        if (oldest < 0 || age > oldestAge) {
            oldest = i;
            oldestAge = age;
        }
    }

    return oldest;
}

void pushPulseHistory(
    uint8_t newCount,
    uint8_t lostCount,
    uint8_t moveCount
)
{
    for (uint8_t i = 1; i < PULSE_HISTORY_COUNT; ++i) {
        pulseHistoryNew[i - 1] = pulseHistoryNew[i];
        pulseHistoryLost[i - 1] = pulseHistoryLost[i];
        pulseHistoryMove[i - 1] = pulseHistoryMove[i];
    }

    pulseHistoryNew[PULSE_HISTORY_COUNT - 1] = newCount;
    pulseHistoryLost[PULSE_HISTORY_COUNT - 1] = lostCount;
    pulseHistoryMove[PULSE_HISTORY_COUNT - 1] = moveCount;

    for (uint8_t i = 0; i < PULSE_HISTORY_COUNT; ++i) {
        const int groupX = 4 + i * 13;

        const int newH = constrain(
            3 + static_cast<int>(pulseHistoryNew[i]) * 7,
            3, 54
        );
        const int lostH = constrain(
            3 + static_cast<int>(pulseHistoryLost[i]) * 7,
            3, 54
        );
        const int moveH = constrain(
            3 + static_cast<int>(pulseHistoryMove[i]) * 7,
            3, 54
        );

        if (pulseGraphNewBars[i]) {
            lv_obj_set_height(pulseGraphNewBars[i], newH);
            lv_obj_align(
                pulseGraphNewBars[i],
                LV_ALIGN_BOTTOM_LEFT,
                groupX,
                -4
            );
        }

        if (pulseGraphLostBars[i]) {
            lv_obj_set_height(pulseGraphLostBars[i], lostH);
            lv_obj_align(
                pulseGraphLostBars[i],
                LV_ALIGN_BOTTOM_LEFT,
                groupX + 3,
                -4
            );
        }

        if (pulseGraphMoveBars[i]) {
            lv_obj_set_height(pulseGraphMoveBars[i], moveH);
            lv_obj_align(
                pulseGraphMoveBars[i],
                LV_ALIGN_BOTTOM_LEFT,
                groupX + 6,
                -4
            );
        }

        const uint8_t opa =
            i == PULSE_HISTORY_COUNT - 1 ? 255 : 170;

        if (pulseGraphNewBars[i]) {
            lv_obj_set_style_bg_opa(
                pulseGraphNewBars[i], opa, 0
            );
        }
        if (pulseGraphLostBars[i]) {
            lv_obj_set_style_bg_opa(
                pulseGraphLostBars[i], opa, 0
            );
        }
        if (pulseGraphMoveBars[i]) {
            lv_obj_set_style_bg_opa(
                pulseGraphMoveBars[i], opa, 0
            );
        }
    }
}

void pulseApEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    auto *ap = static_cast<PulseAp *>(
        lv_event_get_user_data(e)
    );

    if (!ap || !pulseInfoLabel) return;

    noteActivity();
    hapticTap();

    String name = ap->ssid;
    if (name.length() == 0) name = "<hidden>";
    if (name.length() > 22) {
        name = name.substring(0, 19) + "...";
    }

    lv_label_set_text_fmt(
        pulseInfoLabel,
        "%s  %s  CH %d  %d dBm  %s%d dB\n%s",
        name.c_str(),
        ap->fromC5 ? "5G" : "2.4",
        ap->channel,
        ap->rssi,
        ap->deltaRssi >= 0 ? "+" : "",
        ap->deltaRssi,
        ap->bssid.c_str()
    );

    // Signal objects are recreated after the page layout, so explicitly keep
    // the selected-AP information above them in the LVGL draw order.
    lv_obj_move_foreground(pulseInfoLabel);
}

void renderPulseAps()
{
    deletePulseVisuals();

    constexpr int centerX = 205;
    constexpr int centerY = 238;
    const uint32_t now = millis();

    for (uint8_t i = 0; i < PULSE_MAX_APS; ++i) {
        PulseAp &ap = pulseAps[i];
        if (!ap.active) continue;

        const uint32_t age = now - ap.lastSeenMs;
        if (age > PULSE_FADE_MS) {
            ap = PulseAp{};
            continue;
        }

        const float rad =
            static_cast<float>(ap.angleDeg) * PI / 180.0f;

        // Faded RSSI history. Oldest trail is dimmest.
        for (uint8_t t = 0; t < ap.trailCount && t < 4; ++t) {
            const int trailRadius =
                rssiToPulseRadius(ap.trailRssi[t]);

            const int tx =
                centerX + static_cast<int>(cosf(rad) * trailRadius);
            const int ty =
                centerY + static_cast<int>(sinf(rad) * trailRadius);

            lv_obj_t *ghost = lv_image_create(screen);
            ap.trail[t] = ghost;
            lv_image_set_src(ghost, &bunny_skull_graph_img);
            lv_obj_set_pos(ghost, tx - 7, ty - 8);
            lv_obj_set_style_img_recolor(
                ghost,
                lv_color_hex(ap.fromC5 ? 0x2F6BFF : 0x55EEFF),
                0
            );
            lv_obj_set_style_img_recolor_opa(ghost, 70, 0);
            lv_obj_set_style_opa(
                ghost,
                55 + t * 35,
                0
            );
        }

        const int radius = rssiToPulseRadius(ap.rssi);
        const int x =
            centerX + static_cast<int>(cosf(rad) * radius);
        const int y =
            centerY + static_cast<int>(sinf(rad) * radius);

        lv_obj_t *dot = lv_button_create(screen);
        ap.dot = dot;

        lv_obj_remove_style_all(dot);
        lv_obj_set_size(dot, 28, 28);
        lv_obj_set_pos(dot, x - 14, y - 14);
        lv_obj_set_style_bg_opa(dot, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(dot, 0, 0);
        lv_obj_set_style_shadow_width(
            dot,
            abs(ap.deltaRssi) >= 6 ? 18 : 10,
            0
        );
        lv_obj_set_style_shadow_spread(dot, 1, 0);
        lv_obj_set_style_shadow_color(
            dot,
            lv_color_hex(
                abs(ap.deltaRssi) >= 6
                    ? 0x7CFF45
                    : (ap.fromC5 ? 0x2F6BFF : 0x55EEFF)
            ),
            0
        );
        lv_obj_set_style_shadow_opa(dot, 190, 0);
        lv_obj_set_ext_click_area(dot, 5);

        if (ap.missing) {
            const uint8_t opa =
                static_cast<uint8_t>(
                    constrain(
                        map(
                            static_cast<long>(age),
                            0L,
                            static_cast<long>(PULSE_FADE_MS),
                            190L,
                            25L
                        ),
                        25L,
                        190L
                    )
                );
            lv_obj_set_style_opa(dot, opa, 0);
        }

        lv_obj_t *skull = lv_image_create(dot);
        lv_image_set_src(skull, &bunny_skull_graph_img);
        lv_obj_center(skull);

        lv_obj_add_event_cb(
            dot,
            pulseApEvent,
            LV_EVENT_CLICKED,
            &ap
        );
    }

    // Blips are created dynamically after the page widgets. Keep the details
    // strip above them so a lower radar signal can never paint over the text.
    if (pulseInfoLabel) {
        lv_obj_move_foreground(pulseInfoLabel);
    }
}

void updatePulseSummary()
{
    if (pulseNewLabel) {
        lv_label_set_text_fmt(
            pulseNewLabel,
            "NEW\n%u",
            pulseLastNew
        );
    }

    if (pulseLostLabel) {
        lv_label_set_text_fmt(
            pulseLostLabel,
            "LOST\n%u",
            pulseLastLost
        );
    }

    if (pulseMoveLabel) {
        lv_label_set_text_fmt(
            pulseMoveLabel,
            "RSSI Δ\n%u",
            pulseLastMoved
        );
    }
}

bool beginPulseSweep()
{
    if (currentPage != Page::ReconPulse) return false;

    for (auto &ap : pulseAps) {
        if (ap.active) {
            ap.seenThisSweep = false;
        }
    }

    if (pulseStatusLabel) {
        lv_label_set_text(
            pulseStatusLabel,
            "PASSIVE SWEEP  |  SCANNING 2.4 + 5 GHz..."
        );
        lv_obj_set_style_text_color(
            pulseStatusLabel,
            lv_color_hex(0x55EEFF),
            0
        );
    }

    requestC5Scan();

    esp_wifi_scan_stop();
    WiFi.scanDelete();

    const bool connected =
        WiFi.status() == WL_CONNECTED;

    if (!connected) {
        WiFi.disconnect(false, false);
        delay(60);
    }

    WiFi.mode(WIFI_STA);
    delay(90);

    int16_t state = WiFi.scanNetworks(
        true,
        true,
        false,
        220
    );

    if (state == WIFI_SCAN_FAILED && !connected) {
        WiFi.disconnect(true, false);
        delay(70);
        WiFi.mode(WIFI_STA);
        delay(120);
        WiFi.disconnect(false, false);
        delay(50);

        state = WiFi.scanNetworks(
            true,
            true,
            false,
            280
        );
    }

    if (state == WIFI_SCAN_FAILED) {
        pulsePhase = PulsePhase::Failed;

        if (pulseStatusLabel) {
            lv_label_set_text(
                pulseStatusLabel,
                "PASSIVE SWEEP FAILED"
            );
            lv_obj_set_style_text_color(
                pulseStatusLabel,
                lv_color_hex(0xFF667F),
                0
            );
        }

        return false;
    }

    pulsePhase = PulsePhase::WifiScanning;
    return true;
}

void finishPulseSweep(int16_t count)
{
    Serial.printf(
        "[M33K PULSE] finish sweep: free stack=%u bytes, wifi=%d\n",
        static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)),
        count
    );

    const uint32_t now = millis();

    uint8_t newCount = 0;
    uint8_t lostCount = 0;
    uint8_t movedCount = 0;

    captureLocalWifiResults(count);
    mergeC5WifiResults();

    bool selected[MAX_WIFI_DETAILS] = {};
    const uint8_t take = min(PULSE_MAX_APS, wifiDetailCount);

    for (uint8_t slot = 0; slot < take; ++slot) {
        int best = -1;
        for (uint8_t i = 0; i < wifiDetailCount; ++i) {
            if (selected[i]) continue;
            if (best < 0 || wifiDetails[i].rssi > wifiDetails[best].rssi) {
                best = i;
            }
        }
        if (best < 0) break;

        selected[best] = true;
        const WifiResultDetail &detail = wifiDetails[best];
        const String &bssid = detail.bssid;
        const String &ssid = detail.ssid;
        const int rssi = detail.rssi;
        const int channel = detail.channel;
        const bool fromC5 = detail.fromC5 || detail.channel > 14;

        int index = findPulseAp(bssid);

        if (index < 0) {
            index = allocPulseAp();
            if (index < 0) continue;

            pulseAps[index] = PulseAp{};
            PulseAp &ap = pulseAps[index];
            ap.active = true;
            ap.ssid = ssid;
            ap.bssid = bssid;
            ap.rssi = rssi;
            ap.prevRssi = rssi;
            ap.channel = channel;
            ap.fromC5 = fromC5;
            ap.angleDeg =
                static_cast<uint16_t>(
                    m33kHashString(bssid) % 360u
                );
            ap.firstSeenMs = now;
            ap.lastSeenMs = now;
            ap.seenThisSweep = true;
            ap.missing = false;
            ++newCount;
            continue;
        }

        PulseAp &ap = pulseAps[index];
        ap.seenThisSweep = true;

        if (ap.missing) {
            // Reappearance is factual "new to current sweep" activity.
            ++newCount;
        }

        ap.missing = false;
        ap.ssid = ssid;
        ap.channel = channel;
        ap.fromC5 = fromC5;
        ap.prevRssi = ap.rssi;
        ap.deltaRssi = rssi - ap.rssi;

        if (ap.trailCount < 4) {
            ++ap.trailCount;
        }

        for (int t = ap.trailCount - 1; t > 0; --t) {
            ap.trailRssi[t] = ap.trailRssi[t - 1];
        }

        ap.trailRssi[0] = ap.rssi;
        ap.rssi = rssi;
        ap.lastSeenMs = now;

        if (abs(ap.deltaRssi) >= 6) {
            ++movedCount;
        }
    }

    WiFi.scanDelete();

    for (auto &ap : pulseAps) {
        if (!ap.active) continue;

        if (!ap.seenThisSweep) {
            if (!ap.missing) {
                ap.missing = true;
                ++lostCount;
            }

            if (now - ap.lastSeenMs > PULSE_FADE_MS) {
                ap = PulseAp{};
            }
        }
    }

    pulseLastNew = newCount;
    pulseLastLost = lostCount;
    pulseLastMoved = movedCount;

    pushPulseHistory(
        newCount,
        lostCount,
        movedCount
    );
    updatePulseSummary();
    renderPulseAps();

    pulsePhase = PulsePhase::Idle;
    pulseNextSweepMs = now + PULSE_SWEEP_INTERVAL_MS;

    if (pulseStatusLabel) {
        lv_label_set_text_fmt(
            pulseStatusLabel,
            "LIVE  |  NEW %u  LOST %u  RSSI CHANGES %u",
            newCount,
            lostCount,
            movedCount
        );
        lv_obj_set_style_text_color(
            pulseStatusLabel,
            lv_color_hex(0x7CFF45),
            0
        );
    }
}

void pulseTimerCb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    if (currentPage != Page::ReconPulse) return;

    serviceC5Link();

    // Sweep animation is visual only; radius is measured RSSI.
    pulseSweepAngle =
        static_cast<uint16_t>(
            (pulseSweepAngle + 6) % 360
        );

    constexpr int centerX = 205;
    constexpr int centerY = 238;
    const float rad =
        static_cast<float>(pulseSweepAngle) * PI / 180.0f;

    for (uint8_t i = 0; i < 7; ++i) {
        if (!pulseSweepDots[i]) continue;

        const int radius = 12 + i * 15;
        const int x =
            centerX + static_cast<int>(cosf(rad) * radius);
        const int y =
            centerY + static_cast<int>(sinf(rad) * radius);

        lv_obj_set_pos(
            pulseSweepDots[i],
            x - 2,
            y - 2
        );
    }

    if (pulsePhase == PulsePhase::WifiScanning) {
        const int16_t result = WiFi.scanComplete();

        if (result == WIFI_SCAN_RUNNING) {
            return;
        }

        if (result == WIFI_SCAN_FAILED) {
            pulsePhase = PulsePhase::Failed;
            WiFi.scanDelete();

            if (pulseStatusLabel) {
                lv_label_set_text(
                    pulseStatusLabel,
                    "PASSIVE SWEEP FAILED  |  RETRYING..."
                );
                lv_obj_set_style_text_color(
                    pulseStatusLabel,
                    lv_color_hex(0xFF667F),
                    0
                );
            }

            pulseNextSweepMs = millis() + 1600;
            return;
        }

        finishPulseSweep(result);
        return;
    }

    if (pulsePhase == PulsePhase::Failed &&
        millis() >= pulseNextSweepMs) {
        pulsePhase = PulsePhase::Idle;
    }

    if (pulsePhase == PulsePhase::Idle &&
        millis() >= pulseNextSweepMs) {
        beginPulseSweep();
    }
}

void stopReconPulseTool()
{
    if (pulseTimer) {
        lv_timer_del(pulseTimer);
        pulseTimer = nullptr;
    }

    if (pulsePhase == PulsePhase::WifiScanning) {
        esp_wifi_scan_stop();
        WiFi.scanDelete();
    }

    pulsePhase = PulsePhase::Idle;
    deletePulseVisuals();

    for (auto &p : pulseSweepDots) p = nullptr;
    for (auto &p : pulseGraphNewBars) p = nullptr;
    for (auto &p : pulseGraphLostBars) p = nullptr;
    for (auto &p : pulseGraphMoveBars) p = nullptr;

    pulseStatusLabel = nullptr;
    pulseInfoLabel = nullptr;
    pulseNewLabel = nullptr;
    pulseLostLabel = nullptr;
    pulseMoveLabel = nullptr;
}

void reconPulseEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();
    showReconPulsePage();
}

void showReconPulsePage()
{
    clearScreen();
    currentPage = Page::ReconPulse;
    const bool waitingForC5 = startToolC5Discovery();
    createSafeHeaderBack(reconSubBackEvent);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "RECON PULSE");
    lv_obj_set_style_text_color(
        title,
        lv_color_hex(0x55EEFF),
        0
    );
    lv_obj_set_style_text_font(
        title,
        &lv_font_montserrat_20,
        0
    );
    lv_obj_set_width(title, 220);
    lv_obj_set_style_text_align(
        title,
        LV_TEXT_ALIGN_CENTER,
        0
    );
    lv_obj_set_pos(title, 95, 27);

    pulseStatusLabel = lv_label_create(screen);
    lv_label_set_text(
        pulseStatusLabel,
        "STARTING PASSIVE SWEEP..."
    );
    lv_obj_set_width(pulseStatusLabel, 350);
    lv_obj_set_style_text_align(
        pulseStatusLabel,
        LV_TEXT_ALIGN_CENTER,
        0
    );
    lv_obj_set_style_text_color(
        pulseStatusLabel,
        lv_color_hex(0xB6FF00),
        0
    );
    lv_obj_set_style_text_font(
        pulseStatusLabel,
        &lv_font_montserrat_12,
        0
    );
    lv_obj_align(
        pulseStatusLabel,
        LV_ALIGN_TOP_MID,
        0,
        72
    );

    createReconTile(
        30, 96, 104, 54,
        0x55EEFF,
        &pulseNewLabel,
        "NEW\n0",
        0x55EEFF
    );

    createReconTile(
        153, 96, 104, 54,
        0xFF667F,
        &pulseLostLabel,
        "LOST\n0",
        0xFF667F
    );

    createReconTile(
        276, 96, 104, 54,
        0x7CFF45,
        &pulseMoveLabel,
        "RSSI Δ\n0",
        0x7CFF45
    );

    // Radar rings.
    const int ringSizes[3] = {214, 150, 86};

    for (uint8_t i = 0; i < 3; ++i) {
        lv_obj_t *ring = lv_obj_create(screen);
        lv_obj_remove_style_all(ring);
        lv_obj_set_size(ring, ringSizes[i], ringSizes[i]);
        lv_obj_set_pos(
            ring,
            205 - ringSizes[i] / 2,
            238 - ringSizes[i] / 2
        );
        lv_obj_set_style_radius(
            ring,
            ringSizes[i] / 2,
            0
        );
        lv_obj_set_style_bg_opa(
            ring,
            LV_OPA_TRANSP,
            0
        );
        lv_obj_set_style_border_width(ring, 1, 0);
        lv_obj_set_style_border_color(
            ring,
            lv_color_hex(
                i == 0 ? 0x20536B : 0x16384A
            ),
            0
        );
    }

    lv_obj_t *center = lv_obj_create(screen);
    lv_obj_remove_style_all(center);
    lv_obj_set_size(center, 12, 12);
    lv_obj_set_pos(center, 199, 232);
    lv_obj_set_style_radius(center, 6, 0);
    lv_obj_set_style_bg_color(
        center,
        lv_color_hex(0xFFFFFF),
        0
    );
    lv_obj_set_style_bg_opa(
        center,
        LV_OPA_COVER,
        0
    );

    for (uint8_t i = 0; i < 7; ++i) {
        pulseSweepDots[i] = lv_obj_create(screen);
        lv_obj_remove_style_all(pulseSweepDots[i]);
        lv_obj_set_size(pulseSweepDots[i], 4, 4);
        lv_obj_set_style_radius(pulseSweepDots[i], 2, 0);
        lv_obj_set_style_bg_color(
            pulseSweepDots[i],
            lv_color_hex(0x55EEFF),
            0
        );
        lv_obj_set_style_bg_opa(
            pulseSweepDots[i],
            220 - i * 20,
            0
        );
    }

    pulseInfoLabel = lv_label_create(screen);
    lv_label_set_text(
        pulseInfoLabel,
        "Tap a skull for AP details\n"
        "Radius = RSSI  |  Angle = stable visual slot"
    );
    lv_obj_set_width(pulseInfoLabel, 360);
    lv_obj_set_style_text_align(
        pulseInfoLabel,
        LV_TEXT_ALIGN_CENTER,
        0
    );
    lv_obj_set_style_text_color(
        pulseInfoLabel,
        lv_color_hex(0xE8F2FF),
        0
    );
    lv_obj_set_style_text_font(
        pulseInfoLabel,
        &lv_font_montserrat_12,
        0
    );
    lv_obj_set_pos(pulseInfoLabel, 25, 349);

    lv_obj_t *graphCard = lv_obj_create(screen);
    lv_obj_set_size(graphCard, 350, 86);
    lv_obj_set_pos(graphCard, 30, 390);
    lv_obj_set_style_radius(graphCard, 14, 0);
    lv_obj_set_style_bg_color(
        graphCard,
        lv_color_hex(0x030711),
        0
    );
    lv_obj_set_style_bg_opa(graphCard, 235, 0);
    lv_obj_set_style_border_width(graphCard, 1, 0);
    lv_obj_set_style_border_color(
        graphCard,
        lv_color_hex(0x334B61),
        0
    );
    lv_obj_set_style_pad_all(graphCard, 0, 0);

    lv_obj_t *graphTitle = lv_label_create(graphCard);
    lv_label_set_text(
        graphTitle,
        "CHANGE HISTORY"
    );
    lv_obj_set_style_text_color(
        graphTitle,
        lv_color_hex(0xDDE7F2),
        0
    );
    lv_obj_set_style_text_font(
        graphTitle,
        &lv_font_montserrat_10,
        0
    );
    lv_obj_set_pos(graphTitle, 8, 5);

    lv_obj_t *legendNew = lv_label_create(graphCard);
    lv_label_set_text(legendNew, "NEW");
    lv_obj_set_style_text_color(
        legendNew,
        lv_color_hex(0x55EEFF),
        0
    );
    lv_obj_set_style_text_font(
        legendNew,
        &lv_font_montserrat_10,
        0
    );
    lv_obj_set_pos(legendNew, 206, 5);

    lv_obj_t *legendLost = lv_label_create(graphCard);
    lv_label_set_text(legendLost, "LOST");
    lv_obj_set_style_text_color(
        legendLost,
        lv_color_hex(0xFF667F),
        0
    );
    lv_obj_set_style_text_font(
        legendLost,
        &lv_font_montserrat_10,
        0
    );
    lv_obj_set_pos(legendLost, 248, 5);

    lv_obj_t *legendMove = lv_label_create(graphCard);
    lv_label_set_text(legendMove, "RSSI");
    lv_obj_set_style_text_color(
        legendMove,
        lv_color_hex(0x7CFF45),
        0
    );
    lv_obj_set_style_text_font(
        legendMove,
        &lv_font_montserrat_10,
        0
    );
    lv_obj_set_pos(legendMove, 294, 5);

    for (uint8_t i = 0; i < PULSE_HISTORY_COUNT; ++i) {
        const int groupX = 4 + i * 13;

        pulseGraphNewBars[i] = lv_obj_create(graphCard);
        lv_obj_remove_style_all(pulseGraphNewBars[i]);
        lv_obj_set_width(pulseGraphNewBars[i], 2);
        lv_obj_set_height(pulseGraphNewBars[i], 3);
        lv_obj_set_style_radius(pulseGraphNewBars[i], 2, 0);
        lv_obj_set_style_bg_color(
            pulseGraphNewBars[i],
            lv_color_hex(0x55EEFF),
            0
        );
        lv_obj_set_style_bg_opa(
            pulseGraphNewBars[i],
            170,
            0
        );
        lv_obj_align(
            pulseGraphNewBars[i],
            LV_ALIGN_BOTTOM_LEFT,
            groupX,
            -4
        );

        pulseGraphLostBars[i] = lv_obj_create(graphCard);
        lv_obj_remove_style_all(pulseGraphLostBars[i]);
        lv_obj_set_width(pulseGraphLostBars[i], 2);
        lv_obj_set_height(pulseGraphLostBars[i], 3);
        lv_obj_set_style_radius(pulseGraphLostBars[i], 2, 0);
        lv_obj_set_style_bg_color(
            pulseGraphLostBars[i],
            lv_color_hex(0xFF667F),
            0
        );
        lv_obj_set_style_bg_opa(
            pulseGraphLostBars[i],
            170,
            0
        );
        lv_obj_align(
            pulseGraphLostBars[i],
            LV_ALIGN_BOTTOM_LEFT,
            groupX + 3,
            -4
        );

        pulseGraphMoveBars[i] = lv_obj_create(graphCard);
        lv_obj_remove_style_all(pulseGraphMoveBars[i]);
        lv_obj_set_width(pulseGraphMoveBars[i], 2);
        lv_obj_set_height(pulseGraphMoveBars[i], 3);
        lv_obj_set_style_radius(pulseGraphMoveBars[i], 2, 0);
        lv_obj_set_style_bg_color(
            pulseGraphMoveBars[i],
            lv_color_hex(0x7CFF45),
            0
        );
        lv_obj_set_style_bg_opa(
            pulseGraphMoveBars[i],
            170,
            0
        );
        lv_obj_align(
            pulseGraphMoveBars[i],
            LV_ALIGN_BOTTOM_LEFT,
            groupX + 6,
            -4
        );
    }

    for (auto &ap : pulseAps) {
        ap = PulseAp{};
    }

    for (auto &h : pulseHistoryNew) h = 0;
    for (auto &h : pulseHistoryLost) h = 0;
    for (auto &h : pulseHistoryMove) h = 0;

    pulseLastNew = 0;
    pulseLastLost = 0;
    pulseLastMoved = 0;
    pulseSweepAngle = 0;
    pulsePhase = PulsePhase::Idle;
    pulseNextSweepMs = 0;

    pulseTimer = lv_timer_create(
        pulseTimerCb,
        90,
        nullptr
    );

    if (!waitingForC5) {
        beginPulseSweep();
    } else if (pulseStatusLabel) {
        lv_label_set_text(pulseStatusLabel, "CONNECTING C5...");
    }
    noteActivity();
}

// -----------------------------------------------------------------------------
// GPS / GNSS core
// -----------------------------------------------------------------------------

constexpr uint32_t M33K_GPS_BAUD = 38400;

void ensureGpsReady()
{
    // Keep the Ultra GNSS rail and LilyGoLib-managed UART alive continuously.
    // Do NOT Serial1.end()/begin() on page changes: repeated UART teardown was
    // causing GPS -> Wardrive -> GPS re-entry failures on the real watch.
    instance.powerControl(POWER_GPS, true);
    gpsToolActive = true;
}

void recoverGpsStream()
{
    // If NMEA bytes actually stall, recover the GNSS module without destroying
    // the ESP32 UART driver. The UART remains configured by LilyGoLib.
    instance.powerControl(POWER_GPS, false);
    delay(180);
    instance.powerControl(POWER_GPS, true);
    delay(350);

    gpsToolActive = true;
    gpsPageOpenedMs = millis();
    gpsLastByteMs = gpsPageOpenedMs;
}

void feedGpsSerial()
{
    if (!gpsToolActive) return;

    const uint32_t now = millis();
    const bool noStartupData =
        gpsSessionBytes == 0 && (now - gpsPageOpenedMs) > 5000;
    const bool streamStalled =
        gpsSessionBytes > 0 && (now - gpsLastByteMs) > 7000;

    // Recovery guard: if the GNSS UART/rail ever gets left in a bad state,
    // hard-cycle only the GNSS rail once, then let LilyGoLib re-open Serial1
    // at the Ultra's normal 38400 baud. A new byte re-arms this safeguard.
    if ((noStartupData || streamStalled) && !gpsRecoveryAttempted) {
        gpsRecoveryAttempted = true;
        recoverGpsStream();

        if (currentPage == Page::GPS && gpsStatusLabel) {
            lv_label_set_text(gpsStatusLabel, LV_SYMBOL_GPS "  RESTARTING GNSS...");
            lv_obj_set_style_text_color(gpsStatusLabel, lv_color_hex(0xFFB84D), 0);
        }
    }

    // TEMP DIAGNOSTIC: while on the GPS page, print the raw GNSS stream a few
    // lines per second so we can tell valid NMEA ("$GPGGA,..." text) from
    // baud-mismatch garbage (mostly dots). Remove once GPS is sorted.
    static char gpsRawLine[140];
    static uint8_t gpsRawLen = 0;
    static uint32_t gpsRawLastMs = 0;

    bool receivedByte = false;
    while (Serial1.available() > 0) {
        const int c = Serial1.read();
        if (c < 0) break;

        receivedByte = true;
        gpsParser.encode(static_cast<char>(c));
        ++gpsSessionBytes;
        gpsLastByteMs = millis();

        if (currentPage == Page::GPS) {
            if (c == '\n' || gpsRawLen >= sizeof(gpsRawLine) - 1) {
                gpsRawLine[gpsRawLen] = 0;
                // (raw NMEA serial dump removed for public builds)
                gpsRawLen = 0;
            } else if (c != '\r') {
                gpsRawLine[gpsRawLen++] = (c >= 32 && c < 127) ? (char)c : '.';
            }
        }

        if (gpsParser.location.isUpdated()) {
            gpsSessionLocationSeen = true;
        }
    }

    if (receivedByte) {
        gpsRecoveryAttempted = false;
    }
}

void stopGpsTool()
{
    if (gpsTimer) {
        lv_timer_del(gpsTimer);
        gpsTimer = nullptr;
    }

    // Only stop the page UI timer. Keep GNSS power + NMEA parsing alive across
    // Home, GPS, Wardrive, Recon, and Radio page transitions.
    gpsToolActive = true;
}

void updateGpsUi()
{
    if (currentPage != Page::GPS || !gpsToolActive) return;

    const uint32_t now = millis();
    const bool haveData = gpsSessionBytes > 0;
    const bool recentData =
        haveData && (now - gpsLastByteMs < 3000);

    const bool haveFix =
        gpsSessionLocationSeen &&
        gpsParser.location.isValid() &&
        gpsParser.location.age() < 5000;

    if (gpsStatusLabel) {
        if (haveFix) {
            lv_label_set_text(
                gpsStatusLabel,
                LV_SYMBOL_GPS "  FIX ACQUIRED"
            );
            lv_obj_set_style_text_color(
                gpsStatusLabel,
                lv_color_hex(0x7CFF45),
                0
            );
        } else if (recentData) {
            lv_label_set_text(
                gpsStatusLabel,
                LV_SYMBOL_GPS "  GNSS DATA OK - SEARCHING..."
            );
            lv_obj_set_style_text_color(
                gpsStatusLabel,
                lv_color_hex(0xF5FF3B),
                0
            );
        } else if (now - gpsPageOpenedMs < 5000) {
            lv_label_set_text(
                gpsStatusLabel,
                LV_SYMBOL_GPS "  STARTING GNSS..."
            );
            lv_obj_set_style_text_color(
                gpsStatusLabel,
                lv_color_hex(0xF5FF3B),
                0
            );
        } else {
            lv_label_set_text(
                gpsStatusLabel,
                LV_SYMBOL_GPS "  NO GNSS DATA"
            );
            lv_obj_set_style_text_color(
                gpsStatusLabel,
                lv_color_hex(0xFF667F),
                0
            );
        }
    }

    if (gpsSatLabel) {
        if (gpsParser.satellites.isValid()) {
            lv_label_set_text_fmt(
                gpsSatLabel,
                "SATELLITES  %lu",
                (unsigned long)gpsParser.satellites.value()
            );
        } else {
            lv_label_set_text(gpsSatLabel, "SATELLITES  --");
        }
    }

    if (gpsLatLabel) {
        if (haveFix) {
            lv_label_set_text_fmt(
                gpsLatLabel,
                "LAT   %.6f",
                gpsParser.location.lat()
            );
        } else {
            lv_label_set_text(gpsLatLabel, "LAT   --.------");
        }
    }

    if (gpsLonLabel) {
        if (haveFix) {
            lv_label_set_text_fmt(
                gpsLonLabel,
                "LON   %.6f",
                gpsParser.location.lng()
            );
        } else {
            lv_label_set_text(gpsLonLabel, "LON   --.------");
        }
    }

    if (gpsAltLabel) {
        if (haveFix && gpsParser.altitude.isValid()) {
            lv_label_set_text_fmt(
                gpsAltLabel,
                "ALTITUDE\n%.1f m",
                gpsParser.altitude.meters()
            );
        } else {
            lv_label_set_text(gpsAltLabel, "ALTITUDE\n-- m");
        }
    }

    if (gpsSpeedLabel) {
        if (haveFix && gpsParser.speed.isValid()) {
            lv_label_set_text_fmt(
                gpsSpeedLabel,
                "SPEED\n%.1f km/h",
                gpsParser.speed.kmph()
            );
        } else {
            lv_label_set_text(gpsSpeedLabel, "SPEED\n-- km/h");
        }
    }

    if (gpsHdopLabel) {
        if (gpsParser.hdop.isValid()) {
            lv_label_set_text_fmt(
                gpsHdopLabel,
                "HDOP\n%.1f",
                gpsParser.hdop.hdop()
            );
        } else {
            lv_label_set_text(gpsHdopLabel, "HDOP\n--");
        }
    }

    if (gpsCourseLabel) {
        if (haveFix && gpsParser.course.isValid()) {
            lv_label_set_text_fmt(
                gpsCourseLabel,
                "COURSE\n%.0f deg",
                gpsParser.course.deg()
            );
        } else {
            lv_label_set_text(gpsCourseLabel, "COURSE\n-- deg");
        }
    }

    if (gpsUtcLabel) {
        if (gpsParser.date.isValid() && gpsParser.time.isValid()) {
            lv_label_set_text_fmt(
                gpsUtcLabel,
                "UTC  %04u-%02u-%02u   %02u:%02u:%02u",
                gpsParser.date.year(),
                gpsParser.date.month(),
                gpsParser.date.day(),
                gpsParser.time.hour(),
                gpsParser.time.minute(),
                gpsParser.time.second()
            );
        } else {
            lv_label_set_text(
                gpsUtcLabel,
                "UTC  ---- -- --   --:--:--"
            );
        }
    }

    if (gpsDataLabel) {
        if (!haveData) {
            lv_label_set_text(
                gpsDataLabel,
                "UART 38400  |  RX 0 bytes  |  NO DATA"
            );
        } else if (recentData) {
            lv_label_set_text_fmt(
                gpsDataLabel,
                "UART 38400  |  RX %lu bytes  |  DATA LIVE",
                (unsigned long)gpsSessionBytes
            );
        } else {
            const uint32_t staleSeconds =
                (now - gpsLastByteMs) / 1000;

            lv_label_set_text_fmt(
                gpsDataLabel,
                "UART 38400  |  RX %lu bytes  |  STALE %lus",
                (unsigned long)gpsSessionBytes,
                (unsigned long)staleSeconds
            );
        }
    }
}

void gpsTimerCb(lv_timer_t *timer)
{
    LV_UNUSED(timer);
    updateGpsUi();
}

// -----------------------------------------------------------------------------
// NFC-A reader
// -----------------------------------------------------------------------------

#ifdef USING_ST25R3916
const char *nfcStateText(rfalNfcState state)
{
    switch (state) {
    case RFAL_NFC_STATE_NOTINIT: return "NOTINIT";
    case RFAL_NFC_STATE_IDLE: return "IDLE";
    case RFAL_NFC_STATE_START_DISCOVERY: return "START";
    case RFAL_NFC_STATE_WAKEUP_MODE: return "WAKE";
    case RFAL_NFC_STATE_POLL_TECHDETECT: return "TECH";
    case RFAL_NFC_STATE_POLL_COLAVOIDANCE: return "COLL";
    case RFAL_NFC_STATE_POLL_SELECT: return "SELECT";
    case RFAL_NFC_STATE_POLL_ACTIVATION: return "ACTIVATE";
    case RFAL_NFC_STATE_LISTEN_TECHDETECT: return "WAIT";
    case RFAL_NFC_STATE_LISTEN_COLAVOIDANCE: return "L-COLL";
    case RFAL_NFC_STATE_LISTEN_ACTIVATION: return "L-ACT";
    case RFAL_NFC_STATE_LISTEN_SLEEP: return "L-SLEEP";
    case RFAL_NFC_STATE_ACTIVATED: return "ACTIVE";
    case RFAL_NFC_STATE_DATAEXCHANGE: return "XCHG";
    case RFAL_NFC_STATE_DATAEXCHANGE_DONE: return "XCHG DONE";
    case RFAL_NFC_STATE_DEACTIVATION: return "DEACT";
    default: return "OTHER";
    }
}

const char *nfcATagTypeText(rfalNfcaListenDeviceType type)
{
    switch (type) {
    case RFAL_NFCA_T1T: return "NFC-A / TYPE 1";
    case RFAL_NFCA_T2T: return "NFC-A / TYPE 2";
    case RFAL_NFCA_T4T: return "NFC-A / TYPE 4";
    case RFAL_NFCA_NFCDEP: return "NFC-A / NFC-DEP";
    case RFAL_NFCA_T4T_NFCDEP: return "NFC-A / TYPE 4 + NFC-DEP";
    default: return "NFC-A / UNKNOWN";
    }
}

void captureNfcActiveDevice(rfalNfcDevice *device)
{
    if (!device || !device->nfcid || device->nfcidLen == 0) return;

    String uid;
    for (uint8_t i = 0; i < device->nfcidLen; ++i) {
        char octet[4];
        snprintf(octet, sizeof(octet), "%02X", device->nfcid[i]);
        if (i > 0) uid += ':';
        uid += octet;
    }

    String type = "NFC TAG";
    String tech = "TECH DATA UNAVAILABLE";

    // We are the poller, so a discovered card is represented as a LISTEN type.
    if (device->type == RFAL_NFC_LISTEN_TYPE_NFCA) {
        type = nfcATagTypeText(device->dev.nfca.type);

        char techText[48];
        snprintf(
            techText,
            sizeof(techText),
            "ATQA %02X %02X   |   SAK %02X",
            device->dev.nfca.sensRes.anticollisionInfo,
            device->dev.nfca.sensRes.platformInfo,
            device->dev.nfca.selRes.sak
        );
        tech = techText;
    }

    const uint32_t now = millis();
    const bool newPresentation =
        uid != nfcDetectedUid ||
        nfcLastRawDetectionMs == 0 ||
        now - nfcLastRawDetectionMs > 1500;

    nfcLastRawDetectionMs = now;

    if (newPresentation) {
        nfcDetectedUid = uid;
        nfcDetectedType = type;
        nfcDetectedTech = tech;
        ++nfcTagCount;
        nfcTagPending = true;
    }
}

// RFAL notifies us exactly when a target reaches ACTIVATED. Capture the tag
// there, then ask RFAL to restart discovery itself. This follows the working
// T-Watch Ultra NFC reader flow rather than manually chasing the state later.
void nfcRfNotify(rfalNfcState state)
{
    if (!nfcReaderReady || currentPage != Page::NFC) return;

    if (state == RFAL_NFC_STATE_ACTIVATED) {
        rfalNfcDevice *device = nullptr;
        const ReturnCode getResult = NFCReader.rfalNfcGetActiveDevice(&device);

        if (getResult == ST_ERR_NONE && device) {
            captureNfcActiveDevice(device);
        } else {
            Serial.printf(
                "[M33K][NFC] ACTIVATED but active-device read failed: %d\n",
                getResult
            );
        }

        // true tells RFAL to return directly to discovery after deactivation.
        const ReturnCode restartResult = NFCReader.rfalNfcDeactivate(true);
        if (restartResult != ST_ERR_NONE) {
            Serial.printf(
                "[M33K][NFC] automatic discovery restart failed: %d\n",
                restartResult
            );
        }
    }
}

bool beginNfcDiscoveryCycle()
{
    if (!nfcReaderReady || currentPage != Page::NFC) return false;

    const ReturnCode result = NFCReader.rfalNfcDiscover(&nfcDiscoverParams);
    nfcDiscoverCode = static_cast<int>(result);

    if (result != ST_ERR_NONE) {
        nfcDiscoveryActive = false;
        Serial.printf("[M33K][NFC] discovery start failed: %d\n", result);
        return false;
    }

    nfcDiscoveryActive = true;
    nfcDiscoveryStartedMs = millis();
    return true;
}

void updateNfcUiFromWorker()
{
    const uint32_t now = millis();

    if (nfcTagPending) {
        nfcTagPending = false;

        if (nfcStatusLabel) {
            lv_label_set_text(nfcStatusLabel, LV_SYMBOL_OK "  TAG DETECTED");
            lv_obj_set_style_text_color(
                nfcStatusLabel,
                lv_color_hex(0x7CFF45),
                0
            );
        }
        if (nfcTypeLabel) {
            lv_label_set_text(nfcTypeLabel, nfcDetectedType.c_str());
        }
        if (nfcUidLabel) {
            lv_label_set_text_fmt(
                nfcUidLabel,
                "UID\n%s",
                nfcDetectedUid.c_str()
            );
        }
        if (nfcTechLabel) {
            lv_label_set_text(nfcTechLabel, nfcDetectedTech.c_str());
        }
        if (nfcCountLabel) {
            lv_label_set_text_fmt(
                nfcCountLabel,
                "TAGS PRESENTED  %u",
                nfcTagCount
            );
        }

        hapticTap();
        noteActivity();
        Serial.printf(
            "[M33K][NFC] %s  UID %s\n",
            nfcDetectedType.c_str(),
            nfcDetectedUid.c_str()
        );
    }

    if (nfcStatusLabel &&
        nfcLastRawDetectionMs > 0 &&
        now - nfcLastRawDetectionMs > 1400) {
        lv_label_set_text(nfcStatusLabel, "RFAL READY  -  TAP NFC-A TAG");
        lv_obj_set_style_text_color(
            nfcStatusLabel,
            lv_color_hex(0xD0B2FF),
            0
        );
    }

    // Visible diagnostics while no tag has been read yet.
    if (nfcTechLabel &&
        nfcDetectedUid.length() == 0 &&
        now - nfcLastDiagUpdateMs >= 500) {
        nfcLastDiagUpdateMs = now;
        const rfalNfcState state = NFCReader.rfalNfcGetState();
        const int irq = digitalRead(NFC_INT);
        lv_label_set_text_fmt(
            nfcTechLabel,
            "RFAL %s (%d)   |   IRQ %s",
            nfcStateText(state),
            static_cast<int>(state),
            irq ? "HIGH" : "LOW"
        );
    }
}
#endif

bool startNfcTool()
{
    if (nfcTimer) {
        lv_timer_del(nfcTimer);
        nfcTimer = nullptr;
    }

    nfcReaderReady = false;
    nfcTagPending = false;
    nfcDiscoveryActive = false;
    nfcDiscoveryStartedMs = 0;
    nfcNextDiscoveryMs = 0;
    nfcLastRawDetectionMs = 0;
    nfcLastDiagUpdateMs = 0;
    nfcWorkerCalls = 0;
    nfcInitCode = 0;
    nfcDiscoverCode = 0;
    nfcDetectedUid = "";
    nfcDetectedType = "";
    nfcDetectedTech = "";

#ifdef USING_ST25R3916
    // v0.5.4: Follow LilyGO's official T-Watch Ultra NFC_Reader example.
    // instance.begin() already powers/probes the ST25R3916. Do NOT cycle DLDO1
    // here; keep the chip on the same initialized SPI/IRQ path LilyGoLib set up.
    const ReturnCode initResult = NFCReader.rfalNfcInitialize();
    nfcInitCode = static_cast<int>(initResult);

    if (initResult != ST_ERR_NONE) {
        Serial.printf("[M33K][NFC] ST25R3916 init failed: %d\n", initResult);
        return false;
    }

    // Mirror LilyGO examples/peripheral/NFC_Reader/app_nfc.cpp as closely as
    // possible, while zero-initializing the structure for deterministic fields.
    nfcDiscoverParams = {};
    nfcDiscoverParams.compMode = RFAL_COMPLIANCE_MODE_NFC;
    nfcDiscoverParams.devLimit = 1;
    nfcDiscoverParams.techs2Find = RFAL_NFC_POLL_TECH_A;
    nfcDiscoverParams.GBLen = RFAL_NFCDEP_GB_MAX_LEN;
    nfcDiscoverParams.notifyCb = nfcRfNotify;
    nfcDiscoverParams.totalDuration = 1000U;
    nfcDiscoverParams.wakeupEnabled = false;

    nfcReaderReady = true;
    if (!beginNfcDiscoveryCycle()) {
        nfcReaderReady = false;
        return false;
    }

    // RFAL itself is serviced from loop() on every pass. Keep this timer
    // UI-only so LVGL scheduling cannot starve the RF state machine.
    nfcTimer = lv_timer_create(
        [](lv_timer_t *timer) {
            LV_UNUSED(timer);

            if (!nfcReaderReady || currentPage != Page::NFC) return;
            updateNfcUiFromWorker();
        },
        100,
        nullptr
    );

    Serial.printf(
        "[M33K][NFC] RFAL ready init=%d discover=%d CS=%d IRQ=%d\n",
        nfcInitCode,
        nfcDiscoverCode,
        NFC_CS,
        NFC_INT
    );
    return true;
#else
    Serial.println("[M33K][NFC] ST25R3916 support is not compiled in");
    nfcInitCode = -999;
    return false;
#endif
}

void stopNfcTool()
{
    if (nfcTimer) {
        lv_timer_del(nfcTimer);
        nfcTimer = nullptr;
    }

#ifdef USING_ST25R3916
    if (nfcReaderReady) {
        NFCReader.rfalNfcDeactivate(false);
    }
    // Keep POWER_NFC enabled. LilyGO's official reader leaves the ST25R3916
    // powered after instance.begin(); re-entry will reinitialize RFAL cleanly.
#endif

    nfcReaderReady = false;
    nfcTagPending = false;
    nfcDiscoveryActive = false;
    nfcDiscoveryStartedMs = 0;
    nfcNextDiscoveryMs = 0;
}

// -----------------------------------------------------------------------------
// Page cleanup
// -----------------------------------------------------------------------------

void stopWifiTool()
{
    if (wifiTimer) {
        lv_timer_del(wifiTimer);
        wifiTimer = nullptr;
    }
    if (wifiMascotTimer) {
        lv_timer_del(wifiMascotTimer);
        wifiMascotTimer = nullptr;
    }
    wifiMascot = nullptr;

    if (wifiScanning || WiFi.scanComplete() >= 0) {
        WiFi.scanDelete();
    }

    wifiScanning = false;
    wifiGraphLive = false;
    wifiGraphNextScanMs = 0;
    wifiInitialC5LinkPending = false;
    resetWifiGraphDrops();

    // Page navigation must never change the clock and must not tear down
    // a deliberate Wi-Fi connection. Wi-Fi is only turned off here when
    // there is no active connection and Stay Connected is disabled.
    if (!wifiSubpageTransition &&
        WiFi.status() != WL_CONNECTED &&
        !wifiStayConnected) {
        WiFi.mode(WIFI_OFF);
    }
}

void clearPageTimers()
{
    if (matrixTimer) {
        lv_timer_del(matrixTimer);
        matrixTimer = nullptr;
    }

    if (faceAnimTimer) {
        lv_timer_del(faceAnimTimer);
        faceAnimTimer = nullptr;
    }

    if (homeLiveTimer) {
        lv_timer_del(homeLiveTimer);
        homeLiveTimer = nullptr;
    }

    if (reconTimer) {
        lv_timer_del(reconTimer);
        reconTimer = nullptr;
    }
    if (watchTimer) {
        lv_timer_del(watchTimer);
        watchTimer = nullptr;
    }
    if (wardriveTimer) {
        lv_timer_del(wardriveTimer);
        wardriveTimer = nullptr;
    }
    if (mapTimer) {
        lv_timer_del(mapTimer);
        mapTimer = nullptr;
    }
    if (radioTimer) {
        lv_timer_del(radioTimer);
        radioTimer = nullptr;
    }
    if (bleAnimTimer) {
        lv_timer_del(bleAnimTimer);
        bleAnimTimer = nullptr;
    }
    if (gameTimer) {
        lv_timer_del(gameTimer);
        gameTimer = nullptr;
    }
    stopGameTool();
    reconPhase = ReconPhase::Idle;

    stopWatchTool();
    stopWardriveTool();
    stopRadioTool();
    stopNfcTool();
    stopReconPulseTool();
    stopRadarTool();
    stopHunterTool();

    stopWifiTool();
    stopBleTool();
    stopGpsTool();
}

void resetLivePointers()
{
    batteryLabel = nullptr;
    pageBatteryLabel = nullptr;
    homeWifiIcon = nullptr;
    homeWifiDot = nullptr;
    homeC5Label = nullptr;
    homeC5Dot = nullptr;
    hourLabel = nullptr;
    colonLabel = nullptr;
    minuteLabel = nullptr;
    dateTopLabel = nullptr;
    dateBottomLabel = nullptr;

    for (auto &p : hourGlow) p = nullptr;
    for (auto &p : colonGlow) p = nullptr;
    for (auto &p : minuteGlow) p = nullptr;
    for (auto &p : dateTopGlow) p = nullptr;
    for (auto &p : dateBottomGlow) p = nullptr;

    wifiStatusLabel = nullptr;
    wifiList = nullptr;
    wifiGraphContainer = nullptr;
    wifiGraphLegend = nullptr;
    wifiGraphView = false;
    wifiDetailOverlay = nullptr;
    wifiDetailTitle = nullptr;
    wifiDetailBody = nullptr;

    bleStatusLabel = nullptr;
    bleList = nullptr;
    bleCat = nullptr;
    blePingRing = nullptr;
    bleDetailOverlay = nullptr;
    bleDetailTitle = nullptr;
    bleDetailBody = nullptr;
    bleDetailCount = 0;


    gpsStatusLabel = nullptr;
    gpsSatLabel = nullptr;
    gpsLatLabel = nullptr;
    gpsLonLabel = nullptr;
    gpsAltLabel = nullptr;
    gpsSpeedLabel = nullptr;
    gpsHdopLabel = nullptr;
    gpsCourseLabel = nullptr;
    gpsUtcLabel = nullptr;
    gpsDataLabel = nullptr;

    mapStatusLabel = nullptr;
    mapCoordLabel = nullptr;
    mapZoomLabel = nullptr;
    mapSdLabel = nullptr;
    mapMarkerLabel = nullptr;
    mapMarkerDot = nullptr;
    mapViewport = nullptr;
    mapTileCanvas = nullptr;
    mapTilePlaceholder = nullptr;
    for (auto &label : mapVectorLabels) label = nullptr;
    for (auto &line : mapGridVertical) line = nullptr;
    for (auto &line : mapGridHorizontal) line = nullptr;

    nfcStatusLabel = nullptr;
    nfcTypeLabel = nullptr;
    nfcUidLabel = nullptr;
    nfcTechLabel = nullptr;
    nfcCountLabel = nullptr;

    reconStatusLabel = nullptr;
    reconWifiCountLabel = nullptr;
    reconWifiStrongLabel = nullptr;
    reconChannelLabel = nullptr;
    reconBleCountLabel = nullptr;
    reconBleStrongLabel = nullptr;
    reconGpsLabel = nullptr;
    reconSatLabel = nullptr;

    watchStatusLabel = nullptr;
    watchWifiLabel = nullptr;
    watchBleLabel = nullptr;
    watchAlertCountLabel = nullptr;
    watchAlertList = nullptr;

    wardriveStatusLabel = nullptr;
    wardriveWifiLabel = nullptr;
    wardriveBleLabel = nullptr;
    wardriveGpsLabel = nullptr;
    wardriveElapsedLabel = nullptr;
    wardriveLocationLabel = nullptr;
    wardriveFeedScroll = nullptr;
    wardriveFeedLabel = nullptr;
    wardriveFeedStateLabel = nullptr;
    for (auto &row : wardriveFeedRows) row = nullptr;
    wardriveFeedPageLabel = nullptr;
    wardriveStartLabel = nullptr;
    wardriveSdLabel = nullptr;
    wardriveDetailOverlay = nullptr;
    wardriveDetailTitle = nullptr;
    wardriveDetailBody = nullptr;

    radioStatusLabel = nullptr;
    radioChipLabel = nullptr;
    radioFreqLabel = nullptr;
    radioModeLabel = nullptr;
    radioPacketLabel = nullptr;
    radioSignalLabel = nullptr;
    radioLastLabel = nullptr;
    radioProfileLabel = nullptr;
    radioPayloadLabel = nullptr;
    radioListenLabel = nullptr;
    radioSfButtonLabel = nullptr;
    radioBwButtonLabel = nullptr;
    radioAutoButtonLabel = nullptr;
    radioMeshButtonLabel = nullptr;

    radarStatusLabel = nullptr;
    radarInfoLabel = nullptr;
    for (auto &p : radarSweepDots) p = nullptr;

    pulseStatusLabel = nullptr;
    pulseInfoLabel = nullptr;
    pulseNewLabel = nullptr;
    pulseLostLabel = nullptr;
    pulseMoveLabel = nullptr;
    for (auto &p : pulseGraphNewBars) p = nullptr;
    for (auto &p : pulseGraphLostBars) p = nullptr;
    for (auto &p : pulseGraphMoveBars) p = nullptr;
    for (auto &p : pulseSweepDots) p = nullptr;

    hunterStatusLabel = nullptr;
    hunterList = nullptr;
    hunterTrackingPanel = nullptr;
    hunterTargetLabel = nullptr;
    hunterRssiLabel = nullptr;
    hunterStrengthLabel = nullptr;
    hunterSignalBar = nullptr;
    hunterModeLabel = nullptr;
    for (auto &p : hunterHistoryBars) p = nullptr;

    settingsStatusLabel = nullptr;
    settingsTzDropdown = nullptr;
    settingsFormatDropdown = nullptr;
    settingsSsidInput = nullptr;
    settingsWifiStateLabel = nullptr;
    settingsStaySwitch = nullptr;
    settingsPasswordInput = nullptr;
    settingsPasswordCountLabel = nullptr;
    settingsKeyboard = nullptr;
    settingsKeyboardShade = nullptr;
    settingsContent = nullptr;
    settingsAvailableDropdown = nullptr;
    settingsShowPasswordSwitch = nullptr;
    settingsBrightnessSlider = nullptr;
    settingsBrightnessValueLabel = nullptr;
    availableSsidCount = 0;

    manualStatusLabel = nullptr;
    manualYearSpin = nullptr;
    manualMonthSpin = nullptr;
    manualDaySpin = nullptr;
    manualHourSpin = nullptr;
    manualMinuteSpin = nullptr;
}

void clearScreen()
{
    clearPageTimers();

    // Settings keyboard/shade live on lv_layer_top(), not on the page screen.
    // Delete them BEFORE resetLivePointers() so reopening Settings cannot leave
    // an invisible old keyboard intercepting touches over SSID/password fields.
    if (settingsKeyboard) {
        lv_keyboard_set_textarea(settingsKeyboard, nullptr);
        lv_obj_del(settingsKeyboard);
        settingsKeyboard = nullptr;
    }
    if (settingsKeyboardShade) {
        lv_obj_del(settingsKeyboardShade);
        settingsKeyboardShade = nullptr;
    }

    resetLivePointers();

    if (!screen) screen = lv_screen_active();

    lv_obj_clean(screen);

    if (mapTileBuffer) {
        heap_caps_free(mapTileBuffer);
        mapTileBuffer = nullptr;
    }
    mapTileLoaded = false;
    mapLoadedTileX = -1;
    mapLoadedTileY = -1;
    mapLoadedTileZ = -1;
    mapRequestedTileX = -1;
    mapRequestedTileY = -1;
    mapRequestedTileZ = -1;
    mapLoadedWorldX = -1;
    mapLoadedWorldY = -1;
    mapRequestedWorldX = -1;
    mapRequestedWorldY = -1;
    mapTileReloadPending = false;

    lv_obj_set_style_bg_color(screen, lv_color_hex(0x030711), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
}

// -----------------------------------------------------------------------------
// Home screen
// -----------------------------------------------------------------------------

lv_obj_t *createTimeGlowLabel(const char *txt, int x, int y, uint8_t opa)
{
    lv_obj_t *g = lv_label_create(screen);
    lv_label_set_text(g, txt);
    lv_obj_set_style_text_color(g, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_opa(g, opa, 0);
    lv_obj_set_style_text_font(g, &lv_font_montserrat_44, 0);
    lv_obj_set_pos(g, x, y);
    return g;
}

void createTimeGlowSet(lv_obj_t **out, const char *txt, int x, int y)
{
    struct GlowOffset {
        int8_t dx;
        int8_t dy;
        uint8_t opacity;
    };

    static const GlowOffset offsets[16] = {
        {-3,  0, 150}, { 3,  0, 150}, { 0, -3, 150}, { 0,  3, 150},
        {-2, -2, 175}, { 2, -2, 175}, {-2,  2, 175}, { 2,  2, 175},
        {-1,  0, 210}, { 1,  0, 210}, { 0, -1, 210}, { 0,  1, 210},
        {-1, -1, 225}, { 1, -1, 225}, {-1,  1, 225}, { 1,  1, 225}
    };

    for (uint8_t i = 0; i < 16; ++i) {
        out[i] = createTimeGlowLabel(txt,
                                     x + offsets[i].dx,
                                     y + offsets[i].dy,
                                     offsets[i].opacity);
    }
}

lv_obj_t *createDateGlowLabel(const char *txt, int x, int y,
                              const lv_font_t *font, int dx, int dy, uint8_t opa)
{
    lv_obj_t *g = lv_label_create(screen);
    lv_label_set_text(g, txt);
    lv_obj_set_style_text_color(g, lv_color_hex(0x7CFF45), 0);
    lv_obj_set_style_text_opa(g, opa, 0);
    lv_obj_set_style_text_font(g, font, 0);
    lv_obj_set_pos(g, x + dx, y + dy);
    return g;
}

void createDateGlowSet(lv_obj_t **out, const char *txt, int x, int y,
                       const lv_font_t *font)
{
    out[0] = createDateGlowLabel(txt, x, y, font,  1,  1, 110);
    out[1] = createDateGlowLabel(txt, x, y, font, -1, -1,  90);
    out[2] = createDateGlowLabel(txt, x, y, font,  2,  2,  70);
    out[3] = createDateGlowLabel(txt, x, y, font, -2, -2,  55);
}

void createTopBar()
{
    lv_obj_t *top = lv_obj_create(screen);
    lv_obj_remove_style_all(top);
    lv_obj_set_size(top, 328, 30);
    lv_obj_set_pos(top, 41, 10);
    lv_obj_set_style_radius(top, 14, 0);
    lv_obj_set_style_bg_color(top, lv_color_hex(0x07101D), 0);
    lv_obj_set_style_bg_opa(top, 175, 0);
    lv_obj_set_style_border_width(top, 1, 0);
    lv_obj_set_style_border_color(top, lv_color_hex(0x263A52), 0);

    batteryLabel = lv_label_create(top);
    lv_label_set_text(batteryLabel, LV_SYMBOL_BATTERY_FULL " --");
    lv_obj_set_style_text_color(batteryLabel, lv_color_hex(0x7CFF45), 0);
    lv_obj_set_style_text_font(batteryLabel, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(batteryLabel, 24, 7);

    // On the terminal face the decorative top icons (separators, BLE, GPS) take
    // the terminal readout color; battery stays green and Wi-Fi/C5 keep their
    // live status colors.
    const uint32_t topIcon = dockIconColorOverride ? dockIconColorOverride : 0x6B8299;
    const uint32_t bleCol  = dockIconColorOverride ? dockIconColorOverride : 0x6D86FF;
    const uint32_t gpsCol  = dockIconColorOverride ? dockIconColorOverride : 0xF5FF3B;

    lv_obj_t *sep1 = lv_label_create(top);
    lv_label_set_text(sep1, "|");
    lv_obj_set_style_text_color(sep1, lv_color_hex(topIcon), 0);
    lv_obj_set_pos(sep1, 95, 7);

    homeWifiIcon = lv_label_create(top);
    lv_label_set_text(homeWifiIcon, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(homeWifiIcon, lv_color_hex(0x526A82), 0);
    lv_obj_set_style_text_font(homeWifiIcon, &lv_font_montserrat_16, 0);
    lv_obj_set_pos(homeWifiIcon, 112, 6);

    homeWifiDot = lv_obj_create(top);
    lv_obj_remove_style_all(homeWifiDot);
    lv_obj_set_size(homeWifiDot, 7, 7);
    lv_obj_set_pos(homeWifiDot, 133, 7);
    lv_obj_set_style_radius(homeWifiDot, 4, 0);
    lv_obj_set_style_bg_color(homeWifiDot, lv_color_hex(0x526A82), 0);
    lv_obj_set_style_bg_opa(homeWifiDot, LV_OPA_COVER, 0);

    lv_obj_t *sep2 = lv_label_create(top);
    lv_label_set_text(sep2, "|");
    lv_obj_set_style_text_color(sep2, lv_color_hex(topIcon), 0);
    lv_obj_set_pos(sep2, 147, 7);

    homeC5Label = lv_label_create(top);
    lv_label_set_text(homeC5Label, "C5");
    lv_obj_set_style_text_color(homeC5Label, lv_color_hex(0x526A82), 0);
    lv_obj_set_style_text_font(homeC5Label, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(homeC5Label, 162, 8);

    homeC5Dot = lv_obj_create(top);
    lv_obj_remove_style_all(homeC5Dot);
    lv_obj_set_size(homeC5Dot, 7, 7);
    lv_obj_set_pos(homeC5Dot, 183, 7);
    lv_obj_set_style_radius(homeC5Dot, 4, 0);
    lv_obj_set_style_bg_color(homeC5Dot, lv_color_hex(0x526A82), 0);
    lv_obj_set_style_bg_opa(homeC5Dot, LV_OPA_COVER, 0);

    lv_obj_t *sep3 = lv_label_create(top);
    lv_label_set_text(sep3, "|");
    lv_obj_set_style_text_color(sep3, lv_color_hex(topIcon), 0);
    lv_obj_set_pos(sep3, 197, 7);

    lv_obj_t *ble = lv_label_create(top);
    lv_label_set_text(ble, LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_color(ble, lv_color_hex(bleCol), 0);
    lv_obj_set_style_text_font(ble, &lv_font_montserrat_16, 0);
    lv_obj_set_pos(ble, 214, 6);

    lv_obj_t *sep4 = lv_label_create(top);
    lv_label_set_text(sep4, "|");
    lv_obj_set_style_text_color(sep4, lv_color_hex(topIcon), 0);
    lv_obj_set_pos(sep4, 246, 7);

    lv_obj_t *gps = lv_label_create(top);
    lv_label_set_text(gps, LV_SYMBOL_GPS);
    lv_obj_set_style_text_color(gps, lv_color_hex(gpsCol), 0);
    lv_obj_set_style_text_font(gps, &lv_font_montserrat_16, 0);
    lv_obj_set_pos(gps, 264, 6);
}

void createTimeDate()
{
    // Fixed right edge for the hour fixes proportional-font spacing.
    // "11" is narrower than "10"/"12", but every hour now ends at x=138.
    // Keep the same right edge, but give two wide digits (especially "08")
    // enough horizontal room so LVGL never wraps the second digit below.
    const int hourX = 68;
    const int hourW = 70;
    const int hourY = 50;
    const int colonX = 141;
    const int minuteX = 153;
    const int dateX = 266;

    createTimeGlowSet(hourGlow, "10", hourX, hourY);
    createTimeGlowSet(colonGlow, ":", colonX, hourY);
    createTimeGlowSet(minuteGlow, "24", minuteX, hourY);

    for (auto *g : hourGlow) {
        if (!g) continue;
        lv_obj_set_width(g, hourW);
        lv_obj_set_style_text_align(g, LV_TEXT_ALIGN_RIGHT, 0);
    }

    hourLabel = lv_label_create(screen);
    lv_label_set_text(hourLabel, "10");
    lv_obj_set_width(hourLabel, hourW);
    lv_obj_set_style_text_align(hourLabel, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_color(hourLabel, lv_color_hex(0x3AEFFF), 0);
    lv_obj_set_style_text_font(hourLabel, &lv_font_montserrat_44, 0);
    lv_obj_set_pos(hourLabel, hourX, hourY);

    colonLabel = lv_label_create(screen);
    lv_label_set_text(colonLabel, ":");
    lv_obj_set_style_text_color(colonLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(colonLabel, &lv_font_montserrat_44, 0);
    lv_obj_set_pos(colonLabel, colonX, hourY);

    minuteLabel = lv_label_create(screen);
    lv_label_set_text(minuteLabel, "24");
    lv_obj_set_style_text_color(minuteLabel, lv_color_hex(0xFF52C8), 0);
    lv_obj_set_style_text_font(minuteLabel, &lv_font_montserrat_44, 0);
    lv_obj_set_pos(minuteLabel, minuteX, hourY);

    createDateGlowSet(dateTopGlow, "MAY 12", dateX, 56, &lv_font_montserrat_18);
    createDateGlowSet(dateBottomGlow, "MONDAY", dateX, 82, &lv_font_montserrat_16);

    dateTopLabel = lv_label_create(screen);
    lv_label_set_text(dateTopLabel, "MAY 12");
    lv_obj_set_style_text_color(dateTopLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(dateTopLabel, &lv_font_montserrat_18, 0);
    lv_obj_set_pos(dateTopLabel, dateX, 56);

    dateBottomLabel = lv_label_create(screen);
    lv_label_set_text(dateBottomLabel, "MONDAY");
    lv_obj_set_style_text_color(dateBottomLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(dateBottomLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_pos(dateBottomLabel, dateX, 82);
}

void createArt()
{
    lv_obj_t *img = lv_image_create(screen);
    lv_image_set_src(img, &m33k_peek_img);
    lv_obj_set_pos(img, 110, 110);
}

// -----------------------------------------------------------------------------
// Navigation events
// -----------------------------------------------------------------------------

void showHome();
void showWifiPage();
void showWifiGraphPage();
void showBlePage();
void showReconPage();
void showRadarPage();
void showSignalHunterPage();
void showWatchModePage();
void showWardrivePage();
void showGpsPage();
void showMapPage();
void showNfcPage();
void showLogsPage();
void showSettingsPage();
void showManualTimePage();
void showPlaceholder(Page page, const char *title);

void hideSettingsKeyboard()
{
    if (settingsKeyboard) {
        lv_keyboard_set_textarea(
            settingsKeyboard,
            nullptr
        );

        lv_obj_add_flag(
            settingsKeyboard,
            LV_OBJ_FLAG_HIDDEN
        );
    }

    if (settingsKeyboardShade) {
        lv_obj_add_flag(
            settingsKeyboardShade,
            LV_OBJ_FLAG_HIDDEN
        );
    }
}

void performHardwareBack()
{
    if (screenSleeping) {
        wakeScreen();
        return;
    }

    noteActivity();
    hapticTap();

    // Keyboard gets first priority: Back closes it instead of leaving Settings.
    if (settingsKeyboard &&
        !lv_obj_has_flag(
            settingsKeyboard,
            LV_OBJ_FLAG_HIDDEN)) {

        hideSettingsKeyboard();
        return;
    }

    if (currentPage == Page::Home) {
        return;
    }

    if (currentPage == Page::Wardrive) {
        wardriveExitToReconPending = true;
        wardrivePageActive = false;
        wardriveRunning = false;
        wardriveUiSyncPending = false;
        return;
    }

    if (currentPage == Page::WiFi) {
        wifiGraphLive = false;
        wifiSubpageTransition = false;
        wifiPendingNav = WifiPendingNav::ExitHome;
        return;
    }

    if (currentPage == Page::Recon) {
        reconExitHomePending = true;
        return;
    }

    if (currentPage == Page::Map) {
        showGpsPage();
        return;
    }

    if (currentPage == Page::ReconPulse ||
        currentPage == Page::Radar ||
        currentPage == Page::SignalHunter ||
        currentPage == Page::WatchMode ||
        currentPage == Page::Radio) {

        showReconPage();
        return;
    }

    if (currentPage == Page::Stopwatch) {
        if (stopwatchTimer) {
            lv_timer_delete(stopwatchTimer);
            stopwatchTimer = nullptr;
        }
        showSettingsPage();
        return;
    }

    // Manual Time reuses Page::Settings. Its spinbox pointers distinguish it.
    if (currentPage == Page::Settings &&
        manualYearSpin != nullptr) {

        showSettingsPage();
        return;
    }

    showHome();
}

void handleHardwareBackButton()
{
    const bool down =
        digitalRead(
            M33K_BACK_BUTTON_PIN
        ) == LOW;

    const uint32_t now =
        millis();

    if (down != backButtonWasDown &&
        now - backButtonLastChangeMs >=
            BACK_BUTTON_DEBOUNCE_MS) {

        backButtonLastChangeMs = now;
        backButtonWasDown = down;

        // Navigate on press, not release.
        if (down) {
            performHardwareBack();
        }
    }
}


void backEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();

    if (currentPage == Page::WiFi) {
        wifiGraphLive = false;
        wifiSubpageTransition = false;
        wifiPendingNav = WifiPendingNav::ExitHome;
        return;
    }

    if (currentPage == Page::Recon) {
        reconExitHomePending = true;
        return;
    }

    showHome();
}

void dockEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    auto *item = static_cast<DockItem *>(lv_event_get_user_data(e));
    if (!item) return;

    noteActivity();
    hapticTap();

    Serial.printf("[M33K] Open: %s\n", item->label);

    if (item->page == Page::WiFi) {
        showWifiPage();
    } else if (item->page == Page::BLE) {
        showBlePage();
    } else if (item->page == Page::Recon) {
        showReconPage();
    } else if (item->page == Page::GPS) {
        showGpsPage();
    } else if (item->page == Page::NFC) {
        showNfcPage();
    } else if (item->page == Page::Logs) {
        showLogsPage();
    } else if (item->page == Page::Settings) {
        showSettingsPage();
    } else if (item->page == Page::Game) {
        showGamePage();
    } else {
        showPlaceholder(item->page, item->label);
    }
}

void createDock()
{
    // Larger hit areas than v0.2.x, while staying away from rounded corners.
    // Sized to fit eight icons (Game added) across the 410 px screen.
    constexpr int16_t buttonW = 40;
    constexpr int16_t buttonH = 54;
    constexpr int16_t gap = 6;
    int16_t x = 24;

    for (const auto &item : dock) {
        lv_obj_t *btn = lv_button_create(screen);
        lv_obj_set_size(btn, buttonW, buttonH);
        lv_obj_set_pos(btn, x, DOCK_Y);
        lv_obj_set_style_radius(btn, 17, 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x091220), 0);
        lv_obj_set_style_bg_opa(btn, 182, 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_border_color(btn, lv_color_hex(0x1A3456), 0);

        // Gives each icon another 5 px of touch area on every side.
        lv_obj_set_ext_click_area(btn, 5);
        lv_obj_add_event_cb(btn, dockEvent, LV_EVENT_CLICKED,
                            const_cast<DockItem *>(&item));

        lv_obj_t *icon = lv_label_create(btn);
        lv_label_set_text(icon, item.icon);
        const uint32_t iconCol = dockIconColorOverride ? dockIconColorOverride
                                                       : item.iconColorHex;
        lv_obj_set_style_text_color(icon, lv_color_hex(iconCol), 0);
        lv_obj_set_style_text_font(icon, &lv_font_montserrat_22, 0);
        lv_obj_center(icon);

        x += buttonW + gap;
    }
}

// Refresh the live clock on the alternate faces (the Original face keeps using
// updateHomeLiveData()). Safe to call on any face; null pointers are skipped.
void updateFaceClock()
{
    if (homeFaceIndex == 0) return;

    RTC_DateTime dt = instance.rtc.getDateTime();
    uint8_t h24 = dt.getHour();
    uint8_t dh = h24;
    if (!use24Hour) { dh = h24 % 12; if (dh == 0) dh = 12; }

    char hb[4], mb[4], sb[4];
    if (use24Hour) snprintf(hb, sizeof(hb), "%02u", dh);
    else           snprintf(hb, sizeof(hb), "%u", dh);
    snprintf(mb, sizeof(mb), "%02u", dt.getMinute());
    snprintf(sb, sizeof(sb), "%02u", dt.getSecond());

    uint8_t mon = dt.getMonth(); if (mon < 1 || mon > 12) mon = 1;
    struct tm rt = dt.toUnixTime();
    uint8_t wk = static_cast<uint8_t>(rt.tm_wday); if (wk > 6) wk = 0;

    if (faceClockHour) lv_label_set_text(faceClockHour, hb);
    if (faceClockMin)  lv_label_set_text(faceClockMin, mb);
    if (faceClockDate)
        lv_label_set_text_fmt(faceClockDate, "%s  %s %u", WEEKDAYS[wk],
                              MONTHS[mon - 1], dt.getDay());
    if (termTime) lv_label_set_text_fmt(termTime, "%s:%s:%s", hb, mb, sb);
    if (termDate)
        lv_label_set_text_fmt(termDate, "%s %02u-%02u", WEEKDAYS[wk], mon,
                              dt.getDay());
    if (termBatt) {
        int pct = instance.pmu.getBatteryPercent();
        if (pct >= 0) lv_label_set_text_fmt(termBatt, "%d%%  NET UP", pct);
        else          lv_label_set_text(termBatt, "--   NET UP");
    }
}

// 7-segment masks for digits 0-9 (bit i = segment i in order a,b,c,d,e,f,g).
static const uint8_t RADAR_SEG7[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

// Light/dim one digit cell's 7 segments for a value (-1 = blank).
static void radarSetDigit(int cell, int value)
{
    if (cell < 0 || cell > 3) return;
    uint8_t mask = (value >= 0 && value <= 9) ? RADAR_SEG7[value] : 0x00;
    for (int i = 0; i < 7; ++i) {
        lv_obj_t *s = radarSeg[cell][i];
        if (!s) continue;
        const bool on = (mask >> i) & 1;
        lv_obj_set_style_bg_color(s, lv_color_hex(on ? RADAR_NEON : 0x0B3D12), 0);
        lv_obj_set_style_bg_opa(s, on ? LV_OPA_COVER : 90, 0);
        lv_obj_set_style_shadow_opa(s, on ? 180 : 0, 0);
    }
}

// Refresh the Radar face's 7-seg time and the vertical day/date.
void updateRadarClock()
{
    RTC_DateTime dt = instance.rtc.getDateTime();
    uint8_t h24 = dt.getHour(), dh = h24;
    if (!use24Hour) { dh = h24 % 12; if (dh == 0) dh = 12; }
    int htens = dh / 10, hones = dh % 10;
    if (!use24Hour && htens == 0) htens = -1;   // blank the leading zero in 12h
    radarSetDigit(0, htens);
    radarSetDigit(1, hones);
    radarSetDigit(2, dt.getMinute() / 10);
    radarSetDigit(3, dt.getMinute() % 10);

    uint8_t mon = dt.getMonth(); if (mon < 1 || mon > 12) mon = 1;
    struct tm rt = dt.toUnixTime();
    uint8_t wk = static_cast<uint8_t>(rt.tm_wday); if (wk > 6) wk = 0;
    if (radarMetaDay)  lv_label_set_text(radarMetaDay, WEEKDAYS[wk]);
    if (radarMetaDate) lv_label_set_text_fmt(radarMetaDate, "%s %u", MONTHS[mon - 1], dt.getDay());
}

// 50 ms animation driver for the alternate faces (cursor blink, character
// sway/tail twitch, video flipbook). Torn down in clearPageTimers().
void buildWeatherFace();
void serviceWeatherFetch();

// Weather scenes, chosen from the WMO weather_code so the backdrop matches the
// real conditions instead of a timed loop.
enum WxScene : uint8_t { WX_CLEAR, WX_CLOUD, WX_FOG, WX_RAIN, WX_SNOW, WX_STORM };
static WxScene wxSceneForCode(int code)
{
    if (code < 0)                         return WX_CLEAR;   // no data yet
    if (code == 0 || code == 1)           return WX_CLEAR;   // clear / mainly clear
    if (code == 2 || code == 3)           return WX_CLOUD;   // partly cloudy / overcast
    if (code == 45 || code == 48)         return WX_FOG;     // fog
    if ((code >= 51 && code <= 67) ||
        (code >= 80 && code <= 82))       return WX_RAIN;    // drizzle / rain / showers
    if ((code >= 71 && code <= 77) ||
         code == 85 || code == 86)        return WX_SNOW;    // snow
    if (code >= 95)                       return WX_STORM;   // thunderstorm
    return WX_CLOUD;
}

void faceAnimTick(lv_timer_t *timer)
{
    LV_UNUSED(timer);
    if (currentPage != Page::Home) return;
    const uint32_t now = millis();
    const float ph = (now - faceAnimStartMs) / 1000.0f;

    if (homeFaceIndex == 1) {
        if (termCursor && (now - termCursorMs >= 500)) {
            termCursorMs = now; termCursorOn = !termCursorOn;
            lv_obj_set_style_opa(termCursor, termCursorOn ? 255 : 0, 0);
        }
    } else if (homeFaceIndex == 2) {
        if (crewMain) lv_image_set_rotation(crewMain,
            static_cast<int32_t>(sinf(ph * 1.5f) * 16.0f));
        const int catBob = static_cast<int>(sinf(ph * 1.1f) * 3.0f);
        const int monBob = static_cast<int>(sinf(ph * 1.3f + 1.0f) * 3.0f);
        if (crewCatBody) lv_obj_set_y(crewCatBody, crewCatBaseY + catBob);
        if (crewCatTail) {
            lv_obj_set_y(crewCatTail, crewCatBaseY + catBob);
            lv_image_set_rotation(crewCatTail,
                static_cast<int32_t>(sinf(ph * 3.0f) * 70.0f));
        }
        if (crewMonBody) lv_obj_set_y(crewMonBody, crewMonBaseY + monBob);
        if (crewMonTail) {
            lv_obj_set_y(crewMonTail, crewMonBaseY + monBob);
            lv_image_set_rotation(crewMonTail,
                static_cast<int32_t>(sinf(ph * 2.4f + 2.0f) * 60.0f));
        }
    } else if (homeFaceIndex == 3) {
        if (videoImg && m33k_video_frame_count > 0 &&
            (now - videoFrameMs >= 130)) {
            videoFrameMs = now;
            videoFrameIdx = (videoFrameIdx + 1) % m33k_video_frame_count;
            lv_image_set_src(videoImg, m33k_video_frames[videoFrameIdx]);
        }
    } else if (homeFaceIndex == 4) {
        // Background bubbles drift down and wrap at the top.
        for (int i = 0; i < BG_BUBBLE_COUNT; ++i) {
            BubbleEntity &b = gBgBubbles[i];
            if (!b.obj) continue;
            b.y += b.speed;
            b.phase += 0.02f;
            if (b.y > 502.0f + b.size) { b.y = -(float)b.size; b.x = rand() % 410; }
            lv_obj_set_pos(b.obj, (int)b.x + (int)(sinf(b.phase) * b.swayAmp), (int)b.y);
        }
        // Character bubbles float up and wrap at the bottom.
        for (int i = 0; i < BUBBLE_COUNT; ++i) {
            BubbleEntity &b = gBubbles[i];
            if (!b.obj) continue;
            b.y -= b.speed;
            b.phase += 0.016f;   // gentle sway, scaled for the 33 ms tick
            if (b.y < -(float)b.size - 4.0f) { bubbleRespawn(i, false); continue; }
            lv_obj_set_pos(b.obj, (int)b.x + (int)(sinf(b.phase) * b.swayAmp), (int)b.y);
        }
        // Flashy clock: cycle the hour / minute / date through neon hues.
        const uint16_t h1 = (uint16_t)((now / 12) % 360);
        const uint16_t h2 = (uint16_t)((h1 + 140) % 360);
        if (faceClockHour)  lv_obj_set_style_text_color(faceClockHour,  lv_color_hsv_to_rgb(h1, 100, 100), 0);
        if (faceClockMin)   lv_obj_set_style_text_color(faceClockMin,   lv_color_hsv_to_rgb(h2, 100, 100), 0);
        if (faceClockColon) lv_obj_set_style_text_color(faceClockColon, lv_color_hex(0xFFFFFF), 0);
        if (faceClockDate)  lv_obj_set_style_text_color(faceClockDate,  lv_color_hsv_to_rgb(h1, 80, 100), 0);
    } else if (homeFaceIndex == 5) {
        // Weather backdrop driven by the real condition (WMO weather_code)
        // plus day/night, instead of a timed storm->sun loop.
        const WxScene scene = wxSceneForCode(weatherCode);
        // Day/night from the RTC, cached (no need to hit I2C 30x/second).
        static bool wxIsDayCached = true;
        static uint32_t wxDayCheckMs = 0;
        if (wxDayCheckMs == 0 || now - wxDayCheckMs > 5000) {
            wxDayCheckMs = now;
            const int hr = (int)instance.rtc.getDateTime().getHour();
            wxIsDayCached = (hr >= 6 && hr < 19);
        }
        const bool isDay = wxIsDayCached;
        // Wind scales drift/fall speed (1.0 calm ... ~2.6 very gusty).
        const float windF = 1.0f + (weatherWindMph > 6.0f
                            ? fminf((weatherWindMph - 6.0f) / 16.0f, 1.6f) : 0.0f);

        const bool showSun    = (scene == WX_CLEAR && isDay);
        const bool showMoon   = (scene == WX_CLEAR && !isDay);
        const bool showBird   = (scene == WX_CLEAR && isDay);
        const bool showClouds = (scene == WX_CLOUD || scene == WX_RAIN ||
                                 scene == WX_SNOW  || scene == WX_STORM ||
                                 (scene == WX_CLEAR && isDay));
        const bool showRain   = (scene == WX_RAIN || scene == WX_STORM);
        const bool showSnow   = (scene == WX_SNOW);
        const bool showFog    = (scene == WX_FOG);
        const bool showStorm  = (scene == WX_STORM);
        const bool showCity   = (scene == WX_STORM || scene == WX_RAIN);
        const bool showMtn    = (scene == WX_CLEAR || scene == WX_CLOUD ||
                                 scene == WX_FOG   || scene == WX_SNOW);

        // Sky colour: bright day-blue when clear, progressively darker/greyer
        // for cloud, fog, rain, snow and storm, and darker still at night.
        uint32_t sky;
        switch (scene) {
            case WX_CLEAR: sky = isDay ? 0x2E7CC8 : 0x0A1430; break;
            case WX_CLOUD: sky = isDay ? 0x4A6378 : 0x11203A; break;
            case WX_FOG:   sky = isDay ? 0x6E7E8C : 0x1A2230; break;
            case WX_RAIN:  sky = isDay ? 0x37485C : 0x0D1726; break;
            case WX_SNOW:  sky = isDay ? 0x5A6E86 : 0x14203A; break;
            default:       sky = 0x0A1024; break;   // storm
        }
        if (wxBg) lv_obj_set_style_bg_color(wxBg, lv_color_hex(sky), 0);

        // Sun (clear day): soft glow pulse.
        if (wxSun) {
            lv_obj_set_style_opa(wxSun, showSun ? 255 : 0, 0);
            if (showSun) lv_obj_set_style_shadow_width(wxSun,
                (int)(30 + 16 * (0.5f + 0.5f * sinf(ph * 2.0f))), 0);
        }
        // Moon + twinkling stars (clear night).
        if (wxMoon) lv_obj_set_style_opa(wxMoon, showMoon ? 255 : 0, 0);
        for (int i = 0; i < WX_STAR_N; ++i) {
            if (!wxStars[i]) continue;
            if (showMoon) {
                const float tw = 0.5f + 0.5f * sinf(ph * 2.2f + i * 0.9f);
                lv_obj_set_style_opa(wxStars[i], (uint8_t)(120 + 135 * tw), 0);
            } else lv_obj_set_style_opa(wxStars[i], 0, 0);
        }
        // Mountains / city skyline.
        if (wxMtn)  lv_obj_set_style_opa(wxMtn,  showMtn  ? 255 : 0, 0);
        if (wxCity) lv_obj_set_style_opa(wxCity, showCity ? 255 : 0, 0);

        // Clouds drift (faster with wind).
        auto driftCloud = [&](lv_obj_t *cl, float &cx, float base,
                              int wrapNeg, uint8_t opa) {
            if (!cl) return;
            cx += base * windF; if (cx > 410) cx = (float)wrapNeg;
            lv_obj_set_x(cl, (int)cx);
            lv_obj_set_style_opa(cl, showClouds ? opa : 0, 0);
        };
        driftCloud(wxCloud1, wxCloud1X, 0.5f,  -100, 230);
        driftCloud(wxCloud2, wxCloud2X, 0.35f,  -80, 210);

        // Fog: two wide translucent bands sliding opposite ways.
        if (wxFog1) {
            wxFog1X += 0.25f * windF; if (wxFog1X > 150) wxFog1X = -260;
            lv_obj_set_x(wxFog1, (int)wxFog1X);
            lv_obj_set_style_opa(wxFog1, showFog ? 150 : 0, 0);
        }
        if (wxFog2) {
            wxFog2X -= 0.18f * windF; if (wxFog2X < -260) wxFog2X = 150;
            lv_obj_set_x(wxFog2, (int)wxFog2X);
            lv_obj_set_style_opa(wxFog2, showFog ? 120 : 0, 0);
        }

        // Bird glides across and flaps (clear day only).
        if (wxBird) {
            if (showBird) {
                wxBirdX += 0.8f;
                if (wxBirdX > 440) { wxBirdX = -50; wxBirdY = 100 + (rand() % 38); }
                lv_obj_set_pos(wxBird, (int)wxBirdX, wxBirdY);
                lv_obj_set_style_opa(wxBird, 255, 0);
                if (now - wxBirdFlapMs > 260) {
                    wxBirdFlapMs = now; wxBirdWingsUp = !wxBirdWingsUp;
                    static lv_point_precise_t up[5]   = { {0,9},{11,0},{22,9},{33,0},{44,9} };
                    static lv_point_precise_t down[5] = { {0,2},{11,9},{22,2},{33,9},{44,2} };
                    lv_line_set_points(wxBird, wxBirdWingsUp ? up : down, 5);
                }
            } else lv_obj_set_style_opa(wxBird, 0, 0);
        }

        // Rain: streaks falling, tilted by wind. Heavier (all drops) in storms.
        const float rainTilt = (windF - 1.0f) * 2.2f;
        for (int i = 0; i < WX_RAIN_N; ++i) {
            if (!wxRain[i]) continue;
            const bool active = showRain && (showStorm || i < 10);
            if (!active) { lv_obj_set_style_opa(wxRain[i], 0, 0); continue; }
            wxRainY[i] += wxRainSpd[i] * windF;
            wxRainX[i] += rainTilt;
            if (wxRainY[i] > 440) { wxRainY[i] = -20 - (rand() % 60); wxRainX[i] = rand() % 410; }
            if (wxRainX[i] > 420) wxRainX[i] = 0;
            lv_obj_set_pos(wxRain[i], (int)wxRainX[i], (int)wxRainY[i]);
            lv_obj_set_style_opa(wxRain[i], 210, 0);
        }
        // Snow: slow flakes with a sideways sway.
        for (int i = 0; i < WX_SNOW_N; ++i) {
            if (!wxSnow[i]) continue;
            if (!showSnow) { lv_obj_set_style_opa(wxSnow[i], 0, 0); continue; }
            wxSnowY[i]  += wxSnowSpd[i];
            wxSnowPh[i] += 0.03f;
            if (wxSnowY[i] > 440) { wxSnowY[i] = -14 - (rand() % 60); wxSnowX[i] = rand() % 410; }
            lv_obj_set_pos(wxSnow[i], (int)(wxSnowX[i] + sinf(wxSnowPh[i]) * 10.0f), (int)wxSnowY[i]);
            lv_obj_set_style_opa(wxSnow[i], 235, 0);
        }

        // Lightning (storm only): a bright flare fading over ~0.55 s, strikes
        // spaced 1.8-4.4 s apart so it reads like real lightning.
        constexpr uint32_t FLASH_MS = 550;
        if (showStorm && now >= wxNextFlashMs) {
            wxFlashUntil = now + FLASH_MS;
            wxNextFlashMs = now + 1800 + (rand() % 2600);
        }
        uint8_t flashOpa = 0, boltOpa = 0;
        if (showStorm && now < wxFlashUntil) {
            const float rem = (float)(wxFlashUntil - now) / (float)FLASH_MS;
            flashOpa = (uint8_t)(170.0f * rem);
            boltOpa  = (uint8_t)(255.0f * rem);
        }
        if (wxFlash) lv_obj_set_style_bg_opa(wxFlash, flashOpa, 0);
        if (wxBolt)  lv_obj_set_style_opa(wxBolt, boltOpa, 0);
    } else if (homeFaceIndex == 6) {
        // Sweep hands spin in each eye.
        radarSweepAng = (radarSweepAng + 60) % 3600;   // ~2 s / revolution
        const int angR = (radarSweepAng + 1800) % 3600;
        if (radarEyeL)     lv_obj_set_style_transform_rotation(radarEyeL, radarSweepAng, 0);
        if (radarEyeLGlow) lv_obj_set_style_transform_rotation(radarEyeLGlow, radarSweepAng, 0);
        if (radarEyeR)     lv_obj_set_style_transform_rotation(radarEyeR, angR, 0);
        if (radarEyeRGlow) lv_obj_set_style_transform_rotation(radarEyeRGlow, angR, 0);
        // Rings around each eye keep spinning (alternating directions).
        if (now - radarRingMs > 33) {
            radarRingMs = now;
            for (int i = 0; i < 4; ++i) {
                if (!radarRing[i]) continue;
                radarRingAng[i] = (radarRingAng[i] + ((i & 1) ? 354 : 8)) % 360;
                lv_arc_set_angles(radarRing[i], radarRingAng[i], radarRingAng[i] + 95);
                if (radarRingGlow[i])
                    lv_arc_set_angles(radarRingGlow[i], radarRingAng[i], radarRingAng[i] + 95);
            }
        }
        // Cycling HUD readout between the eyes.
        if (radarCenterTxt && now - radarTxtMs > 140) {
            radarTxtMs = now;
            char b[64];
            snprintf(b, sizeof(b), "SCAN %02X\n%04X:%02X\nLOCK %02X\nSYNC %02X",
                     (unsigned)(rand() & 0xFF), (unsigned)(rand() & 0xFFFF),
                     (unsigned)(rand() & 0xFF), (unsigned)(rand() & 0xFF),
                     (unsigned)(rand() & 0xFF));
            lv_label_set_text(radarCenterTxt, b);
        }
        if (now - radarClockMs > 1000) { radarClockMs = now; updateRadarClock(); }
    }
}

// A two-tone HH:MM clock (+ date line) centered near the top, shared by the
// Crew and Video faces.
void buildFaceClock(int y)
{
    faceClockHour = lv_label_create(screen);
    lv_label_set_text(faceClockHour, "--");
    lv_obj_set_style_text_font(faceClockHour, &lv_font_montserrat_44, 0);
    lv_obj_set_style_text_color(faceClockHour, lv_color_hex(0x3AEFFF), 0);
    // Fixed-width, right-aligned so a single-digit hour (e.g. "5") still sits
    // flush against the colon instead of floating left of it.
    lv_obj_set_width(faceClockHour, 60);
    lv_obj_set_style_text_align(faceClockHour, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(faceClockHour, LV_ALIGN_TOP_MID, -38, y);

    faceClockColon = lv_label_create(screen);
    lv_label_set_text(faceClockColon, ":");
    lv_obj_set_style_text_font(faceClockColon, &lv_font_montserrat_44, 0);
    lv_obj_set_style_text_color(faceClockColon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(faceClockColon, LV_ALIGN_TOP_MID, 0, y);

    faceClockMin = lv_label_create(screen);
    lv_label_set_text(faceClockMin, "--");
    lv_obj_set_style_text_font(faceClockMin, &lv_font_montserrat_44, 0);
    lv_obj_set_style_text_color(faceClockMin, lv_color_hex(0xFF52C8), 0);
    lv_obj_align(faceClockMin, LV_ALIGN_TOP_MID, 30, y);

    faceClockDate = lv_label_create(screen);
    lv_label_set_text(faceClockDate, "--");
    lv_obj_set_style_text_font(faceClockDate, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(faceClockDate, lv_color_hex(0xCFE8FF), 0);
    lv_obj_align(faceClockDate, LV_ALIGN_TOP_MID, 0, y + 54);
}

void buildTerminalFace()
{
    lv_obj_t *bg = lv_obj_create(screen);
    lv_obj_remove_style_all(bg);
    lv_obj_set_size(bg, 410, 502);
    lv_obj_set_pos(bg, 0, 0);
    lv_obj_set_style_bg_color(bg, lv_color_hex(0x02060A), 0);
    lv_obj_set_style_bg_opa(bg, LV_OPA_COVER, 0);

    // Green terminal rain behind the readout (kept visible, lightly dimmed).
    matrixColA = 0x45FF8C;
    matrixColB = 0x9CFFC4;
    matrixSkullRecolor = 0xFFFFFF;    // white bunny skulls on this face
    dockIconColorOverride = 0xCFFFE0; // dock icons match the terminal readout
    createMatrixBackground();
    lv_obj_t *shade = lv_obj_create(screen);
    lv_obj_remove_style_all(shade);
    lv_obj_set_size(shade, 410, 502);
    lv_obj_set_pos(shade, 0, 0);
    lv_obj_set_style_bg_color(shade, lv_color_hex(0x02060A), 0);
    lv_obj_set_style_bg_opa(shade, 90, 0);

    createTopBar();

    const uint32_t green = 0x6CFF8F;
    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "M33K X");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xB6FFCF), 0);
    lv_obj_set_pos(title, 40, 70);

    lv_obj_t *boot = lv_label_create(screen);
    lv_label_set_text(boot, "> booting............ ok");
    lv_obj_set_style_text_font(boot, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(boot, lv_color_hex(green), 0);
    lv_obj_set_pos(boot, 40, 120);

    lv_obj_t *tl = lv_label_create(screen);
    lv_label_set_text(tl, "> TIME");
    lv_obj_set_style_text_font(tl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(tl, lv_color_hex(green), 0);
    lv_obj_set_pos(tl, 40, 180);

    termTime = lv_label_create(screen);
    lv_label_set_text(termTime, "--:--:--");
    lv_obj_set_style_text_font(termTime, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(termTime, lv_color_hex(0xCFFFE0), 0);
    lv_obj_set_pos(termTime, 150, 172);

    lv_obj_t *dl = lv_label_create(screen);
    lv_label_set_text(dl, "> DATE");
    lv_obj_set_style_text_font(dl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(dl, lv_color_hex(green), 0);
    lv_obj_set_pos(dl, 40, 230);
    termDate = lv_label_create(screen);
    lv_label_set_text(termDate, "---");
    lv_obj_set_style_text_font(termDate, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(termDate, lv_color_hex(0xCFFFE0), 0);
    lv_obj_set_pos(termDate, 150, 230);

    lv_obj_t *bl = lv_label_create(screen);
    lv_label_set_text(bl, "> BATT");
    lv_obj_set_style_text_font(bl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(bl, lv_color_hex(green), 0);
    lv_obj_set_pos(bl, 40, 270);
    termBatt = lv_label_create(screen);
    lv_label_set_text(termBatt, "--");
    lv_obj_set_style_text_font(termBatt, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(termBatt, lv_color_hex(0xCFFFE0), 0);
    lv_obj_set_pos(termBatt, 150, 270);

    termCursor = lv_label_create(screen);
    lv_label_set_text(termCursor, "> _");
    lv_obj_set_style_text_font(termCursor, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(termCursor, lv_color_hex(green), 0);
    lv_obj_set_pos(termCursor, 40, 320);
}

void buildCrewFace()
{
    lv_obj_t *bg = lv_obj_create(screen);
    lv_obj_remove_style_all(bg);
    lv_obj_set_size(bg, 410, 502);
    lv_obj_set_pos(bg, 0, 0);
    lv_obj_set_style_bg_color(bg, lv_color_hex(0x04060E), 0);
    lv_obj_set_style_bg_opa(bg, LV_OPA_COVER, 0);

    // Reuse the matrix-rain + bunny-skull engine as the backdrop, slanted
    // (diagonal rain) to set this face apart from the vertical ones.
    matrixDriftX = 2;
    createMatrixBackground();

    createTopBar();

    // Faint M33K / X graffiti.
    lv_obj_t *graf = lv_label_create(screen);
    lv_label_set_text(graf, "M33K");
    lv_obj_set_style_text_font(graf, &lv_font_montserrat_44, 0);
    lv_obj_set_style_text_color(graf, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_opa(graf, 34, 0);
    lv_obj_set_pos(graf, 12, 150);
    lv_obj_t *xg = lv_label_create(screen);
    lv_label_set_text(xg, "X");
    lv_obj_set_style_text_font(xg, &lv_font_montserrat_44, 0);
    lv_obj_set_style_text_color(xg, lv_color_hex(0xFF52C8), 0);
    lv_obj_set_style_text_opa(xg, 55, 0);
    lv_obj_set_pos(xg, 44, 210);

    // Characters: cat (left) and monkey (right) behind, main in front.
    const int catW = m33k_cat_body.header.w, catH = m33k_cat_body.header.h;
    const int monW = m33k_mon_body.header.w, monH = m33k_mon_body.header.h;
    const int mw = m33k_main.header.w, mh = m33k_main.header.h;
    crewCatBaseX = 6;            crewCatBaseY = 420 - catH;
    crewMonBaseX = 410 - 6 - monW; crewMonBaseY = 420 - monH;

    crewCatBody = lv_image_create(screen);
    lv_image_set_src(crewCatBody, &m33k_cat_body);
    lv_obj_set_pos(crewCatBody, crewCatBaseX, crewCatBaseY);
    crewCatTail = lv_image_create(screen);
    lv_image_set_src(crewCatTail, &m33k_cat_tail);
    lv_obj_set_pos(crewCatTail, crewCatBaseX, crewCatBaseY);
    lv_image_set_pivot(crewCatTail, (int)(catW * 0.40f), (int)(catH * 0.68f));

    crewMonBody = lv_image_create(screen);
    lv_image_set_src(crewMonBody, &m33k_mon_body);
    lv_obj_set_pos(crewMonBody, crewMonBaseX, crewMonBaseY);
    crewMonTail = lv_image_create(screen);
    lv_image_set_src(crewMonTail, &m33k_mon_tail);
    lv_obj_set_pos(crewMonTail, crewMonBaseX, crewMonBaseY);
    lv_image_set_pivot(crewMonTail, (int)(monW * 0.62f), (int)(monH * 0.52f));

    crewMain = lv_image_create(screen);
    lv_image_set_src(crewMain, &m33k_main);
    lv_obj_set_pos(crewMain, (410 - mw) / 2, 420 - mh);
    lv_image_set_pivot(crewMain, mw / 2, mh);

    lv_obj_t *imgs[5] = { crewCatBody, crewCatTail, crewMonBody, crewMonTail, crewMain };
    for (int i = 0; i < 5; ++i) lv_obj_clear_flag(imgs[i], LV_OBJ_FLAG_CLICKABLE);

    buildFaceClock(46);
}

void buildVideoFace()
{
    lv_obj_t *bg = lv_obj_create(screen);
    lv_obj_remove_style_all(bg);
    lv_obj_set_size(bg, 410, 502);
    lv_obj_set_pos(bg, 0, 0);
    lv_obj_set_style_bg_color(bg, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(bg, LV_OPA_COVER, 0);

    if (m33k_video_frame_count > 0) {
        const int vw = m33k_video_frames[0]->header.w;
        const int vh = m33k_video_frames[0]->header.h;
        videoImg = lv_image_create(screen);
        lv_image_set_src(videoImg, m33k_video_frames[0]);
        // Frames are stored at full screen size, so blit them directly (no
        // runtime scaling): cheap to draw and keeps touch responsive.
        lv_obj_set_pos(videoImg, (410 - vw) / 2, (502 - vh) / 2);
        lv_obj_clear_flag(videoImg, LV_OBJ_FLAG_CLICKABLE);
        videoFrameIdx = 0; videoFrameMs = millis();
    }
    createTopBar();
    buildFaceClock(46);
}

// Radar face: a settled m33k t3ch frame with live overlays - a sweep hand and
// spinning rings in each scope eye, a cycling readout between them, and the
// time as sideways neon-green 7-segment digits with the day/date beside it.
void buildRadarFace()
{
    lv_obj_t *bg = lv_obj_create(screen);
    lv_obj_remove_style_all(bg);
    lv_obj_set_size(bg, 410, 502);
    lv_obj_set_pos(bg, 0, 0);
    lv_obj_set_style_bg_color(bg, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(bg, LV_OPA_COVER, 0);

    if (m33k_radar_frame_count > 0) {
        radarFaceImg = lv_image_create(screen);
        lv_image_set_src(radarFaceImg, m33k_radar_frames[0]);   // settled frame
        lv_obj_set_pos(radarFaceImg, 0, 0);
        lv_obj_clear_flag(radarFaceImg, LV_OBJ_FLAG_CLICKABLE);
    }
    radarSweepAng = 0; radarClockMs = 0; radarRingMs = 0; radarTxtMs = 0;

    // Two spinning rings around each eye (an arc segment that rotates), each
    // with a soft wide halo of the same color beneath so the ring glows.
    auto makeRing = [&](int cx, int cy, int r, uint32_t col,
                        lv_obj_t **glowOut) -> lv_obj_t * {
        auto mkArc = [&](int w, uint8_t opa) -> lv_obj_t * {
            lv_obj_t *a = lv_arc_create(screen);
            lv_obj_set_size(a, 2 * r, 2 * r);
            lv_obj_set_pos(a, cx - r, cy - r);
            lv_obj_clear_flag(a, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
            lv_obj_set_style_arc_opa(a, 0, LV_PART_MAIN);            // hide bg track
            lv_obj_set_style_bg_opa(a, 0, LV_PART_KNOB);             // hide knob
            lv_obj_set_style_arc_color(a, lv_color_hex(col), LV_PART_INDICATOR);
            lv_obj_set_style_arc_width(a, w, LV_PART_INDICATOR);
            lv_obj_set_style_arc_opa(a, opa, LV_PART_INDICATOR);
            lv_arc_set_bg_angles(a, 0, 360);
            lv_arc_set_angles(a, 0, 95);
            return a;
        };
        lv_obj_t *glow = mkArc(12, 70);     // soft wide halo
        if (glowOut) *glowOut = glow;
        return mkArc(3, 230);               // sharp ring on top
    };
    radarRing[0] = makeRing(RADAR_LX, RADAR_LY, RADAR_ER,      0x4DF0FF, &radarRingGlow[0]);
    radarRing[1] = makeRing(RADAR_LX, RADAR_LY, RADAR_ER - 16, RADAR_NEON, &radarRingGlow[1]);
    radarRing[2] = makeRing(RADAR_RX, RADAR_RY, RADAR_ER,      0x4DF0FF, &radarRingGlow[2]);
    radarRing[3] = makeRing(RADAR_RX, RADAR_RY, RADAR_ER - 16, RADAR_NEON, &radarRingGlow[3]);
    for (int i = 0; i < 4; ++i) radarRingAng[i] = (i * 90) % 360;

    // A rotating radial sweep hand over each eye, with a soft wide pink halo
    // beneath the sharp line to make it glow. Both pivot at the eye center.
    auto makeEye = [&](lv_point_precise_t *pts, int cx, int cy, int r,
                       lv_obj_t **glowOut) -> lv_obj_t * {
        pts[0].x = 0;  pts[0].y = 0;
        pts[1].x = r;  pts[1].y = 0;
        // glow halo (wide, soft)
        lv_obj_t *g = lv_line_create(screen);
        lv_line_set_points(g, pts, 2);
        lv_obj_set_pos(g, cx, cy);
        lv_obj_set_style_line_color(g, lv_color_hex(0xFF2EC6), 0);
        lv_obj_set_style_line_width(g, 12, 0);
        lv_obj_set_style_line_rounded(g, true, 0);
        lv_obj_set_style_line_opa(g, 70, 0);
        lv_obj_set_style_transform_pivot_x(g, 0, 0);
        lv_obj_set_style_transform_pivot_y(g, 0, 0);
        lv_obj_clear_flag(g, LV_OBJ_FLAG_CLICKABLE);
        if (glowOut) *glowOut = g;
        // sharp bright-pink line on top
        lv_obj_t *ln = lv_line_create(screen);
        lv_line_set_points(ln, pts, 2);
        lv_obj_set_pos(ln, cx, cy);
        lv_obj_set_style_line_color(ln, lv_color_hex(0xFF2EC6), 0);  // bright pink
        lv_obj_set_style_line_width(ln, 4, 0);
        lv_obj_set_style_line_rounded(ln, true, 0);
        lv_obj_set_style_line_opa(ln, 255, 0);
        lv_obj_set_style_transform_pivot_x(ln, 0, 0);
        lv_obj_set_style_transform_pivot_y(ln, 0, 0);
        lv_obj_clear_flag(ln, LV_OBJ_FLAG_CLICKABLE);
        return ln;
    };
    radarEyeL = makeEye(radarEyeLPts, RADAR_LX, RADAR_LY, RADAR_ER, &radarEyeLGlow);
    radarEyeR = makeEye(radarEyeRPts, RADAR_RX, RADAR_RY, RADAR_ER, &radarEyeRGlow);

    // Cycling HUD readout between the eyes (sits over the baked text, stays live).
    lv_obj_t *ctBox = lv_obj_create(screen);
    lv_obj_remove_style_all(ctBox);
    lv_obj_set_size(ctBox, 56, 96);
    lv_obj_set_pos(ctBox, 174, 196);
    lv_obj_set_style_bg_color(ctBox, lv_color_hex(0x02060A), 0);
    lv_obj_set_style_bg_opa(ctBox, 150, 0);
    lv_obj_set_style_radius(ctBox, 4, 0);
    lv_obj_clear_flag(ctBox, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
    radarCenterTxt = lv_label_create(ctBox);
    lv_label_set_text(radarCenterTxt, "SCAN\n....\n....\n....");
    lv_obj_set_style_text_font(radarCenterTxt, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(radarCenterTxt, lv_color_hex(0x8FFCE8), 0);
    lv_obj_set_style_text_line_space(radarCenterTxt, 5, 0);
    lv_obj_center(radarCenterTxt);

    // Left scrim so the neon time/date stay readable over the busy art.
    lv_obj_t *scrim = lv_obj_create(screen);
    lv_obj_remove_style_all(scrim);
    lv_obj_set_size(scrim, 78, 502);
    lv_obj_set_pos(scrim, 0, 0);
    lv_obj_set_style_bg_color(scrim, lv_color_hex(0x010204), 0);
    lv_obj_set_style_bg_opa(scrim, 110, 0);
    lv_obj_clear_flag(scrim, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));

    // Sideways 7-segment time: [HH] : [MM] in one row, rotated -90 as a unit.
    radarVTime = lv_obj_create(screen);
    lv_obj_remove_style_all(radarVTime);
    lv_obj_set_flex_flow(radarVTime, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(radarVTime, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(radarVTime, 5, 0);
    lv_obj_set_size(radarVTime, 152, 54);
    lv_obj_clear_flag(radarVTime, (lv_obj_flag_t)(LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE));
    lv_obj_set_style_transform_pivot_x(radarVTime, 76, 0);
    lv_obj_set_style_transform_pivot_y(radarVTime, 27, 0);
    lv_obj_set_style_transform_rotation(radarVTime, -900, 0);
    lv_obj_align(radarVTime, LV_ALIGN_LEFT_MID, -46, 0);

    const int DW = 30, DH = 54, T = 6, VL = (DH - 3 * T) / 2;
    const int segGeo[7][4] = {
        { T,       0,        DW - 2*T, T  },   // a top
        { DW - T,  T,        T,        VL },   // b top-right
        { DW - T,  2*T + VL, T,        VL },   // c bottom-right
        { T,       DH - T,   DW - 2*T, T  },   // d bottom
        { 0,       2*T + VL, T,        VL },   // e bottom-left
        { 0,       T,        T,        VL },   // f top-left
        { T,       T + VL,   DW - 2*T, T  },   // g middle
    };
    auto makeDigitCell = [&](int cell) {
        lv_obj_t *dc = lv_obj_create(radarVTime);
        lv_obj_remove_style_all(dc);
        lv_obj_set_size(dc, DW, DH);
        lv_obj_clear_flag(dc, (lv_obj_flag_t)(LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE));
        for (int i = 0; i < 7; ++i) {
            lv_obj_t *s = lv_obj_create(dc);
            lv_obj_remove_style_all(s);
            lv_obj_set_size(s, segGeo[i][2], segGeo[i][3]);
            lv_obj_set_pos(s, segGeo[i][0], segGeo[i][1]);
            lv_obj_set_style_radius(s, 2, 0);
            lv_obj_set_style_bg_color(s, lv_color_hex(0x0B3D12), 0);
            lv_obj_set_style_bg_opa(s, 90, 0);
            lv_obj_set_style_shadow_color(s, lv_color_hex(RADAR_NEON), 0);
            lv_obj_set_style_shadow_width(s, 8, 0);
            lv_obj_set_style_shadow_opa(s, 0, 0);
            lv_obj_clear_flag(s, (lv_obj_flag_t)(LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE));
            radarSeg[cell][i] = s;
        }
    };
    makeDigitCell(0);
    makeDigitCell(1);
    {   // colon between hours and minutes
        lv_obj_t *cc = lv_obj_create(radarVTime);
        lv_obj_remove_style_all(cc);
        lv_obj_set_size(cc, 12, DH);
        lv_obj_clear_flag(cc, (lv_obj_flag_t)(LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE));
        for (int k = 0; k < 2; ++k) {
            lv_obj_t *dot = lv_obj_create(cc);
            lv_obj_remove_style_all(dot);
            lv_obj_set_size(dot, 7, 7);
            lv_obj_set_pos(dot, 2, (k == 0) ? 16 : 32);
            lv_obj_set_style_radius(dot, 2, 0);
            lv_obj_set_style_bg_color(dot, lv_color_hex(RADAR_NEON), 0);
            lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
            lv_obj_clear_flag(dot, (lv_obj_flag_t)(LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE));
        }
    }
    makeDigitCell(2);
    makeDigitCell(3);

    // Vertical day + date just inside the time, in neon green.
    radarVMeta = lv_obj_create(screen);
    lv_obj_remove_style_all(radarVMeta);
    lv_obj_set_size(radarVMeta, 190, 22);
    lv_obj_set_flex_flow(radarVMeta, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(radarVMeta, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(radarVMeta, 10, 0);
    lv_obj_clear_flag(radarVMeta, (lv_obj_flag_t)(LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE));
    lv_obj_set_style_transform_pivot_x(radarVMeta, 95, 0);
    lv_obj_set_style_transform_pivot_y(radarVMeta, 11, 0);
    lv_obj_set_style_transform_rotation(radarVMeta, -900, 0);
    lv_obj_align(radarVMeta, LV_ALIGN_LEFT_MID, -27, 0);

    radarMetaDay = lv_label_create(radarVMeta);
    lv_label_set_text(radarMetaDay, "---");
    lv_obj_set_style_text_font(radarMetaDay, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(radarMetaDay, lv_color_hex(RADAR_NEON), 0);
    radarMetaDate = lv_label_create(radarVMeta);
    lv_label_set_text(radarMetaDate, "---");
    lv_obj_set_style_text_font(radarMetaDate, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(radarMetaDate, lv_color_hex(RADAR_NEON), 0);

    createTopBar();
    updateRadarClock();
}

// ===========================================================================
// M33K Runner - a neon endless-runner game (Page::Game), launched from the dock.
// Tap anywhere to jump neon shards; flying crew come in overhead (don't jump).
// Pick one of four characters; high score is saved to NVS.
// ===========================================================================
static void gameShowPick(bool show)
{
    auto SH = [&](lv_obj_t *o) {
        if (!o) return;
        if (show) lv_obj_clear_flag(o, LV_OBJ_FLAG_HIDDEN);
        else      lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
    };
    SH(gamePickPanel); SH(gamePickTitle); SH(gamePickHint);
    SH(gameModeBtn[0]); SH(gameModeBtn[1]);
    for (int i = 0; i < 4; ++i) SH(gamePickBtn[i]);
    if (show) {   // hide live wardrive entities behind the (slightly sheer) panel
        if (driveCar) lv_obj_add_flag(driveCar, LV_OBJ_FLAG_HIDDEN);
        if (driveHead) lv_obj_add_flag(driveHead, LV_OBJ_FLAG_HIDDEN);
        if (driveBg)  lv_obj_add_flag(driveBg, LV_OBJ_FLAG_HIDDEN);
        for (int i = 0; i < DRV_PICK; ++i) if (drvPick[i]) lv_obj_add_flag(drvPick[i], LV_OBJ_FLAG_HIDDEN);
        for (int i = 0; i < DRV_ENE; ++i)  if (drvEne[i])  lv_obj_add_flag(drvEne[i],  LV_OBJ_FLAG_HIDDEN);
    }
    if (gameOverLbl) {
        if (show && gameState == GameState::Dead) lv_obj_clear_flag(gameOverLbl, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(gameOverLbl, LV_OBJ_FLAG_HIDDEN);
    }
}

// Switch between the two games on the pick screen.
static void gameModeEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED || !clickAllowed()) return;
    gameMode = (int)(intptr_t)lv_event_get_user_data(e);
    hapticTap();
    for (int i = 0; i < 2; ++i) if (gameModeBtn[i]) {
        bool on = (i == gameMode);
        lv_obj_set_style_border_width(gameModeBtn[i], on ? 3 : 1, 0);
        lv_obj_set_style_border_color(gameModeBtn[i], lv_color_hex(on ? 0x39FF14 : 0x214058), 0);
        lv_obj_set_style_bg_color(gameModeBtn[i], lv_color_hex(on ? 0x113022 : 0x0A1726), 0);
    }
    if (gamePickHint)  lv_label_set_text(gamePickHint, gameMode
        ? "PICK A CAR  -  HOLD TO STEER" : "TAP A CHARACTER TO PLAY");
}

static void gameStart(int idx)
{
    if (idx < 0 || idx > 3) idx = 0;
    gameSel = idx; prefs.putInt("gameChar", idx);
    const lv_image_dsc_t *sp = g_run0[idx];
    gamePw = sp->header.w; gamePh = sp->header.h;
    if (gamePlayer) {
        lv_image_set_src(gamePlayer, sp);
        lv_image_set_pivot(gamePlayer, (int)gamePw / 2, (int)gamePh / 2);
        lv_image_set_rotation(gamePlayer, 0);
    }
    gamePy = GAME_GROUND_Y - gamePh; gameVy = 0; gameOnGround = true;
    gameSpd = 280; gameDist = 0; gameSpawnIn = 0.8f; gameShake = 0; gameLastAir = false;
    gameFrame = 0; gameAnimMs = millis();
    for (int i = 0; i < GAME_OBS; ++i) {
        gameObs[i].active = false;
        if (gameObs[i].img) lv_obj_add_flag(gameObs[i].img, LV_OBJ_FLAG_HIDDEN);
    }
    for (int i = 0; i < GAME_BLAST; ++i) {
        gameBlastActive[i] = false;
        if (gameBlast[i]) { lv_image_set_src(gameBlast[i], g_blast[gameSel]); lv_obj_add_flag(gameBlast[i], LV_OBJ_FLAG_HIDDEN); }
    }
    for (int i = 0; i < GAME_POP; ++i) {
        gamePopT[i] = -1.0f;
        if (gamePop[i]) lv_obj_add_flag(gamePop[i], LV_OBJ_FLAG_HIDDEN);
    }
    for (int i = 0; i < GAME_EXPL; ++i) {
        gameExplT[i] = -1.0f;
        if (gameExpl[i]) lv_obj_add_flag(gameExpl[i], LV_OBJ_FLAG_HIDDEN);
    }
    gamePopNextMs = millis() + 400;
    if (gamePlayer) {
        lv_obj_set_pos(gamePlayer, 70, (int)gamePy);
        lv_obj_clear_flag(gamePlayer, LV_OBJ_FLAG_HIDDEN);
    }
    gameState = GameState::Play;
    gameShowPick(false);
}

static void gameDie()
{
    gameState = GameState::Dead; gameShake = 14;
    if ((int)gameDist > gameBest) { gameBest = (int)gameDist; prefs.putInt("gameBest", gameBest); }
    hapticTap(); sfxDie();
    if (gameOverLbl) lv_label_set_text_fmt(gameOverLbl, "SCORE %d   BEST %d", (int)gameDist, gameBest);
    if (gamePlayer) lv_obj_add_flag(gamePlayer, LV_OBJ_FLAG_HIDDEN);
    for (int i = 0; i < GAME_OBS; ++i) {
        gameObs[i].active = false;
        if (gameObs[i].img) lv_obj_add_flag(gameObs[i].img, LV_OBJ_FLAG_HIDDEN);
    }
    for (int i = 0; i < GAME_BLAST; ++i) {
        gameBlastActive[i] = false;
        if (gameBlast[i]) lv_obj_add_flag(gameBlast[i], LV_OBJ_FLAG_HIDDEN);
    }
    gameShowPick(true);
}

// Fill one free obstacle slot at startX. forceType: -1 random, 0 ground spike,
// 1 air bubble. Returns true if a slot was placed.
static bool gameSpawnOne(float startX, int forceType)
{
    int slot = -1;
    for (int i = 0; i < GAME_OBS; ++i) if (!gameObs[i].active) { slot = i; break; }
    if (slot < 0 || !gameObs[slot].img) return false;
    float airP = 0.24f + (gameSpd - 280.0f) / 620.0f; if (airP > 0.66f) airP = 0.66f;
    bool air = (forceType == 0) ? false : (forceType == 1) ? true
                 : (((rand() % 1000) / 1000.0f) < airP);
    gameLastAir = air;
    GameObs &o = gameObs[slot];
    o.active = true; o.air = air; o.x = startX; o.bob = rand() % 60;
    if (air) {
        lv_image_set_src(o.img, g_flyers[rand() % 3]); o.w = 44; o.h = 44;
    } else {
        float tb = (gameSpd - 260.0f) / 720.0f; if (tb > 0.55f) tb = 0.55f;
        bool tall = ((rand() % 1000) / 1000.0f) < tb;
        lv_image_set_src(o.img, tall ? &g_shard_t : &g_shard_s);
        o.w = tall ? 34 : 30; o.h = tall ? 52 : 34;
    }
    lv_obj_clear_flag(o.img, LV_OBJ_FLAG_HIDDEN);
    return true;
}

static void gameSpawn()
{
    if (!gameSpawnOne(430.0f, -1)) return;
    // As the run speeds up, often send a second obstacle right behind the first
    // - the opposite kind, staggered - so you must clear a spike and a bubble
    // close together (jump the spike, then float/shoot the bubble).
    float pDouble = (gameSpd - 360.0f) / 760.0f;
    if (pDouble < 0.0f) pDouble = 0.0f; if (pDouble > 0.55f) pDouble = 0.55f;
    if (((rand() % 1000) / 1000.0f) < pDouble) {
        int otherType = gameLastAir ? 0 : 1;                 // opposite of the first
        gameSpawnOne(430.0f + 76.0f + (rand() % 66), otherType);
    }
}

void gameJumpEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (currentPage != Page::Game || gameState != GameState::Play || gameMode != 0) return;
    if (gameOnGround) {
        gameVy = -GAME_JUMP; gameOnGround = false; hapticTap(); sfxJump();
    } else {
        // In the air: a tap floats (flap) and fires a blast at the bubbles.
        if (gameVy > -260.0f) gameVy = -GAME_FLAP;
        for (int i = 0; i < GAME_BLAST; ++i) {
            if (!gameBlastActive[i] && gameBlast[i]) {
                gameBlastActive[i] = true;
                gameBlastX[i] = 70 + gamePw;             // muzzle at the runner's front
                gameBlastY[i] = gamePy + gamePh * 0.4f;
                lv_obj_clear_flag(gameBlast[i], LV_OBJ_FLAG_HIDDEN);
                break;
            }
        }
        hapticTap(); sfxShoot();
    }
}

void gamePickEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED || !clickAllowed()) return;
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    noteActivity(); hapticTap();
    if (gameMode == 1) driveStart(idx);
    else               gameStart(idx);
}

void driveTick(float dt, uint32_t now);

void gameTimerCb(lv_timer_t *timer)
{
    LV_UNUSED(timer);
    if (currentPage != Page::Game || gameState != GameState::Play) return;
    const uint32_t now = millis();
    const float dt = 0.024f;
    if (gameMode == 1) { driveTick(dt, now); return; }
    gameDist += gameSpd * dt * 0.1f;
    gameSpd  += dt * 22.0f; if (gameSpd > 900.0f) gameSpd = 900.0f;   // steeper ramp, higher cap
    gameVy   += GAME_GRAV * dt; gamePy += gameVy * dt;
    if (gamePy >= GAME_GROUND_Y - gamePh) { gamePy = GAME_GROUND_Y - gamePh; gameVy = 0; gameOnGround = true; }
    int rot = gameOnGround ? 0 : (int)(gameVy * 0.18f);
    if (rot > 250) rot = 250; if (rot < -250) rot = -250;
    if (gamePlayer) { lv_obj_set_pos(gamePlayer, 70, (int)gamePy); lv_image_set_rotation(gamePlayer, rot); }

    // Sprite frame: jump pose while airborne; run cycle (if any) on the ground.
    if (gamePlayer) {
        const lv_image_dsc_t *want;
        if (!gameOnGround) want = g_jump[gameSel];
        else if (g_run_anim[gameSel]) {
            const uint32_t step = (gameSpd > 520) ? 90 : 130;
            if (now - gameAnimMs > step) { gameAnimMs = now; gameFrame ^= 1; }
            want = gameFrame ? g_run1[gameSel] : g_run0[gameSel];
        } else want = g_run0[gameSel];
        lv_image_set_src(gamePlayer, want);
    }

    // Parallax skyline drifts left and wraps.
    for (int i = 0; i < GAME_BLD; ++i) {
        if (!gameBld[i]) continue;
        gameBldX[i] -= gameSpd * dt * 0.35f;
        if (gameBldX[i] < -70.0f) gameBldX[i] += GAME_BLD * 64.0f;
        lv_obj_set_x(gameBld[i], (int)gameBldX[i]);
    }

    // Wi-Fi / BLE signal bursts popping in the sky (expanding rings).
    if (now >= gamePopNextMs) {
        for (int i = 0; i < GAME_POP; ++i) {
            if (gamePopT[i] < 0 && gamePop[i]) {
                gamePopT[i] = 0; gamePopX[i] = 24 + rand() % 360; gamePopY[i] = 40 + rand() % 200;
                lv_obj_set_style_border_color(gamePop[i], lv_color_hex((rand() & 1) ? 0x3AEFFF : 0xFF2EC6), 0);
                lv_obj_clear_flag(gamePop[i], LV_OBJ_FLAG_HIDDEN);
                break;
            }
        }
        gamePopNextMs = now + 300 + rand() % 550;
    }
    for (int i = 0; i < GAME_POP; ++i) {
        if (gamePopT[i] < 0 || !gamePop[i]) continue;
        gamePopT[i] += dt / 0.5f;
        if (gamePopT[i] >= 1.0f) { gamePopT[i] = -1.0f; lv_obj_add_flag(gamePop[i], LV_OBJ_FLAG_HIDDEN); continue; }
        int sz = (int)(8 + gamePopT[i] * 44);
        lv_obj_set_size(gamePop[i], sz, sz);
        lv_obj_set_pos(gamePop[i], gamePopX[i] - sz / 2, gamePopY[i] - sz / 2);
        lv_obj_set_style_border_opa(gamePop[i], (uint8_t)((1.0f - gamePopT[i]) * 220), 0);
    }

    // Explosions (white -> red -> black flash) when a bubble was popped.
    for (int i = 0; i < GAME_EXPL; ++i) {
        if (gameExplT[i] < 0 || !gameExpl[i]) continue;
        gameExplT[i] += dt / 0.30f;
        if (gameExplT[i] >= 1.0f) { gameExplT[i] = -1.0f; lv_obj_add_flag(gameExpl[i], LV_OBJ_FLAG_HIDDEN); continue; }
        float t = gameExplT[i];
        int sz = (int)(12 + t * 46);
        lv_obj_set_size(gameExpl[i], sz, sz);
        lv_obj_set_pos(gameExpl[i], gameExplX[i] - sz / 2, gameExplY[i] - sz / 2);
        uint32_t col = (t < 0.30f) ? 0xFFFFFF : (t < 0.60f ? 0xFF2222 : 0x111111);
        lv_obj_set_style_bg_color(gameExpl[i], lv_color_hex(col), 0);
        lv_obj_set_style_bg_opa(gameExpl[i], (uint8_t)((1.0f - t) * 255), 0);
    }

    gameSpawnIn -= dt;
    if (gameSpawnIn <= 0) {
        gameSpawn();
        float gap = 1.0f - gameSpd / 520.0f; if (gap < 0.30f) gap = 0.30f;
        gameSpawnIn = gap + (rand() % 30) / 100.0f;
    }

    // 1) Move obstacles and record their y.
    for (int i = 0; i < GAME_OBS; ++i) {
        GameObs &o = gameObs[i];
        if (!o.active || !o.img) continue;
        o.x -= gameSpd * dt;
        if (o.air) { o.bob++; o.oy = GAME_GROUND_Y - 120 + (int)(sinf(o.bob * 0.08f) * 7.0f); }
        else o.oy = GAME_GROUND_Y - (int)o.h;
        lv_obj_set_pos(o.img, (int)o.x, o.oy);
        if (o.x + o.w < -12) { o.active = false; lv_obj_add_flag(o.img, LV_OBJ_FLAG_HIDDEN); }
    }

    // 2) Blasts fly right and pop flying bubbles.
    for (int i = 0; i < GAME_BLAST; ++i) {
        if (!gameBlastActive[i] || !gameBlast[i]) continue;
        gameBlastX[i] += 820.0f * dt;
        if (gameBlastX[i] > 420) { gameBlastActive[i] = false; lv_obj_add_flag(gameBlast[i], LV_OBJ_FLAG_HIDDEN); continue; }
        lv_obj_set_pos(gameBlast[i], (int)gameBlastX[i], (int)gameBlastY[i]);
        for (int j = 0; j < GAME_OBS; ++j) {
            GameObs &o = gameObs[j];
            if (!o.active || !o.air) continue;
            if (gameBlastX[i] + 14 > o.x && gameBlastX[i] < o.x + o.w &&
                gameBlastY[i] + 6 > o.oy && gameBlastY[i] < o.oy + o.h) {
                o.active = false; lv_obj_add_flag(o.img, LV_OBJ_FLAG_HIDDEN);
                gameBlastActive[i] = false; lv_obj_add_flag(gameBlast[i], LV_OBJ_FLAG_HIDDEN);
                gameDist += 5;                      // bonus for popping a bubble
                sfxPop();
                for (int k = 0; k < GAME_EXPL; ++k) {
                    if (gameExplT[k] < 0 && gameExpl[k]) {
                        gameExplT[k] = 0.0f;
                        gameExplX[k] = (int)(o.x + o.w / 2);
                        gameExplY[k] = o.oy + (int)o.h / 2;
                        lv_obj_clear_flag(gameExpl[k], LV_OBJ_FLAG_HIDDEN);
                        break;
                    }
                }
                break;
            }
        }
    }

    // 3) Player collision against remaining obstacles.
    const int ax = 70 + 6, ay = (int)gamePy + 6, aw = (int)gamePw - 12, ah = (int)gamePh - 10;
    for (int i = 0; i < GAME_OBS; ++i) {
        GameObs &o = gameObs[i];
        if (!o.active || !o.img) continue;
        int ins = o.air ? 7 : 2;
        int bx = (int)o.x + ins, by = o.oy + ins, bw = (int)o.w - 2 * ins, bh = (int)o.h - 2 * ins;
        if (ax < bx + bw && ax + aw > bx && ay < by + bh && ay + ah > by) { gameDie(); return; }
    }
    if (gameScoreLbl) lv_label_set_text_fmt(gameScoreLbl, "%05d", (int)gameDist);
}

// ===========================================================================
// M33K Wardrive - top-down driving game (second mode on the Game page).
// Drive a character car around a dark grid, collect Wi-Fi / BLE signals, dodge
// roaming malware. Hold a finger on the screen to steer the car toward it.
// ===========================================================================
static void buildDriveCar(int sel)
{
    uint32_t trim = DRV_TRIM[(sel >= 0 && sel < 4) ? sel : 0];
    // Body (points "up" at rotation 0).
    driveCar = lv_obj_create(screen);
    lv_obj_remove_style_all(driveCar);
    lv_obj_set_size(driveCar, 30, 50);
    lv_obj_set_style_radius(driveCar, 9, 0);
    lv_obj_set_style_bg_color(driveCar, lv_color_hex(0x121419), 0);
    lv_obj_set_style_bg_opa(driveCar, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(driveCar, 2, 0);
    lv_obj_set_style_border_color(driveCar, lv_color_hex(trim), 0);
    lv_obj_set_style_shadow_color(driveCar, lv_color_hex(trim), 0);
    lv_obj_set_style_shadow_width(driveCar, 12, 0);
    lv_obj_set_style_shadow_opa(driveCar, 150, 0);
    lv_obj_clear_flag(driveCar, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
    lv_obj_set_style_transform_pivot_x(driveCar, 15, 0);
    lv_obj_set_style_transform_pivot_y(driveCar, 25, 0);
    // Windshield near the front.
    lv_obj_t *ws = lv_obj_create(driveCar); lv_obj_remove_style_all(ws);
    lv_obj_set_size(ws, 20, 12); lv_obj_set_pos(ws, 5, 7);
    lv_obj_set_style_radius(ws, 4, 0);
    lv_obj_set_style_bg_color(ws, lv_color_hex(trim), 0);
    lv_obj_set_style_bg_opa(ws, 150, 0);
    // Headlight dots.
    for (int i = 0; i < 2; ++i) {
        lv_obj_t *h = lv_obj_create(driveCar); lv_obj_remove_style_all(h);
        lv_obj_set_size(h, 5, 5); lv_obj_set_pos(h, i ? 20 : 5, 2);
        lv_obj_set_style_radius(h, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(h, lv_color_hex(0xFFFFFF), 0);
    }
    // Rear light bar.
    lv_obj_t *rb = lv_obj_create(driveCar); lv_obj_remove_style_all(rb);
    lv_obj_set_size(rb, 22, 4); lv_obj_set_pos(rb, 4, 43);
    lv_obj_set_style_radius(rb, 2, 0);
    lv_obj_set_style_bg_color(rb, lv_color_hex(0xFF3B3B), 0);
    lv_obj_add_flag(driveCar, LV_OBJ_FLAG_HIDDEN);

    // Character head rides upright on top of the car (separate object so it
    // stays readable while the car body rotates to its heading).
    driveHead = lv_image_create(screen);
    lv_image_set_src(driveHead, g_carhead[(sel >= 0 && sel < 4) ? sel : 0]);
    lv_obj_clear_flag(driveHead, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(driveHead, LV_OBJ_FLAG_HIDDEN);
}

static void driveSpawnPick(int i)
{
    drvPickX[i] = 30 + rand() % 350;
    drvPickY[i] = 74 + rand() % 392;
    drvPickBle[i] = (rand() & 1);
    if (drvPick[i]) {
        uint32_t col = drvPickBle[i] ? 0x2A8BFF : 0x3AEFFF;
        lv_label_set_text(drvPick[i], drvPickBle[i] ? LV_SYMBOL_BLUETOOTH : LV_SYMBOL_WIFI);
        lv_obj_set_style_text_color(drvPick[i], lv_color_hex(col), 0);
        lv_obj_set_style_shadow_color(drvPick[i], lv_color_hex(col), 0);
        lv_obj_set_pos(drvPick[i], (int)drvPickX[i] - 11, (int)drvPickY[i] - 12);
        lv_obj_clear_flag(drvPick[i], LV_OBJ_FLAG_HIDDEN);
    }
}

static void drivePlaceEnemy(int i)
{
    // Spawn at a screen edge, away from the car.
    int edge = rand() % 4;
    if (edge == 0) { drvEneX[i] = rand() % 410; drvEneY[i] = -10; }
    else if (edge == 1) { drvEneX[i] = rand() % 410; drvEneY[i] = 512; }
    else if (edge == 2) { drvEneX[i] = -10; drvEneY[i] = rand() % 502; }
    else { drvEneX[i] = 420; drvEneY[i] = rand() % 502; }
    drvEneVx[i] = 0; drvEneVy[i] = 0; drvEneActive[i] = true;
    if (drvEne[i]) lv_obj_clear_flag(drvEne[i], LV_OBJ_FLAG_HIDDEN);
}

void driveStart(int sel)
{
    if (sel < 0 || sel > 3) sel = 0;
    gameSel = sel; prefs.putInt("gameChar", sel);
    gameMode = 1;
    // Hide runner-only actors; show the grid map.
    if (gamePlayer) lv_obj_add_flag(gamePlayer, LV_OBJ_FLAG_HIDDEN);
    for (int i = 0; i < GAME_OBS; ++i) { gameObs[i].active = false; if (gameObs[i].img) lv_obj_add_flag(gameObs[i].img, LV_OBJ_FLAG_HIDDEN); }
    for (int i = 0; i < GAME_POP; ++i)   { gamePopT[i] = -1.0f; if (gamePop[i]) lv_obj_add_flag(gamePop[i], LV_OBJ_FLAG_HIDDEN); }
    for (int i = 0; i < GAME_BLAST; ++i) { gameBlastActive[i] = false; if (gameBlast[i]) lv_obj_add_flag(gameBlast[i], LV_OBJ_FLAG_HIDDEN); }
    for (int i = 0; i < GAME_EXPL; ++i)  { gameExplT[i] = -1.0f; if (gameExpl[i]) lv_obj_add_flag(gameExpl[i], LV_OBJ_FLAG_HIDDEN); }
    if (driveBg) lv_obj_clear_flag(driveBg, LV_OBJ_FLAG_HIDDEN);

    // Recolor the car and swap in the chosen character's head.
    if (driveCar) {
        uint32_t trim = DRV_TRIM[sel];
        lv_obj_set_style_border_color(driveCar, lv_color_hex(trim), 0);
        lv_obj_set_style_shadow_color(driveCar, lv_color_hex(trim), 0);
    }
    if (driveHead) {
        lv_image_set_src(driveHead, g_carhead[sel]);
        lv_obj_clear_flag(driveHead, LV_OBJ_FLAG_HIDDEN);
    }

    carX = 205; carY = 380; carHeading = 0; carSpeed = 150;
    carTargetX = 205; carTargetY = 150; carSteer = false;
    driveScore = 0; driveSignals = 0; driveTime = 0; driveActiveEne = 2;
    for (int i = 0; i < DRV_PICK; ++i) driveSpawnPick(i);
    for (int i = 0; i < DRV_ENE; ++i) {
        if (i < driveActiveEne) drivePlaceEnemy(i);
        else { drvEneActive[i] = false; if (drvEne[i]) lv_obj_add_flag(drvEne[i], LV_OBJ_FLAG_HIDDEN); }
    }
    if (driveCar) {
        lv_obj_set_pos(driveCar, (int)carX - 15, (int)carY - 25);
        lv_obj_set_style_transform_rotation(driveCar, 0, 0);
        lv_obj_clear_flag(driveCar, LV_OBJ_FLAG_HIDDEN);
    }
    if (gameScoreLbl) lv_label_set_text(gameScoreLbl, "00000");
    if (gameBestLbl)  lv_label_set_text_fmt(gameBestLbl, "BEST %05d", driveBest);
    gameState = GameState::Play;
    gameShowPick(false);
    if (driveBg) lv_obj_clear_flag(driveBg, LV_OBJ_FLAG_HIDDEN);
    if (driveCar) lv_obj_clear_flag(driveCar, LV_OBJ_FLAG_HIDDEN);
}

void driveDie()
{
    gameState = GameState::Dead;
    if (driveScore > driveBest) { driveBest = driveScore; prefs.putInt("driveBest", driveBest); }
    hapticTap(); sfxDie();
    if (gameOverLbl) lv_label_set_text_fmt(gameOverLbl, "SIGNALS %d   BEST %d", driveScore, driveBest);
    if (driveCar) lv_obj_add_flag(driveCar, LV_OBJ_FLAG_HIDDEN);
    if (driveHead) lv_obj_add_flag(driveHead, LV_OBJ_FLAG_HIDDEN);
    for (int i = 0; i < DRV_PICK; ++i) if (drvPick[i]) lv_obj_add_flag(drvPick[i], LV_OBJ_FLAG_HIDDEN);
    for (int i = 0; i < DRV_ENE; ++i)  if (drvEne[i])  lv_obj_add_flag(drvEne[i], LV_OBJ_FLAG_HIDDEN);
    gameShowPick(true);
}

// Touch anywhere while driving to steer the car toward that point.
void driveInputEvent(lv_event_t *e)
{
    if (gameMode != 1 || currentPage != Page::Game || gameState != GameState::Play) return;
    lv_event_code_t c = lv_event_get_code(e);
    if (c == LV_EVENT_PRESSING || c == LV_EVENT_PRESSED) {
        lv_indev_t *indev = lv_indev_active();
        if (indev) {
            lv_point_t p; lv_indev_get_point(indev, &p);
            carTargetX = p.x; carTargetY = p.y; carSteer = true;
        }
    } else if (c == LV_EVENT_RELEASED || c == LV_EVENT_PRESS_LOST) {
        carSteer = false;
    }
}

void driveTick(float dt, uint32_t now)
{
    driveTime += dt;
    carSpeed = 150.0f + driveTime * 2.0f; if (carSpeed > 275.0f) carSpeed = 275.0f;

    // Steer toward the held finger (limited turn rate); otherwise go straight.
    if (carSteer) {
        float dx = carTargetX - carX, dy = carTargetY - carY;
        if (dx * dx + dy * dy > 64.0f) {
            float ta = atan2f(dx, -dy);          // 0 = pointing up
            float d = ta - carHeading;
            while (d >  3.14159265f) d -= 6.28318531f;
            while (d < -3.14159265f) d += 6.28318531f;
            float maxt = 3.4f * dt;
            if (d >  maxt) d =  maxt;
            if (d < -maxt) d = -maxt;
            carHeading += d;
        }
    }
    carX += sinf(carHeading) * carSpeed * dt;
    carY += -cosf(carHeading) * carSpeed * dt;
    if (carX < -16) carX += 442; if (carX > 426) carX -= 442;
    if (carY < -16) carY += 534; if (carY > 518) carY -= 534;
    if (driveCar) {
        lv_obj_set_pos(driveCar, (int)carX - 15, (int)carY - 25);
        lv_obj_set_style_transform_rotation(driveCar, (int)(carHeading * 572.9578f), 0);
    }

    // Keep the character head riding upright on top of the car.
    if (driveHead) lv_obj_set_pos(driveHead, (int)carX - (g_carhead[gameSel]->header.w / 2),
                                             (int)carY - (g_carhead[gameSel]->header.h / 2) - 4);

    // Pickups: pulse the glow, and collect on contact.
    float pulse = 0.5f + 0.5f * sinf(now * 0.008f);
    for (int i = 0; i < DRV_PICK; ++i) {
        if (!drvPick[i]) continue;
        lv_obj_set_style_shadow_opa(drvPick[i], (uint8_t)(90 + pulse * 150), 0);
        float dx = drvPickX[i] - carX, dy = drvPickY[i] - carY;
        if (dx * dx + dy * dy < 22.0f * 22.0f) {
            driveScore += drvPickBle[i] ? 15 : 10;
            driveSignals++;
            hapticTap(); sfxCollect();
            driveSpawnPick(i);
            // Every few signals, send in another malware (ramps difficulty).
            if (driveActiveEne < DRV_ENE && (driveSignals % 3) == 0) {
                drivePlaceEnemy(driveActiveEne);
                driveActiveEne++;
            }
        }
    }

    // Malware: wander with mild homing toward the car; contact ends the run.
    float emax = 72.0f + driveTime * 1.1f; if (emax > 165.0f) emax = 165.0f;
    for (int i = 0; i < driveActiveEne; ++i) {
        if (!drvEneActive[i]) continue;
        float dx = carX - drvEneX[i], dy = carY - drvEneY[i];
        float dl = sqrtf(dx * dx + dy * dy) + 0.01f;
        drvEneVx[i] += (dx / dl) * 30.0f * dt + ((rand() % 100) - 50) / 50.0f * 10.0f * dt;
        drvEneVy[i] += (dy / dl) * 30.0f * dt + ((rand() % 100) - 50) / 50.0f * 10.0f * dt;
        float es = sqrtf(drvEneVx[i] * drvEneVx[i] + drvEneVy[i] * drvEneVy[i]);
        if (es > emax) { drvEneVx[i] = drvEneVx[i] / es * emax; drvEneVy[i] = drvEneVy[i] / es * emax; }
        drvEneX[i] += drvEneVx[i] * dt; drvEneY[i] += drvEneVy[i] * dt;
        if (drvEneX[i] < -20) drvEneX[i] += 450; if (drvEneX[i] > 430) drvEneX[i] -= 450;
        if (drvEneY[i] < -20) drvEneY[i] += 542; if (drvEneY[i] > 522) drvEneY[i] -= 542;
        if (drvEne[i]) lv_obj_set_pos(drvEne[i], (int)drvEneX[i] - 15, (int)drvEneY[i] - 15);
        if (dx * dx + dy * dy < 21.0f * 21.0f) { driveDie(); return; }
    }

    if (gameScoreLbl) lv_label_set_text_fmt(gameScoreLbl, "%05d", driveScore);
}

void showGamePage()
{
    clearScreen();
    currentPage = Page::Game;
    // Wardrive steers by dragging a finger; stop LVGL from drag-scrolling the
    // whole screen while we do that.
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    for (int i = 0; i < GAME_OBS; ++i) { gameObs[i].img = nullptr; gameObs[i].active = false; }
    gameBg = gameGroundLine = gamePlayer = gameScoreLbl = gameBestLbl = nullptr;
    gamePickPanel = gamePickTitle = gamePickHint = gameOverLbl = nullptr;
    for (int i = 0; i < 4; ++i) gamePickBtn[i] = nullptr;
    for (int i = 0; i < GAME_BLD; ++i) gameBld[i] = nullptr;
    for (int i = 0; i < GAME_STAR; ++i) gameStar[i] = nullptr;
    for (int i = 0; i < GAME_POP; ++i) { gamePop[i] = nullptr; gamePopT[i] = -1.0f; }
    for (int i = 0; i < GAME_BLAST; ++i) { gameBlast[i] = nullptr; gameBlastActive[i] = false; }
    for (int i = 0; i < GAME_EXPL; ++i) { gameExpl[i] = nullptr; gameExplT[i] = -1.0f; }
    driveCar = driveBg = driveHead = nullptr;
    for (int i = 0; i < DRV_PICK; ++i) drvPick[i] = nullptr;
    for (int i = 0; i < DRV_ENE; ++i)  { drvEne[i] = nullptr; drvEneActive[i] = false; }
    for (int i = 0; i < 2; ++i) gameModeBtn[i] = nullptr;
    gameMode = 0;

    gameBest  = prefs.getInt("gameBest", 0);
    driveBest = prefs.getInt("driveBest", 0);
    gameSel  = prefs.getInt("gameChar", 0); if (gameSel < 0 || gameSel > 3) gameSel = 0;

    // Sky background.
    gameBg = lv_obj_create(screen); lv_obj_remove_style_all(gameBg);
    lv_obj_set_size(gameBg, 410, 502); lv_obj_set_pos(gameBg, 0, 0);
    lv_obj_set_style_bg_color(gameBg, lv_color_hex(0x071327), 0);
    lv_obj_set_style_bg_grad_color(gameBg, lv_color_hex(0x04060c), 0);
    lv_obj_set_style_bg_grad_dir(gameBg, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(gameBg, LV_OPA_COVER, 0);
    lv_obj_clear_flag(gameBg, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));

    // Ground.
    lv_obj_t *gfill = lv_obj_create(screen); lv_obj_remove_style_all(gfill);
    lv_obj_set_size(gfill, 410, 502 - GAME_GROUND_Y); lv_obj_set_pos(gfill, 0, GAME_GROUND_Y);
    lv_obj_set_style_bg_color(gfill, lv_color_hex(0x07200c), 0); lv_obj_set_style_bg_opa(gfill, 160, 0);
    lv_obj_clear_flag(gfill, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
    gameGroundLine = lv_obj_create(screen); lv_obj_remove_style_all(gameGroundLine);
    lv_obj_set_size(gameGroundLine, 410, 3); lv_obj_set_pos(gameGroundLine, 0, GAME_GROUND_Y);
    lv_obj_set_style_bg_color(gameGroundLine, lv_color_hex(0x39FF14), 0);
    lv_obj_set_style_bg_opa(gameGroundLine, 255, 0);
    lv_obj_set_style_shadow_color(gameGroundLine, lv_color_hex(0x39FF14), 0);
    lv_obj_set_style_shadow_width(gameGroundLine, 10, 0);
    lv_obj_set_style_shadow_opa(gameGroundLine, 180, 0);
    lv_obj_clear_flag(gameGroundLine, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));

    // Parallax skyline (behind the action) to fill the sky above the runner.
    static const int bldH[GAME_BLD] = { 150, 96, 196, 120, 170, 104, 140 };
    for (int i = 0; i < GAME_BLD; ++i) {
        lv_obj_t *b = lv_obj_create(screen); lv_obj_remove_style_all(b);
        lv_obj_set_size(b, 52, bldH[i]);
        gameBldX[i] = i * 64.0f;
        lv_obj_set_pos(b, (int)gameBldX[i], GAME_GROUND_Y - bldH[i]);
        lv_obj_set_style_bg_color(b, lv_color_hex(0x0C2740), 0);
        lv_obj_set_style_bg_opa(b, 150, 0);
        lv_obj_set_style_border_width(b, 1, 0);
        lv_obj_set_style_border_color(b, lv_color_hex(0x16405E), 0);
        lv_obj_clear_flag(b, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
        gameBld[i] = b;
    }
    static const uint16_t starX[GAME_STAR] = { 30, 90, 150, 212, 270, 330, 384, 60, 300 };
    static const uint16_t starY[GAME_STAR] = { 42, 92, 56, 120, 70, 104, 50, 150, 176 };
    for (int i = 0; i < GAME_STAR; ++i) {
        lv_obj_t *s = lv_obj_create(screen); lv_obj_remove_style_all(s);
        int sz = (i % 3 == 0) ? 4 : 3;
        lv_obj_set_size(s, sz, sz); lv_obj_set_pos(s, starX[i], starY[i]);
        lv_obj_set_style_radius(s, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(s, lv_color_hex(0x9FE8FF), 0);
        lv_obj_set_style_bg_opa(s, 200, 0);
        lv_obj_clear_flag(s, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
        gameStar[i] = s;
    }

    // Wardrive grid map (covers the runner scenery when driving; hidden until
    // the Wardrive mode starts).
    driveBg = lv_obj_create(screen); lv_obj_remove_style_all(driveBg);
    lv_obj_set_size(driveBg, 410, 502); lv_obj_set_pos(driveBg, 0, 0);
    lv_obj_set_style_bg_color(driveBg, lv_color_hex(0x05080f), 0);
    lv_obj_set_style_bg_grad_color(driveBg, lv_color_hex(0x0A0F1C), 0);
    lv_obj_set_style_bg_grad_dir(driveBg, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(driveBg, LV_OPA_COVER, 0);
    lv_obj_clear_flag(driveBg, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
    for (int gx = 1; gx < 6; ++gx) {
        lv_obj_t *v = lv_obj_create(driveBg); lv_obj_remove_style_all(v);
        lv_obj_set_size(v, 1, 502); lv_obj_set_pos(v, gx * 68, 0);
        lv_obj_set_style_bg_color(v, lv_color_hex(0x13304A), 0);
        lv_obj_set_style_bg_opa(v, 120, 0);
    }
    for (int gy = 1; gy < 8; ++gy) {
        lv_obj_t *h = lv_obj_create(driveBg); lv_obj_remove_style_all(h);
        lv_obj_set_size(h, 410, 1); lv_obj_set_pos(h, 0, gy * 64);
        lv_obj_set_style_bg_color(h, lv_color_hex(0x13304A), 0);
        lv_obj_set_style_bg_opa(h, 120, 0);
    }
    lv_obj_add_flag(driveBg, LV_OBJ_FLAG_HIDDEN);

    // Wardrive pickups - real Wi-Fi / Bluetooth icons to catch.
    for (int i = 0; i < DRV_PICK; ++i) {
        lv_obj_t *p = lv_label_create(screen);
        lv_label_set_text(p, LV_SYMBOL_WIFI);
        lv_obj_set_style_text_font(p, &lv_font_montserrat_22, 0);
        lv_obj_set_style_text_color(p, lv_color_hex(0x3AEFFF), 0);
        lv_obj_set_style_shadow_color(p, lv_color_hex(0x3AEFFF), 0);
        lv_obj_set_style_shadow_width(p, 12, 0);
        lv_obj_set_style_shadow_opa(p, 180, 0);
        lv_obj_add_flag(p, LV_OBJ_FLAG_HIDDEN);
        drvPick[i] = p;
    }
    // Malware enemies - the spiky mine sprite.
    for (int i = 0; i < DRV_ENE; ++i) {
        lv_obj_t *en = lv_image_create(screen);
        lv_image_set_src(en, &g_malware);
        lv_obj_add_flag(en, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(en, LV_OBJ_FLAG_CLICKABLE);
        drvEne[i] = en;
    }

    // Signal-burst ring pool (Wi-Fi/BLE pops in the sky).
    for (int i = 0; i < GAME_POP; ++i) {
        lv_obj_t *p = lv_obj_create(screen); lv_obj_remove_style_all(p);
        lv_obj_set_size(p, 8, 8);
        lv_obj_set_style_radius(p, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_opa(p, 0, 0);
        lv_obj_set_style_border_width(p, 3, 0);
        lv_obj_set_style_border_color(p, lv_color_hex(0x3AEFFF), 0);
        lv_obj_add_flag(p, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(p, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
        gamePop[i] = p;
    }
    // Blast pool (fired while airborne) - per-character projectile sprite.
    for (int i = 0; i < GAME_BLAST; ++i) {
        lv_obj_t *bz = lv_image_create(screen);
        lv_image_set_src(bz, g_blast[gameSel]);
        lv_obj_add_flag(bz, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(bz, LV_OBJ_FLAG_CLICKABLE);
        gameBlast[i] = bz;
    }
    // Explosion pool (red/white/black flash on a popped bubble).
    for (int i = 0; i < GAME_EXPL; ++i) {
        lv_obj_t *x = lv_obj_create(screen); lv_obj_remove_style_all(x);
        lv_obj_set_size(x, 12, 12);
        lv_obj_set_style_radius(x, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(x, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(x, 0, 0);
        lv_obj_set_style_border_width(x, 2, 0);
        lv_obj_set_style_border_color(x, lv_color_hex(0x111111), 0);
        lv_obj_add_flag(x, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(x, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
        gameExpl[i] = x;
    }

    // Obstacle pool.
    for (int i = 0; i < GAME_OBS; ++i) {
        lv_obj_t *im = lv_image_create(screen);
        lv_image_set_src(im, &g_shard_s);
        lv_obj_add_flag(im, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(im, LV_OBJ_FLAG_CLICKABLE);
        gameObs[i].img = im; gameObs[i].active = false;
    }

    // Player.
    gamePlayer = lv_image_create(screen);
    lv_image_set_src(gamePlayer, g_run0[gameSel]);
    lv_obj_add_flag(gamePlayer, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(gamePlayer, LV_OBJ_FLAG_CLICKABLE);

    // Wardrive car (built hidden; shown when Wardrive mode starts).
    buildDriveCar(gameSel);

    // Score + best.
    gameScoreLbl = lv_label_create(screen); lv_label_set_text(gameScoreLbl, "00000");
    lv_obj_set_style_text_font(gameScoreLbl, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(gameScoreLbl, lv_color_hex(0x3AEFFF), 0);
    lv_obj_align(gameScoreLbl, LV_ALIGN_TOP_MID, 0, 16);
    gameBestLbl = lv_label_create(screen); lv_label_set_text_fmt(gameBestLbl, "BEST %05d", gameBest);
    lv_obj_set_style_text_font(gameBestLbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(gameBestLbl, lv_color_hex(0x6F86A6), 0);
    lv_obj_align(gameBestLbl, LV_ALIGN_TOP_MID, 0, 44);

    // Full-screen transparent jump button (under the pick UI / back button).
    lv_obj_t *jb = lv_button_create(screen); lv_obj_remove_style_all(jb);
    lv_obj_set_size(jb, 410, 502); lv_obj_set_pos(jb, 0, 0);
    lv_obj_clear_flag(jb, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(jb, gameJumpEvent, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(jb, driveInputEvent, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(jb, driveInputEvent, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(jb, driveInputEvent, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(jb, driveInputEvent, LV_EVENT_PRESS_LOST, nullptr);

    // Pick / game-over overlay.
    gamePickPanel = lv_obj_create(screen); lv_obj_remove_style_all(gamePickPanel);
    lv_obj_set_size(gamePickPanel, 410, 502); lv_obj_set_pos(gamePickPanel, 0, 0);
    lv_obj_set_style_bg_color(gamePickPanel, lv_color_hex(0x04080e), 0);
    lv_obj_set_style_bg_opa(gamePickPanel, 210, 0);
    lv_obj_clear_flag(gamePickPanel, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
    gamePickTitle = lv_label_create(screen); lv_label_set_text(gamePickTitle, "M33K ARCADE");
    lv_obj_set_style_text_font(gamePickTitle, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(gamePickTitle, lv_color_hex(0x3AEFFF), 0);
    lv_obj_align(gamePickTitle, LV_ALIGN_TOP_MID, 0, 70);
    gameOverLbl = lv_label_create(screen); lv_label_set_text(gameOverLbl, "");
    lv_obj_set_style_text_font(gameOverLbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(gameOverLbl, lv_color_hex(0xDCECFF), 0);
    lv_obj_align(gameOverLbl, LV_ALIGN_TOP_MID, 0, 116);
    gamePickHint = lv_label_create(screen); lv_label_set_text(gamePickHint, "TAP A CHARACTER TO PLAY");
    lv_obj_set_style_text_font(gamePickHint, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(gamePickHint, lv_color_hex(0x39FF14), 0);
    lv_obj_align(gamePickHint, LV_ALIGN_TOP_MID, 0, 150);

    // Four character buttons.
    const int cx4[4] = { 54, 152, 256, 356 };
    const char *nm4[4] = { "M33K", "GRUMPY", "MONKIE", "KITTIE" };
    for (int i = 0; i < 4; ++i) {
        lv_obj_t *b = lv_button_create(screen);
        lv_obj_set_size(b, 86, 160); lv_obj_set_pos(b, cx4[i] - 43, 210);
        lv_obj_set_style_radius(b, 14, 0);
        lv_obj_set_style_bg_color(b, lv_color_hex(0x0A1726), 0);
        lv_obj_set_style_bg_opa(b, 220, 0);
        lv_obj_set_style_border_width(b, i == gameSel ? 3 : 1, 0);
        lv_obj_set_style_border_color(b, lv_color_hex(i == gameSel ? 0x39FF14 : 0x214058), 0);
        lv_obj_add_event_cb(b, gamePickEvent, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        lv_obj_t *im = lv_image_create(b);
        const lv_image_dsc_t *dsc = g_pick[i];   // hero portrait, not the run frame
        lv_image_set_src(im, dsc);
        lv_image_set_pivot(im, dsc->header.w / 2, dsc->header.h / 2);
        lv_image_set_scale(im, 300);          // portraits are 64 px tall -> ~75 px
        lv_obj_align(im, LV_ALIGN_TOP_MID, 0, 36);
        lv_obj_clear_flag(im, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_t *l = lv_label_create(b); lv_label_set_text(l, nm4[i]);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(i == gameSel ? 0x39FF14 : 0x9FB6CC), 0);
        lv_obj_align(l, LV_ALIGN_BOTTOM_MID, 0, -8);
        gamePickBtn[i] = b;
    }

    // Game-mode toggle (Runner / Wardrive), above the character row.
    const char *mnm[2] = { "RUNNER", "WARDRIVE" };
    const int mx[2] = { 58, 212 };
    for (int i = 0; i < 2; ++i) {
        lv_obj_t *mb = lv_button_create(screen);
        lv_obj_set_size(mb, 140, 30); lv_obj_set_pos(mb, mx[i], 176);
        lv_obj_set_style_radius(mb, 8, 0);
        bool on = (i == gameMode);
        lv_obj_set_style_bg_color(mb, lv_color_hex(on ? 0x113022 : 0x0A1726), 0);
        lv_obj_set_style_bg_opa(mb, 230, 0);
        lv_obj_set_style_border_width(mb, on ? 3 : 1, 0);
        lv_obj_set_style_border_color(mb, lv_color_hex(on ? 0x39FF14 : 0x214058), 0);
        lv_obj_add_event_cb(mb, gameModeEvent, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        lv_obj_t *ml = lv_label_create(mb); lv_label_set_text(ml, mnm[i]);
        lv_obj_set_style_text_font(ml, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(ml, lv_color_hex(0xDCECFF), 0);
        lv_obj_center(ml);
        gameModeBtn[i] = mb;
    }

    // Back button (top-left, above everything so it always works).
    lv_obj_t *back = lv_button_create(screen);
    lv_obj_set_size(back, 50, 44); lv_obj_set_pos(back, 26, 20);   // in from the corner
    lv_obj_set_style_radius(back, 12, 0);
    lv_obj_set_style_bg_color(back, lv_color_hex(0x10091D), 0);
    lv_obj_set_style_border_width(back, 1, 0);
    lv_obj_set_style_border_color(back, lv_color_hex(0xB37BFF), 0);
    lv_obj_add_event_cb(back, backEvent, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *bl = lv_label_create(back); lv_label_set_text(bl, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(bl, lv_color_hex(0xD9C2FF), 0); lv_obj_center(bl);

    gameState = GameState::Pick;
    gameShowPick(true);
    gameAudioBegin();                 // power the speaker for arcade SFX
    gameTimer = lv_timer_create(gameTimerCb, 24, nullptr);
    noteActivity();
    Serial.println("[M33K] Game open");
}

// Called from clearPageTimers() on every page teardown (after gameTimer is
// deleted and before the screen's objects are freed): reset state + pointers.
void stopGameTool()
{
    gameAudioEnd();                   // release the speaker when leaving the game
    gameState = GameState::Pick;
    for (int i = 0; i < GAME_OBS; ++i) { gameObs[i].img = nullptr; gameObs[i].active = false; }
    gameBg = gameGroundLine = gamePlayer = gameScoreLbl = gameBestLbl = nullptr;
    gamePickPanel = gamePickTitle = gamePickHint = gameOverLbl = nullptr;
    for (int i = 0; i < 4; ++i) gamePickBtn[i] = nullptr;
    for (int i = 0; i < GAME_BLD; ++i) gameBld[i] = nullptr;
    for (int i = 0; i < GAME_STAR; ++i) gameStar[i] = nullptr;
    for (int i = 0; i < GAME_POP; ++i) { gamePop[i] = nullptr; gamePopT[i] = -1.0f; }
    for (int i = 0; i < GAME_BLAST; ++i) { gameBlast[i] = nullptr; gameBlastActive[i] = false; }
    for (int i = 0; i < GAME_EXPL; ++i) { gameExpl[i] = nullptr; gameExplT[i] = -1.0f; }
    driveCar = driveBg = driveHead = nullptr;
    for (int i = 0; i < DRV_PICK; ++i) drvPick[i] = nullptr;
    for (int i = 0; i < DRV_ENE; ++i)  { drvEne[i] = nullptr; drvEneActive[i] = false; }
    for (int i = 0; i < 2; ++i) gameModeBtn[i] = nullptr;
}

// Place a bubble: random x, a drift speed and a gentle sway. On the initial
// build they are scattered up the screen; afterwards they respawn below it.
void bubbleRespawn(int i, bool initial)
{
    BubbleEntity &b = gBubbles[i];
    const int span = 410 - b.size - 16;
    b.x       = 8 + (span > 0 ? (rand() % span) : 0);
    b.y       = initial ? (float)(rand() % 502) : 502.0f + (rand() % 160);
    // Per-tick speeds scaled for the 33 ms tick (same on-screen px/sec as the
    // old 50 ms tick, just smaller, smoother steps). Bigger bubbles drift
    // slower (parallax).
    b.speed   = 2.5f - (float)(b.size - 88) / 32.0f * 0.86f + (rand() % 30) / 150.0f;
    if (b.speed < 1.2f) b.speed = 1.2f;
    b.phase   = (rand() % 628) / 100.0f;
    b.swayAmp = 2.5f + (rand() % 4);   // gentle side drift
    if (b.obj) lv_obj_set_pos(b.obj, (int)b.x, (int)b.y);
}

void buildBubblesFace()
{
    srand(millis());

    lv_obj_t *bg = lv_obj_create(screen);
    lv_obj_remove_style_all(bg);
    lv_obj_set_size(bg, 410, 502);
    lv_obj_set_pos(bg, 0, 0);
    lv_obj_set_style_bg_color(bg, lv_color_hex(0x070612), 0);
    lv_obj_set_style_bg_grad_color(bg, lv_color_hex(0x16092A), 0);
    lv_obj_set_style_bg_grad_dir(bg, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(bg, LV_OPA_COVER, 0);

    const uint32_t ringCols[4] = { 0x3AEFFF, 0xFF52C8, 0x9D7BFF, 0x45FF8C };

    // Background layer: small faint bubbles that fall and wrap at the top.
    for (int i = 0; i < BG_BUBBLE_COUNT; ++i) {
        BubbleEntity &b = gBgBubbles[i];
        const int sz  = 7 + (i * 11) % 16;     // ~7..22 px
        const uint32_t col = ringCols[i % 4];
        b.size    = sz;
        b.x       = rand() % 410;
        b.y       = rand() % 502;
        b.speed   = 3.2f + (rand() % 100) / 55.0f;    // a bit faster, still gentle
        b.phase   = (rand() % 628) / 100.0f;
        b.swayAmp = 2.0f + (rand() % 5);

        lv_obj_t *d = lv_obj_create(screen);
        lv_obj_remove_style_all(d);
        lv_obj_set_size(d, sz, sz);
        lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(d, lv_color_hex(col), 0);
        lv_obj_set_style_bg_opa(d, 40, 0);
        lv_obj_set_style_border_width(d, 1, 0);
        lv_obj_set_style_border_color(d, lv_color_hex(col), 0);
        lv_obj_set_style_border_opa(d, 120, 0);
        lv_obj_clear_flag(d, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
        lv_obj_set_pos(d, (int)b.x, (int)b.y);
        b.obj = d;
    }

    // Character layer: portraits float up in neon-ringed bubbles. Per-bubble
    // size matches the stored image size (no runtime scaling).
    //                            kA   kB   mon  t3ch grmp
    static const int bSize[5] = { 88, 100,  92, 106, 110 };

    for (int i = 0; i < BUBBLE_COUNT; ++i) {
        BubbleEntity &b = gBubbles[i];
        const int sz = (i < 5) ? bSize[i] : 100;
        const uint32_t ring = ringCols[i % 4];
        b.size = sz;
        b.glowL = b.glowR = nullptr;

        lv_obj_t *bub = lv_obj_create(screen);
        lv_obj_remove_style_all(bub);
        lv_obj_set_size(bub, sz, sz);
        lv_obj_set_style_radius(bub, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_clip_corner(bub, true, 0);
        lv_obj_set_style_border_width(bub, 3, 0);
        lv_obj_set_style_border_color(bub, lv_color_hex(ring), 0);
        lv_obj_set_style_border_opa(bub, 210, 0);
        // Neon glow halo around the ring. Kept small so the ~30 fps motion
        // stays smooth (a wide blurred shadow is expensive to redraw each frame).
        lv_obj_set_style_shadow_width(bub, 8, 0);
        lv_obj_set_style_shadow_spread(bub, 0, 0);
        lv_obj_set_style_shadow_color(bub, lv_color_hex(ring), 0);
        lv_obj_set_style_shadow_opa(bub, 160, 0);
        lv_obj_set_style_bg_color(bub, lv_color_hex(0x05040C), 0);
        lv_obj_set_style_bg_opa(bub, LV_OPA_COVER, 0);
        lv_obj_clear_flag(bub, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));

        lv_obj_t *img = lv_image_create(bub);
        lv_image_set_src(img, m33k_bubble_imgs[i % m33k_bubble_img_count]);
        lv_obj_center(img);
        lv_obj_clear_flag(img, LV_OBJ_FLAG_CLICKABLE);

        // Glossy highlight dot in the upper-left for a bubble look.
        lv_obj_t *sh = lv_obj_create(bub);
        lv_obj_remove_style_all(sh);
        const int shd = sz / 5;
        lv_obj_set_size(sh, shd, shd);
        lv_obj_set_style_radius(sh, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(sh, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(sh, 130, 0);
        lv_obj_align(sh, LV_ALIGN_TOP_LEFT, sz / 6, sz / 6);
        lv_obj_clear_flag(sh, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));

        b.obj = bub;
        bubbleRespawn(i, true);
    }

    createTopBar();
    buildFaceClock(205);   // clock + date centered on this face
}

// ---------------------------------------------------------------------------
// Weather face helpers
// ---------------------------------------------------------------------------

// Pull the first number that follows "key": in a JSON blob. Skips an optional
// '[' so it also reads the first element of an array (daily min/max).
static float weatherJsonNum(const String &s, const char *key)
{
    // Open-Meteo lists each field twice: once in a *_units object with a string
    // value (e.g. "temperature_2m":"°F") and once with the real number.
    // Skip the string (units) match and return the first numeric one.
    String k = String("\"") + key + "\":";
    int from = 0;
    while (true) {
        int i = s.indexOf(k, from);
        if (i < 0) return 0.0f;
        int j = i + k.length();
        while (j < (int)s.length() && (s[j] == ' ' || s[j] == '[')) ++j;
        if (j < (int)s.length() && s[j] == '"') { from = j + 1; continue; }  // units string
        return s.substring(j, j + 16).toFloat();
    }
}

static String weatherJsonStr(const String &s, const char *key)
{
    String k = String("\"") + key + "\":\"";
    int i = s.indexOf(k);
    if (i < 0) return "";
    i += k.length();
    int j = s.indexOf('"', i);
    if (j < 0) return "";
    return s.substring(i, j);
}

// WMO weather-code -> short label.
static const char *weatherCondText(int code)
{
    if (code == 0) return "CLEAR";
    if (code == 1 || code == 2) return "PARTLY CLOUDY";
    if (code == 3) return "CLOUDY";
    if (code == 45 || code == 48) return "FOG";
    if (code >= 51 && code <= 57) return "DRIZZLE";
    if (code >= 61 && code <= 67) return "RAIN";
    if (code >= 71 && code <= 77) return "SNOW";
    if (code >= 80 && code <= 82) return "SHOWERS";
    if (code >= 85 && code <= 86) return "SNOW SHOWERS";
    if (code >= 95) return "THUNDERSTORM";
    return "--";
}

static uint32_t weatherCondColor(int code)
{
    if (code == 0) return 0xFFD23A;                 // clear - yellow
    if (code >= 1 && code <= 3) return 0xCFE8FF;     // cloudy - pale blue
    if (code == 45 || code == 48) return 0x9FB4C8;   // fog - grey
    if (code >= 51 && code <= 67) return 0x3AEFFF;    // rain - cyan
    if (code >= 71 && code <= 86) return 0xFFFFFF;    // snow - white
    if (code >= 95) return 0xB388FF;                  // storm - violet
    return 0xCFE8FF;
}

// Persist the last reading so it survives a reboot / power-off.
void saveWeatherCache()
{
    prefs.putInt("wxT",  (int)lroundf(weatherTempF));
    prefs.putInt("wxHi", (int)lroundf(weatherHiF));
    prefs.putInt("wxLo", (int)lroundf(weatherLoF));
    prefs.putInt("wxCode", weatherCode);
    prefs.putInt("wxWind", (int)lroundf(weatherWindMph));
    prefs.putString("wxPlace", weatherPlace);
    prefs.putString("wxUpd", weatherUpdatedStr);
}

void loadWeatherCache()
{
    if (!prefs.isKey("wxUpd")) return;           // nothing saved yet
    weatherTempF = prefs.getInt("wxT", 0);
    weatherHiF   = prefs.getInt("wxHi", 0);
    weatherLoF   = prefs.getInt("wxLo", 0);
    weatherCode  = prefs.getInt("wxCode", -1);
    weatherWindMph = prefs.getInt("wxWind", 0);
    weatherPlace = prefs.getString("wxPlace", "");
    weatherUpdatedStr = prefs.getString("wxUpd", "");
    weatherHaveData = true;
}

// Show current data (from a fresh fetch or the saved cache) plus a status line.
void applyWeatherToUi()
{
    if (weatherHaveData) {
        if (weatherTempLabel)
            lv_label_set_text_fmt(weatherTempLabel, "%d\xC2\xB0", (int)lroundf(weatherTempF));
        if (weatherCondLabel) {
            lv_label_set_text(weatherCondLabel, weatherCondText(weatherCode));
            lv_obj_set_style_text_color(weatherCondLabel,
                                        lv_color_hex(weatherCondColor(weatherCode)), 0);
        }
        if (weatherHiLoLabel)
            lv_label_set_text_fmt(weatherHiLoLabel, "H %d\xC2\xB0   L %d\xC2\xB0",
                                  (int)lroundf(weatherHiF), (int)lroundf(weatherLoF));
        if (weatherPlaceLabel)
            lv_label_set_text(weatherPlaceLabel,
                              weatherPlace.length() ? weatherPlace.c_str() : "");
    }
    if (weatherStatusLabel && weatherHaveData) {
        if (weatherState == WeatherState::Ready)
            lv_label_set_text_fmt(weatherStatusLabel, "Updated %s", weatherUpdatedStr.c_str());
        else if (weatherState == WeatherState::NoWifi || weatherState == WeatherState::Error)
            lv_label_set_text_fmt(weatherStatusLabel, "Offline - last update %s",
                                  weatherUpdatedStr.c_str());
        else   // Idle with cached data (cold boot), before the first fetch
            lv_label_set_text_fmt(weatherStatusLabel, "Last update %s",
                                  weatherUpdatedStr.c_str());
    }
}

// Blocking weather fetch. Serviced from the main loop (never from an LVGL
// timer) so a slow network call cannot stall the LVGL task callbacks.
void doWeatherFetch()
{
    weatherState = WeatherState::Updating;
    if (weatherStatusLabel) lv_label_set_text(weatherStatusLabel, "Updating weather...");
    Serial.printf("[M33K WX] fetch start | wifi=%d ssidLen=%d heap=%u\n",
                  (int)WiFi.status(), (int)savedWifiSsid.length(),
                  (unsigned)ESP.getFreeHeap());

    // 1) Ensure Wi-Fi is up (reuses the saved credentials, same as NTP sync).
    if (WiFi.status() != WL_CONNECTED) {
        if (savedWifiSsid.length() == 0) {
            weatherState = WeatherState::NoWifi;
            if (weatherHaveData) applyWeatherToUi();
            else if (weatherStatusLabel)
                lv_label_set_text(weatherStatusLabel, "Set Wi-Fi in Settings for weather");
            Serial.println("[M33K WX] no saved Wi-Fi credentials");
            return;
        }
        WiFi.mode(WIFI_STA);
        WiFi.begin(savedWifiSsid.c_str(), savedWifiPassword.c_str());
        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 6000) {
            instance.loop();
            lv_timer_handler();
            delay(40);
        }
        if (WiFi.status() != WL_CONNECTED) {
            // Stop the background retry so WiFi isn't left perpetually
            // "connecting" (which spams the log and breaks the Settings scan).
            WiFi.setAutoReconnect(false);
            WiFi.disconnect(false, false);
            weatherState = WeatherState::NoWifi;
            if (weatherHaveData) applyWeatherToUi();   // keep showing cached reading
            else if (weatherStatusLabel)
                lv_label_set_text(weatherStatusLabel, "Offline - no Wi-Fi");
            Serial.printf("[M33K WX] wifi connect failed (status=%d)\n", (int)WiFi.status());
            return;
        }
        Serial.println("[M33K WX] wifi connected");
    }

    // 2) Location: GPS fix if we have a recent one, else runtime IP lookup.
    //    No coordinates are ever stored in source - this keeps the public repo
    //    free of any location data.
    double lat = 0, lon = 0;
    bool haveLoc = false;
    String place = "";
    if (gpsParser.location.isValid() && gpsParser.location.age() < 120000UL) {
        lat = gpsParser.location.lat();
        lon = gpsParser.location.lng();
        haveLoc = true;
    } else {
        HTTPClient http;
        http.setConnectTimeout(5000);
        http.setTimeout(5000);
        if (http.begin("http://ip-api.com/json/?fields=status,lat,lon,city")) {
            if (http.GET() == 200) {
                String p = http.getString();
                lat = weatherJsonNum(p, "lat");
                lon = weatherJsonNum(p, "lon");
                place = weatherJsonStr(p, "city");
                haveLoc = (lat != 0.0 || lon != 0.0);
            }
            http.end();
        }
    }
    if (!haveLoc) {
        weatherState = WeatherState::Error;
        if (weatherHaveData) applyWeatherToUi();
        else if (weatherStatusLabel) lv_label_set_text(weatherStatusLabel, "Couldn't get location");
        Serial.println("[M33K WX] location lookup failed");
        return;
    }
    Serial.printf("[M33K WX] location ok (place=%s)\n",
                  place.length() ? place.c_str() : "GPS");

    // 3) Weather from Open-Meteo (HTTPS, no API key).
    char url[384];
    snprintf(url, sizeof(url),
             "https://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f"
             "&current=temperature_2m,weather_code,is_day,wind_speed_10m"
             "&daily=temperature_2m_max,temperature_2m_min"
             "&temperature_unit=fahrenheit&wind_speed_unit=mph"
             "&timezone=auto&forecast_days=1",
             lat, lon);

    WiFiClientSecure secure;
    secure.setInsecure();   // Open-Meteo is HTTPS; we don't pin a cert here.
    HTTPClient https;
    https.setConnectTimeout(6000);
    https.setTimeout(7000);
    String payload = "";
    int httpCode = 0;
    if (https.begin(secure, url)) {
        httpCode = https.GET();
        if (httpCode == 200) payload = https.getString();
        https.end();
    } else {
        Serial.println("[M33K WX] https.begin failed");
    }
    Serial.printf("[M33K WX] open-meteo code=%d len=%d heap=%u\n",
                  httpCode, (int)payload.length(), (unsigned)ESP.getFreeHeap());
    if (payload.length() < 10) {
        weatherState = WeatherState::Error;
        if (weatherHaveData) applyWeatherToUi();
        else if (weatherStatusLabel) lv_label_set_text(weatherStatusLabel, "Weather fetch failed");
        return;
    }

    weatherTempF = weatherJsonNum(payload, "temperature_2m");
    weatherCode  = (int)weatherJsonNum(payload, "weather_code");
    weatherIsDay = weatherJsonNum(payload, "is_day") >= 0.5f;
    weatherWindMph = weatherJsonNum(payload, "wind_speed_10m");
    weatherHiF   = weatherJsonNum(payload, "temperature_2m_max");
    weatherLoF   = weatherJsonNum(payload, "temperature_2m_min");
    weatherPlace = place;   // "" when from GPS; that's fine
    weatherLastFetchMs = millis();
    weatherState = WeatherState::Ready;
    weatherHaveData = true;

    // Timestamp from the RTC for the "Updated" line, and persist to NVS so the
    // reading survives a power-off.
    {
        RTC_DateTime d = instance.rtc.getDateTime();
        uint8_t h = d.getHour(), dh = h;
        if (!use24Hour) { dh = h % 12; if (dh == 0) dh = 12; }
        char buf[24];
        snprintf(buf, sizeof(buf), "%u/%u %u:%02u%s", d.getMonth(), d.getDay(),
                 dh, d.getMinute(), use24Hour ? "" : (h < 12 ? "a" : "p"));
        weatherUpdatedStr = buf;
    }
    saveWeatherCache();
    applyWeatherToUi();

    Serial.printf("[M33K WX] %d F  code=%d  hi=%d lo=%d\n",
                  (int)lroundf(weatherTempF), weatherCode,
                  (int)lroundf(weatherHiF), (int)lroundf(weatherLoF));
}

// Called from the main loop. Runs a pending fetch, and refreshes periodically
// while the weather face is showing.
void serviceWeatherFetch()
{
    if (currentPage != Page::Home || homeFaceIndex != 5 || screenSleeping) return;

    if (weatherFetchPending) {
        weatherFetchPending = false;
        doWeatherFetch();
        return;
    }
    if (weatherState == WeatherState::Ready &&
        millis() - weatherLastFetchMs > WEATHER_REFRESH_MS) {
        doWeatherFetch();
    }
}

void buildWeatherFace()
{
    // Animated backdrop (storm/lightning <-> sunshine), colored each tick in
    // faceAnimTick. Created first so everything else sits on top.
    wxBg = lv_obj_create(screen);
    lv_obj_remove_style_all(wxBg);
    lv_obj_set_size(wxBg, 410, 502);
    lv_obj_set_pos(wxBg, 0, 0);
    lv_obj_set_style_bg_color(wxBg, lv_color_hex(0x0A1024), 0);
    lv_obj_set_style_bg_grad_color(wxBg, lv_color_hex(0x05080F), 0);
    lv_obj_set_style_bg_grad_dir(wxBg, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(wxBg, LV_OPA_COVER, 0);

    // Sun (hidden during the storm phase): glowing yellow disc, upper area.
    wxSun = lv_obj_create(screen);
    lv_obj_remove_style_all(wxSun);
    lv_obj_set_size(wxSun, 74, 74);
    lv_obj_align(wxSun, LV_ALIGN_TOP_RIGHT, -34, 132);  // upper-right, clear of text
    lv_obj_set_style_radius(wxSun, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(wxSun, lv_color_hex(0xFFD23A), 0);
    lv_obj_set_style_bg_opa(wxSun, LV_OPA_COVER, 0);
    lv_obj_set_style_shadow_color(wxSun, lv_color_hex(0xFFC21A), 0);
    lv_obj_set_style_shadow_width(wxSun, 40, 0);
    lv_obj_set_style_shadow_spread(wxSun, 6, 0);
    lv_obj_set_style_opa(wxSun, 0, 0);
    lv_obj_clear_flag(wxSun, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));

    // Moon (clear night): a pale disc with a sky-coloured disc offset over it
    // to carve a crescent. The carve matches the clear-night sky colour.
    wxMoon = lv_obj_create(screen);
    lv_obj_remove_style_all(wxMoon);
    lv_obj_set_size(wxMoon, 60, 60);
    lv_obj_align(wxMoon, LV_ALIGN_TOP_RIGHT, -40, 136);
    lv_obj_set_style_radius(wxMoon, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(wxMoon, lv_color_hex(0xE8EEF7), 0);
    lv_obj_set_style_bg_opa(wxMoon, LV_OPA_COVER, 0);
    lv_obj_set_style_shadow_color(wxMoon, lv_color_hex(0xBFCBE0), 0);
    lv_obj_set_style_shadow_width(wxMoon, 24, 0);
    lv_obj_set_style_shadow_spread(wxMoon, 3, 0);
    lv_obj_set_style_opa(wxMoon, 0, 0);
    lv_obj_clear_flag(wxMoon, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
    {
        lv_obj_t *mc = lv_obj_create(wxMoon);
        lv_obj_remove_style_all(mc);
        lv_obj_set_size(mc, 52, 52);
        lv_obj_set_pos(mc, 18, -4);
        lv_obj_set_style_radius(mc, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(mc, lv_color_hex(0x0A1430), 0);
        lv_obj_set_style_bg_opa(mc, LV_OPA_COVER, 0);
        lv_obj_clear_flag(mc, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
    }
    // Stars (clear night): small dots scattered in the sky, twinkling.
    static const uint16_t starX[WX_STAR_N] = { 26, 72, 118, 168, 214, 262, 310, 352, 388, 96 };
    static const uint16_t starY[WX_STAR_N] = { 70, 150, 92, 196, 62, 168, 108, 210, 84, 262 };
    for (int i = 0; i < WX_STAR_N; ++i) {
        lv_obj_t *s = lv_obj_create(screen);
        lv_obj_remove_style_all(s);
        int sz = (i % 3 == 0) ? 4 : 3;
        lv_obj_set_size(s, sz, sz);
        lv_obj_set_pos(s, starX[i], starY[i]);
        lv_obj_set_style_radius(s, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(s, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(s, LV_OPA_COVER, 0);
        lv_obj_set_style_opa(s, 0, 0);
        lv_obj_clear_flag(s, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
        wxStars[i] = s;
    }

    // Fluffy clouds (sunny phase): each is a cluster of soft white blobs.
    auto makeCloud = [&](int w, int h) -> lv_obj_t * {
        lv_obj_t *cl = lv_obj_create(screen);
        lv_obj_remove_style_all(cl);
        lv_obj_set_size(cl, w, h);
        lv_obj_clear_flag(cl, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
        const int bw[4] = { h, (int)(h * 1.4f), (int)(h * 1.2f), h };
        const int bx[4] = { 0, (int)(w * 0.22f), (int)(w * 0.5f), w - h };
        for (int k = 0; k < 4; ++k) {
            lv_obj_t *blob = lv_obj_create(cl);
            lv_obj_remove_style_all(blob);
            lv_obj_set_size(blob, bw[k], bw[k]);
            lv_obj_set_pos(blob, bx[k], h - bw[k] + 2);
            lv_obj_set_style_radius(blob, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_bg_color(blob, lv_color_hex(0xEEF3FF), 0);
            lv_obj_set_style_bg_opa(blob, LV_OPA_COVER, 0);
        }
        lv_obj_set_style_opa(cl, 0, 0);
        return cl;
    };
    // Kept in the sky band above the temperature so they never cross it.
    wxCloud1 = makeCloud(62, 22);
    wxCloud2 = makeCloud(46, 16);
    wxCloud1X = 40;  lv_obj_set_pos(wxCloud1, (int)wxCloud1X, 126);
    wxCloud2X = 236; lv_obj_set_pos(wxCloud2, (int)wxCloud2X, 132);

    // Fog (fog phase): two wide soft grey bands drifting opposite ways, kept in
    // the sky band so they don't wash over the temperature reading.
    auto makeFog = [&](int y, int w, int h, uint32_t col) -> lv_obj_t * {
        lv_obj_t *f = lv_obj_create(screen);
        lv_obj_remove_style_all(f);
        lv_obj_set_size(f, w, h);
        lv_obj_set_pos(f, 0, y);
        lv_obj_set_style_radius(f, h / 2, 0);
        lv_obj_set_style_bg_color(f, lv_color_hex(col), 0);
        lv_obj_set_style_bg_opa(f, LV_OPA_COVER, 0);
        lv_obj_set_style_opa(f, 0, 0);
        lv_obj_clear_flag(f, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
        return f;
    };
    wxFog1 = makeFog(110, 330, 24, 0xC2CCD6);
    wxFog2 = makeFog(134, 300, 20, 0xAEB9C6);
    wxFog1X = -40; wxFog2X = 70;

    // A small white bird gliding across the sky, wings flapping.
    static lv_point_precise_t birdUp[5]   = { {0,9},{11,0},{22,9},{33,0},{44,9} };
    wxBird = lv_line_create(screen);
    lv_line_set_points(wxBird, birdUp, 5);
    lv_obj_set_style_line_color(wxBird, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_line_width(wxBird, 3, 0);
    lv_obj_set_style_line_rounded(wxBird, true, 0);
    lv_obj_set_style_opa(wxBird, 0, 0);
    wxBirdX = -50; wxBirdY = 118;
    lv_obj_set_pos(wxBird, (int)wxBirdX, wxBirdY);

    // Lightning bolt (hidden until a flash): a white zig-zag line.
    static lv_point_precise_t boltPts[6] = {
        {214, 120}, {190, 190}, {210, 190}, {184, 272}, {232, 182}, {210, 182}
    };
    wxBolt = lv_line_create(screen);
    lv_line_set_points(wxBolt, boltPts, 6);
    lv_obj_set_style_line_color(wxBolt, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_line_width(wxBolt, 5, 0);
    lv_obj_set_style_line_rounded(wxBolt, true, 0);
    lv_obj_set_style_opa(wxBolt, 0, 0);

    // Full-screen flash overlay for the lightning strobe.
    wxFlash = lv_obj_create(screen);
    lv_obj_remove_style_all(wxFlash);
    lv_obj_set_size(wxFlash, 410, 502);
    lv_obj_set_pos(wxFlash, 0, 0);
    lv_obj_set_style_bg_color(wxFlash, lv_color_hex(0xDDE8FF), 0);
    lv_obj_set_style_bg_opa(wxFlash, 0, 0);
    lv_obj_clear_flag(wxFlash, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));

    wxNextFlashMs = millis() + 600;
    wxFlashUntil = 0;

    // Mountain silhouette (sunny phase) along the bottom, above the dock.
    wxMtn = lv_image_create(screen);
    lv_image_set_src(wxMtn, &wx_mountains);
    lv_obj_align(wxMtn, LV_ALIGN_BOTTOM_MID, 0, -88);  // above the dock
    lv_obj_set_style_opa(wxMtn, 0, 0);
    lv_obj_clear_flag(wxMtn, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));

    // City skyline (storm phase): dark buildings + a few lit windows.
    wxCity = lv_obj_create(screen);
    lv_obj_remove_style_all(wxCity);
    lv_obj_set_size(wxCity, 410, 80);
    lv_obj_align(wxCity, LV_ALIGN_BOTTOM_MID, 0, -74);  // above the dock
    lv_obj_clear_flag(wxCity, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
    static const uint8_t bH[12] = { 30, 54, 40, 68, 36, 60, 46, 72, 34, 56, 44, 64 };
    for (int i = 0; i < 12; ++i) {
        lv_obj_t *b = lv_obj_create(wxCity);
        lv_obj_remove_style_all(b);
        lv_obj_set_size(b, 30, bH[i]);
        lv_obj_set_pos(b, 6 + i * 34, 80 - bH[i]);
        lv_obj_set_style_bg_color(b, lv_color_hex(0x0E1526), 0);
        lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
        lv_obj_clear_flag(b, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
        // Glowing vertical windows in a grid. Taller buildings get more rows
        // of windows; short ones get just a couple.
        if (i % 2 == 0) {
            const int colX[2] = { 7, 19 };   // two columns of windows
            const int wW = 4, wH = 7;         // vertical window
            const int topY = 8, vgap = 13;
            int rows = (bH[i] - topY - 4) / vgap;
            if (rows < 1) rows = 1;
            if (rows > 5) rows = 5;
            for (int r = 0; r < rows; ++r) {
                for (int c = 0; c < 2; ++c) {
                    if (((r + c + i) % 3) == 0) continue;   // some windows dark
                    lv_obj_t *win = lv_obj_create(b);
                    lv_obj_remove_style_all(win);
                    lv_obj_set_size(win, wW, wH);
                    lv_obj_set_pos(win, colX[c], topY + r * vgap);
                    lv_obj_set_style_radius(win, 1, 0);
                    lv_obj_set_style_bg_color(win, lv_color_hex(0xFFF3D6), 0);
                    lv_obj_set_style_bg_opa(win, LV_OPA_COVER, 0);
                    lv_obj_set_style_shadow_color(win, lv_color_hex(0xFFE9B0), 0);
                    lv_obj_set_style_shadow_width(win, 5, 0);
                    lv_obj_set_style_shadow_opa(win, 160, 0);
                }
            }
        }
    }
    lv_obj_set_style_opa(wxCity, 0, 0);

    // Rain streaks (rain/storm): thin pale-blue drops falling, tilted by wind.
    for (int i = 0; i < WX_RAIN_N; ++i) {
        lv_obj_t *d = lv_obj_create(screen);
        lv_obj_remove_style_all(d);
        lv_obj_set_size(d, 2, 12);
        lv_obj_set_style_radius(d, 1, 0);
        lv_obj_set_style_bg_color(d, lv_color_hex(0xBFE0FF), 0);
        lv_obj_set_style_bg_opa(d, LV_OPA_COVER, 0);
        lv_obj_set_style_opa(d, 0, 0);
        lv_obj_clear_flag(d, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
        wxRainX[i]   = rand() % 410;
        wxRainY[i]   = rand() % 502;
        wxRainSpd[i] = 9.0f + (rand() % 100) / 18.0f;   // ~9..14.5 px/tick
        lv_obj_set_pos(d, (int)wxRainX[i], (int)wxRainY[i]);
        wxRain[i] = d;
    }
    // Snowflakes (snow): small white discs drifting down with a sideways sway.
    for (int i = 0; i < WX_SNOW_N; ++i) {
        lv_obj_t *s = lv_obj_create(screen);
        lv_obj_remove_style_all(s);
        int sz = 4 + (rand() % 3);   // 4..6 px
        lv_obj_set_size(s, sz, sz);
        lv_obj_set_style_radius(s, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(s, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(s, LV_OPA_COVER, 0);
        lv_obj_set_style_opa(s, 0, 0);
        lv_obj_clear_flag(s, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
        wxSnowX[i]   = rand() % 410;
        wxSnowY[i]   = rand() % 502;
        wxSnowSpd[i] = 1.6f + (rand() % 100) / 55.0f;   // ~1.6..3.4 px/tick
        wxSnowPh[i]  = (rand() % 628) / 100.0f;
        lv_obj_set_pos(s, (int)wxSnowX[i], (int)wxSnowY[i]);
        wxSnow[i] = s;
    }

    createTopBar();

    // Clock across the top; the date line is replaced by the week strip below.
    buildFaceClock(36);
    if (faceClockDate) lv_obj_add_flag(faceClockDate, LV_OBJ_FLAG_HIDDEN);

    // Day-of-week strip: all seven days, the current one lit with its date.
    RTC_DateTime wdt = instance.rtc.getDateTime();
    struct tm wtm = wdt.toUnixTime();
    int wd = wtm.tm_wday; if (wd < 0 || wd > 6) wd = 0;
    static const char *DOW[7] = { "S", "M", "T", "W", "T", "F", "S" };
    for (int i = 0; i < 7; ++i) {
        lv_obj_t *c = lv_label_create(screen);
        lv_label_set_text(c, DOW[i]);
        lv_obj_set_style_text_font(c, &lv_font_montserrat_16, 0);
        const bool cur = (i == wd);
        lv_obj_set_style_text_color(c, lv_color_hex(cur ? 0x0A1024 : 0x5E7290), 0);
        if (cur) {
            lv_obj_set_style_bg_color(c, lv_color_hex(0x3AEFFF), 0);
            lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);
            lv_obj_set_style_radius(c, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_pad_all(c, 5, 0);
        }
        lv_obj_align(c, LV_ALIGN_TOP_MID, (i - 3) * 44, cur ? 94 : 98);
        weekCells[i] = c;
    }
    weekDateLabel = lv_label_create(screen);
    lv_label_set_text_fmt(weekDateLabel, "%u", wdt.getDay());
    lv_obj_set_style_text_font(weekDateLabel, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(weekDateLabel, lv_color_hex(0x3AEFFF), 0);
    lv_obj_align(weekDateLabel, LV_ALIGN_TOP_MID, (wd - 3) * 44, 124);

    weatherTempLabel = lv_label_create(screen);
    lv_label_set_text(weatherTempLabel, "--\xC2\xB0");
    lv_obj_set_style_text_font(weatherTempLabel, &lv_font_montserrat_44, 0);
    lv_obj_set_style_text_color(weatherTempLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(weatherTempLabel, LV_ALIGN_TOP_MID, 0, 152);

    weatherCondLabel = lv_label_create(screen);
    lv_label_set_text(weatherCondLabel, "");
    lv_obj_set_style_text_font(weatherCondLabel, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(weatherCondLabel, lv_color_hex(0xCFE8FF), 0);
    lv_obj_align(weatherCondLabel, LV_ALIGN_TOP_MID, 0, 210);

    weatherHiLoLabel = lv_label_create(screen);
    lv_label_set_text(weatherHiLoLabel, "");
    lv_obj_set_style_text_font(weatherHiLoLabel, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(weatherHiLoLabel, lv_color_hex(0x9FD0FF), 0);
    lv_obj_align(weatherHiLoLabel, LV_ALIGN_TOP_MID, 0, 250);

    weatherPlaceLabel = lv_label_create(screen);
    lv_label_set_text(weatherPlaceLabel, "");
    lv_obj_set_style_text_font(weatherPlaceLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(weatherPlaceLabel, lv_color_hex(0xCFE8FF), 0);
    lv_obj_align(weatherPlaceLabel, LV_ALIGN_TOP_MID, 0, 286);

    weatherStatusLabel = lv_label_create(screen);
    lv_label_set_text(weatherStatusLabel, "Updating weather...");
    lv_obj_set_style_text_font(weatherStatusLabel, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(weatherStatusLabel, lv_color_hex(0x7FA8D8), 0);
    lv_obj_align(weatherStatusLabel, LV_ALIGN_TOP_MID, 0, 312);

    // Load the saved reading on a cold boot so the face isn't blank offline.
    if (!weatherHaveData) loadWeatherCache();

    // Show whatever we already have (fresh or cached), then refresh if stale.
    applyWeatherToUi();
    if (weatherState == WeatherState::Ready) {
        if (millis() - weatherLastFetchMs > WEATHER_REFRESH_MS) weatherFetchPending = true;
    } else {
        weatherFetchPending = true;   // try to update (shows cache meanwhile)
    }
}

// Home face swipe, serviced from the main loop. Reads the touch input device
// state directly (via LVGL's cached point), so it does not depend on LVGL
// event routing / object z-order, which was swallowing the swipe before.
// Reads the touch controller directly (same source and coordinate space as
// the LVGL display driver's touchpad_read). We deliberately do NOT go through
// lv_indev_get_next(): this board registers touch, encoder and keypad input
// devices, and LVGL inserts them head-first, so the first indev is not the
// touchpad. Polling the hardware removes that ambiguity entirely.
void serviceFaceSwipe()
{
    if (currentPage != Page::Home || screenSleeping) {
        swipeTracking = false;
        swipeLostMs = 0;
        return;
    }

    int16_t tx = 0, ty = 0;
    bool touched = instance.getPoint(&tx, &ty, 1) > 0;

    if (touched) {
        if (!swipeTracking) {
            swipeTracking = true;
            swipeStartX = tx;
            swipeStartY = ty;
            swipeStartMs = millis();
        }
        // Remember the most recent in-contact point; the release poll reports
        // no coordinates, so we measure the swipe from the last point down.
        swipeLastX = tx;
        swipeLastY = ty;
        swipeLostMs = 0;   // touch present again: cancel any pending release
        return;
    }

    // No touch this poll.
    if (!swipeTracking) return;

    // Debounce brief gaps in the touch stream. The controller routinely drops
    // a frame or two mid-drag, which would otherwise look like a finger lift
    // and chop one slow swipe into many tiny fragments. Only finalize once the
    // touch has been continuously absent for the grace period.
    if (swipeLostMs == 0) { swipeLostMs = millis(); return; }
    if (millis() - swipeLostMs < 90) return;

    swipeTracking = false;
    swipeLostMs = 0;

    const int dx = swipeLastX - swipeStartX;
    const int dy = swipeLastY - swipeStartY;
    const uint32_t dt = millis() - swipeStartMs;
    const bool ok = (dt < 2000 && abs(dx) > 40 && abs(dx) > abs(dy) &&
                     millis() - lastFaceSwitchMs > 450);
    if (ok) {
        homeFaceIndex = (dx < 0)
            ? (homeFaceIndex + 1) % HOME_FACE_COUNT
            : (homeFaceIndex + HOME_FACE_COUNT - 1) % HOME_FACE_COUNT;
        lastFaceSwitchMs = millis();
        saveHomeFace();
        hapticTap();
        showHome();
    }
}

void showHome()
{
    clearScreen();
    currentPage = Page::Home;

    // Alternate-face objects are rebuilt each time; clear stale pointers.
    faceClockHour = faceClockColon = faceClockMin = faceClockDate = nullptr;
    termTime = termDate = termBatt = termCursor = nullptr;
    crewMain = crewCatBody = crewCatTail = crewMonBody = crewMonTail = nullptr;
    videoImg = nullptr;
    radarFaceImg = radarEyeL = radarEyeR = nullptr;
    radarEyeLGlow = radarEyeRGlow = nullptr;
    radarVTime = radarVMeta = radarCenterTxt = nullptr;
    radarMetaDay = radarMetaDate = nullptr;
    for (int i = 0; i < 4; ++i) radarRing[i] = radarRingGlow[i] = nullptr;
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 7; ++j) radarSeg[i][j] = nullptr;
    for (int i = 0; i < BUBBLE_COUNT; ++i) {
        gBubbles[i].obj = gBubbles[i].glowL = gBubbles[i].glowR = nullptr;
    }
    for (int i = 0; i < BG_BUBBLE_COUNT; ++i) gBgBubbles[i].obj = nullptr;
    weatherTempLabel = weatherCondLabel = weatherHiLoLabel = nullptr;
    weatherPlaceLabel = weatherStatusLabel = weatherIcon = nullptr;
    wxBg = wxFlash = wxSun = wxBolt = nullptr;
    wxMtn = wxCity = weekDateLabel = nullptr;
    wxCloud1 = wxCloud2 = wxBird = nullptr;
    wxMoon = wxFog1 = wxFog2 = nullptr;
    for (int i = 0; i < WX_STAR_N; ++i) wxStars[i] = nullptr;
    for (int i = 0; i < WX_RAIN_N; ++i) wxRain[i] = nullptr;
    for (int i = 0; i < WX_SNOW_N; ++i) wxSnow[i] = nullptr;
    for (int i = 0; i < 7; ++i) weekCells[i] = nullptr;
    weatherFetchPending = false;

    // Reset matrix-rain tuning to the default (faces override as needed).
    matrixColA = 0x3AEFFF;
    matrixColB = 0xFF4AD5;
    matrixDriftX = 0;
    matrixSkullRecolor = 0;
    dockIconColorOverride = 0;

    // (Swipe-to-switch is serviced from the main loop; see serviceFaceSwipe.)

    if (homeFaceIndex == 1) {
        buildTerminalFace();
    } else if (homeFaceIndex == 2) {
        buildCrewFace();
    } else if (homeFaceIndex == 3) {
        buildVideoFace();
    } else if (homeFaceIndex == 4) {
        buildBubblesFace();
    } else if (homeFaceIndex == 5) {
        buildWeatherFace();
    } else if (homeFaceIndex == 6) {
        buildRadarFace();
    } else {
        // Original face, unchanged.
        createMatrixBackground();
        createArt();
        createTopBar();
        createTimeDate();
    }

    createDock();

    updateHomeLiveData();
    updateFaceClock();
    homeLiveTimer = lv_timer_create(homeLiveTimerCb, 1000, nullptr);

    faceAnimStartMs = millis();
    if (homeFaceIndex != 0) {
        // 33 ms (~30 fps) so motion steps are small and smooth, not choppy.
        faceAnimTimer = lv_timer_create(faceAnimTick, 33, nullptr);
    }

    noteActivity();
    Serial.printf("[M33K] Home (face %u)\n", homeFaceIndex);
}

// -----------------------------------------------------------------------------
// Wi-Fi live scanner
// -----------------------------------------------------------------------------

const char *authLabel(wifi_auth_mode_t auth)
{
    switch (auth) {
        case WIFI_AUTH_OPEN: return "OPEN";
        case WIFI_AUTH_WEP: return "WEP";
        case WIFI_AUTH_WPA_PSK: return "WPA-PSK";
        case WIFI_AUTH_WPA2_PSK: return "WPA2-PSK";
        case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2-PSK";
#ifdef WIFI_AUTH_WPA2_ENTERPRISE
        case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-ENT";
#endif
#ifdef WIFI_AUTH_WPA3_PSK
        case WIFI_AUTH_WPA3_PSK: return "WPA3-PSK";
#endif
#ifdef WIFI_AUTH_WPA2_WPA3_PSK
        case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
#endif
#ifdef WIFI_AUTH_WAPI_PSK
        case WIFI_AUTH_WAPI_PSK: return "WAPI-PSK";
#endif
#ifdef WIFI_AUTH_OWE
        case WIFI_AUTH_OWE: return "OWE";
#endif
        default: return "SECURED";
    }
}


const char *signalLabel(int32_t rssi)
{
    if (rssi >= -50) return "EXCELLENT";
    if (rssi >= -60) return "GOOD";
    if (rssi >= -70) return "FAIR";
    return "WEAK";
}

void wifiDetailCloseEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (wifiDetailOverlay) {
        lv_obj_add_flag(wifiDetailOverlay, LV_OBJ_FLAG_HIDDEN);
    }
    noteActivity();
}

void wifiDetailEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    auto *detail = static_cast<WifiResultDetail *>(lv_event_get_user_data(e));
    if (!detail || !wifiDetailOverlay || !wifiDetailTitle || !wifiDetailBody) return;

    noteActivity();
    hapticTap();

    lv_label_set_text(wifiDetailTitle, detail->ssid.c_str());

    const bool band5 = detail->fromC5 || detail->channel > 14;
    const bool connected =
        !band5 &&
        WiFi.status() == WL_CONNECTED &&
        WiFi.SSID() == detail->ssid;

    char body[320];
    snprintf(
        body, sizeof(body),
        "BSSID\n%s\n\nBAND   %s\nSOURCE   %s\nRSSI   %ld dBm   %s\nCHANNEL   %ld\nSECURITY   %s\nWATCH CONNECTED   %s",
        detail->bssid.c_str(),
        band5 ? "5 GHz" : "2.4 GHz",
        band5 ? "M33K X C5" : "T-WATCH",
        (long)detail->rssi,
        signalLabel(detail->rssi),
        (long)detail->channel,
        authLabel(detail->auth),
        connected ? "YES" : "NO"
    );

    lv_label_set_text(wifiDetailBody, body);
    lv_obj_remove_flag(wifiDetailOverlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(wifiDetailOverlay);
}



void captureLocalWifiResults(int16_t count)
{
    wifiLocalDetailCount = 0;
    wifiDetailCount = 0;

    if (count <= 0) return;

    // Reserve half of the shared cache for the C5 so dense 2.4 GHz areas do
    // not crowd every 5 GHz result out of the merged tool feed.
    const int16_t localCapacity =
        static_cast<int16_t>(MAX_WIFI_DETAILS - MAX_C5_WIFI_DETAILS);
    const int16_t take = min(localCapacity, count);

    for (int16_t i = 0; i < take; ++i) {
        WifiResultDetail &detail = wifiDetails[wifiLocalDetailCount];
        detail.ssid = WiFi.SSID(i);
        if (detail.ssid.length() == 0) detail.ssid = "<hidden>";
        detail.bssid = WiFi.BSSIDstr(i);
        detail.rssi = WiFi.RSSI(i);
        detail.channel = WiFi.channel(i);
        detail.auth = WiFi.encryptionType(i);
        detail.fromC5 = false;
        ++wifiLocalDetailCount;
    }

    wifiDetailCount = wifiLocalDetailCount;
}

void mergeC5WifiResults()
{
    wifiDetailCount = wifiLocalDetailCount;

    // Remote rows are valid only while the C5 link is actually alive.
    // This prevents old 5 GHz graph points from surviving a disconnect.
    if (!c5IsLinked()) return;

    const uint8_t remoteCount = c5WifiDetailCount;
    for (uint8_t i = 0;
         i < remoteCount && wifiDetailCount < MAX_WIFI_DETAILS;
         ++i) {

        const WifiResultDetail &remote = c5WifiDetails[i];

        bool duplicate = false;
        for (uint8_t j = 0; j < wifiDetailCount; ++j) {
            if (wifiDetails[j].bssid.equalsIgnoreCase(remote.bssid)) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) continue;

        wifiDetails[wifiDetailCount] = remote;
        wifiDetails[wifiDetailCount].fromC5 = true;
        ++wifiDetailCount;
    }

}

uint8_t wifiMerged24Count()
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < wifiDetailCount; ++i) {
        if (!wifiDetails[i].fromC5 && wifiDetails[i].channel <= 14) ++count;
    }
    return count;
}

uint8_t wifiMerged5Count()
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < wifiDetailCount; ++i) {
        if (wifiDetails[i].fromC5 || wifiDetails[i].channel > 14) ++count;
    }
    return count;
}

void renderWifiCachedList()
{
    if (!wifiList || !wifiStatusLabel) return;

    lv_obj_clean(wifiList);

    const uint8_t count24 = wifiMerged24Count();
    const uint8_t count5 = wifiMerged5Count();

    if (wifiDetailCount == 0) {
        lv_label_set_text_fmt(
            wifiStatusLabel,
            "0 APs  |  2.4 + 5 GHz  |  %s",
            c5IsLinked() ? "C5 LINKED" : "C5 OFFLINE"
        );

        lv_obj_t *none = lv_label_create(wifiList);
        lv_label_set_text(none, "No Wi-Fi networks detected.");
        lv_obj_set_style_text_color(none, lv_color_hex(0xC7D3E4), 0);
        lv_obj_set_style_text_font(none, &lv_font_montserrat_16, 0);
        return;
    }

    lv_label_set_text_fmt(
        wifiStatusLabel,
        "%u APs  |  2.4:%u  5G:%u  |  %s",
        (unsigned)wifiDetailCount,
        (unsigned)count24,
        (unsigned)count5,
        c5IsLinked() ? "C5" : "NO C5"
    );

    for (uint8_t i = 0; i < wifiDetailCount; ++i) {
        WifiResultDetail &detail = wifiDetails[i];

        String displaySsid = detail.ssid;
        if (displaySsid.length() > 22) {
            displaySsid = displaySsid.substring(0, 19) + "...";
        }

        const bool band5 = detail.fromC5 || detail.channel > 14;

        lv_obj_t *row = lv_button_create(wifiList);
        lv_obj_set_width(row, 322);
        lv_obj_set_height(row, 64);
        lv_obj_set_style_radius(row, 12, 0);
        lv_obj_set_style_bg_color(row, lv_color_hex(0x081321), 0);
        lv_obj_set_style_bg_opa(row, 170, 0);
        lv_obj_set_style_border_width(row, 1, 0);
        lv_obj_set_style_border_color(
            row,
            lv_color_hex(band5 ? 0xFF52C8 : 0x183554),
            0
        );
        lv_obj_set_style_pad_all(row, 7, 0);
        lv_obj_set_ext_click_area(row, 3);
        lv_obj_add_event_cb(
            row,
            wifiDetailEvent,
            LV_EVENT_CLICKED,
            &detail
        );

        lv_obj_t *ssidLabel = lv_label_create(row);
        lv_label_set_text(ssidLabel, displaySsid.c_str());
        lv_obj_set_style_text_color(
            ssidLabel,
            lv_color_hex(band5 ? 0xFF8AD9 : 0x55EEFF),
            0
        );
        lv_obj_set_style_text_font(ssidLabel, &lv_font_montserrat_16, 0);
        lv_obj_set_pos(ssidLabel, 6, 2);

        char info[110];
        snprintf(
            info,
            sizeof(info),
            "%s  %ld dBm   CH %ld   %s   >",
            band5 ? "5G" : "2.4",
            (long)detail.rssi,
            (long)detail.channel,
            authLabel(detail.auth)
        );

        lv_obj_t *infoLabel = lv_label_create(row);
        lv_label_set_text(infoLabel, info);
        lv_obj_set_style_text_color(infoLabel, lv_color_hex(0xD6E0EE), 0);
        lv_obj_set_style_text_font(infoLabel, &lv_font_montserrat_12, 0);
        lv_obj_set_pos(infoLabel, 6, 33);
    }
}

void wifiGraphBackEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();
    wifiSubpageTransition = true;
    wifiGraphLive = false;
    wifiPendingNav = WifiPendingNav::BackToScanner;
}

void wifiGraphOpenEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();
    wifiSubpageTransition = true;
    wifiPendingNav = WifiPendingNav::OpenGraph;
}

int32_t wifiPeakRssiForChannel(uint8_t channel)
{
    int32_t peak = -127;
    for (uint8_t i = 0; i < wifiDetailCount; ++i) {
        if (wifiDetails[i].channel == channel && wifiDetails[i].rssi > peak) {
            peak = wifiDetails[i].rssi;
        }
    }
    return peak;
}

uint8_t wifiCountForChannel(uint8_t channel)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < wifiDetailCount; ++i) {
        if (wifiDetails[i].channel == channel) {
            ++count;
        }
    }
    return count;
}

void resetWifiGraphDrops()
{
    for (auto &drop : wifiGraphDrops) {
        drop.active = false;
        drop.obj = nullptr;
        drop.silhouette = nullptr;
        drop.startY = 22;
        drop.targetY = 162;
        drop.bornMs = 0;
        drop.delayMs = 0;
    }
}

void updateWifiGraphDrops()
{
    if (!wifiGraphView) return;

    const uint32_t now = millis();

    for (auto &drop : wifiGraphDrops) {
        if (!drop.active || !drop.obj) continue;

        const uint32_t age = now - drop.bornMs;

        if (age < drop.delayMs) {
            lv_obj_set_style_opa(drop.obj, 0, 0);
            continue;
        }

        const uint32_t t = age - drop.delayMs;

        if (t <= WIFI_DROP_FALL_MS) {
            const int32_t y =
                drop.startY +
                (
                    static_cast<int32_t>(
                        drop.targetY - drop.startY
                    ) *
                    static_cast<int32_t>(t)
                ) /
                static_cast<int32_t>(WIFI_DROP_FALL_MS);

            lv_obj_set_y(drop.obj, static_cast<int16_t>(y));
            lv_obj_set_style_opa(drop.obj, 255, 0);
            continue;
        }

        if (t <= WIFI_DROP_FALL_MS + WIFI_DROP_HOLD_MS) {
            lv_obj_set_y(drop.obj, drop.targetY);
            lv_obj_set_style_opa(drop.obj, 255, 0);
            continue;
        }

        const uint32_t fadeAge =
            t - WIFI_DROP_FALL_MS - WIFI_DROP_HOLD_MS;

        if (fadeAge <= WIFI_DROP_FADE_MS) {
            const int32_t opa =
                255 -
                (
                    255 *
                    static_cast<int32_t>(fadeAge)
                ) /
                static_cast<int32_t>(WIFI_DROP_FADE_MS);

            lv_obj_set_style_opa(
                drop.obj,
                static_cast<lv_opa_t>(
                    constrain(opa, 0L, 255L)
                ),
                0
            );
            continue;
        }

        // Keep the graph visually continuous while the radios perform their
        // next discrete scan sweeps.  The faint occupancy skull never leaves;
        // recycle the bright skull into another short fall instead of deleting
        // it and waiting several seconds for the next completed scan.
        lv_obj_set_y(drop.obj, drop.startY);
        lv_obj_set_style_opa(drop.obj, 0, 0);
        drop.bornMs = now;
        drop.delayMs =
            60u +
            static_cast<uint32_t>(
                (drop.startY + drop.targetY) % 6
            ) * 28u;
    }
}

uint8_t spawnWifiGraphDrops(lv_obj_t *panel)
{
    if (!panel) return 0;

    uint8_t dropIndex = 0;
    uint8_t channelSlots[15] = {};

    for (uint8_t i = 0;
         i < wifiDetailCount &&
         dropIndex < WIFI_GRAPH_MAX_DROPS;
         ++i) {

        const uint8_t ch = wifiDetails[i].channel;

        if (ch < 1 || ch > 14) {
            continue;
        }

        WifiGraphDrop &drop =
            wifiGraphDrops[dropIndex];

        const int16_t x =
            14 + (static_cast<int16_t>(ch) - 1) * 21;

        // Slot within this channel gives each current AP its own faint
        // landing silhouette. Cap the visual stack so dense channels do not
        // grow outside the graph.
        const uint8_t slot =
            channelSlots[ch] < 7
                ? channelSlots[ch]++
                : 6;

        const uint32_t hash =
            m33kHashString(wifiDetails[i].bssid);

        drop.active = true;
        drop.startY =
            18 + static_cast<int16_t>(hash % 12u);

        const int16_t stackedTarget =
            77 - static_cast<int16_t>(slot) * 8;
        drop.targetY =
            stackedTarget < 34 ? 34 : stackedTarget;

        drop.bornMs = millis();
        drop.delayMs =
            static_cast<uint32_t>(dropIndex % 10u) * 48u;

        // Faint silhouette = latest-sweep occupancy. It stays visible until
        // fresh scan results rebuild the graph.
        lv_obj_t *shadow = lv_image_create(panel);
        drop.silhouette = shadow;

        lv_image_set_src(
            shadow,
            &bunny_skull_graph_img
        );
        lv_obj_set_pos(
            shadow,
            x,
            drop.targetY
        );
        lv_obj_set_style_img_recolor(
            shadow,
            lv_color_hex(0xFF52C8),
            0
        );
        lv_obj_set_style_img_recolor_opa(
            shadow,
            150,
            0
        );
        lv_obj_set_style_opa(
            shadow,
            58,
            0
        );

        // Bright current detection falls onto its silhouette, then fades.
        lv_obj_t *skull = lv_image_create(panel);
        drop.obj = skull;

        lv_image_set_src(
            skull,
            &bunny_skull_graph_img
        );
        lv_obj_set_pos(
            skull,
            x,
            drop.startY
        );
        lv_obj_set_style_img_recolor(
            skull,
            lv_color_hex(0xFF52C8),
            0
        );
        lv_obj_set_style_img_recolor_opa(
            skull,
            80,
            0
        );
        lv_obj_set_style_opa(skull, 0, 0);

        ++dropIndex;
    }

    return dropIndex;
}

void spawnWifiGraphDrops5(lv_obj_t *panel, uint8_t dropIndex)
{
    if (!panel || dropIndex >= WIFI_GRAPH_MAX_DROPS) return;

    uint8_t channelStacks[32] = {};

    for (uint8_t i = 0;
         i < wifiDetailCount && dropIndex < WIFI_GRAPH_MAX_DROPS;
         ++i) {

        const WifiResultDetail &detail = wifiDetails[i];
        if (!(detail.fromC5 || detail.channel > 14)) continue;
        if (detail.channel < 36 || detail.channel > 177) continue;

        const int slotIndex =
            constrain((detail.channel - 36) / 5, 0L, 31L);

        const uint8_t stack = channelStacks[slotIndex] < 4
            ? channelStacks[slotIndex]++
            : 3;

        const int16_t x = map(detail.channel, 36, 177, 18, 296);
        // Match the 2.4 GHz graph: skulls fall all the way to the baseline,
        // then additional APs in the same channel slot stack upward.
        const int16_t targetY =
            187 - static_cast<int16_t>(stack) * 8;

        WifiGraphDrop &drop = wifiGraphDrops[dropIndex];
        const uint32_t hash = m33kHashString(detail.bssid);

        drop.active = true;
        drop.startY = 151 + static_cast<int16_t>(hash % 7u);
        drop.targetY = targetY;
        drop.bornMs = millis();
        drop.delayMs = static_cast<uint32_t>(dropIndex % 10u) * 48u;

        // Faint blue occupancy skull remains until a complete replacement
        // sweep is published and the graph is rebuilt.
        lv_obj_t *shadow = lv_image_create(panel);
        drop.silhouette = shadow;
        lv_image_set_src(shadow, &bunny_skull_graph_img);
        lv_obj_set_pos(shadow, x - 5, drop.targetY);
        lv_obj_set_style_img_recolor(
            shadow,
            lv_color_hex(0x2F6BFF),
            0
        );
        lv_obj_set_style_img_recolor_opa(shadow, 190, 0);
        lv_obj_set_style_opa(shadow, 62, 0);

        // A brighter blue detection lands on the silhouette, then fades in
        // the same way as the pink 2.4 GHz skulls.
        lv_obj_t *skull = lv_image_create(panel);
        drop.obj = skull;
        lv_image_set_src(skull, &bunny_skull_graph_img);
        lv_obj_set_pos(skull, x - 5, drop.startY);
        lv_obj_set_style_img_recolor(
            skull,
            lv_color_hex(0x2F6BFF),
            0
        );
        lv_obj_set_style_img_recolor_opa(skull, 150, 0);
        lv_obj_set_style_opa(skull, 0, 0);

        ++dropIndex;
    }
}

uint32_t wifiGraphTopologySignature()
{
    // Order-independent signature: keep the existing LVGL graph objects alive
    // when the same APs/channels are still present. This preserves the faint
    // landing shadows instead of destroying/recreating them every sweep.
    uint32_t sig = 0x4D33334Bu;

    for (uint8_t i = 0; i < wifiDetailCount; ++i) {
        const WifiResultDetail &detail = wifiDetails[i];
        uint32_t h = m33kHashString(detail.bssid);
        h ^= static_cast<uint32_t>(detail.channel) * 2654435761u;
        h ^= detail.fromC5 ? 0x5A5A5A5Au : 0x24242424u;

        // Only treat a meaningful signal-strength change as a visual topology
        // change. Tiny RSSI movement should not blink/rebuild the graph.
        const int32_t bucket = constrain((detail.rssi + 120) / 8, 0L, 15L);
        h ^= static_cast<uint32_t>(bucket) * 2246822519u;

        // XOR makes the result independent of scan-result ordering.
        sig ^= h;
    }

    sig ^= static_cast<uint32_t>(wifiDetailCount) * 3266489917u;
    return sig;
}

void renderWifiGraph();

void renderWifiGraphIfNeeded(bool force = false)
{
    if (!wifiGraphView || !wifiGraphContainer) return;

    const uint32_t sig = wifiGraphTopologySignature();
    if (!force && wifiGraphHasRender && sig == wifiGraphRenderedSignature) {
        return;
    }

    renderWifiGraph();
}

void renderWifiGraph()
{
    if (!wifiGraphContainer || !wifiStatusLabel) return;

    resetWifiGraphDrops();
    lv_obj_clean(wifiGraphContainer);

    const uint8_t count24 = wifiMerged24Count();
    const uint8_t count5 = wifiMerged5Count();

    lv_label_set_text_fmt(
        wifiStatusLabel,
        "2.4:%u  5G:%u  |  %s",
        (unsigned)count24,
        (unsigned)count5,
        c5IsLinked()
            ? (c5ScanInFlight ? "C5 SCAN" : "C5 LINKED")
            : "C5 OFFLINE"
    );

    lv_obj_t *panel = lv_obj_create(wifiGraphContainer);
    lv_obj_set_size(panel, 320, 248);
    lv_obj_center(panel);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x050B14), 0);
    lv_obj_set_style_bg_opa(panel, 170, 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_border_color(panel, lv_color_hex(0x1C3556), 0);
    lv_obj_set_style_radius(panel, 18, 0);
    lv_obj_set_style_pad_all(panel, 0, 0);

    lv_obj_t *title24 = lv_label_create(panel);
    lv_label_set_text(title24, "2.4 GHz - T-WATCH");
    lv_obj_set_style_text_color(title24, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_text_font(title24, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(title24, 12, 3);

    lv_obj_t *base24 = lv_obj_create(panel);
    lv_obj_remove_style_all(base24);
    lv_obj_set_size(base24, 292, 2);
    lv_obj_set_style_bg_color(base24, lv_color_hex(0x36516D), 0);
    lv_obj_set_pos(base24, 14, 91);

    int32_t best24Rssi = -127;
    uint8_t best24Channel = 0;

    for (uint8_t ch = 1; ch <= 14; ++ch) {
        const int32_t peak = wifiPeakRssiForChannel(ch);
        const uint8_t count = wifiCountForChannel(ch);

        if (count > 0 && peak > best24Rssi) {
            best24Rssi = peak;
            best24Channel = ch;
        }

        const int16_t x = 14 + (ch - 1) * 21;

        if (count > 0) {
            lv_obj_t *countLabel = lv_label_create(panel);
            lv_label_set_text_fmt(countLabel, "%u", count);
            lv_obj_set_width(countLabel, 18);
            lv_obj_set_style_text_align(countLabel, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_text_color(countLabel, lv_color_hex(0x7CFF45), 0);
            lv_obj_set_style_text_font(countLabel, &lv_font_montserrat_12, 0);
            lv_obj_set_pos(countLabel, x - 1, 21);
        }

        lv_obj_t *chLabel = lv_label_create(panel);
        lv_label_set_text_fmt(chLabel, "%u", ch);
        lv_obj_set_style_text_color(chLabel, lv_color_hex(0xC9D6E6), 0);
        lv_obj_set_style_text_font(chLabel, &lv_font_montserrat_12, 0);
        lv_obj_set_pos(chLabel, x + (ch >= 10 ? -1 : 2), 95);
    }

    const uint8_t nextDropIndex = spawnWifiGraphDrops(panel);

    lv_obj_t *divider = lv_obj_create(panel);
    lv_obj_remove_style_all(divider);
    lv_obj_set_size(divider, 296, 1);
    lv_obj_set_style_bg_color(divider, lv_color_hex(0x25364C), 0);
    lv_obj_set_pos(divider, 12, 119);

    lv_obj_t *title5 = lv_label_create(panel);
    lv_label_set_text(title5, "5 GHz - M33K X C5");
    lv_obj_set_style_text_color(title5, lv_color_hex(0xFF8AD9), 0);
    lv_obj_set_style_text_font(title5, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(title5, 12, 123);

    lv_obj_t *base5 = lv_obj_create(panel);
    lv_obj_remove_style_all(base5);
    lv_obj_set_size(base5, 282, 2);
    lv_obj_set_style_bg_color(base5, lv_color_hex(0x5A3557), 0);
    lv_obj_set_pos(base5, 18, 201);

    int32_t best5Rssi = -127;
    int best5Channel = 0;
    uint8_t channelCounts5[32] = {};

    for (uint8_t i = 0; i < wifiDetailCount; ++i) {
        const WifiResultDetail &detail = wifiDetails[i];
        if (!(detail.fromC5 || detail.channel > 14)) continue;
        if (detail.channel < 36 || detail.channel > 177) continue;

        if (detail.rssi > best5Rssi) {
            best5Rssi = detail.rssi;
            best5Channel = detail.channel;
        }

        const int slotIndex = constrain((detail.channel - 36) / 5, 0L, 31L);
        if (channelCounts5[slotIndex] < 99) {
            ++channelCounts5[slotIndex];
        }
    }

    // Green counts mirror the 2.4 GHz count row.  Each value represents the
    // nearby 5 GHz channel slot directly below it.
    for (uint8_t slot = 0; slot < 32; ++slot) {
        if (channelCounts5[slot] == 0) continue;

        const int channel = 36 + static_cast<int>(slot) * 5;
        const int16_t x = map(channel, 36, 177, 18, 296);

        lv_obj_t *countLabel = lv_label_create(panel);
        lv_label_set_text_fmt(countLabel, "%u", channelCounts5[slot]);
        lv_obj_set_width(countLabel, 16);
        lv_obj_set_style_text_align(countLabel, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(
            countLabel,
            lv_color_hex(0x7CFF45),
            0
        );
        lv_obj_set_style_text_font(
            countLabel,
            &lv_font_montserrat_10,
            0
        );
        lv_obj_set_pos(countLabel, x - 7, 140);
    }

    spawnWifiGraphDrops5(panel, nextDropIndex);

    const int labels5[] = {36, 64, 100, 140, 177};
    for (int ch : labels5) {
        const int16_t x = map(ch, 36, 177, 18, 296);
        lv_obj_t *label = lv_label_create(panel);
        lv_label_set_text_fmt(label, "%d", ch);
        lv_obj_set_style_text_color(label, lv_color_hex(0xDCC9DA), 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_10, 0);
        lv_obj_set_pos(label, x - 8, 205);
    }

    lv_obj_t *legend = lv_label_create(wifiGraphContainer);
    if (best24Channel > 0 || best5Channel > 0) {
        lv_label_set_text_fmt(
            legend,
            "BEST  2.4 CH %u %ld dBm   •   5G CH %d %ld dBm",
            best24Channel,
            (long)best24Rssi,
            best5Channel,
            (long)best5Rssi
        );
    } else {
        lv_label_set_text(legend, "Waiting for dual-band scan data...");
    }

    lv_obj_set_width(legend, 326);
    lv_obj_set_style_text_color(legend, lv_color_hex(0xDDE7F2), 0);
    lv_obj_set_style_text_font(legend, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_align(legend, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(legend, LV_ALIGN_BOTTOM_MID, 0, -5);
    wifiGraphLegend = legend;
    wifiGraphRenderedSignature = wifiGraphTopologySignature();
    wifiGraphHasRender = true;
}

void renderWifiResults(int16_t count)
{
    captureLocalWifiResults(count);
    mergeC5WifiResults();
    renderWifiCachedList();
}



void beginWifiScan()
{
    if (!wifiStatusLabel) return;

    noteActivity();

    const bool alreadyConnected =
        WiFi.status() == WL_CONNECTED;

    const bool fastGraphScan =
        wifiGraphView && wifiGraphLive;

    // Keep the watch's 2.4 GHz scanner independent.  A missing C5 must not
    // delay/restart the local scan.  Only ask for 5 GHz when already linked.
    if (c5IsLinked() && !c5ScanInFlight) {
        requestC5Scan();
    }

    lv_label_set_text(
        wifiStatusLabel,
        c5IsLinked()
            ? "STARTING 2.4 + 5 GHz SCAN..."
            : "STARTING 2.4 GHz  |  FINDING C5..."
    );

    // Stop any scan that may still be alive inside ESP-IDF even if Arduino's
    // WiFi scan state flags were cleared by a previous page transition.
    esp_wifi_scan_stop();
    WiFi.scanDelete();

    // If we are not deliberately connected to an AP, cancel any stale
    // association/autoconnect attempt. esp_wifi_scan_start() can reject a
    // scan while STA is still connecting.
    if (!alreadyConnected) {
        WiFi.disconnect(false, false);
        delay(fastGraphScan ? 18 : 120);
    }

    // Do not reinitialize the Wi-Fi driver on every graph sweep. Re-entering
    // WiFi.mode() while BLE callbacks are completing a C5 disconnect was the
    // reproducible path into esp_wifi_init/NVS and a LoadStoreError. Only
    // initialize STA when the radio is actually off.
    const wifi_mode_t currentWifiMode = WiFi.getMode();
    if (currentWifiMode != WIFI_STA && currentWifiMode != WIFI_AP_STA) {
        WiFi.mode(WIFI_STA);
        delay(fastGraphScan ? 90 : 220);
    }

    // Stop/delete once more after STA startup. This also gives us a clean
    // state after WIFI_OFF -> WIFI_STA transitions.
    esp_wifi_scan_stop();
    WiFi.scanDelete();
    delay(50);

    if (wifiList) lv_obj_clean(wifiList);

    // On the live graph, leave the previous sweep visible while the next
    // scan runs. The graph is replaced only when fresh results arrive.
    if (wifiGraphContainer && !wifiGraphView) {
        lv_obj_clean(wifiGraphContainer);
    }

    lv_label_set_text(
        wifiStatusLabel,
        wifiGraphView
            ? "SCANNING..."
            : "DUAL BAND WI-FI SCAN RUNNING..."
    );

    // Use an ACTIVE asynchronous scan for recovery/reliability. The previous
    // passive scan was being rejected with WIFI_SCAN_FAILED on this watch.
    int16_t result =
        WiFi.scanNetworks(
            true,
            true,
            false,
            fastGraphScan ? WIFI_GRAPH_SCAN_DWELL_MS : 300
        );

    if (result == WIFI_SCAN_FAILED) {
        // Hard recovery only when there is no intentional live connection.
        // Power-cycle the Wi-Fi driver, then retry once.
        if (!alreadyConnected) {
            lv_label_set_text(
                wifiStatusLabel,
                "RADIO RESET... RETRYING"
            );

            esp_wifi_scan_stop();
            WiFi.scanDelete();

            WiFi.disconnect(true, false);
            delay(fastGraphScan ? 70 : 120);

            WiFi.mode(WIFI_STA);
            delay(fastGraphScan ? 120 : 350);

            WiFi.disconnect(false, false);
            delay(fastGraphScan ? 45 : 100);

            result =
                WiFi.scanNetworks(
                    true,
                    true,
                    false,
                    fastGraphScan ? 210 : 350
                );
        }
    }

    if (result == WIFI_SCAN_FAILED) {
        wifiScanning = false;
        wifiLastCompleteMs = millis();

        lv_label_set_text(
            wifiStatusLabel,
            "SCAN FAILED AFTER RADIO RESET"
        );

        Serial.printf(
            "[M33K WIFI] scan rejected after reset mode=%d status=%d\n",
            (int)WiFi.getMode(),
            (int)WiFi.status()
        );

        return;
    }

    if (result >= 0) {
        wifiScanning = false;

        Serial.printf(
            "[M33K WIFI] immediate scan result=%d\n",
            (int)result
        );

        if (wifiGraphView) {
            // A transient empty sweep should not blank a previously valid
            // graph.  Keep the last complete 2.4 GHz cache until fresh APs
            // arrive; a manual list scan still reports a real empty result.
            if (result > 0 || wifiLocalDetailCount == 0) {
                captureLocalWifiResults(result);
            }
            mergeC5WifiResults();
            renderWifiGraphIfNeeded();
            wifiLastCompleteMs = millis();
            wifiGraphNextScanMs =
                wifiLastCompleteMs +
                WIFI_GRAPH_RESCAN_MS;
        } else {
            renderWifiResults(result);
        }

        WiFi.scanDelete();
        return;
    }

    wifiScanning = true;

    Serial.printf(
        "[M33K WIFI] async active scan started mode=%d status=%d\n",
        (int)WiFi.getMode(),
        (int)WiFi.status()
    );
}





void wifiTimerCb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    if (currentPage != Page::WiFi) {
        return;
    }

    serviceC5Link();

    // On first opening the normal Wi-Fi scanner, give the C5's BLE
    // discovery/authentication a clean radio window before starting the
    // watch's own 2.4 GHz scan. Wardrive already followed this pattern
    // indirectly by repeatedly calling requestC5Scan().
    if (wifiInitialC5LinkPending && !wifiGraphView) {
        if (c5IsLinked()) {
            wifiInitialC5LinkPending = false;
            requestC5Scan();
            if (wifiStatusLabel) {
                lv_label_set_text(
                    wifiStatusLabel,
                    "C5 LINKED  |  SCANNING 2.4 + 5 GHz..."
                );
            }
            beginWifiScan();
            return;
        }

        // Discovery ended without a transport, or authentication failed and
        // disconnected. Continue as a 2.4 GHz scan instead of getting stuck.
        if (!c5DiscoveryInFlight && !c5TransportConnected()) {
            wifiInitialC5LinkPending = false;
            if (wifiStatusLabel) {
                lv_label_set_text(
                    wifiStatusLabel,
                    "C5 OFFLINE  |  SCANNING 2.4 GHz..."
                );
            }
            beginWifiScan();
            return;
        }

        // Still discovering/authenticating: do not overlap the local Wi-Fi
        // scan with the BLE setup window.
        return;
    }

    if (!wifiScanning &&
        c5ScanGeneration != wifiAppliedC5Generation) {

        mergeC5WifiResults();
        wifiAppliedC5Generation = c5ScanGeneration;

        if (wifiGraphView) renderWifiGraphIfNeeded();
        else renderWifiCachedList();
    }

    if (wifiGraphView) {
        updateWifiGraphDrops();
    }

    if (!wifiScanning) {
        if (wifiGraphView && c5DiscoveryRequestedByWifi && !c5DiscoveryInFlight) {
            // A manually requested C5 search finished/timed out. Resume the
            // local graph regardless of whether the companion was found.
            c5DiscoveryRequestedByWifi = false;
            wifiGraphLive = true;
            wifiGraphNextScanMs = millis() + 250;
        }

        if (wifiGraphView && wifiGraphLive &&
            !c5DiscoveryInFlight &&
            millis() >= wifiGraphNextScanMs) {
            // IMPORTANT: never launch BLE/C5 discovery from the continuous
            // graph loop. The graph must be stable as a 2.4 GHz-only tool.
            beginWifiScan();
        }

        return;
    }

    const int16_t result =
        WiFi.scanComplete();

    if (result == WIFI_SCAN_RUNNING) {
        return;
    }

    wifiScanning = false;

    Serial.printf(
        "[M33K WIFI] scan complete=%d mode=%d status=%d\n",
        (int)result,
        (int)WiFi.getMode(),
        (int)WiFi.status()
    );

    if (result == WIFI_SCAN_FAILED) {
        if (wifiStatusLabel) {
            lv_label_set_text(
                wifiStatusLabel,
                wifiGraphView
                    ? "LIVE SCAN FAILED  |  RETRYING..."
                    : "SCAN STARTED, THEN FAILED"
            );
        }

        WiFi.scanDelete();

        if (wifiGraphView && wifiGraphLive) {
            wifiGraphNextScanMs =
                millis() + 700;
        }

        return;
    }

    if (wifiGraphView) {
        if (result > 0 || wifiLocalDetailCount == 0) {
            captureLocalWifiResults(result);
        }
        mergeC5WifiResults();
        renderWifiGraphIfNeeded();

        wifiLastCompleteMs = millis();
        wifiGraphNextScanMs =
            wifiLastCompleteMs +
            WIFI_GRAPH_RESCAN_MS;

        WiFi.scanDelete();
        return;
    }

    if (result == 0) {
        renderWifiResults(0);
        WiFi.scanDelete();
        return;
    }

    renderWifiResults(result);
    WiFi.scanDelete();
}


void wifiRefreshEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();

    if (wifiGraphView && !c5IsLinked()) {
        // C5 discovery is manual from the graph now. Pause 2.4 GHz scanning
        // first so BLE discovery never runs directly after/inside a live scan.
        if (wifiScanning) {
            esp_wifi_scan_stop();
            WiFi.scanDelete();
            wifiScanning = false;
        }

        wifiGraphLive = false;
        c5DiscoveryRequestedByWifi = true;

        if (wifiStatusLabel) {
            lv_label_set_text(
                wifiStatusLabel,
                "2.4 PAUSED  |  LOOKING FOR C5..."
            );
        }

        if (!startC5DiscoveryAsync()) {
            c5DiscoveryRequestedByWifi = false;
            wifiGraphLive = true;
            wifiGraphNextScanMs = millis() + 300;
        }
        return;
    }

    if (wifiGraphView) {
        wifiGraphNextScanMs = millis() + WIFI_GRAPH_RESCAN_MS;
    }

    beginWifiScan();
}

// m33k t3ch ambles across the Wi-Fi page, cycling through six action poses
// (idle, dash, turn, orb, blast, leap) - a livelier, different animation from
// the two-frame run cycle used in the game.
void wifiMascotCb(lv_timer_t *timer)
{
    LV_UNUSED(timer);
    if (currentPage != Page::WiFi || !wifiMascot) return;
    const uint32_t now = millis();
    // Drift back and forth across the page.
    if (wifiMascotDir > 0) { wifiMascotX += 4; if (wifiMascotX > 300) wifiMascotDir = -1; }
    else                   { wifiMascotX -= 4; if (wifiMascotX < 10)  wifiMascotDir = 1;  }
    const int bob = (int)(sinf(now * 0.010f) * 6.0f);
    lv_obj_set_pos(wifiMascot, wifiMascotX, 286 + bob);
    // Step through all six poses at a relaxed cadence (distinct from the game).
    if (now - wifiMascotAnimMs > 190) {
        wifiMascotAnimMs = now;
        wifiMascotFrame = (wifiMascotFrame + 1) % 6;
        lv_image_set_src(wifiMascot, g_wifi[wifiMascotFrame]);
    }
}

void showWifiPage()
{
    clearScreen();
    currentPage = Page::WiFi;
    wifiPendingNav = WifiPendingNav::None;
    wifiGraphView = false;

    // Do not initialize the BLE scanner at boot: that changed the startup
    // sequence in builds where GNSS went silent. Initialize it on demand when
    // Wi-Fi/C5 is opened; initBleScanner() immediately reasserts GPS Serial1.
    initBleScanner();

    // Wi-Fi art is intentionally small and tucked into the upper-right.
    // Layered translucent blue shapes create a glow that fades into black.
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x010308), 0);

    lv_obj_t *glowOuter = lv_obj_create(screen);
    lv_obj_remove_style_all(glowOuter);
    lv_obj_set_size(glowOuter, 300, 300);
    lv_obj_set_pos(glowOuter, 110, 190);
    lv_obj_set_style_radius(glowOuter, 150, 0);
    lv_obj_set_style_bg_color(glowOuter, lv_color_hex(0x075CFF), 0);
    lv_obj_set_style_bg_opa(glowOuter, 14, 0);

    lv_obj_t *glowMid = lv_obj_create(screen);
    lv_obj_remove_style_all(glowMid);
    lv_obj_set_size(glowMid, 240, 260);
    lv_obj_set_pos(glowMid, 165, 220);
    lv_obj_set_style_radius(glowMid, 130, 0);
    lv_obj_set_style_bg_color(glowMid, lv_color_hex(0x008CFF), 0);
    lv_obj_set_style_bg_opa(glowMid, 20, 0);

    lv_obj_t *glowCore = lv_obj_create(screen);
    lv_obj_remove_style_all(glowCore);
    lv_obj_set_size(glowCore, 190, 220);
    lv_obj_set_pos(glowCore, 212, 248);
    lv_obj_set_style_radius(glowCore, 110, 0);
    lv_obj_set_style_bg_color(glowCore, lv_color_hex(0x22C8FF), 0);
    lv_obj_set_style_bg_opa(glowCore, 26, 0);

    // m33k t3ch paces across the page (bigger, 4-pose animation), in place of
    // the static bunny.
    wifiMascot = lv_image_create(screen);
    lv_image_set_src(wifiMascot, g_wifi[0]);
    wifiMascotX = 20; wifiMascotDir = 1; wifiMascotFrame = 0; wifiMascotAnimMs = millis();
    lv_obj_set_pos(wifiMascot, wifiMascotX, 286);
    lv_obj_clear_flag(wifiMascot, LV_OBJ_FLAG_CLICKABLE);
    wifiMascotTimer = lv_timer_create(wifiMascotCb, 60, nullptr);

    // A very light black veil keeps scan rows readable without hiding the art.
    lv_obj_t *shade = lv_obj_create(screen);
    lv_obj_remove_style_all(shade);
    lv_obj_set_size(shade, 410, 502);
    lv_obj_set_pos(shade, 0, 0);
    lv_obj_set_style_bg_color(shade, lv_color_hex(0x02050B), 0);
    lv_obj_set_style_bg_opa(shade, 28, 0);

    createPageBatteryIndicator();

    // Compact vertical side title.
    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "W\nI\nF\nI\n\nS\nC\nA\nN");
    lv_obj_set_style_text_color(title, lv_color_hex(0xB6FF00), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_bg_color(title, lv_color_hex(0x02060A), 0);
    lv_obj_set_style_bg_opa(title, 180, 0);
    lv_obj_set_style_radius(title, 8, 0);
    lv_obj_set_style_pad_hor(title, 3, 0);
    lv_obj_set_style_pad_ver(title, 4, 0);
    lv_obj_set_pos(title, 9, 137);

    lv_obj_t *back = lv_button_create(screen);
    lv_obj_set_size(back, 44, 40);
    lv_obj_set_pos(back, 54, 28);
    lv_obj_set_style_radius(back, 14, 0);
    lv_obj_set_style_bg_color(back, lv_color_hex(0x101A2A), 0);
    lv_obj_set_style_bg_opa(back, 220, 0);
    lv_obj_set_style_border_width(back, 1, 0);
    lv_obj_set_style_border_color(back, lv_color_hex(0xFF4AD5), 0);
    lv_obj_set_ext_click_area(back, 6);
    lv_obj_add_event_cb(back, backEvent, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *backIcon = lv_label_create(back);
    lv_label_set_text(backIcon, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(backIcon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(backIcon);

    lv_obj_t *graph = lv_button_create(screen);
    lv_obj_set_size(graph, 76, 40);
    lv_obj_set_pos(graph, 167, 28);
    lv_obj_set_style_radius(graph, 14, 0);
    lv_obj_set_style_bg_color(graph, lv_color_hex(0x101A2A), 0);
    lv_obj_set_style_bg_opa(graph, 220, 0);
    lv_obj_set_style_border_width(graph, 1, 0);
    lv_obj_set_style_border_color(graph, lv_color_hex(0x7CFF45), 0);
    lv_obj_set_ext_click_area(graph, 6);
    lv_obj_add_event_cb(graph, wifiGraphOpenEvent, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *graphText = lv_label_create(graph);
    lv_label_set_text(graphText, "GRAPH");
    lv_obj_set_style_text_color(graphText, lv_color_hex(0x7CFF45), 0);
    lv_obj_set_style_text_font(graphText, &lv_font_montserrat_12, 0);
    lv_obj_center(graphText);

    lv_obj_t *refresh = lv_button_create(screen);
    lv_obj_set_size(refresh, 46, 42);
    lv_obj_set_pos(refresh, 326, 17);
    lv_obj_set_style_radius(refresh, 14, 0);
    lv_obj_set_style_bg_color(refresh, lv_color_hex(0x101A2A), 0);
    lv_obj_set_style_bg_opa(refresh, 220, 0);
    lv_obj_set_style_border_width(refresh, 1, 0);
    lv_obj_set_style_border_color(refresh, lv_color_hex(0x2F6BFF), 0);
    lv_obj_set_ext_click_area(refresh, 6);
    lv_obj_add_event_cb(refresh, wifiRefreshEvent, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *refreshIcon = lv_label_create(refresh);
    lv_label_set_text(refreshIcon, LV_SYMBOL_REFRESH);
    lv_obj_set_style_text_color(refreshIcon, lv_color_hex(0x2F6BFF), 0);
    lv_obj_center(refreshIcon);

    wifiStatusLabel = lv_label_create(screen);
    lv_label_set_text(wifiStatusLabel, "Starting dual-band scan...");
    lv_obj_set_width(wifiStatusLabel, 274);
    lv_obj_set_style_text_color(wifiStatusLabel, lv_color_hex(0xB6FF00), 0);
    lv_obj_set_style_text_font(wifiStatusLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(wifiStatusLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_bg_color(wifiStatusLabel, lv_color_hex(0x02060A), 0);
    lv_obj_set_style_bg_opa(wifiStatusLabel, 185, 0);
    lv_obj_set_style_radius(wifiStatusLabel, 9, 0);
    lv_obj_set_style_pad_hor(wifiStatusLabel, 6, 0);
    lv_obj_set_style_pad_ver(wifiStatusLabel, 4, 0);
    lv_obj_set_pos(wifiStatusLabel, 74, 78);

    wifiList = lv_obj_create(screen);
    lv_obj_set_size(wifiList, 358, 350);
    lv_obj_set_pos(wifiList, 42, 116);
    lv_obj_set_style_bg_color(wifiList, lv_color_hex(0x040A12), 0);
    lv_obj_set_style_bg_opa(wifiList, 100, 0);
    lv_obj_set_style_border_width(wifiList, 1, 0);
    lv_obj_set_style_border_color(wifiList, lv_color_hex(0x142A43), 0);
    lv_obj_set_style_radius(wifiList, 16, 0);
    lv_obj_set_style_pad_all(wifiList, 8, 0);
    lv_obj_set_style_pad_row(wifiList, 7, 0);
    lv_obj_set_flex_flow(wifiList, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(wifiList, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(wifiList, LV_SCROLLBAR_MODE_AUTO);

    wifiDetailOverlay = lv_obj_create(screen);
    lv_obj_set_size(wifiDetailOverlay, 346, 276);
    lv_obj_set_pos(wifiDetailOverlay, 50, 126);
    lv_obj_set_style_radius(wifiDetailOverlay, 20, 0);
    lv_obj_set_style_bg_color(wifiDetailOverlay, lv_color_hex(0x050A12), 0);
    lv_obj_set_style_bg_opa(wifiDetailOverlay, 244, 0);
    lv_obj_set_style_border_width(wifiDetailOverlay, 2, 0);
    lv_obj_set_style_border_color(wifiDetailOverlay, lv_color_hex(0x55EEFF), 0);
    lv_obj_add_flag(wifiDetailOverlay, LV_OBJ_FLAG_HIDDEN);

    wifiDetailTitle = lv_label_create(wifiDetailOverlay);
    lv_label_set_text(wifiDetailTitle, "NETWORK");
    lv_obj_set_width(wifiDetailTitle, 260);
    lv_obj_set_style_text_color(wifiDetailTitle, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_text_font(wifiDetailTitle, &lv_font_montserrat_18, 0);
    lv_obj_set_pos(wifiDetailTitle, 14, 12);

    wifiDetailBody = lv_label_create(wifiDetailOverlay);
    lv_label_set_text(wifiDetailBody, "");
    lv_obj_set_width(wifiDetailBody, 310);
    lv_obj_set_style_text_color(wifiDetailBody, lv_color_hex(0xE6EDF7), 0);
    lv_obj_set_style_text_font(wifiDetailBody, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(wifiDetailBody, 14, 52);

    lv_obj_t *closeBtn = lv_button_create(wifiDetailOverlay);
    lv_obj_set_size(closeBtn, 58, 38);
    lv_obj_align(closeBtn, LV_ALIGN_BOTTOM_MID, 0, -12);
    lv_obj_set_style_radius(closeBtn, 13, 0);
    lv_obj_set_style_bg_color(closeBtn, lv_color_hex(0x24101E), 0);
    lv_obj_set_style_border_width(closeBtn, 1, 0);
    lv_obj_set_style_border_color(closeBtn, lv_color_hex(0xFF52C8), 0);
    lv_obj_add_event_cb(closeBtn, wifiDetailCloseEvent, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *closeText = lv_label_create(closeBtn);
    lv_label_set_text(closeText, "CLOSE");
    lv_obj_set_style_text_color(closeText, lv_color_hex(0xFF8AD9), 0);
    lv_obj_center(closeText);

    wifiLastCompleteMs = 0;
    wifiAppliedC5Generation = c5ScanGeneration;
    wifiGraphLive = false;
    wifiGraphNextScanMs = 0;
    wifiTimer = lv_timer_create(wifiTimerCb, 300, nullptr);

    // A normal scan page should not remain stuck in an old failed CONNECT
    // attempt. If we're not actually connected, cancel the pending flag.
    if (WiFi.status() != WL_CONNECTED) {
        wifiConnectPending = false;
    }

    if (c5IsLinked()) {
        // Existing authenticated link (for example after Wardrive): refresh
        // 5 GHz immediately while the watch starts its local 2.4 GHz scan.
        requestC5Scan();
        beginWifiScan();
    } else if (!c5TransportConnected() && startC5DiscoveryAsync()) {
        wifiInitialC5LinkPending = true;
        if (wifiStatusLabel) {
            lv_label_set_text(
                wifiStatusLabel,
                "LOOKING FOR M33K X C5..."
            );
        }
    } else if (c5TransportConnected()) {
        // A transport is already up and authentication is still finishing.
        wifiInitialC5LinkPending = true;
        if (wifiStatusLabel) {
            lv_label_set_text(
                wifiStatusLabel,
                "C5 AUTHENTICATING..."
            );
        }
    } else {
        // Discovery could not start (busy/retry window). Do not block the
        // local scanner; it can still operate as 2.4 GHz-only.
        beginWifiScan();
    }

    noteActivity();
}




// -----------------------------------------------------------------------------
// Time / NTP settings
// -----------------------------------------------------------------------------

void settingsKeyboardEvent(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        lv_obj_t *kb = static_cast<lv_obj_t *>(lv_event_get_target(e));
        lv_keyboard_set_textarea(kb, nullptr);
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
        if (settingsKeyboardShade) {
            lv_obj_add_flag(settingsKeyboardShade, LV_OBJ_FLAG_HIDDEN);
        }
        noteActivity();
    }
}

void openSettingsKeyboardFor(lv_obj_t *ta)
{
    if (!settingsKeyboard || !ta) return;

    if (settingsContent) {
        int32_t targetY = lv_obj_get_y(ta) - 70;
        if (targetY < 0) targetY = 0;
        lv_obj_scroll_to_y(settingsContent, targetY, LV_ANIM_OFF);
    }

    lv_obj_add_state(ta, LV_STATE_FOCUSED);
    lv_keyboard_set_mode(settingsKeyboard, LV_KEYBOARD_MODE_TEXT_LOWER);
    lv_keyboard_set_textarea(settingsKeyboard, ta);
    lv_obj_set_style_bg_color(settingsKeyboard, lv_color_hex(0x07101D), 0);
    lv_obj_set_style_bg_opa(settingsKeyboard, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(settingsKeyboard, 2, 0);
    lv_obj_set_style_border_color(settingsKeyboard, lv_color_hex(0x55EEFF), 0);

    if (settingsKeyboardShade) {
        lv_obj_remove_flag(settingsKeyboardShade, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(settingsKeyboardShade);
    }

    lv_obj_remove_flag(settingsKeyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(settingsKeyboard);
    lv_obj_invalidate(settingsKeyboard);
    // Avoid calling lv_timer_handler() recursively from an LVGL input event;
    // the normal loop will render the keyboard immediately on the next tick.
    noteActivity();
}

void settingsTextAreaEvent(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_PRESSED && code != LV_EVENT_CLICKED && code != LV_EVENT_FOCUSED) return;
    lv_obj_t *ta = static_cast<lv_obj_t *>(lv_event_get_target(e));
    openSettingsKeyboardFor(ta);
}

void updatePasswordCountLabel()
{
    if (!settingsPasswordCountLabel || !settingsPasswordInput) return;
    const char *txt = lv_textarea_get_text(settingsPasswordInput);
    const size_t len = txt ? strlen(txt) : 0;
    lv_label_set_text_fmt(settingsPasswordCountLabel, "%u chars", static_cast<unsigned>(len));
}

void passwordInputChangedEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;

    // Keep the textarea's render mode synchronized with the SHOW PASSWORD
    // switch on every edit. This avoids the first typed character being
    // rendered using the previous hidden-password state.
    if (settingsPasswordInput && settingsShowPasswordSwitch) {
        const bool showPassword =
            lv_obj_has_state(settingsShowPasswordSwitch, LV_STATE_CHECKED);
        lv_textarea_set_password_mode(settingsPasswordInput, !showPassword);
        lv_obj_invalidate(settingsPasswordInput);
    }

    updatePasswordCountLabel();
}

void passwordKeyboardButtonEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;
    noteActivity();
    hapticTap();
    openSettingsKeyboardFor(settingsPasswordInput);
}



void brightnessSliderEvent(lv_event_t *e)
{
    if (!settingsBrightnessSlider) return;

    const lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_VALUE_CHANGED && code != LV_EVENT_RELEASED) return;

    int32_t value = lv_slider_get_value(settingsBrightnessSlider);
    value = constrain(value,
                      static_cast<int32_t>(M33K_MIN_BRIGHTNESS_PERCENT),
                      static_cast<int32_t>(100));
    displayBrightnessPercent = static_cast<uint8_t>(value);

    if (settingsBrightnessValueLabel) {
        lv_label_set_text_fmt(
            settingsBrightnessValueLabel,
            "%u%%",
            static_cast<unsigned>(displayBrightnessPercent)
        );
    }

    instance.setBrightness(displayBrightnessRaw());
    noteActivity();

    // Avoid repeatedly writing NVS while the thumb is moving.
    if (code == LV_EVENT_RELEASED) {
        saveDisplayBrightness();
    }
}

void timezoneChangedEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;

    const uint16_t selected = lv_dropdown_get_selected(settingsTzDropdown);

    // Dropdown item 0 is the explicit unset state. The real timezone list
    // begins at item 1 so saved timezone indices remain backward-compatible.
    if (selected == 0 || selected > TIMEZONE_COUNT) {
        timezoneIndex = TIMEZONE_UNSET;
    } else {
        timezoneIndex = static_cast<uint8_t>(selected - 1);
    }
    saveTimezone();

    if (settingsStatusLabel) {
        if (timezoneIndex < TIMEZONE_COUNT) {
            lv_label_set_text_fmt(settingsStatusLabel,
                                  "Timezone saved: %s  |  %s\nRTC unchanged until you tap SYNC NTP.",
                                  TIMEZONES[timezoneIndex].label,
                                  use24Hour ? "24-hour" : "12-hour");
        } else {
            lv_label_set_text_fmt(settingsStatusLabel,
                                  "TIME ZONE NOT SET  |  %s\nChoose a timezone before SYNC NTP.",
                                  use24Hour ? "24-hour" : "12-hour");
        }
    }

    noteActivity();
}

void setSettingsStatus(const char *text);

void passwordVisibilityEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    if (!settingsPasswordInput || !settingsShowPasswordSwitch) return;

    const bool showPassword =
        lv_obj_has_state(settingsShowPasswordSwitch, LV_STATE_CHECKED);

    lv_textarea_set_password_mode(settingsPasswordInput, !showPassword);
    lv_obj_invalidate(settingsPasswordInput);
    noteActivity();
}

void availableNetworkChangedEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    if (!settingsAvailableDropdown || !settingsSsidInput) return;

    uint16_t selected = lv_dropdown_get_selected(settingsAvailableDropdown);
    if (selected < availableSsidCount) {
        lv_textarea_set_text(settingsSsidInput, availableSsids[selected].c_str());
    }

    noteActivity();
}

void scanConnectNetworksEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();

    setSettingsStatus("Scanning nearby Wi-Fi...");

    // Fully reset the radio before scanning. Scanning while the STA driver is
    // mid-connect (auto-connect/weather retrying a network that isn't in
    // range) triggers "cannot stop scan process" and a LoadProhibited crash.
    // A plain disconnect isn't enough because ESP32 auto-reconnects straight
    // back into "connecting", so disable auto-reconnect, stop any scan, take
    // the radio down, then bring STA back up clean and scan.
    const bool wasConnected = (WiFi.status() == WL_CONNECTED);
    WiFi.setAutoReconnect(false);
    esp_wifi_scan_stop();
    WiFi.disconnect(true, false);      // stop connecting + radio off
    delay(200);
    WiFi.mode(WIFI_STA);
    delay(150);
    WiFi.scanDelete();

    int16_t count = WiFi.scanNetworks(false, true, true, 250);

    // Restore auto-reconnect only if we were deliberately connected before.
    WiFi.setAutoReconnect(wasConnected);

    availableSsidCount = 0;
    String options;

    if (count > 0) {
        for (int16_t i = 0;
             i < count && availableSsidCount < MAX_CONNECT_SSIDS;
             ++i) {

            String ssid = WiFi.SSID(i);
            if (ssid.length() == 0) continue;

            bool duplicate = false;
            for (uint8_t j = 0; j < availableSsidCount; ++j) {
                if (availableSsids[j] == ssid) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) continue;

            availableSsids[availableSsidCount] = ssid;
            if (options.length()) options += '\n';
            options += ssid;
            availableSsidCount++;
        }
    }

    if (settingsAvailableDropdown) {
        if (availableSsidCount > 0) {
            lv_dropdown_set_options(settingsAvailableDropdown, options.c_str());

            uint8_t selected = 0;
            for (uint8_t i = 0; i < availableSsidCount; ++i) {
                if (availableSsids[i] == savedWifiSsid) {
                    selected = i;
                    break;
                }
            }
            lv_dropdown_set_selected(settingsAvailableDropdown, selected);

            if (settingsSsidInput && lv_textarea_get_text(settingsSsidInput)[0] == '\0') {
                lv_textarea_set_text(settingsSsidInput, availableSsids[selected].c_str());
            }

            setSettingsStatus("Nearby networks loaded.\nSelect one, enter password, then CONNECT.");
        } else {
            lv_dropdown_set_options(settingsAvailableDropdown, "No networks found");
            setSettingsStatus("No connectable SSIDs found.\nTap SCAN to try again.");
        }
    }

    WiFi.scanDelete();

    if (WiFi.status() != WL_CONNECTED && !wifiStayConnected) {
        WiFi.mode(WIFI_OFF);
    }
}

void timeFormatChangedEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;

    uint16_t selected = lv_dropdown_get_selected(settingsFormatDropdown);
    use24Hour = (selected == 1);
    saveTimeFormat();
    updateHomeLiveData();

    if (settingsStatusLabel) {
        lv_label_set_text_fmt(settingsStatusLabel,
                              "Clock format saved: %s\nSync NTP only when you want to refresh the RTC from the network.",
                              use24Hour ? "24-hour" : "12-hour");
    }

    noteActivity();
}

String wifiConnectionSummary()
{
    if (WiFi.status() == WL_CONNECTED) {
        String s = "CONNECTED\n";
        s += WiFi.SSID();
        s += "  |  ";
        s += WiFi.localIP().toString();
        return s;
    }

    if (wifiConnectPending) {
        String s = "CONNECTING\n";
        s += savedWifiSsid;
        return s;
    }

    if (WiFi.getMode() != WIFI_OFF) {
        return "NOT CONNECTED\nWi-Fi radio on";
    }

    return "DISCONNECTED\nWi-Fi radio off";
}

void updateWifiManagerUi()
{
    if (!settingsWifiStateLabel) return;

    const auto status = WiFi.status();

    if (wifiConnectPending) {
        if (status == WL_CONNECTED) {
            wifiConnectPending = false;

            String success = "Connected to ";
            success += WiFi.SSID();
            success += "\nIP: ";
            success += WiFi.localIP().toString();
            setSettingsStatus(success.c_str());
        } else if (status == WL_NO_SSID_AVAIL) {
            wifiConnectPending = false;
            setSettingsStatus("Connection failed: SSID not found.\nCheck the selected network.");
        } else if (status == WL_CONNECT_FAILED) {
            wifiConnectPending = false;
            setSettingsStatus("Connection failed.\nCheck the Wi-Fi password/security.");
        } else if (millis() - wifiConnectStartedMs >= WIFI_CONNECT_TIMEOUT_MS) {
            wifiConnectPending = false;
            setSettingsStatus("Connection timed out after 15 sec.\nCheck password, signal, or security.");
            WiFi.disconnect(false, false);
        } else {
            uint32_t elapsed = (millis() - wifiConnectStartedMs) / 1000;
            uint32_t remain = 15 > elapsed ? 15 - elapsed : 0;
            String connecting = "CONNECTING\n";
            connecting += savedWifiSsid;
            connecting += "  |  ";
            connecting += String(remain);
            connecting += "s";
            lv_label_set_text(settingsWifiStateLabel, connecting.c_str());
            lv_obj_set_style_text_color(settingsWifiStateLabel, lv_color_hex(0xF5FF3B), 0);
            return;
        }
    }

    const String summary = wifiConnectionSummary();
    lv_label_set_text(settingsWifiStateLabel, summary.c_str());

    if (status == WL_CONNECTED) {
        lv_obj_set_style_text_color(settingsWifiStateLabel, lv_color_hex(0xB6FF00), 0);
    } else {
        lv_obj_set_style_text_color(settingsWifiStateLabel, lv_color_hex(0xDDE7F2), 0);
    }
}

void stayConnectedEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;

    wifiStayConnected = lv_obj_has_state(settingsStaySwitch, LV_STATE_CHECKED);
    saveStayConnected();

    // Turning persistence off also removes any previously stored Wi-Fi
    // credentials. The current in-memory values remain usable until reboot.
    if (!wifiStayConnected) {
        prefs.remove("ssid");
        prefs.remove("pass");
    }

    if (settingsStatusLabel) {
        lv_label_set_text_fmt(
            settingsStatusLabel,
            "Stay Connected: %s\nWi-Fi credentials persist only when ON.",
            wifiStayConnected ? "ON" : "OFF"
        );
    }

    noteActivity();
}

void wifiConnectEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();

    const char *ssidText = settingsSsidInput ? lv_textarea_get_text(settingsSsidInput) : "";
    const char *passText = settingsPasswordInput ? lv_textarea_get_text(settingsPasswordInput) : "";

    String ssid = String(ssidText);
    String password = String(passText);
    ssid.trim();

    if (ssid.length() == 0) {
        setSettingsStatus("Enter a Wi-Fi SSID first.");
        return;
    }

    saveWifiCredentials(ssid, password);

    if (settingsKeyboard) {
        lv_obj_add_flag(settingsKeyboard, LV_OBJ_FLAG_HIDDEN);
    }
    if (settingsKeyboardShade) {
        lv_obj_add_flag(settingsKeyboardShade, LV_OBJ_FLAG_HIDDEN);
    }

    wifiConnectPending = false;
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, false);
    delay(100);

    wifiConnectStartedMs = millis();
    wifiConnectPending = true;
    WiFi.begin(savedWifiSsid.c_str(), savedWifiPassword.c_str());

    setSettingsStatus("Connecting to Wi-Fi...\nWatching for success/failure for 15 sec.");
    updateWifiManagerUi();

    Serial.printf("[M33K] Wi-Fi connect requested: %s\n", savedWifiSsid.c_str());
}

void wifiDisconnectEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();

    wifiConnectPending = false;
    WiFi.disconnect(true, false);
    WiFi.mode(WIFI_OFF);

    setSettingsStatus("Wi-Fi disconnected.\nRTC time continues unchanged.");
    updateWifiManagerUi();

    Serial.println("[M33K] Wi-Fi disconnected by user");
}

void setSettingsStatus(const char *text)
{
    if (settingsStatusLabel) {
        lv_label_set_text(settingsStatusLabel, text);
        lv_timer_handler();
    }
}

void ntpSyncEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();

    if (timezoneIndex >= TIMEZONE_COUNT) {
        setSettingsStatus("Select a TIME ZONE before NTP sync.");
        return;
    }

    const char *ssidText = settingsSsidInput ? lv_textarea_get_text(settingsSsidInput) : "";
    const char *passText = settingsPasswordInput ? lv_textarea_get_text(settingsPasswordInput) : "";

    String ssid = String(ssidText);
    String password = String(passText);

    ssid.trim();

    if (ssid.length() == 0) {
        setSettingsStatus("Enter a Wi-Fi SSID first.");
        return;
    }

    saveWifiCredentials(ssid, password);
    saveTimezone();

    if (settingsKeyboard) lv_obj_add_flag(settingsKeyboard, LV_OBJ_FLAG_HIDDEN);

    wifiConnectPending = false;

    const bool alreadyConnectedToSelected =
        WiFi.status() == WL_CONNECTED && WiFi.SSID() == ssid;
    const bool connectionExistedBeforeSync = alreadyConnectedToSelected;

    if (!alreadyConnectedToSelected) {
        setSettingsStatus("Connecting to Wi-Fi...");
        WiFi.mode(WIFI_STA);
        WiFi.disconnect(false, false);
        delay(100);
        WiFi.begin(ssid.c_str(), password.c_str());

        uint32_t connectStart = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - connectStart < 12000) {
            instance.loop();
            lv_timer_handler();
            noteActivity();
            delay(40);
        }
    }

    if (WiFi.status() != WL_CONNECTED) {
        setSettingsStatus("Wi-Fi connection failed.\nCheck SSID/password and try again.");
        if (!connectionExistedBeforeSync) {
            WiFi.disconnect(true, false);
            WiFi.mode(WIFI_OFF);
        }
        return;
    }

    setSettingsStatus("Wi-Fi connected. Syncing NTP...");

    configTzTime(
        TIMEZONES[timezoneIndex].posix,
        "pool.ntp.org",
        "time.nist.gov",
        "time.google.com"
    );

    struct tm localTime = {};
    if (!getLocalTime(&localTime, 12000)) {
        setSettingsStatus("NTP did not return a time.\nTry again in a moment.");
        if (!connectionExistedBeforeSync && !wifiStayConnected) {
            WiFi.disconnect(true, false);
            WiFi.mode(WIFI_OFF);
        }
        return;
    }

    // Store local wall-clock time in the hardware RTC.
    // The selected POSIX timezone/DST rule was applied by configTzTime().
    instance.rtc.setDateTime(
        static_cast<uint16_t>(localTime.tm_year + 1900),
        static_cast<uint8_t>(localTime.tm_mon + 1),
        static_cast<uint8_t>(localTime.tm_mday),
        static_cast<uint8_t>(localTime.tm_hour),
        static_cast<uint8_t>(localTime.tm_min),
        static_cast<uint8_t>(localTime.tm_sec)
    );

    char status[128];
    strftime(status, sizeof(status), "NTP synced: %b %d  %I:%M %p", &localTime);

    String finalStatus = String(status);
    finalStatus += "\nRTC locked to this local time.";

    if (connectionExistedBeforeSync) {
        // SYNC NTP must never tear down a connection the user already had.
        finalStatus += " Existing Wi-Fi kept connected.";
    } else if (!wifiStayConnected) {
        // Only a temporary connection created specifically for NTP is closed.
        WiFi.disconnect(true, false);
        WiFi.mode(WIFI_OFF);
        finalStatus += " Temporary Wi-Fi off.";
    } else {
        finalStatus += " Wi-Fi stays connected.";
    }

    setSettingsStatus(finalStatus.c_str());
    updateWifiManagerUi();

    // Home clock reads the hardware RTC only. Network connect/disconnect
    // events never rewrite the RTC after this point.
    Serial.printf("[M33K] NTP wrote RTC once using %s\n", TIMEZONES[timezoneIndex].label);
}

void manualPageEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();
    showManualTimePage();
}

void settingsBackEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();
    showHome();
}

void manualBackEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();
    showSettingsPage();
}

void manualSaveEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();

    if (!manualYearSpin || !manualMonthSpin || !manualDaySpin ||
        !manualHourSpin || !manualMinuteSpin) {
        return;
    }

    int32_t year = lv_spinbox_get_value(manualYearSpin);
    int32_t month = lv_spinbox_get_value(manualMonthSpin);
    int32_t day = lv_spinbox_get_value(manualDaySpin);
    int32_t hour = lv_spinbox_get_value(manualHourSpin);
    int32_t minute = lv_spinbox_get_value(manualMinuteSpin);

    instance.rtc.setDateTime(
        static_cast<uint16_t>(year),
        static_cast<uint8_t>(month),
        static_cast<uint8_t>(day),
        static_cast<uint8_t>(hour),
        static_cast<uint8_t>(minute),
        0
    );

    if (manualStatusLabel) {
        lv_label_set_text_fmt(
            manualStatusLabel,
            "RTC set: %04ld-%02ld-%02ld  %02ld:%02ld",
            (long)year,
            (long)month,
            (long)day,
            (long)hour,
            (long)minute
        );
    }

    Serial.println("[M33K] RTC manually updated");
}


void manualSpinPlusEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    auto *spin = static_cast<lv_obj_t *>(lv_event_get_user_data(e));
    if (!spin) return;

    noteActivity();
    hapticTap();
    lv_spinbox_increment(spin);
}

void manualSpinMinusEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    auto *spin = static_cast<lv_obj_t *>(lv_event_get_user_data(e));
    if (!spin) return;

    noteActivity();
    hapticTap();
    lv_spinbox_decrement(spin);
}

lv_obj_t *createManualSpinRow(
    const char *label,
    int16_t y,
    int32_t minValue,
    int32_t maxValue,
    int32_t value,
    uint8_t digits,
    uint32_t accent,
    lv_obj_t **outSpin
)
{
    lv_obj_t *name = lv_label_create(screen);
    lv_label_set_text(name, label);
    lv_obj_set_style_text_color(name, lv_color_hex(accent), 0);
    lv_obj_set_style_text_font(name, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(name, 54, y + 12);

    lv_obj_t *spin = lv_spinbox_create(screen);
    lv_spinbox_set_range(spin, minValue, maxValue);
    lv_spinbox_set_digit_format(spin, digits, 0);
    lv_spinbox_set_step(spin, 1);
    lv_spinbox_set_value(spin, value);
    lv_obj_set_size(spin, 92, 42);
    lv_obj_set_pos(spin, 196, y);
    lv_obj_set_style_text_align(spin, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *minus = lv_button_create(screen);
    lv_obj_set_size(minus, 46, 42);
    lv_obj_set_pos(minus, 145, y);
    lv_obj_set_style_radius(minus, 13, 0);
    lv_obj_add_event_cb(
        minus,
        manualSpinMinusEvent,
        LV_EVENT_CLICKED,
        spin
    );

    lv_obj_t *minusText = lv_label_create(minus);
    lv_label_set_text(minusText, "-");
    lv_obj_set_style_text_font(minusText, &lv_font_montserrat_22, 0);
    lv_obj_center(minusText);

    lv_obj_t *plus = lv_button_create(screen);
    lv_obj_set_size(plus, 46, 42);
    lv_obj_set_pos(plus, 293, y);
    lv_obj_set_style_radius(plus, 13, 0);
    lv_obj_add_event_cb(
        plus,
        manualSpinPlusEvent,
        LV_EVENT_CLICKED,
        spin
    );

    lv_obj_t *plusText = lv_label_create(plus);
    lv_label_set_text(plusText, "+");
    lv_obj_set_style_text_font(plusText, &lv_font_montserrat_22, 0);
    lv_obj_center(plusText);

    *outSpin = spin;
    return spin;
}

lv_obj_t *createSafeHeaderBack(lv_event_cb_t callback)
{
    lv_obj_t *back = lv_button_create(screen);
    lv_obj_set_size(back, 46, 42);
    lv_obj_set_pos(back, 38, 17);
    lv_obj_set_style_radius(back, 14, 0);
    lv_obj_set_style_bg_color(back, lv_color_hex(0x101A2A), 0);
    lv_obj_set_style_border_width(back, 1, 0);
    lv_obj_set_style_border_color(back, lv_color_hex(0xFF4AD5), 0);
    lv_obj_set_ext_click_area(back, 6);
    lv_obj_add_event_cb(back, callback, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *icon = lv_label_create(back);
    lv_label_set_text(icon, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(icon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(icon);

    createPageBatteryIndicator();

    return back;
}


void updateStopwatchDisplay()
{
    if (!stopwatchTimeLabel) return;

    uint32_t elapsedMs = stopwatchAccumulatedMs;

    if (stopwatchRunning) {
        elapsedMs +=
            static_cast<uint32_t>(
                millis() - stopwatchStartedMs
            );
    }

    const uint32_t hours =
        elapsedMs / 3600000UL;

    const uint32_t minutes =
        (elapsedMs / 60000UL) % 60UL;

    const uint32_t seconds =
        (elapsedMs / 1000UL) % 60UL;

    const uint32_t hundredths =
        (elapsedMs / 10UL) % 100UL;

    char buffer[24];

    snprintf(
        buffer,
        sizeof(buffer),
        "%02lu:%02lu:%02lu.%02lu",
        static_cast<unsigned long>(hours),
        static_cast<unsigned long>(minutes),
        static_cast<unsigned long>(seconds),
        static_cast<unsigned long>(hundredths)
    );

    lv_label_set_text(
        stopwatchTimeLabel,
        buffer
    );

    if (stopwatchStartLabel) {
        lv_label_set_text(
            stopwatchStartLabel,
            stopwatchRunning
                ? "PAUSE"
                : "START"
        );
    }

    if (stopwatchStatusLabel) {
        lv_label_set_text(
            stopwatchStatusLabel,
            stopwatchRunning
                ? "RUNNING"
                : (
                    elapsedMs > 0
                        ? "PAUSED"
                        : "READY"
                )
        );

        lv_obj_set_style_text_color(
            stopwatchStatusLabel,
            stopwatchRunning
                ? lv_color_hex(0x7CFF45)
                : lv_color_hex(0x9AA8BA),
            0
        );
    }
}

void stopwatchTimerCb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    if (currentPage != Page::Stopwatch) {
        return;
    }

    updateStopwatchDisplay();
}

void stopwatchStartPauseEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();

    const uint32_t now =
        millis();

    if (stopwatchRunning) {
        stopwatchAccumulatedMs +=
            static_cast<uint32_t>(
                now - stopwatchStartedMs
            );

        stopwatchRunning = false;
    } else {
        stopwatchStartedMs = now;
        stopwatchRunning = true;
    }

    updateStopwatchDisplay();
}

void stopwatchResetEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();

    stopwatchRunning = false;
    stopwatchStartedMs = 0;
    stopwatchAccumulatedMs = 0;

    updateStopwatchDisplay();
}

void stopwatchBackEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();

    if (stopwatchTimer) {
        lv_timer_delete(stopwatchTimer);
        stopwatchTimer = nullptr;
    }

    showSettingsPage();
}

void showStopwatchPage()
{
    clearScreen();
    currentPage = Page::Stopwatch;

    createSafeHeaderBack(
        stopwatchBackEvent
    );

    lv_obj_t *title =
        lv_label_create(screen);

    lv_label_set_text(
        title,
        "STOPWATCH"
    );

    lv_obj_set_style_text_color(
        title,
        lv_color_hex(0xF4F6FF),
        0
    );

    lv_obj_set_style_text_font(
        title,
        &lv_font_montserrat_20,
        0
    );

    lv_obj_align(
        title,
        LV_ALIGN_TOP_MID,
        8,
        26
    );

    lv_obj_t *card =
        lv_obj_create(screen);

    lv_obj_set_size(
        card,
        350,
        190
    );

    lv_obj_set_pos(
        card,
        30,
        105
    );

    lv_obj_set_style_bg_color(
        card,
        lv_color_hex(0x050B14),
        0
    );

    lv_obj_set_style_bg_opa(
        card,
        230,
        0
    );

    lv_obj_set_style_border_width(
        card,
        1,
        0
    );

    lv_obj_set_style_border_color(
        card,
        lv_color_hex(0x55EEFF),
        0
    );

    lv_obj_set_style_radius(
        card,
        22,
        0
    );

    lv_obj_clear_flag(
        card,
        LV_OBJ_FLAG_SCROLLABLE
    );

    stopwatchStatusLabel =
        lv_label_create(card);

    lv_label_set_text(
        stopwatchStatusLabel,
        "READY"
    );

    lv_obj_set_style_text_font(
        stopwatchStatusLabel,
        &lv_font_montserrat_12,
        0
    );

    lv_obj_set_style_text_color(
        stopwatchStatusLabel,
        lv_color_hex(0x9AA8BA),
        0
    );

    lv_obj_align(
        stopwatchStatusLabel,
        LV_ALIGN_TOP_MID,
        0,
        18
    );

    stopwatchTimeLabel =
        lv_label_create(card);

    lv_label_set_text(
        stopwatchTimeLabel,
        "00:00:00.00"
    );

    lv_obj_set_style_text_font(
        stopwatchTimeLabel,
        &lv_font_montserrat_28,
        0
    );

    lv_obj_set_style_text_color(
        stopwatchTimeLabel,
        lv_color_hex(0x55EEFF),
        0
    );

    lv_obj_set_style_text_align(
        stopwatchTimeLabel,
        LV_TEXT_ALIGN_CENTER,
        0
    );

    lv_obj_align(
        stopwatchTimeLabel,
        LV_ALIGN_CENTER,
        0,
        3
    );

    lv_obj_t *hint =
        lv_label_create(card);

    lv_label_set_text(
        hint,
        "HH : MM : SS . 1/100"
    );

    lv_obj_set_style_text_font(
        hint,
        &lv_font_montserrat_10,
        0
    );

    lv_obj_set_style_text_color(
        hint,
        lv_color_hex(0xFF52C8),
        0
    );

    lv_obj_align(
        hint,
        LV_ALIGN_BOTTOM_MID,
        0,
        -18
    );

    lv_obj_t *startBtn =
        lv_button_create(screen);

    lv_obj_set_size(
        startBtn,
        150,
        60
    );

    lv_obj_set_pos(
        startBtn,
        45,
        330
    );

    lv_obj_set_style_radius(
        startBtn,
        18,
        0
    );

    lv_obj_set_style_bg_color(
        startBtn,
        lv_color_hex(0x09231E),
        0
    );

    lv_obj_set_style_border_width(
        startBtn,
        2,
        0
    );

    lv_obj_set_style_border_color(
        startBtn,
        lv_color_hex(0x7CFF45),
        0
    );

    lv_obj_add_event_cb(
        startBtn,
        stopwatchStartPauseEvent,
        LV_EVENT_CLICKED,
        nullptr
    );

    stopwatchStartLabel =
        lv_label_create(startBtn);

    lv_label_set_text(
        stopwatchStartLabel,
        stopwatchRunning
            ? "PAUSE"
            : "START"
    );

    lv_obj_set_style_text_color(
        stopwatchStartLabel,
        lv_color_hex(0x7CFF45),
        0
    );

    lv_obj_set_style_text_font(
        stopwatchStartLabel,
        &lv_font_montserrat_16,
        0
    );

    lv_obj_center(
        stopwatchStartLabel
    );

    lv_obj_t *resetBtn =
        lv_button_create(screen);

    lv_obj_set_size(
        resetBtn,
        150,
        60
    );

    lv_obj_set_pos(
        resetBtn,
        215,
        330
    );

    lv_obj_set_style_radius(
        resetBtn,
        18,
        0
    );

    lv_obj_set_style_bg_color(
        resetBtn,
        lv_color_hex(0x24101F),
        0
    );

    lv_obj_set_style_border_width(
        resetBtn,
        2,
        0
    );

    lv_obj_set_style_border_color(
        resetBtn,
        lv_color_hex(0xFF52C8),
        0
    );

    lv_obj_add_event_cb(
        resetBtn,
        stopwatchResetEvent,
        LV_EVENT_CLICKED,
        nullptr
    );

    lv_obj_t *resetLabel =
        lv_label_create(resetBtn);

    lv_label_set_text(
        resetLabel,
        "RESET"
    );

    lv_obj_set_style_text_color(
        resetLabel,
        lv_color_hex(0xFF72D8),
        0
    );

    lv_obj_set_style_text_font(
        resetLabel,
        &lv_font_montserrat_16,
        0
    );

    lv_obj_center(
        resetLabel
    );

    updateStopwatchDisplay();

    if (stopwatchTimer) {
        lv_timer_delete(
            stopwatchTimer
        );
    }

    stopwatchTimer =
        lv_timer_create(
            stopwatchTimerCb,
            50,
            nullptr
        );

    noteActivity();
}

void stopwatchSettingsEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();

    showStopwatchPage();
}


void showSettingsPage()
{
    clearScreen();
    currentPage = Page::Settings;

    createSafeHeaderBack(settingsBackEvent);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, LV_SYMBOL_SETTINGS "  SETTINGS");
    lv_obj_set_style_text_color(title, lv_color_hex(0xF4F6FF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 8, 26);

    settingsContent = lv_obj_create(screen);
    lv_obj_set_size(settingsContent, 350, 394);
    lv_obj_set_pos(settingsContent, 30, 78);
    lv_obj_set_style_bg_color(settingsContent, lv_color_hex(0x050B14), 0);
    lv_obj_set_style_bg_opa(settingsContent, 210, 0);
    lv_obj_set_style_border_width(settingsContent, 1, 0);
    lv_obj_set_style_border_color(settingsContent, lv_color_hex(0x172A42), 0);
    lv_obj_set_style_radius(settingsContent, 18, 0);
    lv_obj_set_scroll_dir(settingsContent, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(settingsContent, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_pad_all(settingsContent, 0, 0);

    // Time zone + clock format
    lv_obj_t *tzLabel = lv_label_create(settingsContent);
    lv_label_set_text(tzLabel, "TIME ZONE");
    lv_obj_set_style_text_color(tzLabel, lv_color_hex(0x7CFF45), 0);
    lv_obj_set_style_text_font(tzLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(tzLabel, 12, 12);

    settingsTzDropdown = lv_dropdown_create(settingsContent);
    lv_dropdown_set_options(
        settingsTzDropdown,
        "Not set - choose\n"
        "Pacific (DST)\n"
        "Mountain (DST)\n"
        "Central (DST)\n"
        "Eastern (DST)\n"
        "Arizona\n"
        "Alaska (DST)\n"
        "Hawaii\n"
        "UTC"
    );
    lv_dropdown_set_selected(
        settingsTzDropdown,
        timezoneIndex < TIMEZONE_COUNT ? static_cast<uint16_t>(timezoneIndex + 1) : 0
    );
    lv_obj_set_size(settingsTzDropdown, 198, 40);
    lv_obj_set_pos(settingsTzDropdown, 12, 31);
    lv_obj_add_event_cb(
        settingsTzDropdown,
        timezoneChangedEvent,
        LV_EVENT_VALUE_CHANGED,
        nullptr
    );

    lv_obj_t *fmtLabel = lv_label_create(settingsContent);
    lv_label_set_text(fmtLabel, "CLOCK");
    lv_obj_set_style_text_color(fmtLabel, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_text_font(fmtLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(fmtLabel, 222, 12);

    settingsFormatDropdown = lv_dropdown_create(settingsContent);
    lv_dropdown_set_options(settingsFormatDropdown, "12-hour\n24-hour");
    lv_dropdown_set_selected(settingsFormatDropdown, use24Hour ? 1 : 0);
    lv_obj_set_size(settingsFormatDropdown, 108, 40);
    lv_obj_set_pos(settingsFormatDropdown, 222, 31);
    lv_obj_add_event_cb(
        settingsFormatDropdown,
        timeFormatChangedEvent,
        LV_EVENT_VALUE_CHANGED,
        nullptr
    );

    // Available networks
    lv_obj_t *availableLabel = lv_label_create(settingsContent);
    lv_label_set_text(availableLabel, "AVAILABLE WI-FI");
    lv_obj_set_style_text_color(availableLabel, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_text_font(availableLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(availableLabel, 12, 86);

    settingsAvailableDropdown = lv_dropdown_create(settingsContent);
    lv_dropdown_set_options(settingsAvailableDropdown, "Tap SCAN");
    lv_obj_set_size(settingsAvailableDropdown, 236, 40);
    lv_obj_set_pos(settingsAvailableDropdown, 12, 105);
    lv_obj_add_event_cb(
        settingsAvailableDropdown,
        availableNetworkChangedEvent,
        LV_EVENT_VALUE_CHANGED,
        nullptr
    );

    lv_obj_t *scanBtn = lv_button_create(settingsContent);
    lv_obj_set_size(scanBtn, 76, 40);
    lv_obj_set_pos(scanBtn, 254, 105);
    lv_obj_set_style_radius(scanBtn, 13, 0);
    lv_obj_set_style_bg_color(scanBtn, lv_color_hex(0x0A2030), 0);
    lv_obj_set_style_border_width(scanBtn, 1, 0);
    lv_obj_set_style_border_color(scanBtn, lv_color_hex(0x55EEFF), 0);
    lv_obj_add_event_cb(
        scanBtn,
        scanConnectNetworksEvent,
        LV_EVENT_CLICKED,
        nullptr
    );

    lv_obj_t *scanText = lv_label_create(scanBtn);
    lv_label_set_text(scanText, "SCAN");
    lv_obj_set_style_text_color(scanText, lv_color_hex(0x55EEFF), 0);
    lv_obj_center(scanText);

    // SSID is still editable for hidden/manual networks.
    lv_obj_t *ssidLabel = lv_label_create(settingsContent);
    lv_label_set_text(ssidLabel, "SSID");
    lv_obj_set_style_text_color(ssidLabel, lv_color_hex(0xDDE7F2), 0);
    lv_obj_set_style_text_font(ssidLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(ssidLabel, 12, 157);

    settingsSsidInput = lv_textarea_create(settingsContent);
    lv_textarea_set_one_line(settingsSsidInput, true);
    lv_textarea_set_text(settingsSsidInput, savedWifiSsid.c_str());
    lv_textarea_set_placeholder_text(settingsSsidInput, "Select above or type SSID");
    lv_obj_set_size(settingsSsidInput, 318, 40);
    lv_obj_set_pos(settingsSsidInput, 12, 176);
    lv_obj_set_style_bg_color(settingsSsidInput, lv_color_hex(0x091525), 0);
    lv_obj_set_style_bg_opa(settingsSsidInput, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(settingsSsidInput, 1, 0);
    lv_obj_set_style_border_color(settingsSsidInput, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_text_color(settingsSsidInput, lv_color_hex(0xFFFFFF), 0);
    lv_obj_add_flag(settingsSsidInput, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_obj_add_flag(settingsSsidInput, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(
        settingsSsidInput,
        settingsTextAreaEvent,
        LV_EVENT_ALL,
        nullptr
    );

    // Password + show/hide
    lv_obj_t *passLabel = lv_label_create(settingsContent);
    lv_label_set_text(passLabel, "PASSWORD");
    lv_obj_set_style_text_color(passLabel, lv_color_hex(0xFF52C8), 0);
    lv_obj_set_style_text_font(passLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(passLabel, 12, 229);

    settingsPasswordInput = lv_textarea_create(settingsContent);
    lv_textarea_set_one_line(settingsPasswordInput, true);
    lv_textarea_set_password_mode(settingsPasswordInput, true);
    lv_textarea_set_text(settingsPasswordInput, savedWifiPassword.c_str());
    lv_textarea_set_placeholder_text(settingsPasswordInput, "Wi-Fi password");
    lv_obj_set_size(settingsPasswordInput, 318, 40);
    lv_obj_set_pos(settingsPasswordInput, 12, 248);
    lv_obj_set_style_bg_color(settingsPasswordInput, lv_color_hex(0x190B19), 0);
    lv_obj_set_style_bg_opa(settingsPasswordInput, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(settingsPasswordInput, 1, 0);
    lv_obj_set_style_border_color(settingsPasswordInput, lv_color_hex(0xFF52C8), 0);
    lv_obj_set_style_text_color(settingsPasswordInput, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_color(settingsPasswordInput, lv_color_hex(0x55EEFF), LV_PART_CURSOR);
    lv_obj_add_flag(settingsPasswordInput, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_obj_add_flag(settingsPasswordInput, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(
        settingsPasswordInput,
        settingsTextAreaEvent,
        LV_EVENT_ALL,
        nullptr
    );
    lv_obj_add_event_cb(
        settingsPasswordInput,
        passwordInputChangedEvent,
        LV_EVENT_VALUE_CHANGED,
        nullptr
    );

    settingsPasswordCountLabel = lv_label_create(settingsContent);
    lv_obj_set_style_text_color(settingsPasswordCountLabel, lv_color_hex(0x9AA8BA), 0);
    lv_obj_set_style_text_font(settingsPasswordCountLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(settingsPasswordCountLabel, 252, 298);
    updatePasswordCountLabel();

    lv_obj_t *showLabel = lv_label_create(settingsContent);
    lv_label_set_text(showLabel, "SHOW PASSWORD");
    lv_obj_set_style_text_color(showLabel, lv_color_hex(0xF4F6FF), 0);
    lv_obj_set_style_text_font(showLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(showLabel, 12, 298);

    settingsShowPasswordSwitch = lv_switch_create(settingsContent);
    lv_obj_set_size(settingsShowPasswordSwitch, 64, 30);
    lv_obj_set_pos(settingsShowPasswordSwitch, 132, 291);
    lv_obj_add_event_cb(settingsShowPasswordSwitch, passwordVisibilityEvent, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_t *keyboardBtn = lv_button_create(settingsContent);
    lv_obj_set_size(keyboardBtn, 110, 38);
    lv_obj_set_pos(keyboardBtn, 220, 327);
    lv_obj_set_style_radius(keyboardBtn, 13, 0);
    lv_obj_set_style_bg_color(keyboardBtn, lv_color_hex(0x0A2030), 0);
    lv_obj_set_style_border_width(keyboardBtn, 1, 0);
    lv_obj_set_style_border_color(keyboardBtn, lv_color_hex(0x55EEFF), 0);
    lv_obj_add_event_cb(keyboardBtn, passwordKeyboardButtonEvent, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *keyboardText = lv_label_create(keyboardBtn);
    lv_label_set_text(keyboardText, "KEYBOARD");
    lv_obj_set_style_text_color(keyboardText, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_text_font(keyboardText, &lv_font_montserrat_12, 0);
    lv_obj_center(keyboardText);

    lv_obj_t *stayLabel = lv_label_create(settingsContent);
    lv_label_set_text(stayLabel, "STAY CONNECTED");
    lv_obj_set_style_text_color(stayLabel, lv_color_hex(0x7CFF45), 0);
    lv_obj_set_style_text_font(stayLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(stayLabel, 12, 374);

    settingsStaySwitch = lv_switch_create(settingsContent);
    lv_obj_set_size(settingsStaySwitch, 64, 30);
    lv_obj_set_pos(settingsStaySwitch, 252, 367);
    if (wifiStayConnected) lv_obj_add_state(settingsStaySwitch, LV_STATE_CHECKED);
    lv_obj_add_event_cb(settingsStaySwitch, stayConnectedEvent, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_t *connectBtn = lv_button_create(settingsContent);
    lv_obj_set_size(connectBtn, 148, 42);
    lv_obj_set_pos(connectBtn, 12, 416);
    lv_obj_set_style_radius(connectBtn, 14, 0);
    lv_obj_set_style_bg_color(connectBtn, lv_color_hex(0x10231A), 0);
    lv_obj_set_style_border_width(connectBtn, 1, 0);
    lv_obj_set_style_border_color(connectBtn, lv_color_hex(0x7CFF45), 0);
    lv_obj_add_event_cb(connectBtn, wifiConnectEvent, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *connectText = lv_label_create(connectBtn);
    lv_label_set_text(connectText, "CONNECT");
    lv_obj_set_style_text_color(connectText, lv_color_hex(0x7CFF45), 0);
    lv_obj_center(connectText);

    lv_obj_t *disconnectBtn = lv_button_create(settingsContent);
    lv_obj_set_size(disconnectBtn, 148, 42);
    lv_obj_set_pos(disconnectBtn, 182, 416);
    lv_obj_set_style_radius(disconnectBtn, 14, 0);
    lv_obj_set_style_bg_color(disconnectBtn, lv_color_hex(0x241015), 0);
    lv_obj_set_style_border_width(disconnectBtn, 1, 0);
    lv_obj_set_style_border_color(disconnectBtn, lv_color_hex(0xFF667F), 0);
    lv_obj_add_event_cb(
        disconnectBtn,
        wifiDisconnectEvent,
        LV_EVENT_CLICKED,
        nullptr
    );

    lv_obj_t *disconnectText = lv_label_create(disconnectBtn);
    lv_label_set_text(disconnectText, "DISCONNECT");
    lv_obj_set_style_text_color(disconnectText, lv_color_hex(0xFF8799), 0);
    lv_obj_center(disconnectText);

    settingsWifiStateLabel = lv_label_create(settingsContent);
    lv_label_set_text(settingsWifiStateLabel, "DISCONNECTED\nWi-Fi radio off");
    lv_obj_set_width(settingsWifiStateLabel, 318);
    lv_obj_set_style_text_font(settingsWifiStateLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_align(settingsWifiStateLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(settingsWifiStateLabel, 12, 471);

    lv_obj_t *syncBtn = lv_button_create(settingsContent);
    lv_obj_set_size(syncBtn, 148, 42);
    lv_obj_set_pos(syncBtn, 12, 522);
    lv_obj_set_style_radius(syncBtn, 14, 0);
    lv_obj_set_style_bg_color(syncBtn, lv_color_hex(0x0A2030), 0);
    lv_obj_set_style_border_width(syncBtn, 1, 0);
    lv_obj_set_style_border_color(syncBtn, lv_color_hex(0x55EEFF), 0);
    lv_obj_add_event_cb(syncBtn, ntpSyncEvent, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *syncText = lv_label_create(syncBtn);
    lv_label_set_text(syncText, "SYNC NTP");
    lv_obj_set_style_text_color(syncText, lv_color_hex(0x55EEFF), 0);
    lv_obj_center(syncText);

    lv_obj_t *manualBtn = lv_button_create(settingsContent);
    lv_obj_set_size(manualBtn, 148, 42);
    lv_obj_set_pos(manualBtn, 182, 522);
    lv_obj_set_style_radius(manualBtn, 14, 0);
    lv_obj_set_style_bg_color(manualBtn, lv_color_hex(0x201127), 0);
    lv_obj_set_style_border_width(manualBtn, 1, 0);
    lv_obj_set_style_border_color(manualBtn, lv_color_hex(0xFF52C8), 0);
    lv_obj_add_event_cb(
        manualBtn,
        manualPageEvent,
        LV_EVENT_CLICKED,
        nullptr
    );

    lv_obj_t *manualText = lv_label_create(manualBtn);
    lv_label_set_text(manualText, "MANUAL TIME");
    lv_obj_set_style_text_color(manualText, lv_color_hex(0xFF8AD9), 0);
    lv_obj_center(manualText);

    // Development build: phone/web integration is deliberately not active yet.
    lv_obj_t *dashboardCard = lv_obj_create(settingsContent);
    lv_obj_set_size(dashboardCard, 318, 54);
    lv_obj_set_pos(dashboardCard, 12, 580);
    lv_obj_set_style_radius(dashboardCard, 14, 0);
    lv_obj_set_style_bg_color(dashboardCard, lv_color_hex(0x10172A), 0);
    lv_obj_set_style_border_width(dashboardCard, 1, 0);
    lv_obj_set_style_border_color(dashboardCard, lv_color_hex(0x6D86FF), 0);
    lv_obj_clear_flag(dashboardCard, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *dashboardText = lv_label_create(dashboardCard);
    lv_label_set_text(dashboardText, "M33K X DASHBOARD\nIN PROGRESS  |  NOT ENABLED");
    lv_obj_set_width(dashboardText, 292);
    lv_obj_set_style_text_align(dashboardText, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(dashboardText, lv_color_hex(0x9CB0FF), 0);
    lv_obj_set_style_text_font(dashboardText, &lv_font_montserrat_12, 0);
    lv_obj_center(dashboardText);

    settingsStatusLabel = lv_label_create(settingsContent);
    if (timezoneIndex < TIMEZONE_COUNT) {
        lv_label_set_text_fmt(
            settingsStatusLabel,
            "%s  |  %s\nRTC changes only on manual set or NTP sync.",
            TIMEZONES[timezoneIndex].label,
            use24Hour ? "24-hour" : "12-hour"
        );
    } else {
        lv_label_set_text_fmt(
            settingsStatusLabel,
            "TIME ZONE NOT SET  |  %s\nChoose a timezone before SYNC NTP.",
            use24Hour ? "24-hour" : "12-hour"
        );
    }
    lv_obj_set_width(settingsStatusLabel, 318);
    lv_obj_set_style_text_color(settingsStatusLabel, lv_color_hex(0xE5ECF5), 0);
    lv_obj_set_style_text_font(settingsStatusLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_align(settingsStatusLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(settingsStatusLabel, 12, 642);

    lv_obj_t *brightnessLabel = lv_label_create(settingsContent);
    lv_label_set_text(brightnessLabel, "DISPLAY BRIGHTNESS");
    lv_obj_set_style_text_color(brightnessLabel, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_text_font(brightnessLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(brightnessLabel, 12, 700);

    settingsBrightnessValueLabel = lv_label_create(settingsContent);
    lv_label_set_text_fmt(
        settingsBrightnessValueLabel,
        "%u%%",
        static_cast<unsigned>(displayBrightnessPercent)
    );
    lv_obj_set_style_text_color(
        settingsBrightnessValueLabel,
        lv_color_hex(0x7CFF45),
        0
    );
    lv_obj_set_style_text_font(
        settingsBrightnessValueLabel,
        &lv_font_montserrat_12,
        0
    );
    lv_obj_set_pos(settingsBrightnessValueLabel, 292, 700);

    settingsBrightnessSlider = lv_slider_create(settingsContent);
    lv_slider_set_range(
        settingsBrightnessSlider,
        M33K_MIN_BRIGHTNESS_PERCENT,
        100
    );
    lv_slider_set_value(
        settingsBrightnessSlider,
        displayBrightnessPercent,
        LV_ANIM_OFF
    );
    lv_obj_set_size(settingsBrightnessSlider, 318, 20);
    lv_obj_set_pos(settingsBrightnessSlider, 12, 725);
    lv_obj_set_style_bg_color(
        settingsBrightnessSlider,
        lv_color_hex(0x17314A),
        LV_PART_MAIN
    );
    lv_obj_set_style_bg_color(
        settingsBrightnessSlider,
        lv_color_hex(0x55EEFF),
        LV_PART_INDICATOR
    );
    lv_obj_set_style_bg_color(
        settingsBrightnessSlider,
        lv_color_hex(0xFF52C8),
        LV_PART_KNOB
    );
    lv_obj_add_event_cb(
        settingsBrightnessSlider,
        brightnessSliderEvent,
        LV_EVENT_VALUE_CHANGED,
        nullptr
    );
    lv_obj_add_event_cb(
        settingsBrightnessSlider,
        brightnessSliderEvent,
        LV_EVENT_RELEASED,
        nullptr
    );

    // Utilities
    lv_obj_t *utilitiesLabel = lv_label_create(settingsContent);
    lv_label_set_text(utilitiesLabel, "UTILITIES");
    lv_obj_set_style_text_color(utilitiesLabel, lv_color_hex(0xFF52C8), 0);
    lv_obj_set_style_text_font(utilitiesLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(utilitiesLabel, 12, 772);

    lv_obj_t *stopwatchBtn = lv_button_create(settingsContent);
    lv_obj_set_size(stopwatchBtn, 318, 48);
    lv_obj_set_pos(stopwatchBtn, 12, 795);
    lv_obj_set_style_radius(stopwatchBtn, 15, 0);
    lv_obj_set_style_bg_color(stopwatchBtn, lv_color_hex(0x081823), 0);
    lv_obj_set_style_border_width(stopwatchBtn, 1, 0);
    lv_obj_set_style_border_color(stopwatchBtn, lv_color_hex(0x55EEFF), 0);
    lv_obj_add_event_cb(
        stopwatchBtn,
        stopwatchSettingsEvent,
        LV_EVENT_CLICKED,
        nullptr
    );

    lv_obj_t *stopwatchBtnLabel = lv_label_create(stopwatchBtn);
    lv_label_set_text(stopwatchBtnLabel, "STOPWATCH");
    lv_obj_set_style_text_color(stopwatchBtnLabel, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_text_font(stopwatchBtnLabel, &lv_font_montserrat_14, 0);
    lv_obj_center(stopwatchBtnLabel);

    // Invisible spacer keeps Utilities reachable at the bottom.
    lv_obj_t *spacer = lv_obj_create(settingsContent);
    lv_obj_remove_style_all(spacer);
    lv_obj_set_size(spacer, 1, 1);
    lv_obj_set_pos(spacer, 1, 865);

    updateWifiManagerUi();

    if (settingsKeyboard) {
        lv_obj_del(settingsKeyboard);
        settingsKeyboard = nullptr;
    }
    if (settingsKeyboardShade) {
        lv_obj_del(settingsKeyboardShade);
        settingsKeyboardShade = nullptr;
    }

    settingsKeyboardShade = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(settingsKeyboardShade);
    lv_obj_set_size(settingsKeyboardShade, 410, 502);
    lv_obj_set_pos(settingsKeyboardShade, 0, 0);
    lv_obj_set_style_bg_color(settingsKeyboardShade, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(settingsKeyboardShade, 115, 0);
    lv_obj_add_flag(settingsKeyboardShade, LV_OBJ_FLAG_HIDDEN);

    settingsKeyboard = lv_keyboard_create(lv_layer_top());
    // Narrow enough to clear both rounded edges, then truly center it.
    // Extra clearance for the Ultra's curved lower-right corner.
    // Same keyboard size as v0.3.28, but actually moved away from the
    // lower-right curved corner instead of merely shrinking it.
    // right edge: 345 -> 312, bottom edge: 474 -> 454
    // Shift the keyboard left and make it a little wider so the large
    // empty gap on the left goes away, while also lifting it slightly to
    // protect the lower-right keys from the rounded screen corner.
    // Restore the original compact keyboard layout the user preferred.
    // The invisible top-layer cleanup fix remains in clearScreen().
    lv_obj_set_size(settingsKeyboard, 324, 172);
    lv_obj_set_pos(settingsKeyboard, 0, 268);
    lv_obj_set_style_radius(settingsKeyboard, 18, 0);
    lv_obj_set_style_clip_corner(settingsKeyboard, true, 0);
    lv_keyboard_set_mode(settingsKeyboard, LV_KEYBOARD_MODE_TEXT_LOWER);
    lv_obj_add_flag(settingsKeyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(
        settingsKeyboard,
        settingsKeyboardEvent,
        LV_EVENT_ALL,
        nullptr
    );

    noteActivity();
}


void showManualTimePage()
{
    clearScreen();
    currentPage = Page::Settings;

    createSafeHeaderBack(manualBackEvent);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "MANUAL DATE / TIME");
    lv_obj_set_style_text_color(title, lv_color_hex(0xF4F6FF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 12, 26);

    RTC_DateTime now = instance.rtc.getDateTime();

    createManualSpinRow(
        "YEAR", 88, 2025, 2099, now.getYear(), 4,
        0x7CFF45, &manualYearSpin
    );
    createManualSpinRow(
        "MONTH", 140, 1, 12, now.getMonth(), 2,
        0x7CFF45, &manualMonthSpin
    );
    createManualSpinRow(
        "DAY", 192, 1, 31, now.getDay(), 2,
        0x7CFF45, &manualDaySpin
    );
    createManualSpinRow(
        "HOUR", 260, 0, 23, now.getHour(), 2,
        0x55EEFF, &manualHourSpin
    );
    createManualSpinRow(
        "MINUTE", 312, 0, 59, now.getMinute(), 2,
        0xFF52C8, &manualMinuteSpin
    );

    lv_obj_t *saveBtn = lv_button_create(screen);
    lv_obj_set_size(saveBtn, 180, 50);
    lv_obj_align(saveBtn, LV_ALIGN_TOP_MID, 0, 382);
    lv_obj_set_style_radius(saveBtn, 16, 0);
    lv_obj_set_style_bg_color(saveBtn, lv_color_hex(0x10231A), 0);
    lv_obj_set_style_border_width(saveBtn, 1, 0);
    lv_obj_set_style_border_color(saveBtn, lv_color_hex(0x7CFF45), 0);
    lv_obj_add_event_cb(saveBtn, manualSaveEvent, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *saveText = lv_label_create(saveBtn);
    lv_label_set_text(saveText, "SET RTC");
    lv_obj_set_style_text_color(saveText, lv_color_hex(0x7CFF45), 0);
    lv_obj_center(saveText);

    manualStatusLabel = lv_label_create(screen);
    lv_label_set_text(
        manualStatusLabel,
        "Use - / + to adjust each value."
    );
    lv_obj_set_width(manualStatusLabel, 320);
    lv_obj_set_style_text_color(manualStatusLabel, lv_color_hex(0xDDE7F2), 0);
    lv_obj_set_style_text_font(manualStatusLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_align(manualStatusLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(manualStatusLabel, LV_ALIGN_TOP_MID, 0, 447);

    noteActivity();
}


// -----------------------------------------------------------------------------
// Placeholder pages
// -----------------------------------------------------------------------------


void showWifiGraphPage()
{
    clearScreen();
    currentPage = Page::WiFi;
    wifiPendingNav = WifiPendingNav::None;
    wifiGraphView = true;

    lv_obj_t *back = lv_button_create(screen);
    lv_obj_set_size(back, 44, 40);
    lv_obj_set_pos(back, 54, 24);
    lv_obj_set_style_radius(back, 14, 0);
    lv_obj_set_style_bg_color(back, lv_color_hex(0x101A2A), 0);
    lv_obj_set_style_border_width(back, 1, 0);
    lv_obj_set_style_border_color(back, lv_color_hex(0xFF4AD5), 0);
    lv_obj_set_ext_click_area(back, 6);
    lv_obj_add_event_cb(back, wifiGraphBackEvent, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *backIcon = lv_label_create(back);
    lv_label_set_text(backIcon, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(backIcon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(backIcon);

    lv_obj_t *refresh = lv_button_create(screen);
    lv_obj_set_size(refresh, 44, 40);
    lv_obj_set_pos(refresh, 312, 24);
    lv_obj_set_style_radius(refresh, 14, 0);
    lv_obj_set_style_bg_color(refresh, lv_color_hex(0x101A2A), 0);
    lv_obj_set_style_border_width(refresh, 1, 0);
    lv_obj_set_style_border_color(refresh, lv_color_hex(0x2F6BFF), 0);
    lv_obj_set_ext_click_area(refresh, 6);
    lv_obj_add_event_cb(refresh, wifiRefreshEvent, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *refreshIcon = lv_label_create(refresh);
    lv_label_set_text(refreshIcon, LV_SYMBOL_REFRESH);
    lv_obj_set_style_text_color(refreshIcon, lv_color_hex(0x2F6BFF), 0);
    lv_obj_center(refreshIcon);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "WI-FI CHANNELS");
    lv_obj_set_style_text_color(title, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 72);

    createPageBatteryIndicator();

    wifiStatusLabel = lv_label_create(screen);
    lv_label_set_text(wifiStatusLabel, "Opening channel graph...");
    lv_obj_set_width(wifiStatusLabel, 320);
    lv_obj_set_style_text_color(wifiStatusLabel, lv_color_hex(0xB6FF00), 0);
    lv_obj_set_style_text_font(wifiStatusLabel, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_align(wifiStatusLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_bg_color(wifiStatusLabel, lv_color_hex(0x02060A), 0);
    lv_obj_set_style_bg_opa(wifiStatusLabel, 185, 0);
    lv_obj_set_style_radius(wifiStatusLabel, 9, 0);
    lv_obj_set_style_pad_hor(wifiStatusLabel, 6, 0);
    lv_obj_set_style_pad_ver(wifiStatusLabel, 3, 0);
    lv_obj_set_pos(wifiStatusLabel, 45, 96);

    wifiGraphContainer = lv_obj_create(screen);
    lv_obj_set_size(wifiGraphContainer, 350, 330);
    lv_obj_set_pos(wifiGraphContainer, 30, 136);
    lv_obj_set_style_bg_color(wifiGraphContainer, lv_color_hex(0x040A12), 0);
    lv_obj_set_style_bg_opa(wifiGraphContainer, 103, 0);
    lv_obj_set_style_border_width(wifiGraphContainer, 1, 0);
    lv_obj_set_style_border_color(wifiGraphContainer, lv_color_hex(0x142A43), 0);
    lv_obj_set_style_radius(wifiGraphContainer, 16, 0);
    lv_obj_set_style_pad_all(wifiGraphContainer, 8, 0);
    lv_obj_set_scrollbar_mode(wifiGraphContainer, LV_SCROLLBAR_MODE_OFF);

    wifiGraphLive = true;
    wifiGraphNextScanMs = 0;
    wifiGraphHasRender = false;
    wifiGraphRenderedSignature = 0;
    c5DiscoveryRequestedByWifi = false;
    resetWifiGraphDrops();

    // Fast timer drives the skull-drop animation while asynchronous Wi-Fi
    // scans continue at a much slower interval.
    wifiTimer = lv_timer_create(wifiTimerCb, 60, nullptr);

    if (wifiDetailCount > 0) {
        renderWifiGraphIfNeeded(true);
    }

    beginWifiScan();
    wifiSubpageTransition = false;
    noteActivity();
}


const char *bleAddressTypeLabel(uint8_t type)
{
    return type == 0 ? "PUBLIC" : "RANDOM";
}

void bleDetailCloseEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    if (bleDetailOverlay) {
        lv_obj_add_flag(bleDetailOverlay, LV_OBJ_FLAG_HIDDEN);
    }

    noteActivity();
}

void bleDetailEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    auto *detail = static_cast<BleResultDetail *>(lv_event_get_user_data(e));
    if (!detail || !bleDetailOverlay || !bleDetailTitle || !bleDetailBody) return;

    noteActivity();
    hapticTap();

    lv_label_set_text(bleDetailTitle, detail->name.c_str());

    char txText[20];
    if (detail->haveTxPower) {
        snprintf(txText, sizeof(txText), "%d dBm", detail->txPower);
    } else {
        snprintf(txText, sizeof(txText), "--");
    }

    char body[360];
    snprintf(
        body,
        sizeof(body),
        "ADDRESS\n%s\n\nRSSI   %ld dBm   %s\nCONNECTABLE   %s\nSCANNABLE   %s\nADDR TYPE   %s\nTX POWER   %s\nSERVICE UUIDS   %u\nMFR DATA BLOCKS   %u",
        detail->address.c_str(),
        (long)detail->rssi,
        signalLabel(detail->rssi),
        detail->connectable ? "YES" : "NO",
        detail->scannable ? "YES" : "NO",
        bleAddressTypeLabel(detail->addressType),
        txText,
        detail->serviceCount,
        detail->manufacturerCount
    );

    lv_label_set_text(bleDetailBody, body);
    lv_obj_remove_flag(bleDetailOverlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(bleDetailOverlay);
}

void renderBleResults()
{
    if (!bleList || !bleStatusLabel || !bleScannerInitialized) return;

    NimBLEScan *scan = NimBLEDevice::getScan();
    NimBLEScanResults results = scan->getResults();

    const int count = results.getCount();

    lv_obj_clean(bleList);
    bleDetailCount = 0;

    if (count <= 0) {
        lv_label_set_text(bleStatusLabel, "No BLE devices found  |  tap refresh");

        lv_obj_t *none = lv_label_create(bleList);
        lv_label_set_text(none, "No BLE advertisements detected.");
        lv_obj_set_style_text_color(none, lv_color_hex(0xD6E0EE), 0);
        lv_obj_set_style_text_font(none, &lv_font_montserrat_14, 0);
        // Chip background so the text stays readable over the character.
        lv_obj_set_style_bg_color(none, lv_color_hex(0x07101D), 0);
        lv_obj_set_style_bg_opa(none, 170, 0);
        lv_obj_set_style_radius(none, 9, 0);
        lv_obj_set_style_pad_hor(none, 8, 0);
        lv_obj_set_style_pad_ver(none, 5, 0);
        return;
    }

    const int visibleCount =
        count < MAX_BLE_DETAILS ? count : MAX_BLE_DETAILS;

    lv_label_set_text_fmt(
        bleStatusLabel,
        "%d BLE devices  |  tap one for details",
        visibleCount
    );

    for (int i = 0; i < visibleCount; ++i) {
        const NimBLEAdvertisedDevice *device = results.getDevice(i);
        if (!device) continue;

        BleResultDetail &detail = bleDetails[bleDetailCount];

        const std::string name = device->getName();
        detail.name = name.empty() ? "<unnamed>" : String(name.c_str());
        detail.address = String(device->getAddress().toString().c_str());
        detail.rssi = device->getRSSI();
        detail.connectable = device->isConnectable();
        detail.scannable = device->isScannable();
        detail.addressType = device->getAddressType();
        detail.haveTxPower = device->haveTXPower();
        detail.txPower = detail.haveTxPower ? device->getTXPower() : 0;
        detail.serviceCount = device->getServiceUUIDCount();
        detail.manufacturerCount = device->getManufacturerDataCount();

        String displayName = detail.name;
        if (displayName.length() > 20) {
            displayName = displayName.substring(0, 17) + "...";
        }

        lv_obj_t *row = lv_button_create(bleList);
        lv_obj_set_width(row, 314);
        lv_obj_set_height(row, 66);
        lv_obj_set_style_radius(row, 12, 0);
        lv_obj_set_style_bg_color(row, lv_color_hex(0x07101D), 0);
        lv_obj_set_style_bg_opa(row, 170, 0);
        lv_obj_set_style_border_width(row, 1, 0);
        lv_obj_set_style_border_color(row, lv_color_hex(0x304E82), 0);
        lv_obj_set_style_pad_all(row, 7, 0);
        lv_obj_set_ext_click_area(row, 3);
        lv_obj_add_event_cb(
            row,
            bleDetailEvent,
            LV_EVENT_CLICKED,
            &detail
        );

        lv_obj_t *nameLabel = lv_label_create(row);
        lv_label_set_text(nameLabel, displayName.c_str());
        lv_obj_set_style_text_color(nameLabel, lv_color_hex(0x8EA4FF), 0);
        lv_obj_set_style_text_font(nameLabel, &lv_font_montserrat_16, 0);
        lv_obj_set_pos(nameLabel, 6, 2);

        char info[110];
        snprintf(
            info,
            sizeof(info),
            "%s   %ld dBm   %s  >",
            detail.address.c_str(),
            (long)detail.rssi,
            detail.connectable ? "CONN" : "ADV"
        );

        lv_obj_t *infoLabel = lv_label_create(row);
        lv_label_set_text(infoLabel, info);
        lv_obj_set_style_text_color(infoLabel, lv_color_hex(0xD6E0EE), 0);
        lv_obj_set_style_text_font(infoLabel, &lv_font_montserrat_12, 0);
        lv_obj_set_pos(infoLabel, 6, 35);

        bleDetailCount++;
    }
}

void beginBleScan()
{
    if (!bleStatusLabel || !bleList) return;

    initBleScanner();

    NimBLEScan *scan = NimBLEDevice::getScan();

    if (scan->isScanning()) {
        scan->stop();
    }

    scan->clearResults();
    bleScanFinished = false;
    bleScanEndReason = 0;

    lv_obj_clean(bleList);
    lv_label_set_text(bleStatusLabel, "Scanning BLE advertisements...");

    const bool started = scan->start(5000, false, true);

    if (!started) {
        lv_label_set_text(bleStatusLabel, "BLE scan could not start  |  tap refresh");
    }

    noteActivity();
}

void bleRefreshEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();
    beginBleScan();
}


// ---------------------------------------------------------------------------
// BLE page character animation.
//
// A scaled m33k t3ch sprite loops through idle -> look -> BLE ping -> walk ->
// scan-hit -> glare, drawn behind the (transparent) device list so device rows
// read as chips over him. Serviced by a lightweight lv_timer; torn down in
// clearPageTimers() when the page is left.
// ---------------------------------------------------------------------------
static const lv_image_dsc_t *const BLE_WALK[] = {
    &m_ble_w0, &m_ble_w1, &m_ble_w2, &m_ble_w3, &m_ble_w4, &m_ble_w5
};

struct BleAnimStepDef { const lv_image_dsc_t *img; bool walk; uint32_t dur; bool ping; };
static const BleAnimStepDef BLE_SEQ[] = {
    { &m_ble_idle0, false, 700,  false },
    { &m_ble_look,  false, 500,  false },
    { &m_ble_ping,  false, 1000, true  },
    { nullptr,      true,  1300, false },
    { &m_ble_point, false, 800,  false },
    { &m_ble_glare, false, 750,  false },
};
static const int BLE_SEQ_N = sizeof(BLE_SEQ) / sizeof(BLE_SEQ[0]);

static void blePingExec(void *obj, int32_t v)
{
    lv_obj_t *r = static_cast<lv_obj_t *>(obj);
    const int sz = 26 + (v * 250) / 100;
    lv_obj_set_size(r, sz, sz);
    lv_obj_set_pos(r, blePingCx - sz / 2, blePingCy - sz / 2);
    lv_obj_set_style_radius(r, sz / 2, 0);
    lv_obj_set_style_border_opa(
        r, static_cast<lv_opa_t>(220 * (100 - v) / 100), 0);
}

static void bleFirePing()
{
    if (!blePingRing) return;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, blePingRing);
    lv_anim_set_exec_cb(&a, blePingExec);
    lv_anim_set_values(&a, 0, 100);
    lv_anim_set_time(&a, 1100);
    lv_anim_start(&a);
}

static void bleApplyStep(int idx)
{
    const BleAnimStepDef &s = BLE_SEQ[idx];
    if (s.walk) {
        bleWalkSub = 0;
        bleWalkSubStart = millis();
        if (bleCat) lv_image_set_src(bleCat, BLE_WALK[0]);
    } else if (bleCat) {
        lv_image_set_src(bleCat, s.img);
    }
    if (s.ping) bleFirePing();
}

static void bleAnimTick(lv_timer_t *timer)
{
    LV_UNUSED(timer);
    if (!bleCat || currentPage != Page::BLE) return;

    const uint32_t now = millis();
    const BleAnimStepDef &s = BLE_SEQ[bleAnimStep];

    if (s.walk && (now - bleWalkSubStart >= 130)) {
        bleWalkSubStart = now;
        bleWalkSub = (bleWalkSub + 1) % 6;
        lv_image_set_src(bleCat, BLE_WALK[bleWalkSub]);
    }

    if (now - bleAnimStepStart >= s.dur) {
        bleAnimStep = (bleAnimStep + 1) % BLE_SEQ_N;
        bleAnimStepStart = now;
        bleApplyStep(bleAnimStep);
    }
}

void showBlePage()
{
    clearScreen();
    currentPage = Page::BLE;

    // Dark BLE backdrop (replaces the old bunny background image).
    lv_obj_t *bg = lv_obj_create(screen);
    lv_obj_remove_style_all(bg);
    lv_obj_set_size(bg, 410, 502);
    lv_obj_set_pos(bg, 0, 0);
    lv_obj_set_style_bg_color(bg, lv_color_hex(0x0B1430), 0);
    lv_obj_set_style_bg_grad_color(bg, lv_color_hex(0x05030C), 0);
    lv_obj_set_style_bg_grad_dir(bg, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(bg, LV_OPA_COVER, 0);

    // Faint scanner range rings centered behind the character.
    for (int ri = 0; ri < 3; ++ri) {
        lv_obj_t *ring = lv_obj_create(screen);
        lv_obj_remove_style_all(ring);
        const int rs = 150 + ri * 90;
        lv_obj_set_size(ring, rs, rs);
        lv_obj_set_pos(ring, 205 - rs / 2, 300 - rs / 2);
        lv_obj_set_style_radius(ring, rs / 2, 0);
        lv_obj_set_style_bg_opa(ring, 0, 0);
        lv_obj_set_style_border_width(ring, 1, 0);
        lv_obj_set_style_border_color(ring, lv_color_hex(0x1C3A6B), 0);
        lv_obj_clear_flag(ring, LV_OBJ_FLAG_CLICKABLE);
    }

    lv_obj_t *shade = lv_obj_create(screen);
    lv_obj_remove_style_all(shade);
    lv_obj_set_size(shade, 410, 502);
    lv_obj_set_pos(shade, 0, 0);
    lv_obj_set_style_bg_color(shade, lv_color_hex(0x02050B), 0);
    lv_obj_set_style_bg_opa(shade, 40, 0);

    // Expanding BLE "ping" ring, animated on the ping step (behind the cat).
    blePingCx = 205;
    blePingCy = 300;
    blePingRing = lv_obj_create(screen);
    lv_obj_remove_style_all(blePingRing);
    lv_obj_set_size(blePingRing, 26, 26);
    lv_obj_set_pos(blePingRing, blePingCx - 13, blePingCy - 13);
    lv_obj_set_style_radius(blePingRing, 13, 0);
    lv_obj_set_style_bg_opa(blePingRing, 0, 0);
    lv_obj_set_style_border_width(blePingRing, 3, 0);
    lv_obj_set_style_border_color(blePingRing, lv_color_hex(0x37E3FF), 0);
    lv_obj_set_style_border_opa(blePingRing, 0, 0);
    lv_obj_clear_flag(blePingRing, LV_OBJ_FLAG_CLICKABLE);

    // Animated character.
    bleCat = lv_image_create(screen);
    lv_image_set_src(bleCat, &m_ble_idle0);
    lv_obj_align(bleCat, LV_ALIGN_CENTER, 0, 58);
    lv_obj_clear_flag(bleCat, LV_OBJ_FLAG_CLICKABLE);

    bleAnimStep = 0;
    bleAnimStepStart = millis();
    bleWalkSub = 0;
    bleWalkSubStart = millis();
    bleAnimTimer = lv_timer_create(bleAnimTick, 60, nullptr);

    createPageBatteryIndicator();

    // Safe inset controls for rounded corners.
    lv_obj_t *back = lv_button_create(screen);
    lv_obj_set_size(back, 44, 40);
    lv_obj_set_pos(back, 54, 28);
    lv_obj_set_style_radius(back, 14, 0);
    lv_obj_set_style_bg_color(back, lv_color_hex(0x101A2A), 0);
    lv_obj_set_style_border_width(back, 1, 0);
    lv_obj_set_style_border_color(back, lv_color_hex(0xFF4AD5), 0);
    lv_obj_set_ext_click_area(back, 6);
    lv_obj_add_event_cb(back, backEvent, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *backIcon = lv_label_create(back);
    lv_label_set_text(backIcon, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(backIcon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(backIcon);

    lv_obj_t *titleCard = lv_obj_create(screen);
    lv_obj_set_size(titleCard, 190, 40);
    lv_obj_set_pos(titleCard, 110, 28);
    lv_obj_set_style_radius(titleCard, 16, 0);
    lv_obj_set_style_bg_color(titleCard, lv_color_hex(0x07101D), 0);
    lv_obj_set_style_bg_opa(titleCard, 215, 0);
    lv_obj_set_style_border_width(titleCard, 1, 0);
    lv_obj_set_style_border_color(titleCard, lv_color_hex(0x304E82), 0);

    lv_obj_t *title = lv_label_create(titleCard);
    lv_label_set_text(title, LV_SYMBOL_BLUETOOTH "  BLE SCAN");
    lv_obj_set_style_text_color(title, lv_color_hex(0x8EA4FF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_18, 0);
    lv_obj_center(title);

    lv_obj_t *refresh = lv_button_create(screen);
    lv_obj_set_size(refresh, 44, 40);
    lv_obj_set_pos(refresh, 312, 28);
    lv_obj_set_style_radius(refresh, 14, 0);
    lv_obj_set_style_bg_color(refresh, lv_color_hex(0x101A2A), 0);
    lv_obj_set_style_border_width(refresh, 1, 0);
    lv_obj_set_style_border_color(refresh, lv_color_hex(0x6D86FF), 0);
    lv_obj_set_ext_click_area(refresh, 6);
    lv_obj_add_event_cb(refresh, bleRefreshEvent, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *refreshIcon = lv_label_create(refresh);
    lv_label_set_text(refreshIcon, LV_SYMBOL_REFRESH);
    lv_obj_set_style_text_color(refreshIcon, lv_color_hex(0x8EA4FF), 0);
    lv_obj_center(refreshIcon);

    bleStatusLabel = lv_label_create(screen);
    lv_label_set_text(bleStatusLabel, "Starting BLE scan...");
    lv_obj_set_width(bleStatusLabel, 300);
    lv_obj_set_style_text_color(bleStatusLabel, lv_color_hex(0xB6FF00), 0);
    lv_obj_set_style_text_font(bleStatusLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(bleStatusLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_bg_color(bleStatusLabel, lv_color_hex(0x02060A), 0);
    lv_obj_set_style_bg_opa(bleStatusLabel, 190, 0);
    lv_obj_set_style_radius(bleStatusLabel, 9, 0);
    lv_obj_set_style_pad_hor(bleStatusLabel, 6, 0);
    lv_obj_set_style_pad_ver(bleStatusLabel, 4, 0);
    lv_obj_align(bleStatusLabel, LV_ALIGN_TOP_MID, 0, 79);

    bleList = lv_obj_create(screen);
    lv_obj_set_size(bleList, 350, 350);
    lv_obj_set_pos(bleList, 30, 116);
    // Transparent container so the animated character shows through the gaps;
    // each device row carries its own chip background for readability.
    lv_obj_set_style_bg_opa(bleList, 0, 0);
    lv_obj_set_style_border_width(bleList, 0, 0);
    lv_obj_set_style_radius(bleList, 16, 0);
    lv_obj_set_style_pad_all(bleList, 8, 0);
    lv_obj_set_style_pad_row(bleList, 7, 0);
    lv_obj_set_flex_flow(bleList, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(bleList, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(bleList, LV_SCROLLBAR_MODE_AUTO);

    bleDetailOverlay = lv_obj_create(screen);
    lv_obj_set_size(bleDetailOverlay, 330, 292);
    lv_obj_set_pos(bleDetailOverlay, 40, 118);
    lv_obj_set_style_radius(bleDetailOverlay, 20, 0);
    lv_obj_set_style_bg_color(bleDetailOverlay, lv_color_hex(0x050A12), 0);
    lv_obj_set_style_bg_opa(bleDetailOverlay, 246, 0);
    lv_obj_set_style_border_width(bleDetailOverlay, 2, 0);
    lv_obj_set_style_border_color(bleDetailOverlay, lv_color_hex(0x6D86FF), 0);
    lv_obj_add_flag(bleDetailOverlay, LV_OBJ_FLAG_HIDDEN);

    bleDetailTitle = lv_label_create(bleDetailOverlay);
    lv_label_set_text(bleDetailTitle, "BLE DEVICE");
    lv_obj_set_width(bleDetailTitle, 290);
    lv_obj_set_style_text_color(bleDetailTitle, lv_color_hex(0x8EA4FF), 0);
    lv_obj_set_style_text_font(bleDetailTitle, &lv_font_montserrat_18, 0);
    lv_obj_set_pos(bleDetailTitle, 14, 12);

    bleDetailBody = lv_label_create(bleDetailOverlay);
    lv_label_set_text(bleDetailBody, "");
    lv_obj_set_width(bleDetailBody, 292);
    lv_obj_set_style_text_color(bleDetailBody, lv_color_hex(0xE6EDF7), 0);
    lv_obj_set_style_text_font(bleDetailBody, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(bleDetailBody, 14, 48);

    lv_obj_t *closeBtn = lv_button_create(bleDetailOverlay);
    lv_obj_set_size(closeBtn, 64, 38);
    lv_obj_align(closeBtn, LV_ALIGN_BOTTOM_MID, 0, -12);
    lv_obj_set_style_radius(closeBtn, 13, 0);
    lv_obj_set_style_bg_color(closeBtn, lv_color_hex(0x24101E), 0);
    lv_obj_set_style_border_width(closeBtn, 1, 0);
    lv_obj_set_style_border_color(closeBtn, lv_color_hex(0xFF52C8), 0);
    lv_obj_add_event_cb(closeBtn, bleDetailCloseEvent, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *closeText = lv_label_create(closeBtn);
    lv_label_set_text(closeText, "CLOSE");
    lv_obj_set_style_text_color(closeText, lv_color_hex(0xFF8AD9), 0);
    lv_obj_center(closeText);

    beginBleScan();
    noteActivity();
}


lv_obj_t *createReconTile(
    int x,
    int y,
    int w,
    int h,
    uint32_t borderColor,
    lv_obj_t **labelOut,
    const char *text,
    uint32_t textColor,
    uint8_t bgOpacity)
{
    lv_obj_t *card = lv_obj_create(screen);
    lv_obj_set_size(card, w, h);
    lv_obj_set_pos(card, x, y);
    lv_obj_set_style_radius(card, 17, 0);
    lv_obj_set_style_bg_color(
        card,
        lv_color_hex(0x06101A),
        0
    );
    lv_obj_set_style_bg_opa(card, bgOpacity, 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_border_color(
        card,
        lv_color_hex(borderColor),
        0
    );
    lv_obj_set_style_pad_all(card, 0, 0);

    lv_obj_t *label = lv_label_create(card);
    lv_label_set_text(label, text);
    lv_obj_set_width(label, w - 16);
    lv_obj_set_style_text_align(
        label,
        LV_TEXT_ALIGN_CENTER,
        0
    );
    lv_obj_set_style_text_color(
        label,
        lv_color_hex(textColor),
        0
    );
    lv_obj_set_style_text_font(
        label,
        &lv_font_montserrat_14,
        0
    );
    lv_obj_center(label);

    if (labelOut) {
        *labelOut = label;
    }

    return card;
}

void showReconPage()
{
    clearScreen();
    currentPage = Page::Recon;
    reconExitHomePending = false;

    // Treat Recon art like the BLE page background: large, faded, and behind
    // every interactive control so cards/buttons remain readable on top.
    lv_obj_t *reconArt = lv_image_create(screen);
    lv_image_set_src(reconArt, &recon_dashboard_art_img);
    // 256 = 100%; keep the native 302x245 source sharp while filling most
    // of the dashboard. LVGL scales around the image center, so this position
    // centers the rendered art across the screen and behind both tiles/tools.
    lv_image_set_scale(reconArt, 326);
    lv_obj_set_pos(reconArt, 54, 139);
    lv_obj_clear_flag(reconArt, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *reconShade = lv_obj_create(screen);
    lv_obj_remove_style_all(reconShade);
    lv_obj_set_size(reconShade, SCREEN_W, SCREEN_H);
    lv_obj_set_pos(reconShade, 0, 0);
    lv_obj_set_style_bg_color(reconShade, lv_color_hex(0x02050B), 0);
    // Match the BLE scanner's light veil so the art remains visible.
    lv_obj_set_style_bg_opa(reconShade, 55, 0);
    lv_obj_clear_flag(reconShade, LV_OBJ_FLAG_CLICKABLE);

    createSafeHeaderBack(backEvent);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "RECON DASHBOARD");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFF52C8), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_width(title, 216);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(title, 97, 27);

    lv_obj_t *refresh = lv_button_create(screen);
    lv_obj_set_size(refresh, 46, 42);
    lv_obj_set_pos(refresh, 326, 17);
    lv_obj_set_style_radius(refresh, 14, 0);
    lv_obj_set_style_bg_color(refresh, lv_color_hex(0x101A2A), 0);
    lv_obj_set_style_border_width(refresh, 1, 0);
    lv_obj_set_style_border_color(refresh, lv_color_hex(0xFF52C8), 0);
    lv_obj_set_ext_click_area(refresh, 6);
    lv_obj_add_event_cb(refresh, reconRefreshEvent, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *refreshIcon = lv_label_create(refresh);
    lv_label_set_text(refreshIcon, LV_SYMBOL_REFRESH);
    lv_obj_set_style_text_color(refreshIcon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(refreshIcon);

    reconStatusLabel = lv_label_create(screen);
    lv_label_set_text(reconStatusLabel, "STARTING RECON...");
    lv_obj_set_width(reconStatusLabel, 276);
    lv_obj_set_style_text_align(reconStatusLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(reconStatusLabel, lv_color_hex(0xB6FF00), 0);
    lv_obj_set_style_text_font(reconStatusLabel, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(reconStatusLabel, 18, 82);

    // Compact summaries. Strongest Wi-Fi/BLE stay useful, but are folded
    // into their count tiles instead of consuming separate dashboard boxes.
    createReconTile(
        20, 116, 132, 78,
        0x2F6BFF,
        &reconWifiCountLabel,
        "WI-FI\n0 APs\n2.4:0  5G:0",
        0x55EEFF,
        112
    );

    createReconTile(
        162, 116, 132, 78,
        0x2F6BFF,
        &reconChannelLabel,
        "BUSIEST\nCH --",
        0x55EEFF,
        112
    );

    createReconTile(
        20, 204, 132, 78,
        0x6D86FF,
        &reconBleCountLabel,
        "BLE\n0 DEV\nBEST --",
        0x8EA4FF,
        112
    );

    createReconTile(
        162, 204, 132, 78,
        0xF5FF3B,
        &reconGpsLabel,
        "GPS SEARCH\nSATS 0",
        0xF5FF3B,
        112
    );

    reconWifiStrongLabel = nullptr;
    reconBleStrongLabel = nullptr;
    reconSatLabel = nullptr;

    lv_obj_t *toolsLabel = lv_label_create(screen);
    lv_label_set_text(toolsLabel, "TOOLS");
    lv_obj_set_width(toolsLabel, 82);
    lv_obj_set_style_text_align(toolsLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(toolsLabel, lv_color_hex(0x9BAFC4), 0);
    lv_obj_set_style_text_font(toolsLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(toolsLabel, 310, 91);

    auto makeToolButton = [](int y, const char *labelText,
                             uint32_t borderColor,
                             lv_event_cb_t callback) {
        lv_obj_t *button = lv_button_create(screen);
        lv_obj_set_size(button, 78, 42);
        lv_obj_set_pos(button, 312, y);
        lv_obj_set_style_radius(button, 14, 0);
        lv_obj_set_style_bg_color(button, lv_color_hex(0x08111C), 0);
        lv_obj_set_style_bg_opa(button, 150, 0);
        lv_obj_set_style_border_width(button, 1, 0);
        lv_obj_set_style_border_color(button, lv_color_hex(borderColor), 0);
        lv_obj_set_ext_click_area(button, 4);
        lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, nullptr);

        lv_obj_t *label = lv_label_create(button);
        lv_label_set_text(label, labelText);
        lv_obj_set_style_text_color(label, lv_color_hex(borderColor), 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
        lv_obj_center(label);
    };

    makeToolButton(116, "PULSE", 0x7CFF45, reconPulseEvent);
    makeToolButton(166, "RADAR", 0x55EEFF, reconRadarEvent);
    makeToolButton(216, "HUNT",  0xFF52C8, reconHunterEvent);
    makeToolButton(266, "WATCH", 0xFFB84D, reconWatchEvent);
    makeToolButton(316, "DRIVE", 0x7CFF45, reconWardriveEvent);
    makeToolButton(366, "RADIO", 0xB37BFF, reconRadioEvent);

    gpsToolActive = true;
    gpsPageOpenedMs = millis();

    reconTimer = lv_timer_create(reconTimerCb, 300, nullptr);
    beginReconScan();
    noteActivity();
}

void showWatchModePage()
{
    clearScreen();
    currentPage = Page::WatchMode;
    const bool waitingForC5 = startToolC5Discovery();
    createSafeHeaderBack(reconSubBackEvent);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "WATCH MODE");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFB84D), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_width(title, 214);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(title, 98, 27);

    lv_obj_t *refresh = lv_button_create(screen);
    lv_obj_set_size(refresh, 46, 42);
    lv_obj_set_pos(refresh, 326, 17);
    lv_obj_set_style_radius(refresh, 14, 0);
    lv_obj_set_style_bg_color(refresh, lv_color_hex(0x101A2A), 0);
    lv_obj_set_style_border_width(refresh, 1, 0);
    lv_obj_set_style_border_color(refresh, lv_color_hex(0xFFB84D), 0);
    lv_obj_add_event_cb(refresh, watchRefreshEvent, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *refreshIcon = lv_label_create(refresh);
    lv_label_set_text(refreshIcon, LV_SYMBOL_REFRESH);
    lv_obj_set_style_text_color(refreshIcon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(refreshIcon);

    watchStatusLabel = lv_label_create(screen);
    lv_label_set_text(watchStatusLabel, "CAPTURING BASELINE...");
    lv_obj_set_width(watchStatusLabel, 330);
    lv_obj_set_style_text_align(watchStatusLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(watchStatusLabel, lv_color_hex(0xB6FF00), 0);
    lv_obj_set_style_text_font(watchStatusLabel, &lv_font_montserrat_14, 0);
    lv_obj_align(watchStatusLabel, LV_ALIGN_TOP_MID, 0, 82);

    createReconTile(30, 116, 108, 78, 0x2F6BFF, &watchWifiLabel, "WI-FI\n0", 0x55EEFF);
    createReconTile(151, 116, 108, 78, 0x6D86FF, &watchBleLabel, "BLE\n0", 0x8EA4FF);
    createReconTile(272, 116, 108, 78, 0xFFB84D, &watchAlertCountLabel, "ALERTS\n0", 0xFFB84D);

    lv_obj_t *alertCard = lv_obj_create(screen);
    lv_obj_set_size(alertCard, 350, 268);
    lv_obj_set_pos(alertCard, 30, 204);
    lv_obj_set_style_radius(alertCard, 18, 0);
    lv_obj_set_style_bg_color(alertCard, lv_color_hex(0x07101D), 0);
    lv_obj_set_style_bg_opa(alertCard, 235, 0);
    lv_obj_set_style_border_width(alertCard, 1, 0);
    lv_obj_set_style_border_color(alertCard, lv_color_hex(0xFFB84D), 0);
    lv_obj_set_style_pad_all(alertCard, 8, 0);

    lv_obj_t *alertsTitle = lv_label_create(alertCard);
    lv_label_set_text(alertsTitle, "FACTUAL CHANGE ALERTS  |  BEEP");
    lv_obj_set_style_text_color(alertsTitle, lv_color_hex(0xFFB84D), 0);
    lv_obj_set_style_text_font(alertsTitle, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(alertsTitle, 8, 6);

    watchAlertList = lv_obj_create(alertCard);
    lv_obj_set_size(watchAlertList, 314, 214);
    lv_obj_set_pos(watchAlertList, 8, 34);
    lv_obj_set_style_radius(watchAlertList, 14, 0);
    lv_obj_set_style_bg_color(watchAlertList, lv_color_hex(0x030711), 0);
    lv_obj_set_style_bg_opa(watchAlertList, 255, 0);
    lv_obj_set_style_border_width(watchAlertList, 1, 0);
    lv_obj_set_style_border_color(watchAlertList, lv_color_hex(0x34465A), 0);
    lv_obj_set_style_pad_all(watchAlertList, 6, 0);
    lv_obj_set_style_pad_row(watchAlertList, 6, 0);
    lv_obj_set_flex_flow(watchAlertList, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(watchAlertList, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(watchAlertList, LV_SCROLLBAR_MODE_AUTO);

    watchAlertTotal = 0;
    watchCurrentWifiCount = 0;
    watchCurrentWifi24Count = 0;
    watchCurrentWifi5Count = 0;
    watchCurrentBleCount = 0;
    watchPrevWifiCount = 0;
    watchPrevBleCount = 0;
    watchWifiBaselineReady = false;
    watchBleBaselineReady = false;
    watchAlertsArmed = false;
    watchLastBeepMs = 0;
    watchNextSweepMs = 0;
    updateWatchSummaryLabels();

    pushWatchAlert("Baseline mode: first sweep learns the area.", 0xF4F6FF, false);
    pushWatchAlert("After baseline, real changes trigger a speaker beep.", 0xF4F6FF, false);
    watchAlertTotal = 0;
    updateWatchSummaryLabels();

    watchTimer = lv_timer_create(watchTimerCb, 120, nullptr);
    if (!waitingForC5) {
        beginWatchSweep();
    } else if (watchStatusLabel) {
        lv_label_set_text(watchStatusLabel, "CONNECTING C5...");
    }
    noteActivity();
}


void showWardrivePage()
{
    clearScreen();
    currentPage = Page::Wardrive;
    wardriveExitToReconPending = false;
    wardriveOpenFromReconPending = false;
    wardrivePageActive = true;
    // Wardrive must not launch C5 BLE discovery on page entry.
    // An already-linked C5 can still be used by the Wardrive scan path.
    createSafeHeaderBack(reconSubBackEvent);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "WARDRIVE");
    lv_obj_set_style_text_color(title, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_width(title, 214);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(title, 98, 27);

    wardriveStatusLabel = lv_label_create(screen);
    lv_label_set_text(wardriveStatusLabel, "READY  |  TAP START");
    lv_obj_set_width(wardriveStatusLabel, 340);
    lv_obj_set_style_text_align(wardriveStatusLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(wardriveStatusLabel, lv_color_hex(0xB6FF00), 0);
    lv_obj_set_style_text_font(wardriveStatusLabel, &lv_font_montserrat_14, 0);
    lv_obj_align(wardriveStatusLabel, LV_ALIGN_TOP_MID, 0, 78);

    createReconTile(30, 108, 108, 58, 0x2F6BFF, &wardriveWifiLabel, "WI-FI\n0", 0x55EEFF);
    createReconTile(151, 108, 108, 58, 0x6D86FF, &wardriveBleLabel, "BLE\n0", 0x8EA4FF);
    createReconTile(272, 108, 108, 58, 0xF5FF3B, &wardriveGpsLabel, "GPS\nSEARCHING", 0xF5FF3B);

    lv_obj_t *sessionCard = lv_obj_create(screen);
    // Extended vertically now that the large bottom START/STOP
    // button has been moved to the right side.
    lv_obj_set_size(sessionCard, 350, 210);
    lv_obj_set_pos(sessionCard, 30, 174);
    lv_obj_set_style_radius(sessionCard, 18, 0);
    lv_obj_set_style_bg_color(sessionCard, lv_color_hex(0x07101D), 0);
    lv_obj_set_style_bg_opa(sessionCard, 235, 0);
    lv_obj_set_style_border_width(sessionCard, 1, 0);
    lv_obj_set_style_border_color(sessionCard, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_pad_all(sessionCard, 0, 0);
    lv_obj_clear_flag(sessionCard, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(sessionCard, LV_OBJ_FLAG_SCROLL_CHAIN_VER);
    lv_obj_clear_flag(sessionCard, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);

    // Live session timer intentionally omitted.
    // Internal elapsed-time tracking remains available if needed later.
    wardriveElapsedLabel = nullptr;

    wardriveFeedStateLabel = lv_label_create(sessionCard);
    lv_label_set_text(wardriveFeedStateLabel, "SCAN FEED");
    lv_obj_set_width(wardriveFeedStateLabel, 160);
    lv_obj_set_style_text_color(wardriveFeedStateLabel, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_text_font(wardriveFeedStateLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(wardriveFeedStateLabel, 12, 32);

    // Fixed, non-scrollable feed. Six labels stay at permanent coordinates;
    // page buttons only replace their text. This avoids LVGL scroll/reflow
    // activity while Wi-Fi/BLE/C5 scan callbacks are updating the session.
    wardriveFeedScroll = lv_obj_create(sessionCard);
    // Slightly taller Wardrive feed without creating any
    // additional LVGL objects.
    lv_obj_set_size(wardriveFeedScroll, 228, 158);
    lv_obj_set_pos(wardriveFeedScroll, 10, 50);
    lv_obj_set_style_radius(wardriveFeedScroll, 12, 0);
    lv_obj_set_style_bg_color(wardriveFeedScroll, lv_color_hex(0x030812), 0);
    lv_obj_set_style_bg_opa(wardriveFeedScroll, 175, 0);
    lv_obj_set_style_border_width(wardriveFeedScroll, 1, 0);
    lv_obj_set_style_border_color(wardriveFeedScroll, lv_color_hex(0x123252), 0);
    lv_obj_set_style_pad_all(wardriveFeedScroll, 0, 0);
    lv_obj_clear_flag(wardriveFeedScroll, LV_OBJ_FLAG_SCROLLABLE);

    for (uint8_t i = 0; i < WARD_FEED_PAGE_ROWS; ++i) {
        wardriveFeedRows[i] = lv_label_create(wardriveFeedScroll);
        lv_label_set_text(wardriveFeedRows[i], "");
        lv_obj_set_width(wardriveFeedRows[i], 158);
        lv_obj_set_height(wardriveFeedRows[i], 12);
        lv_label_set_long_mode(wardriveFeedRows[i], LV_LABEL_LONG_CLIP);
        lv_obj_clear_flag(wardriveFeedRows[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_text_color(
            wardriveFeedRows[i],
            lv_color_hex(0xF4F6FF),
            0
        );
        lv_obj_set_style_text_font(
            wardriveFeedRows[i],
            &lv_font_montserrat_10,
            0
        );
        lv_obj_set_pos(
            wardriveFeedRows[i],
            6,
            7 + i * 22
        );
    }

    // --------------------------------------------------------
    // Bunny-skull feed position indicator.
    //
    // Five tiny outlined bunny heads. Cyan = empty,
    // neon pink = current feed progress.
    //
    // Kept intentionally lightweight: 15 tiny LVGL primitives
    // total and no additional timers/buffers.
    // --------------------------------------------------------
    for (uint8_t i = 0;
         i < WARD_BUNNY_PROGRESS_COUNT;
         ++i) {

        const int y =
            40 + static_cast<int>(i) * 16;

        // Head
        wardriveBunnyHeads[i] =
            lv_obj_create(wardriveFeedScroll);

        lv_obj_set_size(
            wardriveBunnyHeads[i],
            11,
            9
        );

        lv_obj_set_pos(
            wardriveBunnyHeads[i],
            200,
            y + 5
        );

        lv_obj_set_style_radius(
            wardriveBunnyHeads[i],
            4,
            0
        );

        lv_obj_set_style_border_width(
            wardriveBunnyHeads[i],
            1,
            0
        );

        lv_obj_set_style_border_color(
            wardriveBunnyHeads[i],
            lv_color_hex(0x55EEFF),
            0
        );

        lv_obj_set_style_bg_opa(
            wardriveBunnyHeads[i],
            LV_OPA_TRANSP,
            0
        );

        lv_obj_set_style_pad_all(
            wardriveBunnyHeads[i],
            0,
            0
        );

        lv_obj_clear_flag(
            wardriveBunnyHeads[i],
            LV_OBJ_FLAG_SCROLLABLE
        );

        // Left ear
        wardriveBunnyEarL[i] =
            lv_obj_create(wardriveFeedScroll);

        lv_obj_set_size(
            wardriveBunnyEarL[i],
            3,
            7
        );

        lv_obj_set_pos(
            wardriveBunnyEarL[i],
            202,
            y
        );

        lv_obj_set_style_radius(
            wardriveBunnyEarL[i],
            2,
            0
        );

        lv_obj_set_style_border_width(
            wardriveBunnyEarL[i],
            1,
            0
        );

        lv_obj_set_style_border_color(
            wardriveBunnyEarL[i],
            lv_color_hex(0x55EEFF),
            0
        );

        lv_obj_set_style_bg_opa(
            wardriveBunnyEarL[i],
            LV_OPA_TRANSP,
            0
        );

        lv_obj_set_style_pad_all(
            wardriveBunnyEarL[i],
            0,
            0
        );

        lv_obj_clear_flag(
            wardriveBunnyEarL[i],
            LV_OBJ_FLAG_SCROLLABLE
        );

        // Right ear
        wardriveBunnyEarR[i] =
            lv_obj_create(wardriveFeedScroll);

        lv_obj_set_size(
            wardriveBunnyEarR[i],
            3,
            7
        );

        lv_obj_set_pos(
            wardriveBunnyEarR[i],
            207,
            y
        );

        lv_obj_set_style_radius(
            wardriveBunnyEarR[i],
            2,
            0
        );

        lv_obj_set_style_border_width(
            wardriveBunnyEarR[i],
            1,
            0
        );

        lv_obj_set_style_border_color(
            wardriveBunnyEarR[i],
            lv_color_hex(0x55EEFF),
            0
        );

        lv_obj_set_style_bg_opa(
            wardriveBunnyEarR[i],
            LV_OPA_TRANSP,
            0
        );

        lv_obj_set_style_pad_all(
            wardriveBunnyEarR[i],
            0,
            0
        );

        lv_obj_clear_flag(
            wardriveBunnyEarR[i],
            LV_OBJ_FLAG_SCROLLABLE
        );
    }

    lv_obj_t *feedUp = lv_button_create(wardriveFeedScroll);
    lv_obj_set_size(feedUp, 34, 32);
    lv_obj_set_pos(feedUp, 188, 5);
    lv_obj_set_style_radius(feedUp, 9, 0);
    lv_obj_set_style_bg_color(feedUp, lv_color_hex(0x101A2A), 0);
    lv_obj_set_style_border_width(feedUp, 1, 0);
    lv_obj_set_style_border_color(feedUp, lv_color_hex(0x55EEFF), 0);
    lv_obj_add_event_cb(
        feedUp,
        wardriveFeedPageEvent,
        LV_EVENT_CLICKED,
        reinterpret_cast<void *>(static_cast<intptr_t>(-1))
    );
    lv_obj_t *feedUpText = lv_label_create(feedUp);
    lv_label_set_text(feedUpText, LV_SYMBOL_UP);
    lv_obj_set_style_text_color(feedUpText, lv_color_hex(0x55EEFF), 0);
    lv_obj_center(feedUpText);

    lv_obj_t *feedDown = lv_button_create(wardriveFeedScroll);
    lv_obj_set_size(feedDown, 34, 32);
    lv_obj_set_pos(feedDown, 188, 120);
    lv_obj_set_style_radius(feedDown, 9, 0);
    lv_obj_set_style_bg_color(feedDown, lv_color_hex(0x101A2A), 0);
    lv_obj_set_style_border_width(feedDown, 1, 0);
    lv_obj_set_style_border_color(feedDown, lv_color_hex(0xFF52C8), 0);
    lv_obj_add_event_cb(
        feedDown,
        wardriveFeedPageEvent,
        LV_EVENT_CLICKED,
        reinterpret_cast<void *>(static_cast<intptr_t>(1))
    );
    lv_obj_t *feedDownText = lv_label_create(feedDown);
    lv_label_set_text(feedDownText, LV_SYMBOL_DOWN);
    lv_obj_set_style_text_color(feedDownText, lv_color_hex(0xFF52C8), 0);
    lv_obj_center(feedDownText);

    wardriveFeedPageLabel = lv_label_create(wardriveFeedScroll);

    // The feed is a rolling live window, not a session-total counter.
    // Hide this legacy page-position label so "/120" cannot be confused
    // with the total number of discovered devices.
    lv_label_set_text(wardriveFeedPageLabel, "");
    lv_obj_add_flag(wardriveFeedPageLabel, LV_OBJ_FLAG_HIDDEN);

    lv_obj_set_width(wardriveFeedPageLabel, 44);
    lv_obj_set_style_text_align(
        wardriveFeedPageLabel,
        LV_TEXT_ALIGN_CENTER,
        0
    );
    lv_obj_set_style_text_color(
        wardriveFeedPageLabel,
        lv_color_hex(0xAFC2D8),
        0
    );
    lv_obj_set_style_text_font(
        wardriveFeedPageLabel,
        &lv_font_montserrat_10,
        0
    );
    lv_obj_set_pos(wardriveFeedPageLabel, 182, 79);

    // Legacy single-label pointer intentionally unused in fixed-page mode.
    wardriveFeedLabel = nullptr;

    wardriveLocationLabel = lv_label_create(sessionCard);
    lv_label_set_text(wardriveLocationLabel, "GPS SEARCHING\n--.----\n--.----");
    lv_obj_set_width(wardriveLocationLabel, 92);
    lv_obj_set_style_text_align(wardriveLocationLabel, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_style_text_color(wardriveLocationLabel, lv_color_hex(0x7CFF45), 0);
    lv_obj_set_style_text_font(wardriveLocationLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(wardriveLocationLabel, 248, 50);

    wardriveSdLabel = lv_label_create(sessionCard);
    lv_label_set_text(wardriveSdLabel, "SD CHECK");
    lv_obj_set_width(wardriveSdLabel, 92);
    lv_obj_set_style_text_align(wardriveSdLabel, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_style_text_color(wardriveSdLabel, lv_color_hex(0xF5FF3B), 0);
    lv_obj_set_style_text_font(wardriveSdLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(wardriveSdLabel, 248, 118);

    lv_obj_t *startBtn = lv_button_create(screen);

    // Compact Wardrive START/STOP control.
    // Positioned beneath the SD-status side of the session card,
    // with extra separation from the feed DOWN button.
    lv_obj_set_size(startBtn, 86, 34);
    lv_obj_set_pos(startBtn, 288, 342);
    lv_obj_set_style_radius(startBtn, 12, 0);
    lv_obj_set_style_bg_color(startBtn, lv_color_hex(0x0B1A1C), 0);
    lv_obj_set_style_border_width(startBtn, 1, 0);
    lv_obj_set_style_border_color(startBtn, lv_color_hex(0x7CFF45), 0);
    lv_obj_add_event_cb(startBtn, wardriveStartEvent, LV_EVENT_CLICKED, nullptr);

    wardriveStartLabel = lv_label_create(startBtn);
    lv_label_set_text(wardriveStartLabel, "START");
    lv_obj_set_style_text_color(wardriveStartLabel, lv_color_hex(0x7CFF45), 0);
    lv_obj_set_style_text_font(
        wardriveStartLabel,
        &lv_font_montserrat_14,
        0
    );
    lv_obj_center(wardriveStartLabel);

    lv_obj_t *bannerFrame = lv_obj_create(screen);
    lv_obj_set_size(bannerFrame, 314, 104);
    lv_obj_set_pos(bannerFrame, 48, 392);
    lv_obj_set_style_radius(bannerFrame, 14, 0);
    lv_obj_set_style_bg_color(bannerFrame, lv_color_hex(0x030711), 0);
    lv_obj_set_style_bg_opa(bannerFrame, 255, 0);
    lv_obj_set_style_border_width(bannerFrame, 1, 0);
    lv_obj_set_style_border_color(bannerFrame, lv_color_hex(0xFF52C8), 0);
    lv_obj_set_style_pad_all(bannerFrame, 0, 0);

    lv_obj_t *bannerImg = lv_image_create(bannerFrame);
    lv_image_set_src(bannerImg, &wardrive_banner_img);
    lv_obj_set_pos(bannerImg, 0, 0);

    wardriveDetailOverlay = lv_obj_create(screen);
    lv_obj_set_size(wardriveDetailOverlay, 340, 320);
    lv_obj_set_pos(wardriveDetailOverlay, 35, 104);
    lv_obj_set_style_radius(wardriveDetailOverlay, 20, 0);
    lv_obj_set_style_bg_color(wardriveDetailOverlay, lv_color_hex(0x050A12), 0);
    lv_obj_set_style_bg_opa(wardriveDetailOverlay, 248, 0);
    lv_obj_set_style_border_width(wardriveDetailOverlay, 2, 0);
    lv_obj_set_style_border_color(wardriveDetailOverlay, lv_color_hex(0x55EEFF), 0);
    lv_obj_add_flag(wardriveDetailOverlay, LV_OBJ_FLAG_HIDDEN);

    wardriveDetailTitle = lv_label_create(wardriveDetailOverlay);
    lv_obj_set_width(wardriveDetailTitle, 270);
    lv_obj_set_style_text_color(wardriveDetailTitle, lv_color_hex(0xFF52C8), 0);
    lv_obj_set_style_text_font(wardriveDetailTitle, &lv_font_montserrat_16, 0);
    lv_obj_set_pos(wardriveDetailTitle, 12, 10);

    wardriveDetailBody = lv_label_create(wardriveDetailOverlay);
    lv_obj_set_width(wardriveDetailBody, 306);
    lv_label_set_long_mode(wardriveDetailBody, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(wardriveDetailBody, lv_color_hex(0xF4F6FF), 0);
    lv_obj_set_style_text_font(wardriveDetailBody, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(wardriveDetailBody, 12, 42);

    lv_obj_t *wardriveDetailClose = lv_button_create(wardriveDetailOverlay);
    lv_obj_set_size(wardriveDetailClose, 44, 36);
    lv_obj_set_pos(wardriveDetailClose, 278, 8);
    lv_obj_set_style_radius(wardriveDetailClose, 12, 0);
    lv_obj_set_style_bg_color(wardriveDetailClose, lv_color_hex(0x21101B), 0);
    lv_obj_set_style_border_width(wardriveDetailClose, 1, 0);
    lv_obj_set_style_border_color(wardriveDetailClose, lv_color_hex(0xFF52C8), 0);
    lv_obj_add_event_cb(wardriveDetailClose, wardriveDetailCloseEvent, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *wardriveDetailCloseText = lv_label_create(wardriveDetailClose);
    lv_label_set_text(wardriveDetailCloseText, "X");
    lv_obj_set_style_text_color(wardriveDetailCloseText, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(wardriveDetailCloseText);

    wardriveWifiCount = 0;
    wardriveTotalWifiCount = 0;
    wardriveBleCount = 0;
    wardriveDetailOpen = false;
    wardriveUiSyncCursor = 0;
    wardriveLastDiagMs = 0;
    wardriveRecordCount = 0;
    wardriveFeedPageStart = 0;
    wardriveFeedFollowLatest = true;
    m33kSdReady = instance.isCardReady();
    for (auto &rec : wardriveRecords) rec = WardriveRecord{};
    wardriveFeedDirty = true;
    wardriveRunning = false;
    wardrivePhase = WardrivePhase::Stopped;
    wardriveSessionStartedMs = 0;
    wardriveNextSweepMs = 0;
    wardriveLastElapsedSeconds = 0;
    gpsToolActive = true;
    gpsSessionBytes = 0;
    gpsSessionLocationSeen = false;
    gpsPageOpenedMs = millis();
    gpsLastByteMs = gpsPageOpenedMs;

    wardriveTimer = lv_timer_create(wardriveTimerCb, 150, nullptr);
    updateWardriveUi();
    noteActivity();
}

void showRadioPage()
{
    clearScreen();
    currentPage = Page::Radio;
    createSafeHeaderBack(reconSubBackEvent);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "LORA MONITOR");
    lv_obj_set_style_text_color(title, lv_color_hex(0xB37BFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_width(title, 214);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(title, 98, 27);

    radioStatusLabel = lv_label_create(screen);
    lv_label_set_text(radioStatusLabel, "PROBING SX1262...");
    lv_obj_set_width(radioStatusLabel, 350);
    lv_obj_set_style_text_align(radioStatusLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(radioStatusLabel, lv_color_hex(0xF5FF3B), 0);
    lv_obj_set_style_text_font(radioStatusLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(radioStatusLabel, LV_ALIGN_TOP_MID, 0, 78);

    createReconTile(
        30, 108, 108, 60,
        0xB37BFF,
        &radioChipLabel,
        "RADIO\nSX1262",
        0xD9C2FF
    );

    createReconTile(
        151, 108, 108, 60,
        0x55EEFF,
        &radioFreqLabel,
        "FREQ\n915.000",
        0x55EEFF
    );

    createReconTile(
        272, 108, 108, 60,
        0x7CFF45,
        &radioModeLabel,
        "MODE\nRX ONLY",
        0x7CFF45
    );

    lv_obj_t *monitorCard = lv_obj_create(screen);
    lv_obj_set_size(monitorCard, 350, 244);
    lv_obj_set_pos(monitorCard, 30, 178);
    lv_obj_set_style_radius(monitorCard, 18, 0);
    lv_obj_set_style_bg_color(monitorCard, lv_color_hex(0x07101D), 0);
    lv_obj_set_style_bg_opa(monitorCard, 235, 0);
    lv_obj_set_style_border_width(monitorCard, 1, 0);
    lv_obj_set_style_border_color(monitorCard, lv_color_hex(0xB37BFF), 0);
    lv_obj_set_style_pad_all(monitorCard, 0, 0);

    radioPacketLabel = lv_label_create(monitorCard);
    lv_label_set_text(radioPacketLabel, "PACKETS\n0");
    lv_obj_set_width(radioPacketLabel, 92);
    lv_obj_set_style_text_align(radioPacketLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(radioPacketLabel, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_text_font(radioPacketLabel, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(radioPacketLabel, 8, 8);

    radioSignalLabel = lv_label_create(monitorCard);
    lv_label_set_text(radioSignalLabel, "LAST SIGNAL\n-- dBm\nSNR --");
    lv_obj_set_width(radioSignalLabel, 132);
    lv_obj_set_style_text_align(radioSignalLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(radioSignalLabel, lv_color_hex(0xFF52C8), 0);
    lv_obj_set_style_text_font(radioSignalLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(radioSignalLabel, 104, 7);

    radioProfileLabel = lv_label_create(monitorCard);
    lv_label_set_text(radioProfileLabel, "SF 9\nBW 125\nCR 4/7");
    lv_obj_set_width(radioProfileLabel, 98);
    lv_obj_set_style_text_align(radioProfileLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(radioProfileLabel, lv_color_hex(0xDDE7F2), 0);
    lv_obj_set_style_text_font(radioProfileLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(radioProfileLabel, 242, 7);

    radioLastLabel = lv_label_create(monitorCard);
    lv_label_set_text(radioLastLabel, "Waiting for a matching LoRa packet...");
    lv_obj_set_width(radioLastLabel, 326);
    lv_obj_set_style_text_align(radioLastLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(radioLastLabel, lv_color_hex(0xF5FF3B), 0);
    lv_obj_set_style_text_font(radioLastLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(radioLastLabel, 12, 68);

    lv_obj_t *historyBox = lv_obj_create(monitorCard);
    lv_obj_set_size(historyBox, 326, 102);
    lv_obj_set_pos(historyBox, 12, 90);
    lv_obj_set_style_radius(historyBox, 12, 0);
    lv_obj_set_style_bg_color(historyBox, lv_color_hex(0x030711), 0);
    lv_obj_set_style_bg_opa(historyBox, 220, 0);
    lv_obj_set_style_border_width(historyBox, 1, 0);
    lv_obj_set_style_border_color(historyBox, lv_color_hex(0x263A52), 0);
    lv_obj_set_style_pad_all(historyBox, 8, 0);
    lv_obj_set_scroll_dir(historyBox, LV_DIR_VER);
    lv_obj_clear_flag(historyBox, LV_OBJ_FLAG_SCROLL_ELASTIC);
    lv_obj_set_scrollbar_mode(historyBox, LV_SCROLLBAR_MODE_ACTIVE);

    radioPayloadLabel = lv_label_create(historyBox);
    lv_label_set_text(radioPayloadLabel, "No matching packets yet.");
    lv_obj_set_width(radioPayloadLabel, 300);
    lv_label_set_long_mode(radioPayloadLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(radioPayloadLabel, lv_color_hex(0xF4F6FF), 0);
    lv_obj_set_style_text_font(radioPayloadLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(radioPayloadLabel, 0, 0);

    lv_obj_t *sfBtn = lv_button_create(monitorCard);
    lv_obj_set_size(sfBtn, 78, 34);
    lv_obj_set_pos(sfBtn, 8, 200);
    lv_obj_set_style_radius(sfBtn, 12, 0);
    lv_obj_set_style_bg_color(sfBtn, lv_color_hex(0x081826), 0);
    lv_obj_set_style_border_width(sfBtn, 1, 0);
    lv_obj_set_style_border_color(sfBtn, lv_color_hex(0x55EEFF), 0);
    lv_obj_add_event_cb(sfBtn, radioSfEvent, LV_EVENT_CLICKED, nullptr);

    radioSfButtonLabel = lv_label_create(sfBtn);
    lv_label_set_text(radioSfButtonLabel, "SF 9");
    lv_obj_set_style_text_color(radioSfButtonLabel, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_text_font(radioSfButtonLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(radioSfButtonLabel);

    lv_obj_t *bwBtn = lv_button_create(monitorCard);
    lv_obj_set_size(bwBtn, 86, 34);
    lv_obj_set_pos(bwBtn, 90, 200);
    lv_obj_set_style_radius(bwBtn, 12, 0);
    lv_obj_set_style_bg_color(bwBtn, lv_color_hex(0x10091D), 0);
    lv_obj_set_style_border_width(bwBtn, 1, 0);
    lv_obj_set_style_border_color(bwBtn, lv_color_hex(0xFF52C8), 0);
    lv_obj_add_event_cb(bwBtn, radioBwEvent, LV_EVENT_CLICKED, nullptr);

    radioBwButtonLabel = lv_label_create(bwBtn);
    lv_label_set_text(radioBwButtonLabel, "BW 125");
    lv_obj_set_style_text_color(radioBwButtonLabel, lv_color_hex(0xFF52C8), 0);
    lv_obj_set_style_text_font(radioBwButtonLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(radioBwButtonLabel);

    lv_obj_t *autoBtn = lv_button_create(monitorCard);
    lv_obj_set_size(autoBtn, 78, 34);
    lv_obj_set_pos(autoBtn, 180, 200);
    lv_obj_set_style_radius(autoBtn, 12, 0);
    lv_obj_set_style_bg_color(autoBtn, lv_color_hex(0x0B1A1C), 0);
    lv_obj_set_style_border_width(autoBtn, 1, 0);
    lv_obj_set_style_border_color(autoBtn, lv_color_hex(0x7CFF45), 0);
    lv_obj_add_event_cb(autoBtn, radioAutoEvent, LV_EVENT_CLICKED, nullptr);

    radioAutoButtonLabel = lv_label_create(autoBtn);
    lv_label_set_text(radioAutoButtonLabel, "AUTO OFF");
    lv_obj_set_style_text_color(radioAutoButtonLabel, lv_color_hex(0x7CFF45), 0);
    lv_obj_set_style_text_font(radioAutoButtonLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(radioAutoButtonLabel);

    // MeshCore Listen preset toggle (forces 910.525 MHz / SF7 / BW62.5 / CR4:5).
    lv_obj_t *meshBtn = lv_button_create(monitorCard);
    lv_obj_set_size(meshBtn, 80, 34);
    lv_obj_set_pos(meshBtn, 262, 200);
    lv_obj_set_style_radius(meshBtn, 12, 0);
    lv_obj_set_style_bg_color(meshBtn, lv_color_hex(0x1C1408), 0);
    lv_obj_set_style_border_width(meshBtn, 1, 0);
    lv_obj_set_style_border_color(meshBtn, lv_color_hex(0xFFB347), 0);
    lv_obj_add_event_cb(meshBtn, radioMeshEvent, LV_EVENT_CLICKED, nullptr);

    radioMeshButtonLabel = lv_label_create(meshBtn);
    lv_label_set_text(radioMeshButtonLabel, "MESH OFF");
    lv_obj_set_style_text_color(radioMeshButtonLabel, lv_color_hex(0xFFB347), 0);
    lv_obj_set_style_text_font(radioMeshButtonLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(radioMeshButtonLabel);

    lv_obj_t *reprobeBtn = lv_button_create(screen);
    lv_obj_set_size(reprobeBtn, 150, 46);
    lv_obj_set_pos(reprobeBtn, 42, 432);
    lv_obj_set_style_radius(reprobeBtn, 15, 0);
    lv_obj_set_style_bg_color(reprobeBtn, lv_color_hex(0x10091D), 0);
    lv_obj_set_style_border_width(reprobeBtn, 1, 0);
    lv_obj_set_style_border_color(reprobeBtn, lv_color_hex(0xB37BFF), 0);
    lv_obj_add_event_cb(reprobeBtn, radioReprobeEvent, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *reprobeTxt = lv_label_create(reprobeBtn);
    lv_label_set_text(reprobeTxt, "REPROBE");
    lv_obj_set_style_text_color(reprobeTxt, lv_color_hex(0xD9C2FF), 0);
    lv_obj_set_style_text_font(reprobeTxt, &lv_font_montserrat_14, 0);
    lv_obj_center(reprobeTxt);

    lv_obj_t *listenBtn = lv_button_create(screen);
    lv_obj_set_size(listenBtn, 150, 46);
    lv_obj_set_pos(listenBtn, 218, 432);
    lv_obj_set_style_radius(listenBtn, 15, 0);
    lv_obj_set_style_bg_color(listenBtn, lv_color_hex(0x071B13), 0);
    lv_obj_set_style_border_width(listenBtn, 1, 0);
    lv_obj_set_style_border_color(listenBtn, lv_color_hex(0x7CFF45), 0);
    lv_obj_add_event_cb(listenBtn, radioToggleEvent, LV_EVENT_CLICKED, nullptr);

    radioListenLabel = lv_label_create(listenBtn);
    lv_label_set_text(radioListenLabel, "LISTEN");
    lv_obj_set_style_text_color(radioListenLabel, lv_color_hex(0x7CFF45), 0);
    lv_obj_set_style_text_font(radioListenLabel, &lv_font_montserrat_14, 0);
    lv_obj_center(radioListenLabel);

    m33kRadioPacketFlag = false;
    m33kRadioPacketCount = 0;
    m33kRadioLastPacketMs = 0;
    m33kRadioLastRssi = -127.0f;
    m33kRadioLastSnr = 0.0f;
    m33kRadioLastFreqError = 0.0f;
    m33kRadioLastLength = 0;

    m33kRadioSfIndex = 2;
    m33kRadioBwIndex = 0;
    m33kRadioAutoScan = false;
    m33kRadioMeshCore = false;
    m33kRadioNextProfileMs = 0;

    resetRadioHistory();

    m33kRadioInitialized = false;
    m33kRadioListening = false;

    radioTimer = lv_timer_create(radioTimerCb, 120, nullptr);

    updateRadioMonitorUi();
    startRadioMonitor();
    noteActivity();
}

void showRadarPage()
{
    clearScreen();
    currentPage = Page::Radar;
    clearRadarBlips();
    const bool waitingForC5 = startToolC5Discovery();
    radarNextScanMs = 0;

    createSafeHeaderBack(
        reconSubBackEvent
    );

    lv_obj_t *title =
        lv_label_create(screen);

    lv_label_set_text(
        title,
        "RECON RADAR"
    );
    lv_obj_set_style_text_color(
        title,
        lv_color_hex(0x55EEFF),
        0
    );
    lv_obj_set_style_text_font(
        title,
        &lv_font_montserrat_20,
        0
    );
    lv_obj_align(
        title,
        LV_ALIGN_TOP_MID,
        0,
        27
    );

    lv_obj_t *refresh =
        lv_button_create(screen);

    lv_obj_set_size(
        refresh,
        46,
        42
    );
    lv_obj_set_pos(
        refresh,
        326,
        17
    );
    lv_obj_set_style_radius(
        refresh,
        14,
        0
    );
    lv_obj_set_style_bg_color(
        refresh,
        lv_color_hex(0x101A2A),
        0
    );
    lv_obj_set_style_border_width(
        refresh,
        1,
        0
    );
    lv_obj_set_style_border_color(
        refresh,
        lv_color_hex(0x55EEFF),
        0
    );
    lv_obj_add_event_cb(
        refresh,
        radarRefreshEvent,
        LV_EVENT_CLICKED,
        nullptr
    );

    lv_obj_t *refreshIcon =
        lv_label_create(refresh);

    lv_label_set_text(
        refreshIcon,
        LV_SYMBOL_REFRESH
    );
    lv_obj_set_style_text_color(
        refreshIcon,
        lv_color_hex(0xFFFFFF),
        0
    );
    lv_obj_center(refreshIcon);

    radarStatusLabel =
        lv_label_create(screen);

    lv_label_set_text(
        radarStatusLabel,
        "STARTING RADAR..."
    );
    lv_obj_set_width(
        radarStatusLabel,
        330
    );
    lv_obj_set_style_text_align(
        radarStatusLabel,
        LV_TEXT_ALIGN_CENTER,
        0
    );
    lv_obj_set_style_text_color(
        radarStatusLabel,
        lv_color_hex(0xB6FF00),
        0
    );
    lv_obj_set_style_text_font(
        radarStatusLabel,
        &lv_font_montserrat_14,
        0
    );
    lv_obj_align(
        radarStatusLabel,
        LV_ALIGN_TOP_MID,
        0,
        78
    );

    // Radar rings
    const int ringSizes[3] = {
        264, 184, 104
    };

    for (uint8_t i = 0; i < 3; ++i) {
        lv_obj_t *ring =
            lv_obj_create(screen);

        lv_obj_remove_style_all(ring);

        lv_obj_set_size(
            ring,
            ringSizes[i],
            ringSizes[i]
        );

        lv_obj_set_pos(
            ring,
            205 - ringSizes[i] / 2,
            258 - ringSizes[i] / 2
        );

        lv_obj_set_style_radius(
            ring,
            ringSizes[i] / 2,
            0
        );
        lv_obj_set_style_bg_opa(
            ring,
            LV_OPA_TRANSP,
            0
        );
        lv_obj_set_style_border_width(
            ring,
            1,
            0
        );
        lv_obj_set_style_border_color(
            ring,
            lv_color_hex(
                i == 0
                    ? 0x20536B
                    : 0x16384A
            ),
            0
        );
    }

    lv_obj_t *center =
        lv_obj_create(screen);

    lv_obj_remove_style_all(center);
    lv_obj_set_size(center, 14, 14);
    lv_obj_set_pos(center, 198, 251);
    lv_obj_set_style_radius(center, 7, 0);
    lv_obj_set_style_bg_color(
        center,
        lv_color_hex(0xFFFFFF),
        0
    );
    lv_obj_set_style_bg_opa(
        center,
        LV_OPA_COVER,
        0
    );

    // Sweep is drawn using glowing dots so it remains lightweight.
    for (uint8_t i = 0; i < 9; ++i) {
        radarSweepDots[i] =
            lv_obj_create(screen);

        lv_obj_remove_style_all(
            radarSweepDots[i]
        );

        lv_obj_set_size(
            radarSweepDots[i],
            5,
            5
        );

        lv_obj_set_style_radius(
            radarSweepDots[i],
            3,
            0
        );

        lv_obj_set_style_bg_color(
            radarSweepDots[i],
            lv_color_hex(0x55EEFF),
            0
        );

        lv_obj_set_style_bg_opa(
            radarSweepDots[i],
            220 - i * 14,
            0
        );
    }

    lv_obj_t *legend =
        lv_label_create(screen);

    lv_label_set_text(
        legend,
        "CYAN Wi-Fi    PINK BLE"
    );
    lv_obj_set_width(legend, 330);
    lv_obj_set_style_text_align(
        legend,
        LV_TEXT_ALIGN_CENTER,
        0
    );
    lv_obj_set_style_text_color(
        legend,
        lv_color_hex(0xC7D5E6),
        0
    );
    lv_obj_set_style_text_font(
        legend,
        &lv_font_montserrat_12,
        0
    );
    lv_obj_align(
        legend,
        LV_ALIGN_BOTTOM_MID,
        0,
        -72
    );

    radarInfoLabel =
        lv_label_create(screen);

    lv_label_set_text(
        radarInfoLabel,
        "Radius = signal strength\nAngle = visual placement"
    );
    lv_obj_set_width(
        radarInfoLabel,
        340
    );
    lv_obj_set_style_text_align(
        radarInfoLabel,
        LV_TEXT_ALIGN_CENTER,
        0
    );
    lv_obj_set_style_text_color(
        radarInfoLabel,
        lv_color_hex(0xF4F6FF),
        0
    );
    lv_obj_set_style_text_font(
        radarInfoLabel,
        &lv_font_montserrat_12,
        0
    );
    lv_obj_align(
        radarInfoLabel,
        LV_ALIGN_BOTTOM_MID,
        0,
        -28
    );

    radarTimer =
        lv_timer_create(
            radarTimerCb,
            80,
            nullptr
        );

    if (!waitingForC5) {
        beginRadarScan();
    } else if (radarStatusLabel) {
        lv_label_set_text(radarStatusLabel, "CONNECTING C5...");
    }
    noteActivity();
}


void showSignalHunterPage()
{
    clearScreen();
    currentPage =
        Page::SignalHunter;
    startToolC5Discovery();

    createSafeHeaderBack(
        reconSubBackEvent
    );

    // Reopen the speaker I2S in the 48 kHz stereo mode that was verified
    // to produce sound on this exact watch.
    hunterAudioConfigured = false;
    configureHunterAudio();
    prepareHunterBeep();

    hunterKind = HunterKind::None;
    hunterPhase = HunterPhase::Idle;
    hunterFeedbackMode =
        HunterFeedbackMode::Off;
    hunterWifiScanInFlight = false;
    hunterBleScanInFlight = false;
    hunterBestRssi = -127;
    hunterWeakestRssi = 0;
    hunterCurrentRssi = -127;
    hunterNextScanMs = 0;
    hunterLastFeedbackMs = 0;
    hunterResetHistory();

    lv_obj_t *title =
        lv_label_create(screen);

    lv_label_set_text(
        title,
        "SIGNAL HUNTER"
    );
    lv_obj_set_style_text_color(
        title,
        lv_color_hex(0xFF52C8),
        0
    );
    lv_obj_set_style_text_font(
        title,
        &lv_font_montserrat_20,
        0
    );
    lv_obj_align(
        title,
        LV_ALIGN_TOP_MID,
        0,
        28
    );

    hunterStatusLabel =
        lv_label_create(screen);

    lv_label_set_text(
        hunterStatusLabel,
        "CHOOSE A RADIO"
    );
    lv_obj_set_width(
        hunterStatusLabel,
        330
    );
    lv_obj_set_style_text_align(
        hunterStatusLabel,
        LV_TEXT_ALIGN_CENTER,
        0
    );
    lv_obj_set_style_text_color(
        hunterStatusLabel,
        lv_color_hex(0xB6FF00),
        0
    );
    lv_obj_set_style_text_font(
        hunterStatusLabel,
        &lv_font_montserrat_12,
        0
    );
    lv_obj_align(
        hunterStatusLabel,
        LV_ALIGN_TOP_MID,
        0,
        72
    );

    // Radio choice buttons remain visible in both states so a different
    // target can be chosen at any time.
    lv_obj_t *wifiBtn =
        lv_button_create(screen);
    lv_obj_set_size(wifiBtn, 96, 42);
    lv_obj_set_pos(wifiBtn, 42, 98);
    lv_obj_set_style_radius(wifiBtn, 14, 0);
    lv_obj_set_style_bg_color(
        wifiBtn,
        lv_color_hex(0x071521),
        0
    );
    lv_obj_set_style_border_width(wifiBtn, 1, 0);
    lv_obj_set_style_border_color(
        wifiBtn,
        lv_color_hex(0x55EEFF),
        0
    );
    lv_obj_add_event_cb(
        wifiBtn,
        hunterWifiButtonEvent,
        LV_EVENT_CLICKED,
        nullptr
    );
    lv_obj_t *wifiTxt =
        lv_label_create(wifiBtn);
    lv_label_set_text(wifiTxt, "WI-FI");
    lv_obj_set_style_text_color(
        wifiTxt,
        lv_color_hex(0x55EEFF),
        0
    );
    lv_obj_set_style_text_font(
        wifiTxt,
        &lv_font_montserrat_12,
        0
    );
    lv_obj_center(wifiTxt);

    lv_obj_t *bleBtn =
        lv_button_create(screen);
    lv_obj_set_size(bleBtn, 96, 42);
    lv_obj_set_pos(bleBtn, 157, 98);
    lv_obj_set_style_radius(bleBtn, 14, 0);
    lv_obj_set_style_bg_color(
        bleBtn,
        lv_color_hex(0x120C24),
        0
    );
    lv_obj_set_style_border_width(bleBtn, 1, 0);
    lv_obj_set_style_border_color(
        bleBtn,
        lv_color_hex(0x8EA4FF),
        0
    );
    lv_obj_add_event_cb(
        bleBtn,
        hunterBleButtonEvent,
        LV_EVENT_CLICKED,
        nullptr
    );
    lv_obj_t *bleTxt =
        lv_label_create(bleBtn);
    lv_label_set_text(bleTxt, "BLE");
    lv_obj_set_style_text_color(
        bleTxt,
        lv_color_hex(0x8EA4FF),
        0
    );
    lv_obj_set_style_text_font(
        bleTxt,
        &lv_font_montserrat_12,
        0
    );
    lv_obj_center(bleTxt);

    lv_obj_t *trackerBtn =
        lv_button_create(screen);
    lv_obj_set_size(trackerBtn, 116, 42);
    lv_obj_set_pos(trackerBtn, 272, 98);
    lv_obj_set_style_radius(trackerBtn, 14, 0);
    lv_obj_set_style_bg_color(
        trackerBtn,
        lv_color_hex(0x1A1207),
        0
    );
    lv_obj_set_style_border_width(trackerBtn, 1, 0);
    lv_obj_set_style_border_color(
        trackerBtn,
        lv_color_hex(0xFFB84D),
        0
    );
    lv_obj_add_event_cb(
        trackerBtn,
        hunterTrackerButtonEvent,
        LV_EVENT_CLICKED,
        nullptr
    );
    lv_obj_t *trackerTxt =
        lv_label_create(trackerBtn);
    lv_label_set_text(trackerTxt, "TRACKERS");
    lv_obj_set_style_text_color(
        trackerTxt,
        lv_color_hex(0xFFB84D),
        0
    );
    lv_obj_set_style_text_font(
        trackerTxt,
        &lv_font_montserrat_12,
        0
    );
    lv_obj_center(trackerTxt);

    // --------------------------------------------------------------
    // CHOOSER STATE
    // This is the ONLY content shown before a target is selected.
    // --------------------------------------------------------------
    hunterList =
        lv_obj_create(screen);

    lv_obj_set_size(
        hunterList,
        340,
        320
    );
    lv_obj_set_pos(
        hunterList,
        35,
        154
    );
    lv_obj_set_flex_flow(
        hunterList,
        LV_FLEX_FLOW_COLUMN
    );
    lv_obj_set_flex_align(
        hunterList,
        LV_FLEX_ALIGN_START,
        LV_FLEX_ALIGN_CENTER,
        LV_FLEX_ALIGN_CENTER
    );
    lv_obj_set_scroll_dir(
        hunterList,
        LV_DIR_VER
    );
    lv_obj_set_scrollbar_mode(
        hunterList,
        LV_SCROLLBAR_MODE_AUTO
    );
    lv_obj_set_style_bg_color(
        hunterList,
        lv_color_hex(0x030711),
        0
    );
    lv_obj_set_style_bg_opa(
        hunterList,
        245,
        0
    );
    lv_obj_set_style_border_width(
        hunterList,
        1,
        0
    );
    lv_obj_set_style_border_color(
        hunterList,
        lv_color_hex(0x34465A),
        0
    );
    lv_obj_set_style_radius(
        hunterList,
        18,
        0
    );
    lv_obj_set_style_pad_all(
        hunterList,
        8,
        0
    );
    lv_obj_set_style_pad_row(
        hunterList,
        7,
        0
    );

    lv_obj_t *chooseArt =
        lv_image_create(hunterList);

    lv_image_set_src(
        chooseArt,
        &hunter_choose_bg_img
    );

    lv_obj_t *choose =
        lv_label_create(hunterList);

    lv_label_set_text(
        choose,
        "Choose Wi-Fi, BLE, or Trackers above."
    );
    lv_obj_set_width(
        choose,
        310
    );
    lv_obj_set_style_text_align(
        choose,
        LV_TEXT_ALIGN_CENTER,
        0
    );
    lv_obj_set_style_text_color(
        choose,
        lv_color_hex(0xFFFFFF),
        0
    );
    lv_obj_set_style_text_font(
        choose,
        &lv_font_montserrat_14,
        0
    );

    // --------------------------------------------------------------
    // TRACKING STATE
    // Entire panel starts hidden and only appears AFTER target select.
    // --------------------------------------------------------------
    hunterTrackingPanel =
        lv_obj_create(screen);

    lv_obj_set_size(
        hunterTrackingPanel,
        350,
        320
    );
    lv_obj_set_pos(
        hunterTrackingPanel,
        30,
        154
    );
    lv_obj_set_style_radius(
        hunterTrackingPanel,
        18,
        0
    );
    lv_obj_set_style_bg_color(
        hunterTrackingPanel,
        lv_color_hex(0x030711),
        0
    );
    lv_obj_set_style_bg_opa(
        hunterTrackingPanel,
        245,
        0
    );
    lv_obj_set_style_border_width(
        hunterTrackingPanel,
        1,
        0
    );
    lv_obj_set_style_border_color(
        hunterTrackingPanel,
        lv_color_hex(0xFF52C8),
        0
    );
    lv_obj_set_style_pad_all(
        hunterTrackingPanel,
        0,
        0
    );
    lv_obj_add_flag(
        hunterTrackingPanel,
        LV_OBJ_FLAG_HIDDEN
    );

    hunterTargetLabel =
        lv_label_create(
            hunterTrackingPanel
        );

    lv_label_set_text(
        hunterTargetLabel,
        "NO TARGET"
    );
    lv_obj_set_width(
        hunterTargetLabel,
        320
    );
    lv_obj_set_style_text_align(
        hunterTargetLabel,
        LV_TEXT_ALIGN_CENTER,
        0
    );
    lv_obj_set_style_text_color(
        hunterTargetLabel,
        lv_color_hex(0xF4F6FF),
        0
    );
    lv_obj_set_style_text_font(
        hunterTargetLabel,
        &lv_font_montserrat_14,
        0
    );
    lv_obj_set_pos(
        hunterTargetLabel,
        15,
        12
    );

    hunterRssiLabel =
        lv_label_create(
            hunterTrackingPanel
        );

    lv_label_set_text(
        hunterRssiLabel,
        "-- dBm"
    );
    lv_obj_set_width(
        hunterRssiLabel,
        320
    );
    lv_obj_set_style_text_align(
        hunterRssiLabel,
        LV_TEXT_ALIGN_CENTER,
        0
    );
    lv_obj_set_style_text_color(
        hunterRssiLabel,
        lv_color_hex(0x55EEFF),
        0
    );
    lv_obj_set_style_text_font(
        hunterRssiLabel,
        &lv_font_montserrat_32,
        0
    );
    lv_obj_set_pos(
        hunterRssiLabel,
        15,
        54
    );

    hunterStrengthLabel =
        lv_label_create(
            hunterTrackingPanel
        );

    lv_label_set_text(
        hunterStrengthLabel,
        "WAITING"
    );
    lv_obj_set_width(
        hunterStrengthLabel,
        320
    );
    lv_obj_set_style_text_align(
        hunterStrengthLabel,
        LV_TEXT_ALIGN_CENTER,
        0
    );
    lv_obj_set_style_text_color(
        hunterStrengthLabel,
        lv_color_hex(0xF5FF3B),
        0
    );
    lv_obj_set_style_text_font(
        hunterStrengthLabel,
        &lv_font_montserrat_16,
        0
    );
    lv_obj_set_pos(
        hunterStrengthLabel,
        15,
        98
    );

    hunterSignalBar =
        lv_bar_create(
            hunterTrackingPanel
        );

    lv_obj_set_size(
        hunterSignalBar,
        292,
        16
    );
    lv_obj_set_pos(
        hunterSignalBar,
        29,
        130
    );
    lv_bar_set_range(
        hunterSignalBar,
        0,
        100
    );
    lv_bar_set_value(
        hunterSignalBar,
        0,
        LV_ANIM_OFF
    );

    lv_obj_t *historyCard =
        lv_obj_create(
            hunterTrackingPanel
        );

    lv_obj_set_size(
        historyCard,
        316,
        72
    );
    lv_obj_set_pos(
        historyCard,
        17,
        158
    );
    lv_obj_set_style_radius(
        historyCard,
        14,
        0
    );
    lv_obj_set_style_bg_color(
        historyCard,
        lv_color_hex(0x050B14),
        0
    );
    lv_obj_set_style_bg_opa(
        historyCard,
        230,
        0
    );
    lv_obj_set_style_border_width(
        historyCard,
        1,
        0
    );
    lv_obj_set_style_border_color(
        historyCard,
        lv_color_hex(0x263A50),
        0
    );
    lv_obj_set_style_pad_all(
        historyCard,
        0,
        0
    );

    for (uint8_t i = 0;
         i < HUNTER_HISTORY_COUNT;
         ++i) {

        hunterHistoryBars[i] =
            lv_obj_create(historyCard);

        lv_obj_remove_style_all(
            hunterHistoryBars[i]
        );

        lv_obj_set_size(
            hunterHistoryBars[i],
            8,
            3
        );

        lv_obj_set_pos(
            hunterHistoryBars[i],
            6 + i * 12,
            50
        );

        lv_obj_set_style_bg_color(
            hunterHistoryBars[i],
            lv_color_hex(0x34465A),
            0
        );

        lv_obj_set_style_bg_opa(
            hunterHistoryBars[i],
            LV_OPA_COVER,
            0
        );
    }

    lv_obj_t *modeBtn =
        lv_button_create(
            hunterTrackingPanel
        );

    lv_obj_set_size(
        modeBtn,
        174,
        42
    );
    lv_obj_set_pos(
        modeBtn,
        16,
        252
    );
    lv_obj_set_style_radius(
        modeBtn,
        14,
        0
    );
    lv_obj_set_style_bg_color(
        modeBtn,
        lv_color_hex(0x0C1520),
        0
    );
    lv_obj_set_style_border_width(
        modeBtn,
        1,
        0
    );
    lv_obj_set_style_border_color(
        modeBtn,
        lv_color_hex(0xFF52C8),
        0
    );
    lv_obj_add_event_cb(
        modeBtn,
        hunterModeEvent,
        LV_EVENT_CLICKED,
        nullptr
    );

    hunterModeLabel =
        lv_label_create(modeBtn);

    lv_obj_set_style_text_color(
        hunterModeLabel,
        lv_color_hex(0xFFFFFF),
        0
    );
    lv_obj_set_style_text_font(
        hunterModeLabel,
        &lv_font_montserrat_12,
        0
    );
    lv_obj_center(hunterModeLabel);
    updateHunterModeLabel();

    lv_obj_t *testBtn =
        lv_button_create(
            hunterTrackingPanel
        );

    lv_obj_set_size(
        testBtn,
        132,
        42
    );
    lv_obj_set_pos(
        testBtn,
        202,
        252
    );
    lv_obj_set_style_radius(
        testBtn,
        14,
        0
    );
    lv_obj_set_style_bg_color(
        testBtn,
        lv_color_hex(0x071521),
        0
    );
    lv_obj_set_style_border_width(
        testBtn,
        1,
        0
    );
    lv_obj_set_style_border_color(
        testBtn,
        lv_color_hex(0x55EEFF),
        0
    );
    lv_obj_add_event_cb(
        testBtn,
        hunterSoundTestEvent,
        LV_EVENT_CLICKED,
        nullptr
    );

    lv_obj_t *testTxt =
        lv_label_create(testBtn);

    lv_label_set_text(
        testTxt,
        "TEST SOUND"
    );
    lv_obj_set_style_text_color(
        testTxt,
        lv_color_hex(0x55EEFF),
        0
    );
    lv_obj_set_style_text_font(
        testTxt,
        &lv_font_montserrat_12,
        0
    );
    lv_obj_center(testTxt);

    hunterTimer =
        lv_timer_create(
            hunterTimerCb,
            100,
            nullptr
        );

    // Explicitly start in chooser mode.
    showHunterChooser();

    noteActivity();
}



void mapOpenEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED || !clickAllowed()) return;
    noteActivity();
    hapticTap();
    mapOpenPending = true;
}

void mapBackEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED || !clickAllowed()) return;
    noteActivity();
    hapticTap();
    mapBackToGpsPending = true;
}

void refreshMapShellZoom()
{
    if (!mapViewport) return;

    // Temporary visual-only grid scaling. Real map tiles will replace this
    // shell, but zoom should already give immediate visual feedback.
    const int spacing = 28 + (static_cast<int>(mapZoomLevel) - 14) * 12;
    const int cx = 175;
    const int cy = 146;

    for (int i = 0; i < 7; ++i) {
        const int offset = (i - 3) * spacing;

        if (mapGridVertical[i]) {
            lv_obj_set_pos(mapGridVertical[i], cx + offset, 10);
        }
        if (mapGridHorizontal[i]) {
            lv_obj_set_pos(mapGridHorizontal[i], 10, cy + offset);
        }
    }

    if (mapZoomLabel) {
        lv_label_set_text_fmt(
            mapZoomLabel,
            "ZOOM %u",
            (unsigned)mapZoomLevel
        );
    }
}

void mapZoomEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED || !clickAllowed()) return;

    const intptr_t delta =
        reinterpret_cast<intptr_t>(lv_event_get_user_data(e));

    int next = static_cast<int>(mapZoomLevel) + static_cast<int>(delta);
    next = constrain(next, 14, 18);
    mapZoomLevel = static_cast<uint8_t>(next);

    refreshMapShellZoom();
    mapLoadedTileZ = -1;
    mapTileReloadPending = true;

    noteActivity();
    hapticTap();
}

bool mapSdFolderReady()
{
    if (!ensureM33KSdReady()) return false;

    bool ready = false;
    if (instance.lockSPI(pdMS_TO_TICKS(250))) {
        if (!SD.exists("/M33KX/MAPS")) {
            SD.mkdir("/M33KX/MAPS");
        }
        ready = SD.exists("/M33KX/MAPS");
        instance.unlockSPI();
    }
    return ready;
}

bool mapGpsToTile(
    double lat,
    double lon,
    uint8_t zoom,
    int32_t &tileX,
    int32_t &tileY,
    uint16_t &pixelX,
    uint16_t &pixelY)
{
    lat = constrain(lat, -85.05112878, 85.05112878);
    lon = constrain(lon, -180.0, 180.0);

    const double n = static_cast<double>(1UL << zoom);
    const double xFloat = ((lon + 180.0) / 360.0) * n;

    const double latRad = lat * PI / 180.0;
    const double mercator =
        log(tan(latRad) + (1.0 / cos(latRad)));
    const double yFloat =
        (1.0 - mercator / PI) * 0.5 * n;

    tileX = static_cast<int32_t>(floor(xFloat));
    tileY = static_cast<int32_t>(floor(yFloat));

    const int32_t maxTile = static_cast<int32_t>((1UL << zoom) - 1UL);
    tileX = constrain(tileX, 0L, maxTile);
    tileY = constrain(tileY, 0L, maxTile);

    const double fracX = xFloat - floor(xFloat);
    const double fracY = yFloat - floor(yFloat);
    pixelX = static_cast<uint16_t>(
        constrain(static_cast<int>(fracX * MAP_TILE_SIZE), 0, 255)
    );
    pixelY = static_cast<uint16_t>(
        constrain(static_cast<int>(fracY * MAP_TILE_SIZE), 0, 255)
    );

    return true;
}

String mapTilePath(uint8_t zoom, int32_t tileX, int32_t tileY)
{
    String path;
    path.reserve(54);
    path += "/M33KX/MAPS/";
    path += String(zoom);
    path += "/";
    path += String(tileX);
    path += "/";
    path += String(tileY);
    path += ".rgb565";
    return path;
}

void mapSetPlaceholder(const String &text, uint32_t color)
{
    if (!mapTilePlaceholder) return;

    lv_label_set_text(mapTilePlaceholder, text.c_str());
    lv_obj_set_style_text_color(
        mapTilePlaceholder,
        lv_color_hex(color),
        0
    );
    lv_obj_remove_flag(mapTilePlaceholder, LV_OBJ_FLAG_HIDDEN);
}

bool mapCopyTileRegion(
    uint8_t zoom,
    int32_t tileX,
    int32_t tileY,
    int32_t viewWorldLeft,
    int32_t viewWorldTop,
    size_t &tilesLoaded,
    size_t &tilesMissing)
{
    const int32_t tileWorldLeft = tileX * MAP_TILE_SIZE;
    const int32_t tileWorldTop = tileY * MAP_TILE_SIZE;
    const int32_t srcX0 = max(0L, viewWorldLeft - tileWorldLeft);
    const int32_t srcY0 = max(0L, viewWorldTop - tileWorldTop);
    const int32_t srcX1 = min(
        static_cast<int32_t>(MAP_TILE_SIZE),
        viewWorldLeft + static_cast<int32_t>(MAP_VIEW_WIDTH) - tileWorldLeft
    );
    const int32_t srcY1 = min(
        static_cast<int32_t>(MAP_TILE_SIZE),
        viewWorldTop + static_cast<int32_t>(MAP_VIEW_HEIGHT) - tileWorldTop
    );

    if (srcX1 <= srcX0 || srcY1 <= srcY0) return true;

    const String path = mapTilePath(zoom, tileX, tileY);
    File file = SD.open(path.c_str(), FILE_READ);
    if (!file || static_cast<size_t>(file.size()) != MAP_TILE_BYTES) {
        if (file) file.close();
        ++tilesMissing;
        Serial.printf("[M33K MAP] missing stitched tile %s\n", path.c_str());
        return false;
    }

    const size_t copyWidth =
        static_cast<size_t>(srcX1 - srcX0) * 2U;
    bool ok = true;

    for (int32_t srcY = srcY0; srcY < srcY1; ++srcY) {
        const int32_t worldY = tileWorldTop + srcY;
        const int32_t dstY = worldY - viewWorldTop;
        const int32_t worldX = tileWorldLeft + srcX0;
        const int32_t dstX = worldX - viewWorldLeft;

        const size_t sourceOffset =
            (static_cast<size_t>(srcY) * MAP_TILE_SIZE +
             static_cast<size_t>(srcX0)) * 2U;
        const size_t destinationOffset =
            (static_cast<size_t>(dstY) * MAP_VIEW_WIDTH +
             static_cast<size_t>(dstX)) * 2U;

        if (!file.seek(sourceOffset) ||
            file.read(mapTileBuffer + destinationOffset, copyWidth) != copyWidth) {
            ok = false;
            break;
        }
    }

    file.close();

    if (ok) {
        ++tilesLoaded;
    } else {
        ++tilesMissing;
        Serial.printf("[M33K MAP] stitched tile read failed %s\n", path.c_str());
    }

    return ok;
}


uint16_t mapRgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return static_cast<uint16_t>(
        ((static_cast<uint16_t>(r) & 0xF8U) << 8) |
        ((static_cast<uint16_t>(g) & 0xFCU) << 3) |
        (static_cast<uint16_t>(b) >> 3)
    );
}

void mapClearVectorBuffer()
{
    if (!mapTileBuffer) return;

    const uint16_t bg = mapRgb565(18, 23, 29);
    uint16_t *pixels = reinterpret_cast<uint16_t *>(mapTileBuffer);
    const size_t count =
        static_cast<size_t>(MAP_VIEW_WIDTH) * MAP_VIEW_HEIGHT;

    for (size_t i = 0; i < count; ++i) {
        pixels[i] = bg;
    }
}

uint8_t mapLineOutCode(int32_t x, int32_t y)
{
    uint8_t code = 0;
    if (x < 0) code |= 1;
    else if (x >= MAP_VIEW_WIDTH) code |= 2;
    if (y < 0) code |= 4;
    else if (y >= MAP_VIEW_HEIGHT) code |= 8;
    return code;
}

bool mapClipLine(
    int32_t &x0,
    int32_t &y0,
    int32_t &x1,
    int32_t &y1)
{
    uint8_t code0 = mapLineOutCode(x0, y0);
    uint8_t code1 = mapLineOutCode(x1, y1);

    while (true) {
        if ((code0 | code1) == 0) return true;
        if ((code0 & code1) != 0) return false;

        const uint8_t out = code0 ? code0 : code1;
        int32_t x = 0;
        int32_t y = 0;

        if (out & 8) {
            if (y1 == y0) return false;
            y = MAP_VIEW_HEIGHT - 1;
            x = x0 + static_cast<int32_t>(
                static_cast<int64_t>(x1 - x0) *
                static_cast<int64_t>(y - y0) /
                static_cast<int64_t>(y1 - y0)
            );
        } else if (out & 4) {
            if (y1 == y0) return false;
            y = 0;
            x = x0 + static_cast<int32_t>(
                static_cast<int64_t>(x1 - x0) *
                static_cast<int64_t>(y - y0) /
                static_cast<int64_t>(y1 - y0)
            );
        } else if (out & 2) {
            if (x1 == x0) return false;
            x = MAP_VIEW_WIDTH - 1;
            y = y0 + static_cast<int32_t>(
                static_cast<int64_t>(y1 - y0) *
                static_cast<int64_t>(x - x0) /
                static_cast<int64_t>(x1 - x0)
            );
        } else {
            if (x1 == x0) return false;
            x = 0;
            y = y0 + static_cast<int32_t>(
                static_cast<int64_t>(y1 - y0) *
                static_cast<int64_t>(x - x0) /
                static_cast<int64_t>(x1 - x0)
            );
        }

        if (out == code0) {
            x0 = x;
            y0 = y;
            code0 = mapLineOutCode(x0, y0);
        } else {
            x1 = x;
            y1 = y;
            code1 = mapLineOutCode(x1, y1);
        }
    }
}

void mapPlotVectorPixel(int32_t x, int32_t y, uint16_t color)
{
    if (!mapTileBuffer ||
        x < 0 || x >= MAP_VIEW_WIDTH ||
        y < 0 || y >= MAP_VIEW_HEIGHT) {
        return;
    }

    uint16_t *pixels = reinterpret_cast<uint16_t *>(mapTileBuffer);
    pixels[
        static_cast<size_t>(y) * MAP_VIEW_WIDTH +
        static_cast<size_t>(x)
    ] = color;
}

void mapDrawVectorLine(
    int32_t x0,
    int32_t y0,
    int32_t x1,
    int32_t y1,
    uint16_t color,
    uint8_t width)
{
    if (!mapClipLine(x0, y0, x1, y1)) return;

    const int32_t dx = abs(x1 - x0);
    const int32_t sx = x0 < x1 ? 1 : -1;
    const int32_t dy = -abs(y1 - y0);
    const int32_t sy = y0 < y1 ? 1 : -1;
    int32_t err = dx + dy;
    const int32_t radius = max(0, static_cast<int>(width) / 2);

    while (true) {
        for (int32_t oy = -radius; oy <= radius; ++oy) {
            for (int32_t ox = -radius; ox <= radius; ++ox) {
                mapPlotVectorPixel(x0 + ox, y0 + oy, color);
            }
        }

        if (x0 == x1 && y0 == y1) break;

        const int32_t e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

bool mapLatLonToWorldPixel(
    double lat,
    double lon,
    uint8_t zoom,
    int32_t &worldX,
    int32_t &worldY)
{
    lat = constrain(lat, -85.05112878, 85.05112878);
    lon = constrain(lon, -180.0, 180.0);

    const double scale =
        static_cast<double>(MAP_TILE_SIZE) *
        static_cast<double>(1UL << zoom);
    const double x = ((lon + 180.0) / 360.0) * scale;

    const double latRad = lat * PI / 180.0;
    const double mercator =
        log(tan(latRad) + (1.0 / cos(latRad)));
    const double y =
        (1.0 - mercator / PI) * 0.5 * scale;

    worldX = static_cast<int32_t>(lround(x));
    worldY = static_cast<int32_t>(lround(y));
    return true;
}

void mapVectorRoadStyle(
    uint8_t roadClass,
    uint8_t zoom,
    uint16_t &color,
    uint8_t &width)
{
    switch (roadClass) {
        case 0:
            color = mapRgb565(238, 126, 107);
            width = zoom >= 17 ? 5 : 4;
            break;
        case 1:
            color = mapRgb565(220, 183, 91);
            width = zoom >= 17 ? 4 : 3;
            break;
        case 2:
            color = mapRgb565(220, 183, 91);
            width = zoom >= 17 ? 3 : 2;
            break;
        case 3:
            color = mapRgb565(194, 197, 201);
            width = zoom >= 17 ? 3 : 2;
            break;
        case 4:
            color = mapRgb565(126, 132, 139);
            width = zoom >= 17 ? 2 : 1;
            break;
        case 5:
            color = mapRgb565(104, 111, 119);
            width = 1;
            break;
        default:
            color = mapRgb565(82, 91, 84);
            width = 1;
            break;
    }
}


String mapVectorV2ChunkPath(int32_t chunkX, int32_t chunkY)
{
    String path;
    path.reserve(64);
    path += MAP_VECTOR_V2_ROOT;
    path += "/";
    path += String(MAP_VECTOR_V2_CHUNK_ZOOM);
    path += "/";
    path += String(chunkX);
    path += "/";
    path += String(chunkY);
    path += ".m33kc";
    return path;
}

void mapResetVectorLabelCandidates()
{
    for (uint8_t i = 0; i < MAP_VECTOR_LABEL_COUNT; ++i) {
        mapVectorLabelText[i] = "";
        mapVectorLabelX[i] = 0;
        mapVectorLabelY[i] = 0;
        mapVectorLabelPriority[i] = 255;
    }
}

void mapHideVectorLabels()
{
    for (auto *label : mapVectorLabels) {
        if (label) lv_obj_add_flag(label, LV_OBJ_FLAG_HIDDEN);
    }
}

bool mapVectorRoadVisibleAtZoom(uint8_t roadClass, uint8_t zoom)
{
    if (zoom <= 14) return roadClass <= 3;
    if (zoom == 15) return roadClass <= 4;
    if (zoom == 16) return roadClass <= 5;
    return roadClass <= 6;
}

bool mapVectorLabelAllowedAtZoom(uint8_t roadClass, uint8_t zoom)
{
    if (zoom <= 14) return roadClass <= 2;
    if (zoom == 15) return roadClass <= 3;
    if (zoom == 16) return roadClass <= 4;
    return roadClass <= 5;
}

void mapOfferVectorLabel(
    const String &text,
    uint8_t priority,
    int16_t x,
    int16_t y)
{
    if (text.length() == 0 ||
        x < 24 || x > MAP_VIEW_WIDTH - 24 ||
        y < 12 || y > MAP_VIEW_HEIGHT - 12) {
        return;
    }

    for (uint8_t i = 0; i < MAP_VECTOR_LABEL_COUNT; ++i) {
        if (mapVectorLabelText[i] == text) return;
    }

    int bestSlot = -1;
    uint8_t worstPriority = 0;

    for (uint8_t i = 0; i < MAP_VECTOR_LABEL_COUNT; ++i) {
        if (mapVectorLabelText[i].length() == 0) {
            bestSlot = i;
            break;
        }

        if (mapVectorLabelPriority[i] >= worstPriority) {
            worstPriority = mapVectorLabelPriority[i];
            bestSlot = i;
        }
    }

    if (bestSlot < 0) return;

    if (mapVectorLabelText[bestSlot].length() != 0 &&
        priority >= mapVectorLabelPriority[bestSlot]) {
        return;
    }

    mapVectorLabelText[bestSlot] = text;
    mapVectorLabelPriority[bestSlot] = priority;
    mapVectorLabelX[bestSlot] = x;
    mapVectorLabelY[bestSlot] = y;
}

void mapApplyVectorLabels()
{
    for (uint8_t i = 0; i < MAP_VECTOR_LABEL_COUNT; ++i) {
        lv_obj_t *label = mapVectorLabels[i];
        if (!label) continue;

        if (mapVectorLabelText[i].length() == 0) {
            lv_obj_add_flag(label, LV_OBJ_FLAG_HIDDEN);
            continue;
        }

        lv_label_set_text(label, mapVectorLabelText[i].c_str());
        lv_obj_set_pos(
            label,
            mapVectorLabelX[i],
            mapVectorLabelY[i]
        );
        lv_obj_remove_flag(label, LV_OBJ_FLAG_HIDDEN);
    }
}

bool mapLoadVectorV2Chunk(
    const String &path,
    int32_t viewLeft,
    int32_t viewTop,
    size_t &featuresDrawn,
    size_t &segmentsDrawn)
{
    File file = SD.open(path.c_str(), FILE_READ);
    if (!file) return false;

    uint8_t magic[8] = {};
    uint32_t featureCount = 0;

    const bool headerOk =
        file.read(magic, sizeof(magic)) == sizeof(magic) &&
        file.read(
            reinterpret_cast<uint8_t *>(&featureCount),
            sizeof(featureCount)
        ) == sizeof(featureCount) &&
        memcmp(magic, "M33KC2\0\0", 8) == 0;

    if (!headerOk || featureCount > 100000U) {
        file.close();
        Serial.printf(
            "[M33K MAP] invalid v2 chunk %s\n",
            path.c_str()
        );
        return false;
    }

    const uint8_t zoom =
        static_cast<uint8_t>(mapRequestedTileZ);

    for (uint32_t featureIndex = 0;
         featureIndex < featureCount;
         ++featureIndex) {
        uint8_t roadClass = 0;
        uint8_t flags = 0;
        uint8_t nameLen = 0;
        uint8_t reserved = 0;
        uint16_t pointCount = 0;

        if (file.read(&roadClass, 1) != 1 ||
            file.read(&flags, 1) != 1 ||
            file.read(&nameLen, 1) != 1 ||
            file.read(&reserved, 1) != 1 ||
            file.read(
                reinterpret_cast<uint8_t *>(&pointCount),
                sizeof(pointCount)
            ) != sizeof(pointCount)) {
            file.close();
            return false;
        }

        if (nameLen > 39 || pointCount < 2 || pointCount > 8192) {
            file.close();
            return false;
        }

        char nameBuffer[40] = {};
        if (nameLen > 0 &&
            file.read(
                reinterpret_cast<uint8_t *>(nameBuffer),
                nameLen
            ) != nameLen) {
            file.close();
            return false;
        }
        nameBuffer[nameLen] = '\0';

        if (!mapVectorRoadVisibleAtZoom(roadClass, zoom)) {
            const size_t skipBytes =
                static_cast<size_t>(pointCount) * 8U;
            if (!file.seek(file.position() + skipBytes)) {
                file.close();
                return false;
            }
            continue;
        }

        uint16_t color = 0;
        uint8_t width = 1;
        mapVectorRoadStyle(roadClass, zoom, color, width);

        int32_t previousX = 0;
        int32_t previousY = 0;
        bool havePrevious = false;
        bool featureTouched = false;
        int16_t labelX = 0;
        int16_t labelY = 0;

        for (uint16_t pointIndex = 0;
             pointIndex < pointCount;
             ++pointIndex) {
            int32_t latE7 = 0;
            int32_t lonE7 = 0;

            if (file.read(
                    reinterpret_cast<uint8_t *>(&latE7),
                    sizeof(latE7)
                ) != sizeof(latE7) ||
                file.read(
                    reinterpret_cast<uint8_t *>(&lonE7),
                    sizeof(lonE7)
                ) != sizeof(lonE7)) {
                file.close();
                return false;
            }

            int32_t worldX = 0;
            int32_t worldY = 0;
            mapLatLonToWorldPixel(
                static_cast<double>(latE7) / 10000000.0,
                static_cast<double>(lonE7) / 10000000.0,
                zoom,
                worldX,
                worldY
            );

            const int32_t screenX = worldX - viewLeft;
            const int32_t screenY = worldY - viewTop;

            if (havePrevious) {
                int32_t x0 = previousX;
                int32_t y0 = previousY;
                int32_t x1 = screenX;
                int32_t y1 = screenY;

                if (mapClipLine(x0, y0, x1, y1)) {
                    mapDrawVectorLine(
                        previousX,
                        previousY,
                        screenX,
                        screenY,
                        color,
                        width
                    );
                    featureTouched = true;
                    ++segmentsDrawn;

                    if (labelX == 0 && labelY == 0) {
                        labelX = static_cast<int16_t>((x0 + x1) / 2);
                        labelY = static_cast<int16_t>((y0 + y1) / 2);
                    }
                }
            }

            previousX = screenX;
            previousY = screenY;
            havePrevious = true;
        }

        if (featureTouched) {
            ++featuresDrawn;

            if (nameLen > 0 &&
                mapVectorLabelAllowedAtZoom(roadClass, zoom)) {
                mapOfferVectorLabel(
                    String(nameBuffer),
                    roadClass,
                    labelX,
                    labelY
                );
            }
        }
    }

    file.close();
    return true;
}

bool loadVectorMapV2()
{
    if (!mapTileBuffer ||
        mapRequestedTileZ < 14 ||
        mapRequestedWorldX < 0 ||
        mapRequestedWorldY < 0) {
        return false;
    }

    const int32_t scale =
        1L << (
            static_cast<uint8_t>(mapRequestedTileZ) -
            MAP_VECTOR_V2_CHUNK_ZOOM
        );

    const int32_t viewLeft =
        mapRequestedWorldX - static_cast<int32_t>(MAP_VIEW_WIDTH / 2);
    const int32_t viewTop =
        mapRequestedWorldY - static_cast<int32_t>(MAP_VIEW_HEIGHT / 2);
    const int32_t viewRight =
        viewLeft + static_cast<int32_t>(MAP_VIEW_WIDTH) - 1;
    const int32_t viewBottom =
        viewTop + static_cast<int32_t>(MAP_VIEW_HEIGHT) - 1;

    const int32_t z12Left = viewLeft / scale;
    const int32_t z12Top = viewTop / scale;
    const int32_t z12Right = viewRight / scale;
    const int32_t z12Bottom = viewBottom / scale;

    const int32_t firstChunkX = z12Left / MAP_TILE_SIZE;
    const int32_t firstChunkY = z12Top / MAP_TILE_SIZE;
    const int32_t lastChunkX = z12Right / MAP_TILE_SIZE;
    const int32_t lastChunkY = z12Bottom / MAP_TILE_SIZE;

    mapClearVectorBuffer();
    mapResetVectorLabelCandidates();

    size_t chunksLoaded = 0;
    size_t featuresDrawn = 0;
    size_t segmentsDrawn = 0;

    if (!instance.lockSPI(pdMS_TO_TICKS(1000))) {
        return false;
    }

    for (int32_t cy = firstChunkY; cy <= lastChunkY; ++cy) {
        for (int32_t cx = firstChunkX; cx <= lastChunkX; ++cx) {
            const String path = mapVectorV2ChunkPath(cx, cy);
            if (mapLoadVectorV2Chunk(
                    path,
                    viewLeft,
                    viewTop,
                    featuresDrawn,
                    segmentsDrawn)) {
                ++chunksLoaded;
            }
        }
    }

    instance.unlockSPI();

    if (chunksLoaded == 0 || featuresDrawn == 0) {
        mapHideVectorLabels();
        return false;
    }

    mapLoadedTileZ = mapRequestedTileZ;
    mapLoadedTileX = mapRequestedTileX;
    mapLoadedTileY = mapRequestedTileY;
    mapLoadedWorldX = mapRequestedWorldX;
    mapLoadedWorldY = mapRequestedWorldY;
    mapTileLoaded = true;

    if (mapTileCanvas) {
        lv_obj_remove_flag(mapTileCanvas, LV_OBJ_FLAG_HIDDEN);
        lv_obj_invalidate(mapTileCanvas);
    }
    if (mapTilePlaceholder) {
        lv_obj_add_flag(mapTilePlaceholder, LV_OBJ_FLAG_HIDDEN);
    }

    mapApplyVectorLabels();

    if (mapSdLabel) {
        lv_label_set_text_fmt(
            mapSdLabel,
            "VECTOR V2  %u CHUNK%s  %u ROADS",
            (unsigned)chunksLoaded,
            chunksLoaded == 1 ? "" : "S",
            (unsigned)featuresDrawn
        );
        lv_obj_set_style_text_color(
            mapSdLabel,
            lv_color_hex(0x55EEFF),
            0
        );
    }

    Serial.printf(
        "[M33K MAP] vector v2 z%d chunks=%u roads=%u segments=%u\n",
        (int)mapLoadedTileZ,
        (unsigned)chunksLoaded,
        (unsigned)featuresDrawn,
        (unsigned)segmentsDrawn
    );

    return true;
}

bool loadVectorPrototypeMap()
{
    if (!mapTileBuffer ||
        mapRequestedTileZ < 0 ||
        mapRequestedWorldX < 0 ||
        mapRequestedWorldY < 0) {
        return false;
    }

    bool rendered = false;
    size_t featuresDrawn = 0;
    size_t segmentsDrawn = 0;

    if (!instance.lockSPI(pdMS_TO_TICKS(1000))) {
        return false;
    }

    File file = SD.open(MAP_VECTOR_PROTOTYPE_PATH, FILE_READ);
    if (!file) {
        instance.unlockSPI();
        return false;
    }

    uint8_t magic[8] = {};
    uint32_t featureCount = 0;

    const bool headerOk =
        file.read(magic, sizeof(magic)) == sizeof(magic) &&
        file.read(
            reinterpret_cast<uint8_t *>(&featureCount),
            sizeof(featureCount)
        ) == sizeof(featureCount) &&
        memcmp(magic, "M33KV1\0\0", 8) == 0;

    if (!headerOk || featureCount > 200000U) {
        file.close();
        instance.unlockSPI();
        Serial.println("[M33K MAP] invalid vector prototype header");
        return false;
    }

    mapClearVectorBuffer();

    const int32_t viewLeft =
        mapRequestedWorldX - static_cast<int32_t>(MAP_VIEW_WIDTH / 2);
    const int32_t viewTop =
        mapRequestedWorldY - static_cast<int32_t>(MAP_VIEW_HEIGHT / 2);

    for (uint32_t featureIndex = 0;
         featureIndex < featureCount;
         ++featureIndex) {
        uint8_t roadClass = 0;
        uint8_t flags = 0;
        uint16_t pointCount = 0;

        if (file.read(&roadClass, 1) != 1 ||
            file.read(&flags, 1) != 1 ||
            file.read(
                reinterpret_cast<uint8_t *>(&pointCount),
                sizeof(pointCount)
            ) != sizeof(pointCount)) {
            break;
        }

        if (pointCount < 2 || pointCount > 8192) {
            file.close();
            instance.unlockSPI();
            Serial.println("[M33K MAP] invalid vector point count");
            return false;
        }

        int32_t previousX = 0;
        int32_t previousY = 0;
        bool havePrevious = false;
        bool featureTouched = false;

        uint16_t color = 0;
        uint8_t width = 1;
        mapVectorRoadStyle(
            roadClass,
            static_cast<uint8_t>(mapRequestedTileZ),
            color,
            width
        );

        for (uint16_t pointIndex = 0;
             pointIndex < pointCount;
             ++pointIndex) {
            int32_t latE7 = 0;
            int32_t lonE7 = 0;

            if (file.read(
                    reinterpret_cast<uint8_t *>(&latE7),
                    sizeof(latE7)
                ) != sizeof(latE7) ||
                file.read(
                    reinterpret_cast<uint8_t *>(&lonE7),
                    sizeof(lonE7)
                ) != sizeof(lonE7)) {
                file.close();
                instance.unlockSPI();
                return false;
            }

            int32_t worldX = 0;
            int32_t worldY = 0;
            mapLatLonToWorldPixel(
                static_cast<double>(latE7) / 10000000.0,
                static_cast<double>(lonE7) / 10000000.0,
                static_cast<uint8_t>(mapRequestedTileZ),
                worldX,
                worldY
            );

            const int32_t screenX = worldX - viewLeft;
            const int32_t screenY = worldY - viewTop;

            if (havePrevious) {
                int32_t x0 = previousX;
                int32_t y0 = previousY;
                int32_t x1 = screenX;
                int32_t y1 = screenY;

                if (mapClipLine(x0, y0, x1, y1)) {
                    mapDrawVectorLine(
                        previousX,
                        previousY,
                        screenX,
                        screenY,
                        color,
                        width
                    );
                    featureTouched = true;
                    ++segmentsDrawn;
                }
            }

            previousX = screenX;
            previousY = screenY;
            havePrevious = true;
        }

        if (featureTouched) ++featuresDrawn;
    }

    file.close();
    instance.unlockSPI();

    if (featuresDrawn == 0) {
        return false;
    }

    mapLoadedTileZ = mapRequestedTileZ;
    mapLoadedTileX = mapRequestedTileX;
    mapLoadedTileY = mapRequestedTileY;
    mapLoadedWorldX = mapRequestedWorldX;
    mapLoadedWorldY = mapRequestedWorldY;
    mapTileLoaded = true;
    rendered = true;

    if (mapTileCanvas) {
        lv_obj_remove_flag(mapTileCanvas, LV_OBJ_FLAG_HIDDEN);
        lv_obj_invalidate(mapTileCanvas);
    }
    if (mapTilePlaceholder) {
        lv_obj_add_flag(mapTilePlaceholder, LV_OBJ_FLAG_HIDDEN);
    }

    if (mapSdLabel) {
        lv_label_set_text_fmt(
            mapSdLabel,
            "VECTOR MAP  %u ROADS",
            (unsigned)featuresDrawn
        );
        lv_obj_set_style_text_color(
            mapSdLabel,
            lv_color_hex(0x55EEFF),
            0
        );
    }

    Serial.printf(
        "[M33K MAP] vector prototype z%d roads=%u segments=%u\n",
        (int)mapLoadedTileZ,
        (unsigned)featuresDrawn,
        (unsigned)segmentsDrawn
    );

    return rendered;
}

bool loadRequestedMapTile()
{
    mapTileReloadPending = false;

    if (currentPage != Page::Map ||
        mapRequestedTileZ < 0 ||
        mapRequestedTileX < 0 ||
        mapRequestedTileY < 0 ||
        mapRequestedWorldX < 0 ||
        mapRequestedWorldY < 0) {
        return false;
    }

    if (!mapTileBuffer) {
        mapTileBuffer = static_cast<uint8_t *>(
            heap_caps_malloc(
                MAP_VIEW_BYTES,
                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
            )
        );

        if (!mapTileBuffer) {
            mapSetPlaceholder("MAP ERROR\nPSRAM ALLOCATION FAILED", 0xFF667F);
            if (mapSdLabel) {
                lv_label_set_text(mapSdLabel, "MAP: PSRAM ALLOC FAILED");
                lv_obj_set_style_text_color(
                    mapSdLabel,
                    lv_color_hex(0xFF667F),
                    0
                );
            }
            return false;
        }

        if (mapTileCanvas) {
            lv_canvas_set_buffer(
                mapTileCanvas,
                mapTileBuffer,
                MAP_VIEW_WIDTH,
                MAP_VIEW_HEIGHT,
                LV_COLOR_FORMAT_RGB565
            );
        }
    }

    if (!ensureM33KSdReady()) {
        mapTileLoaded = false;
        if (mapTileCanvas) {
            lv_obj_add_flag(mapTileCanvas, LV_OBJ_FLAG_HIDDEN);
        }
        mapSetPlaceholder("SD OFFLINE\nNO MAP TILE", 0xFFB84D);
        return false;
    }

    // Prefer chunked vector v2. During development, retain the flat v1
    // prototype and proven stitched RGB565 renderer as fallbacks.
    if (loadVectorMapV2()) {
        return true;
    }

    if (loadVectorPrototypeMap()) {
        return true;
    }

    mapHideVectorLabels();
    memset(mapTileBuffer, 0, MAP_VIEW_BYTES);

    const int32_t viewWorldLeft =
        mapRequestedWorldX - static_cast<int32_t>(MAP_VIEW_WIDTH / 2);
    const int32_t viewWorldTop =
        mapRequestedWorldY - static_cast<int32_t>(MAP_VIEW_HEIGHT / 2);

    const int32_t firstTileX =
        static_cast<int32_t>(floor(
            static_cast<double>(viewWorldLeft) / MAP_TILE_SIZE
        ));
    const int32_t firstTileY =
        static_cast<int32_t>(floor(
            static_cast<double>(viewWorldTop) / MAP_TILE_SIZE
        ));
    const int32_t lastTileX =
        static_cast<int32_t>(floor(
            static_cast<double>(
                viewWorldLeft + static_cast<int32_t>(MAP_VIEW_WIDTH) - 1
            ) / MAP_TILE_SIZE
        ));
    const int32_t lastTileY =
        static_cast<int32_t>(floor(
            static_cast<double>(
                viewWorldTop + static_cast<int32_t>(MAP_VIEW_HEIGHT) - 1
            ) / MAP_TILE_SIZE
        ));

    size_t tilesLoaded = 0;
    size_t tilesMissing = 0;

    if (instance.lockSPI(pdMS_TO_TICKS(1000))) {
        for (int32_t ty = firstTileY; ty <= lastTileY; ++ty) {
            for (int32_t tx = firstTileX; tx <= lastTileX; ++tx) {
                mapCopyTileRegion(
                    static_cast<uint8_t>(mapRequestedTileZ),
                    tx,
                    ty,
                    viewWorldLeft,
                    viewWorldTop,
                    tilesLoaded,
                    tilesMissing
                );
            }
        }
        instance.unlockSPI();
    } else {
        mapSetPlaceholder("MAP BUSY\nSD LOCK TIMEOUT", 0xFFB84D);
        return false;
    }

    if (tilesLoaded == 0) {
        mapTileLoaded = false;
        if (mapTileCanvas) {
            lv_obj_add_flag(mapTileCanvas, LV_OBJ_FLAG_HIDDEN);
        }
        mapSetPlaceholder("MAP TILES NOT FOUND\nCHECK REGIONAL PACK", 0xFFB84D);
        if (mapSdLabel) {
            lv_label_set_text(mapSdLabel, "MAP: NO VISIBLE TILES FOUND");
            lv_obj_set_style_text_color(
                mapSdLabel,
                lv_color_hex(0xFFB84D),
                0
            );
        }
        return false;
    }

    mapLoadedTileZ = mapRequestedTileZ;
    mapLoadedTileX = mapRequestedTileX;
    mapLoadedTileY = mapRequestedTileY;
    mapLoadedWorldX = mapRequestedWorldX;
    mapLoadedWorldY = mapRequestedWorldY;
    mapTileLoaded = true;

    if (mapTileCanvas) {
        lv_obj_remove_flag(mapTileCanvas, LV_OBJ_FLAG_HIDDEN);
        lv_obj_invalidate(mapTileCanvas);
    }
    if (mapTilePlaceholder) {
        lv_obj_add_flag(mapTilePlaceholder, LV_OBJ_FLAG_HIDDEN);
    }

    if (mapSdLabel) {
        lv_label_set_text_fmt(
            mapSdLabel,
            "MAP STITCHED  %u TILE%s%s",
            (unsigned)tilesLoaded,
            tilesLoaded == 1 ? "" : "S",
            tilesMissing ? "  |  EDGE MISSING" : ""
        );
        lv_obj_set_style_text_color(
            mapSdLabel,
            lv_color_hex(tilesMissing ? 0xFFB84D : 0x7CFF45),
            0
        );
    }

    Serial.printf(
        "[M33K MAP] stitched z%d center=%ld/%ld world=%ld,%ld tiles=%u missing=%u\n",
        (int)mapLoadedTileZ,
        (long)mapLoadedTileX,
        (long)mapLoadedTileY,
        (long)mapLoadedWorldX,
        (long)mapLoadedWorldY,
        (unsigned)tilesLoaded,
        (unsigned)tilesMissing
    );

    return true;
}

void updateMapUi()
{
    if (currentPage != Page::Map) return;

    const bool haveFix =
        gpsParser.location.isValid() &&
        gpsParser.location.age() < 5000;

    if (mapStatusLabel) {
        if (haveFix) {
            const unsigned sats = gpsParser.satellites.isValid()
                ? (unsigned)gpsParser.satellites.value()
                : 0U;
            lv_label_set_text_fmt(
                mapStatusLabel,
                "GPS FIX  %u SAT  |  NORTH UP",
                sats
            );
            lv_obj_set_style_text_color(
                mapStatusLabel,
                lv_color_hex(0x7CFF45),
                0
            );
        } else {
            lv_label_set_text(
                mapStatusLabel,
                "GPS SEARCHING  |  NORTH UP"
            );
            lv_obj_set_style_text_color(
                mapStatusLabel,
                lv_color_hex(0xF5FF3B),
                0
            );
        }
    }

    if (mapCoordLabel) {
        if (haveFix) {
            lv_label_set_text_fmt(
                mapCoordLabel,
                "LAT %.6f\nLON %.6f",
                gpsParser.location.lat(),
                gpsParser.location.lng()
            );
        } else {
            lv_label_set_text(
                mapCoordLabel,
                "LAT --.------\nLON --.------"
            );
        }
    }

    if (haveFix) {
        const double mapLat = gpsParser.location.lat();
        const double mapLon = gpsParser.location.lng();

        int32_t tileX = -1;
        int32_t tileY = -1;
        uint16_t pixelX = 0;
        uint16_t pixelY = 0;

        mapGpsToTile(
            mapLat,
            mapLon,
            mapZoomLevel,
            tileX,
            tileY,
            pixelX,
            pixelY
        );

        mapRequestedTileZ = static_cast<int8_t>(mapZoomLevel);
        mapRequestedTileX = tileX;
        mapRequestedTileY = tileY;
        mapRequestedLat = mapLat;
        mapRequestedLon = mapLon;
        mapRequestedWorldX =
            tileX * static_cast<int32_t>(MAP_TILE_SIZE) +
            static_cast<int32_t>(pixelX);
        mapRequestedWorldY =
            tileY * static_cast<int32_t>(MAP_TILE_SIZE) +
            static_cast<int32_t>(pixelY);

        if (mapLoadedTileZ != mapRequestedTileZ ||
            mapLoadedWorldX < 0 ||
            mapLoadedWorldY < 0 ||
            abs(mapRequestedWorldX - mapLoadedWorldX) >=
                MAP_RELOAD_PIXEL_THRESHOLD ||
            abs(mapRequestedWorldY - mapLoadedWorldY) >=
                MAP_RELOAD_PIXEL_THRESHOLD) {
            mapTileReloadPending = true;
        }

        if (mapMarkerDot) {
            lv_obj_set_pos(
                mapMarkerDot,
                static_cast<int>(MAP_VIEW_WIDTH / 2) - 9,
                static_cast<int>(MAP_VIEW_HEIGHT / 2) - 9
            );
            lv_obj_remove_flag(mapMarkerDot, LV_OBJ_FLAG_HIDDEN);
        }

        if (mapMarkerLabel) {
            lv_label_set_text(
                mapMarkerLabel,
                "YOU"
            );
            lv_obj_set_pos(
                mapMarkerLabel,
                static_cast<int>(MAP_VIEW_WIDTH / 2) - 12,
                static_cast<int>(MAP_VIEW_HEIGHT / 2) + 11
            );
            lv_obj_remove_flag(mapMarkerLabel, LV_OBJ_FLAG_HIDDEN);
        }

    } else {
        if (mapMarkerDot) {
            lv_obj_add_flag(mapMarkerDot, LV_OBJ_FLAG_HIDDEN);
        }
        if (mapMarkerLabel) {
            lv_obj_add_flag(mapMarkerLabel, LV_OBJ_FLAG_HIDDEN);
        }
    }

    if (mapZoomLabel) {
        lv_label_set_text_fmt(
            mapZoomLabel,
            "ZOOM %u",
            (unsigned)mapZoomLevel
        );
    }
}

void mapTimerCb(lv_timer_t *timer)
{
    LV_UNUSED(timer);
    updateMapUi();
}

void showMapPage()
{
    clearScreen();
    currentPage = Page::Map;
    mapOpenPending = false;
    mapBackToGpsPending = false;
    mapTileReloadPending = false;
    mapTileLoaded = false;
    mapLoadedTileX = -1;
    mapLoadedTileY = -1;
    mapLoadedTileZ = -1;
    mapRequestedTileX = -1;
    mapRequestedTileY = -1;
    mapRequestedTileZ = -1;
    mapLoadedWorldX = -1;
    mapLoadedWorldY = -1;
    mapRequestedWorldX = -1;
    mapRequestedWorldY = -1;

    ensureGpsReady();
    gpsToolActive = true;

    createSafeHeaderBack(mapBackEvent);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, LV_SYMBOL_GPS "  M33K MAP");
    lv_obj_set_style_text_color(title, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 8, 26);

    // Map viewport shell. Raster SD tiles will be drawn into this area in the
    // next milestone; keeping the viewport isolated makes tile replacement
    // independent from the status/controls below.
    mapViewport = lv_obj_create(screen);
    lv_obj_set_size(mapViewport, 350, 292);
    lv_obj_set_pos(mapViewport, 30, 78);
    lv_obj_set_style_radius(mapViewport, 18, 0);
    lv_obj_set_style_bg_color(mapViewport, lv_color_hex(0x061019), 0);
    lv_obj_set_style_bg_opa(mapViewport, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(mapViewport, 1, 0);
    lv_obj_set_style_border_color(mapViewport, lv_color_hex(0x234460), 0);
    lv_obj_set_style_pad_all(mapViewport, 0, 0);
    lv_obj_clear_flag(mapViewport, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_clip_corner(mapViewport, true, 0);

    mapTileCanvas = lv_canvas_create(mapViewport);
    lv_obj_set_size(mapTileCanvas, MAP_VIEW_WIDTH, MAP_VIEW_HEIGHT);
    lv_obj_set_pos(mapTileCanvas, 0, 0);
    lv_obj_clear_flag(mapTileCanvas, LV_OBJ_FLAG_SCROLLABLE);
    // Never show the canvas until valid tile bytes have been loaded. Its
    // external PSRAM buffer is intentionally allocated lazily, so displaying
    // it before the first successful read can expose uninitialized pixels.
    lv_obj_add_flag(mapTileCanvas, LV_OBJ_FLAG_HIDDEN);

    // Fallback grid stays behind the marker until a tile loads.
    for (int i = 0; i < 7; ++i) {
        mapGridVertical[i] = lv_obj_create(mapViewport);
        lv_obj_remove_style_all(mapGridVertical[i]);
        lv_obj_set_size(mapGridVertical[i], 1, 270);
        lv_obj_set_style_bg_color(
            mapGridVertical[i],
            lv_color_hex(0x163247),
            0
        );
        lv_obj_set_style_bg_opa(mapGridVertical[i], 120, 0);

        mapGridHorizontal[i] = lv_obj_create(mapViewport);
        lv_obj_remove_style_all(mapGridHorizontal[i]);
        lv_obj_set_size(mapGridHorizontal[i], 330, 1);
        lv_obj_set_style_bg_color(
            mapGridHorizontal[i],
            lv_color_hex(0x163247),
            0
        );
        lv_obj_set_style_bg_opa(mapGridHorizontal[i], 120, 0);
    }

    for (uint8_t i = 0; i < MAP_VECTOR_LABEL_COUNT; ++i) {
        mapVectorLabels[i] = lv_label_create(mapViewport);
        lv_label_set_text(mapVectorLabels[i], "");
        lv_obj_set_style_text_color(
            mapVectorLabels[i],
            lv_color_hex(0xFFFFFF),
            0
        );
        lv_obj_set_style_text_font(
            mapVectorLabels[i],
            &lv_font_montserrat_12,
            0
        );
        lv_obj_set_style_bg_color(
            mapVectorLabels[i],
            lv_color_hex(0x071019),
            0
        );
        lv_obj_set_style_bg_opa(
            mapVectorLabels[i],
            LV_OPA_70,
            0
        );
        lv_obj_set_style_pad_left(mapVectorLabels[i], 2, 0);
        lv_obj_set_style_pad_right(mapVectorLabels[i], 2, 0);
        lv_obj_set_style_pad_top(mapVectorLabels[i], 1, 0);
        lv_obj_set_style_pad_bottom(mapVectorLabels[i], 1, 0);
        lv_obj_set_style_radius(mapVectorLabels[i], 3, 0);
        lv_obj_add_flag(mapVectorLabels[i], LV_OBJ_FLAG_HIDDEN);
    }

    mapTilePlaceholder = lv_label_create(mapViewport);
    lv_label_set_text(
        mapTilePlaceholder,
        "WAITING FOR GPS\nOFFLINE RGB565 TILES"
    );
    lv_obj_set_width(mapTilePlaceholder, 300);
    lv_obj_set_style_text_align(
        mapTilePlaceholder,
        LV_TEXT_ALIGN_CENTER,
        0
    );
    lv_obj_set_style_text_color(
        mapTilePlaceholder,
        lv_color_hex(0x5A7892),
        0
    );
    lv_obj_set_style_text_font(
        mapTilePlaceholder,
        &lv_font_montserrat_12,
        0
    );
    lv_obj_align(mapTilePlaceholder, LV_ALIGN_TOP_MID, 0, 18);

    mapMarkerDot = lv_obj_create(mapViewport);
    lv_obj_remove_style_all(mapMarkerDot);
    lv_obj_set_size(mapMarkerDot, 18, 18);
    lv_obj_set_style_radius(mapMarkerDot, 9, 0);
    lv_obj_set_style_bg_color(mapMarkerDot, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_bg_opa(mapMarkerDot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(mapMarkerDot, 3, 0);
    lv_obj_set_style_border_color(mapMarkerDot, lv_color_hex(0xFFFFFF), 0);
    lv_obj_add_flag(mapMarkerDot, LV_OBJ_FLAG_HIDDEN);

    mapMarkerLabel = lv_label_create(mapViewport);
    lv_label_set_text(mapMarkerLabel, "YOU");
    lv_obj_set_style_text_color(
        mapMarkerLabel,
        lv_color_hex(0xFFFFFF),
        0
    );
    lv_obj_set_style_text_font(
        mapMarkerLabel,
        &lv_font_montserrat_12,
        0
    );
    lv_obj_align(mapMarkerLabel, LV_ALIGN_CENTER, 0, 24);
    lv_obj_add_flag(mapMarkerLabel, LV_OBJ_FLAG_HIDDEN);

    mapStatusLabel = lv_label_create(screen);
    lv_label_set_text(mapStatusLabel, "GPS SEARCHING  |  NORTH UP");
    lv_obj_set_width(mapStatusLabel, 236);
    lv_obj_set_style_text_color(mapStatusLabel, lv_color_hex(0xF5FF3B), 0);
    lv_obj_set_style_text_font(mapStatusLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(mapStatusLabel, 36, 380);

    mapCoordLabel = lv_label_create(screen);
    lv_label_set_text(mapCoordLabel, "LAT --.------\nLON --.------");
    lv_obj_set_width(mapCoordLabel, 238);
    lv_obj_set_style_text_color(mapCoordLabel, lv_color_hex(0xDDE7F2), 0);
    lv_obj_set_style_text_font(mapCoordLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(mapCoordLabel, 36, 404);

    mapSdLabel = lv_label_create(screen);
    const bool tilesReady = mapSdFolderReady();
    lv_label_set_text(
        mapSdLabel,
        tilesReady
            ? "SD MAPS: FOUND"
            : "SD MAPS: /M33KX/MAPS MISSING"
    );
    lv_obj_set_width(mapSdLabel, 238);
    lv_obj_set_style_text_color(
        mapSdLabel,
        lv_color_hex(tilesReady ? 0x7CFF45 : 0xFFB84D),
        0
    );
    lv_obj_set_style_text_font(mapSdLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(mapSdLabel, 36, 454);

    mapZoomLabel = lv_label_create(screen);
    lv_label_set_text_fmt(
        mapZoomLabel,
        "ZOOM %u",
        (unsigned)mapZoomLevel
    );
    lv_obj_set_width(mapZoomLabel, 100);
    lv_obj_set_style_text_align(mapZoomLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(mapZoomLabel, lv_color_hex(0xFF52C8), 0);
    lv_obj_set_style_text_font(mapZoomLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(mapZoomLabel, 280, 382);

    lv_obj_t *minus = lv_button_create(screen);
    lv_obj_set_size(minus, 46, 46);
    lv_obj_set_pos(minus, 276, 414);
    lv_obj_set_style_radius(minus, 14, 0);
    lv_obj_set_style_bg_color(minus, lv_color_hex(0x101A2A), 0);
    lv_obj_set_style_border_width(minus, 1, 0);
    lv_obj_set_style_border_color(minus, lv_color_hex(0x55EEFF), 0);
    lv_obj_add_event_cb(
        minus,
        mapZoomEvent,
        LV_EVENT_CLICKED,
        reinterpret_cast<void *>(static_cast<intptr_t>(-1))
    );
    lv_obj_t *minusText = lv_label_create(minus);
    lv_label_set_text(minusText, "-");
    lv_obj_set_style_text_font(minusText, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(minusText, lv_color_hex(0x55EEFF), 0);
    lv_obj_center(minusText);

    lv_obj_t *plus = lv_button_create(screen);
    lv_obj_set_size(plus, 46, 46);
    lv_obj_set_pos(plus, 330, 414);
    lv_obj_set_style_radius(plus, 14, 0);
    lv_obj_set_style_bg_color(plus, lv_color_hex(0x101A2A), 0);
    lv_obj_set_style_border_width(plus, 1, 0);
    lv_obj_set_style_border_color(plus, lv_color_hex(0xFF52C8), 0);
    lv_obj_add_event_cb(
        plus,
        mapZoomEvent,
        LV_EVENT_CLICKED,
        reinterpret_cast<void *>(static_cast<intptr_t>(1))
    );
    lv_obj_t *plusText = lv_label_create(plus);
    lv_label_set_text(plusText, "+");
    lv_obj_set_style_text_font(plusText, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(plusText, lv_color_hex(0xFF52C8), 0);
    lv_obj_center(plusText);

    refreshMapShellZoom();

    mapTimer = lv_timer_create(mapTimerCb, 500, nullptr);
    lv_timer_ready(mapTimer);
    noteActivity();
    Serial.println("[M33K] Map shell opened");
}


void showGpsPage()
{
    clearScreen();
    currentPage = Page::GPS;

    createSafeHeaderBack(backEvent);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, LV_SYMBOL_GPS "  GPS / GNSS");
    lv_obj_set_style_text_color(title, lv_color_hex(0xF5FF3B), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 10, 26);

    // GNSS stays alive across page changes; simply reassert its power rail.
    ensureGpsReady();

    gpsToolActive = true;
    gpsRecoveryAttempted = false;
    gpsSessionBytes = 0;
    gpsSessionLocationSeen = false;
    gpsPageOpenedMs = millis();
    gpsLastByteMs = gpsPageOpenedMs;

    lv_obj_t *statusCard = lv_obj_create(screen);
    lv_obj_set_size(statusCard, 350, 58);
    lv_obj_set_pos(statusCard, 30, 78);
    lv_obj_set_style_radius(statusCard, 17, 0);
    lv_obj_set_style_bg_color(statusCard, lv_color_hex(0x07101D), 0);
    lv_obj_set_style_bg_opa(statusCard, 235, 0);
    lv_obj_set_style_border_width(statusCard, 1, 0);
    lv_obj_set_style_border_color(statusCard, lv_color_hex(0xF5FF3B), 0);
    lv_obj_set_style_pad_all(statusCard, 0, 0);

    gpsStatusLabel = lv_label_create(statusCard);
    lv_label_set_text(
        gpsStatusLabel,
        LV_SYMBOL_GPS "  STARTING GNSS..."
    );
    lv_obj_set_width(gpsStatusLabel, 325);
    lv_obj_set_style_text_align(
        gpsStatusLabel,
        LV_TEXT_ALIGN_CENTER,
        0
    );
    lv_obj_set_style_text_color(
        gpsStatusLabel,
        lv_color_hex(0xF5FF3B),
        0
    );
    lv_obj_set_style_text_font(
        gpsStatusLabel,
        &lv_font_montserrat_16,
        0
    );
    lv_obj_center(gpsStatusLabel);

    // Coordinates
    lv_obj_t *coordCard = lv_obj_create(screen);
    lv_obj_set_size(coordCard, 350, 98);
    lv_obj_set_pos(coordCard, 30, 146);
    lv_obj_set_style_radius(coordCard, 17, 0);
    lv_obj_set_style_bg_color(coordCard, lv_color_hex(0x050B14), 0);
    lv_obj_set_style_bg_opa(coordCard, 225, 0);
    lv_obj_set_style_border_width(coordCard, 1, 0);
    lv_obj_set_style_border_color(coordCard, lv_color_hex(0x234460), 0);
    lv_obj_set_style_pad_all(coordCard, 0, 0);

    gpsSatLabel = lv_label_create(coordCard);
    lv_label_set_text(gpsSatLabel, "SATELLITES  --");
    lv_obj_set_style_text_color(gpsSatLabel, lv_color_hex(0x7CFF45), 0);
    lv_obj_set_style_text_font(gpsSatLabel, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(gpsSatLabel, 14, 10);

    gpsLatLabel = lv_label_create(coordCard);
    lv_label_set_text(gpsLatLabel, "LAT   --.------");
    lv_obj_set_style_text_color(gpsLatLabel, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_text_font(gpsLatLabel, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(gpsLatLabel, 14, 38);

    gpsLonLabel = lv_label_create(coordCard);
    lv_label_set_text(gpsLonLabel, "LON   --.------");
    lv_obj_set_style_text_color(gpsLonLabel, lv_color_hex(0xFF52C8), 0);
    lv_obj_set_style_text_font(gpsLonLabel, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(gpsLonLabel, 14, 66);

    // Metrics
    lv_obj_t *metricCard = lv_obj_create(screen);
    lv_obj_set_size(metricCard, 350, 122);
    lv_obj_set_pos(metricCard, 30, 254);
    lv_obj_set_style_radius(metricCard, 17, 0);
    lv_obj_set_style_bg_color(metricCard, lv_color_hex(0x050B14), 0);
    lv_obj_set_style_bg_opa(metricCard, 225, 0);
    lv_obj_set_style_border_width(metricCard, 1, 0);
    lv_obj_set_style_border_color(metricCard, lv_color_hex(0x234460), 0);
    lv_obj_set_style_pad_all(metricCard, 0, 0);

    const int metricX[4] = {10, 95, 180, 265};
    const uint32_t metricColors[4] = {
        0x55EEFF, 0xFF52C8, 0x7CFF45, 0xF5FF3B
    };

    lv_obj_t **metricLabels[4] = {
        &gpsAltLabel,
        &gpsSpeedLabel,
        &gpsHdopLabel,
        &gpsCourseLabel
    };

    const char *metricText[4] = {
        "ALTITUDE\n-- m",
        "SPEED\n-- km/h",
        "HDOP\n--",
        "COURSE\n-- deg"
    };

    for (uint8_t i = 0; i < 4; ++i) {
        lv_obj_t *divider = nullptr;
        if (i > 0) {
            divider = lv_obj_create(metricCard);
            lv_obj_remove_style_all(divider);
            lv_obj_set_size(divider, 1, 84);
            lv_obj_set_pos(divider, metricX[i] - 7, 18);
            lv_obj_set_style_bg_color(
                divider,
                lv_color_hex(0x244058),
                0
            );
            lv_obj_set_style_bg_opa(divider, 210, 0);
        }

        *metricLabels[i] = lv_label_create(metricCard);
        lv_label_set_text(*metricLabels[i], metricText[i]);
        lv_obj_set_width(*metricLabels[i], 76);
        lv_obj_set_style_text_align(
            *metricLabels[i],
            LV_TEXT_ALIGN_CENTER,
            0
        );
        lv_obj_set_style_text_color(
            *metricLabels[i],
            lv_color_hex(metricColors[i]),
            0
        );
        lv_obj_set_style_text_font(
            *metricLabels[i],
            &lv_font_montserrat_12,
            0
        );
        lv_obj_set_pos(*metricLabels[i], metricX[i], 32);
    }

    // GNSS time / low-level data indicator
    lv_obj_t *dataCard = lv_obj_create(screen);
    lv_obj_set_size(dataCard, 350, 88);
    lv_obj_set_pos(dataCard, 30, 386);
    lv_obj_set_style_radius(dataCard, 17, 0);
    lv_obj_set_style_bg_color(dataCard, lv_color_hex(0x07101D), 0);
    lv_obj_set_style_bg_opa(dataCard, 235, 0);
    lv_obj_set_style_border_width(dataCard, 1, 0);
    lv_obj_set_style_border_color(dataCard, lv_color_hex(0x223A55), 0);
    lv_obj_set_style_pad_all(dataCard, 0, 0);

    gpsUtcLabel = lv_label_create(dataCard);
    lv_label_set_text(
        gpsUtcLabel,
        "UTC  ---- -- --   --:--:--"
    );
    lv_obj_set_width(gpsUtcLabel, 326);
    lv_obj_set_style_text_align(
        gpsUtcLabel,
        LV_TEXT_ALIGN_CENTER,
        0
    );
    lv_obj_set_style_text_color(
        gpsUtcLabel,
        lv_color_hex(0xFFFFFF),
        0
    );
    lv_obj_set_style_text_font(
        gpsUtcLabel,
        &lv_font_montserrat_12,
        0
    );
    lv_obj_set_pos(gpsUtcLabel, 12, 16);

    gpsDataLabel = lv_label_create(dataCard);
    lv_label_set_text(
        gpsDataLabel,
        "UART 38400  |  RX 0 bytes  |  NO DATA"
    );
    lv_obj_set_width(gpsDataLabel, 326);
    lv_obj_set_style_text_align(
        gpsDataLabel,
        LV_TEXT_ALIGN_CENTER,
        0
    );
    lv_obj_set_style_text_color(
        gpsDataLabel,
        lv_color_hex(0xAFC2D8),
        0
    );
    lv_obj_set_style_text_font(
        gpsDataLabel,
        &lv_font_montserrat_12,
        0
    );
    lv_obj_set_pos(gpsDataLabel, 12, 44);

    // Offline map is hidden for now: the "MAP" launch button is not created, so
    // the map page is unreachable from the UI. The map code remains dormant and
    // can be restored (or fully removed) later. (mapOpenEvent/showMapPage unused.)

    gpsTimer = lv_timer_create(gpsTimerCb, 500, nullptr);
    lv_timer_ready(gpsTimer);

    noteActivity();
    Serial.println("[M33K] GPS page started; listening to Serial1 @ 38400");
}

void nfcRestartEvent(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();
    stopNfcTool();

    if (nfcStatusLabel) {
        lv_label_set_text(nfcStatusLabel, "STARTING NFC FIELD...");
        lv_obj_set_style_text_color(
            nfcStatusLabel,
            lv_color_hex(0xF5FF3B),
            0
        );
    }
    if (nfcTypeLabel) {
        lv_label_set_text(nfcTypeLabel, "WAITING FOR TAG");
    }
    if (nfcUidLabel) {
        lv_label_set_text(nfcUidLabel, "UID\n--:--:--:--");
    }
    if (nfcTechLabel) {
        lv_label_set_text(nfcTechLabel, "ATQA -- --   |   SAK --");
    }

    const bool ready = startNfcTool();
    if (nfcStatusLabel) {
        lv_label_set_text(
            nfcStatusLabel,
            ready
                ? "RFAL READY  -  TAP NFC-A TAG"
                : LV_SYMBOL_WARNING "  NFC READER INIT FAILED"
        );
        lv_obj_set_style_text_color(
            nfcStatusLabel,
            lv_color_hex(ready ? 0xD0B2FF : 0xFF667F),
            0
        );
    }
    if (!ready && nfcTechLabel) {
        lv_label_set_text_fmt(
            nfcTechLabel,
            "INIT CODE %d   |   DISC %d",
            nfcInitCode,
            nfcDiscoverCode
        );
    }
}

void showNfcPage()
{
    clearScreen();
    currentPage = Page::NFC;
    nfcTagCount = 0;

    createSafeHeaderBack(backEvent);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "NFC READER");
    lv_obj_set_style_text_color(title, lv_color_hex(0xD0B2FF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 8, 26);

    lv_obj_t *statusCard = lv_obj_create(screen);
    lv_obj_set_size(statusCard, 350, 58);
    lv_obj_set_pos(statusCard, 30, 78);
    lv_obj_set_style_radius(statusCard, 17, 0);
    lv_obj_set_style_bg_color(statusCard, lv_color_hex(0x07101D), 0);
    lv_obj_set_style_bg_opa(statusCard, 235, 0);
    lv_obj_set_style_border_width(statusCard, 1, 0);
    lv_obj_set_style_border_color(statusCard, lv_color_hex(0xB37BFF), 0);
    lv_obj_set_style_pad_all(statusCard, 0, 0);

    nfcStatusLabel = lv_label_create(statusCard);
    lv_label_set_text(nfcStatusLabel, "STARTING NFC FIELD...");
    lv_obj_set_width(nfcStatusLabel, 326);
    lv_obj_set_style_text_align(nfcStatusLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(nfcStatusLabel, lv_color_hex(0xF5FF3B), 0);
    lv_obj_set_style_text_font(nfcStatusLabel, &lv_font_montserrat_14, 0);
    lv_obj_center(nfcStatusLabel);

    lv_obj_t *readerCard = lv_obj_create(screen);
    lv_obj_set_size(readerCard, 350, 236);
    lv_obj_set_pos(readerCard, 30, 146);
    lv_obj_set_style_radius(readerCard, 20, 0);
    lv_obj_set_style_bg_color(readerCard, lv_color_hex(0x050B14), 0);
    lv_obj_set_style_bg_opa(readerCard, 230, 0);
    lv_obj_set_style_border_width(readerCard, 1, 0);
    lv_obj_set_style_border_color(readerCard, lv_color_hex(0x4D3478), 0);
    lv_obj_set_style_pad_all(readerCard, 0, 0);

    lv_obj_t *hint = lv_label_create(readerCard);
    lv_label_set_text(hint, "HOLD NFC-A TAG FLAT AGAINST WATCH BACK");
    lv_obj_set_width(hint, 324);
    lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0xAFC2D8), 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(hint, 13, 14);

    nfcTypeLabel = lv_label_create(readerCard);
    lv_label_set_text(nfcTypeLabel, "WAITING FOR TAG");
    lv_obj_set_width(nfcTypeLabel, 324);
    lv_obj_set_style_text_align(nfcTypeLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(nfcTypeLabel, lv_color_hex(0x7CFF45), 0);
    lv_obj_set_style_text_font(nfcTypeLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_pos(nfcTypeLabel, 13, 48);

    nfcUidLabel = lv_label_create(readerCard);
    lv_label_set_text(nfcUidLabel, "UID\n--:--:--:--");
    lv_obj_set_width(nfcUidLabel, 324);
    lv_obj_set_style_text_align(nfcUidLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(nfcUidLabel, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_text_font(nfcUidLabel, &lv_font_montserrat_18, 0);
    lv_obj_set_pos(nfcUidLabel, 13, 84);

    nfcTechLabel = lv_label_create(readerCard);
    lv_label_set_text(nfcTechLabel, "ATQA -- --   |   SAK --");
    lv_obj_set_width(nfcTechLabel, 324);
    lv_obj_set_style_text_align(nfcTechLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(nfcTechLabel, lv_color_hex(0xFF52C8), 0);
    lv_obj_set_style_text_font(nfcTechLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(nfcTechLabel, 13, 151);

    nfcCountLabel = lv_label_create(readerCard);
    lv_label_set_text(nfcCountLabel, "TAGS PRESENTED  0");
    lv_obj_set_width(nfcCountLabel, 324);
    lv_obj_set_style_text_align(nfcCountLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(nfcCountLabel, lv_color_hex(0xD0B2FF), 0);
    lv_obj_set_style_text_font(nfcCountLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(nfcCountLabel, 13, 198);

    lv_obj_t *restart = lv_button_create(screen);
    lv_obj_set_size(restart, 220, 52);
    lv_obj_set_pos(restart, 95, 402);
    lv_obj_set_style_radius(restart, 16, 0);
    lv_obj_set_style_bg_color(restart, lv_color_hex(0x10182A), 0);
    lv_obj_set_style_border_width(restart, 1, 0);
    lv_obj_set_style_border_color(restart, lv_color_hex(0xB37BFF), 0);
    lv_obj_add_event_cb(
        restart,
        nfcRestartEvent,
        LV_EVENT_CLICKED,
        nullptr
    );

    lv_obj_t *restartText = lv_label_create(restart);
    lv_label_set_text(restartText, "RESTART NFC FIELD");
    lv_obj_set_style_text_color(restartText, lv_color_hex(0xD0B2FF), 0);
    lv_obj_set_style_text_font(restartText, &lv_font_montserrat_14, 0);
    lv_obj_center(restartText);

    const bool ready = startNfcTool();
    lv_label_set_text(
        nfcStatusLabel,
        ready
            ? "RFAL READY  -  TAP NFC-A TAG"
            : LV_SYMBOL_WARNING "  NFC READER INIT FAILED"
    );
    lv_obj_set_style_text_color(
        nfcStatusLabel,
        lv_color_hex(ready ? 0xD0B2FF : 0xFF667F),
        0
    );
    if (!ready && nfcTechLabel) {
        lv_label_set_text_fmt(
            nfcTechLabel,
            "INIT CODE %d   |   DISC %d",
            nfcInitCode,
            nfcDiscoverCode
        );
    }

    noteActivity();
}



uint64_t m33kSdCardSizeBytes()
{
    if (!m33kSdReady) return 0;
    if (!instance.lockSPI(pdMS_TO_TICKS(250))) return 0;

    const uint64_t bytes = SD.cardSize();
    instance.unlockSPI();
    return bytes;
}

String m33kWardriveFileList()
{
    if (!m33kSdReady) {
        return "No SD card mounted.";
    }

    if (!instance.lockSPI(pdMS_TO_TICKS(350))) {
        return "SD busy. Tap REFRESH.";
    }

    String list;
    File root = SD.open("/M33KX/WARDRIVE");

    if (!root || !root.isDirectory()) {
        if (root) root.close();
        instance.unlockSPI();
        return "No Wardrive logs yet.";
    }

    uint8_t shown = 0;
    File entry = root.openNextFile();

    while (entry && shown < 8) {
        if (!entry.isDirectory()) {
            const uint32_t kb =
                static_cast<uint32_t>((entry.size() + 1023ULL) / 1024ULL);

            list += entry.name();
            list += "   ";
            list += String(kb);
            list += " KB\n";
            ++shown;
        }

        entry.close();
        entry = root.openNextFile();
    }

    root.close();
    instance.unlockSPI();

    if (list.length() == 0) {
        return "No Wardrive logs yet.";
    }

    return list;
}

void logsRefreshEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED || !clickAllowed()) return;
    
    noteActivity();
    hapticTap();
    showLogsPage();
}

void showLogsPage()
{
    clearScreen();
    currentPage = Page::Logs;
    createSafeHeaderBack(backEvent);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "SD LOGS");
    lv_obj_set_width(title, 220);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x6AFF78), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_pos(title, 95, 27);

    m33kSdReady = ensureM33KSdReady();

    lv_obj_t *status = lv_label_create(screen);
    lv_obj_set_width(status, 350);
    lv_obj_set_style_text_align(status, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(status, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(status, 30, 78);

    if (m33kSdReady) {
        const uint64_t bytes = m33kSdCardSizeBytes();
        const unsigned long mb =
            static_cast<unsigned long>(bytes / (1024ULL * 1024ULL));

        lv_label_set_text_fmt(
            status,
            "SD ONLINE   |   %lu MB",
            mb
        );
        lv_obj_set_style_text_color(status, lv_color_hex(0x7CFF45), 0);
    } else {
        lv_label_set_text(status, "SD OFFLINE");
        lv_obj_set_style_text_color(status, lv_color_hex(0xF5FF3B), 0);
    }

    lv_obj_t *card = lv_obj_create(screen);
    lv_obj_set_size(card, 350, 250);
    lv_obj_set_pos(card, 30, 108);
    lv_obj_set_style_radius(card, 18, 0);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x07101D), 0);
    lv_obj_set_style_bg_opa(card, 235, 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_border_color(
        card,
        lv_color_hex(m33kSdReady ? 0x6AFF78 : 0xF5FF3B),
        0
    );

    lv_obj_t *folder = lv_label_create(card);
    lv_label_set_text(folder, "/M33KX/WARDRIVE");
    lv_obj_set_style_text_color(folder, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_text_font(folder, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(folder, 12, 8);

    lv_obj_t *privacy = lv_label_create(card);
    lv_label_set_text(
        privacy,
        "Local SD storage only - not uploaded."
    );
    lv_obj_set_width(privacy, 310);
    lv_obj_set_style_text_color(privacy, lv_color_hex(0x9BAFC4), 0);
    lv_obj_set_style_text_font(privacy, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(privacy, 12, 32);

    lv_obj_t *fileBox = lv_obj_create(card);
    lv_obj_set_size(fileBox, 310, 166);
    lv_obj_set_pos(fileBox, 10, 62);
    lv_obj_set_style_radius(fileBox, 12, 0);
    lv_obj_set_style_bg_color(fileBox, lv_color_hex(0x030812), 0);
    lv_obj_set_style_bg_opa(fileBox, 190, 0);
    lv_obj_set_style_border_width(fileBox, 1, 0);
    lv_obj_set_style_border_color(fileBox, lv_color_hex(0x16324D), 0);
    lv_obj_set_scroll_dir(fileBox, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(fileBox, LV_SCROLLBAR_MODE_ACTIVE);

    lv_obj_t *files = lv_label_create(fileBox);
    const String fileText = m33kSdReady
        ? m33kWardriveFileList()
        : String("Insert a microSD card, then tap REFRESH.");
    lv_label_set_text(files, fileText.c_str());
    lv_obj_set_width(files, 282);
    lv_label_set_long_mode(files, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(files, lv_color_hex(0xF4F6FF), 0);
    lv_obj_set_style_text_font(files, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(files, 0, 0);

    lv_obj_t *refresh = lv_button_create(screen);
    lv_obj_set_size(refresh, 160, 46);
    lv_obj_set_pos(refresh, 125, 374);
    lv_obj_set_style_radius(refresh, 16, 0);
    lv_obj_set_style_bg_color(refresh, lv_color_hex(0x0B1A1C), 0);
    lv_obj_set_style_border_width(refresh, 1, 0);
    lv_obj_set_style_border_color(refresh, lv_color_hex(0x6AFF78), 0);
    lv_obj_add_event_cb(refresh, logsRefreshEvent, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *refreshText = lv_label_create(refresh);
    lv_label_set_text(refreshText, "REFRESH");
    lv_obj_set_style_text_color(refreshText, lv_color_hex(0x6AFF78), 0);
    lv_obj_set_style_text_font(refreshText, &lv_font_montserrat_14, 0);
    lv_obj_center(refreshText);

    noteActivity();
}

void showPlaceholder(Page page, const char *title)
{
    clearScreen();
    currentPage = page;

    createSafeHeaderBack(backEvent);

    lv_obj_t *heading = lv_label_create(screen);
    lv_label_set_text(heading, title);
    lv_obj_set_style_text_color(
        heading,
        lv_color_hex(0xF4F6FF),
        0
    );
    lv_obj_set_style_text_font(
        heading,
        &lv_font_montserrat_22,
        0
    );
    lv_obj_align(
        heading,
        LV_ALIGN_TOP_MID,
        8,
        38
    );

    lv_obj_t *card = lv_obj_create(screen);
    lv_obj_set_size(card, 330, 280);
    lv_obj_set_pos(card, 40, 120);
    lv_obj_set_style_radius(card, 20, 0);
    lv_obj_set_style_bg_color(
        card,
        lv_color_hex(0x07101D),
        0
    );
    lv_obj_set_style_bg_opa(card, 230, 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_border_color(
        card,
        lv_color_hex(0x304E82),
        0
    );

    lv_obj_t *body = lv_label_create(card);
    lv_obj_set_width(body, 286);
    lv_obj_set_style_text_align(
        body,
        LV_TEXT_ALIGN_CENTER,
        0
    );
    lv_obj_set_style_text_font(
        body,
        &lv_font_montserrat_14,
        0
    );

    if (page == Page::NFC) {
        lv_label_set_text(
            body,
            "NFC support is in progress.\n\n"
            "The NFC reader is intentionally disabled "
            "in this public beta build."
        );
    } else if (page == Page::Radio) {
        lv_label_set_text(
            body,
            "LoRa / Radio support is in progress."
        );
    } else {
        lv_label_set_text(
            body,
            "Tool module coming next."
        );
    }

    lv_obj_set_style_text_color(
        body,
        lv_color_hex(0xE6EDF7),
        0
    );

    lv_obj_center(body);

    noteActivity();
}


} // namespace

void setup()
{
    Serial.begin(115200);
    delay(250);
    randomSeed(micros());

    // Bottom custom side button; active-low programmable GPIO0.
    pinMode(M33K_BACK_BUTTON_PIN, INPUT_PULLUP);
    backButtonWasDown =
        digitalRead(M33K_BACK_BUTTON_PIN) == LOW;

    Serial.println("\n[M33K X] v0.6.0i-beta dev (no phone companion) starting...");

    // M33K X: T-Watch Ultra display must use DMA-capable
    // LVGL buffers. Without this, LVGL buffers live in PSRAM and
    // ESP-IDF has to allocate a temporary DMA TX buffer on every
    // display flush, which fails under Wi-Fi/BLE radio load.
    instance.setDisplayParams(true, false);

    Serial.println(
        "[M33K DISPLAY] QSPI DMA enabled before board init"
    );

    instance.begin(NO_HW_LORA);
    instance.setRotation(M33K_ROTATION);

    // Keep NFC exactly as LilyGoLib initialized it. The official T-Watch Ultra
    // NFC reader example does not power the ST25R3916 down after instance.begin().

    // GPS stability: LilyGoLib owns the GPS UART setup. Keep the GNSS rail on
    // and continuously parse NMEA across page transitions.
    instance.powerControl(POWER_GPS, true);
    gpsToolActive = true;
    gpsPageOpenedMs = millis();
    gpsLastByteMs = gpsPageOpenedMs;

    prefs.begin("m33kwatch", false);
    loadPersistentSettings();
    c5LoadLinkKey();

    beginLvglHelper(instance);
    instance.setBrightness(displayBrightnessRaw());

    // Queue BLE-host callbacks back to the Arduino task. This prevents
    // cross-task String/cache/NVS mutation while Wi-Fi scans are running.
    c5EventQueue = xQueueCreate(32, sizeof(C5Event));
    if (!c5EventQueue) {
        Serial.println("[M33K C5] event queue allocation failed");
    }

    // Initialize the shared NimBLE host for BLE scanning and the XIAO C5 link.
    // Phone Companion/peripheral mode is intentionally not included in this development build.
    if (!NimBLEDevice::isInitialized()) {
        NimBLEDevice::init("M33K X");
    } else {
        NimBLEDevice::setDeviceName("M33K X");
    }
    // C5 enrollment/auth traffic is sent only across an encrypted BLE Secure
    // Connections link. Do not rely on BLE bonding for long-term identity:
    // the per-device M33K X link key in Preferences is the persistent trust
    // anchor and HMAC authenticates the C5 after every reconnect. Clearing any
    // stale BLE bonds avoids post-reboot key mismatches while still requiring
    // fresh encrypted Just Works transport on each connection.
    NimBLEDevice::deleteAllBonds();
    NimBLEDevice::setSecurityAuth(false, false, true);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

    // Short tactile acknowledgment for tool buttons.
    instance.setHapticEffects(4);

    // Use the Ultra's hardware RTC. If it has never been set (or lost integrity),
    // seed it from the firmware build timestamp; after that the RTC keeps ticking.
    seedRTCIfNeeded();

    screen = lv_screen_active();
    lastActivityMs = millis();

    // Optional Wi-Fi persistence. This never changes the RTC.
    if (wifiStayConnected && savedWifiSsid.length() > 0) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(savedWifiSsid.c_str(), savedWifiPassword.c_str());
        Serial.printf("[M33K] Auto-connect requested: %s\n", savedWifiSsid.c_str());
    }

    showHome();
}

bool wardriveExitRadioIdle()
{
    if (WiFi.scanComplete() == WIFI_SCAN_RUNNING) {
        return false;
    }

    if (bleScannerInitialized) {
        NimBLEScan *scan = NimBLEDevice::getScan();
        if (scan && scan->isScanning()) {
            return false;
        }
    }

    if (c5DiscoveryInFlight) {
        return false;
    }

    return true;
}

void finalizeWardriveExitToRecon()
{
    const int16_t wifiState = WiFi.scanComplete();
    if (wifiState != WIFI_SCAN_RUNNING) {
        WiFi.scanDelete();
    }

    if (bleScannerInitialized) {
        NimBLEScan *scan = NimBLEDevice::getScan();
        if (scan && !scan->isScanning()) {
            scan->clearResults();
            scan->setMaxResults(MAX_BLE_DETAILS);
        }
    }

    bleScanFinished = false;

    // Radios are idle now, so finish cleanup before deleting the
    // Wardrive screen and constructing the Recon dashboard.
    stopWardriveTool();

    wardriveExitToReconPending = false;
    showReconPage();
}

void loop()
{
    instance.loop();

    // Complete Wardrive -> Recon only after both the LVGL callback and all
    // asynchronous Wardrive radio work have gone idle.
    if (wardriveExitToReconPending && wardriveExitRadioIdle()) {
        finalizeWardriveExitToRecon();
    }

    // --------------------------------------------------------
    // Finish cleanup from a stopped Wardrive session.
    //
    // Do this from loop(), not from the button callback, so
    // NimBLE/C5 callbacks can finish safely first.
    // --------------------------------------------------------
    if (wardriveStopCleanupPending) {

        // Continue requesting disconnect while the old transport
        // is still alive.
        if (c5Client && c5Client->isConnected()) {
            c5Client->disconnect();
        }

        const bool c5StillBusy =
            c5DiscoveryInFlight ||
            c5ScanInFlight ||
            c5TransportConnected();

        if (!c5StillBusy) {

            if (c5Client) {
                NimBLEDevice::deleteClient(c5Client);
                c5Client = nullptr;

                Serial.println(
                    "[M33K WARD] stopped-session C5 client released"
                );
            }

            c5Authenticated = false;

            wardriveC5CycleActive = false;
            wardriveC5CycleStartedMs = 0;
            wardriveC5RequestGeneration =
                c5ScanGeneration;

            // Do not merge previous-session C5 data.
            c5WifiDetailCount = 0;
            c5WifiPendingDetailCount = 0;

            // Release completed watch-side BLE results too.
            if (bleScannerInitialized) {
                NimBLEScan *scan =
                    NimBLEDevice::getScan();

                if (scan && !scan->isScanning()) {
                    scan->clearResults();
                    scan->setMaxResults(
                        MAX_BLE_DETAILS
                    );
                }
            }

            bleScanFinished = false;
            bleScanEndReason = 0;

            // Release completed local Wi-Fi scan storage.
            const int16_t wifiState =
                WiFi.scanComplete();

            if (wifiState != WIFI_SCAN_RUNNING) {
                WiFi.scanDelete();
            }

            wardriveMemoryPaused = false;
            wardriveStopCleanupPending = false;

            Serial.printf(
                "[M33K WARD] stop cleanup complete | heap=%u dma=%u\n",
                (unsigned)ESP.getFreeHeap(),
                (unsigned)heap_caps_get_free_size(
                    MALLOC_CAP_DMA |
                    MALLOC_CAP_INTERNAL
                )
            );
        }
    }

    if (wardriveOpenFromReconPending &&
        !wardriveStopCleanupPending) {

        wardriveOpenFromReconPending = false;
        showWardrivePage();
    }

    if (reconExitHomePending) {
        reconExitHomePending = false;
        showHome();
    }

    if (mapOpenPending) {
        mapOpenPending = false;
        showMapPage();
    }

    if (mapBackToGpsPending) {
        mapBackToGpsPending = false;
        showGpsPage();
    }

    if (mapTileReloadPending && currentPage == Page::Map) {
        loadRequestedMapTile();
    }

    if (wifiPendingNav != WifiPendingNav::None) {
        const WifiPendingNav nav = wifiPendingNav;
        wifiPendingNav = WifiPendingNav::None;

        if (nav == WifiPendingNav::OpenGraph) {
            wifiSubpageTransition = true;
            showWifiGraphPage();
        } else if (nav == WifiPendingNav::BackToScanner) {
            wifiSubpageTransition = true;
            showWifiPage();
        } else if (nav == WifiPendingNav::ExitHome) {
            wifiSubpageTransition = false;
            showHome();
        }
    }

    // Apply NimBLE callback results on the main Arduino task before any LVGL
    // Wi-Fi timer can start/restart a scan.
    serviceC5Events();

#ifdef USING_ST25R3916
    // LilyGO official NFC_Reader cadence: service RFAL once per main-loop pass
    // while the NFC page is open, before LVGL processing.
    if (nfcReaderReady && currentPage == Page::NFC) {
        NFCReader.rfalNfcWorker();
        ++nfcWorkerCalls;
    }
#endif

    handleHardwareBackButton();

    // Keep GNSS parsing non-blocking. The MIA-M10Q is on Serial1 at 38400.
    feedGpsSerial();

    if (millis() - lastPageBatteryUpdateMs >= 1000) {
        lastPageBatteryUpdateMs = millis();
        updatePageBatteryIndicator();
    }

    if (settingsWifiStateLabel && millis() - lastWifiUiUpdateMs >= 750) {
        lastWifiUiUpdateMs = millis();
        updateWifiManagerUi();
    }

    if (bleScanFinished) {
        bleScanFinished = false;

        if (c5DiscoveryInFlight) {
            const bool linkedNow = finishC5Discovery();

            // Tool pages wait for the one-time C5 discovery attempt before
            // starting their own scan cycle, so BLE discovery is not stolen
            // by Wi-Fi/BLE tool scans.
            if (currentPage == Page::ReconPulse &&
                pulsePhase == PulsePhase::Idle) {
                beginPulseSweep();
            } else if (currentPage == Page::Radar &&
                       radarPhase == RadarPhase::Idle) {
                beginRadarScan();
            } else if (currentPage == Page::WatchMode &&
                       watchPhase == WatchPhase::Idle) {
                beginWatchSweep();
            }

            if (currentPage == Page::WiFi) {
                if (linkedNow) {
                    // Populate 5 GHz immediately after an explicit C5 link.
                    requestC5Scan();
                }

                if (c5DiscoveryRequestedByWifi) {
                    c5DiscoveryRequestedByWifi = false;
                    wifiGraphLive = true;
                    wifiGraphNextScanMs = millis() + 250;
                    if (wifiStatusLabel) {
                        lv_label_set_text(
                            wifiStatusLabel,
                            linkedNow
                                ? "C5 FOUND  |  AUTHENTICATING"
                                : "C5 NOT FOUND  |  RESUMING 2.4"
                        );
                    }
                }
            }
        } else if (currentPage == Page::BLE && bleStatusLabel && bleList) {
            renderBleResults();

            if (bleScannerInitialized) {
                NimBLEDevice::getScan()->clearResults();
            }
        } else if (
            currentPage == Page::Recon &&
            reconPhase == ReconPhase::BleScanning
        ) {
            finishReconBleScan();
        } else if (
            currentPage == Page::Radar &&
            radarPhase == RadarPhase::BleScanning
        ) {
            finishRadarBleScan();
        } else if (
            currentPage == Page::SignalHunter &&
            hunterPhase == HunterPhase::BleSelecting &&
            hunterBleScanInFlight
        ) {
            hunterBleScanInFlight = false;

            // The currently selected button owns the result.
            // TRACKERS renders filtered Find My frames; BLE renders all BLE.
            if (hunterTrackerSelectionMode) {
                renderHunterTrackerTargets();
            } else {
                renderHunterBleTargets();
            }
        } else if (
            currentPage == Page::SignalHunter &&
            hunterPhase == HunterPhase::Tracking &&
            hunterKind == HunterKind::BLE &&
            hunterBleScanInFlight
        ) {
            finishHunterTrackingBleScan();
        } else if (
            currentPage == Page::WatchMode &&
            watchPhase == WatchPhase::BleScanning
        ) {
            finishWatchBleScan();
        } else if (
            currentPage == Page::Wardrive &&
            wardrivePhase == WardrivePhase::BleScanning
        ) {
            finishWardriveBleScan();
        } else if (bleScannerInitialized) {
            NimBLEDevice::getScan()->clearResults();
        }
    }

    // --------------------------------------------------------
    // Wardrive background engine fallback.
    //
    // Keep the Wardrive scheduler advancing directly from
    // loop() while the screen/backlight is dark.
    // --------------------------------------------------------
    if (screenSleeping &&
        currentPage == Page::Wardrive &&
        wardrivePageActive &&
        wardriveRunning) {

        static uint32_t wardriveBackgroundServiceMs = 0;
        const uint32_t now = millis();

        if (now - wardriveBackgroundServiceMs >= 150UL) {
            wardriveBackgroundServiceMs = now;
            wardriveTimerCb(nullptr);
        }
    }

    // --------------------------------------------------------
    // Temporary Wardrive lifecycle diagnostics.
    // No state is modified here.
    // --------------------------------------------------------
    if (currentPage == Page::Wardrive ||
        wardriveRunning ||
        wardriveExitToReconPending) {

        static uint32_t wardriveDiagPrintMs = 0;
        const uint32_t diagNow = millis();

        if (diagNow - wardriveDiagPrintMs >= 5000UL) {
            wardriveDiagPrintMs = diagNow;

            const int16_t wifiScanState =
                WiFi.scanComplete();

            bool bleScanningNow = false;

            if (bleScannerInitialized) {
                NimBLEScan *scan =
                    NimBLEDevice::getScan();

                if (scan) {
                    bleScanningNow =
                        scan->isScanning();
                }
            }

            Serial.printf(
                "[M33K WARD DIAG] "
                "page=%d run=%d active=%d sleep=%d "
                "phase=%d wifiState=%d wifi=%lu ble=%lu "
                "bleScan=%d "
                "sd=%d sdReady=%d rows=%lu "
                "c5Linked=%d c5Transport=%d "
                "c5Discover=%d c5Scan=%d c5Cycle=%d "
                "exit=%d\n",
                static_cast<int>(currentPage),
                wardriveRunning ? 1 : 0,
                wardrivePageActive ? 1 : 0,
                screenSleeping ? 1 : 0,
                static_cast<int>(wardrivePhase),
                static_cast<int>(wifiScanState),
                (unsigned long)wardriveTotalWifiCount,
                (unsigned long)wardriveTotalBleCount,
                bleScanningNow ? 1 : 0,
                wardriveSdLogging ? 1 : 0,
                m33kSdReady ? 1 : 0,
                (unsigned long)wardriveSdRowsWritten,
                c5IsLinked() ? 1 : 0,
                c5TransportConnected() ? 1 : 0,
                c5DiscoveryInFlight ? 1 : 0,
                c5ScanInFlight ? 1 : 0,
                wardriveC5CycleActive ? 1 : 0,
                wardriveExitToReconPending ? 1 : 0
            );
        }
    }

    // Touch resets the 30-second inactivity timer.
    // When the display is asleep, the first touch wakes it but is suppressed
    // from opening a button underneath the finger.
    if (instance.getTouched()) {
        if (screenSleeping) {
            wakeScreen();
        } else {
            noteActivity();
        }
    }

    if (!screenSleeping && (millis() - lastActivityMs >= SCREEN_TIMEOUT_MS)) {
        sleepScreen();
    }

    // Deep-idle GNSS shed: once the watch has been idle well past the screen
    // timeout and is not in a background Wardrive capture, power the GNSS rail
    // down to save battery. wakeScreen() restores it on the next wake.
    //
    // BUT never shed while the user is on a GPS-consuming page: the screen
    // sleeps after ~30 s on a drive, and cutting GPS power 3 min later meant
    // the module never stayed on long enough to acquire a fix (every wake was
    // a cold restart). Keep it powered so it can lock and hold while tracking.
    const bool onGpsPage =
        (currentPage == Page::GPS || currentPage == Page::Map ||
         currentPage == Page::Recon || currentPage == Page::Wardrive);
    if (screenSleeping && gpsToolActive && !gpsDeepIdleOff && !wardriveRunning &&
        !onGpsPage &&
        (millis() - lastActivityMs >= GPS_DEEP_IDLE_MS)) {
        instance.powerControl(POWER_GPS, false);
        gpsToolActive = false;
        gpsDeepIdleOff = true;
        Serial.println("[M33K POWER] Deep idle: GNSS rail powered down");
    }

    lv_timer_handler();

    // Home face swipe-to-switch (reads the touch indev directly).
    serviceFaceSwipe();

    // Weather face: run a pending/periodic fetch off the LVGL task.
    serviceWeatherFetch();

    // Match LilyGO's official NFC Reader loop cadence: one RFAL worker call
    // before LVGL, then a 5 ms loop delay. Avoid double-driving the state machine.
    delay(5);
}
