#include <Arduino.h>

// Recon/Pulse/Radar process Wi-Fi, BLE and LVGL results on Arduino's loopTask.
// The ESP32 Arduino default loop stack is 8 KB; measured crashes on hardware
// hit the loopTask stack canary after the first Recon Pulse/Radar result pass.
// Give the main task more headroom for those combined scan/render paths.
SET_LOOP_TASK_STACK_SIZE(16 * 1024);

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <SD.h>
#include <time.h>
#include <TinyGPSPlus.h>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "mbedtls/md.h"
#include <LilyGoLib.h>
#include <LV_Helper.h>
#include <lvgl.h>

#include "m33k_peek_art.h"
#include "bunny_skull_small.h"
#include "bunny_skull_graph.h"
#include "wifi_bunny_bg.h"
#include "ble_bunny_bg.h"
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

constexpr uint32_t SCREEN_TIMEOUT_MS = 30000;

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
    NFC,
    Logs,
    Settings
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
    {"Settings", LV_SYMBOL_SETTINGS,  0xF4F6FF, Page::Settings},
};

constexpr uint8_t MATRIX_ENTITY_COUNT = 14;
MatrixEntity gMatrix[MATRIX_ENTITY_COUNT];

Page currentPage = Page::Home;
lv_obj_t *screen = nullptr;

lv_timer_t *matrixTimer = nullptr;
lv_timer_t *homeLiveTimer = nullptr;
lv_timer_t *wifiTimer = nullptr;
lv_timer_t *gpsTimer = nullptr;
lv_timer_t *reconTimer = nullptr;
lv_timer_t *watchTimer = nullptr;
lv_timer_t *radioTimer = nullptr;
lv_timer_t *nfcTimer = nullptr;

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
bool wardriveStopCleanupPending = false;

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

// Independent session-total tracker. This does not control Wardrive rows,
// scrolling, record cards, scan caches, or SD logging.
constexpr uint16_t WARD_TOTAL_WIFI_MAX = 192;
uint64_t wardriveTotalWifiHashes[WARD_TOTAL_WIFI_MAX] = {};
uint16_t wardriveTotalWifiCount = 0;
bool wardriveC5CacheApplied = false;
uint32_t wardriveAppliedC5Generation = 0;
String wardriveBleAddrs[WARD_MAX_BLE];
uint8_t wardriveBleCount = 0;
constexpr uint8_t WARD_MAX_FEED_LINES = 40;
String wardriveSweepLines[WARD_MAX_FEED_LINES];
uint8_t wardriveSweepLineCount = 0;
bool wardriveFeedDirty = true;
constexpr uint8_t WARD_MAX_RECORDS = 40;
WardriveRecord wardriveRecords[WARD_MAX_RECORDS];
uint8_t wardriveRecordCount = 0;
bool wardriveFeedScrolling = false;
bool wardriveUiSyncPending = false;
uint32_t wardriveDetailIgnoreUntilMs = 0;
bool wardriveDetailOpen = false;
uint8_t wardriveUiSyncCursor = 0;
uint32_t wardriveLastDiagMs = 0;
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
constexpr uint32_t C5_DISCOVERY_DURATION_MS = 900;
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
void stopWardriveTool();
void stopRadioTool();
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

void sleepScreen()
{
    if (screenSleeping) return;

    screenSleeping = true;
    if (matrixTimer) lv_timer_pause(matrixTimer);
    if (homeLiveTimer) lv_timer_pause(homeLiveTimer);

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
        const uint32_t colorHex = random(0, 2) ? 0x3AEFFF : 0xFF4AD5;
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
    ent.colorHex = random(0, 2) ? 0x3AEFFF : 0xFF4AD5;
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

        if (gMatrix[i].y > DOCK_Y - 10) {
            resetEntity(gMatrix[i], false);
        } else {
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

                const bool hadRemoteData =
                    c5WifiDetailCount > 0 || c5WifiPendingDetailCount > 0;
                c5WifiDetailCount = 0;
                c5WifiPendingDetailCount = 0;
                if (hadRemoteData) ++c5ScanGeneration;

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
        Serial.println("[M33K C5] M33K X C5 not found (async)");
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

    for (uint16_t i = 0; i < wardriveTotalWifiCount; ++i) {
        if (wardriveTotalWifiHashes[i] == hash) return;
    }

    if (wardriveTotalWifiCount < WARD_TOTAL_WIFI_MAX) {
        wardriveTotalWifiHashes[wardriveTotalWifiCount++] = hash;
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
    for (uint8_t i = 0; i < wardriveRecordCount; ++i) {
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
    if (wardriveFeedScrolling || wardriveDetailOpen) {
        wardriveUiSyncPending = true;
        wardriveUiSyncCursor = 0;
        return;
    }
    if (!rec.label) return;
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
    if (wardriveFeedScrolling || wardriveDetailOpen) {
        wardriveUiSyncPending = true;
        wardriveUiSyncCursor = 0;
        return;
    }
    if (!wardriveFeedScroll || rec.row) return;

    rec.row = lv_button_create(wardriveFeedScroll);
    lv_obj_set_width(rec.row, 204);
    lv_obj_set_height(rec.row, 48);
    lv_obj_set_style_radius(rec.row, 10, 0);
    lv_obj_set_style_bg_color(
        rec.row,
        lv_color_hex(rec.wifi ? (rec.fromC5 ? 0x08112A : 0x071C29) : 0x1A0B1B),
        0
    );
    lv_obj_set_style_bg_opa(rec.row, 220, 0);
    lv_obj_set_style_border_width(rec.row, 1, 0);
    lv_obj_set_style_border_color(
        rec.row,
        lv_color_hex(rec.wifi ? (rec.fromC5 ? 0x2F6BFF : 0x55EEFF) : 0xFF52C8),
        0
    );
    lv_obj_set_style_pad_all(rec.row, 4, 0);
    lv_obj_add_event_cb(rec.row, wardriveDetailEvent, LV_EVENT_CLICKED, &rec);

    rec.label = lv_label_create(rec.row);
    lv_obj_set_width(rec.label, 194);
    lv_label_set_long_mode(rec.label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_color(rec.label, lv_color_hex(0xF4F6FF), 0);
    lv_obj_set_style_text_font(rec.label, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(rec.label, 2, 2);
    updateWardriveRecordLabel(rec);
}

void syncWardriveRecordRows()
{
    if (wardriveFeedScrolling || wardriveDetailOpen || !wardriveFeedScroll) return;

    constexpr uint8_t WARD_UI_ROWS_PER_TICK = 3;

    if (wardriveUiSyncCursor >= wardriveRecordCount) {
        wardriveUiSyncCursor = 0;
    }

    uint8_t processed = 0;

    while (wardriveUiSyncCursor < wardriveRecordCount &&
           processed < WARD_UI_ROWS_PER_TICK) {

        WardriveRecord &rec =
            wardriveRecords[wardriveUiSyncCursor++];

        ++processed;

        if (!rec.active) continue;

        if (!rec.row) {
            createWardriveRecordRow(rec);
        } else if (rec.label) {
            updateWardriveRecordLabel(rec);
        }
    }

    if (wardriveUiSyncCursor >= wardriveRecordCount) {
        wardriveUiSyncCursor = 0;
        wardriveUiSyncPending = false;
    }
}

void wardriveFeedScrollEvent(lv_event_t *e)
{
    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_SCROLL_BEGIN) {
        wardriveFeedScrolling = true;
        wardriveDetailIgnoreUntilMs = millis() + 500;
        noteActivity();
        return;
    }
    if (code == LV_EVENT_SCROLL_END) {
        wardriveFeedScrolling = false;
        wardriveDetailIgnoreUntilMs = millis() + 500;
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
    if (wardriveRecordCount >= WARD_MAX_RECORDS) return nullptr;
    if (ESP.getFreeHeap() < 45000U) {
        wardriveUiSyncPending = true;
        return nullptr;
    }
    if (wardriveRecordCount == 0 && wardriveFeedLabel) {
        lv_obj_delete(wardriveFeedLabel);
        wardriveFeedLabel = nullptr;
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
    wardriveUiSyncPending = true;
    wardriveUiSyncCursor = 0;
    return &rec;
}

void wardriveDetailCloseEvent(lv_event_t *e)
{
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
    if (lv_event_get_code(e) != LV_EVENT_CLICKED || !clickAllowed()) return;
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
    wardriveRecordCount = 0;
    for (auto &rec : wardriveRecords) rec = WardriveRecord{};
    if (wardriveFeedScroll) {
        lv_obj_clean(wardriveFeedScroll);
        wardriveFeedLabel = lv_label_create(wardriveFeedScroll);
        lv_label_set_text(wardriveFeedLabel, "Scanning results will stay here.\nTap a device for details.");
        lv_obj_set_width(wardriveFeedLabel, 200);
        lv_label_set_long_mode(wardriveFeedLabel, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_color(wardriveFeedLabel, lv_color_hex(0xF4F6FF), 0);
        lv_obj_set_style_text_font(wardriveFeedLabel, &lv_font_montserrat_12, 0);
    }
}

void wardriveResetSweepFeed()
{
    wardriveSweepLineCount = 0;
    for (auto &line : wardriveSweepLines) line = "";
}

void wardriveAddSweepLine(const String &line)
{
    if (line.length() == 0) return;

    if (wardriveSweepLineCount >= WARD_MAX_FEED_LINES) {
        for (uint8_t i = 1; i < WARD_MAX_FEED_LINES; ++i) {
            wardriveSweepLines[i - 1] = wardriveSweepLines[i];
        }
        wardriveSweepLines[WARD_MAX_FEED_LINES - 1] = line;
        wardriveFeedDirty = true;
        return;
    }

    wardriveSweepLines[wardriveSweepLineCount++] = line;
    wardriveFeedDirty = true;
}

void updateWardriveFeed()
{
    if (!wardriveFeedLabel) return;

    if (wardriveSweepLineCount == 0) {
        lv_label_set_text(
            wardriveFeedLabel,
            wardriveRunning
                ? "Scanning... results will stay here and you can scroll them."
                : "Tap START to begin wardriving.\nResults will stay here and can be scrolled."
        );
        wardriveFeedDirty = false;
        return;
    }

    String combined;
    for (uint8_t i = 0; i < wardriveSweepLineCount; ++i) {
        combined += wardriveSweepLines[i];
        if (i + 1 < wardriveSweepLineCount) combined += "\n";
    }
    lv_label_set_text(wardriveFeedLabel, combined.c_str());
    wardriveFeedDirty = false;
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
        m33kSdReady = false;
        wardriveSdLogging = false;
        Serial.println("[M33K][SD] Wardrive append open failed; continuing without SD logging");
        return false;
    }

    char timestamp[24] = {};
    m33kRtcTimestampBuf(timestamp, sizeof(timestamp));

    uint32_t rows = 0;

    for (uint8_t i = 0; i < wardriveRecordCount; ++i) {
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
    if (currentPage != Page::Wardrive) return;

    if (wardriveWifiLabel) {
        lv_label_set_text_fmt(wardriveWifiLabel, "WI-FI\n%u", (unsigned)wardriveTotalWifiCount);
    }
    if (wardriveBleLabel) {
        lv_label_set_text_fmt(wardriveBleLabel, "BLE\n%u", (unsigned)wardriveBleCount);
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
    if (wardriveFeedDirty) {
        updateWardriveFeed();
    }
}

void stopWardriveTool()
{
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

    wardriveRunning = false;
    wardriveStopCleanupPending = false;
    stopWardriveSdLog();
    wardrivePhase = WardrivePhase::Stopped;
}

bool startWardriveBleScan()
{
    initBleScanner();
    NimBLEScan *scan = NimBLEDevice::getScan();
    if (scan->isScanning()) scan->stop();
    scan->clearResults();
    scan->setMaxResults(28);
    bleScanFinished = false;
    bleScanEndReason = 0;
    wardrivePhase = WardrivePhase::BleScanning;
    if (wardriveStatusLabel) lv_label_set_text(wardriveStatusLabel, "SESSION LIVE  |  SCANNING BLE...");
    const bool started = scan->start(2200, false, true);
    if (!started) scan->setMaxResults(MAX_BLE_DETAILS);
    return started;
}

void finishWardriveWifiScan()
{
    const int16_t result = WiFi.scanComplete();
    if (result == WIFI_SCAN_FAILED) {
        wardrivePhase = WardrivePhase::Failed;
        if (wardriveStatusLabel) lv_label_set_text(wardriveStatusLabel, "WARDRIVE WI-FI SCAN FAILED");
        WiFi.scanDelete();
        return;
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
        wardriveTrackWifiTotal(bssid);
        String key = ssid + "|" + bssid;
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
            updateWardriveRecordLabel(*rec);
        }
    }

    appendWardriveRecordGroupToSd(true, sdSeenSinceMs);

    if (applyFreshC5) {
        wardriveC5CacheApplied = true;
        wardriveAppliedC5Generation = c5ScanGeneration;
    }
    WiFi.scanDelete();
    updateWardriveUi();

    if (!startWardriveBleScan()) {
        wardrivePhase = WardrivePhase::Failed;
        if (wardriveStatusLabel) lv_label_set_text(wardriveStatusLabel, "WARDRIVE BLE SCAN COULD NOT START");
    }
}

void finishWardriveBleScan()
{
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
        if (!wardriveHasBle(addr) && wardriveBleCount < WARD_MAX_BLE) {
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
            updateWardriveRecordLabel(*rec);
        }
    }

    appendWardriveRecordGroupToSd(false, sdSeenSinceMs);

    scan->clearResults();
    scan->setMaxResults(MAX_BLE_DETAILS);
    wardrivePhase = WardrivePhase::Idle;
    wardriveNextSweepMs = millis() + 1800;
    if (wardriveStatusLabel) lv_label_set_text(wardriveStatusLabel, "SESSION LIVE  |  GPS + RADIO CAPTURE");
    updateWardriveUi();
}

void beginWardriveSweep()
{
    if (currentPage != Page::Wardrive || !wardriveRunning) return;
    requestC5Scan();

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
    if (currentPage != Page::Wardrive) return;
    serviceC5Link();

    if (!wardriveFeedScrolling && wardriveUiSyncPending) {
        syncWardriveRecordRows();
    }

    updateWardriveUi();
    if (!wardriveRunning) return;

    if (millis() - wardriveLastDiagMs >= 5000) {
        wardriveLastDiagMs = millis();

        Serial.printf(
            "[M33K WARD] heap=%u minHeap=%u stack=%u records=%u C5=%s\n",
            (unsigned)ESP.getFreeHeap(),
            (unsigned)ESP.getMinFreeHeap(),
            (unsigned)uxTaskGetStackHighWaterMark(nullptr),
            (unsigned)wardriveRecordCount,
            c5IsLinked() ? "LINKED" : "OFFLINE"
        );
    }

    // Allow a C5 powered on after Wardrive starts to join automatically.
    // Retry only during the quiet gap between Wardrive sweeps.
    if (wardrivePhase == WardrivePhase::Idle && !c5IsLinked()) {
        if (!c5TransportConnected() &&
            !c5DiscoveryInFlight &&
            millis() >= c5NextConnectAttemptMs) {

            if (startC5DiscoveryAsync()) {
                if (wardriveStatusLabel) {
                    lv_label_set_text(
                        wardriveStatusLabel,
                        "SESSION LIVE  |  SEARCHING FOR C5..."
                    );
                }
                return;
            }
        }

        if (c5DiscoveryInFlight || c5TransportConnected()) {
            return;
        }
    }

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
    wardriveBleCount = 0;
    wardriveDetailOpen = false;
    wardriveUiSyncCursor = 0;
    wardriveLastDiagMs = 0;
    wardriveC5CacheApplied = false;
    wardriveAppliedC5Generation = c5ScanGeneration;
    resetWardriveRecords();
    wardriveSessionStartedMs = millis();
    wardriveNextSweepMs = 0;
    wardriveLastElapsedSeconds = 0;
    startWardriveSdLog();
    wardriveRunning = true;
    wardrivePhase = WardrivePhase::Idle;

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
    beginWardriveSweep();
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
    wardriveStopCleanupPending = false;
    stopWardriveSdLog();

    Serial.printf(
        "[M33K WARD] STOP: heap=%u, minHeap=%u, stack=%u\n",
        (unsigned)ESP.getFreeHeap(),
        (unsigned)ESP.getMinFreeHeap(),
        (unsigned)uxTaskGetStackHighWaterMark(nullptr)
    );

    if (wardriveStartLabel) lv_label_set_text(wardriveStartLabel, "START");
    if (wardriveStatusLabel) {
        lv_label_set_text(
            wardriveStatusLabel,
            "SESSION STOPPED  |  RESULTS HELD ON SCREEN"
        );
    }
}

void wardriveStartEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED || !clickAllowed()) return;
    
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
    showWardrivePage();
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
        lv_label_set_text_fmt(
            radioProfileLabel,
            "SF %u\nBW %.0f\nCR 4/7",
            (unsigned)currentRadioSf(),
            currentRadioBw()
        );
    }

    if (radioSfButtonLabel) {
        lv_label_set_text_fmt(
            radioSfButtonLabel,
            "SF %u",
            (unsigned)currentRadioSf()
        );
    }

    if (radioBwButtonLabel) {
        lv_label_set_text_fmt(
            radioBwButtonLabel,
            "BW %.0f",
            currentRadioBw()
        );
    }

    if (radioAutoButtonLabel) {
        lv_label_set_text(
            radioAutoButtonLabel,
            m33kRadioAutoScan ? "AUTO ON" : "AUTO OFF"
        );
    }

    if (radioModeLabel) {
        lv_label_set_text(
            radioModeLabel,
            m33kRadioAutoScan ? "MODE\nAUTO RX" : "MODE\nRX ONLY"
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
        lv_label_set_text(
            radioPayloadLabel,
            "No matching packets yet.\n\n"
            "RX-only. AUTO cycles SF7-SF12 and "
            "125/250/500 kHz receive profiles."
        );
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

        state = radio.setFrequency(M33K_RADIO_FREQ_MHZ);

        if (state == RADIOLIB_ERR_NONE) {
            state = radio.setBandwidth(currentRadioBw());
        }

        if (state == RADIOLIB_ERR_NONE) {
            state = radio.setSpreadingFactor(currentRadioSf());
        }

        if (state == RADIOLIB_ERR_NONE) {
            state = radio.setCodingRate(7);
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
        uint8_t data[96] = {};
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

            String entry;
            entry += "#";
            entry += String(m33kRadioPacketCount);
            entry += "  SF";
            entry += String(currentRadioSf());
            entry += " BW";
            entry += String((int)currentRadioBw());
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

    bool receivedByte = false;
    while (Serial1.available() > 0) {
        const int c = Serial1.read();
        if (c < 0) break;

        receivedByte = true;
        gpsParser.encode(static_cast<char>(c));
        ++gpsSessionBytes;
        gpsLastByteMs = millis();

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
    if (radioTimer) {
        lv_timer_del(radioTimer);
        radioTimer = nullptr;
    }
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

    lv_obj_t *sep1 = lv_label_create(top);
    lv_label_set_text(sep1, "|");
    lv_obj_set_style_text_color(sep1, lv_color_hex(0x6B8299), 0);
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
    lv_obj_set_style_text_color(sep2, lv_color_hex(0x6B8299), 0);
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
    lv_obj_set_style_text_color(sep3, lv_color_hex(0x6B8299), 0);
    lv_obj_set_pos(sep3, 197, 7);

    lv_obj_t *ble = lv_label_create(top);
    lv_label_set_text(ble, LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_color(ble, lv_color_hex(0x6D86FF), 0);
    lv_obj_set_style_text_font(ble, &lv_font_montserrat_16, 0);
    lv_obj_set_pos(ble, 214, 6);

    lv_obj_t *sep4 = lv_label_create(top);
    lv_label_set_text(sep4, "|");
    lv_obj_set_style_text_color(sep4, lv_color_hex(0x6B8299), 0);
    lv_obj_set_pos(sep4, 246, 7);

    lv_obj_t *gps = lv_label_create(top);
    lv_label_set_text(gps, LV_SYMBOL_GPS);
    lv_obj_set_style_text_color(gps, lv_color_hex(0xF5FF3B), 0);
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

    if (currentPage == Page::ReconPulse ||
        currentPage == Page::Radar ||
        currentPage == Page::SignalHunter ||
        currentPage == Page::WatchMode ||
        currentPage == Page::Wardrive ||
        currentPage == Page::Radio) {

        showReconPage();
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
        showPlaceholder(Page::NFC, "NFC - IN PROGRESS");
    } else if (item->page == Page::Logs) {
        showLogsPage();
    } else if (item->page == Page::Settings) {
        showSettingsPage();
    } else {
        showPlaceholder(item->page, item->label);
    }
}

void createDock()
{
    // Larger hit areas than v0.2.x, while staying away from rounded corners.
    constexpr int16_t buttonW = 44;
    constexpr int16_t buttonH = 54;
    constexpr int16_t gap = 8;
    int16_t x = 27;

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
        lv_obj_set_style_text_color(icon, lv_color_hex(item.iconColorHex), 0);
        lv_obj_set_style_text_font(icon, &lv_font_montserrat_22, 0);
        lv_obj_center(icon);

        x += buttonW + gap;
    }
}

void showHome()
{
    clearScreen();
    currentPage = Page::Home;

    createMatrixBackground();
    createArt();
    createTopBar();
    createTimeDate();
    createDock();

    updateHomeLiveData();
    homeLiveTimer = lv_timer_create(homeLiveTimerCb, 1000, nullptr);

    noteActivity();
    Serial.println("[M33K] Home");
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
    showWifiPage();
}

void wifiGraphOpenEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!clickAllowed()) return;

    noteActivity();
    hapticTap();
    wifiSubpageTransition = true;
    showWifiGraphPage();
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


void showWifiPage()
{
    clearScreen();
    currentPage = Page::WiFi;
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
    lv_obj_set_style_bg_opa(glowOuter, 18, 0);

    lv_obj_t *glowMid = lv_obj_create(screen);
    lv_obj_remove_style_all(glowMid);
    lv_obj_set_size(glowMid, 240, 260);
    lv_obj_set_pos(glowMid, 165, 220);
    lv_obj_set_style_radius(glowMid, 130, 0);
    lv_obj_set_style_bg_color(glowMid, lv_color_hex(0x008CFF), 0);
    lv_obj_set_style_bg_opa(glowMid, 28, 0);

    lv_obj_t *glowCore = lv_obj_create(screen);
    lv_obj_remove_style_all(glowCore);
    lv_obj_set_size(glowCore, 190, 220);
    lv_obj_set_pos(glowCore, 212, 248);
    lv_obj_set_style_radius(glowCore, 110, 0);
    lv_obj_set_style_bg_color(glowCore, lv_color_hex(0x22C8FF), 0);
    lv_obj_set_style_bg_opa(glowCore, 38, 0);

    lv_obj_t *wifiArt = lv_image_create(screen);
    lv_image_set_src(wifiArt, &wifi_bunny_bg_img);
    lv_obj_set_pos(wifiArt, 224, 304);

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

    WiFi.mode(WIFI_STA);
    WiFi.scanDelete();

    int16_t count = WiFi.scanNetworks(false, true, true, 250);

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

    // Invisible spacer makes sure the scroll area extends past the brightness slider.
    lv_obj_t *spacer = lv_obj_create(settingsContent);
    lv_obj_remove_style_all(spacer);
    lv_obj_set_size(spacer, 1, 1);
    lv_obj_set_pos(spacer, 1, 780);

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


void showBlePage()
{
    clearScreen();
    currentPage = Page::BLE;

    lv_obj_t *bg = lv_image_create(screen);
    lv_image_set_src(bg, &ble_bunny_bg_img);
    lv_obj_set_pos(bg, 0, 0);

    lv_obj_t *shade = lv_obj_create(screen);
    lv_obj_remove_style_all(shade);
    lv_obj_set_size(shade, 410, 502);
    lv_obj_set_pos(shade, 0, 0);
    lv_obj_set_style_bg_color(shade, lv_color_hex(0x02050B), 0);
    lv_obj_set_style_bg_opa(shade, 55, 0);

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
    lv_obj_set_style_bg_color(bleList, lv_color_hex(0x030711), 0);
    lv_obj_set_style_bg_opa(bleList, 102, 0);
    lv_obj_set_style_border_width(bleList, 1, 0);
    lv_obj_set_style_border_color(bleList, lv_color_hex(0x213A64), 0);
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
    startToolC5Discovery();
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
    lv_obj_set_size(sessionCard, 350, 160);
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

    wardriveElapsedLabel = lv_label_create(sessionCard);
    lv_label_set_text(wardriveElapsedLabel, "SESSION  00:00");
    lv_obj_set_width(wardriveElapsedLabel, 326);
    lv_obj_set_style_text_align(wardriveElapsedLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(wardriveElapsedLabel, lv_color_hex(0xFF52C8), 0);
    lv_obj_set_style_text_font(wardriveElapsedLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_pos(wardriveElapsedLabel, 12, 8);

    wardriveFeedStateLabel = lv_label_create(sessionCard);
    lv_label_set_text(wardriveFeedStateLabel, "SCAN FEED");
    lv_obj_set_width(wardriveFeedStateLabel, 160);
    lv_obj_set_style_text_color(wardriveFeedStateLabel, lv_color_hex(0x55EEFF), 0);
    lv_obj_set_style_text_font(wardriveFeedStateLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(wardriveFeedStateLabel, 12, 32);

    wardriveFeedScroll = lv_obj_create(sessionCard);
    lv_obj_set_size(wardriveFeedScroll, 228, 100);
    lv_obj_set_pos(wardriveFeedScroll, 10, 50);
    lv_obj_set_style_radius(wardriveFeedScroll, 12, 0);
    lv_obj_set_style_bg_color(wardriveFeedScroll, lv_color_hex(0x030812), 0);
    lv_obj_set_style_bg_opa(wardriveFeedScroll, 175, 0);
    lv_obj_set_style_border_width(wardriveFeedScroll, 1, 0);
    lv_obj_set_style_border_color(wardriveFeedScroll, lv_color_hex(0x123252), 0);
    lv_obj_set_style_pad_all(wardriveFeedScroll, 6, 0);
    lv_obj_set_style_pad_row(wardriveFeedScroll, 5, 0);
    lv_obj_set_flex_flow(wardriveFeedScroll, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(wardriveFeedScroll, LV_DIR_VER);
    lv_obj_add_flag(wardriveFeedScroll, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(wardriveFeedScroll, LV_OBJ_FLAG_SCROLL_ELASTIC);
    lv_obj_clear_flag(wardriveFeedScroll, LV_OBJ_FLAG_SCROLL_CHAIN_VER);
    lv_obj_clear_flag(wardriveFeedScroll, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
    lv_obj_set_scrollbar_mode(wardriveFeedScroll, LV_SCROLLBAR_MODE_ACTIVE);
    lv_obj_add_event_cb(wardriveFeedScroll, wardriveFeedScrollEvent, LV_EVENT_ALL, nullptr);

    wardriveFeedLabel = lv_label_create(wardriveFeedScroll);
    lv_label_set_text(wardriveFeedLabel, "Tap START to begin wardriving.\nTap a device later for details.");
    lv_obj_set_width(wardriveFeedLabel, 208);
    lv_label_set_long_mode(wardriveFeedLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(wardriveFeedLabel, lv_color_hex(0xF4F6FF), 0);
    lv_obj_set_style_text_font(wardriveFeedLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(wardriveFeedLabel, 0, 0);

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
    lv_obj_set_size(startBtn, 180, 48);
    lv_obj_set_pos(startBtn, 115, 342);
    lv_obj_set_style_radius(startBtn, 16, 0);
    lv_obj_set_style_bg_color(startBtn, lv_color_hex(0x0B1A1C), 0);
    lv_obj_set_style_border_width(startBtn, 1, 0);
    lv_obj_set_style_border_color(startBtn, lv_color_hex(0x7CFF45), 0);
    lv_obj_add_event_cb(startBtn, wardriveStartEvent, LV_EVENT_CLICKED, nullptr);

    wardriveStartLabel = lv_label_create(startBtn);
    lv_label_set_text(wardriveStartLabel, "START");
    lv_obj_set_style_text_color(wardriveStartLabel, lv_color_hex(0x7CFF45), 0);
    lv_obj_set_style_text_font(wardriveStartLabel, &lv_font_montserrat_16, 0);
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
    lv_obj_set_size(sfBtn, 92, 34);
    lv_obj_set_pos(sfBtn, 12, 200);
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
    lv_obj_set_size(bwBtn, 104, 34);
    lv_obj_set_pos(bwBtn, 122, 200);
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
    lv_obj_set_size(autoBtn, 100, 34);
    lv_obj_set_pos(autoBtn, 238, 200);
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

void loop()
{
    instance.loop();

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

    lv_timer_handler();

    // Match LilyGO's official NFC Reader loop cadence: one RFAL worker call
    // before LVGL, then a 5 ms loop delay. Avoid double-driving the state machine.
    delay(5);
}
