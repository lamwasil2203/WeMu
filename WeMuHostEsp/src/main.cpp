// =============================================================================
// WeMu Host ESP32 — UWB distance receiver + on-screen display
//
// Receives DistanceMsg packets via ESP-NOW from two paired client ESP32s and
// renders the two distances on the TTGO T-Display's 240x135 ST7789 LCD.
//
// All tunable knobs come from platformio.ini build_flags. The #ifndef defaults
// below exist only so the file still compiles in isolation.
// =============================================================================

// -----------------------------------------------------------------------------
// 1) Includes
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <TFT_eSPI.h>
#include <math.h>

// -----------------------------------------------------------------------------
// 2) Config defaults (overridden by -D flags in platformio.ini)
// -----------------------------------------------------------------------------
#ifndef FAKE_DATA
#define FAKE_DATA 0
#endif

#ifndef DISPLAY_REFRESH_MS
#define DISPLAY_REFRESH_MS 50
#endif

#ifndef STALE_THRESHOLD_MS
#define STALE_THRESHOLD_MS 1000
#endif

#ifndef CLIENT_1_ID
#define CLIENT_1_ID 1
#endif

#ifndef CLIENT_2_ID
#define CLIENT_2_ID 2
#endif

#ifndef FAKE_C1_PERIOD_MS
#define FAKE_C1_PERIOD_MS 4000
#endif

#ifndef FAKE_C2_PERIOD_MS
#define FAKE_C2_PERIOD_MS 3000
#endif

#ifndef UART_FWD_TX_PIN
#define UART_FWD_TX_PIN 17
#endif

#ifndef UART_FWD_BAUD
#define UART_FWD_BAUD 38400
#endif

#ifndef UART_FWD_PORT_NUM
#define UART_FWD_PORT_NUM 2
#endif

// Fake-packet injection cadence (ms). Not exposed to platformio.ini because
// 100 ms = 10 Hz is plenty fast for visual smoke testing.
static const uint32_t FAKE_PACKET_INTERVAL_MS = 100;

// -----------------------------------------------------------------------------
// 3) Wire format — must match the clients exactly
// -----------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
    uint8_t  client_id;     // 1 or 2
    uint16_t distance_cm;   // 0..65535 cm
} DistanceMsg;

// -----------------------------------------------------------------------------
// 4) Globals
// -----------------------------------------------------------------------------
TFT_eSPI tft = TFT_eSPI();

// Hardware UART used to forward distances to the Arduino (WeMuAudio).
// Port number comes from platformio.ini; pins set in setupUartForward().
HardwareSerial UartFwd(UART_FWD_PORT_NUM);

// State updated by the ESP-NOW callback (or fake-data injector).
// volatile because the callback runs on a different FreeRTOS task than loop().
volatile uint16_t      client1_distance_cm    = 0;
volatile uint16_t      client2_distance_cm    = 0;
volatile unsigned long client1_last_update_ms = 0;
volatile unsigned long client2_last_update_ms = 0;
volatile uint32_t      client1_packet_count   = 0;
volatile uint32_t      client2_packet_count   = 0;

// New-packet flags so loop() can log packets without doing it from the ISR-like
// callback. Cleared once handled.
volatile bool client1_new_packet = false;
volatile bool client2_new_packet = false;

// Display-redraw pacing.
static unsigned long last_redraw_ms = 0;

// Fake-data pacing (only used when FAKE_DATA == 1).
static unsigned long last_fake_inject_ms = 0;

// Cached values for the previous frame so we only repaint regions that changed.
// -1 sentinels force a full first paint.
static int  prev_c1_dist        = -1;
static int  prev_c2_dist        = -1;
static long prev_c1_age_ms      = -1;
static long prev_c2_age_ms      = -1;
static long prev_c1_pkts        = -1;
static long prev_c2_pkts        = -1;
static bool prev_c1_stale       = false;
static bool prev_c2_stale       = false;
static bool prev_c1_ever        = false;
static bool prev_c2_ever        = false;
static bool first_paint         = true;

// -----------------------------------------------------------------------------
// 5) ESP-NOW callback signature compatibility
//
// Arduino-ESP32 core 3.0+ uses (const esp_now_recv_info_t*, ...).
// Older 2.x cores use (const uint8_t *mac, ...). Pick based on core major.
// -----------------------------------------------------------------------------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define WEMU_ESPNOW_NEW_CB 1
#else
  #define WEMU_ESPNOW_NEW_CB 0
#endif

// -----------------------------------------------------------------------------
// 6) Forward declarations
// -----------------------------------------------------------------------------
#if WEMU_ESPNOW_NEW_CB
void onDataReceived(const esp_now_recv_info_t *recv_info,
                    const uint8_t *data, int len);
#else
void onDataReceived(const uint8_t *mac, const uint8_t *data, int len);
#endif

void printConfiguration();
void setupDisplay();
void setupEspNow();
void setupUartForward();
void drawClientPanel(int client_id, int y_start);
void redrawIfNeeded();
void injectFakePacketsIfEnabled();
void logNewPackets(bool c1_new, bool c2_new);
void forwardSnapshotIfNew(bool c1_new, bool c2_new);

// -----------------------------------------------------------------------------
// 7) ESP-NOW receive callback
//
// Keep this minimal: validate length, memcpy, update state, set new-packet
// flag. No Serial / TFT here — those happen in loop().
// -----------------------------------------------------------------------------
#if WEMU_ESPNOW_NEW_CB
void onDataReceived(const esp_now_recv_info_t * /*recv_info*/,
                    const uint8_t *data, int len) {
#else
void onDataReceived(const uint8_t * /*mac*/,
                    const uint8_t *data, int len) {
#endif
    if (len != (int)sizeof(DistanceMsg)) return;

    DistanceMsg msg;
    memcpy(&msg, data, sizeof(msg));

    const unsigned long now = millis();

    if (msg.client_id == CLIENT_1_ID) {
        client1_distance_cm    = msg.distance_cm;
        client1_last_update_ms = now;
        client1_packet_count++;
        client1_new_packet     = true;
    } else if (msg.client_id == CLIENT_2_ID) {
        client2_distance_cm    = msg.distance_cm;
        client2_last_update_ms = now;
        client2_packet_count++;
        client2_new_packet     = true;
    }
    // Unknown client_ids are silently ignored.
}

// -----------------------------------------------------------------------------
// 8) printConfiguration — echoes resolved -D flags so I can verify the build
// -----------------------------------------------------------------------------
void printConfiguration() {
    Serial.println();
    Serial.println(F("=== WeMu Host Configuration ==="));
    Serial.printf("FAKE_DATA          = %d\n", (int)FAKE_DATA);
    Serial.printf("DISPLAY_REFRESH_MS = %d\n", (int)DISPLAY_REFRESH_MS);
    Serial.printf("STALE_THRESHOLD_MS = %d\n", (int)STALE_THRESHOLD_MS);
    Serial.printf("CLIENT_1_ID        = %d\n", (int)CLIENT_1_ID);
    Serial.printf("CLIENT_2_ID        = %d\n", (int)CLIENT_2_ID);
    Serial.printf("FAKE_C1_PERIOD_MS  = %d\n", (int)FAKE_C1_PERIOD_MS);
    Serial.printf("FAKE_C2_PERIOD_MS  = %d\n", (int)FAKE_C2_PERIOD_MS);
    Serial.printf("UART_FWD_TX_PIN    = %d\n", (int)UART_FWD_TX_PIN);
    Serial.printf("UART_FWD_BAUD      = %d\n", (int)UART_FWD_BAUD);
    Serial.printf("UART_FWD_PORT_NUM  = %d\n", (int)UART_FWD_PORT_NUM);
    Serial.println(F("================================"));
}

// -----------------------------------------------------------------------------
// 9) setupDisplay — TFT init + splash screen
// -----------------------------------------------------------------------------
void setupDisplay() {
    tft.init();
    tft.setRotation(1); // landscape, 240x135
    tft.fillScreen(TFT_BLACK);

    // Splash
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("WeMu Host", 120, 50, 4);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.drawString("Waiting for clients...", 120, 85, 2);
    tft.setTextDatum(TL_DATUM); // restore default
}

// -----------------------------------------------------------------------------
// 10) setupEspNow — Wi-Fi STA, esp_now_init, register cb, print MAC
// -----------------------------------------------------------------------------
void setupEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    Serial.printf("Host MAC: %s\n", WiFi.macAddress().c_str());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        // Halt visibly: leave the splash up and stop. Unrecoverable without
        // code changes, so blocking forever is appropriate.
        while (true) {
            delay(1000);
        }
    }

    esp_now_register_recv_cb(onDataReceived);
    Serial.println("ESP-NOW receiver ready.");
}

// -----------------------------------------------------------------------------
// 10a) setupUartForward — open the hardware UART that talks to the Arduino
// -----------------------------------------------------------------------------
void setupUartForward() {
    // RX=-1 because we don't read anything back from the Arduino.
    UartFwd.begin(UART_FWD_BAUD, SERIAL_8N1, /*rx=*/-1, /*tx=*/UART_FWD_TX_PIN);
    Serial.printf("UART forward: TX=GPIO%d @ %d baud (UART%d)\n",
                  (int)UART_FWD_TX_PIN, (int)UART_FWD_BAUD, (int)UART_FWD_PORT_NUM);
}

// -----------------------------------------------------------------------------
// 10) drawClientPanel — render one half (top = client 1, bottom = client 2)
//
// Per-half layout (each half is 67 px tall):
//   y_start +  0  : header row ("CLIENT N" + optional "[STALE]")
//   y_start + 18  : large distance value, horizontally centered
//   y_start + 50  : stats line ("pkts: N   last: X ms ago")
//
// We clear only the rectangles we draw into, to avoid full-screen flicker.
// -----------------------------------------------------------------------------
void drawClientPanel(int client_id, int y_start) {
    // Snapshot the volatile state once so the rendering is self-consistent.
    uint16_t      dist;
    unsigned long last_ms;
    uint32_t      pkts;
    if (client_id == 1) {
        dist    = client1_distance_cm;
        last_ms = client1_last_update_ms;
        pkts    = client1_packet_count;
    } else {
        dist    = client2_distance_cm;
        last_ms = client2_last_update_ms;
        pkts    = client2_packet_count;
    }

    const unsigned long now    = millis();
    const bool ever_received   = (pkts > 0);
    const long age_ms          = ever_received ? (long)(now - last_ms) : -1;
    const bool is_stale        = ever_received && (age_ms > (long)STALE_THRESHOLD_MS);

    // Pull the previous-frame cache for this client.
    int  &prev_dist  = (client_id == 1) ? prev_c1_dist   : prev_c2_dist;
    long &prev_age   = (client_id == 1) ? prev_c1_age_ms : prev_c2_age_ms;
    long &prev_pkts  = (client_id == 1) ? prev_c1_pkts   : prev_c2_pkts;
    bool &prev_stale = (client_id == 1) ? prev_c1_stale  : prev_c2_stale;
    bool &prev_ever  = (client_id == 1) ? prev_c1_ever   : prev_c2_ever;

    // ---- Header row ("CLIENT N" + optional "[STALE]") -----------------------
    if (first_paint || is_stale != prev_stale || ever_received != prev_ever) {
        tft.fillRect(0, y_start, 240, 16, TFT_BLACK);
        tft.setTextDatum(TL_DATUM);
        tft.setTextColor(is_stale ? TFT_RED : TFT_GREEN, TFT_BLACK);
        char header[32];
        if (is_stale) {
            snprintf(header, sizeof(header), "CLIENT %d   [STALE]", client_id);
        } else {
            snprintf(header, sizeof(header), "CLIENT %d", client_id);
        }
        tft.drawString(header, 4, y_start + 2, 2);
    }

    // ---- Distance value (large, centered) -----------------------------------
    if (first_paint || (int)dist != prev_dist || is_stale != prev_stale ||
        ever_received != prev_ever) {
        tft.fillRect(0, y_start + 18, 240, 32, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(is_stale ? TFT_DARKGREY : TFT_WHITE, TFT_BLACK);
        char value[16];
        if (!ever_received) {
            snprintf(value, sizeof(value), "-- cm");
        } else {
            snprintf(value, sizeof(value), "%3d cm", (int)dist);
        }
        tft.drawString(value, 120, y_start + 33, 6);
    }

    // ---- Stats line ---------------------------------------------------------
    if (first_paint || age_ms != prev_age || (long)pkts != prev_pkts ||
        ever_received != prev_ever) {
        tft.fillRect(0, y_start + 50, 240, 16, TFT_BLACK);
        tft.setTextDatum(TL_DATUM);
        tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        char stats[40];
        if (!ever_received) {
            snprintf(stats, sizeof(stats), "pkts: 0   last: never");
        } else {
            snprintf(stats, sizeof(stats), "pkts: %lu   last: %ld ms ago",
                     (unsigned long)pkts, age_ms);
        }
        tft.drawString(stats, 4, y_start + 51, 2);
    }

    // Update the cache for next frame.
    prev_dist  = (int)dist;
    prev_age   = age_ms;
    prev_pkts  = (long)pkts;
    prev_stale = is_stale;
    prev_ever  = ever_received;
}

// -----------------------------------------------------------------------------
// 11) redrawIfNeeded — rate-limited repaint of both panels
// -----------------------------------------------------------------------------
void redrawIfNeeded() {
    const unsigned long now = millis();
    if ((now - last_redraw_ms) < (unsigned long)DISPLAY_REFRESH_MS) return;
    last_redraw_ms = now;

    if (first_paint) {
        // Wipe the splash before the first real frame.
        tft.fillScreen(TFT_BLACK);
    }

    // Top half: y = 0..66, divider at y = 67, bottom half: y = 68..134.
    drawClientPanel(1, 0);
    tft.drawFastHLine(0, 67, 240, TFT_DARKGREY);
    drawClientPanel(2, 68);

    first_paint = false;
}

// -----------------------------------------------------------------------------
// 12) injectFakePacketsIfEnabled — runtime gated on FAKE_DATA define
//
// Runtime `if` (not #if) so both branches stay compilable. The compiler folds
// the dead branch away when FAKE_DATA == 0.
// -----------------------------------------------------------------------------
void injectFakePacketsIfEnabled() {
    if (FAKE_DATA == 0) return;

    const unsigned long now = millis();
    if ((now - last_fake_inject_ms) < FAKE_PACKET_INTERVAL_MS) return;
    last_fake_inject_ms = now;

    const float t = (float)now;
    const float two_pi = 6.28318530718f;

    // Client 1: sin → [20, 200] cm with period FAKE_C1_PERIOD_MS.
    const float s1 = sinf(two_pi * t / (float)FAKE_C1_PERIOD_MS); // [-1, 1]
    const uint16_t d1 = (uint16_t)(20.0f + (s1 * 0.5f + 0.5f) * (200.0f - 20.0f));

    // Client 2: cos → [30, 150] cm with period FAKE_C2_PERIOD_MS.
    const float c2 = cosf(two_pi * t / (float)FAKE_C2_PERIOD_MS); // [-1, 1]
    const uint16_t d2 = (uint16_t)(30.0f + (c2 * 0.5f + 0.5f) * (150.0f - 30.0f));

    // Update the same state vars the real callback would touch — keeps the
    // rendering path identical between fake and real modes.
    client1_distance_cm    = d1;
    client1_last_update_ms = now;
    client1_packet_count++;
    client1_new_packet     = true;

    client2_distance_cm    = d2;
    client2_last_update_ms = now;
    client2_packet_count++;
    client2_new_packet     = true;
}

// -----------------------------------------------------------------------------
// 13) logNewPackets — USB-serial debug log for new packets
//
// The new-packet flags are snapshotted and cleared in loop() so that both this
// logger and forwardSnapshotIfNew() observe the same edge.
// -----------------------------------------------------------------------------
void logNewPackets(bool c1_new, bool c2_new) {
    if (c1_new) {
        Serial.printf("[%lu] client=1  dist=%4d cm  (count=%lu)\n",
                      (unsigned long)millis(),
                      (int)client1_distance_cm,
                      (unsigned long)client1_packet_count);
    }
    if (c2_new) {
        Serial.printf("[%lu] client=2  dist=%4d cm  (count=%lu)\n",
                      (unsigned long)millis(),
                      (int)client2_distance_cm,
                      (unsigned long)client2_packet_count);
    }
}

// -----------------------------------------------------------------------------
// 14) forwardSnapshotIfNew — emit a snapshot line over UART for the Arduino
//
// Triggered by any new ESP-NOW packet (or fake-data tick). Always emits a
// snapshot of BOTH clients, so the Arduino sees one self-contained record
// per line. A stale client is reported as "--".
//
// Wire format:   "D1:<int|--> D2:<int|-->\n"
// Examples:      "D1:142 D2:67\n"
//                "D1:-- D2:67\n"
// -----------------------------------------------------------------------------
void forwardSnapshotIfNew(bool c1_new, bool c2_new) {
    if (!c1_new && !c2_new) return;

    // Snapshot state for self-consistent rendering.
    const uint16_t      d1 = client1_distance_cm;
    const uint16_t      d2 = client2_distance_cm;
    const uint32_t      p1 = client1_packet_count;
    const uint32_t      p2 = client2_packet_count;
    const unsigned long t1 = client1_last_update_ms;
    const unsigned long t2 = client2_last_update_ms;

    const unsigned long now = millis();
    const bool c1_stale = (p1 == 0) || ((now - t1) > (unsigned long)STALE_THRESHOLD_MS);
    const bool c2_stale = (p2 == 0) || ((now - t2) > (unsigned long)STALE_THRESHOLD_MS);

    char line[32];
    int n;
    if (c1_stale && c2_stale) {
        n = snprintf(line, sizeof(line), "D1:-- D2:--\n");
    } else if (c1_stale) {
        n = snprintf(line, sizeof(line), "D1:-- D2:%u\n", (unsigned)d2);
    } else if (c2_stale) {
        n = snprintf(line, sizeof(line), "D1:%u D2:--\n", (unsigned)d1);
    } else {
        n = snprintf(line, sizeof(line), "D1:%u D2:%u\n",
                     (unsigned)d1, (unsigned)d2);
    }
    if (n > 0) {
        UartFwd.write(reinterpret_cast<const uint8_t *>(line), (size_t)n);
    }
}

// -----------------------------------------------------------------------------
// 15) setup
// -----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    // Brief pause so the serial console can attach before the boot log.
    delay(200);

    printConfiguration();
    setupDisplay();
    setupEspNow();
    setupUartForward();

    // Leave the splash up briefly so it's actually visible.
    delay(750);
    last_redraw_ms = 0; // force redraw on first loop()
}

// -----------------------------------------------------------------------------
// 16) loop
//
// Snapshot+clear the new-packet flags once per iteration so the USB-serial
// logger and the UART forwarder both act on the same edge.
// -----------------------------------------------------------------------------
void loop() {
    injectFakePacketsIfEnabled();

    bool c1_new = client1_new_packet; client1_new_packet = false;
    bool c2_new = client2_new_packet; client2_new_packet = false;

    logNewPackets(c1_new, c2_new);
    forwardSnapshotIfNew(c1_new, c2_new);
    redrawIfNeeded();
}
