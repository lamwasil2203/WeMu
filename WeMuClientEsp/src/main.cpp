// =============================================================================
// WeMu Client ESP32 — UWB distance relay
//
// Sits between a Qorvo DWM3001CDK ranging pair (controller + controlee, both
// powered by this ESP32 via Y-split 5V/GND) and the WeMu Host ESP32 over
// ESP-NOW.
//
// Reads "distance: <float> cm" lines emitted by the controller CDK on its Pi
// header UART (115200 8N1), filters to a sane range, packs the integer cm
// into a 3-byte DistanceMsg, and ships it to the host MAC over ESP-NOW.
//
// Build with one of two envs in platformio.ini:
//   pio run -e client1 -t upload   → CLIENT_ID=1, paired with CDKs on UWB ch 9
//   pio run -e client2 -t upload   → CLIENT_ID=2, paired with CDKs on UWB ch 5
//
// Wiring (per dancer unit):
//   Controller CDK Pi pin 4  (5V)  ───► ESP32 VIN   (or independent 5 V source)
//   Controller CDK Pi pin 6  (GND) ───► ESP32 GND   (common ground required)
//   Controller CDK Pi pin 8  (TX)  ───► ESP32 GPIO33  (UART2 RX, remapped)
//   Controller CDK Pi pin 10 (RX)  ─── NOT CONNECTED — we never send to the CDK
//   Controlee  CDK runs on its own power; no data wires.
// =============================================================================

// -----------------------------------------------------------------------------
// 1) Includes
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <HardwareSerial.h>

// -----------------------------------------------------------------------------
// 2) Config defaults (overridden by -D flags in platformio.ini)
// -----------------------------------------------------------------------------
#ifndef CLIENT_ID
#define CLIENT_ID 1
#endif

#ifndef UWB_CHANNEL
#define UWB_CHANNEL 9
#endif

#ifndef CTRL_RX_PIN
#define CTRL_RX_PIN 16
#endif
#ifndef CTRL_TX_PIN
#define CTRL_TX_PIN 17
#endif
#ifndef CTRL_UART_BAUD
#define CTRL_UART_BAUD 115200
#endif

#ifndef UART_DEBUG_ECHO
#define UART_DEBUG_ECHO 0
#endif

#ifndef DIST_MIN_CM
#define DIST_MIN_CM 10
#endif
#ifndef DIST_MAX_CM
#define DIST_MAX_CM 400
#endif

#ifndef HOST_MAC_0
#define HOST_MAC_0 0xA0
#endif
#ifndef HOST_MAC_1
#define HOST_MAC_1 0xDD
#endif
#ifndef HOST_MAC_2
#define HOST_MAC_2 0x6C
#endif
#ifndef HOST_MAC_3
#define HOST_MAC_3 0x74
#endif
#ifndef HOST_MAC_4
#define HOST_MAC_4 0xD3
#endif
#ifndef HOST_MAC_5
#define HOST_MAC_5 0xC8
#endif

// -----------------------------------------------------------------------------
// 3) Wire format — MUST stay binary-identical with WeMuHostEsp/src/main.cpp.
//    The host length-checks against sizeof(DistanceMsg) and silently drops
//    packets that don't match.
// -----------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
    uint8_t  client_id;
    uint16_t distance_cm;
} DistanceMsg;

// -----------------------------------------------------------------------------
// 4) Globals
// -----------------------------------------------------------------------------
HardwareSerial CtrlSerial(2);  // UART2 — controller CDK

static uint8_t hostMac[6] = {
    HOST_MAC_0, HOST_MAC_1, HOST_MAC_2,
    HOST_MAC_3, HOST_MAC_4, HOST_MAC_5,
};

static String   lineBuf;
static uint32_t txCount      = 0;
static uint32_t sendFailures = 0;

// Max line length accepted from the CDK. A full SESSION_INFO_NTF measurement
// line with AoA + RSSI fields runs ~250 chars; 512 leaves headroom.
static const int UART_LINE_MAX = 512;

// -----------------------------------------------------------------------------
// 5) Forward declarations
// -----------------------------------------------------------------------------
void onSent(const uint8_t *mac, esp_now_send_status_t status);
void printConfiguration();
void setupEspNow();
void handleLine(const String &line);

// -----------------------------------------------------------------------------
// 6) ESP-NOW send callback — rate-limited failure log so a wrong host MAC
//    doesn't spam the serial monitor.
// -----------------------------------------------------------------------------
void onSent(const uint8_t * /*mac*/, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) return;
    sendFailures++;
    if (sendFailures == 1 || sendFailures % 50 == 0) {
        Serial.printf("[client %d] send failed (count=%lu) — host MAC correct?\n",
                      (int)CLIENT_ID, (unsigned long)sendFailures);
    }
}

// -----------------------------------------------------------------------------
// 7) printConfiguration — boot banner mirroring WeMuHostEsp style.
// -----------------------------------------------------------------------------
void printConfiguration() {
    Serial.println();
    Serial.printf("=== WeMu Client %d ===\n", (int)CLIENT_ID);
    Serial.printf("CTRL UART:    GPIO%d (RX) / GPIO%d (TX) @ %d baud\n",
                  (int)CTRL_RX_PIN, (int)CTRL_TX_PIN, (int)CTRL_UART_BAUD);
    Serial.printf("UWB channel:  %d  (informational — set in CDK firmware)\n",
                  (int)UWB_CHANNEL);
    Serial.printf("Self MAC:     %s\n", WiFi.macAddress().c_str());
    Serial.printf("Host MAC:     %02X:%02X:%02X:%02X:%02X:%02X\n",
                  hostMac[0], hostMac[1], hostMac[2],
                  hostMac[3], hostMac[4], hostMac[5]);
    Serial.printf("Range filter: %d..%d cm\n",
                  (int)DIST_MIN_CM, (int)DIST_MAX_CM);
    Serial.println("=====================");
}

// -----------------------------------------------------------------------------
// 8) setupEspNow — Wi-Fi STA, esp_now_init, register cb, add host peer.
// -----------------------------------------------------------------------------
void setupEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("[client] ESP-NOW init failed — halting");
        while (true) delay(1000);
    }
    esp_now_register_send_cb(onSent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, hostMac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("[client] failed to add host peer — halting");
        while (true) delay(1000);
    }
    Serial.println("[client] ESP-NOW ready.");
}

// -----------------------------------------------------------------------------
// 9) handleLine — parse one complete UART line from the controller CDK.
//
// The Qorvo CLI firmware emits ranging notifications spanning multiple lines:
//
//   SESSION_INFO_NTF: {session_handle=2, sequence_number=14, ...
//    [mac_address=0x0001, status="Ok", distance[cm]=142, ...]}
//
// Only OK-status measurements include the `distance[cm]=N` field, so finding
// that token is sufficient — no separate status gating needed. The integer
// after `=` is already in cm.
// -----------------------------------------------------------------------------
void handleLine(const String &line) {
    static const char TOKEN[] = "distance[cm]=";
    static const int  TOKEN_LEN = sizeof(TOKEN) - 1;

    int idx = line.indexOf(TOKEN);
    if (idx == -1) return;

    String after = line.substring(idx + TOKEN_LEN);
    long cm = after.toInt();  // toInt stops at first non-digit
    if (cm < (long)DIST_MIN_CM || cm > (long)DIST_MAX_CM) return;

    DistanceMsg msg;
    msg.client_id   = (uint8_t)CLIENT_ID;
    msg.distance_cm = (uint16_t)cm;
    esp_now_send(hostMac, reinterpret_cast<uint8_t *>(&msg), sizeof(msg));
    txCount++;

    Serial.printf("[client %d] tx %4ld cm  (count=%lu)\n",
                  (int)CLIENT_ID, cm, (unsigned long)txCount);
}

// -----------------------------------------------------------------------------
// 10) setup
// -----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(200);

    // Wi-Fi must be in a defined state before WiFi.macAddress() returns a
    // useful value, so kick the radio up before the banner.
    WiFi.mode(WIFI_STA);

    printConfiguration();

    CtrlSerial.begin(CTRL_UART_BAUD, SERIAL_8N1, CTRL_RX_PIN, CTRL_TX_PIN);
    setupEspNow();

    Serial.println("[client] waiting for distance lines from controller CDK...");
}

// -----------------------------------------------------------------------------
// 11) loop — drain UART2 byte-by-byte, dispatch complete lines.
// -----------------------------------------------------------------------------
void loop() {
    while (CtrlSerial.available()) {
        char c = (char)CtrlSerial.read();

        // Bring-up diagnostic: echo every UART2 byte to USB serial so we can
        // verify bytes are even arriving on GPIO33. Runtime `if` against the
        // build flag — compiler folds the dead branch when UART_DEBUG_ECHO=0.
        if (UART_DEBUG_ECHO) {
            Serial.write((uint8_t)c);
        }

        if (c == '\n' || c == '\r') {
            if (lineBuf.length() > 0) {
                handleLine(lineBuf);
                lineBuf = "";
            }
        } else if ((int)lineBuf.length() < UART_LINE_MAX) {
            lineBuf += c;
        } else {
            // Runaway / non-newline input — drop the buffer and resync.
            lineBuf = "";
        }
    }
}
