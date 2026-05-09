/**
 * wemu-esp32/src/main.cpp — relay firmware
 *
 * Flash to BOTH relay ESP32s (one per pair).
 *
 * Each relay ESP32 handles a full pair of CDKs:
 *   - Powers BOTH CDKs via Y-split on VIN and GND
 *   - Reads distance from the CONTROLLER CDK over UART2
 *     (controlee CDK outputs no distance data, so no second UART needed)
 *   - Forwards distance to audio master via ESP-NOW
 *
 * Wiring:
 *   Controller CDK Pi header:
 *     Pin 4 (5V)  → ESP32 VIN  ← also Y-split to controlee CDK Pin 4
 *     Pin 6 (GND) → ESP32 GND  ← also Y-split to controlee CDK Pin 6
 *     Pin 8 (TX)  → GPIO16 (RX2)
 *     Pin 10 (RX) → GPIO17 (TX2)
 *
 *   Controlee CDK Pi header:
 *     Pin 4 (5V)  → Y-split from VIN
 *     Pin 6 (GND) → Y-split from GND
 *     (no data lines)
 *
 * Pair ID:
 *   GPIO34 floating → Pair 1 (run CDKs on channel 9)
 *   GPIO34 to GND   → Pair 2 (run CDKs on channel 5)
 *
 * Flash order:
 *   1. Flash wemu_master_esp32 first
 *   2. Copy MAC from serial monitor into shared.h (both projects)
 *   3. Flash this to both relay ESP32s
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include "shared.h"

HardwareSerial CtrlSerial(2);  // UART2 — controller CDK

uint8_t pairId      = 1;
bool    espNowReady = false;

// ─── ESP-NOW send callback ────────────────────────────────────────────────────

void onSent(const uint8_t *mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("[relay] send failed — is master powered on?");
    }
}

// ─── Distance parser ──────────────────────────────────────────────────────────

float parseDistance(const String &line) {
    int idx = line.indexOf("distance:");
    if (idx == -1) return -1.0f;
    String after = line.substring(idx + 9);
    after.trim();
    return after.toFloat();
}

// ─── Setup ────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);

    // Pair ID from strap pin
    pinMode(PAIR_ID_PIN, INPUT_PULLUP);
    delay(20);
    pairId = (digitalRead(PAIR_ID_PIN) == HIGH) ? 1 : 2;
    Serial.printf("[relay] pair %d starting\n", pairId);
    Serial.printf("[relay] powering both CDKs, reading controller on UART2\n");

    // UART2 → controller CDK
    CtrlSerial.begin(115200, SERIAL_8N1, CTRL_RX, CTRL_TX);

    // WiFi for ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("[relay] ESP-NOW init failed — halting");
        while (true) delay(1000);
    }
    esp_now_register_send_cb(onSent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, MASTER_MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("[relay] failed to add master — check MASTER_MAC in shared.h");
        while (true) delay(1000);
    }

    espNowReady = true;
    Serial.printf("[relay] pair %d ready → master %02X:%02X:%02X:%02X:%02X:%02X\n",
        pairId,
        MASTER_MAC[0], MASTER_MAC[1], MASTER_MAC[2],
        MASTER_MAC[3], MASTER_MAC[4], MASTER_MAC[5]);
}

// ─── Loop ─────────────────────────────────────────────────────────────────────

void loop() {
    static String buf        = "";
    static String lastStatus = "Ok";

    while (CtrlSerial.available()) {
        char c = (char)CtrlSerial.read();

        if (c == '\n') {
            buf.trim();

            if (buf.indexOf("status:") != -1) {
                lastStatus = (buf.indexOf("Ok") != -1) ? "Ok" : "Error";
            }

            if (buf.indexOf("distance:") != -1 && lastStatus == "Ok" && espNowReady) {
                float d = parseDistance(buf);
                if (d >= DIST_MIN && d <= DIST_MAX) {
                    DistMessage msg;
                    msg.pair_id = pairId;
                    msg.dist_cm = d;
                    esp_now_send(MASTER_MAC, (uint8_t *)&msg, sizeof(msg));
                    Serial.printf("[relay] pair%d → %.1f cm\n", pairId, d);
                }
            }

            buf = "";
        } else {
            buf += c;
        }
    }
}