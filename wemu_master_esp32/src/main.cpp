/**
 * wemu_master_esp32/src/main.cpp — audio master firmware
 *
 * Flash to the single audio master ESP32.
 *
 * Receives distance from 2 relay ESP32s (one per pair) via ESP-NOW.
 * Synthesizes sound and streams to BT speaker via A2DP.
 *
 * Sound design:
 *   Pair 1 distance → pitch and harmony
 *   Pair 2 distance → rhythm and texture
 *
 * IMPORTANT: flash this first, copy the printed MAC into
 * shared.h in BOTH projects, then flash the 2 relay ESP32s.
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "AudioTools.h"
#include "BluetoothA2DPSource.h"
#include <math.h>
#include "shared.h"

// ─── CHANGE THIS ──────────────────────────────────────────────────────────────
#define BT_SPEAKER_NAME  "Your_Speaker_Name"
// ─────────────────────────────────────────────────────────────────────────────

#define SAMPLE_RATE  44100

BluetoothA2DPSource a2dp_source;

volatile float targetDist1 = 150.0f;
volatile float targetDist2 = 150.0f;
float smoothDist1  = 150.0f;
float smoothDist2  = 150.0f;

float phase1      = 0.0f;
float phase2      = 0.0f;
float phase3      = 0.0f;
float driftPhase  = 0.0f;
float rhythmPhase = 0.0f;

// ─── Helpers ──────────────────────────────────────────────────────────────────

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    if (x < in_min) x = in_min;
    if (x > in_max) x = in_max;
    return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
}

float lerpf(float a, float b, float t) { return a + (b - a) * t; }

float wavefold(float x, float amount) {
    x *= (1.0f + amount * 2.0f);
    while (x >  1.0f) x =  2.0f - x;
    while (x < -1.0f) x = -2.0f - x;
    return x;
}

// ─── Pair 1 → pitch / harmony ────────────────────────────────────────────────

struct PitchParams {
    float baseFreq, ratio, fold, osc2level, driftRate, driftDepth;
};

PitchParams computePitch(float dist) {
    PitchParams p;
    if (dist < 60.0f) {
        float t = mapf(dist, 10.0f, 60.0f, 0.0f, 1.0f);
        p.baseFreq   = lerpf(220.0f, 196.0f, t);
        p.ratio      = lerpf(1.059f, 1.125f, t);
        p.fold       = lerpf(0.8f,   0.4f,   t);
        p.osc2level  = 0.9f;
        p.driftRate  = 0.3f;
        p.driftDepth = 0.15f;
    } else if (dist < 200.0f) {
        float t = mapf(dist, 60.0f, 200.0f, 0.0f, 1.0f);
        p.baseFreq   = lerpf(196.0f, 174.6f, t);
        p.ratio      = lerpf(1.5f,   1.333f, t);
        p.fold       = lerpf(0.15f,  0.02f,  t);
        p.osc2level  = lerpf(0.6f,   0.35f,  t);
        p.driftRate  = 0.15f;
        p.driftDepth = 0.08f;
    } else {
        float t = mapf(dist, 200.0f, 400.0f, 0.0f, 1.0f);
        p.baseFreq   = lerpf(174.6f, 164.8f, t);
        p.ratio      = 2.0f;
        p.fold       = 0.0f;
        p.osc2level  = lerpf(0.25f,  0.1f,   t);
        p.driftRate  = 0.05f;
        p.driftDepth = 0.04f;
    }
    return p;
}

// ─── Pair 2 → rhythm / texture ───────────────────────────────────────────────

struct RhythmParams {
    float pulseRate, pulseDepth, osc3freq, osc3level, masterVol;
};

RhythmParams computeRhythm(float dist) {
    RhythmParams p;
    if (dist < 60.0f) {
        float t = mapf(dist, 10.0f, 60.0f, 0.0f, 1.0f);
        p.pulseRate  = lerpf(8.0f,  4.0f,  t);
        p.pulseDepth = lerpf(0.85f, 0.6f,  t);
        p.osc3freq   = lerpf(55.0f, 65.0f, t);
        p.osc3level  = lerpf(0.5f,  0.35f, t);
        p.masterVol  = 0.9f;
    } else if (dist < 200.0f) {
        float t = mapf(dist, 60.0f, 200.0f, 0.0f, 1.0f);
        p.pulseRate  = lerpf(3.0f,  1.0f,  t);
        p.pulseDepth = lerpf(0.5f,  0.25f, t);
        p.osc3freq   = lerpf(65.0f, 80.0f, t);
        p.osc3level  = lerpf(0.25f, 0.1f,  t);
        p.masterVol  = lerpf(0.8f,  0.65f, t);
    } else {
        float t = mapf(dist, 200.0f, 400.0f, 0.0f, 1.0f);
        p.pulseRate  = lerpf(0.6f,  0.2f,  t);
        p.pulseDepth = lerpf(0.15f, 0.05f, t);
        p.osc3freq   = 82.0f;
        p.osc3level  = 0.05f;
        p.masterVol  = lerpf(0.6f,  0.45f, t);
    }
    return p;
}

// ─── Audio callback ───────────────────────────────────────────────────────────

int32_t audioCallback(Frame *frame, int32_t frame_count) {
    smoothDist1 += (targetDist1 - smoothDist1) * 0.002f;
    smoothDist2 += (targetDist2 - smoothDist2) * 0.002f;
    smoothDist1 = constrain(smoothDist1, DIST_MIN, DIST_MAX);
    smoothDist2 = constrain(smoothDist2, DIST_MIN, DIST_MAX);

    PitchParams  pp = computePitch(smoothDist1);
    RhythmParams rp = computeRhythm(smoothDist2);

    float inc1     = 2.0f * M_PI * pp.baseFreq / SAMPLE_RATE;
    float inc2     = 2.0f * M_PI * pp.baseFreq * pp.ratio / SAMPLE_RATE;
    float inc3     = 2.0f * M_PI * rp.osc3freq / SAMPLE_RATE;
    float pulseInc = 2.0f * M_PI * rp.pulseRate / SAMPLE_RATE;
    float driftInc = 2.0f * M_PI * pp.driftRate / SAMPLE_RATE;

    for (int i = 0; i < frame_count; i++) {
        float drift  = 1.0f + sinf(driftPhase) * pp.driftDepth * 0.0577f;
        driftPhase   = fmodf(driftPhase + driftInc, 2.0f * M_PI);

        float pulse  = 1.0f - rp.pulseDepth * (0.5f + 0.5f * sinf(rhythmPhase));
        rhythmPhase  = fmodf(rhythmPhase + pulseInc, 2.0f * M_PI);

        float o1 = sinf(phase1 * drift);
        phase1   = fmodf(phase1 + inc1, 2.0f * M_PI);

        float o2 = sinf(phase2 * drift);
        phase2   = fmodf(phase2 + inc2, 2.0f * M_PI);

        float o3 = sinf(phase3);
        phase3   = fmodf(phase3 + inc3, 2.0f * M_PI);

        float pitched = (o1 + o2 * pp.osc2level) / (1.0f + pp.osc2level);
        pitched = wavefold(pitched, pp.fold);

        float mixed = (pitched + o3 * rp.osc3level) / (1.0f + rp.osc3level);
        mixed *= pulse * rp.masterVol;

        int16_t sample = (int16_t)(mixed * 27000.0f);
        frame[i].channel1 = sample;
        frame[i].channel2 = sample;
    }

    return frame_count;
}

// ─── ESP-NOW receive callback ─────────────────────────────────────────────────

void onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
    if (len != sizeof(DistMessage)) return;

    DistMessage msg;
    memcpy(&msg, data, sizeof(msg));

    float d = constrain(msg.dist_cm, DIST_MIN, DIST_MAX);

    if (msg.pair_id == 1) {
        targetDist1 = d;
        Serial.printf("[master] pair1 → %.1f cm\n", d);
    } else if (msg.pair_id == 2) {
        targetDist2 = d;
        Serial.printf("[master] pair2 → %.1f cm\n", d);
    }
}

// ─── Setup ────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);

    // WiFi must init before BT for coexistence
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Print MAC — copy into shared.h in BOTH projects
    Serial.println("─────────────────────────────────────────────");
    Serial.printf("[master] MAC address: %s\n", WiFi.macAddress().c_str());
    Serial.println("Copy MAC above into shared.h MASTER_MAC (both projects)");
    Serial.println("─────────────────────────────────────────────");

    if (esp_now_init() != ESP_OK) {
        Serial.println("[master] ESP-NOW init failed — halting");
        while (true) delay(1000);
    }
    esp_now_register_recv_cb(onDataReceived);
    Serial.println("[master] ESP-NOW ready, listening for 2 relays");

    a2dp_source.set_auto_reconnect(true);
    a2dp_source.start(BT_SPEAKER_NAME, audioCallback);
    Serial.println("[master] BT connecting — put speaker in pairing mode");
}

// ─── Loop ─────────────────────────────────────────────────────────────────────

void loop() {
    delay(2000);
    Serial.printf("[master] pair1=%.1fcm  pair2=%.1fcm\n",
                  targetDist1, targetDist2);
}