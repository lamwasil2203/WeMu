#include <Arduino.h>

/**
 * dancer_esp32.ino
 *
 * Reads distance from bridge.py over USB serial.
 * Synthesizes evolving sound and streams via Bluetooth A2DP to a speaker.
 *
 * Libraries needed (clone into ~/Arduino/libraries):
 *   https://github.com/pschatzmann/ESP32-A2DP.git
 *   https://github.com/pschatzmann/arduino-audio-tools.git
 *
 * In Arduino IDE:
 *   Board: ESP32 Dev Module
 *   Partition Scheme: Huge APP (3MB No OTA)
 */

#include "AudioTools.h"
#include "BluetoothA2DPSource.h"
#include <math.h>

// ─── CHANGE THIS to your speaker's Bluetooth name ────────────────────────────
#define BT_SPEAKER_NAME  "JBL Flip 6"
// ─────────────────────────────────────────────────────────────────────────────

#define SAMPLE_RATE     44100
#define DIST_MIN        10.0f
#define DIST_MAX        400.0f

BluetoothA2DPSource a2dp_source;

volatile float targetDist  = 150.0f;
float smoothDist           = 150.0f;

float phase1    = 0.0f;
float phase2    = 0.0f;
float lfoPhase  = 0.0f;
float driftPhase = 0.0f;

// ─── HELPERS ─────────────────────────────────────────────────────────────────

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

// ─── SOUND PARAMETER ZONES ───────────────────────────────────────────────────

struct SoundParams {
    float baseFreq;
    float ratio;
    float lfoRate;
    float lfoDepth;
    float fold;
    float osc2level;
    float driftRate;
    float driftDepth;
};

SoundParams computeParams(float dist) {
    SoundParams p;
    if (dist < 60.0f) {
        float t = mapf(dist, 10.0f, 60.0f, 0.0f, 1.0f);
        p.baseFreq   = lerpf(220.0f, 196.0f, t);
        p.ratio      = lerpf(1.059f, 1.125f, t);
        p.lfoRate    = lerpf(12.0f,  6.0f,   t);
        p.lfoDepth   = lerpf(0.7f,   0.5f,   t);
        p.fold       = lerpf(0.8f,   0.35f,  t);
        p.osc2level  = lerpf(0.9f,   0.7f,   t);
        p.driftRate  = 0.3f;
        p.driftDepth = 0.15f;
    } else if (dist < 200.0f) {
        float t = mapf(dist, 60.0f, 200.0f, 0.0f, 1.0f);
        p.baseFreq   = lerpf(196.0f, 174.6f, t);
        p.ratio      = lerpf(1.5f,   1.333f, t);
        p.lfoRate    = lerpf(4.0f,   1.5f,   t);
        p.lfoDepth   = lerpf(0.35f,  0.2f,   t);
        p.fold       = lerpf(0.15f,  0.03f,  t);
        p.osc2level  = lerpf(0.6f,   0.4f,   t);
        p.driftRate  = 0.15f;
        p.driftDepth = 0.08f;
    } else {
        float t = mapf(dist, 200.0f, 400.0f, 0.0f, 1.0f);
        p.baseFreq   = lerpf(174.6f, 164.8f, t);
        p.ratio      = 2.0f;
        p.lfoRate    = lerpf(0.8f,   0.25f,  t);
        p.lfoDepth   = lerpf(0.15f,  0.06f,  t);
        p.fold       = 0.0f;
        p.osc2level  = lerpf(0.3f,   0.1f,   t);
        p.driftRate  = 0.05f;
        p.driftDepth = 0.04f;
    }
    return p;
}

// ─── AUDIO CALLBACK ──────────────────────────────────────────────────────────

int32_t audioCallback(Frame *frame, int32_t frame_count) {
    smoothDist += (targetDist - smoothDist) * 0.002f;
    if (smoothDist < DIST_MIN) smoothDist = DIST_MIN;
    if (smoothDist > DIST_MAX) smoothDist = DIST_MAX;

    SoundParams p = computeParams(smoothDist);

    float phaseInc1  = 2.0f * M_PI * p.baseFreq / SAMPLE_RATE;
    float phaseInc2  = 2.0f * M_PI * p.baseFreq * p.ratio / SAMPLE_RATE;
    float lfoInc     = 2.0f * M_PI * p.lfoRate / SAMPLE_RATE;
    float driftInc   = 2.0f * M_PI * p.driftRate / SAMPLE_RATE;

    for (int i = 0; i < frame_count; i++) {
        float drift = 1.0f + sinf(driftPhase) * p.driftDepth * 0.0577f;
        driftPhase += driftInc;
        if (driftPhase > 2.0f * M_PI) driftPhase -= 2.0f * M_PI;

        float lfo = 1.0f - p.lfoDepth * (0.5f + 0.5f * sinf(lfoPhase));
        lfoPhase += lfoInc;
        if (lfoPhase > 2.0f * M_PI) lfoPhase -= 2.0f * M_PI;

        float osc1 = sinf(phase1 * drift);
        phase1 += phaseInc1;
        if (phase1 > 2.0f * M_PI) phase1 -= 2.0f * M_PI;

        float osc2 = sinf(phase2 * drift);
        phase2 += phaseInc2;
        if (phase2 > 2.0f * M_PI) phase2 -= 2.0f * M_PI;

        float mixed = (osc1 + osc2 * p.osc2level) / (1.0f + p.osc2level);
        mixed = wavefold(mixed, p.fold);
        mixed *= lfo;

        int16_t sample = (int16_t)(mixed * 28000.0f);
        frame[i].channel1 = sample;
        frame[i].channel2 = sample;
    }

    return frame_count;
}

// ─── SERIAL TASK ─────────────────────────────────────────────────────────────

void serialTask(void *param) {
    String buf = "";
    while (true) {
        while (Serial.available()) {
            char c = (char)Serial.read();
            if (c == '\n') {
                buf.trim();
                if (buf.startsWith("DIST:")) {
                    float d = buf.substring(5).toFloat();
                    if (d >= DIST_MIN && d <= DIST_MAX) {
                        targetDist = d;
                        Serial.printf("[rx] dist = %.1f cm\n", d);
                    }
                }
                buf = "";
            } else {
                buf += c;
            }
        }
        vTaskDelay(1);
    }
}

// ─── SETUP & LOOP ────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    Serial.println("[dancer] starting...");

    xTaskCreatePinnedToCore(serialTask, "serial", 4096, NULL, 1, NULL, 0);

    a2dp_source.set_auto_reconnect(true);
    a2dp_source.start(BT_SPEAKER_NAME, audioCallback);

    Serial.println("[dancer] BT connecting — make sure speaker is in pairing mode");
}

void loop() {
    delay(2000);
    Serial.printf("[status] target=%.1fcm  smooth=%.1fcm\n", targetDist, smoothDist);
}