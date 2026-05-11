// =============================================================================
// WeMuAudio — UWB-driven theremin synth
// -----------------------------------------------------------------------------
// Arduino UNO + Adafruit Music Maker shield (VS1053). Holds a single GM "Voice
// Oohs" note and modulates pitch (via pitch-bend, ±24 semi range) and volume
// (via CC7) from two distance readings streamed in over UART from the WeMu
// Host ESP32 (TTGO T-Display).
//
// Wire format on the UART input (SoftwareSerial RX on D5 @ 38400 8N1):
//   "D1:142 D2:67\n"   D1 controls pitch, D2 controls volume (cm).
//   "D1:-- D2:67\n"    "--" = stale client — that axis freezes at last value.
//
// Distances are mapped linearly from PLAY_RANGE_MIN_CM..PLAY_RANGE_MAX_CM to
// 0.0..1.0, with clamping at the edges.
//
// HARDWARE PREREQ: the small "MIDI" jumper on the back of the Music Maker
// shield must be soldered closed. That jumper ties the VS1053's GPIO1 high
// at boot (so it starts in real-time MIDI synth mode) and connects the chip's
// MIDI-in to Arduino D2. With the jumper open, this sketch produces silence.
// =============================================================================

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <stdlib.h>

// -----------------------------------------------------------------------------
// Config defaults (overridden by -D flags in platformio.ini)
// -----------------------------------------------------------------------------
#ifndef UART_RX_PIN
#define UART_RX_PIN 5
#endif
#ifndef UART_BAUD
#define UART_BAUD 38400
#endif
#ifndef PLAY_RANGE_MIN_CM
#define PLAY_RANGE_MIN_CM 10
#endif
#ifndef PLAY_RANGE_MAX_CM
#define PLAY_RANGE_MAX_CM 150
#endif

// -----------------------------------------------------------------------------
// MIDI link to VS1053 (SoftwareSerial TX-only on D2 @ 31250 baud)
// -----------------------------------------------------------------------------
const uint8_t MIDI_TX_PIN = 2;
const long    MIDI_BAUD   = 31250;
SoftwareSerial VS1053_MIDI(/*RX unused*/ 0, MIDI_TX_PIN);

// -----------------------------------------------------------------------------
// Inbound link from the WeMu Host ESP32 (RX-only on UART_RX_PIN).
// TX pin = -1: SoftwareSerial accepts this and won't try to drive a TX pin.
// -----------------------------------------------------------------------------
SoftwareSerial HostUart(UART_RX_PIN, /*tx unused*/ -1);

// -----------------------------------------------------------------------------
// MIDI / synth constants
// -----------------------------------------------------------------------------
const uint8_t CHANNEL          = 0;     // MIDI channel 1
const uint8_t PATCH_VOICE_OOHS = 54;    // GM patch (0-indexed)
const uint8_t NOTE_A4          = 69;
const uint8_t NOTE_VELOCITY    = 100;
const uint8_t INITIAL_VOLUME   = 100;
const uint8_t BEND_RANGE_SEMI  = 24;

const uint8_t MIDI_NOTE_OFF    = 0x80;
const uint8_t MIDI_NOTE_ON     = 0x90;
const uint8_t MIDI_CC          = 0xB0;
const uint8_t MIDI_PROGRAM     = 0xC0;
const uint8_t MIDI_PITCH_BEND  = 0xE0;

// CCs we use
const uint8_t CC_VOLUME        = 7;
const uint8_t CC_DATA_ENTRY    = 6;
const uint8_t CC_DATA_ENTRY_LSB= 38;
const uint8_t CC_RPN_LSB       = 100;
const uint8_t CC_RPN_MSB       = 101;

// -----------------------------------------------------------------------------
// State
// -----------------------------------------------------------------------------
float pitchPosition  = 0.5f;   // 0..1, 0.5 = no bend
float volumePosition = 0.0f;   // 0..1, mapped to CC7 0..127; start silent
                               // so the boot note is inaudible until a real
                               // D2 value arrives from the host.
bool  noteHeld       = false;

// vibrato (phase accumulator — survives rate changes without clicks)
const float    VIBRATO_RATE_HZ = 5.5f;
const float    VIBRATO_DEPTH   = 60.0f;   // pitch-bend units (~17 cents at ±24st)
float          vibratoPhase    = 0.0f;
unsigned long  lastVibratoUpdate = 0;

// MIDI send rate-limit (~100 Hz)
const unsigned long MIDI_SEND_INTERVAL_MS = 10;
unsigned long       lastMidiSend = 0;

// state-change tracking for printing (ignore vibrato wiggle)
int lastPrintedBend = INT16_MIN;
int lastPrintedCC7  = INT16_MIN;

// -----------------------------------------------------------------------------
// MIDI helpers — raw 3-byte messages out the SoftwareSerial port.
// -----------------------------------------------------------------------------
static inline void midiNoteOn(uint8_t ch, uint8_t note, uint8_t vel) {
  VS1053_MIDI.write((uint8_t)(MIDI_NOTE_ON | (ch & 0x0F)));
  VS1053_MIDI.write((uint8_t)(note & 0x7F));
  VS1053_MIDI.write((uint8_t)(vel & 0x7F));
}

static inline void midiNoteOff(uint8_t ch, uint8_t note) {
  VS1053_MIDI.write((uint8_t)(MIDI_NOTE_OFF | (ch & 0x0F)));
  VS1053_MIDI.write((uint8_t)(note & 0x7F));
  VS1053_MIDI.write((uint8_t)0);
}

static inline void midiCC(uint8_t ch, uint8_t cc, uint8_t value) {
  VS1053_MIDI.write((uint8_t)(MIDI_CC | (ch & 0x0F)));
  VS1053_MIDI.write((uint8_t)(cc & 0x7F));
  VS1053_MIDI.write((uint8_t)(value & 0x7F));
}

static inline void midiProgram(uint8_t ch, uint8_t patch) {
  VS1053_MIDI.write((uint8_t)(MIDI_PROGRAM | (ch & 0x0F)));
  VS1053_MIDI.write((uint8_t)(patch & 0x7F));
}

// 14-bit pitch bend: LSB byte first, then MSB. 8192 = no bend.
static inline void midiPitchBend(uint8_t ch, int value14) {
  if (value14 < 0)     value14 = 0;
  if (value14 > 16383) value14 = 16383;
  VS1053_MIDI.write((uint8_t)(MIDI_PITCH_BEND | (ch & 0x0F)));
  VS1053_MIDI.write((uint8_t)(value14 & 0x7F));
  VS1053_MIDI.write((uint8_t)((value14 >> 7) & 0x7F));
}

// -----------------------------------------------------------------------------
// Synth init — patch select, ±24-semi pitch-bend range via RPN, initial note.
// -----------------------------------------------------------------------------
void setupSynth() {
  midiProgram(CHANNEL, PATCH_VOICE_OOHS);
  midiCC(CHANNEL, CC_VOLUME, INITIAL_VOLUME);

  // RPN 0,0 = pitch-bend sensitivity. Set to BEND_RANGE_SEMI semitones.
  midiCC(CHANNEL, CC_RPN_MSB, 0);
  midiCC(CHANNEL, CC_RPN_LSB, 0);
  midiCC(CHANNEL, CC_DATA_ENTRY, BEND_RANGE_SEMI);
  midiCC(CHANNEL, CC_DATA_ENTRY_LSB, 0);
  // Null the RPN so later CC6/CC38 messages don't accidentally re-target it.
  midiCC(CHANNEL, CC_RPN_MSB, 127);
  midiCC(CHANNEL, CC_RPN_LSB, 127);

  midiPitchBend(CHANNEL, 8192);
  midiNoteOn(CHANNEL, NOTE_A4, NOTE_VELOCITY);
  noteHeld = true;
}

// -----------------------------------------------------------------------------
// Input handling — line-based parser for the host's "D1:N D2:N\n" protocol.
// -----------------------------------------------------------------------------

// Worst-case line is "D1:65535 D2:65535\n" ≈ 19 chars. 32 leaves headroom.
static char    uartLineBuf[32];
static uint8_t uartLineLen = 0;

// Map cm → 0..1, clamped, using PLAY_RANGE_MIN_CM..PLAY_RANGE_MAX_CM.
static inline float cmToUnit(long cm) {
  long span = (long)PLAY_RANGE_MAX_CM - (long)PLAY_RANGE_MIN_CM;
  if (span <= 0) return 0.0f;  // defensive — shouldn't happen
  float t = (float)(cm - (long)PLAY_RANGE_MIN_CM) / (float)span;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return t;
}

// Parse one complete line. "D1:" / "D2:" can appear in any order. A value of
// "--" means the corresponding client is stale — leave that position float
// untouched so the last known value is held.
static void handleUartLine(const char *line) {
  const char *p1 = strstr(line, "D1:");
  const char *p2 = strstr(line, "D2:");
  if (p1) {
    p1 += 3;
    if (p1[0] != '-') {
      long cm = atol(p1);
      pitchPosition = cmToUnit(cm);
    }
  }
  if (p2) {
    p2 += 3;
    if (p2[0] != '-') {
      long cm = atol(p2);
      volumePosition = cmToUnit(cm);
    }
  }
}

void handleInput() {
  while (HostUart.available() > 0) {
    int c = HostUart.read();
    if (c < 0) break;
    if (c == '\n' || c == '\r') {
      if (uartLineLen > 0) {
        uartLineBuf[uartLineLen] = '\0';
        handleUartLine(uartLineBuf);
        uartLineLen = 0;
      }
    } else if (uartLineLen < sizeof(uartLineBuf) - 1) {
      uartLineBuf[uartLineLen++] = (char)c;
    } else {
      // Overflow — discard and resync at the next newline. Expected to
      // happen occasionally when a MIDI TX burst eats RX bytes (see note in
      // setup()); newline framing recovers automatically.
      uartLineLen = 0;
    }
  }
}

// -----------------------------------------------------------------------------
// Setup / loop
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  VS1053_MIDI.begin(MIDI_BAUD);

  // Open the inbound link from the WeMu Host and make it the active
  // SoftwareSerial RX listener. VS1053_MIDI is TX-only and never reads, so
  // it doesn't compete for the listener slot.
  //
  // Caveat: SoftwareSerial disables interrupts during .write(), so MIDI TX
  // bursts (~1.9 ms each at the 100 Hz rate-limit) will eat some RX bytes.
  // Lines that get partially clobbered are discarded by the newline-framed
  // parser; the host emits multiple lines per second so we recover quickly.
  HostUart.begin(UART_BAUD);
  HostUart.listen();

  // Let the VS1053 finish its boot-time MIDI-mode init before we shove bytes at it.
  delay(500);

  setupSynth();
  lastVibratoUpdate = millis();
  lastMidiSend      = millis();

  Serial.print(F("Listening for D1/D2 stream on pin "));
  Serial.print((int)UART_RX_PIN);
  Serial.print(F(" @ "));
  Serial.print((long)UART_BAUD);
  Serial.print(F(" baud, range "));
  Serial.print((int)PLAY_RANGE_MIN_CM);
  Serial.print(F(".."));
  Serial.print((int)PLAY_RANGE_MAX_CM);
  Serial.println(F(" cm"));
}

void loop() {
  handleInput();

  unsigned long now = millis();

  // Phase-accumulator vibrato. Accumulate every iteration so phase is correct
  // regardless of how often we end up actually sending MIDI.
  float dt = (now - lastVibratoUpdate) / 1000.0f;
  lastVibratoUpdate = now;
  vibratoPhase += 2.0f * (float)PI * VIBRATO_RATE_HZ * dt;
  while (vibratoPhase > 2.0f * (float)PI) vibratoPhase -= 2.0f * (float)PI;

  if (now - lastMidiSend < MIDI_SEND_INTERVAL_MS) return;
  lastMidiSend = now;

  // While silenced, don't bother streaming pitch-bend/CC7.
  if (!noteHeld) return;

  int basePitchBend = (int)(pitchPosition * 16383.0f);
  int vibratoOffset = (int)(sin(vibratoPhase) * VIBRATO_DEPTH);
  int finalBend     = basePitchBend + vibratoOffset;
  if (finalBend < 0)     finalBend = 0;
  if (finalBend > 16383) finalBend = 16383;

  int cc7 = (int)(volumePosition * 127.0f);
  if (cc7 < 0)   cc7 = 0;
  if (cc7 > 127) cc7 = 127;

  midiPitchBend(CHANNEL, finalBend);
  midiCC(CHANNEL, CC_VOLUME, (uint8_t)cc7);

  // Print only when commanded state changes — vibrato wiggle would spam otherwise.
  if (basePitchBend != lastPrintedBend || cc7 != lastPrintedCC7) {
    lastPrintedBend = basePitchBend;
    lastPrintedCC7  = cc7;
    Serial.print(F("pitch="));
    Serial.print(pitchPosition, 2);
    Serial.print(F(" (bend="));
    Serial.print(basePitchBend);
    Serial.print(F(")  vol="));
    Serial.print(volumePosition, 2);
    Serial.print(F(" (cc7="));
    Serial.print(cc7);
    Serial.println(F(")"));
  }
}
