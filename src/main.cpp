#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
// Proximity Display — TTGO T1 (ST7789 240x135) + DWM3001C via UART
//
// Wiring:
//   DWM3001C TX  →  GPIO16 (ESP32 RX2)
//   DWM3001C RX  →  GPIO17 (ESP32 TX2)
//   DWM3001C GND →  GND
//   DWM3001C VDD →  3.3V
//
// Libraries needed (install via Arduino Library Manager):
//   - TFT_eSPI  (configure User_Setup.h for ST7789 240x135, see below)
//   - Arduino ESP32 core
//
// TFT_eSPI User_Setup.h key settings for TTGO T1:
//   #define ST7789_DRIVER
//   #define TFT_WIDTH  135
//   #define TFT_HEIGHT 240
//   #define TFT_MOSI   19
//   #define TFT_SCLK   18
//   #define TFT_CS     5
//   #define TFT_DC     16   <- check your board variant, may differ
//   #define TFT_RST    23
//   #define TFT_BL     4    <- backlight pin
// ─────────────────────────────────────────────────────────────────────────────

#include <TFT_eSPI.h>
#include <SPI.h>
#include <math.h>

// ── UART ─────────────────────────────────────────────────────────────────────
#define RXD2        16
#define TXD2        17
#define UART_BAUD   115200

// ── Display ──────────────────────────────────────────────────────────────────
TFT_eSPI tft = TFT_eSPI();
// Sprite is an off-screen buffer — draws to RAM then blits to screen.
// This eliminates all flicker between frames.
TFT_eSprite sprite = TFT_eSprite(&tft);

#define SCREEN_W    240
#define SCREEN_H    135

// ── Proximity config ─────────────────────────────────────────────────────────
#define MAX_DIST_M        3.0f   // beyond this = minimum intensity
#define CLOSE_DIST_M      0.3f   // below this = full bloom / flash
#define NUM_SENSORS       4      // how many DWM3001C sensors

// Smoothing: exponential moving average.
// Lower = smoother but slower to react. 0.15 is a good starting point.
#define SMOOTH_ALPHA      0.15f

// ── State ────────────────────────────────────────────────────────────────────
float distances[NUM_SENSORS] = {MAX_DIST_M, MAX_DIST_M, MAX_DIST_M, MAX_DIST_M};
float smoothedDist = MAX_DIST_M;   // averaged + smoothed distance
float animTime = 0.0f;             // drives all animations
String uartBuffer = "";            // accumulates serial chars

// ─────────────────────────────────────────────────────────────────────────────
// Color interpolation helpers
// ─────────────────────────────────────────────────────────────────────────────

struct Color { uint8_t r, g, b; };

Color lerpColor(Color a, Color b, float t) {
  t = constrain(t, 0.0f, 1.0f);
  return {
    (uint8_t)(a.r + (b.r - a.r) * t),
    (uint8_t)(a.g + (b.g - a.g) * t),
    (uint8_t)(a.b + (b.b - a.b) * t)
  };
}

// Convert 8-bit RGB to TFT_eSPI's 16-bit 565 color format
uint16_t to565(Color c) {
  return ((c.r & 0xF8) << 8) | ((c.g & 0xFC) << 3) | (c.b >> 3);
}

// ── Color gradient stages (far → close) ──────────────────────────────────────
const int NUM_STAGES = 6;
Color colorStages[NUM_STAGES] = {
  {0,   80,  255},  // deep blue      ~3m+
  {0,   180, 255},  // icy blue       ~2.5m
  {0,   220, 160},  // cyan           ~2m
  {255, 220, 0},    // yellow         ~1m
  {255, 100, 0},    // orange         ~0.5m
  {255, 20,  0},    // red            ~0.2m
};

Color getColorForIntensity(float intensity) {
  float idx = intensity * (NUM_STAGES - 1);
  int lo = (int)idx;
  int hi = min(lo + 1, NUM_STAGES - 1);
  return lerpColor(colorStages[lo], colorStages[hi], idx - lo);
}

// ─────────────────────────────────────────────────────────────────────────────
// Distance → intensity (0.0 = far, 1.0 = touching)
// ─────────────────────────────────────────────────────────────────────────────
float distToIntensity(float dist) {
  return constrain(1.0f - (dist / MAX_DIST_M), 0.0f, 1.0f);
}

// ─────────────────────────────────────────────────────────────────────────────
// UART parsing
//
// Expected format from DWM3001C CLI (adapt to your firmware output):
//   DIST:1.23\n       → single sensor
//   DIST0:1.23\n      → sensor 0
//   DIST1:2.45\n      → sensor 1
//   etc.
//
// Use the serial passthrough sketch first to see your exact output format,
// then update the parsing below to match.
// ─────────────────────────────────────────────────────────────────────────────
void parseUartLine(String line) {
  line.trim();

  // Multi-sensor format: "DIST0:1.23"
  for (int i = 0; i < NUM_SENSORS; i++) {
    String prefix = "DIST" + String(i) + ":";
    if (line.startsWith(prefix)) {
      float d = line.substring(prefix.length()).toFloat();
      if (d > 0.01f && d < 20.0f) {  // sanity check
        distances[i] = d;
      }
      return;
    }
  }

  // Single sensor fallback: "DIST:1.23"
  if (line.startsWith("DIST:")) {
    float d = line.substring(5).toFloat();
    if (d > 0.01f && d < 20.0f) {
      distances[0] = d;
    }
  }
}

void readUart() {
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n') {
      if (uartBuffer.length() > 0) {
        parseUartLine(uartBuffer);
        uartBuffer = "";
      }
    } else if (c != '\r') {
      uartBuffer += c;
      if (uartBuffer.length() > 64) uartBuffer = ""; // overflow guard
    }
  }
}

// Average only the valid (non-stale) sensor readings
float averageDistances() {
  float sum = 0;
  int count = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (distances[i] > 0.01f) {
      sum += distances[i];
      count++;
    }
  }
  return (count > 0) ? sum / count : MAX_DIST_M;
}

// ─────────────────────────────────────────────────────────────────────────────
// Drawing
// ─────────────────────────────────────────────────────────────────────────────

// Fast circle fill using TFT_eSprite's built-in — no pixel loop needed
void drawFilledCircleAlpha(int cx, int cy, int r, Color col, float alpha) {
  // Approximate alpha by blending toward black (screen is black background)
  Color c = lerpColor({0, 0, 0}, col, alpha);
  sprite.fillCircle(cx, cy, r, to565(c));
}

void renderFrame(float intensity, Color baseColor) {
  const int cx = SCREEN_W / 2;
  const int cy = SCREEN_H / 2;
  float pulse = 0.5f + 0.5f * sinf(animTime * (1.5f + intensity * 2.0f));
 
  // ── 1. Flood the whole screen with the current color ──────────────────────
  // The entire display IS the color — bright, saturated, no black anywhere
  float brightness = 0.18f + intensity * 0.75f + pulse * 0.07f;
  brightness = constrain(brightness, 0.0f, 1.0f);
  Color bgColor = lerpColor({0, 0, 0}, baseColor, brightness);
  sprite.fillSprite(to565(bgColor));
 
  // ── 2. Slightly brighter center so it feels like light emanating outward ──
  // Just one filled circle — large enough to cover the center third
  Color centerColor = lerpColor(baseColor, {255, 255, 255}, 0.3f + intensity * 0.3f);
  sprite.fillCircle(cx, cy, (int)(SCREEN_W * 0.45f), to565(centerColor));
 
  // Softer blend circle on top to smooth the edge of the center circle
  Color blendColor = lerpColor(bgColor, centerColor, 0.5f);
  sprite.fillCircle(cx, cy, (int)(SCREEN_W * 0.32f), to565(blendColor));
 
  // ── 3. Orbs — two white dots moving toward each other ─────────────────────
  float separation = (1.0f - intensity) * (SCREEN_W * 0.32f) + intensity * 4.0f;
  int o1x  = cx - (int)separation;
  int o2x  = cx + (int)separation;
  int oy   = cy;
  int orbR = 6 + (int)(intensity * 8.0f);
 
  // Aura — a slightly larger white-tinted circle behind the core
  Color auraColor = lerpColor(baseColor, {255, 255, 255}, 0.6f);
  sprite.fillCircle(o1x, oy, orbR + 8, to565(auraColor));
  sprite.fillCircle(o2x, oy, orbR + 8, to565(auraColor));
 
  // Hard white cores
  sprite.fillCircle(o1x, oy, orbR, TFT_WHITE);
  sprite.fillCircle(o2x, oy, orbR, TFT_WHITE);
 
  // ── 4. Beam connecting the orbs (grows as they get closer) ───────────────
  if (intensity > 0.05f) {
    float ba = constrain((intensity - 0.05f) / 0.95f, 0.0f, 1.0f);
    Color beamColor = lerpColor({0,0,0}, {255,255,255}, ba * 0.9f);
    uint16_t bc = to565(beamColor);
    int beamThick = 1 + (int)(intensity * 3.0f);
    for (int dy = -beamThick; dy <= beamThick; dy++) {
      sprite.drawLine(o1x + orbR, oy + dy, o2x - orbR, oy + dy, bc);
    }
  }
 
  // ── 5. Push to display ────────────────────────────────────────────────────
  sprite.pushSprite(0, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Setup & loop
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial2.begin(UART_BAUD, SERIAL_8N1, RXD2, TXD2);

  tft.init();
  tft.setRotation(1);        // landscape
  tft.fillScreen(TFT_BLACK);

  // Allocate the sprite (off-screen framebuffer)
  sprite.createSprite(SCREEN_W, SCREEN_H);
  sprite.setSwapBytes(true);

  // Backlight on (adjust pin if needed)
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  Serial.println("Proximity display ready");
}

void loop() {
  // Type a distance like "1.5" in the PlatformIO serial monitor and hit enter
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    float val = input.toFloat();
    if (val > 0.0f && val < 10.0f) {
      smoothedDist = val;
      Serial.printf("Distance set to %.2f m\n", smoothedDist);
    }
  }

  float intensity = distToIntensity(smoothedDist);
  float pulseSpeed = 0.02f + intensity * 0.07f;
  animTime += pulseSpeed;

  Color baseColor = getColorForIntensity(intensity);
  renderFrame(intensity, baseColor);

  delay(16);
}