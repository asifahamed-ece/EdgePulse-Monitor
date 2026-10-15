// ═══════════════════════════════════════════════════════════════════════
//  EdgePulse — optional TFT status display  (ST7789 over SPI)
//  BOARD: Espressif ESP32-S3-DevKitC-1
//
//  See TftDisplay.h for the contract. The short version: this file renders
//  numbers other tasks measured, and renders nothing at all for anything
//  they did not measure.
//
//  Layout, colour and the two policy functions (bannerFor / noteFor) are
//  below. Nothing here touches the acquisition path — the panel is driven
//  exclusively by tftTask on CORE 0 at 2 Hz.
// ═══════════════════════════════════════════════════════════════════════

#include "TftDisplay.h"

// The rest of the translation unit is empty when the gate is off. That is
// the point: nothing is compiled, nothing is linked, no task exists.
#if EDGE_TFT_ENABLED

#include "pins.h"
#include "Sensors.h"   // SENSOR_MPU6050
#include <Arduino.h>
#include <string.h>
#include <math.h>      // isfinite
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// ── Palette (RGB565) ──────────────────────────────────────────────────
// The three health colours are picked so that ONE foreground beats all
// three. Black text is 15.3:1 on this green, 14.4:1 on this amber and
// 5.3:1 on this red; white would manage 2.9:1 on the green. Since the
// verdict is the thing being read from a metre away, it gets the highest
// contrast available rather than a colour chosen by eye.
static constexpr uint16_t C_BG        = 0x0000;  // black
static constexpr uint16_t C_GOOD      = 0x07E0;  // green
static constexpr uint16_t C_FAULTY    = 0xFD20;  // amber
static constexpr uint16_t C_DANGER    = 0xF800;  // red
static constexpr uint16_t C_NO_DATA   = 0x4208;  // neutral dark grey
static constexpr uint16_t C_ON_BANNER = 0x0000;  // black, on the banner
static constexpr uint16_t C_VALUE     = 0xFFFF;  // a real measurement
static constexpr uint16_t C_MISSING   = 0x7BEF;  // "---", deliberately dim
static constexpr uint16_t C_LABEL     = 0xC618;  // light grey
static constexpr uint16_t C_FOOT      = 0x7BEF;  // dark grey, non-critical
static constexpr uint16_t C_RULE      = 0x2104;  // separator hairlines

// ── Type scale (built-in 5x7 font, 6 px advance per character per unit) ──
static constexpr uint8_t SZ_BANNER = 4;   // 20x28 px — the word you read first
static constexpr uint8_t SZ_VALUE  = 3;   // 15x21 px — the numbers
static constexpr uint8_t SZ_LABEL  = 1;   // 5x7 px  — deliberately quiet

// ── Layout ────────────────────────────────────────────────────────────
// Every band is derived from the panel height so one code path lays out a
// 240x240 and a 240x320 panel. The value rows are what get read at a
// distance, so they get the space that is left over rather than a fixed
// pixel count that would overflow the shorter glass.
static constexpr int16_t PAD       = 6;
static constexpr int16_t HEADER_H  = 16;
static constexpr int16_t BANNER_Y  = HEADER_H;
static constexpr int16_t BANNER_H  = 52;
static constexpr int16_t NOTE_Y    = BANNER_Y + BANNER_H;    // 68
static constexpr int16_t NOTE_H    = 14;                      // rows start at 82
static constexpr int16_t ROWS_Y    = NOTE_Y + NOTE_H;
static constexpr int16_t FOOTER_H  = 14;
static constexpr int16_t ROW_COUNT = 6;

static constexpr int16_t GLYPH_H = 7;    // built-in font is 5x7

// ── SPI ───────────────────────────────────────────────────────────────
// A dedicated SPIClass, not the global SPI.
//
// Two reasons, both load-bearing:
//
//  1. Adafruit_SPITFT::initSPI() only calls begin() for itself when the
//     SPIClass pointer is one of the core's established interfaces
//     (&SPI, &SPI1 ...). Hand it a private object and it leaves the bus
//     alone, so this file can call begin() with EXPLICIT pins from
//     include/pins.h — which is the whole point of having a pin map.
//
//  2. begin(sck, miso, mosi) with no ss argument leaves the peripheral's
//     hardware chip-select unattached (Adafruit SPITFT never calls
//     setHwCs(), because SPISettings(freq, MSBFIRST, mode) declares no ss).
//     GPIO10 is therefore ours as D/C and can never be driven by the SPI
//     hardware behind our back.
//
// GSPI/FSPI rather than HSPI: on the S3 the peripheral at index 0 is the
// one with the best-tested pin path, and nothing else in this firmware
// touches SPI at all.
static SPIClass tftSpi(FSPI);
static Adafruit_ST7789 tft(&tftSpi, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

// ── Text width ────────────────────────────────────────────────────────
// Adafruit GFX 1.12 dropped textWidth(), and the obvious replacement — a
// GFXfont — would break this, because a proportional font has no fixed
// advance. The built-in font advances exactly 6 px per character per unit
// of size, which is why this file never loads a custom font. Changing that
// means changing this function too.
static int16_t textW(const char* s, uint8_t size) {
    return (int16_t)(strlen(s) * 6 * size);
}

// ═══════════════════════════════════════════════════════════════════════
//  POLICY
// ═══════════════════════════════════════════════════════════════════════

// The verdict word.
//
// NO DATA is tested FIRST and outranks every health verdict, because it is
// not a competing opinion — it is the absence of one. "GOOD" asserts that
// the motor is healthy; with the vibration channel silent that assertion
// has no evidence behind it, and a panel that says GOOD to an unmonitored
// machine is precisely the false assurance this project exists to refuse.
//
// The trigger is specifically SENSOR_MPU6050 and not "all sensors". Every
// threshold in Rules.cpp is computed from the vibration features, so an
// MPU that is not delivering means no rule can be evaluated — whatever the
// DHT22 happens to be doing. This introduces NO new threshold: it is an
// instrumentation state, not a health state, and nothing about it claims
// anything about the motor.
//
// The remaining three map one-to-one onto what Rules.h actually
// distinguishes. WARNING and CRITICAL become FAULTY and DANGER; NORMAL and
// UNCALIBRATED both become GOOD, with UNCALIBRATED carrying the note
// below. A fourth alert level would need a limit no measurement supports,
// so there isn't one.
static TftBanner bannerFor(const TftSnapshot& s) {
    if (!(s.live_status & SENSOR_MPU6050)) return BANNER_NO_DATA;
    if (s.status == STATUS_WARNING)          return BANNER_FAULTY;
    if (s.status == STATUS_CRITICAL)         return BANNER_DANGER;
    return BANNER_GOOD;
}

// The small line under the word.
//
// UNCALIBRATED is returned for two different reasons — no baseline was
// loaded, or the current window had too few samples to score — so the text
// names the actual cause instead of asserting one of them.
static const char* noteFor(const TftSnapshot& s) {
    if (!(s.live_status & SENSOR_MPU6050)) return "NO SENSOR DATA - CHECK WIRING";
    if (s.status != STATUS_UNCALIBRATED)   return "";
    if (!s.calibrated)                     return "THRESHOLDS OFF - NO BASELINE";
    return "THRESHOLDS ON - WINDOW INCOMPLETE";
}

// ═══════════════════════════════════════════════════════════════════════
//  LIFECYCLE
// ═══════════════════════════════════════════════════════════════════════

bool TftDisplay::begin() {
    // Refuse an unsupported geometry rather than centre-cropping it. A
    // wrong geometry produces a picture that looks entirely plausible and
    // is quietly wrong, which is the exact failure mode being avoided.
    const bool known = (TFT_PANEL_W == 240 && TFT_PANEL_H == 240) ||
                       (TFT_PANEL_W == 240 && TFT_PANEL_H == 320);
    if (!known) {
        Serial.printf("[TFT] unsupported panel %ux%u — expected 240x240 or 240x320\n",
                      (unsigned)TFT_PANEL_W, (unsigned)TFT_PANEL_H);
        return false;
    }

    // Explicit pins, from include/pins.h. Must precede init(): initSPI()
    // will not touch a private SPIClass (see the note on tftSpi above).
    tftSpi.begin(PIN_TFT_SCLK, PIN_TFT_MISO, PIN_TFT_MOSI);

#if EDGE_TFT_BACKLIGHT
    pinMode(PIN_TFT_BL, OUTPUT);
    digitalWrite(PIN_TFT_BL, HIGH);
#endif

    tft.init(TFT_PANEL_W, TFT_PANEL_H);
    tft.setRotation(TFT_ROTATION);
    tft.setTextWrap(false);

    _w = (uint16_t)tft.width();
    _h = (uint16_t)tft.height();

    Serial.printf("[TFT] ST7789 up: %ux%u rot=%u  SCLK=%u MOSI=%u MISO=%u DC=%u CS=%u RST=%u BL=%u\n",
                  _w, _h, (unsigned)TFT_ROTATION,
                  PIN_TFT_SCLK, PIN_TFT_MOSI, PIN_TFT_MISO,
                  PIN_TFT_DC, PIN_TFT_CS, PIN_TFT_RST,
#if EDGE_TFT_BACKLIGHT
                  PIN_TFT_BL
#else
                  255
#endif
    );

    splash();
    _present = true;
    return true;
}

// Panel identity and geometry. Nothing else.
//
// No verdict word and no readings are shown here: the first show() lands
// within TFT_REDRAW_MS with real data, and anything drawn in between would
// be a value nobody measured.
void TftDisplay::splash() {
    tft.fillScreen(C_BG);
    tft.setTextSize(SZ_BANNER);
    tft.setTextColor(C_LABEL);
    const char* name = "EdgePulse";
    tft.setCursor((int16_t)((_w - textW(name, SZ_BANNER)) / 2),
                  (int16_t)(_h / 2 - GLYPH_H * SZ_BANNER));
    tft.print(name);

    tft.setTextSize(SZ_LABEL);
    tft.setTextColor(C_FOOT);
    char geo[24];
    snprintf(geo, sizeof geo, "%ux%u", _w, _h);
    tft.setCursor((int16_t)((_w - textW(geo, SZ_LABEL)) / 2),
                  (int16_t)(_h / 2 + GLYPH_H * SZ_BANNER + 6));
    tft.print(geo);
}

// ═══════════════════════════════════════════════════════════════════════
//  ONE REDRAW
// ═══════════════════════════════════════════════════════════════════════

void TftDisplay::show(const TftSnapshot& s) {
    if (!_present) return;

    // Full repaint. Every redraw is ~60-100 ms of CORE 0 at 16 MHz and
    // 2 Hz, which is a few percent of one core — a whole-panel repaint is
    // far easier to reason about than dirty rectangles, and this panel is a
    // status word, not a plotter.
    tft.fillScreen(C_BG);

    // ── header ────────────────────────────────────────────────────────
    tft.setTextSize(SZ_LABEL);
    tft.setTextColor(C_LABEL);
    tft.setCursor(PAD, 5);
    tft.print("EDGEPULSE");

    if (s.recording) {
        // A dot and two letters. At a glance from a metre this reads as
        // "this run is being logged" without competing with the verdict for
        // attention. Recording is a fact about the firmware's own state,
        // never about the machine.
        const char* rec = "REC";
        tft.setTextColor(C_DANGER);
        const int16_t rx = _w - PAD - textW(rec, SZ_LABEL) - 8;
        tft.setCursor(rx + 8, 5);
        tft.print(rec);
        tft.fillCircle(rx + 2, 9, 3, C_DANGER);
    }
    tft.drawFastHLine(0, HEADER_H - 1, _w, C_RULE);

    // ── verdict word ──────────────────────────────────────────────────
    banner(bannerFor(s), noteFor(s));

    // ── readings ──────────────────────────────────────────────────────
    const int16_t rowH = (int16_t)((_h - FOOTER_H - ROWS_Y) / ROW_COUNT);
    uint8_t i = 0;

    // One place decides "value" versus "---". `have` is the reader's own
    // success flag and `isfinite` rejects a NaN that slipped through; if
    // either fails the cell reads "---", never 0.
    auto cell = [&](const char* label, bool have, double v, const char* fmt) {
        char b[24];
        const bool ok = have && isfinite(v);
        if (ok) snprintf(b, sizeof b, fmt, v);
        else    snprintf(b, sizeof b, "---");
        row(ROWS_Y + (int16_t)i++ * rowH, rowH, label, b, ok);
    };

    // Vibration shows the WINDOWED RMS only.
    //
    // wsTask substitutes an instantaneous |a| - 1 g estimate while the
    // first window is still filling. The panel deliberately does not use
    // it: that number is a different quantity, and printing it under the
    // label VIB RMS would be a small lie even though every sample behind it
    // was real. Two seconds of "---" is a much cheaper price.
    cell("VIB RMS",     s.have_vib,        s.vib_rms,        "%.4f g");

    // Temperatures and humidity come from the sensor caches. The snapshot
    // carries the readers' success flags, so a probe that has never
    // returned a value cannot borrow the value from a window where it did.
    cell("MOTOR TEMP",  s.have_motor_temp, s.motor_temp_C,   "%.1f C");
    cell("AMBIENT",     s.have_ambient,    s.ambient_temp_C, "%.1f C");
    cell("HUMIDITY",    s.have_ambient,    s.humidity_pct,   "%.0f %%");

    // Sample rate: measured, not the nominal HZ target. NaN until the first
    // one-second window completes, so it reads "---" rather than asserting
    // a rate nobody has timed yet.
    cell("SAMPLE RATE", true,              s.hz,            "%.1f Hz");

    // Dropped samples. A firmware counter, not a sensor reading: it is
    // always known, so it never takes the "---" path. Zero here is a real
    // zero and means something different from "---".
    char nbuf[16];
    snprintf(nbuf, sizeof nbuf, "%lu", (unsigned long)s.dropped);
    row(ROWS_Y + (int16_t)i * rowH, rowH, "DROPPED", nbuf, true);

    // ── provenance footer ─────────────────────────────────────────────
    footer(s);
}

// ── the verdict word, full width ──────────────────────────────────────
void TftDisplay::banner(TftBanner b, const char* note) {
    const char* word;
    uint16_t bg;

    switch (b) {
        case BANNER_GOOD:   word = "GOOD";   bg = C_GOOD;      break;
        case BANNER_FAULTY: word = "FAULTY"; bg = C_FAULTY;    break;
        case BANNER_DANGER: word = "DANGER"; bg = C_DANGER;    break;
        default:            word = "NO DATA"; bg = C_NO_DATA;   break;
    }

    tft.fillRect(0, BANNER_Y, _w, BANNER_H, bg);

    // NO DATA carries white on grey: black-on-grey would read as a muted
    // health verdict rather than as the absence of one.
    tft.setTextColor(b == BANNER_NO_DATA ? C_VALUE : C_ON_BANNER);
    tft.setTextSize(SZ_BANNER);
    tft.setCursor((int16_t)((_w - textW(word, SZ_BANNER)) / 2),
                  (int16_t)(BANNER_Y + (BANNER_H - GLYPH_H * SZ_BANNER) / 2));
    tft.print(word);

    // The note band is reserved even when empty, so the reading rows do not
    // jump up and down when the calibration state changes. A layout that
    // moves is a layout people stop trusting.
    if (note && note[0]) {
        tft.setTextSize(SZ_LABEL);
        tft.setTextColor(C_FOOT);
        tft.setCursor((int16_t)((_w - textW(note, SZ_LABEL)) / 2), NOTE_Y + 4);
        tft.print(note);
    }
}

// ── one label / one value ─────────────────────────────────────────────
void TftDisplay::row(int16_t rowY, int16_t rowH, const char* label,
                     const char* value, bool measured) {
    const int16_t mid = (int16_t)(rowY + rowH / 2);

    tft.setTextSize(SZ_LABEL);
    tft.setTextColor(C_LABEL);
    tft.setCursor(PAD, (int16_t)(mid - GLYPH_H * SZ_LABEL / 2));
    tft.print(label);

    // A missing value is drawn dimmer than a real one, so a screen full of
    // dashes cannot be misread at a glance for a screen full of zeros.
    tft.setTextSize(SZ_VALUE);
    tft.setTextColor(measured ? C_VALUE : C_MISSING);
    tft.setCursor((int16_t)(_w - PAD - textW(value, SZ_VALUE)),
                  (int16_t)(mid - GLYPH_H * SZ_VALUE / 2));
    tft.print(value);
}

// ── which rule fired, and how much real data it rested on ─────────────
void TftDisplay::footer(const TftSnapshot& s) {
    const int16_t y = (int16_t)(_h - FOOTER_H + 3);
    tft.setTextSize(SZ_LABEL);

    // The rule name, but ONLY when a rule actually fired. gHit keeps the
    // last hit between verdicts, so showing it unconditionally would name a
    // feature that is not currently responsible for anything.
    const bool firing = (s.status >= STATUS_WARNING) && s.hit_feature[0];
    tft.setTextColor(firing ? C_VALUE : C_FOOT);
    tft.setCursor(PAD, y);
    tft.print(firing ? s.hit_feature : "-");

    // How many real samples the window behind those numbers contained.
    // A window built from 12 samples is a much weaker statement than one
    // built from 200, and the panel should say which one it is showing.
    char right[20];
    if (s.have_window && s.n_samples > 0) {
        snprintf(right, sizeof right, "n %u/%u",
                 (unsigned)s.n_samples, (unsigned)WINDOW_SAMPLES);
    } else {
        snprintf(right, sizeof right, "n ---");
    }
    tft.setTextColor(C_FOOT);
    tft.setCursor((int16_t)(_w - PAD - textW(right, SZ_LABEL)), y);
    tft.print(right);
}

#endif  // EDGE_TFT_ENABLED
