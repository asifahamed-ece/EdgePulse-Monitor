// ═══════════════════════════════════════════════════════════════════════
//  EdgePulse — optional TFT status display  (ST7789 over SPI)
//  BOARD: Espressif ESP32-S3-DevKitC-1
//
//  WHAT THIS IS
//  A status panel, not a plotter. One word of verdict across the top and
//  the six numbers behind it. It is meant to be read from a metre away
//  while somebody else is holding the motor.
//
//  WHAT THIS IS NOT
//  It is not a second source of truth. Every number rendered here is a
//  copy of something another task already measured; this file never
//  computes, averages, holds for a while, or substitutes a reading. A
//  value that was not measured is drawn as "---" and never as 0 — see
//  Sensors.h and the project standing rules.
//
//  FEATURE GATE
//  EDGE_TFT_ENABLED defaults to 0. With it off, this header and
//  TftDisplay.cpp compile to nothing and main.cpp creates no task, so the
//  firmware is byte-identical to a build that never had the display.
//
//  Main.cpp includes this header UNCONDITIONALLY: the #define below has to
//  exist before main.cpp's own #if can be evaluated.
// ═══════════════════════════════════════════════════════════════════════

#pragma once

// ── The gate ──────────────────────────────────────────────────────────
// 0 = no display code, no task, no SPI traffic. Build this way by default.
#ifndef EDGE_TFT_ENABLED
#define EDGE_TFT_ENABLED 0
#endif

#if EDGE_TFT_ENABLED

#include <stdint.h>
#include <stddef.h>
#include "Rules.h"     // Status, statusName()

// ── Panel geometry ────────────────────────────────────────────────────
// ST7789 glass is sold in two practical shapes:
//     240x240  — 1.3" / 1.54". The controller still has 240x320 of memory
//               and the driver offsets the window down, so the geometry is
//               passed explicitly here rather than inferred.
//     240x320  — 1.47" / 1.69" / 1.9" / 2.0".
// begin() refuses anything else rather than silently centre-cropping,
// because a wrong guess here produces a picture that is merely plausible.
static constexpr uint16_t TFT_PANEL_W    = 240;
static constexpr uint16_t TFT_PANEL_H    = 240;
static constexpr uint8_t  TFT_ROTATION   = 0;

// Set to 0 when the module straps its backlight to 3V3 and PIN_TFT_BL is
// therefore left unwired. See include/pins.h.
#ifndef EDGE_TFT_BACKLIGHT
#define EDGE_TFT_BACKLIGHT 1
#endif

// ── Timing ────────────────────────────────────────────────────────────
// 2 Hz. Not a limit imposed by the panel — it is the right rate for a
// status word and the measured features only change every WINDOW_MS
// (2 s) anyway. Faster redraws would buy nothing and would keep the SPI
// bus busy on a core that also runs the DHT22 and the WebSocket.
//
// 16 MHz is comfortably inside the ST7789 write limit and the TFT is the
// only SPI device in this build, so a full redraw costs roughly 60-100 ms
// of CORE 0 time. That is a real but small duty cycle; see the note on
// task placement in main.cpp for why it cannot reach acquisition.
static constexpr uint32_t TFT_REDRAW_MS = 500;
static constexpr uint32_t TFT_SPI_HZ    = 16000000;

// ── The word across the top ───────────────────────────────────────────
// GOOD / FAULTY / DANGER map one-to-one onto the WARNING / CRITICAL /
// NORMAL distinction that Rules.h actually makes. The threshold system has
// two alert levels and one quiet level; inventing a third alert level here
// would require a threshold that no measurement supports.
//
// NO DATA is not a fourth verdict and carries no limit, no sigma band and
// no opinion about the motor. It exists because a panel that says "GOOD"
// while nothing has ever been measured is making a claim it has no
// evidence for — the single worst failure mode this project has already
// been pulled up for. See bannerFor() in TftDisplay.cpp.
enum TftBanner : uint8_t {
    BANNER_NO_DATA,   // the vibration channel has delivered nothing
    BANNER_GOOD,
    BANNER_FAULTY,
    BANNER_DANGER,
};

// ── One redraw's worth of state ───────────────────────────────────────
// A flat copy of things other tasks already measured. The display task
// fills this once per TFT_REDRAW_MS and hands it over; nothing else in the
// firmware reads or writes it.
struct TftSnapshot {
    // Latest feature row. NaN where the window had no input to compute
    // from — see Features::compute(), which leaves them NaN rather than 0.
    float vib_rms;          // window AC RMS, g
    float motor_temp_C;     // °C
    float ambient_temp_C;   // °C
    float humidity_pct;     // %

    // Validity, taken straight from the readers that produced the values
    // above. None of these is inferred from "the sensor was found at boot".
    bool    have_vib;
    bool    have_motor_temp;
    bool    have_ambient;
    uint8_t live_status;    // SensorBit mask, refreshed every acquisition tick

    // Verdict and its provenance.
    Status  status;
    bool    calibrated;     // a measured baseline is loaded
    char    hit_feature[24];// which feature fired, or "" for none

    // Acquisition health.
    float    hz;            // MEASURED rate, never the nominal HZ target
    uint32_t dropped;       // timer ticks the acquisition task did not serve
    bool     have_window;   // a feature window has actually been scored
    uint16_t n_samples;     // valid samples in the last window

    // Run context.
    bool     recording;
};

class TftDisplay {
public:
    // Brings up SPI and the panel. Safe to call only from CORE 0, and only
    // before the acquisition task matters (main.cpp calls it from tftTask,
    // never from setup()). Blocking for a few hundred ms.
    // Returns false when the panel geometry is not one the driver supports;
    // present() then stays false and show() becomes a no-op.
    bool begin();

    // One complete redraw. A no-op until begin() has succeeded.
    void show(const TftSnapshot& s);

    // True once the driver is initialised and the panel is being driven.
    //
    // NOTE ON WHAT THIS IS NOT: it is a DRIVER flag, not an electrical
    // probe. The ST7789 has no reliable presence handshake over a
    // floating-capable MISO line, so asserting "absent" would be an
    // unverified claim — and a wrong "absent" blacks out a panel that is
    // working. A panel that is not wired is immediately and obviously
    // blank, so there is nothing to gain by guessing.
    bool present() const { return _present; }

private:
    void splash();
    void banner(TftBanner b, const char* note);
    void row(int rowY, int rowH, const char* label, const char* value,
             bool measured);
    void footer(const TftSnapshot& s);

    bool     _present = false;
    uint16_t _w = 0, _h = 0;
};

#endif  // EDGE_TFT_ENABLED
