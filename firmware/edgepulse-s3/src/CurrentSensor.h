// ═══════════════════════════════════════════════════════════════════════
//  EdgePulse — current sensor abstraction
//  BOARD: Espressif ESP32-S3-DevKitC-1
//
//  WHY THIS FILE EXISTS
//  The INA219 is deferred out of Milestone 1 (see spec §3.3). Rather than
//  wire current sensing in later and touch every consumer — features,
//  thresholds, CSV schema, dashboard — the current channel is defined as
//  an interface now and filled with a null implementation.
//
//  To add the INA219 later:
//    1. implement CurrentSensor with a driver
//    2. call setCurrentSensor(&ina219) once at boot
//  Nothing else changes. Features, Rules, CsvRecorder and the dashboard
//  already consume the interface.
//
//  CONTRACT
//  read() returns false when no valid current measurement exists. The
//  caller then writes a BLANK CSV field and leaves the feature NaN.
//  Returning 0.0 would assert "zero amps measured", which is a different
//  and much stronger claim than "not measured".
// ═══════════════════════════════════════════════════════════════════════

#pragma once
#include <stdint.h>
#include <math.h>   // NAN

class CurrentSensor {
public:
    virtual ~CurrentSensor() {}

    // Human-readable name, e.g. "INA219@0x40" or "null".
    virtual const char* name() const = 0;

    // True once the device has been identified and configured.
    virtual bool present() const = 0;

    // Non-blocking read. Must be safe to call from the acquisition task.
    virtual bool read(float& outAmps, float& outVolts, float& outWatts) = 0;
};

// ── Milestone 1 implementation ─────────────────────────────────────────
// Reports "not present" and yields NaN. Every downstream consumer then
// leaves its current fields empty, which is the correct representation of
// a deferred sensor.
class NullCurrentSensor : public CurrentSensor {
public:
    const char* name() const override { return "null"; }
    bool present() const override { return false; }

    bool read(float& outAmps, float& outVolts, float& outWatts) override {
        outAmps = NAN;
        outVolts = NAN;
        outWatts = NAN;
        return false;
    }
};

// Single global accessor. Setting a new sensor is a one-line change at
// boot; no consumer is ever edited.
void        setCurrentSensor(CurrentSensor* s);
CurrentSensor* currentSensor();
