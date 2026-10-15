// ═══════════════════════════════════════════════════════════════════════
//  EdgePulse — window feature extraction
//  BOARD: Espressif ESP32-S3-DevKitC-1
//
//  MUST match tools/extract_features.py bit-for-bit. The host version is
//  the reference; a mismatch is a bug, not a rounding difference
//  (design spec gate G4).
//
//  GRAVITY HANDLING — fixed decision, spec §6.2:
//  Features are computed on the AC component. Magnitude is formed, then
//  the window mean is subtracted before RMS/variance/peak-to-peak.
//  This is orientation-independent: mounting angle is not fixed between
//  runs, so subtracting 1 g from a named axis would be wrong.
// ═══════════════════════════════════════════════════════════════════════

#pragma once
#include <stdint.h>

struct Features {
    // vibration, computed on the AC component
    float vib_rms;
    float vib_variance;
    float vib_peak_to_peak;

    // current — deferred with the INA219; NaN, never 0
    float current_mean_A;
    float current_rms_A;
    float current_variance;

    // temperatures and humidity, last valid values in the window
    float motor_temp_C;
    float ambient_temp_C;
    float humidity_pct;
    float temp_delta_C;      // valid only when BOTH temperatures are

    // provenance — every row carries its own evidence
    uint16_t n_samples;      // ACTUAL valid samples, exposes dropouts
    uint16_t window_ms;
    uint8_t  sensor_status;
    bool     valid;          // false if the window had too few samples
};

class FeatureExtractor {
public:
    void reset();

    // Accumulate one real sample. ax/ay/az in g.
    void addSample(float ax, float ay, float az);

    // Fill `out` from the accumulated window. Resets the accumulator.
    // Features whose inputs were never measured stay NaN.
    //
    // The slow-sensor values are passed in rather than fetched here: this
    // runs on CORE 1 and must not trigger a blocking read. temp_delta_C is
    // only computed when BOTH temperatures are real — a delta against an
    // absent sensor would be a fabricated number.
    void compute(uint16_t windowSamples, uint16_t windowMs,
                 uint8_t sensorStatus,
                 bool haveMotorTemp, float motorTempC,
                 bool haveAmbient, float ambientTempC, float humidityPct,
                 bool haveCurrent, float currentA,
                 Features& out);

    uint16_t count() const { return _n; }

private:
    double   _sumSq  = 0.0;   // Σ ac², for RMS and variance (double precision avoids cancellation)
    double   _sumAbs = 0.0;   // for mean-absolute deviation
    float    _peak   = 0.0f;
    float    _min    = 0.0f;
    float    _max    = 0.0f;
    uint16_t _n      = 0;
};