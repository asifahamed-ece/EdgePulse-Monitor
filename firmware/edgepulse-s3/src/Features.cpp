#include "Features.h"
#include <math.h>

void FeatureExtractor::reset() {
    _sumSq = _sumAbs = _peak = _min = _max = 0.0f;
    _n = 0;
}

void FeatureExtractor::addSample(float ax, float ay, float az) {
    // Magnitude, matching spec §6.2 exactly.
    const float mag = sqrtf(ax * ax + ay * ay + az * az);

    if (_n == 0) { _min = mag; _max = mag; }
    else { if (mag < _min) _min = mag; if (mag > _max) _max = mag; }

    _sumSq += mag * mag;
    _sumAbs += mag;
    _n++;
}

void FeatureExtractor::compute(uint16_t windowSamples, uint16_t windowMs,
                               uint8_t sensorStatus,
                               bool haveMotorTemp, float motorTempC,
                               bool haveAmbient, float ambientTempC,
                               float humidityPct,
                               bool haveCurrent, float currentA,
                               float busV, float watts,
                               Features& out) {
    // Start from "no data". Every field below is explicitly assigned only
    // when its inputs were actually measured.
    out.vib_rms          = NAN;
    out.vib_variance     = NAN;
    out.vib_peak_to_peak = NAN;
    out.current_mean_A   = NAN;
    out.current_rms_A    = NAN;
    out.current_variance = NAN;
    out.bus_V            = NAN;
    out.power_W          = NAN;
    out.motor_temp_C     = NAN;
    out.ambient_temp_C   = NAN;
    out.humidity_pct     = NAN;
    out.temp_delta_C     = NAN;
    out.n_samples        = _n;
    out.window_ms        = windowMs;
    out.sensor_status    = sensorStatus;
    out.valid            = false;

    const uint16_t n = _n;

    // Require at least half the nominal window so a window truncated by a
    // dropout is reported invalid rather than silently under-counted.
    if (n >= windowSamples / 2 && n > 0) {
        const double nF = (double)n;
        const double mean = _sumAbs / nF;

        // AC component: variance of (mag - mean), RMS of the same.
        const double var = (_sumSq / nF) - (mean * mean);

        out.vib_rms = sqrtf((float)(var > 0.0 ? var : 0.0));
        out.vib_variance = (float)(var > 0.0 ? var : 0.0);
        out.vib_peak_to_peak = _max - _min;
        out.valid = true;
    }

    // ── Temperatures ───────────────────────────────────────────────────
    // Copied verbatim, or left NaN. Never substituted.
    if (haveMotorTemp && isfinite(motorTempC)) out.motor_temp_C = motorTempC;
    if (haveAmbient  && isfinite(ambientTempC)) out.ambient_temp_C = ambientTempC;
    if (isfinite(humidityPct))                  out.humidity_pct = humidityPct;

    // ── Current ────────────────────────────────────────────────────────
    // Populated only when the CurrentSensor actually measured. A single
    // instantaneous sample is carried per window; mean/RMS/variance are
    // refined once the current is accumulated over the window rather than
    // sampled once at its end.
    if (haveCurrent && isfinite(currentA)) {
        out.current_mean_A = currentA;
        out.current_rms_A  = currentA;
        out.current_variance = 0.0f;
    }
    if (isfinite(busV))  out.bus_V  = busV;
    if (isfinite(watts)) out.power_W = watts;

    // A temperature rise above ambient is only meaningful when both
    // sides were actually measured. One missing sensor means no delta —
    // not a delta against zero.
    if (isfinite(out.motor_temp_C) && isfinite(out.ambient_temp_C)) {
        out.temp_delta_C = out.motor_temp_C - out.ambient_temp_C;
    }

    reset();
}