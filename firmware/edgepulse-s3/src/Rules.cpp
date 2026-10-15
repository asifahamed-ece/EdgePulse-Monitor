#include "Rules.h"
#include "pins.h"
#include <ArduinoJson.h>
#include <math.h>
#include <string.h>

const char* statusName(Status s) {
    switch (s) {
        case STATUS_NORMAL:       return "NORMAL";
        case STATUS_WARNING:      return "WARNING";
        case STATUS_CRITICAL:     return "CRITICAL";
        default:                  return "UNCALIBRATED";
    }
}

bool ThresholdEvaluator::loadBaseline(uint8_t* buf, size_t len) {
    JsonDocument doc;
    if (deserializeJson(doc, buf, len)) return false;

    JsonObject b = doc["baseline"];
    if (b.isNull()) return false;

    // A baseline is only accepted if it was derived from real runs.
    _muRms = b["vib_rms"]["mu"]          | NAN;
    _sdRms = b["vib_rms"]["sd"]          | NAN;
    _muVar = b["vib_variance"]["mu"]     | NAN;
    _sdVar = b["vib_variance"]["sd"]     | NAN;
    _muPp  = b["vib_peak_to_peak"]["mu"] | NAN;
    _sdPp  = b["vib_peak_to_peak"]["sd"] | NAN;

    const int nRuns = b["n_runs"] | 0;

    // Refuse to act on a baseline built from fewer than 3 independent runs.
    if (nRuns < 3 || !isfinite(_muRms) || !isfinite(_sdRms) || _sdRms <= 0.0f) {
        return false;
    }

    _calibrated = true;
    return true;
}

Status ThresholdEvaluator::evaluate(const Features& f, RuleHit* outHit) {
    // No measured baseline → no opinion. Never a false "NORMAL".
    if (!_calibrated || !f.valid) {
        _lastHit.set("-", NAN, NAN);
        if (outHit) *outHit = _lastHit;
        return STATUS_UNCALIBRATED;
    }

    Status worst = STATUS_NORMAL;
    RuleHit worstHit;

    struct { const char* name; float v, mu, sd; } checks[] = {
        {"vib_rms",          f.vib_rms,          _muRms, _sdRms},
        {"vib_variance",     f.vib_variance,     _muVar, _sdVar},
        {"vib_peak_to_peak", f.vib_peak_to_peak, _muPp,  _sdPp },
    };

    for (auto& c : checks) {
        // Skip features whose inputs were never measured.
        if (!isfinite(c.v)) continue;

        const float warnLim = c.mu + _kWarn * c.sd;
        const float critLim = c.mu + _kCrit * c.sd;

        if (c.v >= critLim) {
            worst = STATUS_CRITICAL;
            worstHit.set(c.name, c.v, critLim);
            break;                      // critical dominates; stop here
        }
        if (c.v >= warnLim && worst < STATUS_WARNING) {
            worst = STATUS_WARNING;
            worstHit.set(c.name, c.v, warnLim);
        }
    }

    // Absolute thermal limit, independent of the σ bands.
    if (isfinite(f.motor_temp_C) && f.motor_temp_C >= _tempCritC) {
        worst = STATUS_CRITICAL;
        worstHit.set("motor_temp_C", f.motor_temp_C, _tempCritC);
    }

    _lastHit = worstHit;
    if (outHit) *outHit = _lastHit;
    return worst;
}