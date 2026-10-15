// ═══════════════════════════════════════════════════════════════════════
//  EdgePulse — threshold rules
//  BOARD: Espressif ESP32-S3-DevKitC-1
//
//  μ and σ come from real NORMAL runs (tools/calibrate_baseline.py).
//  The k-multipliers are declared policy, not discovery.
//
//  CRITICAL DESIGN RULE: if a baseline has not been calibrated from real
//  data, the evaluator returns UNCALIBRATED — never a confident-looking
//  NORMAL. Reporting "everything is normal" because no thresholds were
//  loaded is exactly the kind of false assurance this project exists to
//  avoid.
// ═══════════════════════════════════════════════════════════════════════

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>     // NAN
#include "Features.h"

enum Status : uint8_t {
    STATUS_UNCALIBRATED = 0,   // no measured baseline loaded
    STATUS_NORMAL       = 1,
    STATUS_WARNING      = 2,
    STATUS_CRITICAL     = 3,
};

const char* statusName(Status s);

struct RuleHit {
    char feature[24];    // which feature fired
    float value;
    float limit;

    // RuleHit carries a char array, so it is not an aggregate that can be
    // brace-assigned implicitly. Every field is set explicitly instead.
    RuleHit() : value(NAN), limit(NAN) { feature[0] = '\0'; }

    void set(const char* f, float v, float l) {
        strncpy(feature, f, sizeof(feature) - 1);
        feature[sizeof(feature) - 1] = '\0';
        value = v;
        limit = l;
    }
};

class ThresholdEvaluator {
public:
    // Load measured μ/σ from tools/calibrate_baseline.py.
    bool loadBaseline(uint8_t* buf, size_t len);
    bool calibrated() const { return _calibrated; }

    Status evaluate(const Features& f, RuleHit* hit = nullptr);

    // Declared policy, adjustable at runtime from the dashboard.
    void setWarningK(float k) { _kWarn = k; }
    void setCriticalK(float k) { _kCrit = k; }
    float warningK() const { return _kWarn; }
    float criticalK() const { return _kCrit; }

    // Absolute limits override the σ bands. Set to NAN to disable.
    void setTempCritC(float t) { _tempCritC = t; }

    const RuleHit& lastHit() const { return _lastHit; }

private:
    bool _calibrated = false;

    float _muRms = NAN, _sdRms = NAN;
    float _muVar = NAN, _sdVar = NAN;
    float _muPp  = NAN, _sdPp  = NAN;

    float _kWarn = 3.0f;
    float _kCrit = 5.0f;
    float _tempCritC = 80.0f;

    RuleHit _lastHit;
};