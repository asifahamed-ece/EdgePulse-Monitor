// ═══════════════════════════════════════════════════════════════════════
//  EdgePulse — measured baseline (firmware-side copy)
//
//  THIS FILE IS A PLACEHOLDER AND IS DELIBERATELY EMPTY OF NUMBERS.
//
//  It exists so the firmware has a loadBaseline() target, and so that
//  "not calibrated" is a normal, expected state rather than a crash.
//
//  Do NOT invent values here. μ and σ must come from real NORMAL runs
//  processed by tools/calibrate_baseline.py. Writing plausible-looking
//  numbers here would produce confident, wrong NORMAL/WARNING verdicts —
//  precisely the fabricated-result failure this project must not have.
//
//  To generate a real file:
//      python3 tools/calibrate_baseline.py \
//          --raw data/raw/*.csv --out firmware/edgepulse-s3/src/baseline.h
// ═══════════════════════════════════════════════════════════════════════

#pragma once

#define FS_BASELINE_JSON "{}"
#define FS_BASELINE_LEN  2