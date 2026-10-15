// ═══════════════════════════════════════════════════════════════════════
//  EdgePulse — INA219 current sensor
//  BOARD: Espressif ESP32-S3-DevKitC-1
//
//  Replaces NullCurrentSensor. The interface and every consumer already
//  existed, so nothing else in the firmware changes — this is the
//  one-line swap the abstraction was designed for.
//
//  ─────────────────────────────────────────────────────────────────────
//  ⚠️  WIRING HAZARD — READ BEFORE CONNECTING
//  ─────────────────────────────────────────────────────────────────────
//  Many INA219 breakouts (CJMCU-519 and clones) pull SDA and SCL up to
//  **VCC**. Some variants wire them to **VIN** instead.
//
//  This module sits in SERIES with the motor, so VIN is the 12 V motor
//  bus. If the pull-ups go to VIN, 12 V is pushed straight onto GPIO 8
//  and GPIO 9 and the ESP32-S3 is destroyed instantly.
//
//  WITH THE 12 V ADAPTER DISCONNECTED:
//    1. Measure resistance from SDA to VIN. If it reads open (high
//       resistance), the pull-ups are on VIN — REMOVE those two
//       resistors and fit 4.7 kΩ to 3.3 V instead.
//    2. Measure resistance from SDA to VCC. That should be ~3.3 kΩ to
//       10 kΩ if the onboard pull-ups are fitted to VCC.
//    3. Only then connect the motor supply.
// ═══════════════════════════════════════════════════════════════════════

#pragma once
#include <stdint.h>
#include <math.h>
#include "CurrentSensor.h"

class Ina219 : public CurrentSensor {
public:
    Ina219(uint8_t addr = 0x40);

    bool begin(uint8_t addr);
    const char* name() const override { return _name; }
    bool present() const override { return _present; }

    bool read(float& outAmps, float& outVolts, float& outWatts) override;

    // Measured shunt value written into the calibration register. Reading
    // the marking on the board is mandatory: 0.1 Ω and 0.2 Ω modules are
    // both common, and a wrong calibration makes every current value a
    // confident lie.
    void setShuntOhms(float r) { _shuntOhms = r; _calibrated = false; }
    float shuntOhms() const { return _shuntOhms; }

    void setAverage(uint8_t n);   // 0..7, power of two, 1 = 12-bit single

private:
    void writeConfig(uint8_t cfg);
    void calibrate();

    uint16_t _read16(uint8_t reg);
    void    _write16(uint8_t reg, uint16_t val);

    uint8_t  _addr = 0x40;
    float    _shuntOhms = 0.1f;
    bool     _present = false;
    bool     _calibrated = false;
    char     _name[16] = "INA219";
};
