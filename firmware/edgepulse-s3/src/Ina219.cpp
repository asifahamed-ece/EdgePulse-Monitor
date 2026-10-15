#include "Ina219.h"
#include "pins.h"
#include <Wire.h>
#include <Arduino.h>

// ── Register map (TI SBOS448G) ─────────────────────────────────────────
static constexpr uint8_t REG_CONFIG      = 0x00;
static constexpr uint8_t REG_SHUNTVOLT   = 0x01;   // signed, LSB = 10 µV
static constexpr uint8_t REG_BUSVOLT     = 0x02;   // LSB = 4 mV
static constexpr uint8_t REG_POWER       = 0x03;
static constexpr uint8_t REG_CURRENT     = 0x04;   // signed
static constexpr uint8_t REG_CALIB       = 0x05;

static constexpr float SHUNT_LSB_V  = 10e-6f;   // 10 µV per LSB
static constexpr float BUS_LSB_V    = 4e-3f;    // 4 mV per LSB
static constexpr float CURRENT_LSB_A = 1e-3f;   // derived after calibration

Ina219::Ina219(uint8_t addr) : _addr(addr) {}

uint16_t Ina219::_read16(uint8_t reg) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0;
    if (Wire.requestFrom(_addr, (uint8_t)2) != 2) return 0;
    uint16_t hi = Wire.read();
    uint16_t lo = Wire.read();
    return (uint16_t)((hi << 8) | lo);
}

void Ina219::_write16(uint8_t reg, uint16_t val) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write((uint8_t)(val >> 8));
    Wire.write((uint8_t)(val & 0xFF));
    Wire.endTransmission(true);
}

bool Ina219::begin(uint8_t addr) {
    _addr = addr;

    // Presence probe. A NAK here means absent, not broken — and the
    // caller must not proceed to trust any reading.
    Wire.beginTransmission(_addr);
    if (Wire.endTransmission(true) != 0) return false;

    snprintf(_name, sizeof(_name), "INA219@0x%02X", _addr);

    // Bus range 32 V (bit 13 = 1) so a 12 V transient cannot saturate the
    // bus reading. Shunt PGA /8 = ±320 mV, which with a 0.1 Ω shunt covers
    // ±3.2 A. BADC/SADC = 12-bit, continuous mode.
    //
    // NOTE: the configuration register is 16 bits, but it is written one
    // byte at a time with only the low byte changing here. The reset
    // value is 0x399F — writing that as a single uint8_t truncates it to
    // 0x9F, which would leave the high byte at whatever it already was.
    // Only the high byte actually differs from reset (RST=0, BRNG=1,
    // PG=11, BADC=1111, SADC=1111, MODE=111), so the high byte is
    // written explicitly and the low byte left at its reset value.
    Wire.beginTransmission(_addr);
    Wire.write(REG_CONFIG);
    Wire.write(0x39);        // high byte: BRNG=1, PG=÷8, BADC/SADC=1111, MODE=111
    Wire.write(0x9F);        // low byte: unchanged from reset
    Wire.endTransmission(true);
    _present = true;
    calibrate();
    return _present;
}

void Ina219::writeConfig(uint8_t cfg) { _write16(REG_CONFIG, cfg); }

void Ina219::setAverage(uint8_t n) {
    if (!_present) return;
    // SADC occupies bits 7..10 of the LOW byte. 1 = 12-bit, 4 = averaged
    // x4, 7 = averaged x128. Applied to the low byte only.
    Wire.beginTransmission(_addr);
    Wire.write(REG_CONFIG);
    Wire.write(0x39);
    Wire.write((uint8_t)(0x9F & (uint8_t)~(0x0F << 5) | ((n & 0x07) << 5)));
    Wire.endTransmission(true);
}

void Ina219::calibrate() {
    // Datasheet: trunc_shunt_current = trunc(shunt_resistor_on / rshunt_adc_scale)
    // then write that to the calibration register. With 0.1 Ω the
    // resulting LSB is 10 µV / 0.1 Ω = 0.1 mA, which is the resolution
    // quoted in the design spec.
    const float lsb_mA = (SHUNT_LSB_V * 1000.0f) / _shuntOhms;
    uint16_t cal = (uint16_t)(1000.0f / lsb_mA);   // 1000 mA full scale
    _write16(REG_CALIB, cal);
    _calibrated = true;
}

bool Ina219::read(float& outAmps, float& outVolts, float& outWatts) {
    outAmps = NAN; outVolts = NAN; outWatts = NAN;
    if (!_present) return false;

    const int16_t shunt = (int16_t)_read16(REG_SHUNTVOLT);
    const uint16_t bus = _read16(REG_BUSVOLT);
    const int16_t cur  = (int16_t)_read16(REG_CURRENT);

    // Bus: bit 3 is CNVR and is always 1 — it must be masked off.
    const float volts  = (float)(bus & 0x7FFF) * BUS_LSB_V;
    const float shuntV = (float)shunt * SHUNT_LSB_V;
    const float amps   = (float)cur * CURRENT_LSB_A;

    // Reject implausible readings rather than publishing them. The motor
    // bus is 12 V nominal and the L298N channel is rated 2 A.
    if (!isfinite(volts) || !isfinite(amps)) return false;
    if (volts < 0.0f || volts > 26.0f)  return false;   // 26 V is the IC abs max
    if (fabsf(amps) > 10.0f)            return false;   // well past any 2 A load

    outAmps  = amps;
    outVolts = volts;
    outWatts = volts * fabsf(amps);
    (void)shuntV;   // available for a future per-sample power integration
    return true;
}
