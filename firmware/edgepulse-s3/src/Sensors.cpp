#include "Sensors.h"
#include "pins.h"

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

static Adafruit_MPU6050   mpu;
static DHT                dht(PIN_DHT22, DHT22);
static OneWire            oneWire(PIN_DS18B20);
static DallasTemperature  ds18b20(&oneWire);

// Smallest plausible sanity band for an MPU6050 reading. Values outside
// this mean the bus is floating or the device is not really there, so we
// reject the sample rather than log noise as vibration.
static constexpr float MPU_ABS_LIMIT_G   = 16.0f;
static constexpr float MPU_ABS_LIMIT_DPS = 1000.0f;

// ── I²C bus scan ───────────────────────────────────────────────────────
// Addresses this project actually uses. A short targeted list keeps boot
// fast: on a floating bus every miss costs a full timeout.
static const uint8_t KNOWN_ADDRS[] = {0x68, 0x69, 0x40};

bool Sensors::i2cScan(uint8_t* out, uint8_t max) {
    uint8_t found = 0;

    for (uint8_t a : KNOWN_ADDRS) {
        if (found >= max) break;
        Wire.beginTransmission(a);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            out[found++] = a;
        }
    }

    if (_fullScan || found == 0) {
        for (uint16_t addr = 0x08; addr <= 0x77 && found < max; addr++) {
            bool known = false;
            for (uint8_t k : KNOWN_ADDRS) if (addr == k) { known = true; break; }
            if (known) continue;
            Wire.beginTransmission((uint8_t)addr);
            uint8_t err = Wire.endTransmission();
            if (err == 0) {
                out[found++] = (uint8_t)addr;
            }
        }
    }
    _i2cFound = found;
    return found > 0;
}

void Sensors::begin() {
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);
    pinMode(PIN_DS18B20, INPUT_PULLUP);
    pinMode(PIN_DHT22, INPUT_PULLUP);

    if (!Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000)) {
        Serial.println("  ! Wire.begin() failed");
    }
    Wire.setTimeOut(I2C_TIMEOUT_MS);
    Wire.setClock(100000);

    _i2cFound = 0;
    i2cScan(_i2cAddr, 8);

    // ── MPU6050 ────────────────────────────────────────────────────────
    // Address is 0x68 with AD0 low, 0x69 with AD0 high. Try both before
    // declaring it absent. Also try any other I2C address found.
    _mpuPresent = false;
    uint8_t candidates[4] = {0x68, 0x69, 0, 0};
    uint8_t cCount = 2;
    for (uint8_t i = 0; i < _i2cFound && cCount < 4; i++) {
        if (_i2cAddr[i] != 0x68 && _i2cAddr[i] != 0x69 && _i2cAddr[i] != 0x40) {
            candidates[cCount++] = _i2cAddr[i];
        }
    }

    for (uint8_t i = 0; i < cCount; i++) {
        uint8_t candidate = candidates[i];
        if (candidate == 0) continue;
        if (mpu.begin(candidate, &Wire)) {
            _mpuPresent = true;
            _mpuAddr = candidate;
            mpu.enableSleep(false);
            mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
            mpu.setGyroRange(MPU6050_RANGE_500_DEG);
            mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
            break;
        }
    }

    if (_mpuPresent) {
        mpu.enableSleep(false);
    }

    // ── DS18B20 ────────────────────────────────────────────────────────
    ds18b20.begin();
    ds18b20.setResolution(12);        // 0.0625 °C, ~750 ms conversion
    uint8_t dsCount = ds18b20.getDeviceCount();
    _dsPresent = (dsCount > 0);

    // ── DHT22 ──────────────────────────────────────────────────────────
    dht.begin();
    _dhtPresent = true;               // no addressable ID; validated per read

    // Prime the 1-Wire conversion so the first real reading is available
    // one conversion time from now rather than two.
    if (_dsPresent) {
        ds18b20.requestTemperatures();
        _dsConvStart  = millis();
        _dsConvBusy   = true;
        _dsLastMs     = millis();
    }
}

// ── MPU6050 ────────────────────────────────────────────────────────────
bool Sensors::readVibration(VibSample& out) {
    if (!_mpuPresent) return false;

    sensors_event_t a, g, temp;
    if (!mpu.getEvent(&a, &g, &temp)) {
        // A bus-level failure. This is the counter that distinguishes
        // "sensor is broken" from "gate is too strict" — with the old
        // all-zero rejection the two were indistinguishable.
        _mpuBusFails++;
        return false;
    }
    _mpuReads++;

    // Adafruit library gives acceleration in m/s^2, gyro in rad/s.
    // Convert acceleration to g and gyro to deg/s (dps).
    constexpr float RAD_TO_DEG_F = 57.2957795f;

    const float ax = a.acceleration.x / SENSORS_GRAVITY_STANDARD;
    const float ay = a.acceleration.y / SENSORS_GRAVITY_STANDARD;
    const float az = a.acceleration.z / SENSORS_GRAVITY_STANDARD;
    const float gx = g.gyro.x * RAD_TO_DEG_F;
    const float gy = g.gyro.y * RAD_TO_DEG_F;
    const float gz = g.gyro.z * RAD_TO_DEG_F;

    // ── Sanity gate ────────────────────────────────────────────────────
    // Rejects NaN and physically impossible values. It must NOT reject
    // small readings: at rest on a bench the AC vibration can be well
    // under 1 mg, and rejecting that would report "no data" forever.
    //
    // The ±4 g range is enforced by the sensor, so the ceiling here is
    // generous headroom, not a tuning knob.
    if (!isfinite(ax) || !isfinite(ay) || !isfinite(az) ||
        !isfinite(gx) || !isfinite(gy) || !isfinite(gz)) {
        _mpuRejects++;
        return false;
    }
    if (fabsf(ax) > MPU_ABS_LIMIT_G || fabsf(ay) > MPU_ABS_LIMIT_G ||
        fabsf(az) > MPU_ABS_LIMIT_G ||
        fabsf(gx) > MPU_ABS_LIMIT_DPS || fabsf(gy) > MPU_ABS_LIMIT_DPS ||
        fabsf(gz) > MPU_ABS_LIMIT_DPS) {
        _mpuRejects++;
        return false;
    }

    out.t_us = esp_timer_get_time();
    out.ax = ax; out.ay = ay; out.az = az;
    out.gx = gx; out.gy = gy; out.gz = gz;
    return true;
}

// Diagnostic counters, read by the status line. A bus failure and a
// rejected-but-plausible reading are different faults and must be
// distinguishable from the outside.
void Sensors::mpuCounters(uint32_t& reads, uint32_t& busFails, uint32_t& rejects) const {
    reads    = _mpuReads;
    busFails = _mpuBusFails;
    rejects  = _mpuRejects;
}

// ── DS18B20 — non-blocking, because 750 ms > the 10 ms tick ────────────
bool Sensors::pollMotorTemp() {
    if (!_dsPresent) return false;

    const uint32_t now = millis();

    if (_dsConvBusy) {
        if (now - _dsConvStart < DS18B20_INTERVAL_MS) return false;
        _dsConvBusy = false;

        float t = ds18b20.getTempCByIndex(0);
        if (isnan(t) || t < -55.0f || t > 125.0f) {
            _dsValid = false;        // bad read → no data, not zero
        } else {
            _dsTempC  = t;
            _dsValid  = true;
            _dsLastMs = now;
        }
        return _dsValid;
    }

    // Idle → kick off the next conversion.
    ds18b20.requestTemperatures();
    _dsConvStart = now;
    _dsConvBusy  = true;
    return false;
}

bool Sensors::readMotorTemp(float& outC, uint32_t& ageMs) {
    if (!_dsPresent || !_dsValid) { ageMs = 0; return false; }
    outC   = _dsTempC;
    ageMs  = millis() - _dsLastMs;
    return true;
}

// ── DHT22 ──────────────────────────────────────────────────────────────
// BLOCKING. Call from task_dht on CORE 0 only. This is the single place
// in the firmware permitted to stall the CPU for milliseconds.
bool Sensors::sampleAmbient(float& outTempC, float& outHumidity,
                            uint32_t& outAgeMs) {
    if (!_dhtPresent) { outAgeMs = 0; return false; }

    const float t = dht.readTemperature();
    if (isnan(t) || t < -40.0f || t > 80.0f) {
        outAgeMs = _ambValid ? (millis() - _ambLastMs) : 0;
        return false;
    }

    const float h = dht.readHumidity();
    if (isnan(h) || h < 0.0f || h > 100.0f) {
        outAgeMs = _ambValid ? (millis() - _ambLastMs) : 0;
        return false;
    }

    _ambTempC  = t;
    _ambHumPct = h;
    _ambLastMs = millis();
    _ambValid  = true;

    outTempC = _ambTempC;
    outHumidity = _ambHumPct;
    outAgeMs = 0;
    return true;
}

// NON-BLOCKING. Safe from any core, including the 100 Hz acquisition
// task. Returns the last real reading plus how stale it is.
bool Sensors::readAmbient(float& outTempC, float& outHumidity,
                          uint32_t& outAgeMs) {
    outAgeMs = _ambValid ? (millis() - _ambLastMs) : 0;
    if (!_dhtPresent || !_ambValid) return false;
    outTempC   = _ambTempC;
    outHumidity = _ambHumPct;
    return true;
}