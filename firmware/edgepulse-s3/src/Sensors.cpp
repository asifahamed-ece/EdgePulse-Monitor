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
static constexpr float MPU_ABS_LIMIT_G   = 8.0f;
static constexpr float MPU_ABS_LIMIT_DPS = 400.0f;

// ── I²C bus scan ───────────────────────────────────────────────────────
// Addresses this project actually uses. A short targeted list keeps boot
// fast: on a floating bus every miss costs a full timeout.
static const uint8_t KNOWN_ADDRS[] = {0x68, 0x69, 0x40};

bool Sensors::i2cScan(uint8_t* out, uint8_t max) {
    uint8_t found = 0;

    for (uint8_t a : KNOWN_ADDRS) {
        if (found >= max) break;
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) out[found++] = a;
    }

    // Full 0x08..0x77 pass only on request — 112 probes x the timeout is
    // several seconds on an unterminated bus, so it is not on the boot path.
    if (_fullScan) {
        for (uint16_t addr = 0x08; addr <= 0x77 && found < max; addr++) {
            bool known = false;
            for (uint8_t k : KNOWN_ADDRS) if (addr == k) { known = true; break; }
            if (known) continue;
            Wire.beginTransmission((uint8_t)addr);
            if (Wire.endTransmission() == 0) out[found++] = (uint8_t)addr;
        }
    }
    return found > 0;
}

void Sensors::begin() {
    // ── CRITICAL ───────────────────────────────────────────────────────
    // Enable the internal pull-ups BEFORE initialising the bus.
    // With no MPU6050 connected and no external 4.7 kΩ fitted, SDA/SCL
    // float. The I2C master drives SCL low, releases it, then waits for the
    // line to return high — and it never does, so endTransmission() stalls
    // and setup() appears to hang. The internal pull-ups are weak (~45 kΩ),
    // which is exactly why the external 4.7 kΩ must still be fitted in the
    // final build; they exist here to make an unwired board safe to boot.
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);

    if (!Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000)) {
        Serial.println("  ! Wire.begin() failed");
    }
    // Bound every transaction. The core default is 50 ms, and x112
    // addresses that is well over five seconds of stalling.
    Wire.setTimeOut(I2C_TIMEOUT_MS);
    Wire.setClock(100000);

    _i2cFound = 0;
    i2cScan(_i2cAddr, 8);

    // ── MPU6050 ────────────────────────────────────────────────────────
    // Address is 0x68 with AD0 low, 0x69 with AD0 high. Try both before
    // declaring it absent.
    _mpuPresent = false;
    for (uint8_t candidate : {0x68, 0x69}) {
        if (mpu.begin(candidate, &Wire, I2C_TIMEOUT_MS)) {
            _mpuPresent = true;
            _mpuAddr = candidate;
            break;
        }
    }

    // Explicit ranges, recorded so the dataset states the settings rather
    // than leaving them at power-on defaults. The Adafruit Unified Sensor
    // API fixes these in begin(); they are ±2 g and ±250 °/s by default.
    if (_mpuPresent) {
        Serial.printf("MPU6050 ranges: accel=+-2g gyro=+-250dps "
                      "(Unified Sensor defaults)\n");
    }

    // ── DS18B20 ────────────────────────────────────────────────────────
    // 1-Wire enumeration on a floating DQ is safe but must be bounded: if
    // no device answers we simply continue without motor temperature.
    Serial.println("  probing DS18B20 (1-Wire)...");
    ds18b20.begin();
    ds18b20.setResolution(12);        // 0.0625 °C, ~750 ms conversion
    _dsPresent = (ds18b20.getDeviceCount() > 0);

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

    // The Unified Sensor API exposes the MPU's fixed ±2 g / ±250 °/s
    // ranges and returns SI units directly (m/s² and rad/s), so the
    // conversion to g and °/s happens here and is documented rather than
    // left implicit.
    sensors_event_t a{}, g{}, t{};
    if (!mpu.getEvent(&a, &g, &t)) return false;

    constexpr float M_S2_TO_G   = 1.0f / 9.80665f;
    constexpr float RAD_TO_DPS  = 57.2957795f;

    const float ax = a.acceleration.x * M_S2_TO_G;
    const float ay = a.acceleration.y * M_S2_TO_G;
    const float az = a.acceleration.z * M_S2_TO_G;
    const float gx = g.gyro.x * RAD_TO_DPS;
    const float gy = g.gyro.y * RAD_TO_DPS;
    const float gz = g.gyro.z * RAD_TO_DPS;

    // Reject physically impossible readings: a free MPU6050 cannot
    // exceed ~1 g plus gravity, and cannot spin at 400 °/s stationary.
    if (!isfinite(ax) || !isfinite(ay) || !isfinite(az) ||
        !isfinite(gx) || !isfinite(gy) || !isfinite(gz) ||
        fabsf(ax) > MPU_ABS_LIMIT_G || fabsf(ay) > MPU_ABS_LIMIT_G ||
        fabsf(az) > MPU_ABS_LIMIT_G ||
        fabsf(gx) > MPU_ABS_LIMIT_DPS || fabsf(gy) > MPU_ABS_LIMIT_DPS ||
        fabsf(gz) > MPU_ABS_LIMIT_DPS) {
        return false;
    }

    out.t_us = esp_timer_get_time();
    out.ax = ax; out.ay = ay; out.az = az;
    out.gx = gx; out.gy = gy; out.gz = gz;
    return true;
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

    const float t = dht.readTemperature();   // ~5 ms, non-interruptible
    const float h = dht.readHumidity();

    // The DHT22 reports failure as NaN and protects the payload with a
    // checksum. Treat NaN, or anything out of physical range, as
    // "no measurement this cycle" and leave the cache untouched.
    if (isnan(t) || isnan(h) || t < -40.0f || t > 80.0f ||
        h < 0.0f || h > 100.0f) {
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