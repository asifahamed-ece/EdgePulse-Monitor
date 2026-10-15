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
            Serial.printf("  [I2C] ACK at 0x%02X\n", a);
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
                Serial.printf("  [I2C] ACK at 0x%02X\n", (uint8_t)addr);
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
    delay(20);

    Serial.printf("[DIAG] GPIO states (with internal pullup):\n");
    Serial.printf("  SDA (GPIO %d) = %d\n", PIN_I2C_SDA, digitalRead(PIN_I2C_SDA));
    Serial.printf("  SCL (GPIO %d) = %d\n", PIN_I2C_SCL, digitalRead(PIN_I2C_SCL));
    Serial.printf("  DS18B20 (GPIO %d) = %d\n", PIN_DS18B20, digitalRead(PIN_DS18B20));
    Serial.printf("  DHT22 (GPIO %d) = %d\n", PIN_DHT22, digitalRead(PIN_DHT22));

    if (!Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000)) {
        Serial.println("  ! Wire.begin() failed");
    }
    Wire.setTimeOut(I2C_TIMEOUT_MS);
    // 400 kHz. The MPU6050 and INA219 both support it, and a 14-byte read
    // every 10 ms at 100 kHz was consuming ~14% of the bus before Wi-Fi
    // traffic joined. 100 kHz was chosen defensively while chasing the
    // earlier stall and is no longer justified.
    Wire.setClock(400000);

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
        delay(20);

        Wire.beginTransmission(_mpuAddr);
        Wire.write(0x6B);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)_mpuAddr, (uint8_t)1);
        uint8_t pwr = Wire.available() ? Wire.read() : 0xFF;

        Wire.beginTransmission(_mpuAddr);
        Wire.write(0x75); // WHO_AM_I
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)_mpuAddr, (uint8_t)1);
        uint8_t who = Wire.available() ? Wire.read() : 0xFF;

        Serial.printf("MPU6050 configured: who=0x%02X pwr=0x%02X accel=+-4g gyro=+-500dps filter=44Hz at 0x%02X\n",
                      who, pwr, _mpuAddr);
    }

    // ── DS18B20 ────────────────────────────────────────────────────────
    pinMode(PIN_DS18B20, INPUT_PULLUP);
    Serial.println("  probing DS18B20 (1-Wire)...");
    uint8_t owReset = oneWire.reset();
    Serial.printf("  OneWire raw reset on GPIO %d returned: %u (1 = presence pulse detected)\n", PIN_DS18B20, owReset);

    uint8_t owRom[8] = {0};
    oneWire.reset_search();
    bool foundRom = oneWire.search(owRom);
    if (foundRom) {
        Serial.printf("  OneWire ROM found: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n",
                      owRom[0], owRom[1], owRom[2], owRom[3], owRom[4], owRom[5], owRom[6], owRom[7]);
    } else {
        Serial.println("  OneWire search found no device on GPIO 6.");
    }

    ds18b20.begin();
    ds18b20.setResolution(12);        // 0.0625 °C, ~750 ms conversion
    uint8_t dsCount = ds18b20.getDeviceCount();
    _dsPresent = (dsCount > 0);
    Serial.printf("  DS18B20 device count: %u\n", dsCount);

    // ── DHT22 ──────────────────────────────────────────────────────────
    pinMode(PIN_DHT22, INPUT_PULLUP);
    Serial.println("  probing DHT22 on GPIO 7...");
    pinMode(PIN_DHT22, OUTPUT);
    digitalWrite(PIN_DHT22, LOW);
    delay(20); // 20ms start pulse
    pinMode(PIN_DHT22, INPUT_PULLUP);
    uint32_t t0 = micros();
    bool pulledLow = false;
    uint32_t lowTimeUs = 0;
    while (micros() - t0 < 500) {
        if (digitalRead(PIN_DHT22) == 0) {
            pulledLow = true;
            lowTimeUs = micros() - t0;
            break;
        }
    }
    Serial.printf("  DHT22 handshake on GPIO 7: pulledLow=%d at %lu us (expect 1 at 20-50us)\n", pulledLow, (unsigned long)lowTimeUs);
    delay(100);

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

// ── MPU6050 register burst ─────────────────────────────────────────────
// WHY THIS IS HAND-ROLLED INSTEAD OF USING A LIBRARY
//
// Adafruit_MPU6050::_read() does this:
//
//     uint8_t buffer[14];
//     data_reg.read(buffer, 14);     // ← return status DISCARDED
//     rawAccX = buffer[0] << 8 | buffer[1];
//
// and getEvent() then unconditionally `return true;`.
//
// When the I2C transaction fails, `buffer` is never written, its contents
// are uninitialised stack memory, and that garbage is published as a valid
// measurement. The library has no way to report a bus error, which is why
// the fault counters could never move and why the vibration trace sat
// frozen at one exact triple — a stale buffer being re-read every 10 ms.
//
// Reading the registers directly makes the transfer status checkable. A
// short read is a real failure and is reported as one.

// ACCEL_OUT 0x3B, 14 contiguous bytes through GYRO_ZOUT_L.
static constexpr uint8_t MPU_REG_WHO_AM_I = 0x75;

// Scale factors must match the ranges configured in begin() (±4 g, ±500 dps).
// A mismatch scales every feature by a constant factor, which would still
// look plausible on a chart and be quietly wrong.
static constexpr float ACCEL_LSB_PER_G  = 8192.0f;   // ±4 g
static constexpr float GYRO_LSB_PER_DPS = 65.5f;     // ±500 dps

bool Sensors::readVibration(VibSample& out) {
    if (!_mpuPresent) return false;

    // ── WHO_AM_I liveness, amortised ──────────────────────────────────
    // Cheap, definitive proof the device is still on the bus. Once every
    // ~10 s is enough: it detects a disconnected or wedged sensor without
    // adding a transaction to every 100 Hz sample.
    const uint32_t now = millis();
    if (now - _lastWhoCheckMs >= MPU_WHO_INTERVAL_MS) {
        _lastWhoCheckMs = now;
        Wire.beginTransmission(_mpuAddr);
        Wire.write(MPU_REG_WHO_AM_I);
        if (Wire.endTransmission(false) != 0 ||
            Wire.requestFrom((uint8_t)_mpuAddr, (uint8_t)1) != 1) {
            _mpuBusFails++;
            _alive = false;
            return false;
        }
        const uint8_t who = Wire.read();
        // 0x68 on most boards; clones report 0x70/0x71/0x73. Anything
        // that is not 0x00 or 0xFF proves a real device answered.
        _alive     = (who != 0x00 && who != 0xFF);
        _whoAmI    = who;
        if (!_alive) { _mpuRejects++; return false; }
    } else if (!_alive) {
        return false;
    }

    // ── Single 14-byte burst: accel(6) + temp(2) + gyro(6) ───────────
    Wire.beginTransmission(_mpuAddr);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) { _mpuBusFails++; return false; }

    if (Wire.requestFrom((uint8_t)_mpuAddr, (uint8_t)14) != 14) {
        // A short read is a genuine bus failure. Reported as such rather
        // than parsed as data.
        _mpuBusFails++;
        return false;
    }

    uint8_t b[14];
    for (uint8_t i = 0; i < 14; i++) b[i] = Wire.read();

    // Big-endian signed 16-bit, exactly as laid out in the register map.
    const int16_t rawAx = (int16_t)((b[0]  << 8) | b[1]);
    const int16_t rawAy = (int16_t)((b[2]  << 8) | b[3]);
    const int16_t rawAz = (int16_t)((b[4]  << 8) | b[5]);
    const int16_t rawGx = (int16_t)((b[8]  << 8) | b[9]);
    const int16_t rawGy = (int16_t)((b[10] << 8) | b[11]);
    const int16_t rawGz = (int16_t)((b[12] << 8) | b[13]);

    const float ax = rawAx / ACCEL_LSB_PER_G;
    const float ay = rawAy / ACCEL_LSB_PER_G;
    const float az = rawAz / ACCEL_LSB_PER_G;
    const float gx = rawGx / GYRO_LSB_PER_DPS;
    const float gy = rawGy / GYRO_LSB_PER_DPS;
    const float gz = rawGz / GYRO_LSB_PER_DPS;

    _mpuReads++;

    // ── Sanity gate ───────────────────────────────────────────────────
    // Rejects NaN and values past the configured full scale. It must NOT
    // reject small readings: at rest on a bench the AC vibration can be
    // well under 1 mg, and discarding that would report "no data" forever.
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

    // ── ONE forced transaction, not two cached ones ───────────────────
    // Adafruit's DHT library blocks ~250 ms per real conversion and
    // caches for MIN_INTERVAL (2000 ms) internally. Calling
    // readTemperature() and readHumidity() separately therefore performs
    // the 250 ms conversion once and then spends another 250 ms
    // re-reading the value it already has.
    //
    // `force = true` makes the first call perform the transaction and the
    // second return instantly from the same conversion. Halves the block
    // and removes a confusing cache layer from the picture.
    const float t = dht.readTemperature(false, /*force=*/true);
    if (isnan(t) || t < -40.0f || t > 80.0f) {
        outAgeMs = _ambValid ? (millis() - _ambLastMs) : 0;
        return false;
    }

    const float h = dht.readHumidity(/*force=*/false);
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