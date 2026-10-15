// ═══════════════════════════════════════════════════════════════════════
//  EdgePulse — sensors
//  BOARD: Espressif ESP32-S3-DevKitC-1
//
//  HARD RULE OF THIS FILE: a value is only ever returned if it was
//  actually measured. Every read reports success or failure explicitly.
//  A sensor that is absent, unaddressed, or returning garbage returns
//  `false` and its value is left untouched — never zero-filled.
//
//  Empty ≠ zero. The jury rejected synthesised data once; this file
//  exists to make that class of failure structurally impossible.
// ═══════════════════════════════════════════════════════════════════════

#pragma once
#include <stdint.h>
#include <math.h>     // NAN, isfinite

// ── Sensor presence and validity bitmask ──────────────────────────────
// Row-level `sensor_status` in the CSV is this mask.
enum SensorBit : uint8_t {
    SENSOR_MPU6050   = 1 << 0,   // vibration
    SENSOR_DS18B20   = 1 << 1,   // motor surface temperature
    SENSOR_DHT22     = 1 << 2,   // ambient temperature + humidity
    SENSOR_CURRENT   = 1 << 3,   // reserved — INA219 deferred
};

// ── One vibration sample ──────────────────────────────────────────────
struct VibSample {
    uint32_t t_us;      // real timestamp from esp_timer_get_time()
    float    ax, ay, az;
    float    gx, gy, gz;
};

// ── One complete acquisition frame ────────────────────────────────────
// Slow sensors are NOT resampled. Each carries the real timestamp at
// which it was last read, plus a validity flag, so downstream code can
// never mistake a 0.5 Hz reading for a 100 Hz one.
struct Frame {
    uint32_t t_us;

    bool     have_vib;
    float    ax, ay, az;
    float    gx, gy, gz;

    bool     have_motor_temp;
    float    motor_temp_C;
    uint32_t motor_temp_age_ms;   // how stale this value is

    bool     have_ambient;
    float    ambient_temp_C;
    float    humidity_pct;
    uint32_t ambient_age_ms;

    // Current channel. Deferred with the INA219: absent in Milestone 1,
    // and written as BLANK CSV fields while so.
    bool     have_current;
    float    current_A;
    float    bus_V;
    float    power_W;

    uint8_t  sensor_status;       // SensorBit mask
    uint32_t dropped_samples;
};

class Sensors {
public:
    void begin();

    // Reads the MPU6050 only. Returns false if absent, if the burst
    // transfer was short, or if the reading was implausible.
    bool readVibration(VibSample& out);

    // Diagnostic counters. Distinguishes an I²C bus failure from a
    // reading that arrived but was rejected — two very different hardware
    // faults that otherwise look identical from the outside.
    void mpuCounters(uint32_t& reads, uint32_t& busFails,
                     uint32_t& rejects) const {
        reads = _mpuReads; busFails = _mpuBusFails; rejects = _mpuRejects;
    }
    bool mpuAlive() const { return _alive; }
    uint8_t mpuWhoAmI() const { return _whoAmI; }

    // Non-blocking 1-Wire: starts conversion, reports whether it finished.
    // A 12-bit conversion takes ~750 ms, so this is called far more often
    // than the temperature actually changes.
    bool pollMotorTemp();
    bool readMotorTemp(float& outC, uint32_t& ageMs);

    // ── DHT22 ──────────────────────────────────────────────────────────
    // The DHT22 read BLOCKS for ~5 ms (it is microsecond-timing-sensitive
    // and cannot be interrupted). It must therefore run ONLY on the
    // non-real-time core. Two entry points keep that separation honest:
    //
    //   sampleAmbient() — BLOCKING. Called by task_dht on CORE 0 only.
    //   readAmbient()   — non-blocking. Reads the cache. Safe from any
    //                    core, including the acquisition task on CORE 1.
    //
    // Conflating the two previously put a 5 ms stall inside the 10 ms
    // acquisition period, which is exactly what spec §4.1 forbids.
    bool sampleAmbient(float& outTempC, float& outHumidity, uint32_t& outAgeMs);
    bool readAmbient(float& outTempC, float& outHumidity, uint32_t& outAgeMs);

    // Presence flags, fixed at begin() but re-checked by the scanner.
    bool hasMPU6050() const { return _mpuPresent; }
    bool hasDS18B20() const { return _dsPresent; }
    bool hasDHT22()   const { return _dhtPresent; }

    // Raw I2C scan results, used by the boot report.
    uint8_t i2cCount() const { return _i2cFound; }
    uint8_t i2cAddrAt(uint8_t i) const { return _i2cAddr[i]; }
    uint8_t mpuAddr() const { return _mpuAddr; }

    // Per-transaction timeout. The core default is 50 ms, which is far too
    // long when every address misses on a floating bus.
    static constexpr uint16_t I2C_TIMEOUT_MS = 20;

    // Full 0x08..0x77 bus sweep. Off by default: 112 probes on an
    // unterminated bus costs seconds. Turn on only to hunt an unknown
    // device, never during normal boot.
    void setFullScan(bool on) { _fullScan = on; }

private:
    bool i2cScan(uint8_t* out, uint8_t max);

    bool     _mpuPresent = false;
    bool     _dsPresent  = false;
    bool     _dhtPresent = false;
    uint8_t  _mpuAddr    = 0;

    uint8_t  _i2cFound = 0;
    uint8_t  _i2cAddr[8] = {0};
    bool     _fullScan  = false;

    // MPU6050 diagnostics. These distinguish three genuinely different
    // faults that otherwise look identical from the outside:
    //   busFails — NAK, or a short read; the device is not answering
    //   rejects  — data arrived but was implausible; sensor or wiring fault
    //   alive    — WHO_AM_I no longer answers at all
    volatile uint32_t _mpuReads    = 0;
    volatile uint32_t _mpuBusFails = 0;
    volatile uint32_t _mpuRejects  = 0;
    bool     _alive          = false;
    uint8_t  _whoAmI         = 0;
    uint32_t _lastWhoCheckMs = 0;

    // WHO_AM_I is polled this often: it proves the device is still on the
    // bus without adding a transaction to every 100 Hz sample.
    static constexpr uint32_t MPU_WHO_INTERVAL_MS = 10000;

    uint32_t _dsLastMs    = 0;
    uint32_t _dsConvStart = 0;
    float    _dsTempC     = NAN;
    bool     _dsValid     = false;
    bool     _dsConvBusy  = false;

    // DHT22 cache, written only by task_dht on CORE 0.
    float    _ambTempC  = NAN;
    float    _ambHumPct = NAN;
    uint32_t _ambLastMs = 0;
    bool     _ambValid  = false;
};