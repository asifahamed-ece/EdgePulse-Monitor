#include "CsvRecorder.h"
#include "pins.h"
#include <Arduino.h>
#include <math.h>

// Static buffer, defined once in exactly one translation unit.
char CsvRecorder::gCsvBuf[CsvRecorder::CAP];

// Append helpers. Kept tiny and bounded — this runs on the acquisition
// core and must not allocate.
static void appStr(char*& b, size_t& n, const char* s) {
    while (*s && n < CsvRecorder::CAP - 1) b[n++] = *s++;
}
static void appChar(char*& b, size_t& n, char c) { if (n < CsvRecorder::CAP - 1) b[n++] = c; }
static void appU32(char*& b, size_t& n, uint32_t v) {
    char tmp[11]; int i = 0;
    if (v == 0) tmp[i++] = '0';
    while (v && i < 10) { tmp[i++] = char('0' + v % 10); v /= 10; }
    while (i-- > 0) appChar(b, n, tmp[i]);
}
static void appF(char*& b, size_t& n, float v, int dp) {
    if (!isfinite(v)) return;                 // blank, NOT "0.00"
    char tmp[24];
    snprintf(tmp, sizeof(tmp), "%.*f", dp, (double)v);
    for (const char* p = tmp; *p; ++p) appChar(b, n, *p);
}

void CsvRecorder::begin() {
    _len = 0; _rows = 0; _rowsDropped = 0;
    Serial.println("# ── EdgePulse raw CSV ──");
    Serial.println(F("t_us,run_id,condition,pwm,motor_running,"
                     "ax,ay,az,gx,gy,gz,"
                     "current_raw,current_A,"
                     "motor_temp_C,motor_temp_age_ms,"
                     "ambient_temp_C,ambient_age_ms,humidity_pct,"
                     "temp_delta_C,sensor_status,dropped_samples"));
}

void CsvRecorder::beginRun(const char* runId, const char* condition) {
    flush();
    snprintf(_runId, sizeof(_runId), "%s", runId);
    snprintf(_condition, sizeof(_condition), "%s", condition);
    _rows = 0;
    Serial.printf("# RUN START id=%s condition=%s t=%lu ms\n",
                  _runId, _condition, millis());
}

void CsvRecorder::writeFrame(const Frame& f) {
    // Motor state is filled in by main() via the global accessor.
    extern uint8_t gMotorPwm;
    extern uint8_t gMotorRun;
    extern uint8_t gMotorDir;

    appU32(_buf, _len, f.t_us);            appChar(_buf, _len, ',');
    appStr(_buf, _len, _runId);            appChar(_buf, _len, ',');
    appStr(_buf, _len, _condition);        appChar(_buf, _len, ',');
    appU32(_buf, _len, gMotorPwm);         appChar(_buf, _len, ',');
    appU32(_buf, _len, gMotorRun);         appChar(_buf, _len, ',');

    if (f.have_vib) {
        appF(_buf, _len, f.ax, 4); appChar(_buf, _len, ',');
        appF(_buf, _len, f.ay, 4); appChar(_buf, _len, ',');
        appF(_buf, _len, f.az, 4); appChar(_buf, _len, ',');
        appF(_buf, _len, f.gx, 4); appChar(_buf, _len, ',');
        appF(_buf, _len, f.gy, 4); appChar(_buf, _len, ',');
        appF(_buf, _len, f.gz, 4);
    } else {
        for (int i = 0; i < 5; ++i) appChar(_buf, _len, ',');
    }
    appChar(_buf, _len, ',');

    // Current is behind the CurrentSensor interface (spec §3.3). With the
    // INA219 deferred the fields stay EMPTY — a blank column is the honest
    // representation of "not measured". Writing 0.0 would claim zero amps.
    if (f.have_current) {
        appF(_buf, _len, f.current_A, 4); appChar(_buf, _len, ',');
        appF(_buf, _len, f.bus_V, 3);
    } else {
        appChar(_buf, _len, ',');
        appChar(_buf, _len, ',');
    }
    appChar(_buf, _len, ',');

    if (f.have_motor_temp) {
        appF(_buf, _len, f.motor_temp_C, 4);   appChar(_buf, _len, ',');
        appU32(_buf, _len, f.motor_temp_age_ms);
    } else {
        appChar(_buf, _len, ',');
        appChar(_buf, _len, ',');
    }
    appChar(_buf, _len, ',');

    if (f.have_ambient) {
        appF(_buf, _len, f.ambient_temp_C, 4); appChar(_buf, _len, ',');
        appU32(_buf, _len, f.ambient_age_ms);   appChar(_buf, _len, ',');
        appF(_buf, _len, f.humidity_pct, 2);
    } else {
        appChar(_buf, _len, ',');
        appChar(_buf, _len, ',');
        appChar(_buf, _len, ',');
    }
    appChar(_buf, _len, ',');

    // temp_delta_C only when BOTH temperatures are real.
    if (f.have_motor_temp && f.have_ambient) {
        appF(_buf, _len, f.motor_temp_C - f.ambient_temp_C, 4);
    }
    appChar(_buf, _len, ',');

    appU32(_buf, _len, f.sensor_status);  appChar(_buf, _len, ',');
    appU32(_buf, _len, f.dropped_samples);
    appChar(_buf, _len, '\n');

    _rows++;

    if (_len >= CAP - 128) flush();
}

void CsvRecorder::flush() {
    if (_len == 0) return;
    Serial.write((const uint8_t*)_buf, _len);
    _len = 0;
}

void CsvRecorder::endRun() {
    flush();
    Serial.printf("# RUN END id=%s rows=%lu dropped_buffer=%lu\n",
                  _runId, (unsigned long)_rows, (unsigned long)_rowsDropped);
}