// ═══════════════════════════════════════════════════════════════════════
//  EdgePulse — CSV recorder
//  BOARD: Espressif ESP32-S3-DevKitC-1
//
//  Batched writes. A 100 Hz row with ~15 columns is 80–100 bytes, i.e.
//  8–10 kB/s — that saturates 115200 baud and strains 921600 once
//  headers are added. Rows are buffered and flushed as a block so the
//  acquisition loop never waits on the serial driver.
//
//  UNAVAILABLE IS BLANK, NEVER ZERO. A DHT22 that failed its checksum
//  writes an empty field. Writing 0 would assert a measurement of zero
//  degrees, which is a false claim.
// ═══════════════════════════════════════════════════════════════════════

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>     // NAN
#include "Sensors.h"

class CsvRecorder {
public:
    // Buffer size is declared before the static array that uses it.
    static constexpr size_t CAP = 4096;

    // The buffer is a file-scope static so a 4 KB object never lands on
    // any task's stack, and so the append helpers in the .cpp can size
    // their bounds without an extra parameter.
    static char gCsvBuf[CAP];

    CsvRecorder() : _buf(gCsvBuf) {}

    void begin();
    void beginRun(const char* runId, const char* condition);

    // Queue one frame. Flushes automatically when the buffer fills.
    void writeFrame(const Frame& f);

    void endRun();
    void flush();

    uint32_t rowsWritten() const { return _rows; }
    uint32_t rowsDropped() const { return _rowsDropped; }

    const char* runId() const { return _runId; }
    const char* condition() const { return _condition; }


private:
    char*   _buf;
    size_t  _len = 0;
    char   _runId[12]      = "-";
    char   _condition[16]  = "-";

    uint32_t _rows        = 0;
    uint32_t _rowsDropped = 0;
};