// ═══════════════════════════════════════════════════════════════════════
//  EdgePulse — ESP32-S3 main firmware
//  BOARD: Espressif ESP32-S3-DevKitC-1  (PlatformIO: esp32-s3-devkitc-1)
//
//  ┌── Sensors ───────────────────────────────────────────────────────┐
//  │ MPU6050  I²C  GPIO8/9    100 Hz   vibration                     │
//  │ DS18B20  1-Wire GPIO6   ~1.3 Hz  motor surface temperature      │
//  │ DHT22    GPIO7          0.5 Hz   ambient temperature, humidity  │
//  │ L298N    GPIO4/17/18    PWM      motor control, ESP32 drives it │
//  └──────────────────────────────────────────────────────────────────┘
//
//  TASK PLACEMENT (spec §4.1). The rule that matters:
//  acquisition timing is NEVER gated by a slow sensor or a slow consumer.
//  A DHT22 read blocks ~5 ms against a 10 ms acquisition period, so it
//  lives on the other core entirely.
//
//  CORE 1 — real time, nothing blocking
//    task_acq      100 Hz  high prio  MPU6050 burst → frame → CSV
//    task_feature  2 s              accumulate → compute features
//  CORE 0 — everything else
//    task_dht      0.5 Hz low prio   DHT22 (blocks ~5 ms)
//    task_1wire    polled            DS18B20 (750 ms conversion)
//    task_rules    on feature        thresholds → status
//    task_ws       10 Hz             JSON → browser
//
//  NO SYNTHETIC DATA. Every number reaching the dashboard came from a
//  real sensor read. A sensor that is absent or fails validation leaves
//  its field empty and sets a bit in sensor_status. The dashboard is told
//  so explicitly. If nothing is measured, nothing is shown.
// ═══════════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

#include "pins.h"
#include "Sensors.h"
#include "Features.h"
#include "Rules.h"
#include "CsvRecorder.h"
#include "CurrentSensor.h"
#include "MotorDriver.h"
#include "baseline.h"

// ── Globals shared across tasks ────────────────────────────────────────
// Access is single-writer / multi-reader and each field is naturally
// word-sized, so no critical section is needed. The frame itself is
// passed by queue.
uint8_t gMotorPwm = 0;
uint8_t gMotorRun = 0;
uint8_t gMotorDir = 0;

// ── Objects ────────────────────────────────────────────────────────────
static Sensors          sensors;
static MotorDriver      motor;
static FeatureExtractor extractor;
static ThresholdEvaluator evaluator;
static CsvRecorder      csv;

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

static constexpr char AP_SSID[] = "EdgePulse";
static constexpr char AP_PASS[] = "edgepulse123";

// ── Shared run state ───────────────────────────────────────────────────
static char     gRunId[12]     = "R001";
static char     gCondition[16] = "NORMAL";
static bool     gRecording     = false;
static Status   gStatus        = STATUS_UNCALIBRATED;
static RuleHit  gHit;
static uint32_t gDropped       = 0;
static float    gHz            = 0.0f;

// Latest feature row, for the dashboard and the debug line.
static Features gFeat{};
static uint32_t gFeatSeq = 0;
static VibSample gLatestVib{};
// Per-frame validity mask, refreshed every acquisition tick so the
// dashboard reports what is actually arriving right now.
static volatile uint8_t gLiveStatus = 0;

// ── FreeRTOS handles ───────────────────────────────────────────────────
static QueueHandle_t   qFeature;      // Features  → task_rules
static SemaphoreHandle_t mFeature;    // guards gFeat / gFeatSeq
static volatile uint32_t acqTicks = 0;   // expected
static volatile uint32_t acqDone  = 0;   // completed acquisitions

// ═══════════════════════════════════════════════════════════════════════
//  ACQUISITION — CORE 1, HIGH PRIORITY
// ═══════════════════════════════════════════════════════════════════════

static void acqTask(void*) {
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / HZ));

        Frame f{};
        f.t_us = esp_timer_get_time();

        // The current channel is read once, here, and its result stored on
        // the frame. It was previously read in two places.
        float amps, volts, watts;
        bool haveI = currentSensor()->read(amps, volts, watts);

        VibSample vs;
        f.have_vib = sensors.readVibration(vs);
        if (f.have_vib) {
            gLatestVib = vs;
            f.ax = vs.ax; f.ay = vs.ay; f.az = vs.az;
            f.gx = vs.gx; f.gy = vs.gy; f.gz = vs.gz;
            extractor.addSample(vs.ax, vs.ay, vs.az);
        }
        if (haveI) { f.have_current = true; f.current_A = amps; }
        f.bus_V = volts; f.power_W = watts;

        // Slow sensors: copy last real values + their true age.
        // readAmbient() is NON-BLOCKING (cache only) — the blocking DHT22
        // read happens exclusively in task_dht on CORE 0.
        float t; uint32_t age;
        if (sensors.readMotorTemp(t, age)) {
            f.have_motor_temp = true;
            f.motor_temp_C = t;
            f.motor_temp_age_ms = age;
        }
        float amb, hum; uint32_t aage;
        if (sensors.readAmbient(amb, hum, aage)) {
            f.have_ambient = true;
            f.ambient_temp_C = amb;
            f.humidity_pct = hum;
            f.ambient_age_ms = aage;   // was previously read uninitialised
        }

        if (haveI) { f.have_current = true; f.current_A = amps; }
        f.bus_V = volts; f.power_W = watts;

        // Provenance mask. This MUST be written back to the frame — the
        // per-row validity record in the CSV and the dashboard's sensor
        // pills both read it. It was previously computed into a local and
        // discarded, so every logged row claimed sensor_status 0 even while
        // carrying valid readings.
        uint8_t status = 0;
        if (f.have_vib)        status |= SENSOR_MPU6050;
        if (f.have_motor_temp) status |= SENSOR_DS18B20;
        if (f.have_ambient)    status |= SENSOR_DHT22;
        if (f.have_current)    status |= SENSOR_CURRENT;
        f.sensor_status = status;
        gLiveStatus     = status;
        f.dropped_samples = (acqTicks > acqDone) ? (acqTicks - acqDone) : 0;

        if (gRecording) csv.writeFrame(f);

        acqDone++;
        gDropped = (acqTicks > acqDone) ? (acqTicks - acqDone) : 0;
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  FEATURES — CORE 1
// ═══════════════════════════════════════════════════════════════════════

static void featureTask(void*) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(WINDOW_MS));

        Features f;
        float mt; uint32_t mage = 0;
        bool haveM = sensors.readMotorTemp(mt, mage);
        float amb, hum; uint32_t aamb = 0;
        bool haveA = sensors.readAmbient(amb, hum, aamb);
        float amps, volts, watts;
        bool haveI = currentSensor()->read(amps, volts, watts);

        // Slow-sensor values are read from cache here — never by a
        // blocking call on this core.
        extractor.compute(WINDOW_SAMPLES, WINDOW_MS, 0,
                          haveM, mt, haveA, amb, hum, haveI, amps, f);
        f.sensor_status = sensors.hasMPU6050() ? SENSOR_MPU6050 : 0;
        if (haveM) f.sensor_status |= SENSOR_DS18B20;
        if (haveA) f.sensor_status |= SENSOR_DHT22;
        if (haveI) f.sensor_status |= SENSOR_CURRENT;

        xQueueSend(qFeature, &f, 0);
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  SLOW SENSORS — CORE 0
// ═══════════════════════════════════════════════════════════════════════

static void dhtTask(void*) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(DHT22_INTERVAL_MS));
        // The ONLY blocking DHT22 read in the firmware. CORE 0 only.
        static float t, h; static uint32_t a;
        sensors.sampleAmbient(t, h, a);
    }
}

static void oneWireTask(void*) {
    for (;;) {
        sensors.pollMotorTemp();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  RULES — CORE 0
// ═══════════════════════════════════════════════════════════════════════

static void rulesTask(void*) {
    for (;;) {
        Features f;
        if (xQueueReceive(qFeature, &f, portMAX_DELAY) != pdPASS) continue;

        RuleHit hit;
        gStatus = evaluator.evaluate(f, &hit);
        gHit    = hit;

        if (xSemaphoreTake(mFeature, pdMS_TO_TICKS(50))) {
            gFeat    = f;
            gFeatSeq++;
            xSemaphoreGive(mFeature);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  WEBSOCKET PUSH — CORE 0, 10 Hz
// ═══════════════════════════════════════════════════════════════════════

static void wsTask(void*) {
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(100));

        ws.cleanupClients();
        if (ws.count() == 0) continue;      // don't serialise for nobody

        Features f{};
        if (xSemaphoreTake(mFeature, pdMS_TO_TICKS(5))) {
            f = gFeat;
            xSemaphoreGive(mFeature);
        }

        // Measured acquisition rate, not the nominal target.
        static uint32_t lastDone = 0, lastT = 0;
        uint32_t now = millis(), done = acqDone;
        if (now - lastT >= 1000) {
            gHz = (done - lastDone) * 1000.0f / (float)(now - lastT);
            lastDone = done; lastT = now;
        }

        JsonDocument doc;
        doc["t"]         = (uint64_t)esp_timer_get_time();
        doc["run"]       = gRunId;
        doc["cond"]      = gCondition;
        doc["recording"] = gRecording;
        doc["pwm"]       = motor.duty();
        doc["motor_dir"] = motor.dir();
        doc["motor_run"] = motor.running();
        doc["status"]    = statusName(gStatus);
        doc["rule"]      = gHit.feature;
        doc["calibrated"]= evaluator.calibrated();
        doc["dropped"]   = gDropped;
        doc["fs_hz"]     = gHz;
        doc["n_samples"] = f.n_samples;

        // NaN is emitted as JSON null so the browser can distinguish
        // "not measured" from a real zero.
        doc["vib_rms"]          = isfinite(f.vib_rms)          ? f.vib_rms          : JsonVariant();
        doc["vib_variance"]     = isfinite(f.vib_variance)     ? f.vib_variance     : JsonVariant();
        doc["vib_peak_to_peak"] = isfinite(f.vib_peak_to_peak) ? f.vib_peak_to_peak : JsonVariant();
        doc["motor_temp_C"]     = isfinite(f.motor_temp_C)     ? f.motor_temp_C     : JsonVariant();
        doc["ambient_temp_C"]   = isfinite(f.ambient_temp_C)   ? f.ambient_temp_C   : JsonVariant();
        doc["humidity"]         = isfinite(f.humidity_pct)     ? f.humidity_pct     : JsonVariant();
        doc["temp_delta_C"]     = isfinite(f.temp_delta_C)     ? f.temp_delta_C     : JsonVariant();
        doc["current_A"]        = JsonVariant();   // INA219 deferred

        // Per-frame validity, published by acqTask. The dashboard's sensor
        // pills must reflect whether THIS reading is real, not whether the
        // device was found at boot. Reporting boot presence would show
        // "MPU6050 ok" beside an empty vibration tile, which reads as a
        // broken dashboard rather than a missing reading.
        JsonObject sens = doc["sensors"].to<JsonObject>();
        sens["mpu6050"] = (gLiveStatus & SENSOR_MPU6050) != 0;
        sens["ds18b20"] = (gLiveStatus & SENSOR_DS18B20) != 0;
        sens["dht22"]   = (gLiveStatus & SENSOR_DHT22)   != 0;
        sens["present_mask"] = gLiveStatus;

        String out;
        serializeJson(doc, out);
        ws.textAll(out);
    }
}

// ═══════════════════════════════════════════════════════════════════════


// ═══════════════════════════════════════════════════════════════════════
//  MOTOR COMMANDS
//  Applied and recorded in the same place — the CSV's motor_setting
//  column is therefore the value actually sent to the driver.
// ═══════════════════════════════════════════════════════════════════════

static void cmdSetMotor(uint8_t speed, uint8_t dir) {
    motor.setSpeed(speed, dir);
    gMotorPwm = motor.duty();
    gMotorRun = motor.running();
    gMotorDir = motor.dir();
    Serial.printf("[MOTOR] pwm=%u dir=%u duty=%u running=%d\n",
                  speed, dir, motor.duty(), (int)motor.running());
}
static void cmdRunMotor(void)  {
    if (motor.duty() == 0) motor.setSpeed(128, 0);
    motor.run();
    gMotorRun = 1;
    gMotorPwm = motor.duty();
    Serial.printf("[MOTOR] RUN (duty=%u)\n", motor.duty());
}
static void cmdStopMotor(void) { motor.stop(); gMotorRun = 0; gMotorPwm = 0; Serial.println("[MOTOR] STOP"); }
static void cmdBrakeMotor(void){ motor.brake(); gMotorRun = 0; gMotorPwm = 0; Serial.println("[MOTOR] BRAKE"); }

// ═══════════════════════════════════════════════════════════════════════
//  STATUS LINE — CORE 0, 1 Hz
//
//  One line, once a second. Everything a bench session needs to know
//  without scrolling: which channels are delivering, what the motor is
//  doing, the sample rate actually achieved, and whether a recording is
//  running. Replaces the previous three-line [DBG] block, which repeated
//  unchanged values dozens of times between real changes.
// ═══════════════════════════════════════════════════════════════════════

static const char* yn(bool v) { return v ? "OK" : "--"; }

static void statusTask(void*) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        Features f{};
        if (xSemaphoreTake(mFeature, pdMS_TO_TICKS(5))) {
            f = gFeat;
            xSemaphoreGive(mFeature);
        }

        float mt = NAN; uint32_t mage = 0;
        bool haveM = sensors.readMotorTemp(mt, mage);
        float amb = NAN, hum = NAN; uint32_t aamb = 0;
        bool haveA = sensors.readAmbient(amb, hum, aamb);

        const uint8_t st = gLiveStatus;
        const bool haveV = (st & SENSOR_MPU6050) != 0;
        const bool haveCur = (st & SENSOR_CURRENT) != 0;

        char vibBuf[24];
        if (haveV && isfinite(f.vib_rms)) snprintf(vibBuf, sizeof(vibBuf), "%.4f", (double)f.vib_rms);
        else                              snprintf(vibBuf, sizeof(vibBuf), "--");

        char motorBuf[20], ambBuf[20], humBuf[16];
        if (haveM) snprintf(motorBuf, sizeof(motorBuf), "%.2fC", (double)mt);
        else       snprintf(motorBuf, sizeof(motorBuf), "--");
        if (haveA) snprintf(ambBuf, sizeof(ambBuf), "%.1fC/%.0f%%", (double)amb, (double)hum);
        else       snprintf(ambBuf, sizeof(ambBuf), "--");
        (void)humBuf;

        // MPU fault counters, only surfaced when something is actually
        // wrong so a healthy run stays a single tidy line.
        static uint32_t shownBus = 0, shownRej = 0;
        char fault[48] = "";
        if (!haveV) {
            uint32_t rd = 0, bf = 0, rj = 0;
            sensors.mpuCounters(rd, bf, rj);
            snprintf(fault, sizeof(fault), " | mpu reads=%lu bus_fail=%lu reject=%lu",
                     (unsigned long)rd, (unsigned long)bf, (unsigned long)rj);
            shownBus = bf; shownRej = rj;
        } else {
            (void)shownBus; (void)shownRej;
        }

        Serial.printf("[STATUS] motor=%s vib=%s amb=%s | pwm=%u run=%d | "
                      "T_motor=%s vib_rms=%s hz=%.1f rec=%d%s\n",
                      yn(haveM), yn(haveV), yn(haveA),
                      motor.duty(), (int)motor.running(),
                      motorBuf, vibBuf, (double)gHz, (int)gRecording, fault);
        (void)haveCur;
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(921600);
    delay(1500);

    Serial.println("\n╔═══════════════════════════════════════════╗");
    Serial.println("║  EDGEPULSE — ESP32-S3-DevKitC-1          ║");
    Serial.println("╚═══════════════════════════════════════════╝");
    Serial.printf("chip=%s rev=%d cores=%d psram=%s\n",
                  ESP.getChipModel(), ESP.getChipRevision(),
                  ESP.getChipCores(), ESP.getPsramSize() ? "yes" : "none");

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, PIN_LED_ACTIVE);

    motor.begin(PIN_L298_ENA, PIN_L298_IN1, PIN_L298_IN2, MOTOR_PWM_HZ);
    motor.stop();

    Serial.println("\n── Sensor bring-up ───────────────────────");
    // A marker is printed BEFORE each stage. If a stage ever stalls, the
    // last marker printed is the culprit — no more guessing where a boot
    // stopped.
    Serial.println("[1/5] I2C + MPU6050");
    sensors.begin();
    Serial.printf("I²C devices found: %u (targeted probe 0x68/0x69/0x40)\n",
                  sensors.i2cCount());
    for (uint8_t i = 0; i < sensors.i2cCount() && i < 8; i++) {
        Serial.printf("  0x%02X%s\n", sensors.i2cAddrAt(i),
                      sensors.i2cAddrAt(i) == sensors.mpuAddr() ? "  <- MPU6050" : "");
    }
    Serial.printf("MPU6050 : %s (addr 0x%02X)\n",
                  sensors.hasMPU6050() ? "PRESENT" : "ABSENT", sensors.mpuAddr());
    Serial.printf("DS18B20 : %s\n", sensors.hasDS18B20() ? "PRESENT" : "ABSENT");
    Serial.printf("DHT22   : %s\n", sensors.hasDHT22() ? "PRESENT" : "ABSENT");
    if (!sensors.hasMPU6050())
        Serial.println("  ! No vibration data. Check SDA=GPIO8, SCL=GPIO9, 3V3, GND.");
    if (!sensors.hasDS18B20())
        Serial.println("  ! No motor temperature. Check DQ=GPIO6 and the 4.7k pull-up.");

    Serial.println("───────────────────────────────────────────\n");

    csv.begin();
    Serial.println("[2/5] baseline");

    // A baseline only exists once real runs have been calibrated.
    // Until then the status is UNCALIBRATED, not a false NORMAL.
    evaluator.loadBaseline((uint8_t*)FS_BASELINE_JSON, FS_BASELINE_LEN);
    Serial.printf("baseline: %s\n",
                  evaluator.calibrated() ? "loaded" : "NOT loaded → UNCALIBRATED");

    Serial.println("[3/5] LittleFS");
    if (!LittleFS.begin(true)) {
        Serial.println("FATAL: LittleFS mount failed");
    } else {
        Serial.println("LittleFS mounted");
        for (const char* p : {"/index.html", "/style.css", "/script.js"}) {
            File f = LittleFS.open(p);
            if (f && !f.isDirectory()) {
                Serial.printf("  %-14s %6d bytes\n", p, (int)f.size());
                f.close();
            } else {
                Serial.printf("  %-14s MISSING — run: pio run -t uploadfs\n", p);
                if (f) f.close();
            }
        }
    }

    Serial.println("[4/5] SoftAP");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("\nSoftAP '%s' at http://%s\n\n", AP_SSID,
                  WiFi.softAPIP().toString().c_str());

    Serial.println("[5/5] tasks");

    qFeature = xQueueCreate(4, sizeof(Features));
    mFeature = xSemaphoreCreateMutex();

    ws.onEvent([](AsyncWebSocket*, AsyncWebSocketClient* client,
                  AwsEventType type, void* arg, uint8_t* data, size_t len) {
        if (type == WS_EVT_CONNECT) {
            Serial.printf("[WS] client #%u connected\n", client->id());
            return;
        }
        if (type != WS_EVT_DATA) return;

        JsonDocument cmd;
        if (deserializeJson(cmd, data, len)) { Serial.println("[WS] malformed"); return; }

        const char* c = cmd["cmd"] | "";
        if (!strcmp(c, "SET_PWM")) {
            int pwm = cmd["pwm"] | -1;
            int dir = cmd["dir"] | 0;
            if (pwm < 0 || pwm > 255) Serial.println("[WS] pwm out of range 0-255");
            else cmdSetMotor((uint8_t)pwm, (uint8_t)(dir ? 1 : 0));
        }
        else if (!strcmp(c, "RUN"))   cmdRunMotor();
        else if (!strcmp(c, "STOP"))  cmdStopMotor();
        else if (!strcmp(c, "BRAKE")) cmdBrakeMotor();
        else if (!strcmp(c, "START_RUN")) {
            snprintf(gRunId, sizeof(gRunId), "%s", cmd["run"]   | "R001");
            snprintf(gCondition, sizeof(gCondition), "%s",
                     cmd["cond"] | "NORMAL");
            csv.beginRun(gRunId, gCondition);
            gRecording = true;
        }
        else if (!strcmp(c, "STOP_RUN")) { csv.endRun(); gRecording = false; }
        else if (!strcmp(c, "SET_K")) {
            evaluator.setWarningK(cmd["warn"] | 3.0);
            evaluator.setCriticalK(cmd["crit"] | 5.0);
            Serial.printf("[WS] k_warn=%.1f k_crit=%.1f\n",
                          evaluator.warningK(), evaluator.criticalK());
        }
        else Serial.printf("[WS] unknown cmd '%s'\n", c);
    });
    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
        r->send(LittleFS, "/index.html", "text/html");
    });
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest* r) {
        r->send(LittleFS, "/style.css", "text/css");
    });
    server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest* r) {
        r->send(LittleFS, "/script.js", "application/javascript");
    });
    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest* r) { r->send(204); });
    server.begin();

    // Acquisition timer: 100 Hz, hardware driven, never gated.
    static esp_timer_handle_t acqTimer{};
    esp_timer_create_args_t targs{};
    targs.callback        = [](void*){ acqTicks++; };
    targs.name            = "acq";
    targs.arg             = nullptr;
    esp_timer_create(&targs, &acqTimer);
    esp_timer_start_periodic(acqTimer, US_PER_SAMPLE);

    // CORE 1 — real time. CORE 0 — everything else.
    xTaskCreatePinnedToCore(acqTask,     "acq",     4096, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(featureTask, "feature", 4096, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(dhtTask,     "dht",     3072, nullptr, 1, nullptr, 0);
    xTaskCreatePinnedToCore(oneWireTask, "1wire",   3072, nullptr, 1, nullptr, 0);
    xTaskCreatePinnedToCore(rulesTask,   "rules",   4096, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(wsTask,      "ws",      6144, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(statusTask,  "status",  4096, nullptr, 1, nullptr, 0);

    Serial.println("Tasks started. Type HELP or STATUS for console control.");
}

void loop() {
    static uint32_t lastLed = 0;
    if (millis() - lastLed >= 1000) {
        lastLed = millis();
        digitalWrite(PIN_LED, motor.running() ? !PIN_LED_ACTIVE : PIN_LED_ACTIVE);
    }

    while (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;
        if (line.equalsIgnoreCase("RUN")) {
            cmdRunMotor();
        } else if (line.equalsIgnoreCase("STOP")) {
            cmdStopMotor();
        } else if (line.equalsIgnoreCase("BRAKE")) {
            cmdBrakeMotor();
        } else if (line.startsWith("PWM ") || line.startsWith("pwm ") || line.startsWith("M ") || line.startsWith("m ")) {
            int val = line.substring(line.indexOf(' ') + 1).toInt();
            if (val >= 0 && val <= 255) {
                cmdSetMotor((uint8_t)val, 0);
                if (!motor.running()) cmdRunMotor();
            } else {
                Serial.println("[CMD] PWM must be 0..255");
            }
        } else if (line.equalsIgnoreCase("START")) {
            csv.beginRun(gRunId, gCondition);
            gRecording = true;
            Serial.printf("[CMD] Recording START run=%s cond=%s\n", gRunId, gCondition);
        } else if (line.equalsIgnoreCase("END")) {
            csv.endRun();
            gRecording = false;
            Serial.println("[CMD] Recording STOPPED");
        } else if (line.equalsIgnoreCase("STATUS")) {
            float mt = NAN; uint32_t mage = 0;
            bool haveM = sensors.readMotorTemp(mt, mage);
            float amb = NAN, hum = NAN; uint32_t aage = 0;
            bool haveA = sensors.readAmbient(amb, hum, aage);
            Features f{};
            if (xSemaphoreTake(mFeature, pdMS_TO_TICKS(5))) {
                f = gFeat;
                xSemaphoreGive(mFeature);
            }
            Serial.printf("[STATUS] motor=%s vib=%s amb=%s | pwm=%u run=%d | T_motor=%.2fC vib_rms=%.4f hz=%.1f rec=%d\n",
                          haveM ? "OK" : "--",
                          isfinite(f.vib_rms) ? "OK" : "--",
                          haveA ? "OK" : "--",
                          motor.duty(), (int)motor.running(),
                          haveM ? (double)mt : 0.0,
                          isfinite(f.vib_rms) ? (double)f.vib_rms : 0.0,
                          (double)gHz, (int)gRecording);
        } else if (line.equalsIgnoreCase("HELP")) {
            Serial.println("[CMD] Available commands: RUN, STOP, BRAKE, PWM <0-255>, START, END, STATUS");
        }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
}