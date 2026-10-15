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

        uint8_t status = 0;
        if (f.have_vib)                 status |= SENSOR_MPU6050;
        if (f.have_motor_temp)          status |= SENSOR_DS18B20;
        if (f.have_ambient)             status |= SENSOR_DHT22;
        if (f.have_current)             status |= SENSOR_CURRENT;
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

        // UART debug line — one per 2 s window, cheap enough to read live.
        if (isfinite(f.vib_rms)) {
            Serial.printf("[FEAT] vib_rms=%.4f var=%.6f pp=%.4f n=%u/%u "
                          "motor=%.2f amb=%.2f hum=%.1f st=%s rule=%s\n",
                          (double)f.vib_rms, (double)f.vib_variance,
                          (double)f.vib_peak_to_peak,
                          f.n_samples, (unsigned)WINDOW_SAMPLES,
                          (double)f.motor_temp_C, (double)f.ambient_temp_C,
                          (double)f.humidity_pct,
                          statusName(gStatus), gHit.feature);
        } else {
            Serial.printf("[FEAT] no valid vibration data n=%u/%u "
                          "— sensors absent or rejecting readings\n",
                          f.n_samples, (unsigned)WINDOW_SAMPLES);
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

        JsonObject sens = doc["sensors"].to<JsonObject>();
        sens["mpu6050"]   = sensors.hasMPU6050();
        sens["ds18b20"]   = sensors.hasDS18B20();
        sens["dht22"]     = sensors.hasDHT22();
        sens["present_mask"] = f.sensor_status;

        String out;
        serializeJson(doc, out);
        ws.textAll(out);
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  UART DEBUG — CORE 0, 2 Hz
//  Plain human-readable telemetry, independent of CSV recording.
// ═══════════════════════════════════════════════════════════════════════

static void debugTask(void*) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));

        Features f{};
        if (xSemaphoreTake(mFeature, pdMS_TO_TICKS(5))) {
            f = gFeat;
            xSemaphoreGive(mFeature);
        }

        float mt; uint32_t mage = 0;
        bool haveM = sensors.readMotorTemp(mt, mage);
        float amb, hum; uint32_t aamb = 0;
        bool haveA = sensors.readAmbient(amb, hum, aamb);

        Serial.printf("[DBG] run=%s cond=%s rec=%d | "
                      "vib=%s motor=%s amb=%s hum=%s | "
                      "hz=%.1f drop=%lu status=%s rule=%s\n",
                      gRunId, gCondition, (int)gRecording,
                      isfinite(f.vib_rms)     ? "OK " : "-- ",
                      haveM ? "OK " : "-- ",
                      haveA ? "OK " : "-- ",
                      haveA ? "OK " : "-- ",
                      (double)gHz, (unsigned long)gDropped,
                      statusName(gStatus), gHit.feature);

        Serial.printf("[DBG] vib_rms=%s pp=%s raw=[%.2f,%.2f,%.2f]g motor=%s°C age=%lums "
                      "amb=%s°C hum=%s%% pwm=%u running=%d\n",
                      isfinite(f.vib_rms) ? String((double)f.vib_rms, 4).c_str() : "n/a",
                      isfinite(f.vib_peak_to_peak) ? String((double)f.vib_peak_to_peak, 4).c_str() : "n/a",
                      gLatestVib.ax, gLatestVib.ay, gLatestVib.az,
                      haveM ? String((double)mt, 2).c_str() : "n/a",
                      (unsigned long)mage,
                      haveA ? String((double)amb, 2).c_str() : "n/a",
                      haveA ? String((double)hum, 1).c_str() : "n/a",
                      motor.duty(), (int)motor.running());

        Serial.printf("[DBG] sensors: MPU6050=%s(addr 0x%02X) DS18B20=%s "
                      "DHT22=%s i2c_devices=%u rows=%lu\n",
                      sensors.hasMPU6050() ? "present" : "ABSENT",
                      sensors.mpuAddr(),
                      sensors.hasDS18B20() ? "present" : "ABSENT",
                      sensors.hasDHT22()   ? "present" : "ABSENT",
                      sensors.i2cCount(), (unsigned long)csv.rowsWritten());
    }
}

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
    xTaskCreatePinnedToCore(debugTask,   "debug",   4096, nullptr, 1, nullptr, 0);

    Serial.println("Tasks started. Send START_RUN over the dashboard to record.");
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
        } else if (line.equalsIgnoreCase("HELP")) {
            Serial.println("[CMD] Available commands: RUN, STOP, BRAKE, PWM <0-255>, START, END");
        }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
}