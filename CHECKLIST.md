# EdgePulse — Working Checklist

> ## ⏸️ PAUSED as of 2026-10-15
>
> The hackathon ended. **1st runner-up**, 8-hour build, AI-accelerated.
> Development is stopping here and the project may be inactive for a while.
>
> ### State at pause
>
> `main` is clean and synced with `origin`. A fresh `git clone` builds to
> `[SUCCESS]` in ~36 s with no hardware attached. Nothing is left uncommitted.
>
> ### Verified on real hardware before pausing
>
> | | Evidence |
> |---|---|
> | 100 Hz acquisition | `hz=100.0`, `drop=0`, 200/200 samples per window |
> | MPU6050 | answers at `0x68`, WHO_AM_I confirmed |
> | DS18B20 | live motor temperature, ~31.7 °C with correct staleness |
> | DHT22 | ambient temperature and humidity |
> | INA219 | driver written and compiling — **not yet confirmed on hardware** |
> | Dashboard | live values, charts, motor control, served over SoftAP |
>
> ### Known incomplete — do not mistake these for done
>
> 1. **`tools/` does not exist.** No host-side feature extractor, no baseline
>    calibration script, no threshold validation. The firmware can record a
>    dataset that nothing currently analyses. **This is the first thing to
>    build on resumption.**
> 2. **No threshold baseline exists yet.** `src/baseline.h` is deliberately
>    empty of numbers, so the status reports `UNCALIBRATED`. That is honest,
>    not a bug — do not "fix" it with invented values.
> 3. **No experiments have been run.** Experiment A (normal baseline, ≥3
>    runs) and Experiment B (speed variation) are still outstanding, and
>    Experiment C remains out of scope without a real load fixture.
> 4. **Hardware safety gate G2 is still open.** The motor's current draw has
>    never been measured with the driver disconnected. Until it is, do not
>    power the motor.
> 5. **TFT display is unverified.** It compiles and is feature-gated off by
>    default (`EDGE_TFT_ENABLED`); no board has been connected to it.
>
> ### First three steps when this resumes
>
> 1. Wire the MPU6050 and confirm the vibration trace **moves**. Data recorded
>    before the register-burst fix (commit `483adec`) was reading stale stack
>    memory and must be discarded.
> 2. Build `tools/extract_features.py` so the on-device features can be
>    recomputed and cross-checked against the firmware (gate G4).
> 3. Record ≥3 NORMAL runs, then generate a real `baseline.h` from them.
>
> ### Last known-good hardware state
>
> ESP32-S3 DevKitC-1, `board = esp32-s3-devkitc-1`, Arduino via PlatformIO.
> Flash with `pio run -t uploadfs && pio run -t upload` from
> `firmware/edgepulse-s3`. The board enumerates as
> `/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*`; the exact
> `ttyACM` number shifts between reboots, so pass `--upload-port` rather than
> hardcoding it.

---

**Project:** EdgePulse — Multisensor Edge-Based Anomaly Detection for DC Motor Health Monitoring
**Team:** Axioms
**Last updated:** 2026-10-15 (after full firmware implementation)

Discipline: exactly **one** item is `in_progress` at a time. An item moves to
`completed` only when its work is done **and verified by observation**. Code that
compiles is `built`, not `verified` — only flashed hardware can verify.

**Status vocabulary**
- ✅ **verified** — built, flashed, and observed working on hardware
- 🔧 **built** — compiles and links, hardware observation still pending
- ⬜ todo
- ⛔ cancelled

---

## A — Project setup

| # | Item | Status | Notes |
|---|---|---|---|
| A1 | Design decisions locked | ✅ verified | S3 · Arduino/PIO · **L298N direct** · SoftAP+WS · INA219 deferred · no ML in M1 |
| A2 | Project tree | ✅ verified | `/home/shadow/EdgePulse-Moni` |
| A3 | Design spec written | ✅ verified | `docs/2026-10-15-edgepulse-design.md` |
| A4 | Spec self-review + fixes | ✅ verified | 3 fixes, plus later board + driver corrections |
| A5 | GitHub reference research | ✅ verified | 4 repos cloned to `docs/references/` |
| A6 | `platformio.ini` | ✅ verified | `esp32-s3-devkitc-1`, builds + flashes |
| A7 | `.gitignore` | ✅ verified | |
| A8 | `README.md` | ⬜ todo | |
| A9 | `git init` + first commit | ⬜ todo | Needs go-ahead on remote |

---

## B — Design docs

| # | Item | Status | Notes |
|---|---|---|---|
| B1 | Spec: dashboard stack | ✅ verified | ESPAsyncWebServer + LittleFS + ArduinoJson v7 |
| B2 | Spec: board identity | ✅ verified | DevKitC-1, 8 MB, no PSRAM, `0x303A:0x1001` |
| B3 | Spec: close R1 / R7 | ✅ verified | Both closed |
| B4 | Spec: octal-PSRAM caveat | ✅ verified | N/A on this board; retained as a note |
| B5 | `docs/WIRING.md` | ✅ verified | Pin map + divider rationale + pre-power checklist |
| B6 | `docs/EXPERIMENT_LOG.md` | ⬜ todo | Template with run ID, condition, PWM, notes |
| B7 | `docs/DATASET.md` | ⬜ todo | Columns, units, gravity policy, missing-data policy |
| B8 | `include/pins.h` source of truth | ✅ verified | `pins.h` + WIRING.md + spec §3.2 agree |
| B9 | Spec: L298N replaces Uno+L293D | ✅ verified | §2, §3.1, §3.2, §3.4, §4.3, §11 updated |

---

## C — Firmware: ESP32-S3

| # | Item | Status | Notes |
|---|---|---|---|
| C1 | Board bring-up | ✅ verified | Flashed, boots, SoftAP up |
| C2 | MPU6050 driver | ✅ verified | `Sensors.cpp`. 0x68 probe, ±4 g / ±500 dps / 44 Hz DLPF, live stream |
| C3 | DS18B20 driver | ✅ verified | Non-blocking 12-bit, 1-Wire on GPIO 6, live motor temp (30.1 °C) |
| C4 | DHT22 driver | 🔧 built | Checksum + range validated, WDT panic fixed, 18ms wake-up |
| C5 | `task_acq` 100 Hz | ✅ verified | `esp_timer`, ring-free, `dropped_samples` tracked |
| C6 | `task_dht` + `task_1wire` on CORE 0 | ✅ verified | Physically separated from acquisition |
| C7 | `task_feature` 2 s windows | ✅ verified | AC-magnitude, matches spec §6.2 (`n=200/200`) |
| C8 | `task_rules` | ✅ verified | Returns **UNCALIBRATED** when no baseline loaded |
| C9 | `task_csv` batched | 🔧 built | NaN → **blank field**, never `0` |
| C10 | ~~UART2 link to Uno~~ | ⛔ cancelled | L298N driven directly |
| C11 | `CurrentSensor` interface | ⬜ todo | INA219 deferred; schema reserved |
| C12 | **Verify 100 Hz on hardware** | ✅ verified | Hardware confirmed: 200 samples/window, 0 drop |
| C13 | `MotorDriver` (L298N, LEDC) | ✅ verified | Built + flashed. WS + UART `PWM`/`RUN`/`STOP`/`BRAKE` |
| C14 | Boot report on UART | ✅ **verified** | Full boot observed: banner → sensors → LittleFS → SoftAP → tasks. Hang **fixed** |
| C15 | Fix DHT22 off the real-time core | ✅ completed | Split into `sampleAmbient()` (blocking, CORE 0) and `readAmbient()` (cache, any core) |
| C16 | Fix uninitialized `aage` | ✅ completed | `ambient_age_ms` now receives the real age |
| C17 | Compute `temp_delta_C` | ✅ completed | Computed only when BOTH temperatures are real |
| C18 | `CurrentSensor` + `NullCurrentSensor` | ✅ completed | INA219 now swaps in via one line; features/rules/CSV/dashboard untouched |
| C19 | Dashboard `SIM = false` | ✅ completed | Data now comes from the WebSocket only |
| C20 | Dashboard: null handling + sensor presence | ✅ completed | `—` for unmeasured, line breaks on gaps, sensor pills on screen |
| C21 | Dashboard: motor control UI | ✅ completed | PWM slider + RUN/STOP/BRAKE + START/STOP RUN |
| C22 | Dashboard: staleness watchdog | ✅ completed | Badge goes STALE if the socket goes quiet 3 s |

---

## D — Firmware: Arduino Uno

| # | Item | Status | Notes |
|---|---|---|---|
| D1–D2 | Uno + L293D firmware | ⛔ cancelled | Uno eliminated entirely |

---

## E — Dashboard

| # | Item | Status | Notes |
|---|---|---|---|
| E1 | AsyncWebServer + WS stack | ✅ verified | Compiles, links, serves |
| E2 | LittleFS pipeline | ✅ verified | `pio run -t uploadfs`, assets on flash |
| E3 | Live value panels | 🔧 built | HTML present, **fed by WebSocket now, not SIM** |
| E4 | `ws.count()==0` + change-detect | 🔧 built | In `wsTask` |
| E5 | Priority push on status change | ⬜ todo | Not yet split from the 10 Hz path |
| E6 | Time-series charts | 🔧 built | Hand-rolled canvas, no CDN |
| E7 | Run control (start/stop, PWM) | 🔧 built | Firmware side complete; **UI buttons not yet added** |
| E8 | Firing-rule display | 🔧 built | `doc["rule"]` sent |
| E9 | **Remove SIM mode** | ⬜ todo | `script.js` must switch `SIM=false` once sensors live |
| E10 | Show UNCALIBRATED honestly | ⬜ todo | Banner must not read NORMAL pre-baseline |

---

## F — Host tools

| # | Item | Status | Notes |
|---|---|---|---|
| F1 | `tools/extract_features.py` | ⬜ todo | **Must match `Features.cpp` exactly (gate G4)** |
| F2 | Feature unit tests | ⬜ todo | Host-verifiable with no hardware |
| F3 | `tools/calibrate_baseline.py` | ⬜ todo | Needs real NORMAL runs |
| F4 | `tools/validate_thresholds.py` | ⬜ todo | Held-out runs, stated FPR |
| F5 | `tools/plot_comparison.py` | ⬜ todo | |
| F6 | CSV round-trip test | ⬜ todo | |

---

## G — Hardware safety — **BLOCKS C12 AND ALL EXPERIMENTS**

| # | Item | Status | Notes |
|---|---|---|---|
| G1 | Identify motor model / ratings | ⬜ todo | |
| G2 | **Measure motor current, driver disconnected** | ⬜ todo | ⚠️ **> 2 A ⇒ motor stays disconnected** |
| G3 | Verify 12 V adapter vs driver input | ⬜ todo | |
| G4 | All ESP32 inputs within 3.3 V | ⬜ todo | GPIO is **not 5 V tolerant** |
| G5 | MPU6050 breakout supply/logic | ⬜ todo | Power from 3.3 V |
| G6 | Secure motor, guard shaft | ⬜ todo | |
| G7 | *(deferred)* INA219 pull-up check | ⬜ todo | Only when INA219 is added |

---

## H — Experiments and demo

| # | Item | Status | Notes |
|---|---|---|---|
| H1 | Experiment A — NORMAL baseline, ≥3 runs | ⬜ todo | Startup separate from steady state |
| H2 | Experiment B — speed variation | ⬜ todo | Only if G2 clears the driver |
| H3 | Experiment C — mechanical load | ⛔ out of scope | Needs a real fixture. **Never** hand pressure |
| H4 | Experiment D — repeatability | ⬜ todo | |
| H5 | Report: limitations | ⬜ todo | |
| H6 | Demo script | ⬜ todo | |

---

## Deferred to Milestone 2

- [ ] INA219 behind the existing `CurrentSensor` interface
- [ ] ML classifier split by recording run
- [ ] TFLite Micro — team already has `quantize_model.py` and a working
      `esp-tflite-micro` setup in RescuePulse
- [ ] Rebaseline thresholds with the current channel

---

## Standing rules

1. **Never fabricate a measurement.** No random numbers, no assumed values.
2. **Unavailable is blank, never `0`.**
3. **UNCALIBRATED ≠ NORMAL.** No measured baseline means no opinion.
4. **Thresholds come from measurements.** μ/σ from real runs.
5. **Split by recording run**, never random adjacent windows.
6. **No fault claims** without independent verification.
7. **No accuracy numbers** not measured on held-out runs.

---

## Changelog of errors caught

Recorded because each would have failed later and more expensively.

| Where | Error | Caught by |
|---|---|---|
| `platformio.ini` | `DHT.h`/`MPU6050.h` assumed to ship with core | Compiler |
| `platformio.ini` | `data_dir` invalid inside `[env:]` | PlatformIO warning |
| `platformio.ini` | MPU6050 pinned `0.5.1`; real version `1.4.5` | Dependency resolution |
| `lib_deps` | `electroniccats/MPU6050` has no `getEvent`/`Mpu6050Data` — only `MPU6050_Base` | Compiler |
| `MotorDriver.cpp` | `ledcAttach(pin,freq,res)` does not exist; real API is `ledcSetup`+`ledcAttachPin` | Compiler |
| `main.cpp` | `esp_timer_create_args_t.timer_group` removed in IDF v5.x | Compiler |
| `main.cpp` | `LittleFS.openDir()` removed in core 3.x | Compiler |
| `main.cpp` | `ESP_BOARD_NAME` not available in Arduino | Compiler |
| `Sensors.h` / `Rules.h` | `NAN` needed `<math.h>` | Compiler |
| `Rules.h` | `RuleHit` has a char array → not brace-assignable | Compiler |
| `CsvRecorder` | 4 KB buffer cannot live on a task stack → static | Compiler |
| `Sensors.cpp` | **I²C hang** — no pull-ups + no devices → SCL never rises → `endTransmission()` stalls. Fixed with internal `INPUT_PULLUP`, 20 ms timeout, targeted probe | **Boot log stopped at "Sensor bring-up"** |
| `main.cpp` | **DHT22 blocking read was on CORE 1** — `acqTask` called it, violating spec §4.1 and risking 100 Hz dropout | Code audit after C14 |
| `main.cpp` | `f.ambient_age_ms = aage;` read an uninitialised variable | Compiler |
| `Features.cpp` | `temp_delta_C` never computed — always NaN despite being a listed feature | Code audit |
| Spec §3.2 | Claimed LOLIN S3 Pro / 16 MB after board switch | Manual audit |
| Spec §3.2 | Status LED GPIO 4 vs actual GPIO 48 | Manual audit |