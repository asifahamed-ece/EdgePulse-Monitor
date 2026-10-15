# EdgePulse — Design Specification

**Project:** EdgePulse — Multisensor Edge-Based Anomaly Detection for DC Motor Health Monitoring
**Team:** Axioms (Badhri Narrayanan B · Asif Ahamed S · Subiksha A · Sivadharsan M S — all ECE, 4th year)
**Date:** 2026-10-15
**Status:** Draft for review — not yet approved
**Scope of this document:** Milestone 1 only (first evaluation / first review). Machine learning is explicitly out of scope.

---

## 1. Purpose and success criteria

Milestone 1 succeeds when a reviewer can watch, in real time, an ESP32-S3 monitoring a small brushed DC motor and see: live vibration, live motor and ambient temperature, live humidity, a NORMAL/WARNING/CRITICAL status, and time-series graphs — with every underlying number traceable to a real measurement in a real CSV file.

Success is measured by these gates, all of which must pass:

| # | Gate | Evidence required |
|---|---|---|
| G1 | Every sensor reads plausible, non-constant values | Serial Monitor output captured per sensor |
| G2 | Acquisition holds 100 Hz with zero dropped samples | `dropped_samples == 0` over a 5-minute run |
| G3 | Raw CSV is real recorded data | File on disk with real timestamps |
| G4 | Features recompute identically off-device | Host and firmware feature values match |
| G5 | Thresholds come from measurements, not guesses | Baseline μ/σ recorded from real runs |
| G6 | Thresholds validated on held-out runs | Stated false-positive rate |
| G7 | Dashboard shows live values and graphs | Demo screenshot / live run |

**Milestone 1 does not claim:** bearing-fault detection, remaining-useful-life estimation, failure prediction, or any accuracy percentage. See §12.

---

## 2. Decisions locked at design time

| Decision | Choice | Rationale |
|---|---|---|
| Controller | **ESP32-S3** (8 MB flash / 16 MB PSRAM class board) | Native USB is a separate interface, freeing all 3 UARTs. eFuse-calibrated ADC. Dual LX7 core at 240 MHz. |
| Framework | **Arduino via PlatformIO** | Installed and verified on this machine (`arduino-cli`, esp32 core 3.3.12). ESP-IDF is not installed. |
| Motor control | **L298N H-bridge, driven directly by the ESP32-S3** | Single board. Removes the Uno, the L293D shield, the UART link and the level-shifting divider. 2 A/channel vs the L293D's 0.6 A. |
| Controller coupling | **None — ESP32 drives ENA/IN1/IN2 directly** | The commanded PWM is applied and recorded in the same handler, so the CSV value is the value actually sent to the driver. |
| Dashboard transport | **ESP32 SoftAP + WebSocket**, real time | Requested. See §9 for the no-internet constraint. |
| CSV transport | **USB-CDC to laptop at 921600 baud** | Deferred refinement; batched writes (§5.4). |
| Current sensing | **Deferred.** Interface reserved, `NullCurrentSensor` in review 1 | See §3.3. |
| Machine learning | **Out of scope for Milestone 1** | Threshold-based detection is the deliverable and the correct fallback. |
| Hardware condition | **Motor disconnected until Phase 0 passes** | Non-negotiable. |

---

## 3. Hardware

### 3.1 Component register

| # | Component | Interface | Address | Role | Status |
|---|---|---|---|---|---|
| 1 | ESP32-S3 dev board | — | — | Acquisition, features, rules, AP, WebSocket | Primary |
| 2 | L298N dual H-bridge module | GPIO 4 / 17 / 18 | — | Motor control. 2 A DC per channel, ~1.8 V drop at 1 A | Primary |
| 4 | Brushed DC motor, nominal 12 V | — | — | Device under test | **Model unverified** |
| 5 | MPU6050 | I²C | `0x68` | Vibration | Confirmed |
| 6 | DHT22 | single GPIO | — | Ambient temp + humidity | Confirmed |
| 7 | DS18B20 waterproof probe | 1-Wire | — | Motor surface temp | Confirmed |
| 8 | INA219 | I²C | `0x40` | Current / bus voltage / power | **DEFERRED** |
| 9 | 12 V motor adapter | — | — | Motor supply | Rating unverified |
| 10 | DC-DC converters, breadboard, jumpers, perfboard | — | — | Assembly | Available |

### 3.2 Pin map

| Function | GPIO | Bus | Notes |
|---|---|---|---|
| I²C SDA | **8** | shared | MPU6050 + (later) INA219 |
| I²C SCL | **9** | shared | 4.7 kΩ pull-ups to **3.3 V** |
| DS18B20 DQ | **6** | 1-Wire | 4.7 kΩ pull-up to 3.3 V. **3-wire, not parasite.** |
| DHT22 DATA | **7** | GPIO | — |
| L298N ENA | **4** | PWM | LEDC hardware PWM @ 16 kHz. Remove ENA jumper |
| L298N IN1 | **17** | GPIO | Direction |
| L298N IN2 | **18** | GPIO | Direction |
| USB-CDC | **19 / 20** | — | **RESERVED — never assign** (D− / D+) |
| Status LED | **48** | GPIO | Onboard LED on DevKitC-1. Drive LOW for on |
| Buzzer | **16** | GPIO | Via 2N2222 + flyback diode. Never direct-drive. |

**Board: Espressif ESP32-S3-DevKitC-1** (generic). PlatformIO id `esp32-s3-devkitc-1`.

Verified against `.platformio/platforms/espressif32/boards/esp32-s3-devkitc-1.json`:

| Property | Value | Consequence for EdgePulse |
|---|---|---|
| `frameworks` | `["arduino", "espidf"]` | The Arduino decision holds on this board |
| `-DARDUINO_USB_MODE=1` | set by the board definition | `Serial` is **native USB CDC** — no CP2102 bridge needed |
| `partitions` | `default_8MB.csv` | **8 MB flash**, max app 8,388,608 bytes. Do **not** claim 16 MB unless the module is physically marked `N16R8` — an over-claimed flash size fails at the very end of an upload |
| PSRAM | **none declared** | Not required for Milestone 1. 100 Hz acquisition and the JSON payload fit in 512 KB internal SRAM |
| `hwids` | `0x303A:0x1001` | Appears as `/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*` |

**No ADC pin is used in Milestone 1.** All current-sensing pins remain free for the INA219 later.

⚠️ **Onboard LED is GPIO48 on this board**, not GPIO4. GPIO4 is free for other use.

### 3.3 Current sensing — deferred, but designed in

`NullCurrentSensor` returns `NaN` for current. CSV columns `current_raw` and `current_A` are **written as empty fields**, never `0`. Feature functions that consume current are implemented but emit `NaN`.

When the INA219 arrives, it plugs in behind the same `CurrentSensor` interface with no changes to features, thresholds, CSV schema, or dashboard.

**INA219 facts to reuse later** (verified against TI datasheet Rev. G):
- 26 V bus, 12-bit, I²C, 16 addresses, supply 3.0–5.5 V
- PGA ranges ±40 mV / ±80 mV / ±160 mV / ±320 mV (default ÷8)
- Shunt LSB 10 µV → 0.1 mA on a 0.1 Ω shunt
- Reports current, bus voltage **and** power

⚠️ **Destructive hazard to check before wiring (applies when INA is added, but verify the board now if it is already connected):** many INA219 breakouts pull SDA/SCL up to **VCC**; some wire them to **VIN**. If the pull-ups go to VIN and VIN is the 12 V motor bus, 12 V is pushed onto GPIO8/9 and the S3 is destroyed instantly. With the 12 V adapter **disconnected**, measure resistance from SDA to VIN and from SDA to VCC. If it reads to VIN, remove those two resistors and fit 4.7 kΩ to 3.3 V.

### 3.4 Phase 0 safety gates — nothing is powered until these pass

1. Identify the motor's rated voltage, rated current, and startup/stall current.
2. Verify the L298N can safely drive it. **2 A DC per channel is the ceiling** (ST datasheet; 2.5 A repetitive, 3 A peak). Note the module runs hot — the L298 is a bipolar TTL driver with ~1.80 V total drop at 1 A.
3. Measure the motor's actual current draw with the shield disconnected. If it exceeds 0.6 A, the motor stays disconnected.
4. Confirm every signal entering the ESP32 stays within 3.3 V. **ESP32 GPIO/ADC inputs are not 5 V tolerant.**
5. Verify the MPU6050 breakout supply and logic levels. Power from 3.3 V.
6. Inspect the 12 V adapter rating against the shield's motor-power input.
7. Secure the motor, guard the rotating shaft, insulate exposed wiring.

**Standing prohibitions:** never power the motor from an ESP32 GPIO. Never stop the powered motor by hand, short its terminals, jam the shaft, or attach loose weights. Do not use hand pressure, shaft obstruction, improvised friction, or rotor jamming as a load experiment. If no safe load fixture exists, Experiment C is omitted — not faked.

**Regression to flag:** with current sensing deferred, software over-current protection is unavailable. The L298N is protected only by Phase-0 multimeter verification plus short, supervised runs. The larger 2 A ceiling lowers this risk relative to the L293D, but does not remove it.

### 3.5 ESP32-S3 constraints carried from datasheet verification

- **ADC2 cannot be read while Wi-Fi is running** (`esp_wifi_start()`→`esp_wifi_stop()` window). Relevant only when the INA219 or an analog sensor is added.
- **ADC2 continuous mode is unsupported on the S3**; results are unstable.
- ADC1 = GPIO1–GPIO10; ADC2 = GPIO11–GPIO20.
- GPIO19 = USB D−, GPIO20 = USB D+; reserved while native USB is in use.
- ESP32-S3 has 3 hardware UARTs. **Do not use `SoftwareSerial`** — it is a known-flaky anti-pattern on ESP32. Use `HardwareSerial Serial2`.
- **Octal PSRAM (`qio_opi`) occupies internal GPIOs.** On the ESP32-S3, octal PSRAM shares the SPI bus with flash and claims GPIO35–37. The pin map in §3.2 avoids those pins entirely, so there is no conflict. **However, Arduino framework support for octal PSRAM is not yet verified on this board** — the team's RescuePulse project uses ESP-IDF for it and needed explicit flags (`-mfix-esp32-psram-cache-issue`). Milestone 1 does **not** require PSRAM: 100 Hz acquisition, a ring buffer, and a JSON payload all fit comfortably in the 512 KB of internal SRAM. If PSRAM proves troublesome under Arduino, simply run without it. **Do not port RescuePulse's ESP-IDF build flags into this Arduino project.**

---

## 4. Firmware architecture

### 4.1 Task layout

The central design rule: **acquisition timing must never be gated by a slow sensor or a slow consumer.**

```
CORE 1 — real time (nothing blocking lives here)
  task_acq        100 Hz   high prio   MPU6050 burst read → ring buffer
  task_feature    2 s      on N=200    compute window features

CORE 0 — everything else
  task_dht        0.5 Hz   low prio    DHT22 (blocks ~5 ms, timing-critical)
  task_1wire      1 Hz     low prio    DS18B20 (1-Wire, timing-critical)
  task_rules      on each feature        evaluate thresholds → status + firing rule
  task_ws         10 Hz                  serialise JSON → WebSocket broadcast
  task_csv        batched                100 rows → one USB write
  task_cmd        on arrival             parse WebSocket commands
```

A DHT22 read blocks for ~5 ms against a 10 ms acquisition period, and 1-Wire bit timing is microsecond-sensitive. Both are therefore assigned to **CORE 0**, physically separated from `task_acq` on CORE 1. Running either inside the acquisition loop silently destroys vibration sample counts, so CORE 1 carries nothing blocking. `task_rules` also drives the status LED (GPIO 4) and the buzzer (GPIO 5, via 2N2222) so alerts never add load to the acquisition core.

### 4.2 Acquisition

- `esp_timer` at exactly 100 Hz (10 ms) drives `task_acq`. Not `delay()`, not `loop()`.
- MPU6050 read as a **single 14-byte burst** (`0x3B`–`0x48`) to hold I²C bus occupancy low.
- Each sample is stamped with `esp_timer_get_time()` microseconds, not an assumed nominal time.
- A `dropped_samples` counter compares expected vs actual ticks and is logged per row and per run.

### 4.3 Motor command path

**Single board.** The ESP32-S3 drives an L298N H-bridge directly over three GPIOs. There is no second microcontroller, no UART link, no level shifter, and no Arduino sketch to maintain.

The browser sends `SET_PWM`, `RUN`, `STOP`, or `BRAKE` over the WebSocket. The firmware:

1. Applies the duty cycle to ENA via LEDC hardware PWM and sets IN1/IN2 for direction.
2. **In the same handler**, stamps the active `run_id`, `condition` and commanded PWM into the acquisition frame.

This is the structural guarantee that `motor_setting` in the CSV is never a human guess.

**3.3 V logic is valid without a level shifter.** ST L298 datasheet gives `VenH` = 2.3 V min and `VenL` = 1.5 V max. The ESP32-S3's 3.3 V output clears both. The L298N's 5 V pin is never connected to the ESP32.

**Known cost:** the L298 is a bipolar TTL driver with ~1.80 V total drop at 1 A (4.9 V worst case at 2 A). A 12 V supply reaches the motor at roughly 10 V. This operating point is part of the measured condition and must be recorded in the experiment log.

---

## 5. Data contracts

### 5.1 `edgepulse_raw.csv`

| Column | Unit | Notes |
|---|---|---|
| `timestamp_us` | µs | `esp_timer_get_time()`, real |
| `run_id` | — | e.g. `R003` |
| `condition` | — | `NORMAL`, `SPEED_LOW`, `SPEED_MEDIUM`, `SPEED_HIGH` |
| `motor_setting` | PWM 0–255 | Written by the command handler |
| `ax`, `ay`, `az` | g | MPU6050, ±2 g range |
| `gx`, `gy`, `gz` | °/s | Retained where practical, ±250 °/s |
| `current_raw` | ADC counts | **EMPTY** — INA219 deferred |
| `current_A` | A | **EMPTY** — INA219 deferred |
| `motor_temp_C` | °C | DS18B20, its real ~750 ms timestamp |
| `ambient_temp_C` | °C | DHT22, its real ~2 s timestamp |
| `humidity_pct` | % | DHT22 |
| `sensor_status` | bitmask | Which channels are valid this row |
| `dropped_samples` | count | Cumulative since run start |

**Integrity rules, enforced by construction:**

- Unavailable sensors emit **empty fields**, never `0`. `0 °C` is a claim; blank is not.
- Slow sensors are **not resampled** to look like 100 Hz. A DHT22 value appears on the row where it was actually read, with its own age recorded.
- Failed experiments and sensor errors are **recorded**, never silently deleted or relabelled.
- Gravity handling for vibration features is documented explicitly and applied identically on-device and on-host (§6.2).

### 5.2 `edgepulse_features.csv`

Fixed 2-second windows. At 100 Hz that is 200 samples per window for vibration.

| Feature | Definition |
|---|---|
| `vib_rms` | RMS of vibration magnitude over window |
| `vib_variance` | Variance of vibration magnitude |
| `vib_peak_to_peak` | max − min of vibration magnitude |
| `current_mean_A` | `NaN` in Milestone 1 |
| `current_rms_A` | `NaN` in Milestone 1 |
| `current_variance` | `NaN` in Milestone 1 |
| `motor_temp_C` | Last valid DS18B20 value in window |
| `ambient_temp_C` | Last valid DHT22 value in window |
| `humidity_pct` | Last valid DHT22 value in window |
| `temp_delta_C` | `motor_temp_C − ambient_temp_C`, only when both valid |
| `n_samples` | **Actual** sample count in window — exposes dropped samples |
| `run_id`, `condition`, `motor_setting` | Carried through unchanged |

### 5.3 Dataset integrity

- Train/validation/test splits are **by recording run**, never by randomly mixing adjacent windows from one recording. Mixing adjacent windows leaks near-identical data across splits and inflates results.
- Every feature row retains its run ID, condition, and motor setting.
- Speed variation labels are **operating-condition labels, not faults**.

### 5.4 USB-CDC transport

921600 baud. The firmware **never** writes a row per sample at 100 Hz — a row is ~80–100 bytes, i.e. 8–10 kB/s, which saturates a 115200-baud link and strains 921600 once headers are added. Rows are batched (100 per write) and flushed every ~1 s. A browser refresh, a dropped client, or laptop sleep cannot affect acquisition, because acquisition is owned by the hardware timer and nothing else.

---

## 6. Feature extraction

### 6.1 Where it runs

Implemented twice, deliberately:

- **On-device** — so the dashboard can show live status.
- **On-host (Python)** — so features can be recomputed, plotted and validated independently.

Both must produce identical values from identical input (Gate G4). The Python implementation is the reference; a mismatch is a bug, not a rounding difference.

### 6.2 Vibration magnitude and gravity

Magnitude is `sqrt(ax² + ay² + az²)`. Gravity is **removed by subtracting the window mean** before any feature is computed — that is, features are computed on the AC component:

```
mag[i]   = sqrt(ax[i]^2 + ay[i]^2 + az[i]^2)
ac[i]    = mag[i] - mean(mag over the window)
vib_rms  = sqrt(mean(ac[i]^2))
```

**Why window-mean removal rather than subtracting 1 g from one axis:** the motor's mounting orientation is not fixed and will differ between runs and sessions. Subtracting a fixed 1 g from a named axis is only correct if that axis is known to be vertical, which is not guaranteed. Window-mean removal is orientation-independent, needs no calibration, and is trivially identical in C and Python.

This decision is **fixed** for Milestone 1 and applied byte-identically in the firmware and the host extractor. It is recorded in the dataset description so no reader mistakes the features for gravity-inclusive ones.

### 6.3 Toolchain (already installed and verified)

numpy 2.5.4, pandas 2.3.3, scikit-learn 1.9.1, scipy 1.18.1, matplotlib 3.11.2, pyserial 3.5, joblib 1.6.0.

---

## 7. Threshold rules

**Measured, not invented.** μ and σ come from real baseline runs. The k-multipliers are stated policy.

| State | Condition |
|---|---|
| NORMAL | All features inside the validated baseline band |
| WARNING | Any feature beyond μ + 3σ |
| CRITICAL | Any feature beyond μ + 5σ, or a configured absolute limit |

Procedure: collect ≥3 independent NORMAL runs → compute per-feature μ/σ over 2 s windows → record the baseline file → set the bands → **validate on held-out runs** and report the observed false-positive rate (Gate G6).

These states describe the prototype's monitoring rules. **They are not certified safety conditions.**

---

## 8. Experiments

| ID | Experiment | Status in Milestone 1 |
|---|---|---|
| A | Normal baseline at one verified safe setting; startup recorded separately from steady state; ≥3 independent runs | **In scope** |
| B | Controlled speed variation across a small number of documented PWM settings, each repeated in separate runs | **In scope**, if the driver is verified safe |
| C | Controlled mechanical load variation | **OUT OF SCOPE.** Requires a real, safe, repeatable load fixture. If none exists, it is omitted. Not faked, never hand pressure. |
| D | Repeatability across runs and sessions with consistent mounting and procedure | **In scope** |

**Speed change is not a fault.** `SPEED_LOW` / `SPEED_MEDIUM` / `SPEED_HIGH` are operating-condition labels.

---

## 9. Dashboard

### 9.1 Verified dashboard stack

The stack is **not speculative** — the team has it working in production firmware on another project (`wireless-quiz-buzzer`, ESP32 + Arduino + PlatformIO). Reuse it rather than re-deriving it.

```ini
lib_deps =
    https://github.com/me-no-dev/ESPAsyncWebServer.git
    https://github.com/me-no-dev/AsyncTCP.git
    bblanchon/ArduinoJson @ ^7.2.0
board_build.filesystem = littlefs
board_build.partitions = default_16MB.csv
data_dir = ../../dashboard/data
```

Dashboard assets live in a `dashboard/data/` folder, are flashed to the device with `pio run -t uploadfs`, and are served by explicit route handlers that call `request->send(LittleFS, "/index.html", "text/html")`.

**The no-internet constraint is already solved.** The laptop joins the ESP32 SoftAP, which has no upstream route. The team's existing dashboard references `/style.css` and `/script.js` — **local absolute paths, zero CDN references** — and works. EdgePulse must keep that property. Any `<script src="https://cdn...">` tag produces a blank dashboard on demo day while appearing to work perfectly on a developer machine that has internet. Any charting library must be bundled into `dashboard/data/` and served from flash.

**Charting:** prefer **uPlot** (small, built for real-time ring buffers) served from LittleFS. Chart.js is ~200 KB and workable but heavier. Browsers cache aggressively, so add cache-busting query strings during development.

**Residual risk, narrowed:** the stack is proven on `esp32dev` (classic ESP32). The open question is narrower than "does this work" — it is only whether ESPAsyncWebServer builds cleanly on `lolin_s3_pro` under esp32 core 3.3.12. Build the dashboard skeleton first, before any sensor wiring, to settle it cheaply.

### 9.2 WebSocket protocol

Browser → ESP32:
```
{"cmd":"START_RUN","cond":"SPEED_MEDIUM","pwm":128}
{"cmd":"SET_PWM","pwm":180}
{"cmd":"RUN"}  {"cmd":"STOP"}  {"cmd":"STOP_RUN"}
```

ESP32 → browser, at 10 Hz:
```json
{"t":1234567,"run":"R003","cond":"SPEED_MEDIUM","pwm":128,
 "vib":{"rms":0.0121,"var":0.00014,"pp":0.083},
 "temp":{"motor":41.6,"ambient":28.4,"delta":13.2},
 "humidity":52.1,
 "status":"NORMAL","rule":"-" ,
 "dropped":0,"fs_hz":99.98}
```

`fs_hz` — the *actual* measured acquisition rate — is displayed on the dashboard so a reviewer can see the 100 Hz claim is verified, not asserted.

### 9.3 Dashboard panels

1. **Status banner** — NORMAL / WARNING / CRITICAL, high contrast.
2. **Live values** — vibration RMS, motor temp, ambient temp, humidity, actual sample rate, dropped count.
3. **Time-series graphs** — vibration RMS, motor vs ambient temperature, humidity.
4. **Firing rule** — names the specific rule and feature that triggered the current state.
5. **Run control** — start/stop run, set condition, set PWM.
6. **Recording indicator** — CSV writing, run ID, elapsed time.

### 9.4 Four transfer patterns from existing team code

The team's `wireless-quiz-buzzer` firmware solves the real-time dashboard problem. These four patterns transfer directly:

| Pattern | Why EdgePulse needs it |
|---|---|
| `if (ws.count() == 0) return;` before serialising | Do not spend CPU building a 10 Hz JSON payload when nobody is connected |
| `if (jsonString == lastBroadcastData) return;` | Change-detection — push only when a value actually moved |
| `broadcastUpdatePriority()` that bypasses the throttle | A transition into WARNING or CRITICAL must push **immediately**, not wait for the next 10 Hz slot |
| `ws.cleanupClients()` on a periodic timer | Prevents client memory growth over a long demo session |

**What must NOT be copied.** That firmware is a `delay(10)` + `millis()` polling loop with no FreeRTOS tasks, updating at roughly 100 ms granularity, and calls `ws.textAll()` directly from `loop()`. That structure is fatal at EdgePulse's 100 Hz acquisition rate. All WebSocket traffic originates from the dedicated `task_ws` in §4.1, never from a polling `loop()`.

---

## 10. Verification strategy

| Level | What is tested | How |
|---|---|---|
| Host | Feature maths | Python unit tests against hand-computed fixtures |
| Host | CSV parsing | Round-trip test: write → read → assert |
| Firmware | Sensor drivers | Per-sensor Serial Monitor check before integration |
| Firmware | Acquisition rate | `dropped_samples` and `fs_hz` over 5 minutes |
| Firmware | Cross-implementation | Firmware features vs host features on the same input |
| System | Thresholds | Held-out runs, stated FPR |
| System | Dashboard | Live run with a deliberate induced deviation |

No success claim is made without running the verification and observing the output.

---

## 11. Risks and open questions

| # | Risk | Impact | Mitigation |
|---|---|---|---|
| R1 | `ESPAsyncWebServer` does not build on `lolin_s3_pro` under core 3.3.12 | Dashboard blocked | **CLOSED.** The full stack was compiled and linked on `lolin_s3_pro`: ESPAsyncWebServer 3.6.0, AsyncTCP 3.3.2, ArduinoJson 7.4.3. RAM 13.6%, Flash 12.6%. Only runtime behaviour on a physical board remains unverified. |
| R2 | Motor exceeds L298N 2 A DC per channel | Motor/driver damage, overheating | Phase-0 current measurement with the driver disconnected; motor stays disconnected if over. Thermal rise on the L298N heatsink is expected at sustained load |
| R3 | INA219 pull-ups wired to VIN (when added) | **Destroys the S3** | §3.3 resistance check with 12 V disconnected |
| R4 | DHT22's 5 ms block corrupts 100 Hz timing | Silent sample loss | Separate task, different core; verified by `dropped_samples` |
| R5 | 1-Wire timing under Wi-Fi load | Intermittent DS18B20 failures | Dedicated low-priority task; 3-wire with 4.7 kΩ pull-up |
| R6 | No current sensing in review 1 | Weaker health story, no software over-current protection | Interface reserved; manual protection via Phase-0 verification |
| R7 | Dashboard depends on CDN assets | Blank dashboard on demo day | **Closed.** The team's existing dashboard serves `/style.css` and `/script.js` from LittleFS with zero external references. Keep that property (§9.1). |
| R8 | USB bandwidth contention | Truncated CSV | Batched writes; acquisition owned by hardware timer |

**Open questions:**

- Exact motor model and rated current — **unverified**.
- Whether a safe, repeatable mechanical load fixture exists (decides Experiment C).
- ~~Whether `ESPAsyncWebServer` builds on `lolin_s3_pro` under esp32 core 3.3.12.~~ **RESOLVED — it builds and links.** R1 closed.
- Whether Arduino framework drives the LOLIN S3 PRO's octal PSRAM correctly. Not required for Milestone 1; running without PSRAM is an acceptable fallback (§3.5). **Confirm from the boot banner's PSRAM line when a board is attached.**

---

## 12. Claims this project does not make

Stated plainly, so that nothing in the report or the demo overreaches:

- **No confirmed fault detection.** Nothing here is a validated bearing-fault or fault-severity classifier.
- **No remaining-useful-life or failure prediction.**
- **No vehicle-level validation.** A bench DC motor is not an automotive component under real duty cycles.
- **No quantified downtime reduction.** No production data supports such a claim.
- **No industrial readiness.**
- **No invented accuracy.** If the dataset is insufficient, threshold-based detection stands alone and that is reported as the result.

The three states NORMAL / WARNING / CRITICAL describe this prototype's monitoring rules. They are not certified safety conditions.

---

## 13. Suggested work split

The team lead coordinates and owns the run log and experiment conditions. Suggested allocation, to be reassigned as people find their strengths:

| Area | Owner | First deliverable |
|---|---|---|
| Firmware: acquisition, ring buffer, CSV | — | P1/P2 gates G1, G2 |
| Firmware: features, rules, thresholds | — | G4, G5, G6 |
| Dashboard: SoftAP, WebSocket, charts | — | E1 build check, then G7 |
| Experiments, dataset integrity, plots, report | — | Experiments A/B/D, comparison plots |

---

## 14. Phase order

| Phase | Deliverable | Gates |
|---|---|---|
| **P0** | Hardware verification and safety. Nothing powered until complete. | §3.4 all pass |
| **P1** | Each sensor read individually → Serial Monitor | G1 |
| **P2** | 100 Hz `edgepulse_raw.csv` over USB-CDC | G2, G3 |
| **P3** | Python feature extractor → `edgepulse_features.csv` | G4 |
| **P4** | Baseline runs, thresholds, held-out validation | G5, G6 |
| **P5** | Wi-Fi AP dashboard: live values, then graphs | G7 |
| **P6** | Experiments A/B/D, comparison plots, demo | — |

**The dashboard build check is done (E1).** `ESPAsyncWebServer` was confirmed to compile and link on `lolin_s3_pro`, closing the last software risk. It does not replace P0 — hardware safety still blocks any powered sensor work.

P1 precedes P5 deliberately: the Serial Monitor proves a sensor is real at zero dependency cost, before a WebSocket server is entangled with it.

---

## Appendix A — Verified environment facts

| Check | Result |
|---|---|
| `arduino-cli` esp32 core | 3.3.12 installed |
| PlatformIO Core | 6.2.0 |
| Python | numpy 2.5.4, pandas 2.3.3, scikit-learn 1.9.1, scipy 1.18.1, matplotlib 3.11.2, pyserial 3.5, joblib 1.6.0 |
| ESP-IDF | **Not installed** — `idf.py` absent, no `~/.espressif/frameworks` |
| Debug probes | **None attached, and none needed.** The ESP32-S3 has a built-in USB Serial/JTAG controller on GPIO19/20, so flashing and debugging need only the native USB cable. An external J-Link/ST-Link/DAPLink is optional, not required. |
| `embedded-debugger.list_probes` | Reports none because no external probe is attached. Expected, and **not** a blocker — the S3 has built-in USB Serial/JTAG. |
| USB serial devices | **None attached** at design time |

## Appendix B — Existing project assets

| Asset | Relevance |
|---|---|
| `Projects/FreeRTOS_Esp32/SerialMessaging` | FreeRTOS queues, task pinning, explicit heap alloc/free |
| `Documents/PlatformIO/.../Mic_SerialPlot` | I²S + DMA, SPI/SD, hand-built headers, register-level reasoning |
| `Downloads/Whatsapp/AircraftAR_Firmware` | ESP32 Wi-Fi streaming — **contains a `SoftwareSerial` anti-pattern not to be copied** |
| `Documents/PlatformIO/.../ESP32-S3-SDCard-Audio_Logger` | ESP32-S3 with 8 MB flash / 16 MB PSRAM, ESP-IDF style |

## Appendix C — References

- Texas Instruments, *INA219 Zerø-Drift, Bidirectional Current/Power Monitor With I²C Interface*, SBOS448G Rev. G
- Espressif, *ESP32-S3 ADC* documentation — ADC1 GPIO1–10, ADC2 GPIO11–20, ADC2 unavailable during `esp_wifi_start()`/`esp_wifi_stop()`
- Espressif, *ESP32-S3 Series Datasheet* — GPIO19 = USB D−, GPIO20 = USB D+
- Espressif, *Configure ESP32-S3 Built-in JTAG Interface* — built-in USB Serial/JTAG, no external probe required
- STMicroelectronics, *L298 Dual Full-Bridge Driver*, DS0218 Rev 5 — `VenH` 2.3 V min, `VenL` 1.5 V max (why 3.3 V logic needs no shifter), 2 A DC per channel, V<sub>CEsat</sub> total drop 1.80 V at 1 A