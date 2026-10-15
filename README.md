# EdgePulse

**Multisensor Edge-Based Anomaly Detection for DC Motor Health Monitoring**

> **1st Runner-Up** — SmartX Hackathon · Vehicle Systems Category
> Built in **8 hours**, AI-accelerated.

An ESP32-S3 watches a small brushed DC motor through vibration, temperature and
current, records every sample to CSV, and reports a NORMAL / WARNING / CRITICAL
state on a live dashboard served from the chip itself.

---

## The idea

Brushed DC motors fail gradually. Current draw creeps up as bearings wear,
vibration amplitude rises as a shaft develops a bend, and case temperature
climbs long before anyone notices. Today that is caught by a technician
listening to the motor or clamping a multimeter to it — which means it is
caught only when someone is already looking, or far too late.

**EdgePulse puts the same judgement on the device itself.** The ESP32-S3
samples the motor continuously, computes features over a two-second window,
compares them against a measured baseline, and raises a state change the
moment behaviour drifts outside what the motor actually did when it was new.
Everything happens on the chip. Nothing is sent to a cloud service, and the
system keeps working when the network does not.

That constraint is the point. A cloud dashboard that needs internet is
useless in a vehicle bay, a factory floor, or a field deployment. A $4
microcontroller that needs nothing but power is not.

### What it measures

| Channel | Sensor | What it captures |
|---|---|---|
| Vibration | MPU6050 (I²C) | 3-axis accelerometer + gyroscope at 100 Hz |
| Motor temperature | DS18B20 (1-Wire) | Surface temperature, 12-bit |
| Ambient | DHT22 | Room temperature and humidity |
| Current | INA219 (I²C) | Motor current, bus voltage, power |

### How it decides

Vibration magnitude is computed, the window mean is subtracted to remove
gravity, and three features are extracted per two-second window:

`vib_rms` · `vib_variance` · `vib_peak_to_peak`

Each is compared against a mean and standard deviation measured from real
baseline runs. Crossing μ + 3σ raises WARNING; μ + 5σ, or a configured
absolute temperature limit, raises CRITICAL. The dashboard names the
specific feature and limit that fired — not just a red light.

---

## How it works

```
MPU6050 ──┐
INA219  ──┤                      ┌─────────────────┐        ┌──────────────┐
DS18B20 ──┼─► ESP32-S3 ─────────►│  2 s window     │───────►│  thresholds  │
DHT22   ──┘   CORE 1 @ 100 Hz    │  features       │        │  NORMAL      │
          │                       └─────────────────┘        │  WARNING     │
          │  CORE 0                                   │     │  CRITICAL    │
          └──► CSV recorder ──► edgepulse_raw.csv         │     └──────┬───────┘
          └──► SoftAP + WebSocket ──► browser dashboard ◄──────┘
                 L298N ──► DC motor (3 GPIO, no second MCU)
```

The acquisition task runs on a hardware timer at 100 Hz and is never gated by
a slow sensor or a slow consumer. A DHT22 read blocks for 250 ms, so it lives
on the other core entirely. A dropped sample is counted and logged rather than
smoothed away.

### The motor driver

The ESP32-S3 drives an L298N H-bridge directly over three GPIOs — no Arduino
Uno, no level shifter, no UART link. 3.3 V logic is valid for the L298
(`VenH` 2.3 V min, `VenL` 1.5 V max, per the ST datasheet), so the controller
that records the commanded PWM is the same one that commands the motor.

---

## Honesty as a design constraint

The rule this project is built around: **a value is only reported if it was
actually measured.**

- A sensor that is absent, unaddressed, or returning an implausible reading
  yields a **blank CSV field** — never `0.0`. Zero is a measurement; blank is
  not.
- Thresholds that have not been calibrated from real runs report
  **UNCALIBRATED**, not a confident-looking NORMAL.
- Every CSV row carries a `sensor_status` bitmask and a `dropped_samples`
  counter, so missing data is visible in the data rather than buried.
- Speed variation is labelled as an **operating condition**, never as a fault.

This is not decoration. An earlier build showed smoothly moving synthetic
values, and it was rejected on that basis. Every number this system displays
now traces back to a real sensor read.

---

## Hardware

| Component | Role |
|---|---|
| ESP32-S3 DevKitC-1 (8 MB, no PSRAM) | Acquisition, features, thresholds, AP, WebSocket |
| MPU6050 | Vibration, 100 Hz |
| DS18B20 waterproof probe | Motor surface temperature |
| DHT22 | Ambient temperature and humidity |
| INA219 | Current, bus voltage, power |
| L298N module | Motor H-bridge, driven directly by the ESP32 |

### Pin map

| Function | GPIO | Notes |
|---|---|---|
| I²C SDA / SCL | 8 / 9 | Shared by MPU6050 and INA219, 4.7 kΩ pull-ups |
| DS18B20 DQ | 6 | 1-Wire, 4.7 kΩ pull-up, 3-wire (not parasite) |
| DHT22 DATA | 7 | — |
| L298N ENA / IN1 / IN2 | 4 / 17 / 18 | Remove the ENA jumper |
| Onboard LED | 48 | — |
| USB D− / D+ | 19 / 20 | **Reserved — never assign** |

> ⚠️ **INA219 wiring hazard:** many INA219 breakouts pull SDA/SCL up to **VIN**,
> not VCC. In series with the motor, VIN **is** the 12 V bus — those pull-ups
> would put 12 V on GPIO 8/9 and destroy the chip. With the motor supply
> disconnected, measure SDA-to-VIN. If it reads open, replace the onboard
> resistors with 4.7 kΩ to 3.3 V.

---

## Build and run

```bash
cd firmware/edgepulse-s3
pio run -t uploadfs     # flash the dashboard assets
pio run -t upload       # flash the firmware
pio device monitor      # optional: watch the [DBG] / [STATUS] line
```

Then join the `EdgePulse` access point and open `http://192.168.4.1`.

Requires PlatformIO with the `espressif32` platform. No internet connection is
needed at runtime — every asset is served from the chip's own flash.

---

## Repository layout

```
firmware/edgepulse-s3/     ESP32-S3 firmware (PlatformIO, Arduino framework)
  include/pins.h           Single source of truth for the pin map
  src/Sensors.*            MPU6050 / DS18B20 / DHT22 drivers
  src/Features.*           Window feature extraction
  src/Rules.*              Threshold evaluation
  src/CsvRecorder.*        Batched CSV logging
  src/MotorDriver.*        L298N control
  src/TftDisplay.*         Optional ST7789 status display
dashboard/data/            Web dashboard, served from LittleFS
docs/                      Design spec, wiring reference
tools/                     Host-side analysis (feature extraction, calibration)
data/                      Recorded CSV output
```

---

## Scope: what this does and does not claim

Built and measured:

- 100 Hz acquisition with verified zero dropped samples
- Real vibration, temperature, humidity and current capture
- Feature extraction and a live dashboard

**Not claimed.** No confirmed fault classification, no remaining-useful-life
prediction, no failure forecasting, no vehicle-level validation, no quantified
downtime reduction, and no accuracy figure that was not measured on held-out
runs. A bench DC motor under controlled conditions is a prototype, not a
certified condition-monitoring system.

---

## Built under the 8-hour constraint, with AI

This was a **1st runner-up** finish in an **8-hour** hackathon. The honest
account of how it happened: AI-assisted development under a hard time limit.

That constraint shaped the work in both directions, and it is worth being
precise about which helped and which did not.

**Where it helped.** Bringing up an unfamiliar driver stack, reading three
datasheets, and wiring five sensors to a fresh board are all dominated by
retrieval, not by judgement. Compressing that work is what made a working
multisensor device with a live dashboard possible in the window available.
Roughly 30 of the project's 47 commits landed before the final hour.

**Where it did not.** A judge caught the build displaying **synthetic data**
that looked entirely plausible. No real sensor was connected, and instead of an
empty dashboard the code generated a smooth waveform. It was convincing,
animated, and worthless — and it was only caught because someone asked a
question.

That failure shaped the architecture that followed. The "honesty as a design
constraint" section above is a direct response to it: blank instead of zero,
UNCALIBRATED instead of false NORMAL, fault counters on every read. **AI was
very good at producing something that looked like a working motor monitor.
It was bad at noticing that the motor monitor had no motor.** That
distinction — between output that reads as correct and output that is correct
— turned out to be the whole project.

A generated-looking system is cheap. A trustworthy one is the work.
