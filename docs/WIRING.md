// ═══════════════════════════════════════════════════════════════════════
//  EdgePulse — physical wiring reference
//  BOARD: Espressif ESP32-S3-DevKitC-1 (generic)
//
//  Mirror of firmware/edgepulse-s3/include/pins.h. Keep both in sync.
//  Verify electrical safety in §3.4 of the design spec BEFORE powering.
// ═══════════════════════════════════════════════════════════════════════

## I²C bus (shared)

| ESP32-S3 | Module | Module pin |
|---|---|---|
| GPIO 8 | MPU6050 | SDA |
| GPIO 9 | MPU6050 | SCL |
| 3V3 | MPU6050 | VCC |
| GND | MPU6050 | GND |

**4.7 kΩ pull-up from SDA to 3V3, and from SCL to 3V3.**

- MPU6050 address is `0x68` when AD0 is low. If your module has AD0 tied
  high it becomes `0x69` — check before assuming.
- Power the MPU6050 from **3.3 V**, not 5 V.
- The INA219 (deferred) joins this same bus at `0x40` later.

⚠️ **When you add the INA219:** many breakouts pull SDA/SCL up to **VCC**,
but some wire them to **VIN**. With the 12 V adapter **disconnected**,
measure resistance from SDA to VIN. If it reads to VIN, 12 V would be
pushed onto GPIO 8/9 and destroy the ESP32. Remove those resistors and
fit 4.7 kΩ to 3.3 V.

## DS18B20 waterproof probe (motor surface temperature)

| ESP32-S3 | Module |
|---|---|
| GPIO 6 | DQ (data) |
| 3V3 | VCC |
| GND | GND |

**4.7 kΩ pull-up from DQ to 3V3.** Use 3-wire, NOT parasite power.

## DHT22 (ambient temperature and humidity)

| ESP32-S3 | Module |
|---|---|
| GPIO 7 | DATA |
| 3V3 | VCC |
| GND | GND |

No pull-up needed — DHT22 has one on board. Reads at ~0.5 Hz; blocks ~5 ms.

## L298N motor driver — ESP32-S3 drives it directly

**No Arduino Uno. No L293D shield. No level shifter. No divider.**

| ESP32-S3 | L298N | Purpose |
|---|---|---|
| GPIO 4 | ENA | Speed — LEDC hardware PWM at 16 kHz |
| GPIO 17 | IN1 | Direction |
| GPIO 18 | IN2 | Direction |
| GND | GND | **Common ground — mandatory** |

Remove the **ENA jumper** from the L298N so GPIO 4 can drive it.
The **ENB jumper** stays on if bridge B is unused.

### Why 3.3 V logic works

ST L298 datasheet (DS0218 Rev 5):

| Parameter | Value |
|---|---|
| `VenH` — enable HIGH threshold | **2.3 V min** |
| `VenL` — enable LOW threshold | **1.5 V max** |

The ESP32-S3 drives 3.3 V — above 2.3 V for HIGH, and near 0 V which is
below 1.5 V for LOW. **No level shifting is required.**

### 5 V pin — leave it alone

**Do not connect the L298N 5 V pin to the ESP32.** ESP32 GPIO is not 5 V
tolerant. With a 12 V motor supply the onboard 78M05 regulator feeds the
driver's logic section correctly on its own; the ESP32 is powered from USB.

### Expected voltage drop — record this

The L298 is a bipolar TTL driver. Datasheet total drop:

| Current | Total drop |
|---|---|
| 1 A | 1.80 V typical (3.2 V max) |
| 2 A | 4.90 V max |

A 12 V supply therefore delivers roughly **10 V to the motor terminals**.
The motor is **not** running at its nominal 12 V. Log this operating point
in the experiment log — it is part of the measured condition, not an
afterthought.

### Control truth table

| ENA | IN1 | IN2 | Result |
|---|---|---|---|
| 0 | x | x | Stop / coast |
| 1 | 0 | 0 | Brake |
| 1 | 0 | 1 | Forward |
| 1 | 1 | 0 | Reverse |
| 1 | 1 | 1 | Brake |

### WebSocket commands

```json
{"cmd":"SET_PWM","pwm":128,"dir":0}
{"cmd":"RUN"}   {"cmd":"STOP"}   {"cmd":"BRAKE"}
```

The firmware applies and records the commanded PWM **in the same place**,
so the CSV's `motor_setting` column holds the value actually sent to the
driver rather than a number a human remembered.

## Status outputs

| ESP32-S3 | Device | Notes |
|---|---|---|
| GPIO 48 | Onboard LED | DevKitC-1 LED. Active **low** |
| GPIO 16 | Buzzer via 2N2222 | Add a flyback diode across the buzzer. **Never drive a buzzer directly from a GPIO** |

## ⚠️ RESERVED — never wire here

| GPIO | Function |
|---|---|
| **19** | USB D− (native USB CDC) |
| **20** | USB D+ (native USB CDC) |

Anything connected to GPIO 19/20 breaks flashing and serial logging.

## Do not use

| GPIO | Why |
|---|---|
| 0, 3, 45, 46 | Boot strapping pins — wrong state at reset bricks the boot |
| 26–37 | Reserved for flash/PSRAM on many S3 modules |

## Power

- ESP32-S3: USB-C
- Motor: **12 V adapter → L298N 12V terminal only** (never through an ESP32 GPIO)
- Never power the motor from an ESP32 GPIO.

## Before first power-on (spec §3.4)

1. Motor rated voltage and current identified.
2. Motor current measured with the **shield disconnected**. **Above 0.6 A the
   motor stays disconnected** — 1.2 A is peak, not continuous.
3. Every signal entering the ESP32 confirmed within 3.3 V. **ESP32 GPIO is
   not 5 V tolerant.**
4. 12 V adapter rating checked against the shield's motor-power input.
5. Motor secured, rotating shaft guarded, exposed wiring insulated.