// ═══════════════════════════════════════════════════════════════════════
//  EdgePulse — pin map
//  BOARD: Espressif ESP32-S3-DevKitC-1 (generic)   PlatformIO: esp32-s3-devkitc-1
//
//  This file is the SINGLE SOURCE OF TRUTH for pin assignments.
//  docs/WIRING.md mirrors it for the physical build. If a pin changes
//  here, change it there too — and re-check §3.2 of the design spec.
//
//  Verified against:
//    .platformio/platforms/espressif32/boards/esp32-s3-devkitc-1.json
// ═══════════════════════════════════════════════════════════════════════

#pragma once
#include <stdint.h>

// ── I²C bus — shared by MPU6050 (0x68) and, later, INA219 (0x40) ──────
// Pull-ups: 4.7 kΩ to 3.3 V on SDA and SCL.
static constexpr uint8_t PIN_I2C_SDA = 8;
static constexpr uint8_t PIN_I2C_SCL = 9;

// ── DS18B20 waterproof probe — motor surface temperature ──────────────
// 3-wire (VDD, GND, DQ) with a 4.7 kΩ pull-up to 3.3 V.
// NOT parasitally powered: a contact probe on a motor has poor
// capacitive coupling and parasite mode is unreliable here.
// 12-bit resolution = 0.0625 °C, ~750 ms conversion.
static constexpr uint8_t PIN_DS18B20 = 6;

// ── DHT22 — ambient temperature and relative humidity ─────────────────
// ~0.5 Hz effective. Blocks ~5 ms per read, so it must NOT run on the
// acquisition core (spec §4.1).
static constexpr uint8_t PIN_DHT22 = 7;

// ── L298N motor driver — ESP32-S3 drives it directly ──────────────────
// No Arduino Uno, no L293D shield, no UART link, no level shifter.
//
// ST L298 datasheet DS0218 Rev 5:
//     VenH (enable HIGH)  2.3 V min      ← ESP32 drives 3.3 V, passes
//     VenL (enable LOW)   1.5 V max      ← ESP32 low ~0 V, passes
// So 3.3 V logic is valid. Never connect the L298N's 5 V pin to the
// ESP32: ESP32 GPIO is not 5 V tolerant.
//
// PWM frequency is set above audible range so the switching ripple does
// not contaminate the vibration or current measurements.
static constexpr uint8_t  PIN_L298_ENA  = 4;   // speed  (LEDC hardware PWM)
static constexpr uint8_t  PIN_L298_IN1  = 17;  // direction
static constexpr uint8_t  PIN_L298_IN2  = 18;  // direction
static constexpr uint32_t MOTOR_PWM_HZ  = 16000;

// ── Status outputs ─────────────────────────────────────────────────────
// Onboard LED on the DevKitC-1 is GPIO48 (driven LOW for on).
// Buzzer goes through a 2N2222 with a flyback diode — never driven
// directly from a GPIO.
static constexpr uint8_t PIN_LED        = 48;
static constexpr uint8_t PIN_LED_ACTIVE = 0;   // 0 = active-low
static constexpr uint8_t PIN_BUZZER     = 16;

// ── RESERVED — do not assign ──────────────────────────────────────────
// GPIO19 / GPIO20 are USB D− / D+ for the native USB CDC interface.
// Any sensor wired here breaks flashing and serial logging.
static constexpr uint8_t USB_D_MINUS = 19;
static constexpr uint8_t USB_D_PLUS  = 20;

// ── Acquisition timing ────────────────────────────────────────────────
static constexpr uint32_t HZ             = 100;   // target sample rate
static constexpr uint32_t US_PER_SAMPLE  = 1000000 / HZ;
static constexpr uint16_t WINDOW_SAMPLES = 200;   // 2 s at 100 Hz
static constexpr uint32_t WINDOW_MS      = (WINDOW_SAMPLES * 1000) / HZ;

// ── Slow-sensor intervals ─────────────────────────────────────────────
// Real sensor periods. Never pretend these are 100 Hz.
static constexpr uint32_t DS18B20_INTERVAL_MS = 800;   // 12-bit conversion
static constexpr uint32_t DHT22_INTERVAL_MS   = 2000;  // sensor minimum