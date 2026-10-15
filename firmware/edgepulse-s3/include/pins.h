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

// ── INA219 — current, in SERIES with the motor ───────────────────────
// Shares the I²C bus above at 0x40. It does not need its own pins.
//
// ⚠️ WIRING HAZARD: many INA219 breakouts pull SDA/SCL up to VIN, not
// VCC. In series with the motor, VIN IS THE 12 V BUS — those pull-ups
// would put 12 V on GPIO 8/9 and destroy the S3. With the 12 V adapter
// DISCONNECTED, measure SDA-to-VIN. If it reads open, remove the two
// onboard resistors and fit 4.7 kΩ to 3.3 V. See src/Ina219.h.
static constexpr uint8_t  INA219_I2C_ADDR = 0x40;

// Shunt resistor value marked on YOUR breakout. 0.1 Ω and 0.2 Ω are both
// common; a wrong value makes every current reading confidently wrong.
static constexpr float    INA219_SHUNT_OHMS = 0.1f;

// ── Status outputs ─────────────────────────────────────────────────────
// Onboard LED on the DevKitC-1 is GPIO48 (driven LOW for on).
// Buzzer goes through a 2N2222 with a flyback diode — never driven
// directly from a GPIO.
static constexpr uint8_t PIN_LED        = 48;
static constexpr uint8_t PIN_LED_ACTIVE = 0;   // 0 = active-low
static constexpr uint8_t PIN_BUZZER     = 16;

// ── TFT status display — ST7789 over SPI  ·  OPTIONAL ─────────────────
// Whole feature is behind EDGE_TFT_ENABLED (see src/TftDisplay.h), which
// defaults to 0. These pins exist on paper only until it is switched on.
//
// SCLK / MOSI / MISO are the ESP32-S3 FSPI native pins. They are used
// rather than matrixed alternatives because the GPIO matrix detour costs
// throughput, and the TFT is the only SPI device in the build.
//
// CS is GPIO5, deliberately NOT GPIO9. GPIO9 is this project's I²C SCL
// (PIN_I2C_SCL above) and is about to carry the INA219 as well; a second
// device on that line would be a hard conflict, not a shared one. GPIO5 is
// free, is not a strapping pin, and is well outside the flash/PSRAM bank.
//
// DC is GPIO10, which is the controller's FSPICS0 pin. It is safe to use
// as data/command because the driver drives CS itself and never attaches
// the peripheral's hardware SS (Adafruit_SPITFT only calls setHwCs() when
// a transaction declares use_ss, and SPISettings(freq, MSBFIRST, mode)
// never does). Using the native CS0 pin as DC also leaves the panel's
// chip-select entirely in software, where it can be seen.
//
// RST is GPIO21 and BL is GPIO14. BL is the only optional wire here: many
// breakouts strap the backlight to 3V3 directly, and EDGE_TFT_BACKLIGHT=0
// in TftDisplay.h turns the drive off for those.
static constexpr uint8_t PIN_TFT_SCLK = 12;
static constexpr uint8_t PIN_TFT_MOSI = 11;
static constexpr uint8_t PIN_TFT_MISO = 13;  // unused by the driver today; wire
                                           // it anyway so the panel is
                                           // complete for a later readback
static constexpr uint8_t PIN_TFT_DC   = 10;
static constexpr uint8_t PIN_TFT_CS   = 5;
static constexpr uint8_t PIN_TFT_RST  = 21;
static constexpr uint8_t PIN_TFT_BL   = 14;

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
//
// The DHT22 interval must be SHORTER than the feature window. With both
// at 2000 ms, dhtTask and featureTask wake on the same boundary;
// featureTask runs first (higher priority, different core) and publishes
// the ambient cache before dhtTask has refreshed it, so roughly half the
// windows carry a stale or empty ambient/humidity reading. 1200 ms gives
// a guaranteed 800 ms margin before each 2000 ms window.
static constexpr uint32_t DS18B20_INTERVAL_MS = 800;   // 12-bit conversion
static constexpr uint32_t DHT22_INTERVAL_MS   = 1200;  // < WINDOW_MS, see note