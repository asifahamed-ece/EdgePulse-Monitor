// ═══════════════════════════════════════════════════════════════════════
//  EdgePulse — L298N motor driver
//  BOARD: Espressif ESP32-S3-DevKitC-1
//
//  Replaces the Arduino Uno + ZORLINA L293D shield entirely. The ESP32-S3
//  drives the L298N directly over three GPIOs; there is no second
//  microcontroller and no UART link.
//
//  Why this is electrically valid (ST L298 datasheet DS0218 Rev 5):
//      VenH  (enable high)  2.3 V min
//      VenL  (enable low)   1.5 V max
//  The ESP32-S3 drives 3.3 V — above 2.3 V for HIGH, and its near-zero V
//  output is below 1.5 V for LOW. 3.3 V logic is therefore valid and NO
//  level shifter or divider is required. ESP32 GPIO is not 5 V tolerant,
//  so 5 V must never reach these pins — the L298N's 5 V pin is left
//  unconnected to the ESP32 entirely.
//
//  TRUTH TABLE (one bridge)
//      ENA  IN1 IN2   Result
//      0    x   x     Stop / coast
//      1    0   0     Brake (both low, motor terminals shorted)
//      1    0   1     Forward
//      1    1   0     Reverse
//      1    1   1     Brake
//
//  KNOWN LOSS: the L298 is a bipolar TTL driver. Datasheet total drop is
//  1.80 V typical at 1 A (4.9 V worst case at 2 A). A 12 V supply
//  therefore delivers roughly 10 V to the motor terminals. Record this
//  operating point in the experiment log — the motor is NOT running at
//  its nominal 12 V.
// ═══════════════════════════════════════════════════════════════════════

#pragma once
#include <stdint.h>

class MotorDriver {
public:
    // dir: 0 = forward, 1 = reverse. speed: 0-255.
    void begin(uint8_t enaPin, uint8_t in1Pin, uint8_t in2Pin,
               uint32_t pwmHz = 16000);

    void setSpeed(uint8_t speed, uint8_t dir = 0);
    void run(void);
    void stop(void);      // coast: ENA low
    void brake(void);     // IN1 = IN2 = low with ENA high

    uint8_t speed(void) const { return _speed; }
    uint8_t dir(void)   const { return _dir; }
    bool    running(void) const { return _running; }

    // Commanded setpoint, in hardware terms — recorded in the CSV so the
    // dataset carries the real command rather than a human assumption.
    uint16_t duty(void) const { return _duty; }

private:
    void apply(void);

    uint8_t  _ena, _in1, _in2;
    uint8_t  _speed = 0;
    uint8_t  _dir   = 0;
    uint16_t _duty  = 0;
    bool     _running = false;
};