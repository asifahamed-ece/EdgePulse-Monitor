#include "MotorDriver.h"
#include <Arduino.h>

// LEDC timer/channel. Channel 0 is used; the S3 has 8.
static constexpr uint8_t LEDC_CH = 0;
static constexpr uint8_t LEDC_RES = 8;      // 8-bit duty = 0-255
static constexpr uint32_t PWM_HZ = 16000;   // above audible, avoids adding
                                            // PWM ripple to the readings

void MotorDriver::begin(uint8_t enaPin, uint8_t in1Pin, uint8_t in2Pin,
                        uint32_t pwmHz) {
    _ena = enaPin; _in1 = in1Pin; _in2 = in2Pin;

    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);

    // LEDC API as actually declared in esp32-hal-ledc.h for the core in
    // use: ledcSetup(channel, freq, bits) then ledcAttachPin(pin, ch).
    // (A `ledcAttach(pin, freq, res)` form does not exist here — the
    // compiler rejects it, so this header is the authority.)
    ledcSetup(LEDC_CH, pwmHz, LEDC_RES);
    ledcAttachPin(_ena, LEDC_CH);
    ledcWrite(LEDC_CH, 0);

    stop();
}

void MotorDriver::setSpeed(uint8_t speed, uint8_t dir) {
    _speed = speed;
    _dir   = dir ? 1 : 0;
    _running = (speed > 0);
    _duty  = speed;              // 8-bit duty maps 1:1 to 0-255
    apply();
}

void MotorDriver::run(void) {
    _running = true;
    apply();
}

void MotorDriver::stop(void) {
    _running = false;
    _speed = 0;
    _duty  = 0;
    // Coast: ENA low. Direction inputs parked low so the motor is not
    // driven into an undefined state on the next start.
    ledcWrite(LEDC_CH, 0);
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
}

void MotorDriver::brake(void) {
    // Both direction inputs low with ENA high shorts the motor terminals,
    // which produces a braking torque rather than free-running.
    _running = false;
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
    ledcWrite(LEDC_CH, _duty ? _duty : 255);
}

void MotorDriver::apply(void) {
    if (!_running || _duty == 0) {
        ledcWrite(LEDC_CH, 0);
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, LOW);
        return;
    }
    digitalWrite(_in1, _dir ? LOW : HIGH);
    digitalWrite(_in2, _dir ? HIGH : LOW);
    ledcWrite(LEDC_CH, _duty);
}