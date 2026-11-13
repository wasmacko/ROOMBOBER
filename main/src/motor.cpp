#include "motor.hpp"
#include <cmath>
#include <Arduino.h>

Motor::Motor(const MotorConfig &config) : m_cfg(config) {
    ledcAttach(m_cfg.speedPin, 16000, 8);
    pinMode(m_cfg.startPin, OUTPUT);
    pinMode(m_cfg.dirPin, OUTPUT);
    pinMode(m_cfg.encoderPin, INPUT_PULLUP);
    digitalWrite(m_cfg.startPin, HIGH);
    digitalWrite(m_cfg.dirPin, LOW);
}

void Motor::forward() {
    m_direction = 0;
    digitalWrite(m_cfg.dirPin, LOW);
}
void Motor::backward() {
    m_direction = 1;
    digitalWrite(m_cfg.dirPin, HIGH);
}
uint8_t Motor::direction() const {
    return m_direction;
}
void Motor::start() {
    m_on = 1;
    digitalWrite(m_cfg.startPin, LOW);
}
void Motor::stop() {
    m_on = 0;
    digitalWrite(m_cfg.startPin, HIGH);
}
uint8_t Motor::isOn() const {
    if (m_on) {
        return 1;
    }
    return 0;
};
void Motor::setSpeed(double speedValPercent) {   //speedVal is percentage!
    uint8_t speedVal;
    if (speedValPercent > 1) {
        speedVal = 0;
    } else if (speedValPercent < 0) {
        speedVal = 255;
    } else {
        speedVal = 255 - round(255 * speedValPercent);
    }
    ledcWrite(m_cfg.speedPin, speedVal);
};
uint8_t Motor::stopped() {
    if (!m_hasLastValue) {
        m_lastEncoderValue = encoderValue;
        m_hasLastValue = 1;
        return 0;
    }
    if (m_lastEncoderValue == encoderValue) {
        m_hasLastValue = 0;
        return 1;
    }
    m_lastEncoderValue = encoderValue;
    return 0;
};