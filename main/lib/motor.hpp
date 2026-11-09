#pragma once
#include <cmath>
#include <Arduino.h>

class Motor {
private:
uint8_t m_speedPin;
uint8_t m_startPin;
uint8_t m_dirPin;
uint8_t m_encoderPin;

uint8_t m_direction = 0; //0 means forward, 1 backward

public:
uint64_t encoderValue = 0;

Motor(const uint8_t &speedPin, const uint8_t &startPin, const uint8_t &dirPin, const uint8_t &encoderPin) : 
            m_speedPin(speedPin), m_startPin(startPin), m_dirPin(dirPin), m_encoderPin(encoderPin) {
    ledcAttach(m_speedPin, 16000, 8);
}

void forward() {
    m_direction = 0;
    digitalWrite(m_dirPin, LOW);
}
void backward() {
    m_direction = 1;
    digitalWrite(m_dirPin, HIGH);
}
uint8_t direction() const {
    return m_direction;
}
void start() const {
    digitalWrite(m_startPin, LOW);
}
void stop() const {
    digitalWrite(m_startPin, HIGH);
}
void setSpeed(const double_t &speedValPercent) {   //speedVal is percentage!
    uint8_t speedVal;
    if (speedValPercent > 1) {
        speedVal = 0;
    } else if (speedValPercent < 0) {
        speedVal = 1;
    } else {
        speedVal = 255 - round(255 * speedValPercent);
    }
    ledcWrite(m_speedPin, speedVal);
    delay(100);
}
};