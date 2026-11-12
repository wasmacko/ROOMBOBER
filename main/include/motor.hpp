#pragma once
#include <stdint.h>

struct MotorConfig {
    uint8_t speedPin;
    uint8_t startPin;
    uint8_t dirPin;
    uint8_t encoderPin; 
};

class Motor {
private:
MotorConfig m_cfg;

uint8_t m_direction = 0; //0 means forward, 1 backward
uint8_t m_on = 0;

public:
uint64_t encoderValue = 0;

Motor(const MotorConfig &config);

void forward();
void backward();
uint8_t direction() const;
void start();
void stop();
uint8_t isOn() const;
void setSpeed(double speedValPercent);
};