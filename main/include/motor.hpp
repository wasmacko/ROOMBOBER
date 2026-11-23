#pragma once
#include <stdint.h>
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

struct MotorConfig {
    uint8_t speedPin;
    uint8_t startPin;
    uint8_t dirPin;
    gpio_num_t encoderPin;
};

class Motor {
private:
MotorConfig m_cfg;

uint8_t m_direction = 0; //0 means forward, 1 backward
uint8_t m_on = 0;   //0 off, 1 on
uint64_t m_lastEncoderValue;
uint8_t m_hasLastValue = 0;

pcnt_unit_handle_t m_encoder;
int m_encoderValue;

pcnt_unit_handle_t init_single_encoder(gpio_num_t pin);

public:
Motor(const MotorConfig &config);
~Motor();

void forward();
void backward();
uint8_t direction() const;
void start();
uint8_t stop();
uint8_t isOn() const;
int getEncoderValue();
void setSpeed(double speedValPercent);
};