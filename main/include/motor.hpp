#pragma once
#include <stdint.h>
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

struct MotorConfig {
    uint8_t speedPin;
    uint8_t startPin;
    uint8_t dirPin;
    gpio_num_t encoderPin;
};

class Motor {
private:
    MotorConfig m_cfg;
    uint8_t m_direction = 0; // 0 = forward, 1 = backward
    uint8_t m_on = 0;        // 0 = off, 1 = on
    pcnt_unit_handle_t m_encoder = nullptr;
    SemaphoreHandle_t m_mutex = nullptr;

    int m_lastRawEncoder = 0;
    int64_t m_signedEncoder = 0;

    pcnt_unit_handle_t init_single_encoder(gpio_num_t pin);
    void syncEncoderLocked();

public:
    Motor(const MotorConfig &config);
    ~Motor();

    void forward();
    void backward();
    uint8_t direction() const;

    void start();
    void stop();
    uint8_t isOn() const;

    int getEncoderValue();            // impulses with sign +-
    int64_t getSignedEncoderValue();  // counter 64bit
    void resetEncoder();              // reset impules counter
    void setSpeed(double speedValPercent);
};