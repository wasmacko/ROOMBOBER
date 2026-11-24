#include "motor.hpp"
#include <cmath>
#include <Arduino.h>

pcnt_unit_handle_t Motor::init_single_encoder(gpio_num_t pin) {
    pcnt_unit_config_t unit_config = {
        .low_limit = -30000,
        .high_limit = 30000,
        .flags = { .accum_count = true }
    };
    pcnt_unit_handle_t unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 5000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter_config));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = pin,
        .level_gpio_num = -1, 
    };
    pcnt_channel_handle_t chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_config, &chan));
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan, PCNT_CHANNEL_EDGE_ACTION_HOLD, PCNT_CHANNEL_EDGE_ACTION_INCREASE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, 30000));

    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));

    return unit;
}

Motor::Motor(const MotorConfig &config) : m_cfg(config) {
    ledcAttach(m_cfg.speedPin, 16000, 8);
    pinMode(m_cfg.startPin, OUTPUT);
    pinMode(m_cfg.dirPin, OUTPUT);
    digitalWrite(m_cfg.startPin, HIGH);
    digitalWrite(m_cfg.dirPin, LOW);
    m_encoder = init_single_encoder(m_cfg.encoderPin);
}
Motor::~Motor() {}

int Motor::getEncoderValue() {
    ESP_ERROR_CHECK(pcnt_unit_get_count(m_encoder, &m_encoderValue));
    return m_encoderValue; 
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