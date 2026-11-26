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
    m_mutex = xSemaphoreCreateMutex();
    ledcAttach(m_cfg.speedPin, 16000, 8);
    pinMode(m_cfg.startPin, OUTPUT);
    pinMode(m_cfg.dirPin, OUTPUT);
    digitalWrite(m_cfg.startPin, HIGH);
    digitalWrite(m_cfg.dirPin, LOW);
    m_encoder = init_single_encoder(m_cfg.encoderPin);
}

Motor::~Motor() {
    if (m_mutex) {
        vSemaphoreDelete(m_mutex);
    }
}

void Motor::syncEncoderLocked() {
    int currentRaw = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(m_encoder, &currentRaw));
    int delta = currentRaw - m_lastRawEncoder;
    m_lastRawEncoder = currentRaw;

    int8_t dirSign = (m_direction == 0) ? 1 : -1;
    m_signedEncoder += (int64_t)delta * dirSign;
}

int64_t Motor::getSignedEncoderValue() {
    if (!m_mutex) return m_signedEncoder;
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    syncEncoderLocked();
    int64_t val = m_signedEncoder;
    xSemaphoreGive(m_mutex);
    return val;
}

int Motor::getEncoderValue() {
    return (int)getSignedEncoderValue();
}

void Motor::resetEncoder() {
    if (!m_mutex) return;
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    ESP_ERROR_CHECK(pcnt_unit_clear_count(m_encoder));
    m_lastRawEncoder = 0;
    m_signedEncoder = 0;
    xSemaphoreGive(m_mutex);
}

void Motor::forward() {
    if (m_direction == 0) return;
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    syncEncoderLocked();
    m_direction = 0;
    digitalWrite(m_cfg.dirPin, LOW);
    xSemaphoreGive(m_mutex);
}

void Motor::backward() {
    if (m_direction == 1) return;
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    syncEncoderLocked();
    m_direction = 1;
    digitalWrite(m_cfg.dirPin, HIGH);
    xSemaphoreGive(m_mutex);
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
    return m_on ? 1 : 0;
}

void Motor::setSpeed(double speedValPercent) {
    uint8_t speedVal;
    if (speedValPercent >= 1.0) {
        speedVal = 0;
    } else if (speedValPercent <= 0.0) {
        speedVal = 255;
    } else {
        speedVal = 255 - (uint8_t)round(255.0 * speedValPercent);
    }
    ledcWrite(m_cfg.speedPin, speedVal);
}