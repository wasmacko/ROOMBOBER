#include "tof.hpp"
#include <Arduino.h>
#include <Wire.h>
#include "esp_log.h"

static const char* TAG = "TOF";

ToFManager::ToFManager(const ToFBusConfig &busCfg, const ToFSensorConfig sensorCfgs[NUM_SENSORS])
    : m_busCfg(busCfg) {
    for (int i = 0; i < NUM_SENSORS; i++) {
        m_sensorCfg[i] = sensorCfgs[i];
    }
}

bool ToFManager::init() {
    // 1. Put all sensors into hardware standby (XSHUT LOW)
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(m_sensorCfg[i].xshutPin, OUTPUT);
        digitalWrite(m_sensorCfg[i].xshutPin, LOW);
    }
    delay(20);

    // 2. Initialize I2C Bus
    Wire.begin(m_busCfg.sdaPin, m_busCfg.sclPin, m_busCfg.i2cFreqHz);
    delay(10);

    // 3. Wake each sensor one-by-one and assign its unique I2C address
    bool allOk = true;
    for (int i = 0; i < NUM_SENSORS; i++) {
        digitalWrite(m_sensorCfg[i].xshutPin, HIGH);
        delay(20); // Wait for VL53L0X boot

        m_sensors[i].setTimeout(200);
        if (!m_sensors[i].init()) {
            ESP_LOGE(TAG, "Sensor %d (XSHUT GPIO %d) failed to init!", i, m_sensorCfg[i].xshutPin);
            m_sensorOk[i] = 0;
            allOk = false;
            continue;
        }

        m_sensors[i].setAddress(m_sensorCfg[i].i2cAddress);
        m_sensors[i].startContinuous();
        m_sensorOk[i] = 1;
        ESP_LOGI(TAG, "Sensor %d online at I2C 0x%02X", i, m_sensorCfg[i].i2cAddress);
    }
    return allOk;
}

void ToFManager::update() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (!m_sensorOk[i]) continue;

        uint16_t dist = m_sensors[i].readRangeContinuousMillimeters();
        if (!m_sensors[i].timeoutOccurred()) {
            m_distances_mm[i] = dist;
        }
    }
}

uint16_t ToFManager::getDistanceMM(int idx) const {
    if (idx < 0 || idx >= NUM_SENSORS) return 0;
    return m_distances_mm[idx];
}

uint8_t ToFManager::isOnline(int idx) const {
    if (idx < 0 || idx >= NUM_SENSORS) return 0;
    return m_sensorOk[idx];
}

ToFSensorConfig ToFManager::getSensorConfig(int idx) const {
    if (idx < 0 || idx >= NUM_SENSORS) return {};
    return m_sensorCfg[idx];
}

void ToFManager::setSensorPosition(int idx, float x_mm, float y_mm, float angle_deg) {
    if (idx < 0 || idx >= NUM_SENSORS) return;
    m_sensorCfg[idx].offsetX_mm = x_mm;
    m_sensorCfg[idx].offsetY_mm = y_mm;
    m_sensorCfg[idx].angle_deg = angle_deg;
    ESP_LOGI(TAG, "Updated Sensor %d Pos -> X:%.1fmm, Y:%.1fmm, Angle:%.1f deg", idx, x_mm, y_mm, angle_deg);
}