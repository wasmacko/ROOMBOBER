#pragma once
#include <stdint.h>
#include "VL53L0X.h"

struct ToFSensorConfig {
    uint8_t xshutPin;       // GPIO connected to XSHUT
    uint8_t i2cAddress;     // Unique I2C address (e.g. 0x30, 0x31, 0x32)
    float offsetX_mm;       // Sensor X position relative to robot center (mm, forward is +)
    float offsetY_mm;       // Sensor Y position relative to robot center (mm, left is +)
    float angle_deg;        // Sensor facing angle in degrees (0 = straight ahead, +45 = left, -45 = right)
};

struct ToFBusConfig {
    uint8_t sdaPin;
    uint8_t sclPin;
    uint32_t i2cFreqHz = 400000;
    uint16_t maxValidDist_mm = 1200; // Ignore noisy readings above this distance
};

class ToFManager {
public:
    static constexpr int NUM_SENSORS = 3;

private:
    ToFBusConfig m_busCfg;
    ToFSensorConfig m_sensorCfg[NUM_SENSORS];
    VL53L0X m_sensors[NUM_SENSORS];
    uint16_t m_distances_mm[NUM_SENSORS] = {0, 0, 0};
    uint8_t m_sensorOk[NUM_SENSORS] = {0, 0, 0};

public:
    ToFManager(const ToFBusConfig &busCfg, const ToFSensorConfig sensorCfgs[NUM_SENSORS]);

    bool init();
    void update();

    uint16_t getDistanceMM(int idx) const;
    uint8_t isOnline(int idx) const;
    uint16_t getMaxValidDistMM() const { return m_busCfg.maxValidDist_mm; }
    void setMaxValidDistMM(uint16_t maxDist) { m_busCfg.maxValidDist_mm = maxDist; }

    // Getter & Setter for runtime configuration (via code or Web UI)
    ToFSensorConfig getSensorConfig(int idx) const;
    void setSensorPosition(int idx, float x_mm, float y_mm, float angle_deg);
};