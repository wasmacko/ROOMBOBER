#include <cstdio>
#include <Arduino.h>
#include "motor.hpp"
#include "robot.hpp"
#include "webserver.hpp"
#include "wifimanager.hpp"
#include "tof.hpp"
#include "bumper.hpp"
#include "driver/pulse_cnt.h"

#define WIFI_SSID "NETIASPOT-H2rM"
#define WIFI_PASS "pCZk6DAq4VxK"

Robot robot;

MotorConfig motorLeftConfig {
    .speedPin = 6,
    .startPin = 4,
    .dirPin = 5,
    .encoderPin = GPIO_NUM_7,
};

MotorConfig motorRightConfig {
    .speedPin = 17,
    .startPin = 15,
    .dirPin = 16,
    .encoderPin = GPIO_NUM_18,
};

ToFBusConfig tofBusConfig {
    .sdaPin = 8,
    .sclPin = 9,
    .i2cFreqHz = 400000,
    .maxValidDist_mm = 1200,
};

BumperConfig bumperConfig {
    .leftSwitchPin = 99,    //those pins are placeholders
    .rightSwitchPin = 99,
};

// VL53L0X Sensors config (XSHUT pin, unique i2c address, X[mm], Y[mm], Angle[deg])
ToFSensorConfig tofSensorsConfig[ToFManager::NUM_SENSORS] = {
    { .xshutPin = 10, .i2cAddress = 0x30, .offsetX_mm = 80.0f,  .offsetY_mm = 60.0f,  .angle_deg = 45.0f  }, // Left
    { .xshutPin = 11, .i2cAddress = 0x31, .offsetX_mm = 100.0f, .offsetY_mm = 0.0f,   .angle_deg = 0.0f   }, // Center
    { .xshutPin = 12, .i2cAddress = 0x32, .offsetX_mm = 80.0f,  .offsetY_mm = -60.0f, .angle_deg = -45.0f }, // Right
};

volatile uint8_t autoMode = 1;
volatile uint8_t commandNum = 0;
volatile uint8_t commandEnd = 0;

// FreeRTOS background read&update task
void tofUpdateTask(void* pvParameters) {
    while (1) {
        if (robot.tof) {
            robot.tof->update();
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz update rate (enough?)
    }
}

extern "C" void app_main(void)
{
    initArduino();

    robot.initLeftMotor(motorLeftConfig);
    robot.initRightMotor(motorRightConfig);
    robot.initToF(tofBusConfig, tofSensorsConfig);
    robot.initBumper(bumperConfig);

    // i2c reading on core 1 for sensors
    xTaskCreatePinnedToCore(tofUpdateTask, "tof_task", 4096, nullptr, 5, nullptr, 1);

    //WifiManager::connect(WIFI_SSID, WIFI_PASS);
    //WebServer::start();

    robot.speed = 0.7;
    robot.uploadSpeed();
    robot.start();
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (robot.motorLeft->getEncoderValue() > 1200*2) {
            robot.stop();
        }
    }
}