#include <cstdio>
#include <Arduino.h>
#include "motor.hpp"
#include "robot.hpp"
#include "webserver.hpp"
#include "wifimanager.hpp"
#include "tof.hpp"

#define WIFI_SSID "BUBA3"
#define WIFI_PASS "KN14738x"

Robot robot;

MotorConfig motorLeftConfig {
    .speedPin = 4,
    .startPin = 5,
    .dirPin = 6,
    .encoderPin = 7,
};

MotorConfig motorRightConfig {
    .speedPin = 15,
    .startPin = 16,
    .dirPin = 17,
    .encoderPin = 18,
};

ToFBusConfig tofBusConfig {
    .sdaPin = 8,
    .sclPin = 9,
    .i2cFreqHz = 400000,
    .maxValidDist_mm = 1200,
};

// VL53L0X Sensors config (XSHUT pin, unique i2c address, X[mm], Y[mm], Angle[deg])
ToFSensorConfig tofSensorsConfig[ToFManager::NUM_SENSORS] = {
    { .xshutPin = 10, .i2cAddress = 0x30, .offsetX_mm = 80.0f,  .offsetY_mm = 60.0f,  .angle_deg = 45.0f  }, // Left
    { .xshutPin = 11, .i2cAddress = 0x31, .offsetX_mm = 100.0f, .offsetY_mm = 0.0f,   .angle_deg = 0.0f   }, // Center
    { .xshutPin = 12, .i2cAddress = 0x32, .offsetX_mm = 80.0f,  .offsetY_mm = -60.0f, .angle_deg = -45.0f }, // Right
};

volatile uint8_t autoMode = 1;

void IRAM_ATTR ISR_ENCODER_LEFT() {
    robot.motorLeft->encoderValue++;
}

void IRAM_ATTR ISR_ENCODER_RIGHT() {
    robot.motorRight->encoderValue++;
}

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

    attachInterrupt(digitalPinToInterrupt(motorLeftConfig.encoderPin), ISR_ENCODER_LEFT, RISING);
    attachInterrupt(digitalPinToInterrupt(motorRightConfig.encoderPin), ISR_ENCODER_RIGHT, RISING);

    // i2c reading on core 1 for sensors
    xTaskCreatePinnedToCore(tofUpdateTask, "tof_task", 4096, nullptr, 5, nullptr, 1);

    WifiManager::connect(WIFI_SSID, WIFI_PASS);
    WebServer::start();

    robot.speed(0.3);

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        while (autoMode) {
            robot.rotateLeft();
            robot.start();
            delay(2000);
            robot.stop();
            delay(1000);
            if (!autoMode) break;

            robot.rotateRight();
            robot.start();
            delay(2000);
            robot.stop();
            delay(1000);
        }
    }
}