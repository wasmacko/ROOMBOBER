#include <cstdio>
#include <Arduino.h>
#include "motor.hpp"
#include "robot.hpp"
#include "webserver.hpp"
#include "wifimanager.hpp"
#include "tof.hpp"
#include "bumper.hpp"
#include "collision.hpp"
#include "controller.hpp"
#include "odometry.hpp"

#define WIFI_SSID "LAPTOP-QP4D10V7 6758"
#define WIFI_PASS "KN14738x"

Robot robot;
CollisionManager* collision = nullptr;
RobotController* controller = nullptr;
Odometry* odometry = nullptr;

MotorConfig motorLeftConfig {
    .speedPin = 42,
    .startPin = 1,
    .dirPin = 2,
    .encoderPin = GPIO_NUM_41,
};

MotorConfig motorRightConfig {
    .speedPin = 5,
    .startPin = 7,
    .dirPin = 6,
    .encoderPin = GPIO_NUM_4,
};

ToFBusConfig tofBusConfig {
    .sdaPin = 8,
    .sclPin = 9,
    .i2cFreqHz = 400000,
    .maxValidDist_mm = 1200,
};

BumperConfig bumperConfig {
    .leftSwitchPin = 40,
    .rightSwitchPin = 15,
};

ToFSensorConfig tofSensorsConfig[ToFManager::NUM_SENSORS] = {
    { .xshutPin = 10, .i2cAddress = 0x30, .offsetX_mm = 80.0f,  .offsetY_mm = 60.0f,  .angle_deg = 45.0f  }, // Left
    { .xshutPin = 11, .i2cAddress = 0x31, .offsetX_mm = 100.0f, .offsetY_mm = 0.0f,   .angle_deg = 0.0f   }, // Center
    { .xshutPin = 12, .i2cAddress = 0x32, .offsetX_mm = 80.0f,  .offsetY_mm = -60.0f, .angle_deg = -45.0f }, // Right
};

// Domyślna konfiguracja odometrii lub przez web
OdometryConfig odomConfig {
    .wheelDiameter_mm = 70.0f,
    .wheelBase_mm     = 230.0f,
    .ticksPerRevLeft  = 600.0f,
    .ticksPerRevRight = 600.0f,
    .invertLeft       = false,
    .invertRight      = true,
};

volatile uint8_t autoMode = 0; // Startujemy bezpiecznie w trybie ręcznym żeby nie uciekał

void tofUpdateTask(void* pvParameters) {
    while (1) {
        if (robot.tof) {
            robot.tof->update();
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz
    }
}

extern "C" void app_main(void)
{
    initArduino();

    robot.initLeftMotor(motorLeftConfig);
    robot.initRightMotor(motorRightConfig);
    robot.initToF(tofBusConfig, tofSensorsConfig);
    robot.initBumper(bumperConfig);

    odometry   = new Odometry(robot.motorLeft, robot.motorRight, odomConfig);
    collision  = new CollisionManager(robot.bumper, robot.tof);
    controller = new RobotController(&robot, collision);

    xTaskCreatePinnedToCore(tofUpdateTask, "tof_task", 4096, nullptr, 5, nullptr, 1);

    WifiManager::connect(WIFI_SSID, WIFI_PASS);
    WebServer::start();

    robot.speed = 0.5;
    robot.uploadSpeed();

    // Pętla główna 100 Hz: Warstwa 2 (Odometria) + Warstwa 1 (Kolizje i ruch)
    while (1) {
        if (odometry)   odometry->update();
        if (controller) controller->update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}