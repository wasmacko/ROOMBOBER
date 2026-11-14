#include <cstdio>
#include <Arduino.h>
#include "motor.hpp"
#include "robot.hpp"
#include "webserver.hpp"
#include "wifimanager.hpp"

#define WIFI_SSID      "NETIASPOT-H2rM"
#define WIFI_PASS      "pCZk6DAq4VxK"

Robot robot;
MotorConfig motorLeftConfig {
    .speedPin = 4,
    .startPin = 5,
    .dirPin = 6,
    .encoderPin = 7,
};
MotorConfig motorRightConfig {      //those are example pins
    .speedPin = 15,
    .startPin = 16,
    .dirPin = 17,
    .encoderPin = 18,
};

volatile uint8_t autoMode = 1;

void IRAM_ATTR ISR_ENCODER() {
    robot.motorLeft->encoderValue++;
};

extern "C" void app_main(void)
{
    WifiManager::connect(WIFI_SSID, WIFI_PASS);
    WebServer::start();

    initArduino();
    robot.initLeftMotor(motorLeftConfig);
    robot.initRightMotor(motorRightConfig);

    robot.speed(0.3);
    attachInterrupt(digitalPinToInterrupt(motorLeftConfig.encoderPin), ISR_ENCODER, RISING);
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        while (autoMode) {
            robot.rotateLeft();
            robot.start();
            delay(2000);
            robot.stop();
            delay(1000);
            robot.rotateRight();
            robot.start();
            delay(2000);
            robot.stop();
            delay(1000);
        }
    }
};
