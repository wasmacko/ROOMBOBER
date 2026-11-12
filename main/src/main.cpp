#include <cstdio>
#include <Arduino.h>
#include "motor.hpp"
#include "robot.hpp"

Robot *robot;
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

void uartTask(void *pvParameters) {
    Serial.begin(115200);
    
    delay(1000); 

    while (true) {
        Serial.print("\033[2J\033[H"); //clear console and set cursor at upper left corner
        Serial.printf("Encoder data: %llu\n", robot->motorLeft->encoderValue);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
};

void IRAM_ATTR ISR_ENCODER() {
    robot->motorLeft->encoderValue++;
};

extern "C" void app_main(void)
{
    initArduino();
    robot = new Robot();
    robot->initLeftMotor(motorLeftConfig);
    robot->initRightMotor(motorRightConfig);

    xTaskCreatePinnedToCore(
        uartTask,
        "USB_Task",
        4096,
        NULL,
        1,
        NULL,
        0
    );
    robot->speed(0.9);
    robot->start();
    attachInterrupt(digitalPinToInterrupt(motorLeftConfig.encoderPin), ISR_ENCODER, RISING);
    while (1) {
        if (robot->motorLeft->encoderValue > 1115) {
            robot->stop();
        }
    };
};
