#include <cstdio>
#include <Arduino.h>
#include "lib/motor.hpp"

const int speedPin = 4;
const int startPin = 5;
const int dirPin = 6;
const int encPin = 7;


Motor motor(speedPin,startPin,dirPin,encPin);

void uartTask(void *pvParameters) {
    Serial.begin(115200);
    
    delay(1000); 

    while (true) {
        Serial.print("\033[2J\033[H"); //clear console and set cursor at upper left corner
        Serial.printf("Encoder data: %llu\n", motor.encoderValue);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
};

void IRAM_ATTR ISR_ENCODER() {
    motor.encoderValue++;
};

extern "C" void app_main(void)
{
    initArduino();

    pinMode(startPin, OUTPUT);
    digitalWrite(startPin, HIGH);
    pinMode(dirPin, OUTPUT);
    digitalWrite(dirPin, LOW);
    pinMode(encPin, INPUT_PULLUP);
    
    xTaskCreatePinnedToCore(
        uartTask,
        "USB_Task",
        4096,
        NULL,
        1,
        NULL,
        0 // Rdzeń 0
    );
    motor.setSpeed(0.9);
    motor.start();
    attachInterrupt(digitalPinToInterrupt(encPin), ISR_ENCODER, RISING);
    while (1) {
        if (motor.encoderValue > 1115) {
            motor.stop();
        }
    };
};
