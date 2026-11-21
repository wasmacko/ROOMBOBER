#include "robot.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

Robot::Robot() {};
Robot::~Robot() {
    delete motorLeft;
    delete motorRight;
    delete tof;
};
bool Robot::initToF(const ToFBusConfig &busCfg, const ToFSensorConfig sensorCfgs[ToFManager::NUM_SENSORS]) {
    tof = new ToFManager(busCfg, sensorCfgs);
    return tof->init();
};
void Robot::initLeftMotor(const MotorConfig &config) {
    motorLeft = new Motor(config);
};
void Robot::initRightMotor(const MotorConfig &config) {
    motorRight = new Motor(config);
};
void Robot::rotateRight() {
    motorLeft->forward();
    motorRight->backward();
};
void Robot::rotateLeft() {
    motorLeft->backward();
    motorRight->forward();
};
void Robot::forward() {
    motorLeft->forward();
    motorRight->forward();
};
void Robot::backward() {
    motorLeft->backward();
    motorRight->backward();
};
void Robot::speed(double speedValPercent) {
    motorLeft->setSpeed(speedValPercent);
    motorRight->setSpeed(speedValPercent);
};
void Robot::start() {
    motorLeft->start();
    motorRight->start();
};
void Robot::stop() {
    while(motorLeft->stop() || motorRight->stop()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    };
}