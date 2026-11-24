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
void Robot::initBumper(const BumperConfig &config) {
    bumper = new Bumper(config);
};

//strange directions of morots because in case motors are mirrored.
void Robot::rotateRight() {
    motorLeft->forward();
    motorRight->forward();
};
void Robot::rotateLeft() {
    motorLeft->backward();
    motorRight->backward();
};
void Robot::forward() {
    motorLeft->forward();
    motorRight->backward();
};
void Robot::backward() {
    motorLeft->backward();
    motorRight->forward();
};
void Robot::uploadSpeed() {
    if (speed > 1) {
        speed = 1;
    } else if (speed < 0) {
        speed = 0;
    }
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
};
void Robot::start() {
    motorLeft->start();
    motorRight->start();
};
void Robot::stop() {
    motorLeft->stop();
    motorRight->stop();
}