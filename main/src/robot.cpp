#include "robot.hpp"
#include <cmath>

Robot::Robot() {}

Robot::~Robot() {
    delete motorLeft;
    delete motorRight;
    delete tof;
    delete bumper;
}

bool Robot::initToF(const ToFBusConfig &busCfg, const ToFSensorConfig sensorCfgs[ToFManager::NUM_SENSORS]) {
    tof = new ToFManager(busCfg, sensorCfgs);
    return tof->init();
}

void Robot::initLeftMotor(const MotorConfig &config) {
    motorLeft = new Motor(config);
}

void Robot::initRightMotor(const MotorConfig &config) {
    motorRight = new Motor(config);
}

void Robot::initBumper(const BumperConfig &config) {
    bumper = new Bumper(config);
}

// mirroring right motor
void Robot::rotateRight() {
    if (motorLeft) motorLeft->forward();
    if (motorRight) motorRight->forward();
}

void Robot::rotateLeft() {
    if (motorLeft) motorLeft->backward();
    if (motorRight) motorRight->backward();
}

void Robot::forward() {
    if (motorLeft) motorLeft->forward();
    if (motorRight) motorRight->backward();
}

void Robot::backward() {
    if (motorLeft) motorLeft->backward();
    if (motorRight) motorRight->forward();
}

void Robot::drive(double leftSpeed, double rightSpeed) {
    if (leftSpeed > 1.0) leftSpeed = 1.0;
    if (leftSpeed < -1.0) leftSpeed = -1.0;
    if (rightSpeed > 1.0) rightSpeed = 1.0;
    if (rightSpeed < -1.0) rightSpeed = -1.0;

    if (motorLeft) {
        if (leftSpeed > 0.02) {
            motorLeft->forward();
            motorLeft->setSpeed(leftSpeed);
            motorLeft->start();
        } else if (leftSpeed < -0.02) {
            motorLeft->backward();
            motorLeft->setSpeed(-leftSpeed);
            motorLeft->start();
        } else {
            motorLeft->stop();
        }
    }

    // !mirrored right motor
    if (motorRight) {
        if (rightSpeed > 0.02) {
            motorRight->backward();
            motorRight->setSpeed(rightSpeed);
            motorRight->start();
        } else if (rightSpeed < -0.02) {
            motorRight->forward();
            motorRight->setSpeed(-rightSpeed);
            motorRight->start();
        } else {
            motorRight->stop();
        }
    }
}

void Robot::uploadSpeed() {
    if (speed > 1.0) speed = 1.0;
    else if (speed < 0.0) speed = 0.0;
    if (motorLeft) motorLeft->setSpeed(speed);
    if (motorRight) motorRight->setSpeed(speed);
}

void Robot::start() {
    if (motorLeft) motorLeft->start();
    if (motorRight) motorRight->start();
}

void Robot::stop() {
    if (motorLeft) motorLeft->stop();
    if (motorRight) motorRight->stop();
}

uint8_t Robot::stopped() {
    bool lOff = !motorLeft || !motorLeft->isOn();
    bool rOff = !motorRight || !motorRight->isOn();
    return (lOff && rOff) ? 1 : 0;
}

void Robot::turnLeft() {
    if (motorLeft) motorLeft->stop();
    if (motorRight) motorRight->backward();
}

void Robot::turnRight() {
    if (motorLeft) motorLeft->forward();
    if (motorRight) motorRight->stop();
}