#include "robot.hpp"

Robot::Robot() {};
Robot::~Robot() {
    delete motorLeft;
    delete motorRight;
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
    motorLeft->stop();
    motorRight->stop();
}