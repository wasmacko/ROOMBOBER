#pragma once
#include "motor.hpp"

class Robot {
public:
Motor* motorLeft;
Motor* motorRight;

Robot();
~Robot();
void initLeftMotor(const MotorConfig &config);
void initRightMotor(const MotorConfig &config);
void rotateRight();
void speed(double speedValPercent);
void start();
void stop();
};