#pragma once
#include "motor.hpp"

class Robot {
public:
Motor* motorLeft;
Motor* motorRight;

Robot();
~Robot();
//INITIALIZATION:
void initLeftMotor(const MotorConfig &config);
void initRightMotor(const MotorConfig &config);
//MOVEMENT:
void rotateRight();
void rotateLeft();
void forward();
void backward();
void speed(double speedValPercent);
void start();
void stop();
};