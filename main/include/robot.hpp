#pragma once
#include "motor.hpp"
#include "tof.hpp"

class Robot {
public:
    Motor* motorLeft = nullptr;
    Motor* motorRight = nullptr;
    ToFManager* tof = nullptr;

    Robot();
    ~Robot();
    // INITIALIZATION:
    void initLeftMotor(const MotorConfig &config);
    void initRightMotor(const MotorConfig &config);
    bool initToF(const ToFBusConfig &busCfg, const ToFSensorConfig sensorCfgs[ToFManager::NUM_SENSORS]);

    // MOVEMENT:
    void rotateRight();
    void rotateLeft();
    void forward();
    void backward();
    void speed(double speedValPercent);
    void start();
    void stop();
    uint8_t stopped();
};