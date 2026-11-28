#pragma once
#include "motor.hpp"
#include "tof.hpp"
#include "bumper.hpp"

class Robot {
public:
    Motor* motorLeft = nullptr;
    Motor* motorRight = nullptr;
    ToFManager* tof = nullptr;
    Bumper* bumper = nullptr;
    double speed = 0.0;

    Robot();
    ~Robot();

    // INITIALIZATION:
    void initLeftMotor(const MotorConfig &config);
    void initRightMotor(const MotorConfig &config);
    bool initToF(const ToFBusConfig &busCfg, const ToFSensorConfig sensorCfgs[ToFManager::NUM_SENSORS]);
    void initBumper(const BumperConfig &config);

    // MOVEMENT:
    void rotateRight();
    void rotateLeft();
    void turnRight();
    void turnLeft();
    void forward();
    void backward();
    void drive(double leftSpeed, double rightSpeed); // [-1.0 .. 1.0], + = przód robota, - = tył robota
    void uploadSpeed();
    void start();
    void stop();
    uint8_t stopped();
};