#pragma once
#include <stdint.h>
#include "motor.hpp"

struct OdometryConfig {
    float wheelDiameter_mm = 70.0f;   // wheel diameter
    float wheelBase_mm     = 230.0f;  // wheelbase
    float ticksPerRevLeft  = 600.0f;  // impulse per rotate left
    float ticksPerRevRight = 600.0f;  // impulse per rotate right
    bool  invertLeft       = false;
    bool  invertRight      = true;
};

struct RobotPose {
    float x_mm            = 0.0f; // Forward (+)
    float y_mm            = 0.0f; // Left (+)
    float theta_rad       = 0.0f; // angle in radians [-PI, PI], left (+)
    float totalDist_mm    = 0.0f;
    float linearVel_mms   = 0.0f;
    float angularVel_degs = 0.0f;
};

class Odometry {
private:
    Motor* m_left;
    Motor* m_right;
    OdometryConfig m_cfg;
    RobotPose m_pose;

    int64_t m_prevLeftTicks  = 0;
    int64_t m_prevRightTicks = 0;
    uint32_t m_prevUpdate_ms = 0;

    static float normalizeAngle(float rad);

public:
    Odometry(Motor* leftMotor, Motor* rightMotor, const OdometryConfig &cfg = {});

    void update();
    void reset(bool alsoResetEncoders = true);

    RobotPose getPose() const { return m_pose; }
    float getThetaDeg() const;
    OdometryConfig getConfig() const { return m_cfg; }
    void setConfig(const OdometryConfig &cfg) { m_cfg = cfg; }
};