#include "odometry.hpp"
#include <Arduino.h>
#include <cmath>

static constexpr float PI_F = 3.14159265358979323846f;

Odometry::Odometry(Motor* leftMotor, Motor* rightMotor, const OdometryConfig &cfg)
    : m_left(leftMotor), m_right(rightMotor), m_cfg(cfg) {
    reset(false);
}

float Odometry::normalizeAngle(float rad) {
    while (rad > PI_F)  rad -= 2.0f * PI_F;
    while (rad < -PI_F) rad += 2.0f * PI_F;
    return rad;
}

float Odometry::getThetaDeg() const {
    return m_pose.theta_rad * (180.0f / PI_F);
}

void Odometry::reset(bool alsoResetEncoders) {
    if (alsoResetEncoders) {
        if (m_left)  m_left->resetEncoder();
        if (m_right) m_right->resetEncoder();
    }
    m_prevLeftTicks  = m_left  ? m_left->getSignedEncoderValue()  : 0;
    m_prevRightTicks = m_right ? m_right->getSignedEncoderValue() : 0;
    m_prevUpdate_ms  = millis();
    m_pose = RobotPose{};
}

void Odometry::update() {
    if (!m_left || !m_right) return;
    if (m_cfg.ticksPerRevLeft <= 0.1f || m_cfg.ticksPerRevRight <= 0.1f || m_cfg.wheelBase_mm <= 1.0f) return;

    uint32_t now = millis();
    float dt_s = (now - m_prevUpdate_ms) / 1000.0f;
    if (dt_s <= 0.0f) return;
    m_prevUpdate_ms = now;

    int64_t curL = m_left->getSignedEncoderValue();
    int64_t curR = m_right->getSignedEncoderValue();

    int64_t dTicksL = curL - m_prevLeftTicks;
    int64_t dTicksR = curR - m_prevRightTicks;
    m_prevLeftTicks  = curL;
    m_prevRightTicks = curR;

    if (m_cfg.invertLeft)  dTicksL = -dTicksL;
    if (m_cfg.invertRight) dTicksR = -dTicksR;

    const float circumference = PI_F * m_cfg.wheelDiameter_mm;
    const float dsL = ((float)dTicksL / m_cfg.ticksPerRevLeft)  * circumference;
    const float dsR = ((float)dTicksR / m_cfg.ticksPerRevRight) * circumference;

    const float ds     = 0.5f * (dsL + dsR);
    const float dTheta = (dsR - dsL) / m_cfg.wheelBase_mm;

    // Całkowanie metodą punktu środkowego dokładniejsze na łukach
    const float midTheta = m_pose.theta_rad + 0.5f * dTheta;
    m_pose.x_mm         += ds * std::cos(midTheta);
    m_pose.y_mm         += ds * std::sin(midTheta);
    m_pose.theta_rad     = normalizeAngle(m_pose.theta_rad + dTheta);
    m_pose.totalDist_mm += std::fabs(ds);

    m_pose.linearVel_mms   = ds / dt_s;
    m_pose.angularVel_degs = (dTheta * (180.0f / PI_F)) / dt_s;
}