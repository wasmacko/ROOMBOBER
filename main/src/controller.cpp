#include "controller.hpp"
#include <cstdio>
#include <cmath>

RobotController::RobotController(Robot* robotPtr, CollisionManager* collisionPtr)
    : m_robot(robotPtr), m_collision(collisionPtr) {}

const char* RobotController::getStateName() const {
    switch (m_state) {
        case RobotState::IDLE:          return "IDLE";
        case RobotState::MANUAL:        return "MANUAL";
        case RobotState::AUTO_DRIVE:    return "AUTO_DRIVE";
        case RobotState::AVOID_REVERSE: return "AVOID_REVERSE";
        case RobotState::AVOID_TURN:    return "AVOID_TURN";
        case RobotState::BLOCKED_STOP:  return "BLOCKED_STOP";
        default:                        return "UNKNOWN";
    }
}

const char* RobotController::getStrategyName() const {
    switch (strategy) {
        case AvoidStrategy::STOP_ONLY:        return "STOP_ONLY";
        case AvoidStrategy::CLASSIC_REV_TURN: return "CLASSIC_REV_TURN";
        case AvoidStrategy::SMART_OPEN_SPACE: return "SMART_OPEN_SPACE";
        case AvoidStrategy::SMOOTH_ARC:       return "SMOOTH_ARC";
        default:                              return "UNKNOWN";
    }
}

void RobotController::decideTurnDirection() {
    uint16_t dL = m_collision->getActiveToFDistMM(0);
    uint16_t dR = m_collision->getActiveToFDistMM(2);

    if (strategy == AvoidStrategy::SMART_OPEN_SPACE || strategy == AvoidStrategy::SMOOTH_ARC) {
        // choosing side with more space
        if (dL != dR) {
            m_turnRightNext = (dR > dL);
            return;
        }
    }
    // turn away from colision
    m_turnRightNext = (m_lastHitSide == CollisionSide::LEFT);
}

void RobotController::driveAutoSmooth() {
    uint16_t dL = m_collision->getActiveToFDistMM(0);
    uint16_t dC = m_collision->getActiveToFDistMM(1);
    uint16_t dR = m_collision->getActiveToFDistMM(2);

    uint16_t slowDist = m_collision->config.slowDownDist_mm;
    uint16_t hardDist = m_collision->config.hardStopDist_mm;
    if (slowDist <= hardDist + 20) slowDist = hardDist + 20;

    // linear slow down
    uint16_t minD = dC;
    if (dL < minD) minD = dL;
    if (dR < minD) minD = dR;

    double baseSpd = cruiseSpeed;
    if (minD < slowDist) {
        double t = (double)(minD - hardDist) / (double)(slowDist - hardDist);
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        baseSpd = slowSpeed + t * (cruiseSpeed - slowSpeed);
    }

    if (strategy != AvoidStrategy::SMOOTH_ARC) {
        m_robot->drive(baseSpd, baseSpd);
        return;
    }

    // SMOOTH_ARC: proportional deviation of the course along an arc away from approaching walls
    double steer = 0.0; // > 0 turn right (slowdown right wheel), < 0 turn left
    const double arcRange = (double)slowDist * 1.4;

    if (dL < arcRange) {
        steer += (arcRange - dL) / arcRange;
    }
    if (dR < arcRange) {
        steer -= (arcRange - dR) / arcRange;
    }
    // IF front sensor detects obstacle, turn lightly to free space
    if (dC < slowDist) {
        double frontPush = ((double)(slowDist - dC) / (double)slowDist) * 0.85;
        steer += (dR >= dL) ? frontPush : -frontPush;
    }

    if (steer > 1.0)  steer = 1.0;
    if (steer < -1.0) steer = -1.0;

    double leftSpd  = baseSpd;
    double rightSpd = baseSpd;
    if (steer > 0.0) {
        rightSpd = baseSpd * (1.0 - 0.85 * steer);
    } else if (steer < 0.0) {
        leftSpd  = baseSpd * (1.0 - 0.85 * (-steer));
    }

    m_robot->drive(leftSpd, rightSpd);
}

void RobotController::update() {
    if (!m_robot || !m_collision) return;

    m_collision->update();
    uint32_t now = millis();

    //manual mode
    if (!autoMode) {
        m_state = RobotState::MANUAL;
        if (m_collision->isHardCollision()) {
            if (!m_manualColLatched) {
                m_robot->stop();
                m_manualColLatched = true;
                printf("[MANUAL] Bezpieczny STOP przed kolizją!\n");
            }
        } else {
            m_manualColLatched = false;
        }
        return;
    }
    m_manualColLatched = false;

    //  return from MANUAL / IDLE to AUTO_DRIVE
    if (m_state == RobotState::MANUAL || m_state == RobotState::IDLE) {
        m_state = RobotState::AUTO_DRIVE;
    }

    // hard collison in auto
    if (m_collision->isHardCollision() &&
        m_state != RobotState::AVOID_REVERSE &&
        m_state != RobotState::AVOID_TURN)
    {
        m_robot->stop();
        m_lastHitSide   = m_collision->getCollisionSide();
        m_stateTimer_ms = now;
        decideTurnDirection();

        if (strategy == AvoidStrategy::STOP_ONLY) {
            m_state = RobotState::BLOCKED_STOP;
            return;
        }

        // SMART_OPEN_SPACE return when collide on front or bumper
        bool needReverse = m_collision->isBumperCollision() ||
                           (strategy == AvoidStrategy::CLASSIC_REV_TURN) ||
                           (m_lastHitSide == CollisionSide::CENTER);

        m_state = needReverse ? RobotState::AVOID_REVERSE : RobotState::AVOID_TURN;
        printf("[AUTO:%s] Kolizja(%u) -> %s (obrót: %s)\n",
               getStrategyName(), (unsigned)m_lastHitSide, getStateName(),
               m_turnRightNext ? "PRAWO" : "LEWO");
        return;
    }

    // State machine for auto mode
    switch (m_state) {
        case RobotState::AUTO_DRIVE:
            driveAutoSmooth();
            break;

        case RobotState::BLOCKED_STOP:
            m_robot->stop();
            if (!m_collision->isHardCollision()) {
                m_state = RobotState::AUTO_DRIVE;
            }
            break;

        case RobotState::AVOID_REVERSE:
            m_robot->drive(-reverseSpeed, -reverseSpeed);
            if (now - m_stateTimer_ms >= reverseDuration_ms) {
                m_state = RobotState::AVOID_TURN;
                m_stateTimer_ms = now;
            }
            break;

        case RobotState::AVOID_TURN:
            if (m_turnRightNext) {
                m_robot->drive(turnSpeed, -turnSpeed);
            } else {
                m_robot->drive(-turnSpeed, turnSpeed);
            }

            if (now - m_stateTimer_ms >= turnDuration_ms) {
                m_robot->stop();
                m_state = RobotState::AUTO_DRIVE;
            }
            break;

        default:
            m_robot->stop();
            break;
    }
}