#include "controller.hpp"
#include <cstdio>
#include <cmath>
#include <algorithm>

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
    uint16_t dLeft = m_collision->getActiveToFDistMM(0);
    uint16_t dRight = m_collision->getActiveToFDistMM(2);

    if (strategy == AvoidStrategy::SMART_OPEN_SPACE || strategy == AvoidStrategy::SMOOTH_ARC) {
        // choosing side with more space
        if (dLeft != dRight) {
            m_turnRightNext = (dRight > dLeft);
            return;
        }
    }
    // turn away from colision
    m_turnRightNext = (m_lastHitSide == CollisionSide::LEFT);
}

void RobotController::driveAuto() {
    uint16_t dLeft = m_collision->getActiveToFDistMM(0);
    uint16_t dCenter = m_collision->getActiveToFDistMM(1);
    uint16_t dRight = m_collision->getActiveToFDistMM(2);
    uint8_t dBumper = m_robot->bumper->hit();

    uint16_t slowDist = m_collision->config.slowDownDist_mm;
    uint16_t hardDist = m_collision->config.hardStopDist_mm;

    int64_t actualLeftEncoderValue = m_robot->motorLeft->getSignedEncoderValue();
    int64_t actualRightEncoderValue = m_robot->motorLeft->getSignedEncoderValue();
    if (slowDist <= hardDist + 20) slowDist = hardDist + 20;

    switch(m_makingAction){
        case 1: //turn left
        if(actualRightEncoderValue-m_valueBuffer > 200){
            m_robot->forward();
            if(actualRightEncoderValue-m_valueBuffer > 600){
                m_makingAction = 0;
            }
        }
        return;
        
        case 2: //turn right
        if(actualLeftEncoderValue-m_valueBuffer > 200){
            m_robot->forward();
            if(actualLeftEncoderValue-m_valueBuffer > 600){
                m_makingAction = 0;
            }
        }
        return;

        case 3: //rotate left
        if(actualLeftEncoderValue-m_valueBuffer > 1000){
            m_robot->forward();
            m_makingAction = 0;
        }
        return;

        case 4: //rotate right
        if(actualLeftEncoderValue-m_valueBuffer > 1000){
            m_robot->forward();
            m_makingAction = 0;
        }
        return;

        case 5: //backward to left
        if(actualRightEncoderValue-m_valueBuffer > 500){
            m_robot->turnLeft();
            if(actualRightEncoderValue-m_valueBuffer > 900){
                m_makingAction = 0;
            }
        }
        return;

        case 6: //backward to right
        if(actualLeftEncoderValue-m_valueBuffer > 500){
            m_robot->turnRight();
            if(actualLeftEncoderValue-m_valueBuffer > 900){
                m_makingAction = 0;
            }
        }
        return;
    }

    //TODO: slow down on turning, specify basic speed - need to discuss about values
    if(dBumper == BUMPER_NONE && dCenter > hardDist && !m_makingAction){
        //NO OBSTACLE, NO MANOUVERS - CAN DRIVE
        if(dRight > hardDist+10 && dLeft > hardDist+10){
            //BOTH WALLS MISSING - TURNING LEFT BY DEFAULT
        }else if(dRight > hardDist+10){
            //RIGHT WALL MISSING - NEED TO STABILIZE PATH
            //TURNING RIGHT FOR A LITTLE
            m_valueBuffer = actualLeftEncoderValue;
            m_robot->turnRight();
            m_makingAction = 2;
        }else if(dLeft > hardDist+10){
            //LEFT WALL MISSING - NEED TO STABILIZE PATH
            //TURNING LEFT FOR A LITTLE
            m_valueBuffer = actualRightEncoderValue;
            m_robot->turnLeft();
            m_makingAction = 1;
        }else if(dRight < hardDist){
            //TOO CLOSE RIGHT WALL - NEED TO STABILIZE PATH
            //TURNING LEFT FOR A LITTLE
            m_valueBuffer = actualRightEncoderValue;
            m_robot->turnLeft();
            m_makingAction = 1;
        }else if(dLeft < hardDist){
            //TOO CLOSE LEFT WALL - NEED TO STABILIZE PATH
            //TURNING RIGHT FOR A LITTLE
            m_valueBuffer = actualLeftEncoderValue;
            m_robot->turnRight();
            m_makingAction = 2;
        }else{
            //NO PROBLEMS
            m_robot->forward();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }else if((dBumper !=BUMPER_NONE || dCenter < hardDist+5) && !m_makingAction){
        //THERE IS OBSTACLE
        switch(dBumper){
            case BUMPER_LEFT:
            //OBSTACLE ON LEFT, NEED TO REVERSE, AND TURN RIGHT
            m_valueBuffer = actualLeftEncoderValue;
            m_robot->backward();
            m_makingAction = 6;
            return;

            case BUMPER_RIGHT:
            //OBSTACLE ON RIGHT, NEED TO REVERSE, AND TURN LEFT
            m_valueBuffer = actualRightEncoderValue;
            m_robot->backward();
            m_makingAction = 5;
            return;
            
            case BUMPER_BOTH:
            //OBSTACLE IN FRONT, NEED TO REVERSE, AND TURN SOMEWHERE
            m_valueBuffer = actualRightEncoderValue;
            m_robot->backward();
            m_makingAction = 5;
            return;

        }
        if(dRight > hardDist+10){
            //RIGHT WALL MISSING - CAN TURN RIGHT
            //ROTATE RIGHT
            m_valueBuffer = actualLeftEncoderValue;
            m_robot->rotateRight();
            m_makingAction = 4;
        }else if(dLeft > hardDist+10){
            //LEFT WALL MISSING - CAN TURN LEFT
            //ROTATE LEFT
            m_valueBuffer = actualRightEncoderValue;
            m_robot->rotateLeft();
            m_makingAction = 3;
        }
    }
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
            driveAuto();
            return;

        case RobotState::BLOCKED_STOP:
            m_robot->stop();
            if (!m_collision->isHardCollision()) {
                m_state = RobotState::AUTO_DRIVE;
            }
            return;

        case RobotState::AVOID_REVERSE:
            m_robot->drive(-reverseSpeed, -reverseSpeed);
            if (now - m_stateTimer_ms >= reverseDuration_ms) {
                m_state = RobotState::AVOID_TURN;
                m_stateTimer_ms = now;
            }
            return;

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
            return;

        default:
            m_robot->stop();
            return;
    }
}