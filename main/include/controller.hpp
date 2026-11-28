#pragma once
#include <Arduino.h>
#include "robot.hpp"
#include "collision.hpp"
#include "webserver.hpp"

enum class RobotState : uint8_t {
    IDLE          = 0,
    MANUAL        = 1,
    AUTO_DRIVE    = 2,
    AVOID_REVERSE = 3,
    AVOID_TURN    = 4,
    BLOCKED_STOP  = 5
};

enum class AvoidStrategy : uint8_t {
    STOP_ONLY        = 0, // Stop in front of object
    CLASSIC_REV_TURN = 1, // Stop -> return -> rotate from obejct
    SMART_OPEN_SPACE = 2, // Rotate to space with more space
    SMOOTH_ARC       = 3  // Smooth travel around object
};

class RobotController {
public:
    RobotController(Robot* robotPtr, CollisionManager* collisionPtr);

    void update();

    RobotState getState() const { return m_state; }
    const char* getStateName() const;
    const char* getStrategyName() const;

    // Konfigurowalne z kodu i z WWW
    AvoidStrategy strategy      = AvoidStrategy::SMOOTH_ARC;
    double   cruiseSpeed        = 0.55;
    double   slowSpeed          = 0.35;
    double   reverseSpeed       = 0.45;
    double   turnSpeed          = 0.45;
    uint32_t reverseDuration_ms = 350;
    uint32_t turnDuration_ms    = 450;

private:
    void driveAuto();
    void decideTurnDirection();

    Robot* m_robot;
    CollisionManager* m_collision;

    RobotState m_state = RobotState::IDLE;
    uint32_t m_stateTimer_ms = 0;
    CollisionSide m_lastHitSide = CollisionSide::NONE;
    bool m_turnRightNext = true;
    bool m_manualColLatched = false;
    int64_t m_valueBuffer;  //buffer for data 
    uint8_t m_makingAction = 0; //0-no action, 1-turn left, 2-turn right,3-rotate left, 4-rotate right, 5-backward to left, 6-backward to right
};