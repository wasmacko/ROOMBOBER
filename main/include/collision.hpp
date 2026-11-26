#pragma once
#include <Arduino.h>
#include "bumper.hpp"
#include "tof.hpp"

enum class CollisionSide : uint8_t {
    NONE   = 0,
    LEFT   = 1,
    RIGHT  = 2,
    CENTER = 3
};

struct CollisionConfig {
    uint16_t hardStopDist_mm   = 70;    // Crtitical dist ToF [mm]
    uint16_t slowDownDist_mm   = 220;   // Slow down dist ToF [mm]
    uint32_t debounce_ms       = 15;    // bumper debounce [ms]
    uint32_t tofConfirm_ms     = 120;   // Time to confirm collsion stop [ms]

    // Sensor switches for web
    bool tofEnabled[ToFManager::NUM_SENSORS] = {true, true, true};
    bool tofHardStopEnabled = true;     // false = ToF - only avoid /travel around, no returning or stopping
    bool bumperLeftEnabled  = true;
    bool bumperRightEnabled = true;
};

class CollisionManager {
public:
    CollisionManager(Bumper* bumperPtr, ToFManager* tofPtr);

    void update();

    bool isHardCollision() const { return m_currentSide != CollisionSide::NONE; }
    bool isBumperCollision() const { return m_leftBumperActive || m_rightBumperActive; }
    bool isSlowZone() const { return m_slowZoneActive; }
    CollisionSide getCollisionSide() const { return m_currentSide; }

    // Safe read tof (9999 if offline)
    uint16_t getActiveToFDistMM(int idx) const;

    bool isLeftBumperHit() const { return m_leftBumperActive; }
    bool isRightBumperHit() const { return m_rightBumperActive; }

    CollisionConfig config;

private:
    Bumper* m_bumper;
    ToFManager* m_tof;

    uint8_t  m_rawBumperPrev = BUMPER_NONE;
    uint8_t  m_debouncedBumper = BUMPER_NONE;
    uint32_t m_lastBumperChange_ms = 0;

    uint32_t m_tofCritSince_ms[ToFManager::NUM_SENSORS] = {0, 0, 0};

    bool m_leftBumperActive = false;
    bool m_rightBumperActive = false;
    bool m_slowZoneActive = false;
    CollisionSide m_currentSide = CollisionSide::NONE;
};