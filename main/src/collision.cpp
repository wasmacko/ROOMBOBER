#include "collision.hpp"

CollisionManager::CollisionManager(Bumper *bumperPtr, ToFManager *tofPtr)
    : m_bumper(bumperPtr), m_tof(tofPtr) {}

uint16_t CollisionManager::getActiveToFDistMM(int idx) const
{
    if (!m_tof || idx < 0 || idx >= ToFManager::NUM_SENSORS)
        return 9999;
    if (!config.tofEnabled[idx] || !m_tof->isOnline(idx))
        return 9999;
    uint16_t d = m_tof->getDistanceMM(idx);
    return (d == 0) ? 9999 : d;
}

void CollisionManager::update()
{
    uint32_t now = millis();

    // 1. Read and debounce bumpers - to check bumpers dont work atm
    uint8_t rawBumper = m_bumper ? m_bumper->hit() : BUMPER_NONE;
    if (rawBumper != m_rawBumperPrev)
    {
        m_lastBumperChange_ms = now;
        m_rawBumperPrev = rawBumper;
    }
    if ((now - m_lastBumperChange_ms) >= config.debounce_ms)
    {
        m_debouncedBumper = rawBumper;
    }

    m_leftBumperActive = config.bumperLeftEnabled &&
                         (m_debouncedBumper == BUMPER_LEFT || m_debouncedBumper == BUMPER_BOTH);
    m_rightBumperActive = config.bumperRightEnabled &&
                          (m_debouncedBumper == BUMPER_RIGHT || m_debouncedBumper == BUMPER_BOTH);

    // 2. read tof sensors with time measurement
    bool tofCrit[ToFManager::NUM_SENSORS] = {false, false, false};
    m_slowZoneActive = false;

    for (int i = 0; i < ToFManager::NUM_SENSORS; i++)
    {
        uint16_t d = getActiveToFDistMM(i);

        if (d < config.slowDownDist_mm)
        {
            m_slowZoneActive = true;
        }

        if (config.tofHardStopEnabled && d < config.hardStopDist_mm)
        {
            if (m_tofCritSince_ms[i] == 0)
            {
                m_tofCritSince_ms[i] = now;
            }
            else if ((now - m_tofCritSince_ms[i]) >= config.tofConfirm_ms)
            {
                tofCrit[i] = true;
            }
        }
        else
        {
            m_tofCritSince_ms[i] = 0;
        }
    }

    // 3. CollisionFusionn
    bool hitLeft = m_leftBumperActive || tofCrit[0];
    bool hitRight = m_rightBumperActive || tofCrit[2];

    if ((hitLeft && hitRight) || tofCrit[1])
    {
        m_currentSide = CollisionSide::CENTER;
    }
    else if (hitLeft)
    {
        m_currentSide = CollisionSide::LEFT;
    }
    else if (hitRight)
    {
        m_currentSide = CollisionSide::RIGHT;
    }
    else
    {
        m_currentSide = CollisionSide::NONE;
    }
}