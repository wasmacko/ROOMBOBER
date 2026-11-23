#pragma once
#include <stdint.h>

enum BumperHit : uint8_t {
    BUMPER_NONE = 0,
    BUMPER_LEFT,
    BUMPER_RIGHT,
    BUMPER_BOTH,
};

struct BumperConfig {
    uint8_t leftSwitchPin;
    uint8_t rightSwitchPin;
};

class Bumper {
private:
    uint8_t m_leftSwitchPin;
    uint8_t m_rightSwitchPin;
public:
    Bumper(const BumperConfig &config);
    ~Bumper();
    uint8_t hit();
};