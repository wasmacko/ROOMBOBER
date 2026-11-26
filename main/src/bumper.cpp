#include "bumper.hpp"
#include <Arduino.h>

Bumper::Bumper(const BumperConfig &config)
{
    Bumper::m_leftSwitchPin = config.leftSwitchPin;
    Bumper::m_rightSwitchPin = config.rightSwitchPin;
    pinMode(m_leftSwitchPin, INPUT_PULLUP);
    pinMode(m_rightSwitchPin, INPUT_PULLUP);
};
Bumper::~Bumper() {};
uint8_t Bumper::hit()
{
    auto readLeft = digitalRead(Bumper::m_leftSwitchPin);
    auto readRight = digitalRead(Bumper::m_rightSwitchPin);
    if (readLeft == 1 && readRight == 1)
    {
        return BUMPER_NONE;
    }
    else if (readLeft == 0 && readRight == 0)
    {
        return BUMPER_BOTH;
    }
    else if (readRight == 0)
    {
        return BUMPER_RIGHT;
    }
    else
    {
        return BUMPER_LEFT;
    }
};