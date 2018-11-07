#include "OI.h"

OI::OI() {
    m_pDriverStick = new Joystick2481(0);
    m_pOperatorStick = new Joystick2481(1);
}

~OI:OI() {
    delete m_pDriverStick;
    m_pDriverStick = nullptr;

    delete m_pOperatorStick;
    m_pOperatorStick = nullptr;
}

Joystick2481* OI::GetDriverStick() {
    return m_pDriverStick;
}

Joystick2481* OI::GetOperatorStick() {
    return m_pOperatorStick;
}