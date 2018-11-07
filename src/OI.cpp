#include "OI.h"

OI::OI() {
    m_driverStick = new Joystick2481(0);
    m_operatorStick = new Joystick2481(1);
}

~OI:OI() {
    delete m_driverStick;
    m_driverStick = nullptr;

    delete m_operatorStick;
    m_operatorStick = nullptr;
}

Joystick2481* OI::GetDriverStick() {
    return m_driverStick;
}

Joystick2481* OI::GetOperatorStick() {
    return m_operatorStick;
}