#include "OI.h"

OI::OI() {
    m_driverStick = new Joystick2481(0);
    m_operatorStick = new Joystick2481(1);
}

Joystick2481* OI::GetDriverStick() {
    return m_driverStick;
}

Joystick2481* OI::GetOperatorStick() {
    return m_operatorStick;
}