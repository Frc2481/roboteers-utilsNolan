#include "OI.h"
#include "RobotMap.h"

std::unique_ptr<Joystick2481> OI::m_pDriverStick = std::make_unique<Joystick2481>(DRIVER_XBOX_CONTROLLER_ID);
std::unique_ptr<Joystick2481> OI::m_pOperatorStick = std::make_unique<Joystick2481>(OPERATOR_XBOX_CONTROLLER_ID);

OI::OI() {
}

OI::~OI() {
}
