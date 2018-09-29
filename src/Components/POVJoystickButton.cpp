#include "POVJoystickButton.h"

POVJoystickButton::POVJoystickButton(GenericHID *joystick, uint32_t povNumber, int angle)
	: m_pJoystick(joystick),
	m_povNumber(povNumber), {
}

POVJoystickButton::~POVJoystickButton() {
}

bool POVJoystickButton::Get()
{
	return m_pJoystick->GetPOV(m_povNumber) == m_angle;
}