#include "Components/POVJoystickButton.h"

POVJoystickButton::POVJoystickButton(frc::GenericHID *joystick, uint32_t povNumber, int angle)
	: m_pJoystick(joystick),
	m_povNumber(povNumber),
	m_angle(0) {
}

POVJoystickButton::~POVJoystickButton() {
}

bool POVJoystickButton::Get()
{
	return m_pJoystick->GetPOV(m_povNumber) == m_angle;
}
