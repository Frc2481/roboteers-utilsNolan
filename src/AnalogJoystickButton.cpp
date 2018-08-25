#include "AnalogJoystickButton.h"

AnalogJoystickButton::AnalogJoystickButton(GenericHID *joystick, uint32_t axisNumber, double threshold)
	: m_threshold(threshold),
	m_pJoystick(joystick),
	m_axisNumber(axisNumber) {
	
}

~AnalogJoystickButton() {
	
}

bool AnalogJoystickButton::Get()
{
	if(m_threshold < 0) {
		return m_pJoystick->GetRawAxis(m_axisNumber) < m_threshold;
	}
	else if(m_threshold > 0) {
		return m_pJoystick->GetRawAxis(m_axisNumber) > m_threshold;
	}
	
	return false;
}