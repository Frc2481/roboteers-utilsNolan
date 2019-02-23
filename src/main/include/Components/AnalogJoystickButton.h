#ifndef ANALOG_JOYSTICK_BUTTON_H
#define ANALOG_JOYSTICK_BUTTON_H

#include <frc/WPILib.h>

class AnalogJoystickButton : public frc::Button {
public:
    AnalogJoystickButton(frc::GenericHID *joystick, uint32_t axisNumber, double threshold);
    virtual ~AnalogJoystickButton();

    virtual bool Get();

private:
	double m_threshold;
    frc::GenericHID *m_pJoystick;
	uint32_t m_axisNumber;
};

#endif // ANALOG_JOYSTICK_BUTTON_H
