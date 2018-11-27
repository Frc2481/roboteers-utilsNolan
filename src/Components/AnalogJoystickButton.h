#ifndef ANALOG_JOYSTICK_BUTTON_H
#define ANALOG_JOYSTICK_BUTTON_H

#include <WPILib.h>

class AnalogJoystickButton : public Button {
public:
    AnalogJoystickButton(GenericHID *joystick, uint32_t axisNumber, const double &threshold);
    virtual ~AnalogJoystickButton();

    virtual bool Get();

private:
	double m_threshold;
    GenericHID *m_pJoystick;
	uint32_t m_axisNumber;
};

#endif // ANALOG_JOYSTICK_BUTTON_H
