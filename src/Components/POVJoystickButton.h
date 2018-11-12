#ifndef POV_JOYSTICK_BUTTON_H
#define POV_JOYSTICK_BUTTON_H

#include <WPILib.h>

class POVJoystickButton : public Button {
public:
    POVJoystickButton(GenericHID *joystick, uint32_t povNumber, int angle);
    virtual ~POVJoystickButton();

    virtual bool Get();

private:
    GenericHID *m_pJoystick;
	uint32_t m_povNumber;
	int m_angle;
};

#endif // POV_JOYSTICK_BUTTON_H
