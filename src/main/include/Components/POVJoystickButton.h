#ifndef POV_JOYSTICK_BUTTON_H
#define POV_JOYSTICK_BUTTON_H

#include <frc/WPILib.h>

class POVJoystickButton : public frc::Button {
public:
    POVJoystickButton(frc::GenericHID *joystick, uint32_t povNumber, int angle);
    virtual ~POVJoystickButton();

    virtual bool Get();

private:
    frc::GenericHID *m_pJoystick;
	uint32_t m_povNumber;
	int m_angle;
};

#endif // POV_JOYSTICK_BUTTON_H
