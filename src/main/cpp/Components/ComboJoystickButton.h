#ifndef COMBO_JOYSTICK_BUTTON_H
#define COMBO_JOYSTICK_BUTTON_H

#include <frc/WPILib.h>

class ComboJoystickButton : public frc::Button {
public:
    ComboJoystickButton(frc::Button *primaryButton, frc::Button *secondaryButton, bool secondaryPressed);
    virtual ~ComboJoystickButton();

    virtual bool Get();

private:
    frc::Button *m_pPrimaryButton;
	frc::Button *m_pSecondaryButton;
	bool m_secondaryPressed;
};

#endif // COMBO_JOYSTICK_BUTTON_H
