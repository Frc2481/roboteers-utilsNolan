#ifndef COMBO_JOYSTICK_BUTTON_H
#define COMBO_JOYSTICK_BUTTON_H

#include <WPILib.h>

class ComboJoystickButton : public Button {
public:
    ComboJoystickButton(Button *primaryButton, Button *secondaryButton, bool secondaryPressed);
    virtual ~ComboJoystickButton();

    virtual bool Get();

private:
    Button *m_pPrimaryButton;
	Button *m_pSecondaryButton;
	bool m_secondaryPressed;
};

#endif // COMBO_JOYSTICK_BUTTON_H
