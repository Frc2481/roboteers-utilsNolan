#ifndef OI_H
#define OI_H

#include "Components/XboxController.h"
#include "Components/Joystick2481.h"

class OI {
public:
    OI();
    ~OI();

    Joystick2481* GetDriverStick();
    Joystick2481* GetOperatorStick();

    Joystick2481 *m_pDriverStick;
    Joystick2481 *m_pOperatorStick;
};

#endif // OI_H
