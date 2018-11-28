#ifndef OI_H
#define OI_H

#include "Components/XboxController.h"
#include "Components/Joystick2481.h"

class OI {
public:
    OI();
    ~OI();

    static std::unique_ptr<Joystick2481> m_pDriverStick;
    static std::unique_ptr<Joystick2481> m_pOperatorStick;
};

#endif // OI_H
