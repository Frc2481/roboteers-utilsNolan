#ifndef OI_H
#define OI_H

#include "WPILib.h"
#include "Components/XboxController.h"
#include "Components/Joystick2481.h"

class OI {
public:
    OI();
    
    Joystick2481* GetDriverStick();
    Joystick2481* GetOperatorStick();
    
    Joystick2481* m_driverStick;
    Joystick2481* m_operatorStick;
};

#endif // OI_H