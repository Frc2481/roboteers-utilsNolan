#ifndef JOYSTICK_2481_H
#define JOYSTICK_2481_H

#include <frc/WPILib.h>

class Joystick2481 : public frc::Joystick {
public:
    Joystick2481(int port);
	virtual ~Joystick2481();

	virtual float GetRawAxis(int axis);

private:

};

#endif // JOYSTICK_2481_H
