#ifndef COMMANDS_TANK_DRIVETRAIN_STOP_H
#define COMMANDS_TANK_DRIVETRAIN_STOP_H

#include <vector>
#include <limits>
#include "../CommandBase.h"

class TankDrivetrainStop : public CommandBase {
public:
	TankDrivetrainStop()
		: CommandBase("TankDrivetrainStop") {
		
		Requires(CommandBase::m_pTankDrivetrain.get());
		SetInterruptible(false);
	}

	~TankDrivetrainStop() {
	}

	void Initialize() {
		m_pTankDrivetrain->stop();
	}

	void Execute() {
	}

	void Interrupted() {
		End();
	}

	bool IsFinished() {
		return true;
	}

	void End() {
	}

private:

};

#endif // COMMANDS_TANK_DRIVETRAIN_STOP_H
