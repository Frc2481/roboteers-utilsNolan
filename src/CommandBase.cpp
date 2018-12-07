#include "CommandBase.h"

std::unique_ptr<OI> CommandBase::m_pOI;
std::unique_ptr<TankDrivetrain> CommandBase::m_pTankDrivetrain;

CommandBase::CommandBase(const std::string &name) : Command(name) {
}

CommandBase::CommandBase() : Command() {
}

void CommandBase::Init() {
	m_pTankDrivetrain.reset(new TankDrivetrain());

	m_pOI.reset(new OI()); // OI must be last subsystem to reset

	Wait(1); // avoid race condition after constructing objects
	CommandBase::m_pTankDrivetrain->zeroDriveEncoders();
	CommandBase::m_pTankDrivetrain->zeroGyroYaw();
}

void CommandBase::Periodic() {
	m_pTankDrivetrain->Periodic();
}
