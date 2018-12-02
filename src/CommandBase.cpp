#include "CommandBase.h"

std::unique_ptr<OI> CommandBase::m_pOI;
std::unique_ptr<TankDrivetrain> CommandBase::m_pTankDrivetrain;

CommandBase::CommandBase(const std::string &name) : Command(name) {
}

CommandBase::CommandBase() : Command() {
}

void CommandBase::Init() {
	m_pTankDrivetrain.reset(new TankDrivetrain());

    // reset OI must be last thing to execute
	m_pOI.reset(new OI());
}

void CommandBase::Periodic() {
	m_pTankDrivetrain->Periodic();
}
