#include "CommandBase.h"

std::unique_ptr<OI> CommandBase::m_pOI;
std::unique_ptr<TankDrivetrain> CommandBase::m_pTankDrivetrain;

CommandBase::CommandBase(const std::string &name) : Command(name)
{
}

CommandBase::CommandBase() : Command()
{
}

void CommandBase::init()
{    
    // reset OI must be last thing to execute
	m_pOI.reset(new OI());
	m_pTankDrivetrain.reset(new TankDrivetrain());
}
