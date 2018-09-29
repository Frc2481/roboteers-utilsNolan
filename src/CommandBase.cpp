#include "CommandBase.h"
#include "Commands/Scheduler.h"

std::unique_ptr<OI> CommandBase::oi;

CommandBase::CommandBase(const std::string &name) : Command(name)
{
}

CommandBase::CommandBase() : Command()
{
}

void CommandBase::init()
{    
    // reset OI must be last thing to execute
    oi.reset(new OI());
}