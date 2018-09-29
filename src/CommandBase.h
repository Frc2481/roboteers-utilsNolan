#ifndef COMMAND_BASE_H
#define COMMAND_BASE_H

#include "WPILib.h"
#include "Commands/Command.h"
#include "Commands/Scheduler.h"
#include "OI.h"

class CommandBase: public Command
{
public:
    CommandBase(const std::string &name);
    CommandBase();

    static void init();
	
    static std::unique_ptr<OI> oi;
};

#endif // COMMAND_BASE_H
