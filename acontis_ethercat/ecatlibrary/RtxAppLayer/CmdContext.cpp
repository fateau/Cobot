#include "CmdContext.h"
#include "MathTool_Robot.h"
#include "CmdStrategy.h"
#include "Robot.h"

CmdContext::CmdContext(Robot* robot, IOGroup* ioGroup)
{
	csMove		= new CmdStrategyMove(robot, ioGroup);
	csIO		= new CmdStrategyIO(robot, ioGroup);
	csDelay		= new CmdStrategyDelay(robot);
	csSync		= new CmdStrategySync(robot);
	csHMISync	= new CmdStrategyHMISync(robot);
	csMS		= new CmdStrategyMS(robot);

	csMove->csMS = csMS;
	csMove->csSync = csSync;
}
CmdContext::~CmdContext()
{
	delete csMove;
	delete csIO;
	delete csDelay;
	delete csSync;
	delete csHMISync;
	delete csMS;
}

void CmdContext::selectStrategy(eScriptCmdType::e type)
{	
	switch(type)
	{
		case eScriptCmdType::MOVE:
			cs = csMove;
			break;
		case eScriptCmdType::IO_OUT:
			cs = csIO;
			break;
		case eScriptCmdType::DELAY:
			cs = csDelay;
			break;
		case eScriptCmdType::SYNC:
			cs = csSync;
			break;
		case eScriptCmdType::HMI_SYNC:
			cs = csHMISync;
			break;
		case eScriptCmdType::MS:
			cs = csMS;
			break;
	}
}
int CmdContext::raw2Path(ScriptPathCmd& pCmd, ScriptRawCmd& rCmd)
{
	selectStrategy(rCmd.info.type);	
	return cs->raw2Path(pCmd, rCmd);
}
int CmdContext::processIntpCmd(const IntpCmd& iCmd)
{
	selectStrategy(iCmd.info.type);
	return cs->processIntpCmd(iCmd);
}
void CmdContext::reset()
{
	csMove->reset();
	csDelay->reset();
}