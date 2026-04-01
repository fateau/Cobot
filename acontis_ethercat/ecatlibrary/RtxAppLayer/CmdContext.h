#pragma once

 
#include "Def.h"

class Robot;
class IOGroup;

class CmdStrategy;
class CmdStrategyMove;
class CmdStrategyIO;	
class CmdStrategyDelay;
class CmdStrategySync;
class CmdStrategyHMISync;
class CmdStrategyMS;

class CmdContext
{
public:
	CmdContext(Robot* robot, IOGroup* ioGroup);
	~CmdContext(void);

	int raw2Path(ScriptPathCmd& pCmd, ScriptRawCmd& rCmd);
	int processIntpCmd(const IntpCmd& iCmd);

	void reset();
	
private:
	CmdStrategy*		cs;
	CmdStrategyMove*	csMove;
	CmdStrategyIO*		csIO;
	CmdStrategyDelay*	csDelay;
	CmdStrategySync*	csSync;
	CmdStrategyHMISync* csHMISync; 
	CmdStrategyMS*		csMS;

	void selectStrategy(eScriptCmdType::e type);
};
