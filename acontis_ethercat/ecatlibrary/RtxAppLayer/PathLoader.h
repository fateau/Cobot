#pragma once
#include "Def.h"
//#include "Script.h"

class Script;
class PathLoader
{
public:
	PathLoader(){};
	~PathLoader(){};

	void init(Script* script);
	void reset();
	int  load();
	ScriptPathCmd takePCmd();
	eScriptCmdType::e pCmdType();

private:
	Script*			script;
	ScriptPathCmd	pCmd;

	bool isHoldPath;
};

