#include "PathLoader.h"
#include "Script.h"

void PathLoader::init(Script* script)
{
	this->script = script;
	isHoldPath = false;
}

void PathLoader::reset()
{
	isHoldPath = false;
}

int PathLoader::load()
{
	if(isHoldPath == true) return 1;
	
	if(script->ppQueue.isEmpty()) {

		//如果RawQ已結束，宣告ppQ結束後返回
		if(script->isRawQueueTerminate == true) 
			script->isPPQueueTerminate = true;

		return -1;
	} 
			
	script->ppQueue.dequeue(&pCmd);	
	
	isHoldPath = true;
	return 1;
}

ScriptPathCmd PathLoader::takePCmd()
{
	isHoldPath = false;
	return pCmd;
}
eScriptCmdType::e PathLoader::pCmdType()
{
	return pCmd.info.type;
}