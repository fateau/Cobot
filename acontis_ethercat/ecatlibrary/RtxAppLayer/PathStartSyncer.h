#pragma once
#include "Def.h"
#include "EventHandler.h"

class PathStartSyncer//SyncHMIHandler
{
private:
	bool	_isNeedSync;
	HANDLE	_evt;

public:
	PathStartSyncer(int r) { 
		char str[20];
		sprintf(str, "evtSyncHMIabc%d", r);

		_evt = EventHandler::Create(str);
		EventHandler::Reset(_evt);
	};
	~PathStartSyncer(void){};

	bool getIsNeedSync() {return _isNeedSync;}
	void setIsNeedSync(bool b) {_isNeedSync = b;}

	void waitEvt(){ 
		EventHandler::WaitFor(_evt, INFINITE);
		_isNeedSync = false;
	}
	void setEvt() { 
		EventHandler::Set(_evt);
	}
	void reset() {
		_isNeedSync = false;
		EventHandler::Reset(_evt);
	}
	void release() {
		EventHandler::Set(_evt);
	}
};
