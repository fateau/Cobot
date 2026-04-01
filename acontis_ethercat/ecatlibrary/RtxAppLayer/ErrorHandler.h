#pragma once

#include "Def.h"

class ErrorHandler
{
public:

	static void Init();
	static void Set(eError::e type, char* msgFormat, ...);
	static void Set(); //單純觸發人機的ErrorListener
private:
	
	static HANDLE	evtRTXError;
	static char		msg[ERROR_MSG_LEN];

	static void SetInfo(eError::e type);
};