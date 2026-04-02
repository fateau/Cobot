#include "ErrorHandler.h"
#include "Shm.h"
#include "Def.h"
#include "Debug.h"
#include "EventHandler.h"

//<目的> <2.0> ErrorHandler.m4a
//<使用> <2.0> ErrorHandler.m4a
//<設計> <2.0> ErrorHandler.m4a
extern SHMData* shm;

HANDLE	ErrorHandler::evtRTXError = EventHandler::Open("evtRTXError");
char	ErrorHandler::msg[ERROR_MSG_LEN];

void ErrorHandler::Init()
{
	evtRTXError = EventHandler::Open("evtRTXError");

	EventHandler::Reset(evtRTXError );
}
void ErrorHandler::Set()
{
	EventHandler::Set(evtRTXError);
}
void ErrorHandler::Set(eError::e type, char* msgFormat, ...)
{
	// 劇本急停
	shm->isSlowStop = false;
	shm->stopScript = true;

	// 處理訊息字串
	va_list args;
	va_start( args, msgFormat );
	vsprintf( msg, msgFormat, args);	
	va_end	( args );
		
		
	SetInfo(type);

	EventHandler::Set(evtRTXError);
}
void ErrorHandler::SetInfo(eError::e type)
{
	shm->errorCode = type;

	strncpy(shm->errorMsg, msg, ERROR_MSG_LEN);	//訊息寫入Shm讓人機拿

}