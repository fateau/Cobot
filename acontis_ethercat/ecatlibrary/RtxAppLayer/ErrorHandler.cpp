/*---------------------------------------------------------------------------
 * ErrorHandler.cpp - Linux-compatible error handler implementation
 *---------------------------------------------------------------------------*/
#include "ErrorHandler.h"
#include "Shm.h"
#include "Def.h"
#include "Debug.h"
#include "EventHandler.h"
#include <cstdio>
#include <cstdarg>
#include <cstring>

extern SHMData* shm;

void*   ErrorHandler::evtRTXError = nullptr;
char    ErrorHandler::msg[ERROR_MSG_LEN];

void ErrorHandler::Init()
{
    evtRTXError = EventHandler::Open("evtRTXError");
    EventHandler::Reset(evtRTXError);
}
void ErrorHandler::Set()
{
    EventHandler::Set(evtRTXError);
}
void ErrorHandler::Set(eError::e type, const char* msgFormat, ...)
{
    // Immediate stop
    shm->isSlowStop = false;
    shm->stopScript = true;

    va_list args;
    va_start(args, msgFormat);
    vsnprintf(msg, ERROR_MSG_LEN, msgFormat, args);
    va_end(args);

    SetInfo(type);
    EventHandler::Set(evtRTXError);
}
void ErrorHandler::SetInfo(eError::e type)
{
    shm->errorCode = type;
    strncpy(shm->errorMsg, msg, ERROR_MSG_LEN);
    shm->errorMsg[ERROR_MSG_LEN - 1] = '\0';
}
