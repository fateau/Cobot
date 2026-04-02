#pragma once

/*---------------------------------------------------------------------------
 * ErrorHandler.h - Linux-compatible error handler
 *---------------------------------------------------------------------------*/

#include "Def.h"

class ErrorHandler
{
public:
    static void Init();
    static void Set(eError::e type, const char* msgFormat, ...);
    static void Set();

private:
    static void* evtRTXError;
    static char  msg[ERROR_MSG_LEN];
    static void  SetInfo(eError::e type);
};
