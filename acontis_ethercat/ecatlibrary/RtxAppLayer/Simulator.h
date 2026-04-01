#pragma once

#include <stdio.h>
#include <Windows.h>
#include "Def.h"

#ifdef WIN32_SIMULATE
class Simulator
{

public:
	static void Start(void (CALLBACK *func)(UINT, UINT, DWORD, DWORD, DWORD));
	static void Stop();

private:
	static TIMECAPS tc;
	static MMRESULT timerID;
	
};

#endif