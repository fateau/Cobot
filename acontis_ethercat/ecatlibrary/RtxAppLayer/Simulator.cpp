#include "Simulator.h"
#ifndef UNDER_RTSS

TIMECAPS Simulator::tc;
MMRESULT Simulator::timerID;

void Simulator::Start(void (CALLBACK *func)(UINT, UINT, DWORD, DWORD, DWORD))
{
	unsigned int SAMPLING_Time = 1; //ms

	int nRet = timeGetDevCaps( &tc , sizeof(TIMECAPS) );
	if( nRet != TIMERR_NOERROR ) {
		fprintf(stderr, "Err: timeGetDevCaps");
		return;
	}

	nRet = timeBeginPeriod( tc.wPeriodMin );
	if( nRet != TIMERR_NOERROR ) {
		fprintf(stderr, "Err: timeBeginPeriod");
		return;
	}

	timerID = timeSetEvent(
				SAMPLING_Time, 1,
				(LPTIMECALLBACK)func,0,
				TIME_PERIODIC | TIME_CALLBACK_FUNCTION
				);

	if( timerID == (MMRESULT)NULL)
	{
		fprintf(stderr, "Err: timeSetEvent %d\n", timerID);
		return;
	}

}
void Simulator::Stop()
{
	int nRet;

	nRet = timeKillEvent( timerID );
	if(nRet) {
	   fprintf(stderr, "Err: timeKillEvent\n");
	   return;
	}
	
	nRet = timeEndPeriod( tc.wPeriodMin );

	if(nRet != TIMERR_NOERROR) {
		fprintf(stderr, "Err: timeEndPeriod\n");
		return;
	}

}
#endif