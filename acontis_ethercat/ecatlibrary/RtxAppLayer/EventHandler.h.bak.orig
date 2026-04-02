
#include "Def.h"

class EventHandler
{
public:
	
	// if you have build error, USE RTSS-RELEASE, not RTSS-DEBUG !!!
	static HANDLE Open(char* strName)
	{
		#if defined UNDER_RTSS
		return RtOpenEventA(0, 0, strName);
		#else
		return OpenEvent(EVENT_ALL_ACCESS, 0, strName);
		#endif
	}
	static HANDLE Create(char* strName)
	{
		#if defined UNDER_RTSS
		return RtCreateEventA(0, 0, 0, strName);
		#else
	    return CreateEvent(0, 0, 0, strName);
		#endif
	}
	static void Set(HANDLE evt)
	{
		#if defined UNDER_RTSS
		RtSetEvent(evt);
		#else
		SetEvent(evt);
		#endif
	}
	static void Reset(HANDLE evt)
	{
		#if defined UNDER_RTSS
		RtResetEvent(evt);	
		#else
		ResetEvent(evt);
		#endif
	}
	static DWORD WaitFor(HANDLE evt, DWORD dwMilliSeconds)
	{
		#if defined UNDER_RTSS
		return RtWaitForSingleObject(evt, dwMilliSeconds);
		#else
		return WaitForSingleObject(evt, dwMilliSeconds);
		#endif
	}
	static void Close(HANDLE thread)
	{
		#if defined UNDER_RTSS
		RtCloseHandle(thread);
		#else
		CloseHandle(thread);
		#endif
	}
};
class ThreadHandler
{
public:
	

	static HANDLE Create(
    _In_opt_ LPSECURITY_ATTRIBUTES lpThreadAttributes,
    _In_ SIZE_T dwStackSize,
    _In_ LPTHREAD_START_ROUTINE lpStartAddress,
    _In_opt_ __drv_aliasesMem LPVOID lpParameter,
    _In_ DWORD dwCreationFlags,
    _Out_opt_ LPDWORD lpThreadId
    )
	{
		#if defined UNDER_RTSS //KING_STAR_IO && defined _AMD64_
		return RtCreateThread(lpThreadAttributes, dwStackSize, lpStartAddress, lpParameter, dwCreationFlags, lpThreadId);		
		#else
		return CreateThread(lpThreadAttributes, dwStackSize, lpStartAddress, lpParameter, dwCreationFlags, lpThreadId);
		#endif
	}

	static void SetPriority(HANDLE thread, int priority)
	{
		#if defined UNDER_RTSS //KING_STAR_IO && defined _AMD64_
		RtSetThreadPriority(thread, priority);
		#else
		SetThreadPriority(thread, priority);
		#endif
	}

	static void Suspend(HANDLE thread)
	{
		#if defined UNDER_RTSS
		RtSuspendThread(thread);
		#else
		SuspendThread(thread);
		#endif
	}
	static void Resume(HANDLE thread)
	{
		#if defined UNDER_RTSS
		RtResumeThread(thread);
		#else
		ResumeThread(thread);
		#endif
	}

};
