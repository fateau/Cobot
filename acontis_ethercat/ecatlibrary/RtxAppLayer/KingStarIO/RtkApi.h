///////////////////////////////////////////////////////////////////////////////
//
//
//			Copyright (c) 2013 - 2018 IntervalZero, Inc.  All rights reserved.
//
//
//File: RtKApi.h
//
//Abstract:
//		    Real-time Kernel API data types and function prototypes for 
//			windows kernel drivers to call RTX64/RTSS IPC functions
//
//
//Revision History: 5/13/2013 Initial Revision
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "Licensing.h"

#define LPSECURITY_ATTRIBUTES PVOID

//
// RtssInstance - returned from call to RtkRtssAttach/RtkRtssAttachEx and required for all
// calls to RTK functions
//
#define RTSSINST  ULONG_PTR
#define PRTSSINST PULONG_PTR

//
// Before any of the calls listed below, the driver writer is required to call 
// RtkRtssAttach(Ex) to obtain the Rtss Instance. All RTKAPI calls require the 
// instance identifier.
//

///////////////////////////////////////////////////////////////////////////////
//
//RTK Load/UnLoad functions.
//
///////////////////////////////////////////////////////////////////////////////
NTSTATUS 
RtkRtssAttachEx( 
                       	 ULONG MaxWFSO,  // Requires the maximum WaitForSingleObjects value, must be > 0
						 PRTSSINST RtssInstPtr,
                       	 VOID (*Routine)(PVOID context, LONG reason),
						 PVOID Context );

NTSTATUS 
RtkRtssAttach( 
                       	 ULONG MaxWFSO, 
						 PRTSSINST RtssInstPtr );

NTSTATUS
RtkRtssDetach( 
                       	 RTSSINST RtssInst );

///////////////////////////////////////////////////////////////////////////////
//
//Wait functions.
//
///////////////////////////////////////////////////////////////////////////////

#define MAX_WFMO 16 //Maximum number of objects per wait for multiple objects.

NTSTATUS
RtkWaitForSingleObject(
                       	  RTSSINST RtssInst,
                       	  HANDLE   Handle,
                       	  ULONG    Milliseconds);

NTSTATUS
RtkWaitForMultipleObjects( 
					 	  RTSSINST     RtssInst,
					 	  ULONG        Count,
                          CONST HANDLE *Handles,
					      BOOLEAN	   WaitAll,
                       	  ULONG	       Milliseconds);

///////////////////////////////////////////////////////////////////////////////
//
//Mutex object functions.
//
///////////////////////////////////////////////////////////////////////////////
NTSTATUS
RtkCreateMutex(
						  RTSSINST			    RtssInst,
						  PHANDLE				MutexHandle,
						  LPSECURITY_ATTRIBUTES Security,
                       	  BOOLEAN				InitialOwner,
                       	  PUNICODE_STRING		Name);

NTSTATUS
RtkOpenMutex(
                       	  RTSSINST	      RtssInst,
						  PHANDLE	      MutexHandle,
						  ULONG           DesiredAccess,
						  BOOLEAN         InheritHandle,
						  PUNICODE_STRING Name);

NTSTATUS
RtkReleaseMutex(
                       	  RTSSINST RtssInst,
                       	  HANDLE   Mutex);

///////////////////////////////////////////////////////////////////////////////
//
//Semaphore object functions.
//
///////////////////////////////////////////////////////////////////////////////
NTSTATUS
RtkCreateSemaphore(
						  RTSSINST              RtssInst,
				   		  PHANDLE               SemaphoreHandle,
						  LPSECURITY_ATTRIBUTES Security,
						  LONG	                InitialCount,
						  LONG	                MaximumCount,
						  PUNICODE_STRING	    Name);

NTSTATUS
RtkOpenSemaphore(
                          RTSSINST		  RtssInst,
						  PHANDLE		  SemaphoreHandle,
						  ULONG		      DesiredAccess,
						  BOOLEAN		  InheritHandle,
                       	  PUNICODE_STRING Name);

NTSTATUS
RtkReleaseSemaphore(
                       	  RTSSINST RtssInst,
                          HANDLE   SemaphoreHandle,
                          LONG	   ReleaseCount,
						  PLONG    PreviousCount);

///////////////////////////////////////////////////////////////////////////////
//
//Event object functions.
//
///////////////////////////////////////////////////////////////////////////////
NTSTATUS
RtkCreateEvent(
                          RTSSINST              RtssInst,
						  PHANDLE		        EventHandle,
						  LPSECURITY_ATTRIBUTES Security,
						  BOOLEAN               ManualReset,
                       	  BOOLEAN               InitialState,
                          PUNICODE_STRING       Name);

NTSTATUS
RtkOpenEvent(
                       	  RTSSINST        RtssInst,
						  PHANDLE	      EventHandle,
						  ULONG           DesiredAccess,
						  BOOLEAN         InheritHandle,
						  PUNICODE_STRING Name);

NTSTATUS
RtkPulseEvent(
                       	  RTSSINST RtssInst,
                       	  HANDLE   EventHandle);

NTSTATUS
RtkResetEvent(
                       	  RTSSINST   RtssInst,
                       	  HANDLE	 EventHandle);

NTSTATUS
RtkSetEvent(
                       	  RTSSINST RtssInst,
                       	  HANDLE   EventHandle);

///////////////////////////////////////////////////////////////////////////////
//
//Shared Memory functions.
//
///////////////////////////////////////////////////////////////////////////////

//
//Shared memory read and write desired access defines
//
#define SHM_MAP_READ 1
#define SHM_MAP_WRITE 2

NTSTATUS
RtkCreateSharedMemory(
						  RTSSINST        RtssInst,
						  PHANDLE	      SharedMemoryHandle,
						  ULONG           Protect,
                       	  ULONG           MaximumSizeHigh,
                       	  ULONG           MaximumSizeLow,
                       	  PUNICODE_STRING Name,
                       	  VOID **         Location);

NTSTATUS
RtkOpenSharedMemory(
                       	  RTSSINST        RtssInst,
						  PHANDLE	      SharedMemoryHandle,
                       	  ULONG           DesiredAccess,
						  BOOLEAN         InheritHandle,
						  PUNICODE_STRING Name,
                       	  VOID **         Location);


///////////////////////////////////////////////////////////////////////////////
//
//Close object function.
//
///////////////////////////////////////////////////////////////////////////////
NTSTATUS
RtkCloseHandle(
                       	  RTSSINST RtssInst,
                       	  HANDLE   Object);


///////////////////////////////////////////////////////////////////////////////
//
//Proxy Thread Priority Function.
//
///////////////////////////////////////////////////////////////////////////////

NTSTATUS 
RtkGetProxyThreadPriority(
						  RTSSINST		RtssInst, 
						  HANDLE		hThread, 
						  int			*RtssPriority);

NTSTATUS 
RtkSetProxyThreadPriority(
						  RTSSINST		RtssInst, 
						  HANDLE		hThread, 
						  int			RtssPriority);



// This function retrieves information about all licenses available on the system
NTSTATUS RtkGetLicenses(
	PRT_LICENSE_INFO_W pResult,
	PULONG pLength );

// This function verifies whether the specified version of the RTX TCP-IP stack is installed and has
// a valid license.
BOOLEAN RtkIsTcpStackLicensed(
	unsigned int majorVersion );

// This function verifies whether the specified version of the RTX runtime is installed and has a
// valid license.
BOOLEAN RtkIsRuntimeLicensed(
	unsigned int majorVersion );

// This function verifies whether the specified version of the RTX TCP-IP stack is installed and has
// a valid license.
NTSTATUS RtkIsTcpStackLicensedEx(
	BOOLEAN * pResult,
	unsigned int majorVersion);

// This function verifies whether the specified version of the RTX runtime is installed and has a
// valid license.
NTSTATUS RtkIsRuntimeLicensedEx(
	BOOLEAN * pResult,
	unsigned int majorVersion);

// This function returns whether the specified RTSS application binary can run.  This means it has
// been built to run with the provided license feature and that there is a valid license for the
// feature on the system.
NTSTATUS  RtkIsAppRunnable(
	BOOLEAN * pResult,
	PUNICODE_STRING applicationPath,
	PUNICODE_STRING featureName,
	unsigned int majorVersion);

// Retrieves runtime version.
NTSTATUS RtkGetRuntimeVersionEx(
	PRT_VERSION_INFO versionInfo);

#ifdef __cplusplus
}
#endif


