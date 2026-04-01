///////////////////////////////////////////////////////////////////////////////
//
//
//			Copyright (c) 2011 - 2018 IntervalZero, Inc.  All rights reserved.
//
//
//File: RtApi.h
//
//Abstract:
//
//		This file defines the functions which only RtApi applications can use.
//	RtApi applications use a subset of the RTSS API.
//
///////////////////////////////////////////////////////////////////////////////
#pragma once

#ifndef _RTAPI_H_
#define _RTAPI_H_

#include "Licensing.h"
#include "RTX64Version.h"

#ifdef __cplusplus
extern "C" {
#endif

#if !defined ( _AMD64_ )
	#define RTAPI __stdcall
	#define RTFCNDCL __stdcall
#else
	#define RTAPI
	#define RTFCNDCL
#endif

//
// Max length used for names by RtCreate[Objtype], RtOpen[Objtype]
//
#define RTX_MAX_OBJECT_NAME_LENGTH	256

//
// Define maximum number of RTSS processors
//
#if (!defined(MAXNUM_RTPROCESSORS))
#define MAXNUM_RTPROCESSORS 0x3F
#endif

//
// Define maximum number of RTSS+Windows processors
//
#if (!defined(MAXNUM_TOTALPROCESSORS))
#define MAXNUM_TOTALPROCESSORS 0x40
#endif

//
//Define RealTime functions in terms of Windows functions.
//
#define	RtGetLastError		GetLastError
#define	RtSetLastError		SetLastError
#define RtCreateThread		CreateThread
#define	RtExitThread		ExitThread
#define	RtGetCurrentThread	GetCurrentThread
#define	RtResumeThread		ResumeThread
#define	RtSuspendThread		SuspendThread
#define	RtTerminateThread	TerminateThread
#define	RtExitProcess		ExitProcess
#define	RtGetStdHandle		GetStdHandle
#define	RtSleep				Sleep
#define	RtSleepEx			SleepEx
#ifdef UNICODE
#define	RtWriteConsole		WriteConsoleW
#define	RtGetCommandLine	GetCommandLineW
#else
#define	RtWriteConsole		WriteConsoleA
#define	RtGetCommandLine	GetCommandLineA
#endif // !UNICODE


///////////////////////////////////////////////////////////////////////////////
//
//Contiguous memory functions.
//
///////////////////////////////////////////////////////////////////////////////

#if (!defined(_NTDDK_)&& !defined(__RTX_H__))
typedef enum _MEMORY_CACHING_TYPE
{  
	MmNonCached,  
	MmCached,  
	MmWriteCombined
} MEMORY_CACHING_TYPE;
#endif
// This function allocates physically contiguous memory.
PVOID RTAPI RtAllocateContiguousMemory(ULONG Length, LARGE_INTEGER PhysAddr);
// This routine allocates a range of physically contiguous, cache-aligned memory.
PVOID RTAPI RtAllocateContiguousMemorySpecifyCache(ULONG  Length, LARGE_INTEGER  LowestPhysAddr, LARGE_INTEGER  HighestPhysAddr, LARGE_INTEGER  BoundaryPhysAddrMultiple, MEMORY_CACHING_TYPE  CacheType);
// This function frees a previously allocated physically contiguous memory region.
BOOL RTAPI RtFreeContiguousMemory(PVOID pVirtualAddr);
// This function returns the physical address for the virtual address of a contiguous physical memory buffer 
// previously allocated by RtAllocateContiguousMemory or RtAllocateContiguousMemorySpecifyCache.
// In the RTSS environment, this function applies to all memory allocation.
// In the Win32 environment, it applies to contiguous memory allocation only.
LARGE_INTEGER RTAPI RtGetPhysicalAddress(PVOID pVirtualAddr);

///////////////////////////////////////////////////////////////////////////////
//
//Interrupt functions.
//
///////////////////////////////////////////////////////////////////////////////

#if ( !defined(_NTDDK_) && !defined(_NTHAL_H_) && !defined(__RTX_H__) )
//
// Define the I/O bus interface types.
//
typedef enum _INTERFACE_TYPE {
    InterfaceTypeUndefined = -1,
    Internal,
    Isa,
    Eisa,
    MicroChannel,
    TurboChannel,
    PCIBus,
    VMEBus,
    NuBus,
    PCMCIABus,
    CBus,
    MPIBus,
    MPSABus,
    ProcessorInternal,
    InternalPowerBus,
    PNPISABus,
    MaximumInterfaceType
}INTERFACE_TYPE, *PINTERFACE_TYPE;


typedef enum _KINTERRUPT_MODE
{
    LevelSensitive,
    Latched
}KINTERRUPT_MODE;

#endif


#if ( !defined(_NTDDK_) )
//
// Define types of bus information.
//
typedef enum _BUS_DATA_TYPE
{
    ConfigurationSpaceUndefined = -1,
    Cmos,
    EisaConfiguration,
    Pos,
    CbusConfiguration,
    PCIConfiguration,
    VMEConfiguration,
    NuBusConfiguration,
    PCMCIAConfiguration,
    MPIConfiguration,
    MPSAConfiguration,
    PNPISAConfiguration,
    MaximumBusDataType
}BUS_DATA_TYPE, *PBUS_DATA_TYPE;

#pragma warning (push)
#pragma warning( disable : 4214 )
#pragma warning( disable : 4201 )
//
//Define the format of the PCI Slot parameter.
//
typedef struct _PCI_SLOT_NUMBER {
    union {
        struct {
            ULONG   DeviceNumber:5;
            ULONG   FunctionNumber:3;
            ULONG   Reserved:24;
        } bits;
        ULONG   AsULONG;
    } u;
} PCI_SLOT_NUMBER, *PPCI_SLOT_NUMBER;
#pragma warning (pop)

#define PCI_TYPE0_ADDRESSES             6
#define PCI_TYPE1_ADDRESSES             2


//
// Define the standard PCI configuration information.
//
typedef struct _PCI_COMMON_CONFIG {
    USHORT  VendorID;                   // (ro)
    USHORT  DeviceID;                   // (ro)
    USHORT  Command;                    // Device control
    USHORT  Status;
    UCHAR   RevisionID;                 // (ro)
    UCHAR   ProgIf;                     // (ro)
    UCHAR   SubClass;                   // (ro)
    UCHAR   BaseClass;                  // (ro)
    UCHAR   CacheLineSize;              // (ro+)
    UCHAR   LatencyTimer;               // (ro+)
    UCHAR   HeaderType;                 // (ro)
    UCHAR   BIST;                       // Built in self test

    union {
        struct _PCI_HEADER_TYPE_0 {
            ULONG   BaseAddresses[PCI_TYPE0_ADDRESSES];
            ULONG   CIS;
            USHORT  SubVendorID;
            USHORT  SubSystemID;
            ULONG   ROMBaseAddress;
            ULONG   Reserved2[2];

            UCHAR   InterruptLine;      //
            UCHAR   InterruptPin;       // (ro)
            UCHAR   MinimumGrant;       // (ro)
            UCHAR   MaximumLatency;     // (ro)
        } type0;


    } u;

    UCHAR   DeviceSpecific[192];

} PCI_COMMON_CONFIG, *PPCI_COMMON_CONFIG;


#define PCI_COMMON_HDR_LENGTH (FIELD_OFFSET (PCI_COMMON_CONFIG, DeviceSpecific))

#define PCI_MAX_DEVICES                     32
#define PCI_MAX_FUNCTION                    8

#define PCI_INVALID_VENDORID                0xFFFF


//
// Bit encodings for  PCI_COMMON_CONFIG.HeaderType
//
#define PCI_MULTIFUNCTION                   0x80
#define PCI_DEVICE_TYPE                     0x00
#define PCI_BRIDGE_TYPE                     0x01


//
// Bit encodings for PCI_COMMON_CONFIG.Command
//
#define PCI_ENABLE_IO_SPACE                 0x0001
#define PCI_ENABLE_MEMORY_SPACE             0x0002
#define PCI_ENABLE_BUS_MASTER               0x0004
#define PCI_ENABLE_SPECIAL_CYCLES           0x0008
#define PCI_ENABLE_WRITE_AND_INVALIDATE     0x0010
#define PCI_ENABLE_VGA_COMPATIBLE_PALETTE   0x0020
#define PCI_ENABLE_PARITY                   0x0040  // (ro+)
#define PCI_ENABLE_WAIT_CYCLE               0x0080  // (ro+)
#define PCI_ENABLE_SERR                     0x0100  // (ro+)
#define PCI_ENABLE_FAST_BACK_TO_BACK        0x0200  // (ro)


//
// Bit encodings for PCI_COMMON_CONFIG.Status
//
#define PCI_STATUS_FAST_BACK_TO_BACK        0x0080  // (ro)
#define PCI_STATUS_DATA_PARITY_DETECTED     0x0100
#define PCI_STATUS_DEVSEL                   0x0600  // 2 bits wide
#define PCI_STATUS_SIGNALED_TARGET_ABORT    0x0800
#define PCI_STATUS_RECEIVED_TARGET_ABORT    0x1000
#define PCI_STATUS_RECEIVED_MASTER_ABORT    0x2000
#define PCI_STATUS_SIGNALED_SYSTEM_ERROR    0x4000
#define PCI_STATUS_DETECTED_PARITY_ERROR    0x8000


//
// Bit encodes for PCI_COMMON_CONFIG.u.type0.BaseAddresses
//
#define PCI_ADDRESS_IO_SPACE                0x00000001  // (ro)
#define PCI_ADDRESS_MEMORY_TYPE_MASK        0x00000006  // (ro)
#define PCI_ADDRESS_MEMORY_PREFETCHABLE     0x00000008  // (ro)

#define PCI_TYPE_32BIT      0
#define PCI_TYPE_20BIT      2
#define PCI_TYPE_64BIT      4


//
// Bit encodes for PCI_COMMON_CONFIG.u.type0.ROMBaseAddresses
//
#define PCI_ROMADDRESS_ENABLED              0x00000001


//
// Reference notes for PCI configuration fields:
//
// ro   these field are read only.  changes to these fields are ignored
//
// ro+  these field are intended to be read only and should be initialized
//      by the system to their proper values.  However, driver may change
//      these settings.
//
#endif // _NTDDK_

typedef struct _ISA_PNP_LOGICAL_DEVICE_INFO
{
	DWORD LogicalDeviceID;
	DWORD CompatibleIDs[8];
} ISA_PNP_LOGICAL_DEVICE_INFO, *PISA_PNP_LOGICAL_DEVICE_INFO;

typedef struct _ISA_PNP_CONFIG
{
	DWORD VendorID;
	DWORD SerialNumber;
	DWORD NumberLogicalDevices;
	ISA_PNP_LOGICAL_DEVICE_INFO LogicalDeviceInfo[8];
} ISA_PNP_CONFIG, *PISA_PNP_CONFIG;

typedef struct _ISA_PNP_RESOURCES
{
	USHORT Flags;
	struct
	{
		USHORT MemoryBase;
		USHORT MemoryUpperLimit;
		UCHAR MemoryControl;
	} Memory24Descriptors[4];
	struct
	{
		DWORD MemoryBase;
		DWORD MemoryUpperLimit;
		UCHAR MemoryControl;
	} Memory32Descriptors[4];
	USHORT IoPortDescriptors[8];
	struct
	{
		UCHAR IRQLevel;
		UCHAR IRQType;
	} IRQDescriptors[2];
	UCHAR DMADescriptors[2];
} ISA_PNP_RESOURCES, *PISA_PNP_RESOURCES;

#define ISA_PNP_RESOURCE_FLAG_ACTIVE 0x00000001


#if ( !defined(__RTX_H__) )

typedef enum _INTERRUPT_DISPOSITION
{
    PassToNextDevice,
    Dismiss,
    CallInterruptThread
}INTERRUPT_DISPOSITION, *PINTERRUPT_DISPOSITION;

#endif

//
// Line based and message based interrupt attachment
// parameter structure definitions.
//
#pragma pack(push, 4)
typedef struct _ATTACH_INTERRUPT_LINE_BASED_PARAMETERS {
    PSECURITY_ATTRIBUTES pThreadAttributes;
    ULONG                StackSize;
    BOOLEAN              (RTFCNDCL * pRoutine)(PVOID context);
    PVOID                Context;
    ULONG                Priority;
    INTERFACE_TYPE       InterfaceType;
    ULONG                BusNumber;
    PCI_SLOT_NUMBER      SlotNumber;
    ULONG                BusInterruptLevel;
    ULONG                BusInterruptVector;
    BOOLEAN              Shared;
    KINTERRUPT_MODE      InterruptMode;
    INTERRUPT_DISPOSITION    (RTFCNDCL * MyInterrupt)(PVOID context);
    KAFFINITY            ProcessorEnableMask;
} ATTACH_INTERRUPT_LINE_BASED_PARAMETERS,
  *PATTACH_INTERRUPT_LINE_BASED_PARAMETERS;

typedef struct _ATTACH_INTERRUPT_MESSAGE_BASED_PARAMETERS {
    PSECURITY_ATTRIBUTES pThreadAttributes;
    ULONG                StackSize;
    BOOLEAN              (RTFCNDCL * pRoutine)(PVOID context);
    PVOID                Context;
    ULONG                Priority;
    ULONG                BusNumber;
    PCI_SLOT_NUMBER      SlotNumber;
    INTERRUPT_DISPOSITION    (RTFCNDCL * MyInterrupt)(PVOID context);
    KAFFINITY            ProcessorEnableMask;
} ATTACH_INTERRUPT_MESSAGE_BASED_PARAMETERS,
  *PATTACH_INTERRUPT_MESSAGE_BASED_PARAMETERS;

#if ( !defined(_INTERRUPT_MESSAGE_INFO_ENTRY_) )
#define _INTERRUPT_MESSAGE_INFO_ENTRY_
typedef struct _INTERRUPT_MESSAGE_INFO_ENTRY {
    ULONG                MessageId;        // Message Id for message-signaled interupt (IN)
    ULONG                TargetProcessor;  // Target processor to receive interrupt (IN)
    ULONG                Priority;         // Priority of interrupt handling thread (IN)
    ULONG                Vector;           // Interrupt vector on target processor (OUT)
} INTERRUPT_MESSAGE_INFO_ENTRY, *PINTERRUPT_MESSAGE_INFO_ENTRY;
#endif

typedef struct _INTERRUPT_MESSAGE_INFO {
    ULONG                        MessageCount;   // The number of message-signaled interrupts requested for allocating
    INTERRUPT_MESSAGE_INFO_ENTRY MessageInfo[1]; // Specify target and priority for each message-signaled interrupt
} INTERRUPT_MESSAGE_INFO, *PINTERRUPT_MESSAGE_INFO;

typedef struct _ATTACH_INTERRUPT_MESSAGE_BASED_MVECTOR_PARAMETERS {
    PSECURITY_ATTRIBUTES    pThreadAttributes;
    ULONG                   StackSize;
    BOOLEAN                 (RTFCNDCL * pRoutine)(PVOID context, ULONG MessageId);
    PVOID                   Context;
    ULONG                   BusNumber;
    PCI_SLOT_NUMBER         SlotNumber;
    INTERRUPT_DISPOSITION   (RTFCNDCL * MyInterrupt)(PVOID context, ULONG MessageId);
    VOID                    (RTFCNDCL * EnableInterruptFromSpecifiedMessage)(PVOID context, ULONG MessageId, BOOLEAN bEnable);
    PINTERRUPT_MESSAGE_INFO InterruptMessageTable; // Specify info of message-signaled interrupts for attaching
} ATTACH_INTERRUPT_MESSAGE_BASED_MVECTOR_PARAMETERS,
*PATTACH_INTERRUPT_MESSAGE_BASED_MVECTOR_PARAMETERS;
#pragma pack(pop)

//
// Top level interrupt attachment parameter definitions.
//
#define ATTACH_LINE_BASED                     0x1
#define ATTACH_MESSAGE_BASED                  0x2
#define ATTACH_MESSAGE_BASED_PREFER_MSI       0x3
#define ATTACH_MESSAGE_BASED_MULTI_VECTOR     0x4

//
// MACRO to get failed MessageId for not enough free vectors in ATTACH_MESSAGE_BASED_MULTI_VECTOR
//
#define RT_ERROR_NOT_ENOUGH_FREE_VECTOR       ((RT_CUSTOM_ERROR(0)) | (ERROR_IRQ_BUSY << 16))
#define NOT_ENOUGH_FREE_VECTOR(LastError)     ((LastError & 0xFFFF0000) == RT_ERROR_NOT_ENOUGH_FREE_VECTOR)
#define GET_FAILED_MESSAGE_ID(LastError)      (LastError & 0x0000FFFF)

//
// MACRO to get invalid MessageId for invalid message parameters in ATTACH_MESSAGE_BASED_MULTI_VECTOR
//
#define RT_ERROR_INVALID_MESSAGE_PARAMETER    ((RT_CUSTOM_ERROR(0)) | (ERROR_INVALID_PARAMETER << 16))
#define INVALID_MESSAGE_PARAMETER(LastError)  ((LastError & 0xFFFF0000) == RT_ERROR_INVALID_MESSAGE_PARAMETER)
#define GET_INVALID_MESSAGE_ID(LastError)     (LastError & 0x0000FFFF)


#pragma warning (push)
#pragma warning( disable : 4201 )
// RtAttachInterrupt can be used in two different ways: for line - based interrupts or message - based interrupts.
// Each interrupt type requires filling in a different portion of the ATTACH_INTERRUPT_PARAMETERS structure.
// The caller identifies which set of input arguments it is providing (and, hence, which portion of the structure
// has been filled in) by setting the AttachVersion field of the ATTACH_INTERRUPT_PARAMETERS structure to a 
// particular value.
typedef struct _ATTACH_INTERRUPT_PARAMETERS {
    ULONG AttachVersion;
    union {
        ATTACH_INTERRUPT_LINE_BASED_PARAMETERS LineBased;
        ATTACH_INTERRUPT_MESSAGE_BASED_PARAMETERS MessageBased;
        ATTACH_INTERRUPT_MESSAGE_BASED_MVECTOR_PARAMETERS MessageBasedMvector;
    };
} ATTACH_INTERRUPT_PARAMETERS, *PATTACH_INTERRUPT_PARAMETERS;
#pragma warning (pop)
// This function allows the user to associate an Interrupt Service Thread (IST) and Interrupt Service
// Routine(ISR) with a line - based or message - based hardware interrupt.
HANDLE	RTAPI RtAttachInterrupt(PATTACH_INTERRUPT_PARAMETERS pParameters);
// This function releases an interrupt previously attached using RtAttachInterrupt.
// This breaks the association between a user's interrupt handling routine 
// and the hardware interrupt.
// Note that this function is only supported in the RTSS environment. It cannot be called from a 
// Windows application linked to RTX64.
BOOL	RTAPI RtReleaseInterrupt(HANDLE hInterrupt);
// This function obtains details, starting at the given offset, about a given slot on an I/O bus.
// Note that RtGetBusDataByOffset is only supported in the RTSS environment. It cannot be called from 
// a Windows application linked to RTX64.
ULONG	RTAPI RtGetBusDataByOffset(BUS_DATA_TYPE BusDataType, ULONG BusNumber, ULONG SlotNumber, PVOID pBuffer, ULONG Offset, ULONG Length);
// This function sets bus-configuration data for a device on a dynamically configurable I/O bus with a published,
// standard interface. 
// Note that RtSetBusDataByOffset is only supported in the RTSS environment.It cannot be called from a Windows application
// linked to RTX64.
ULONG	RTAPI RtSetBusDataByOffset(BUS_DATA_TYPE BusDataType, ULONG BusNumber, ULONG SlotNumber, PVOID pBuffer, ULONG Offset, ULONG Length);
// This function translates a bus - specific address into the corresponding system logical address.
// Note that RtTranslateBusAddress is only supported in the RTSS environment.It cannot be called from a Windows application
// linked to RTX64.
BOOL	RTAPI RtTranslateBusAddress(INTERFACE_TYPE InterfaceType, ULONG BusNumber, LARGE_INTEGER BusAddress, PULONG pAddressSpace, PLARGE_INTEGER pTranslatedAddress);

#ifdef UNDER_RTSS
	VOID _disable(VOID);
	VOID _enable(VOID);
	#pragma intrinsic (_disable)
	#pragma intrinsic (_enable)
	#define RtEnableInterrupts()     _enable()
	#define RtDisableInterrupts()    (_disable(), TRUE)
#else
	//This function enables user-level interrupt handling for all interrupts to which the process is attached.
	VOID	RTAPI RtEnableInterrupts(VOID);
	//In the Win32 environment, RtDisableInterrupts disables all user-level interrupt handling for all interrupts 
	//to which the Win32 process is attached.
	//In an RTSS environment, RtDisableInterrupts disables all interrupts at the processor level including timer 
	//interrupts.
	BOOL	RTAPI RtDisableInterrupts(VOID);
#endif

///////////////////////////////////////////////////////////////////////////////
//
//Local memory functions.
//
///////////////////////////////////////////////////////////////////////////////

// This function allocates memory from a pre-allocated RTSS local memory pool to avoid SRI activity if allocating 
// memory from the Windows memory pool.
PVOID	RTAPI RtAllocateLocalMemory(ULONG Size);
// This function  frees memory previously allocated by a call to RtAllocateLocalMemory or RtAllocateLocalMemoryEx.
BOOL	RTAPI RtFreeLocalMemory(PVOID pVirtualAddress);
// This function returns RTSS local memory pool information, including the pool size as a whole, the free memory 
// size available, and the maximum virtually contiguous free memory size.
// Note that RtQueryLocalMemory is only supported in the RTSS environment. It cannot be called from a Windows application 
// linked to RTX64.
BOOL	RTAPI RtQueryLocalMemory(PULONG MemSize, PULONG MemAvail, PULONG MemContig);


///////////////////////////////////////////////////////////////////////////////
//
//Physical memory mapping functions.
//
///////////////////////////////////////////////////////////////////////////////

//This function maps a range of physical memory addresses into the user's virtual address space.
PVOID	RTAPI RtMapMemory(LARGE_INTEGER PhysAddr, ULONG Length, MEMORY_CACHING_TYPE CacheType);
//This function releases a mapping made by a previous call to RtMapMemory.
BOOL	RTAPI RtUnmapMemory(PVOID VirtualAddr);


///////////////////////////////////////////////////////////////////////////////
//
//Processor IO port space functions.
//
///////////////////////////////////////////////////////////////////////////////

// This function enables direct I / O port access from user context.
BOOL	RTAPI RtEnablePortIo(PUCHAR Port, ULONG Count);
// This function disables direct I / O port access from user context.
BOOL	RTAPI RtDisablePortIo(PUCHAR Port, ULONG Count);
// This function calls read data directly from an I/O port. Each function reads a different number of bytes of data.
UCHAR	RTAPI RtReadPortUchar(PUCHAR Port);
// This function calls read data directly from an I/O port. Each function reads a different number of bytes of data.
USHORT	RTAPI RtReadPortUshort(PUSHORT Port);
// This function calls read data directly from an I/O port. Each function reads a different number of bytes of data.
ULONG	RTAPI RtReadPortUlong(PULONG Port);
// This function calls copy data from an I/O port to a buffer until the buffer has been filled using a number of bytes 
// that you specify for each read operation.
VOID	RTAPI RtReadPortBufferUchar(PUCHAR Port, PUCHAR pBuffer, ULONG Count);
// This function calls copy data from an I/O port to a buffer until the buffer has been filled using a number of bytes 
// that you specify for each read operation.
VOID	RTAPI RtReadPortBufferUshort(PUSHORT Port, PUSHORT pBuffer, ULONG Count);
// This function calls copy data from an I/O port to a buffer until the buffer has been filled using a number of bytes 
// that you specify for each read operation.
VOID	RTAPI RtReadPortBufferUlong(PULONG Port, PULONG pBuffer, ULONG Count);
// This function calls write data directly to an I/O port from a specified buffer. Each function writes a different 
// number of bytes of data.
VOID	RTAPI RtWritePortUchar(PUCHAR Port, UCHAR Data);
// This function calls write data directly to an I/O port from a specified buffer. Each function writes a different 
// number of bytes of data.
VOID	RTAPI RtWritePortUshort(PUSHORT Port, USHORT Data);
// This function calls write data directly to an I/O port from a specified buffer. Each function writes a different 
// number of bytes of data.
VOID	RTAPI RtWritePortUlong(PULONG Port, ULONG Data);
// This function calls copy data from a buffer to an I/O port until the buffer has been emptied using a number of bytes 
// that you specify for each write operation.
VOID	RTAPI RtWritePortBufferUchar(PUCHAR Port, PUCHAR pData, ULONG Count);
// This function calls copy data from a buffer to an I/O port until the buffer has been emptied using a number of bytes 
// that you specify for each write operation.
VOID	RTAPI RtWritePortBufferUshort(PUSHORT Port, PUSHORT pData, ULONG Count);
// This function calls copy data from a buffer to an I/O port until the buffer has been emptied using a number of bytes 
// that you specify for each write operation.
VOID	RTAPI RtWritePortBufferUlong(PULONG Port, PULONG pData, ULONG Count);


///////////////////////////////////////////////////////////////////////////////
//
//Process functions.
//
///////////////////////////////////////////////////////////////////////////////

typedef struct _RTPROCESS_INFORMATION {
	DWORD		ProcessId;					// The Process ID.  If a Proxy Process we will assign this the Windows PID otherwise it's a RealTime ProcessID
	DWORD_PTR   ProcessorAffinityMask;		// A bit vector, each bit represents an allowed processor
	DWORD       AffinedProcessorNumber;		// The ideal processor number of the process.
	BOOL		UseLocalMemory;				// Boolean describing whether any localMemory has been used by this process
}RTPROCESS_INFORMATION, *PRTPROCESS_INFORMATION;

#if !defined(_WINDEF_)

typedef struct _PROC_THREAD_ATTRIBUTE_LIST *PPROC_THREAD_ATTRIBUTE_LIST, *LPPROC_THREAD_ATTRIBUTE_LIST;

typedef struct _PROCESS_INFORMATION {
    HANDLE  hProcess;
    HANDLE  hThread;
    DWORD   dwProcessId;
    DWORD   dwThreadId;
} PROCESS_INFORMATION, *LPPROCESS_INFORMATION;

typedef struct _STARTUPINFOA {
    DWORD   cb;
    LPSTR   lpReserved;
    LPSTR   lpDesktop;
    LPSTR   lpTitle;
    DWORD   dwX;
    DWORD   dwY;
    DWORD   dwXSize;
    DWORD   dwYSize;
    DWORD   dwXCountChars;
    DWORD   dwYCountChars;
    DWORD   dwFillAttribute;
    DWORD   dwFlags;
    WORD    wShowWindow;
    WORD    cbReserved2;
    LPBYTE  lpReserved2;
    HANDLE  hStdInput;
    HANDLE  hStdOutput;
    HANDLE  hStdError;
} STARTUPINFOA, *LPSTARTUPINFOA;

typedef struct _STARTUPINFOW {
    DWORD   cb;
    LPWSTR  lpReserved;
    LPWSTR  lpDesktop;
    LPWSTR  lpTitle;
    DWORD   dwX;
    DWORD   dwY;
    DWORD   dwXSize;
    DWORD   dwYSize;
    DWORD   dwXCountChars;
    DWORD   dwYCountChars;
    DWORD   dwFillAttribute;
    DWORD   dwFlags;
    WORD    wShowWindow;
    WORD    cbReserved2;
    LPBYTE  lpReserved2;
    HANDLE  hStdInput;
    HANDLE  hStdOutput;
    HANDLE  hStdError;
} STARTUPINFOW, *LPSTARTUPINFOW;

#ifdef UNICODE
typedef STARTUPINFOW STARTUPINFO;
typedef LPSTARTUPINFOW LPSTARTUPINFO;
#else
typedef STARTUPINFOA STARTUPINFO;
typedef LPSTARTUPINFOA LPSTARTUPINFO;
#endif // UNICODE

typedef struct _STARTUPINFOEXA {
    STARTUPINFOA StartupInfo;
    LPPROC_THREAD_ATTRIBUTE_LIST lpAttributeList;
} STARTUPINFOEXA, *LPSTARTUPINFOEXA;
typedef struct _STARTUPINFOEXW {
    STARTUPINFOW StartupInfo;
    LPPROC_THREAD_ATTRIBUTE_LIST lpAttributeList;
} STARTUPINFOEXW, *LPSTARTUPINFOEXW;

#ifdef UNICODE
typedef STARTUPINFOEXW STARTUPINFOEX;
typedef LPSTARTUPINFOEXW LPSTARTUPINFOEX;
#else
typedef STARTUPINFOEXA STARTUPINFOEX;
typedef LPSTARTUPINFOEXA LPSTARTUPINFOEX;
#endif // UNICODE

#define EXTENDED_STARTUPINFO_PRESENT		0x00080000




//
// Extended process and thread attribute support
//

#define PROC_THREAD_ATTRIBUTE_NUMBER    0x0000FFFF
#define PROC_THREAD_ATTRIBUTE_THREAD    0x00010000  // Attribute may be used with thread creation
#define PROC_THREAD_ATTRIBUTE_INPUT     0x00020000  // Attribute is input only
#define PROC_THREAD_ATTRIBUTE_ADDITIVE  0x00040000  // Attribute may be "accumulated," e.g. bitmasks, counters, etc.

//typedef enum _PROC_THREAD_ATTRIBUTE_NUM {
//    ProcThreadAttributeParentProcess = 0,
//    ProcThreadAttributeExtendedFlags,
//    ProcThreadAttributeHandleList,
//    ProcThreadAttributeGroupAffinity,
//    ProcThreadAttributePreferredNode,
//    ProcThreadAttributeIdealProcessor,
//    ProcThreadAttributeUmsThread,
//    ProcThreadAttributeMitigationPolicy,
//    ProcThreadAttributeMax
//} PROC_THREAD_ATTRIBUTE_NUM;

#define ProcThreadAttributeValue(Number, Thread, Input, Additive) \
    (((Number) & PROC_THREAD_ATTRIBUTE_NUMBER) | \
     ((Thread != FALSE) ? PROC_THREAD_ATTRIBUTE_THREAD : 0) | \
     ((Input != FALSE) ? PROC_THREAD_ATTRIBUTE_INPUT : 0) | \
     ((Additive != FALSE) ? PROC_THREAD_ATTRIBUTE_ADDITIVE : 0))

#define PROC_THREAD_ATTRIBUTE_PARENT_PROCESS \
    ProcThreadAttributeValue (ProcThreadAttributeParentProcess, FALSE, TRUE, FALSE)
#define PROC_THREAD_ATTRIBUTE_EXTENDED_FLAGS \
    ProcThreadAttributeValue (ProcThreadAttributeExtendedFlags, FALSE, TRUE, TRUE)
#define PROC_THREAD_ATTRIBUTE_HANDLE_LIST \
    ProcThreadAttributeValue (ProcThreadAttributeHandleList, FALSE, TRUE, FALSE)
#define PROC_THREAD_ATTRIBUTE_GROUP_AFFINITY \
    ProcThreadAttributeValue (ProcThreadAttributeGroupAffinity, TRUE, TRUE, FALSE)
#define PROC_THREAD_ATTRIBUTE_PREFERRED_NODE \
    ProcThreadAttributeValue (ProcThreadAttributePreferredNode, FALSE, TRUE, FALSE)
#define PROC_THREAD_ATTRIBUTE_IDEAL_PROCESSOR \
    ProcThreadAttributeValue (ProcThreadAttributeIdealProcessor, TRUE, TRUE, FALSE)
#define PROC_THREAD_ATTRIBUTE_UMS_THREAD \
    ProcThreadAttributeValue (ProcThreadAttributeUmsThread, TRUE, TRUE, FALSE)
#define PROC_THREAD_ATTRIBUTE_MITIGATION_POLICY \
    ProcThreadAttributeValue (ProcThreadAttributeMitigationPolicy, FALSE, TRUE, FALSE)

#define PROCESS_CREATION_MITIGATION_POLICY_DEP_ENABLE            0x01
#define PROCESS_CREATION_MITIGATION_POLICY_DEP_ATL_THUNK_ENABLE  0x02
#define PROCESS_CREATION_MITIGATION_POLICY_SEHOP_ENABLE          0x04

#define PROC_THREAD_ATTRIBUTE_REPLACE_VALUE     0x00000001

#endif //!defined(_WINDEF_)

// This function initializes a specified list of attributes for process and thread creation.
BOOL RTAPI RtInitializeProcThreadAttributeList(LPPROC_THREAD_ATTRIBUTE_LIST lpAttributeList, DWORD dwAttributeCount, DWORD dwFlags, PSIZE_T lpSize);
// This function deletes the specified list of attributes for process and thread creation.
VOID RTAPI RtDeleteProcThreadAttributeList(LPPROC_THREAD_ATTRIBUTE_LIST lpAttributeList);
// This function updates the specified attribute in a list of attributes for process and thread creation.
BOOL RTAPI RtUpdateProcThreadAttribute(LPPROC_THREAD_ATTRIBUTE_LIST lpAttributeList, DWORD dwFlags, DWORD_PTR Attribute, PVOID lpValue, SIZE_T cbSize, PVOID lpPreviousValue, PSIZE_T lpReturnSize);

typedef enum _RT_PROC_THREAD_ATTRIBUTE_NUM {
	RtProcThreadAttributeUseLocalMemory = 0x1008, //This is a magic number, AKA don't go changing
	RtProcThreadAttributeIdealProcessor,
	RtProcThreadAttributeProcessAffinity,
	RtProcThreadAttributeMax
} RT_PROC_THREAD_ATTRIBUTE_NUM;

#define RT_PROC_THREAD_ATTRIBUTE_USE_LOCAL_MEMORY \
	ProcThreadAttributeValue(RtProcThreadAttributeUseLocalMemory, FALSE, FALSE, FALSE)
#define RT_PROC_THREAD_ATTRIBUTE_IDEAL_PROCESSOR \
	ProcThreadAttributeValue(RtProcThreadAttributeIdealProcessor, FALSE, FALSE, FALSE)
#define RT_PROC_THREAD_ATTRIBUTE_PROCESS_AFFINITY \
	ProcThreadAttributeValue(RtProcThreadAttributeProcessAffinity, FALSE, FALSE, FALSE)

// This function creates and starts a new RTSS process. The new RTSS process runs the specified RTSS
// executable file.
BOOL RTAPI RtCreateProcessA(
						LPCSTR lpApplicationName,
						LPSTR lpCommandLine,
						LPSECURITY_ATTRIBUTES lpProcessAttributes,	// ignored 
						LPSECURITY_ATTRIBUTES lpThreadAttributes,	// ignored 
						BOOL bInheritHandles,						// ignored 
						DWORD dwCreationFlags,						// support CREATE_SUSPENDED
						LPVOID lpEnvironment,						// ignored 
						LPCSTR lpCurrentDirectory,					// ignored
						LPSTARTUPINFO lpStartupInfo,				// ignored
						LPPROCESS_INFORMATION lpProcessInformation);
// This function creates and starts a new RTSS process. The new RTSS process runs the specified RTSS
// executable file.
BOOL RTAPI RtCreateProcessW(
						LPCWSTR lpApplicationName,
						LPWSTR lpCommandLine,
						LPSECURITY_ATTRIBUTES lpProcessAttributes,	// ignored 
						LPSECURITY_ATTRIBUTES lpThreadAttributes,	// ignored 
						BOOL bInheritHandles,						// ignored 
						DWORD dwCreationFlags,						// support CREATE_SUSPENDED
						LPVOID lpEnvironment,						// ignored 
						LPCWSTR lpCurrentDirectory,					// ignored
						LPSTARTUPINFO lpStartupInfo,				// ignored
						LPPROCESS_INFORMATION lpProcessInformation);

#ifdef UNICODE
#define RtCreateProcess RtCreateProcessW
#else
#define RtCreateProcess RtCreateProcessA
#endif

// This function returns a handle to an existing process object.
HANDLE	RTAPI RtOpenProcess(DWORD dwAccess, BOOL bInherit, DWORD dwProcessId);
// This function retrieves the termination status of the specified process.
BOOL	RTAPI RtGetExitCodeProcess(HANDLE hProcess, LPDWORD lpExitCode);
// This function terminates the specified process and all of its threads.
BOOL	RTAPI RtTerminateProcess(HANDLE hProcess, UINT uExitCode);
// This function retrieves the process affinity mask for the specified RTSS process and the system 
// affinity mask for RTSS.
BOOL	RTAPI RtGetProcessAffinityMask(HANDLE hProcess, PDWORD_PTR ProcessAffinityMask, PDWORD_PTR SystemAffinityMask);
// This function sets a processor affinity mask for the threads of the specified RTSS process.
BOOL	RTAPI RtSetProcessAffinityMask(HANDLE hProcess, DWORD_PTR ProcessAffinityMask);
// This function gets the number represention for the lowest and highest rtss processors
BOOL	RTAPI RtGetProcessorInfo(PDWORD_PTR lpLowestSystemProcessor, PDWORD_PTR lpHighestSystemProcessor);
// This function retrieves the process identifier for each RTSS process object.
BOOL	RTAPI RtEnumProcesses(DWORD *pProcessIds, DWORD cb, DWORD *pBytesReturned);
// This function enumerates proxy processes associated with Windows processes linked to RTAPI.
BOOL	RTAPI RtEnumProxyProcesses(DWORD *pProcessIds, DWORD cb, DWORD *pBytesReturned);
// This function retrieves the process identifier for each RTSS process object.
BOOL	RTAPI RtEnumProcessesEx(PRTPROCESS_INFORMATION pProcesses, DWORD cb, DWORD *pBytesReturned);
// This function enumerates proxy processes associated with Windows processes linked to RTAPI.
BOOL	RTAPI RtEnumProxyProcessesEx(PRTPROCESS_INFORMATION pProcesses, DWORD cb, DWORD *pBytesReturned);
// This function retrieves a handle for each module in a specified RTSS process.
BOOL	RTAPI RtEnumProcessModules(HANDLE hProcess, HMODULE *lphModule, DWORD cb, LPDWORD lpcbNeeded);
// This function retrieves the base name of a specified module.
DWORD	RTAPI RtGetModuleBaseNameW(HANDLE hProcess, HMODULE hModule, LPWSTR lpBaseName, DWORD nSize);
// This function retrieves the base name of a specified module.
DWORD	RTAPI RtGetModuleBaseNameA(HANDLE hProcess, HMODULE hModule, LPSTR lpBaseName, DWORD nSize);
// This function determines if process with the passed process ID is being debugged.
BOOL RTAPI RtIsDebuggerPresent(DWORD processID, BOOL * pIsPresent);

// This function retrieves the fully-qualified path for the file that contains the specified module. 
// The module must have been loaded by the current process.
// (This Function is being Deprecated and eventually corrected to match with windows parameters)
DWORD	RTAPI RtGetModuleFileNameW(HANDLE hProcess, HMODULE hModule, LPWSTR lpBaseName, DWORD nSize);
// This function retrieves the fully-qualified path for the file that contains the specified module. 
// The module must have been loaded by the current process. (This Function is being Deprecated and eventually corrected to match with windows parameters)
DWORD	RTAPI RtGetModuleFileNameA(HANDLE hProcess, HMODULE hModule, LPSTR lpBaseName, DWORD nSize);

// This function retrieves the fully-qualified path for the file that contains the specified module. 
// The module must have been loaded by the current process.
DWORD	RTAPI RtGetModuleFileNameExW(HANDLE hProcess, HMODULE hModule, LPWSTR lpBaseName, DWORD nSize);
// This function retrieves the fully-qualified path for the file that contains the specified module. 
// The module must have been loaded by the current process.
DWORD	RTAPI RtGetModuleFileNameExA(HANDLE hProcess, HMODULE hModule, LPSTR lpBaseName, DWORD nSize);


#ifdef UNICODE
#define RtGetModuleBaseName RtGetModuleBaseNameW
#else
#define RtGetModuleBaseName RtGetModuleBaseNameA
#endif

#ifdef UNICODE
#define RtGetModuleFileName RtGetModuleFileNameW
#else
#define RtGetModuleFileName RtGetModuleFileNameA
#endif

#ifdef UNICODE
#define RtGetModuleFileNameEx RtGetModuleFileNameExW
#else
#define RtGetModuleFileNameEx RtGetModuleFileNameExA
#endif

///////////////////////////////////////////////////////////////////////////////
//
//Shared memory functions.
//
///////////////////////////////////////////////////////////////////////////////

#define SHM_MAP_WRITE 2
#define SHM_MAP_READ 1
#define SHM_MAP_ALL_ACCESS (SHM_MAP_WRITE + SHM_MAP_READ)

BOOL	RTAPI RtInitSharedMemory(VOID);
VOID	RTAPI RtUninitSharedMemory(VOID);
// This function creates a named region of physical memory that can be mapped by any process.
HANDLE	RTAPI RtCreateSharedMemoryA(DWORD flProtect, DWORD dwMaximumSizeHigh, DWORD dwMaximumSizeLow, LPCSTR lpName, VOID** location);
// This function creates a named region of physical memory that can be mapped by any process.
HANDLE	RTAPI RtCreateSharedMemoryW(DWORD flProtect, DWORD dwMaximumSizeHigh, DWORD dwMaximumSizeLow, LPCWSTR lpName, VOID** location);


#ifdef UNICODE
#define RtCreateSharedMemory  RtCreateSharedMemoryW
#else
#define RtCreateSharedMemory  RtCreateSharedMemoryA
#endif

// This function opens a named physical-mapping object.
HANDLE	RTAPI RtOpenSharedMemoryA(DWORD dwDesiredAccess, BOOL bInheritHandle, LPCSTR lpName, VOID** location);
// This function opens a named physical-mapping object.
HANDLE	RTAPI RtOpenSharedMemoryW(DWORD dwDesiredAccess, BOOL bInheritHandle, LPCWSTR lpName, VOID** location);


#ifdef UNICODE
#define RtOpenSharedMemory  RtOpenSharedMemoryW
#else
#define RtOpenSharedMemory  RtOpenSharedMemoryA
#endif


LPVOID	RTAPI RtMapViewOfSharedMemory(HANDLE hShmObj, DWORD dwDesiredAccess, DWORD dwOffsetHigh, DWORD dwOffsetLow, DWORD dwNumberOfBytesToMap);
BOOL	RTAPI RtUnmapViewOfSharedMemory(LPVOID lpBaseAddress);


///////////////////////////////////////////////////////////////////////////////
//
//Shutdown handler functions.
//
///////////////////////////////////////////////////////////////////////////////

#define SHTDN_REASON_RTX_SYSTEM_SHUTDOWN      1
#define SHTDN_REASON_WINDOWS_SYSTEM_SHUTDOWN  2
#define SHTDN_REASON_WINDOWS_STOP             3


// This function registers a stop/shutdown notification handler function with RTSS. The handler 
// function is called in its own thread when one of the system stop events occurs.
HANDLE	RTAPI RtAttachShutdownHandler(PSECURITY_ATTRIBUTES  pThreadAttributes, ULONG StackSize, VOID (RTFCNDCL *Routine)(PVOID context, LONG reason), PVOID Context, ULONG Priority);
// This function destroys the shutdown handler object created by RtAttachShutdownHandler.
BOOL	RTAPI RtReleaseShutdownHandler(HANDLE hShutdown);


///////////////////////////////////////////////////////////////////////////////
//
//Thread functions.
//
///////////////////////////////////////////////////////////////////////////////

#define RT_PRIORITY_MAX 127
#define RT_PRIORITY_MIN 0
// This function returns the priority value for the specified thread. In an RTSS environment, 
// RTSS has no distinct priority classes and the priority value specified is the only
// determination of a thread's priority.
// NOTE: For Windows applications linked with RTX64, RtGetThreadPriority has been deprecated 
// and will be removed from RTAPI in a future release. For compatibility, a new call - 
// RtGetProxyThreadPriority - has been defined. IntervalZero recommends that all references to 
// RtGetThreadPriority in a Windows application be replaced with RtGetProxyThreadPriority if 
// you want to control how a thread interacts with the Real-time Subsystem.
int		RTAPI RtGetThreadPriority(HANDLE hThread);
// This function sets the priority value for the specified thread.
// NOTE: For Windows applications linked with RTX64, RtSetThreadPriority has been deprecated 
// and will be removed from RTAPI in a future release. For compatibility, a new call -
// RtSetProxyThreadPriority - has been defined. IntervalZero recommends that all references to 
// RtSetThreadPriority in a Windows application be replaced with RtSetProxyThreadPriority if 
// you want to control how a thread interacts with the Real-time Subsystem.
BOOL	RTAPI RtSetThreadPriority(HANDLE hThread, int RtssPriority);

#ifndef UNDER_RTSS
	int		RTAPI RtGetProxyThreadPriority(HANDLE hThread);
	BOOL	RTAPI RtSetProxyThreadPriority(HANDLE hThread, int RtssPriority);
#endif

///////////////////////////////////////////////////////////////////////////////
//
//Time functions.
//
///////////////////////////////////////////////////////////////////////////////

#define CLOCK_1		1	// System Clock
#define CLOCK_2		2	// Real-time HAL Clock
#define CLOCK_3		3
#define CLOCK_4		4
#define CLOCK_FASTEST	0xFFFF	// Fastest available clock & timer
#define CLOCK_SYSTEM	CLOCK_1

typedef unsigned long CLOCK, *PCLOCK;

// This function suspends the current thread for the specified time with the granularity of the RTSS timer.
VOID	RTAPI RtSleepFt(PLARGE_INTEGER pSleepTime);
// This function suspends execution of the current thread for a specified time period.
ULONG	RTAPI RtSleepFtEx(PLARGE_INTEGER pSleepTime, BOOL bAlertable);
// This function obtains the current value of the specified clock.
BOOL	RTAPI RtGetClockTime(CLOCK Clock, PLARGE_INTEGER pTime);
// This function sets the current value of the specified clock.
BOOL	RTAPI RtSetClockTime(CLOCK Clock, PLARGE_INTEGER pTime);
// This function obtains the resolution of the specified clock.
BOOL	RTAPI RtGetClockResolution(CLOCK Clock, PLARGE_INTEGER pResolution);
// This function obtains the minimum timer period of the specified clock.The RtGetClockTime call delivers
// the clock time as 64-bit quantity of 100ns.
BOOL	RTAPI RtGetClockTimerPeriod(CLOCK Clock, PLARGE_INTEGER pTime);
// This function retrieves the counts per RTSS HAL Timer period
BOOL    RTAPI RtGetHalTimerPeriodCounts(ULONG *pPeriodicCounts, ULONG *pPeriodicCountsBase);
// This function sets the counts per RTSS HAL Timer period
BOOL    RTAPI RtSetHalTimerPeriodCounts(ULONG PeriodicCounts);
// This function creates a timer associated with the specified clock, and returns a handle to the timer.
HANDLE	RTAPI RtCreateTimer(PSECURITY_ATTRIBUTES pThreadAttributes, ULONG StackSize, VOID (RTFCNDCL *pRoutine) (PVOID context), PVOID Context, ULONG Priority, CLOCK Clock);
// This function creates a timer associated with the specified clock, and returns a handle to the timer.
// Use this call instead of RtCreateTimer if you  want to set the affinity for the timer handler.
HANDLE	RTAPI RtCreateTimerEx(PSECURITY_ATTRIBUTES pThreadAttributes, ULONG StackSize, VOID (RTFCNDCL *pRoutine) (PVOID context), PVOID Context, ULONG Priority, CLOCK Clock, KAFFINITY ProcessorEnableMask);
// This function deletes the timer specified by the given handle.
BOOL	RTAPI RtDeleteTimer(HANDLE hTimer);
// This function cancels the expiration of the indicated timer.
BOOL	RTAPI RtCancelTimer(HANDLE hTimer, PLARGE_INTEGER pTimeRemaining);
// This function returns the remaining relative time until the next expiration of the specified timer.
BOOL	RTAPI RtGetTimer(HANDLE hTimer, PLARGE_INTEGER pTimeRemaining);
// This function sets the expiration time and repeat interval on the specified timer.
BOOL	RTAPI RtSetTimer(HANDLE hTimer, PLARGE_INTEGER pExpiration, PLARGE_INTEGER pInterval);
// This function sets the expiration time and repeat interval on the specified timer.
BOOL	RTAPI RtSetTimerRelative(HANDLE hTimer, PLARGE_INTEGER pExpiration, PLARGE_INTEGER pInterval);
// This function retrieves the frequency of the high-resolution performance counter (based on the processor's 
// time-stamp counter, TSC). The frequency cannot change while the system is running.
BOOL    RTAPI RtQueryPerformanceFrequency(LARGE_INTEGER *lpFrequency);
// This function retrieves the current value of the high-resolution performance counter (based on the processor's 
// time-stamp counter, TSC).
BOOL    RTAPI RtQueryPerformanceCounter(LARGE_INTEGER *lpPerformanceCount);


///////////////////////////////////////////////////////////////////////////////
//
//Wait and object functions.
//
///////////////////////////////////////////////////////////////////////////////

#define	MAX_WFMO	16	// Maximum number of objects per WFMO

// This function allows a thread to wait on an object to be signaled.
DWORD	RTAPI RtWaitForSingleObject(HANDLE hHandle, DWORD dwMilliseconds);
// This function allows a thread to wait on one of multiple objects to be signaled.
DWORD	RTAPI RtWaitForMultipleObjects(DWORD dwCount, CONST HANDLE *lpHandles, BOOL bWaitAll, DWORD dwMilliseconds);
// This function closes an open RTSS object handle.
BOOL	RTAPI RtCloseHandle(HANDLE hHandle);
// This function creates a named or unnamed event object.
HANDLE	RTAPI RtCreateEventA(LPSECURITY_ATTRIBUTES lpEventAttributes, BOOL bManualReset, BOOL bInitialState, LPCSTR lpName);
// This function creates a named or unnamed event object.
HANDLE	RTAPI RtCreateEventW(LPSECURITY_ATTRIBUTES lpEventAttributes, BOOL bManualReset, BOOL bInitialState, LPCWSTR lpName);


#ifdef UNICODE
#define RtCreateEvent RtCreateEventW
#else
#define RtCreateEvent RtCreateEventA
#endif

// This function returns a handle of an existing named event object.
HANDLE	RTAPI RtOpenEventA(DWORD dwDesiredAccess, BOOL bInheritHandle, LPCSTR lpName);
// This function returns a handle of an existing named event object.
HANDLE	RTAPI RtOpenEventW(DWORD dwDesiredAccess, BOOL bInheritHandle, LPCWSTR lpName);


#ifdef UNICODE
#define RtOpenEvent RtOpenEventW
#else
#define RtOpenEvent RtOpenEventA
#endif

// This function provides a single operation that sets(to signaled) the state of the specified event object
// and then resets it(to non-signaled) after releasing the appropriate number of waiting threads.
BOOL	RTAPI RtPulseEvent(HANDLE hEvent);
// This function sets the state of the specified event object to non-signaled.
BOOL	RTAPI RtResetEvent(HANDLE hEvent);
// This function sets the state of the specified event object to signaled.
BOOL	RTAPI RtSetEvent(HANDLE hEvent);
// This function creates an RTSS mutex. A handle is returned to the newly created mutex object.
HANDLE	RTAPI RtCreateMutexA(LPSECURITY_ATTRIBUTES MutexAttributes, BOOL InitialOwner, LPCSTR lpName);
// This function creates an RTSS mutex. A handle is returned to the newly created mutex object.
HANDLE	RTAPI RtCreateMutexW(LPSECURITY_ATTRIBUTES MutexAttributes, BOOL InitialOwner, LPCWSTR lpName);


#ifdef UNICODE
#define RtCreateMutex RtCreateMutexW
#else
#define RtCreateMutex RtCreateMutexA
#endif

// This function returns a handle to the named RTSS mutex.
HANDLE	RTAPI RtOpenMutexA(DWORD DesiredAccess, BOOL InheritHandle, LPCSTR lpName);
// This function returns a handle to the named RTSS mutex.
HANDLE	RTAPI RtOpenMutexW(DWORD DesiredAccess, BOOL InheritHandle, LPCWSTR lpName);


#ifdef UNICODE
#define RtOpenMutex RtOpenMutexW
#else
#define RtOpenMutex RtOpenMutexA
#endif

// This function relinquishes ownership of an RTSS mutex.
BOOL	RTAPI RtReleaseMutex(HANDLE hMutex);
// This function creates a named or unnamed semaphore object.
HANDLE	RTAPI RtCreateSemaphoreA(LPSECURITY_ATTRIBUTES lpSemaphoreAttributes, LONG lInitialCount, LONG lMaximumCount, LPCSTR lpName);
// This function creates a named or unnamed semaphore object.
HANDLE	RTAPI RtCreateSemaphoreW(LPSECURITY_ATTRIBUTES lpSemaphoreAttributes, LONG lInitialCount, LONG lMaximumCount, LPCWSTR lpName);


#ifdef UNICODE
#define RtCreateSemaphore RtCreateSemaphoreW
#else
#define RtCreateSemaphore RtCreateSemaphoreA
#endif

// This function returns a handle of an existing named semaphore object.
HANDLE	RTAPI RtOpenSemaphoreA(DWORD dwDesiredAccess, BOOL bInheritHandle, LPCSTR lpName);
// This function returns a handle of an existing named semaphore object.
HANDLE	RTAPI RtOpenSemaphoreW(DWORD dwDesiredAccess, BOOL bInheritHandle, LPCWSTR lpName);


#ifdef UNICODE
#define RtOpenSemaphore RtOpenSemaphoreW
#else
#define RtOpenSemaphore RtOpenSemaphoreA
#endif

// This function increases the count of the specified semaphore object by a specified amount.
BOOL RTAPI RtReleaseSemaphore(HANDLE hSemaphore, LONG lReleaseCount, LPLONG lpPreviousCount);

///////////////////////////////////////////////////////////////////////////////
//
// Monitoring Framework APIs.
//
///////////////////////////////////////////////////////////////////////////////

// This function generates a custom monitoring event (an event of kind MF_EVENT_KIND_CUSTOM) having the
// specified custom kind and binary data.  Parameter "customKind" specifies the kind of the custom event.
// Parameter "customKind" must be an integer in the range 0 to 999 (inclusive).  Parameter "size" specifies
// the number of bytes pointed to by parameter "data" that should be copied into the custom event.  Parameter
// "size" cannot be greater than 500.  If parameter "size" is non-zero, parameter "data" must point to a
// buffer containing "size" bytes of custom binary data, otherwise parameter "data" must be NULL.
//
// This function returns TRUE if successful or if monitoring is not started, otherwise it returns FALSE and
// sets the last error value.  The following error values have special meaning:
//
//    ERROR_BUFFER_OVERFLOW  -- The internal buffer used to transport monitor data from the RTSS
//                              processors to the Windows processors are full.  Over time, more
//                              space will become available in the buffer.
//
//    ERROR_INVALID_FUNCTION -- An internal consistency check has failed.
BOOL RtGenerateEvent(unsigned int customKind, void * data, size_t size);

// MF_EVENT_KIND
// Each enumerator in this enumeration represents one kind of monitoring event.
typedef enum _MF_EVENT_KIND
{
    MF_EVENT_KIND_RESERVED = 0,                                  // 0 - This MUST be first and MUST have value 0!
    MF_EVENT_KIND_BUGCHECK_RTSS_RESERVED,                        // 1 - NOTE: This event will never appear.
    MF_EVENT_KIND_CONTEXTSWITCH,                                 // 2
    MF_EVENT_KIND_CONTIGUOUS_MEMORY_ALLOC,                       // 3
    MF_EVENT_KIND_CONTIGUOUS_MEMORY_ALLOC_FAIL,                  // 4
    MF_EVENT_KIND_CONTIGUOUS_MEMORY_ALLOC_SPECIFY_CACHE,         // 5
    MF_EVENT_KIND_CONTIGUOUS_MEMORY_ALLOC_SPECIFY_CACHE_FAIL,    // 6
    MF_EVENT_KIND_CONTIGUOUS_MEMORY_FREE,                        // 7
    MF_EVENT_KIND_CRITICAL_SECTION_DELETE,                       // 8
    MF_EVENT_KIND_CRITICAL_SECTION_ENTER,                        // 9
    MF_EVENT_KIND_CRITICAL_SECTION_INIT,                         // 10
    MF_EVENT_KIND_CRITICAL_SECTION_LEAVE,                        // 11
    MF_EVENT_KIND_CUSTOM,                                        // 12
    MF_EVENT_KIND_DATALOST,                                      // 13 - NOTE: This event cannot be disabled.
    MF_EVENT_KIND_EVENT_CREATE,                                  // 14
    MF_EVENT_KIND_EVENT_DESTROY,                                 // 15
    MF_EVENT_KIND_EVENT_OPEN,                                    // 16
    MF_EVENT_KIND_EXCEPTION_INTERRUPT,                           // 17
    MF_EVENT_KIND_FAST_SEMAPHORE_ACQUIRE,                        // 18
    MF_EVENT_KIND_FAST_SEMAPHORE_INIT,                           // 19
    MF_EVENT_KIND_FAST_SEMAPHORE_RELEASE,                        // 20
    MF_EVENT_KIND_FAST_SEMAPHORE_RELEASE_ALL,                    // 21
    MF_EVENT_KIND_HANDLE_CLOSE,                                  // 22
    MF_EVENT_KIND_HEAP_ALLOC,                                    // 23
    MF_EVENT_KIND_HEAP_ALLOC_FAIL,                               // 24
    MF_EVENT_KIND_HEAP_CREATE,                                   // 25
    MF_EVENT_KIND_HEAP_DESTROY,                                  // 26
    MF_EVENT_KIND_HEAP_FREE,                                     // 27
    MF_EVENT_KIND_IDEAL_PROCESSOR_SET,                           // 28
    MF_EVENT_KIND_INTERRUPT_LINEBASED_ATTACH,                    // 29
    MF_EVENT_KIND_INTERRUPT_LINEBASED_RELEASE,                   // 30
    MF_EVENT_KIND_INTERRUPT_MESSAGEBASED_ATTACH,                 // 31
    MF_EVENT_KIND_INTERRUPT_MESSAGEBASED_RELEASE,                // 32
    MF_EVENT_KIND_IST_HANDLER,                                   // 33
    MF_EVENT_KIND_LOCAL_MEMORY_ALLOC,                            // 34
    MF_EVENT_KIND_LOCAL_MEMORY_ALLOC_FAIL,                       // 35
    MF_EVENT_KIND_LOCAL_MEMORY_EXPAND,                           // 36
    MF_EVENT_KIND_LOCAL_MEMORY_FREE,                             // 37
    MF_EVENT_KIND_LOCAL_MEMORY_SHRINK,                           // 38
    MF_EVENT_KIND_MARKER,                                        // 39 - NOTE: This event cannot be disabled.
    MF_EVENT_KIND_MEMORY_MAP,                                    // 40
    MF_EVENT_KIND_MEMORY_UNMAP,                                  // 41
    MF_EVENT_KIND_MODULE_LOAD,                                   // 42
    MF_EVENT_KIND_MODULE_UNLOAD,                                 // 43
    MF_EVENT_KIND_MUTEX_CREATE,                                  // 44
    MF_EVENT_KIND_MUTEX_DESTROY,                                 // 45
    MF_EVENT_KIND_MUTEX_OPEN,                                    // 46
    MF_EVENT_KIND_MUTEX_RELEASE,                                 // 47
    MF_EVENT_KIND_PRIORITY_DEMOTION,                             // 48
    MF_EVENT_KIND_PRIORITY_PROMOTION,                            // 49
    MF_EVENT_KIND_PROCESS_AFFINITY_MASK_SET,                     // 50
    MF_EVENT_KIND_PROCESS_CREATE,                                // 51
    MF_EVENT_KIND_PROCESS_DESTROY,                               // 52
    MF_EVENT_KIND_PROCESS_OPEN,                                  // 53
    MF_EVENT_KIND_EVENT_PULSE,                                   // 54
    MF_EVENT_KIND_EVENT_RESET,                                   // 55
    MF_EVENT_KIND_THREAD_RESUME,                                 // 56
    MF_EVENT_KIND_SEMAPHORE_CREATE,                              // 57
    MF_EVENT_KIND_SEMAPHORE_DESTROY,                             // 58
    MF_EVENT_KIND_SEMAPHORE_OPEN,                                // 59
    MF_EVENT_KIND_SEMAPHORE_RELEASE,                             // 60
    MF_EVENT_KIND_EVENT_SET,                                     // 61
    MF_EVENT_KIND_UNHANDLED_EXCEPTION_FILTER_SET,                // 62
    MF_EVENT_KIND_SHARED_MEMORY_CREATE,                          // 63
    MF_EVENT_KIND_SHARED_MEMORY_DESTROY,                         // 64
    MF_EVENT_KIND_SHARED_MEMORY_OPEN,                            // 65
    MF_EVENT_KIND_SHUTDOWN_HANDLER_CALL,                         // 66 - NOTE: This event will never appear.
    MF_EVENT_KIND_SHUTDOWN_HANDLER_CREATE,                       // 67
    MF_EVENT_KIND_SHUTDOWN_HANDLER_DESTROY,                      // 68
    MF_EVENT_KIND_SUBSYSTEM_STOP,                                // 69
    MF_EVENT_KIND_THREAD_SUSPEND,                                // 70
    MF_EVENT_KIND_THREAD_AFFINITY_MASK_SET,                      // 71
    MF_EVENT_KIND_THREAD_CREATE,                                 // 72
    MF_EVENT_KIND_THREAD_DESTROY,                                // 73
    MF_EVENT_KIND_THREAD_PRIORITY_SET,                           // 74
    MF_EVENT_KIND_THREAD_QUANTUM_SET,                            // 75
    MF_EVENT_KIND_THREAD_SLEEP,                                  // 76
    MF_EVENT_KIND_TIMER_CANCEL,                                  // 77
    MF_EVENT_KIND_TIMER_CREATE,                                  // 78
    MF_EVENT_KIND_TIMER_DESTROY,                                 // 79
    MF_EVENT_KIND_TIMER_EXPIRATION,                              // 80
    MF_EVENT_KIND_TIMER_SET,                                     // 81
    MF_EVENT_KIND_TIME_QUANTUM_EXPIRE,                           // 82
    MF_EVENT_KIND_TLS_ALLOC,                                     // 83
    MF_EVENT_KIND_TLS_FREE,                                      // 84
    MF_EVENT_KIND_UNHANDLED_EXCEPTION_FILTER_CALL,               // 85
    MF_EVENT_KIND_WFMOEX_RETURN,                                 // 86
    MF_EVENT_KIND_WFMOEX_WAIT,                                   // 87
    MF_EVENT_KIND_WFSOEX_RETURN,                                 // 88
    MF_EVENT_KIND_WFSOEX_WAIT,                                   // 89
    MF_EVENT_KIND_WINDOWS_MEMORY_ALLOC,                          // 90
    MF_EVENT_KIND_WINDOWS_MEMORY_ALLOC_FAIL,                     // 91
    MF_EVENT_KIND_WINDOWS_MEMORY_FREE,                           // 92
    MF_EVENT_KIND_WINDOWS_MEMORY_FREE_FAIL,                      // 93
    MF_EVENT_KIND_TLS_FREE_FAIL,                                 // 94
    MF_EVENT_KIND_HEAP_FREE_FAIL,                                // 95
    MF_EVENT_KIND_CONTIGUOUS_MEMORY_FREE_FAIL,                   // 96
    MF_EVENT_KIND_LOCAL_MEMORY_FREE_FAIL,                        // 97
	MF_EVENT_KIND_FILE_CREATE,									 // 98
	MF_EVENT_KIND_FILE_DESTROY,									 // 99
	MF_EVENT_KIND_SRI_TO_WINDOWS,								 // 100
	MF_EVENT_KIND_SRI_TO_WINDOWS_RETURN,						 // 101
	MF_EVENT_KIND_THREAD_TERMINATE,								 // 102
    
	MF_EVENT_KIND_MAX  // This MUST be last!
} MF_EVENT_KIND;

// This typedef represents a set of triggers for a single event kind.  Each bit in this type
// represents one kind of trigger.
typedef unsigned long long MF_TRIGGERS;

// These macros define bit flags that represent each kind of trigger for use in instances of type
// MF_TRIGGERS.  Presently, there is only one kind of trigger (MF_TRIGGER_START_MONITORING).
#define MF_TRIGGER_NONE                 0ULL  // No trigger.
#define MF_TRIGGER_START_MONITORING     1ULL  // Start monitoring.

// This is the number of custom event kinds that can have triggers set/reset with RtMonitorControl
// operations MONITOR_CONTROL_SET_CUSTOM_EVENT_TRIGGERS and
// MONITOR_CONTROL_RESET_CUSTOM_EVENT_TRIGGERS.
#define MF_TRIGGERS_FOR_CUSTOM_EVENT_IDS_MAX 1000

// This enumeration defines control operations for use with function RtMonitorControl.  Some of these
// operations require monitoring to be enabled, otherwise they do not succeed.  This is noted in the
// documentation for each enumerator.  Monitoring can be enabled/disabled using the RTX64 Control Panel and
// the managed APIs Enable and Disable in class IntervalZero.RTX64.Monitor.Subsystem.
typedef enum _RTX64_MONITOR_CONTROL_OP
{
    // Reserved.  Do not use.
    MONITOR_CONTROL_RESERVED = 0,

    // This operation deterministically starts RTX64 monitoring.  This will succeed only when monitoring is
    // enabled and stopped.  If monitoring is not enabled, this operation fails with ERROR_NOT_READY.  If
    // monitoring is already started, this operation fails with ERROR_INVALID_FUNCTION.
    //
    // Parameters "data" and "size" are ignored for this operation.
    MONITOR_CONTROL_START,

    // This operation deterministically stops RTX64 monitoring.  This will succeed only when monitoring is
    // enabled and started, otherwise function RtMonitorControl will fail and set the last error value to
    // ERROR_NOT_READY.
    //
    // Parameters "data" and "size" are ignored for this operation.
    MONITOR_CONTROL_STOP,

    // This operation deterministically enables monitoring events and returns the resulting set of enabled and
    // disabled events.  This can be used to query which events are enabled by specifying no events to be
    // enabled.  Parameter "data" should point to an array of BOOLEANs that has MF_EVENT_KIND_MAX elements,
    // each of which specifies whether or not to enable the event that corresponds to an enumerator in
    // enumeration MF_EVENT_KIND.  If an array element is TRUE, the corresponding event is enabled, otherwise
    // its state remains unchanged.  The indices of the array should be the enumerators in enumeration
    // MF_EVENT_KIND.
    //
    // Parameter "size" should be the size (in bytes) of the array pointed to by parameter "data".
    //
    // If monitoring is not enabled, this operation fails with error ERROR_NOT_READY.
    //
    // When this function returns, the contents of the array pointed to by parameter "data" represent which
    // events are enabled and disabled (TRUE means enabled, FALSE means disabled).  NOTE: Any array element
    // passed in as FALSE (meaning "do not enable"), might come back as TRUE (meaning "it was already
    // enabled").
    MONITOR_CONTROL_ENABLE_EVENTS,

    // This operation deterministically disables zero or more monitoring events and returns the resulting set
    // of enabled and disabled events.  This can be used to query which events are enabled by specifying no
    // events to be disabled.  Parameter "data" should point to an array of BOOLEANs that has
    // MF_EVENT_KIND_MAX elements, each of which specifies whether or not to disable the event that
    // corresponds to an enumerator in enumeration MF_EVENT_KIND.  If an array element is TRUE, the
    // corresponding event is disabled, otherwise its state remains unchanged.  The indices of the array
    // should be the enumerators in enumeration MF_EVENT_KIND.
    //
    // Parameter "size" should be the size (in bytes) of the array pointed to by parameter "data".
    //
    // If monitoring is not enabled, this operation fails with error ERROR_NOT_READY.
    //
    // When this function returns, the contents of the array pointed to by parameter "data" represent which
    // events are enabled and disabled (TRUE means enabled, FALSE means disabled).  NOTE: Any array element
    // passed in as FALSE (meaning "do not disable"), might come back as TRUE (meaning "it was already
    // enabled").
    MONITOR_CONTROL_DISABLE_EVENTS,

    // This operation deterministically sets triggers for monitoring events.  A trigger causes one of a set of
    // pre-defined effects when monitoring is enabled and a given event can be generated, even if that event
    // is disabled.  For instance, if the trigger MF_TRIGGER_START_MONITORING is set for event
    // MF_EVENT_KIND_THREAD_SLEEP, then when that event can be generated (even if that event is disabled), the
    // trigger will cause monitoring to start.
    //
    // Parameter "data" should point to an array of MF_TRIGGERS that contains exactly MF_EVENT_KIND_MAX
    // elements, each of which specifies the triggers to set for the corresponding enumerator in enumeration
    // MF_EVENT_KIND.  NOTE: Each bit that is set in this array represents triggers to be set.  Bits in this
    // array that are zero do not set or reset any triggers.
    //
    // Parameter "size" is ignored for this operation.
    //
    // The following kinds of monitoring events cannot have triggers set on them.  Attempting to do so will
    // cause RtMonitorControl to fail with ERROR_INVALID_PARAMETER.
    //
    //   -- MF_EVENT_KIND_RESERVED (Reserved)
    //   -- MF_EVENT_KIND_BUGCHECK_RTSS_RESERVED (Bugcheck RTSS)
    //   -- MF_EVENT_KIND_MARKER (Marker)
    //   -- MF_EVENT_KIND_SUBSYSTEM_STOP (Subsystem Stop)
    //   -- MF_EVENT_KIND_FAST_SEMAPHORE_ACQUIRE (Fast Semaphore Acquire)
    //   -- MF_EVENT_KIND_FAST_SEMAPHORE_RELEASE (Fast Semaphore Release)
    //   -- MF_EVENT_KIND_FAST_SEMAPHORE_RELEASE_ALL (Fast Semaphore Release All)
    //   -- MF_EVENT_KIND_WFSOEX_WAIT (WaitForSingleObjectEx Wait)
    //   -- MF_EVENT_KIND_WFMOEX_WAIT (WaitForMultipleObjectEx Wait)
    //   -- MF_EVENT_KIND_THREAD_SLEEP (Thread Sleep)
    //
    // This operation can be performed at any time, and its effects remain until the RTX64 subsystem stops,
    // but triggers do nothing unless monitoring is enabled.  Monitoring does not need to be started for
    // triggers to work.
    //
    // When this function returns, the contents of the array pointed to by parameter "data" represent the
    // triggers that are set for each event kind.  NOTE: This operation can only increase the number of
    // triggers that are set.  Use operation MONITOR_CONTROL_RESET_EVENT_TRIGGERS to reset (i.e., turn off)
    // triggers for certain events.
    MONITOR_CONTROL_SET_EVENT_TRIGGERS,

    // This operation deterministically resets (i.e., turns off) triggers for monitoring events.  A trigger
    // causes one of a set of pre-defined effects when monitoring is enabled and a given event can be
    // generated, even if that event is disabled.  For instance, if the trigger MF_TRIGGER_START_MONITORING is
    // set for event MF_EVENT_KIND_THREAD_SLEEP, then when that event can be generated (even if that event is
    // disabled), the trigger will cause monitoring to start.
    //
    // Parameter "data" should point to an array of MF_TRIGGERS that contains exactly MF_EVENT_KIND_MAX
    // elements, each of which specifies the triggers to reset for the corresponding enumerator in enumeration
    // MF_EVENT_KIND.  NOTE: Each bit that is set in this array represents triggers to be _reset_.  Bits in
    // this array that are zero do not set or reset any triggers.
    //
    // Parameter "size" is ignored for this operation.
    //
    // This operation can be performed at any time, and its effects remain until the RTX64 subsystem stops,
    // but triggers do nothing unless monitoring is enabled.  Monitoring does not need to be started for
    // triggers to work.
    //
    // When this function returns, the contents of the array pointed to by parameter "data" represent the
    // triggers that are set for each event kind.  NOTE: This operation can only decrease the number of
    // triggers that are set.  Use operation MONITOR_CONTROL_SET_EVENT_TRIGGERS to set triggers for certain
    // events.
    MONITOR_CONTROL_RESET_EVENT_TRIGGERS,

    // This operation deterministically sets triggers for custom monitoring events.  A trigger causes one of a
    // set of pre-defined effects when monitoring is enabled and a given event can be generated, even if that
    // event is disabled.  For instance, if the trigger MF_TRIGGER_START_MONITORING is set for custom event
    // kind 42, then when that custom event kind can be generated (even if custom events are disabled), the
    // trigger will cause monitoring to start.
    //
    // Parameter "data" should point to an array of MF_TRIGGERS that contains exactly
    // MF_TRIGGERS_FOR_CUSTOM_EVENT_IDS_MAX elements, each of which specifies the triggers to set for the
    // corresponding custom event kind.  NOTE: Each bit that is set in this array represents triggers to be
    // set.  Bits in this array that are zero do not set or reset any triggers.
    //
    // Parameter "size" is ignored for this operation.
    //
    // This operation can be performed at any time, and its effects remain until the RTX64 subsystem stops,
    // but triggers do nothing unless monitoring is enabled.  Monitoring does not need to be started for
    // triggers to work.
    //
    // When this function returns, the contents of the array pointed to by parameter "data" represent the
    // triggers that are set for each custom event kind in the range 0 to
    // MF_TRIGGERS_FOR_CUSTOM_EVENT_IDS_MAX - 1.  NOTE: This operation can only increase the number of
    // triggers that are set.  Use operation MONITOR_CONTROL_RESET_CUSTOM_EVENT_TRIGGERS to reset (i.e., turn
    // off) triggers for certain custom event kinds.
    MONITOR_CONTROL_SET_CUSTOM_EVENT_TRIGGERS,

    // This operation deterministically resets (i.e., turns off) triggers for custom monitoring events.  A
    // trigger causes one of a set of pre-defined effects when monitoring is enabled and a given event can be
    // generated, even if that event is disabled.  For instance, if the trigger MF_TRIGGER_START_MONITORING is
    // set for custom event kind 42, then when that custom event kind can be generated (even if custom events
    // are disabled), the trigger will cause monitoring to start.
    //
    // Parameter "data" should point to an array of MF_TRIGGERS that contains exactly
    // MF_TRIGGERS_FOR_CUSTOM_EVENT_IDS_MAX elements, each of which specifies the triggers to reset for the
    // corresponding custom event kind.  NOTE: Each bit that is set in this array represents triggers to be
    // _reset_.  Bits in this array that are zero do not set or reset any triggers.
    //
    // Parameter "size" is ignored for this operation.
    //
    // This operation can be performed at any time, and its effects remain until the RTX64 subsystem stops,
    // but triggers do nothing unless monitoring is enabled.  Monitoring does not need to be started for
    // triggers to work.
    //
    // When this function returns, the contents of the array pointed to by parameter "data" represent the
    // triggers that are set for each custom event kind in the range 0 to
    // MF_TRIGGERS_FOR_CUSTOM_EVENT_IDS_MAX - 1.  NOTE: This operation can only decrease the number of
    // triggers that are set.  Use operation MONITOR_CONTROL_SET_CUSTOM_EVENT_TRIGGERS to set triggers for
    // certain custom event kinds.
    MONITOR_CONTROL_RESET_CUSTOM_EVENT_TRIGGERS,
} RTX64_MONITOR_CONTROL_OP, * PRTX64_MONITOR_CONTROL_OP;

// This function performs monitoring control operations.  Parameter "operation" specifies which control
// operation to perform.  Parameter "data" points to (optional) operand data (see documentation for
// RTX64_MONITOR_CONTROL_OP for the structure of this data).  Parameter "data" is ignored if it is not needed
// for a given operation.  Parameter "size" specifies the size (in bytes) of the operand data pointed to by
// parameter "data".  Parameter "size" is ignored if parameter "data" is not needed for a given operation (see
// the documentation for enumeration RTX64_MONITOR_CONTROL_OP).
//
// This function is not thread-safe.  Client code should implement any needed serialization.
//
// This function returns TRUE if successful.  Otherwise, it returns FALSE and sets the last error value.
BOOL RTAPI RtMonitorControl(RTX64_MONITOR_CONTROL_OP operation, void * data, size_t size);

///////////////////////////////////////////////////////////////////////////////
//
// Miscellanious functions.
//
///////////////////////////////////////////////////////////////////////////////

BOOL RTAPI RtQueryRtssInformation(ULONG *nRtssProcesses, ULONG *nProxyProcesses, ULONG *nRttcpipStacks, ULONG *nDebuggers);
BOOL RTAPI RtGetAllObjects(void * objectData, size_t * pSize);
// This function provides the ability to determine whether code is running in RTSS or Windows.
BOOL RTAPI RtIsInRtss();
int __cdecl RtAtoi(LPCSTR pString);
int __cdecl RtWtoi(const WCHAR *string);
int __cdecl RtPrintf(LPCSTR pFormat, ...);
int __cdecl RtWprintf(const WCHAR *fmt, ...);

// Gets the names of dongles attached to the machine and writes them to the passed buffer
// If user passed an out buffer for dongle names populates it along with dongle count 
// otherwise just populates the dongle count.
//
//Parameters:	
//	dongleData -	pointer to array of strings to write dongle data to. Each string must be
//					allocated and has to be at least DONGLE_ID_MAX_LENGTH + 1 characters long.
//					Can be NULL. In this case only dongleCount is popupated. 
//	dongleCount -	before the call represents number of elements in the passed dongleData array
//					after call updated to the number of found dongles. Cannot be null.	
BOOL 
RTAPI 
//This function retrieves a list of the IntervalZero-provided dongles attached to the machine.
RtGetDonglesA( 
	LPSTR * dongleData,
	DWORD * dongleCount );

// Gets the names of dongles attached to the machine and writes them to the passed buffer
// If user passed an out buffer for dongle names populates it along with dongle count 
// otherwise just populates the dongle count.
//
//Parameters:	
//	dongleData -	pointer to array of strings to write dongle data to. Each string must be
//					allocated and has to be at least DONGLE_ID_MAX_LENGTH + 1 characters long.
//					Can be NULL. In this case only dongleCount is popupated. 
//	dongleCount -	before the call represents number of elements in the passed dongleData array
//					after call updated to the number of found dongles. Cannot be null.	
BOOL 
RTAPI 
// This function retrieves a list of the IntervalZero-provided dongles attached to the machine.
RtGetDonglesW( 
	LPWSTR * dongleData  , 
	DWORD * dongleCount );

#ifdef UNICODE
#define RtGetDongles RtGetDonglesW
#else
#define RtGetDongles RtGetDonglesA
#endif

// This function retrieves information about all licenses available on the system.
BOOL RTAPI RtGetLicensesA(	
	PRT_LICENSE_INFO_A result,
	DWORD * length );

// This function retrieves information about all licenses available on the system.
BOOL RTAPI RtGetLicensesW(	
	PRT_LICENSE_INFO_W result,
	DWORD * length );

#ifdef UNICODE
#define RT_LICENSE_INFO RT_LICENSE_INFO_W
#define RtGetLicenses RtGetLicensesW
#else
#define RT_LICENSE_INFO RT_LICENSE_INFO_A
#define RtGetLicenses RtGetLicensesA
#endif

// This function verifies whether the specified version of the RTX TCP-IP stack is installed and has
// a valid license.
BOOL RTAPI RtIsTcpStackLicensed (DWORD majorVersion);

// This function verifies whether the specified version of the RTX runtime is installed and has a
// valid license.
BOOL RTAPI RtIsRuntimeLicensed (DWORD majorVersion);

#ifdef UNICODE
// This function returns whether the specified RTSS application binary can run.  This means it has
// been built to run with the provided license feature and that there is a valid license for the
// feature on the system.
#define RtIsAppRunnable RtIsAppRunnableW
#else
// This function returns whether the specified RTSS application binary can run.  This means it has
// been built to run with the provided license feature and that there is a valid license for the
// feature on the system.
#define RtIsAppRunnable RtIsAppRunnableA
#endif

// This function returns whether the specified RTSS application binary can run.  This means it has
// been built to run with the provided license feature and that there is a valid license for the
// feature on the system.
BOOL RTAPI RtIsAppRunnableA(
	LPSTR applicationPath,
	LPSTR featureName,
	DWORD majorVersion 
	);

// This function returns whether the specified RTSS application binary can run.  This means it has
// been built to run with the provided license feature and that there is a valid license for the
// feature on the system.
BOOL RTAPI RtIsAppRunnableW(
	LPWSTR applicationPath,
	LPWSTR featureName,
	DWORD majorVersion
	);

// This function retrieves the version information about the currently installer runtime.
BOOL RTAPI RtGetRuntimeVersionEx(RT_VERSION_INFO* versionInfo );

#ifdef __cplusplus
}
#endif

#ifdef UNDER_RTSS
#ifdef UNDER_RTSS_UNSUPPORTED_CRT_APIS
#pragma deprecated(	_CreateFrameInfo,_CxxThrowException,_FPE_Raise,_FindAndUnlinkFrame,_Getdays,_Getmonths,_Gettnames,_HUGE,\
					_IsExceptionObjectToBeDestroyed,_Lock_shared_ptr_spin_lock,_Strftime,_Unlock_shared_ptr_spin_lock,_W_Getdays,\
					_W_Getmonths,_W_Gettnames,_Wcsftime,_XcptFilter,__AdjustPointer,__BuildCatchObject,__BuildCatchObjectHelper,\
					__C_specific_handler,__CppXcptFilter,__CxxDetectRethrow,__CxxExceptionFilter,__CxxFrameHandler,__CxxFrameHandler2,\
					__CxxFrameHandler3,__CxxQueryExceptionSize,__CxxRegisterExceptionObject,__CxxUnregisterExceptionObject,\
					__DestructExceptionObject,__FrameUnwindFilter,__NLG_Dispatch2,__NLG_Return2,__RTCastToVoid,__RTDynamicCast,\
					__RTtypeid,__STRINGTOLD,__STRINGTOLD_L,__TypeMatch,___lc_codepage_func,___lc_collate_cp_func,___lc_locale_name_func,\
					___mb_cur_max_func,___mb_cur_max_l_func,___setlc_active_func,___unguarded_readlc_active_add_func,__argc,__argv,\
					__badioinfo,__clean_type_info_names_internal,__create_locale,__crtCompareStringA,__crtCompareStringEx,\
					__crtCompareStringW,__crtCreateSemaphoreExW,__crtCreateSymbolicLinkW,__crtEnumSystemLocalesEx,__crtFlsAlloc,\
					__crtFlsFree,__crtFlsGetValue,__crtFlsSetValue,__crtGetDateFormatEx,__crtGetLocaleInfoEx,__crtGetShowWindowMode,\
					__crtGetTimeFormatEx,__crtGetUserDefaultLocaleName,__crtInitializeCriticalSectionEx,__crtIsPackagedApp,\
					__crtIsValidLocaleName,__crtLCMapStringA,__crtLCMapStringEx,__crtLCMapStringW,__crtSetThreadStackGuarantee,\
					__crtSetUnhandledExceptionFilter,__crt_debugger_hook,__daylight,__dllonexit,__doserrno,__dstbias,__fpecode,\
					__free_locale,__get_current_locale,__get_flsindex,__get_tlsindex,__getmainargs,__initenv,__iob_func,__isascii,\
					__iscsym,__iscsymf,__iswcsym,__iswcsymf,__jump_unwind,__lconv,__lconv_init,__mb_cur_max,__p___argc,__p___argv,\
					__p___initenv,__p___mb_cur_max,__p___wargv,__p___winitenv,__p__acmdln,__p__commode,__p__daylight,__p__dstbias,\
					__p__environ,__p__fmode,__p__iob,__p__mbcasemap,__p__mbctype,__p__pctype,__p__pgmptr,__p__pwctype,__p__timezone,\
					__p__tzname,__p__wcmdln,__p__wenviron,__p__wpgmptr,__pctype_func,__pioinfo,__pwctype_func,__pxcptinfoptrs,\
					__report_gsfailure,__set_app_type,__setlc_active,__setusermatherr,__strncnt,__swprintf_l,__sys_errlist,\
					__sys_nerr,__threadhandle,__threadid,__timezone,__toascii,__tzname,__unDName,__unDNameEx,__unDNameHelper,\
					__uncaught_exception,__unguarded_readlc_active,__vswprintf_l,__wargv,__wcserror,__wcserror_s,__wcsncnt,\
					__wgetmainargs,__winitenv,_abs64,_access,_access_s,_acmdln,_amsg_exit,_assert,_atodbl,_atodbl_l,_atof_l,\
					_atoflt,_atoflt_l,_atoi64,_atoi64_l,_atoi_l,_atol_l,_atoldbl,_atoldbl_l,_beep,_beginthread,_beginthreadex,\
					_byteswap_uint64,_byteswap_ulong,_byteswap_ushort,_c_exit,_cabs,_callnewh,_calloc_crt,_cexit,_cgets,_cgets_s,\
					_cgetws,_cgetws_s,_chdir,_chdrive,_chgsign,_chmod,_chsize,_chsize_s,_clearfp,_close,_commit,_commode,\
					_configthreadlocale,_control87 ,_controlfp_s,_copysign,_cprintf,_cprintf_l,_cprintf_p,_cprintf_p_l,_cprintf_s,\
					_cprintf_s_l,_cputs,_cputws,_creat,_create_locale,_cscanf,_cscanf_l,_cscanf_s,_cscanf_s_l,_ctime32,_ctime32_s,\
					_ctime64,_ctime64_s,_cwait,_cwprintf,_cwprintf_l,_cwprintf_p,_cwprintf_p_l,_cwprintf_s,_cwprintf_s_l,_cwscanf,\
					_cwscanf_l,_cwscanf_s,_cwscanf_s_l,_daylight,_difftime32,_difftime64,_dosmaperr,_dstbias,_dup,_dup2,_dupenv_s,\
					_ecvt,_ecvt_s,_endthread,_endthreadex,_environ,_eof,_errno,_execl,_execle,_execlp,_execlpe,_execv,_execve,_execvp,\
					_execvpe,_exit,_expand,_fclose_nolock,_fcloseall,_fcvt,_fcvt_s,_fdopen,_fflush_nolock,_fgetchar,_fgetwc_nolock,\
					_fgetwchar,_filbuf,_filelength,_filelengthi64,_fileno,_findclose,_findfirst32,_findfirst32i64,_findfirst64,\
					_findfirst64i32,_findnext32,_findnext32i64,_findnext64,_findnext64i32,_finite,_flsbuf,_flushall,_fmode,_fpclass,\
					_fpieee_flt,_fprintf_l,_fprintf_p,_fprintf_p_l,_fprintf_s_l,_fputchar,_fputwc_nolock,_fputwchar,_fread_nolock,\
					_fread_nolock_s,_free_locale,_freea,_freea_s,_freefls,_fscanf_l,_fscanf_s_l,_fseek_nolock,_fseeki64,_fseeki64_nolock,\
					_fsopen,_fstat32,_fstat32i64,_fstat64,_fstat64i32,_ftell_nolock,_ftelli64,_ftelli64_nolock,_ftime32,_ftime32_s,\
					_ftime64,_ftime64_s,_fullpath,_futime32,_futime64,_fwprintf_l,_fwprintf_p,_fwprintf_p_l,_fwprintf_s_l,_fwrite_nolock,\
					_fwscanf_l,_fwscanf_s_l,_gcvt,_gcvt_s,_get_current_locale,_get_daylight,_get_doserrno,_get_dstbias,_get_errno,\
					_get_fmode,_get_heap_handle,_get_invalid_parameter_handler,_get_osfhandle,_get_output_format,_get_pgmptr,\
					_get_printf_count_output,_get_purecall_handler,_get_terminate,_get_timezone,_get_tzname,_get_unexpected,_get_wpgmptr,\
					_getc_nolock,_getch,_getch_nolock,_getche,_getche_nolock,_getcwd,_getdcwd,_getdiskfree,_getdllprocaddr,_getdrive,\
					_getdrives,_getmaxstdio,_getmbcp,_getpid,_getptd,_getsystime,_getw,_getwch,_getwch_nolock,_getwche,_getwche_nolock,\
					_getws,_getws_s,_gmtime32,_gmtime32_s,_gmtime64,_gmtime64_s,_heapadd,_heapchk,_heapmin,_heapset,_heapused,_heapwalk,\
					_hypot,_hypotf,_i64toa,_i64toa_s,_i64tow,_i64tow_s,_initptd,_initterm,_initterm_e,_invalid_parameter,\
					_invalid_parameter_noinfo,_invalid_parameter_noinfo_noreturn,_invoke_watson,_iob,_isalnum_l,_isalpha_l,_isatty,\
					_iscntrl_l,_isctype,_isctype_l,_isdigit_l,_isgraph_l,_isleadbyte_l,_islower_l,_ismbbalnum,_ismbbalnum_l,_ismbbalpha,\
					_ismbbalpha_l,_ismbbgraph,_ismbbgraph_l,_ismbbkalnum,_ismbbkalnum_l,_ismbbkana,_ismbbkana_l,_ismbbkprint,\
					_ismbbkprint_l,_ismbbkpunct,_ismbbkpunct_l,_ismbblead,_ismbblead_l,_ismbbprint,_ismbbprint_l,_ismbbpunct,\
					_ismbbpunct_l,_ismbbtrail,_ismbbtrail_l,_ismbcalnum,_ismbcalnum_l,_ismbcalpha,_ismbcalpha_l,_ismbcdigit,\
					_ismbcdigit_l,_ismbcgraph,_ismbcgraph_l,_ismbchira,_ismbchira_l,_ismbckata,_ismbckata_l,_ismbcl0,_ismbcl0_l,\
					_ismbcl1,_ismbcl1_l,_ismbcl2,_ismbcl2_l,_ismbclegal,_ismbclegal_l,_ismbclower,_ismbclower_l,_ismbcprint,_ismbcprint_l,\
					_ismbcpunct,_ismbcpunct_l,_ismbcspace,_ismbcspace_l,_ismbcsymbol,_ismbcsymbol_l,_ismbcupper,_ismbcupper_l,_ismbslead,\
					_ismbslead_l,_ismbstrail,_ismbstrail_l,_isnan,_isprint_l,_ispunct_l,_isspace_l,_isupper_l,_iswalnum_l,_iswalpha_l,\
					_iswcntrl_l,_iswcsym_l,_iswcsymf_l,_iswctype_l,_iswdigit_l,_iswgraph_l,_iswlower_l,_iswprint_l,_iswpunct_l,_iswspace_l,\
					_iswupper_l,_iswxdigit_l,_isxdigit_l,_itoa,_itoa_s,_itow,_itow_s,_j0,_j1,_jn,_kbhit,_lfind,_lfind_s,_loaddll,\
					_localtime32,_localtime32_s,_localtime64,_localtime64_s,_lock,_lock_file,_locking,_logb,_logbf,_lrotl,_lrotr,_lsearch,\
					_lsearch_s,_lseek,_lseeki64,_ltoa,_ltoa_s,_ltow,_ltow_s,_makepath,_makepath_s,_malloc_crt,_mbbtombc,_mbbtombc_l,\
					_mbbtype,_mbbtype_l,_mbcasemap,_mbccpy,_mbccpy_l,_mbccpy_s,_mbccpy_s_l,_mbcjistojms,_mbcjistojms_l,_mbcjmstojis,\
					_mbcjmstojis_l,_mbclen,_mbclen_l,_mbctohira,_mbctohira_l,_mbctokata,_mbctokata_l,_mbctolower,_mbctolower_l,_mbctombb,\
					_mbctombb_l,_mbctoupper,_mbctoupper_l,_mbctype,_mblen_l,_mbsbtype,_mbsbtype_l,_mbscat_s,_mbscat_s_l,_mbschr,_mbschr_l,\
					_mbscmp,_mbscmp_l,_mbscoll,_mbscoll_l,_mbscpy_s,_mbscpy_s_l,_mbscspn,_mbscspn_l,_mbsdec,_mbsdec_l,_mbsicmp,_mbsicmp_l,\
					_mbsicoll,_mbsicoll_l,_mbsinc,_mbsinc_l,_mbslen,_mbslen_l,_mbslwr,_mbslwr_l,_mbslwr_s,_mbslwr_s_l,_mbsnbcat,_mbsnbcat_l,\
					_mbsnbcat_s,_mbsnbcat_s_l,_mbsnbcmp,_mbsnbcmp_l,_mbsnbcnt,_mbsnbcnt_l,_mbsnbcoll,_mbsnbcoll_l,_mbsnbcpy,_mbsnbcpy_l,\
					_mbsnbcpy_s,_mbsnbcpy_s_l,_mbsnbicmp,_mbsnbicmp_l,_mbsnbicoll,_mbsnbicoll_l,_mbsnbset,_mbsnbset_l,_mbsnbset_s,\
					_mbsnbset_s_l,_mbsncat,_mbsncat_l,_mbsncat_s,_mbsncat_s_l,_mbsnccnt,_mbsnccnt_l,_mbsncmp,_mbsncmp_l,_mbsncoll,\
					_mbsncoll_l,_mbsncpy,_mbsncpy_l,_mbsncpy_s,_mbsncpy_s_l,_mbsnextc,_mbsnextc_l,_mbsnicmp,_mbsnicmp_l,_mbsnicoll,\
					_mbsnicoll_l,_mbsninc,_mbsninc_l,_mbsnlen,_mbsnlen_l,_mbsnset,_mbsnset_l,_mbsnset_s,_mbsnset_s_l,_mbspbrk,_mbspbrk_l,\
					_mbsrchr,_mbsrchr_l,_mbsrev,_mbsrev_l,_mbsset,_mbsset_l,_mbsset_s,_mbsset_s_l,_mbsspn,_mbsspn_l,_mbsspnp,_mbsspnp_l,\
					_mbsstr,_mbsstr_l,_mbstok,_mbstok_l,_mbstok_s,_mbstok_s_l,_mbstowcs_l,_mbstowcs_s_l,_mbstrlen,_mbstrlen_l,_mbstrnlen,\
					_mbstrnlen_l,_mbsupr,_mbsupr_l,_mbsupr_s,_mbsupr_s_l,_mbtowc_l,_memccpy,_memicmp,_memicmp_l,_mkdir,_mkgmtime32,\
					_mkgmtime64,_mktemp,_mktemp_s,_mktime32,_mktime64,_msize,_nextafter,_onexit,_open,_open_osfhandle,_pclose,_pctype,\
					_pgmptr,_pipe,_popen,_printf_l,_printf_p,_printf_p_l,_printf_s_l,_purecall,_putch,_putch_nolock,_putenv,_putenv_s,\
					_putw,_putwch,_putwch_nolock,_putws,_pwctype,_read,_realloc_crt,_recalloc,_recalloc_crt,_resetstkoflw,_rmdir,_rmtmp,\
					_rotl,_rotl64,_rotr,_rotr64,_scalb,_scanf_l,_scanf_s_l,_scprintf,_scprintf_l,_scprintf_p,_scprintf_p_l,_scwprintf,\
					_scwprintf_l,_scwprintf_p,_scwprintf_p_l,_searchenv,_searchenv_s,_set_abort_behavior,_set_controlfp,_set_doserrno,\
					_set_errno,_set_error_mode,_set_fmode,_set_invalid_parameter_handler,_set_malloc_crt_max_wait,_set_output_format,\
					_set_printf_count_output,_set_purecall_handler,_seterrormode,_setjmp,_setjmpex,_setmaxstdio,_setmbcp,_setmode,\
					_setsystime,_sleep,_snprintf,_snprintf_c,_snprintf_c_l,_snprintf_l,_snprintf_s,_snprintf_s_l,_snscanf,_snscanf_l,\
					_snscanf_s,_snscanf_s_l,_snwprintf,_snwprintf_l,_snwprintf_s,_snwprintf_s_l,_snwscanf,_snwscanf_l,_snwscanf_s,\
					_snwscanf_s_l,_sopen,_sopen_s,_spawnl,_spawnle,_spawnlp,_spawnlpe,_spawnv,_spawnve,_spawnvp,_spawnvpe,_splitpath,\
					_splitpath_s,_sprintf_l,_sprintf_p,_sprintf_p_l,_sprintf_s_l,_sscanf_l,_sscanf_s_l,_stat32,_stat32i64,_stat64,\
					_stat64i32,_statusfp,_strcoll_l,_strdate,_strdate_s,_strdup,_strerror,_strerror_s,_strftime_l,_stricmp,_stricmp_l,\
					_stricoll,_stricoll_l,_strlwr,_strlwr_l,_strlwr_s,_strlwr_s_l,_strncoll,_strncoll_l,_strnicmp,_strnicmp_l,_strnicoll,\
					_strnicoll_l,_strnset,_strnset_s,_strrev,_strset,_strset_s,_strtime,_strtime_s,_strtod_l,_strtoi64,_strtoi64_l,\
					_strtol_l,_strtoui64,_strtoui64_l,_strtoul_l,_strupr,_strupr_l,_strupr_s,_strupr_s_l,_strxfrm_l,_swab,_swprintf,\
					_swprintf_c,_swprintf_c_l,_swprintf_p,_swprintf_p_l,_swprintf_s_l,_swscanf_l,_swscanf_s_l,_sys_errlist,_sys_nerr,\
					_tell,_telli64,_tempnam,_time32,_time64,_timezone,_tolower,_tolower_l,_toupper,_toupper_l,_towlower_l,_towupper_l,\
					_tzname,_tzset,_ui64toa,_ui64toa_s,_ui64tow,_ui64tow_s,_ultoa,_ultoa_s,_ultow,_ultow_s,_umask,_umask_s,_ungetc_nolock,\
					_ungetch,_ungetch_nolock,_ungetwc_nolock,_ungetwch,_ungetwch_nolock,_unlink,_unloaddll,_unlock,_unlock_file,_utime32,\
					_utime64,_vcprintf,_vcprintf_l,_vcprintf_p,_vcprintf_p_l,_vcprintf_s,_vcprintf_s_l,_vcwprintf,_vcwprintf_l,_vcwprintf_p,\
					_vcwprintf_p_l,_vcwprintf_s,_vcwprintf_s_l,_vfprintf_l,_vfprintf_p,_vfprintf_p_l,_vfprintf_s_l,_vfwprintf_l,\
					_vfwprintf_p,_vfwprintf_p_l,_vfwprintf_s_l,_vprintf_l,_vprintf_p,_vprintf_p_l,_vprintf_s_l,_vscprintf,_vscprintf_l,\
					_vscprintf_p,_vscprintf_p_l,_vscwprintf,_vscwprintf_l,_vscwprintf_p,_vscwprintf_p_l,_vsnprintf,_vsnprintf_c,\
					_vsnprintf_c_l,_vsnprintf_l,_vsnprintf_s,_vsnprintf_s_l,_vsnwprintf,_vsnwprintf_l,_vsnwprintf_s,_vsnwprintf_s_l,\
					_vsprintf_l,_vsprintf_p,_vsprintf_p_l,_vsprintf_s_l,_vswprintf,_vswprintf_c,_vswprintf_c_l,_vswprintf_l,_vswprintf_p,\
					_vswprintf_p_l,_vswprintf_s_l,_vwprintf_l,_vwprintf_p,_vwprintf_p_l,_vwprintf_s_l,_waccess,_waccess_s,_wasctime,\
					_wasctime_s,_wassert,_wchdir,_wchmod,_wcmdln,_wcreat,_wcreate_locale,_wcscoll_l,_wcsdup,_wcserror,_wcserror_s,\
					_wcsftime_l,_wcsicmp,_wcsicmp_l,_wcsicoll,_wcsicoll_l,_wcslwr,_wcslwr_l,_wcslwr_s,_wcslwr_s_l,_wcsncoll,_wcsncoll_l,\
					_wcsnicmp,_wcsnicmp_l,_wcsnicoll,_wcsnicoll_l,_wcsnset,_wcsnset_s,_wcsrev,_wcsset,_wcsset_s,_wcstod_l,_wcstoi64,\
					_wcstoi64_l,_wcstol_l,_wcstombs_l,_wcstombs_s_l,_wcstoui64,_wcstoui64_l,_wcstoul_l,_wcsupr,_wcsupr_l,_wcsupr_s,\
					_wcsupr_s_l,_wcsxfrm_l,_wctime32,_wctime32_s,_wctime64,_wctime64_s,_wctomb_l,_wctomb_s_l,_wctype,_wdupenv_s,_wenviron,\
					_wexecl,_wexecle,_wexeclp,_wexeclpe,_wexecv,_wexecve,_wexecvp,_wexecvpe,_wfdopen,_wfindfirst32,_wfindfirst32i64,\
					_wfindfirst64,_wfindfirst64i32,_wfindnext32,_wfindnext32i64,_wfindnext64,_wfindnext64i32,_wfopen,_wfopen_s,_wfreopen,\
					_wfreopen_s,_wfsopen,_wfullpath,_wgetcwd,_wgetdcwd,_wgetenv,_wgetenv_s,_wmakepath,_wmakepath_s,_wmkdir,_wmktemp,\
					_wmktemp_s,_wopen,_wperror,_wpgmptr,_wpopen,_wprintf_l,_wprintf_p,_wprintf_p_l,_wprintf_s_l,_wputenv,_wputenv_s,\
					_wremove,_wrename,_write,_wrmdir,_wscanf_l,_wscanf_s_l,_wsearchenv,_wsearchenv_s,_wsetlocale,_wsopen,_wsopen_s,\
					_wspawnl,_wspawnle,_wspawnlp,_wspawnlpe,_wspawnv,_wspawnve,_wspawnvp,_wspawnvpe,_wsplitpath,_wsplitpath_s,_wstat32,\
					_wstat32i64,_wstat64,_wstat64i32,_wstrdate,_wstrdate_s,_wstrtime,_wstrtime_s,_wsystem,_wtempnam,_wtmpnam,_wtmpnam_s,\
					_wtof,_wtof_l,_wtoi,_wtoi64,_wtoi64_l,_wtoi_l,_wtol,_wtol_l,_wunlink,_wutime32,_wutime64,_y0,_y1,_yn,abort,acosf,\
					asctime,asctime_s,asinf,atan2f,atanf,atexit,bsearch_s,btowc,ceilf,clearerr,clearerr_s,clock,cosf,coshf,expf,fabsf,\
					feof,ferror,fflush,fgetc,fgetpos,fgetwc,fgetws,floorf,fmodf,fopen_s,fprintf_s,fputwc,fputws,fread_s,freopen,freopen_s,\
					fscanf,fscanf_s,fsetpos,fwprintf_s,fwrite,fwscanf,fwscanf_s,getchar,getenv,getenv_s,gets,gets_s,getwc,getwchar,\
					is_wctype,isleadbyte,llabs,lldiv,localeconv,log10f,logf,mblen,mbrlen,mbrtowc,mbsrtowcs,mbsrtowcs_s,mbstowcs,mbstowcs_s,\
					mbtowc,memcpy_s,memmove_s,modff,powf,printf_s,puts,putwc,putwchar,qsort_s,raise,rand_s,remove,rename,scanf,scanf_s,\
					setbuf,setlocale,setvbuf,sinf,sinhf,sprintf_s,sqrtf,sscanf_s,strcat_s,strcoll,strcpy_s,strerror_s,strftime,strncat_s,\
					strncpy_s,strnlen,strtok_s,strxfrm,swprintf_s,swscanf,swscanf_s,system,tanf,tanhf,tmpfile,tmpfile_s,tmpnam,tmpnam_s,\
					ungetc,ungetwc,vfprintf,vfprintf_s,vfwprintf,vfwprintf_s,vprintf,vprintf_s,vsprintf_s,vswprintf_s,vwprintf,vwprintf_s,\
					wcrtomb,wcrtomb_s,wcscat_s,wcscoll,wcscpy_s,wcsncat_s,wcsncpy_s,wcsnlen,wcspbrk,wcsrchr,wcsrtombs,wcsrtombs_s,wcsspn,\
					wcstok_s,wcstombs,wcstombs_s,wcstoul,wcsxfrm,wctob,wctomb,wctomb_s,wmemcpy_s,wmemmove_s,wprintf_s,wscanf,wscanf_s)

#endif
#endif

#pragma data_seg(push, "rtx64")
static unsigned char _rtx64_data_[256] = { 2, 0, 0, 0,
                                           RTX64_MAJOR, 0, 0, 0,
                                           RTX64_MINOR, 0, 0, 0,
                                           0, 0,
                                           0xba, 0xbb, 0x1e, 0xd2,
                                           0 };
#pragma data_seg(pop)

// This is necessary to prevent the Intel compiler from ignoring the #pragma data_seg(pop) above.
int static ___RTX64_DUMMY = 5;

#endif
