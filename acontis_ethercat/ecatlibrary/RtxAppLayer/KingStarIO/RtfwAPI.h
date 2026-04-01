// RtfwAPI.h
// Copyright (c) 2018 IntervalZero, Inc.  All rights reserved.
//
// This header file declares the public interface of the RTX64 Native Framework
// (RtfwAPI.DLL).  To implicitly link to that DLL, add import library RtfwAPI.LIB to your
// project's linker inputs.  This header file contains only brief summaries of each API,
// type, and constant.  See the RTX64 SDK documentation for detailed documentation.

#pragma once

#include <stdint.h>
#include <ErrorCodes.h>
#include <RtAPI.h>
#include <RTX64Base.h>

#ifdef __cplusplus
extern "C" {
#else
# include <stdbool.h>  // Needed for C clients to use type bool.
#endif

#if defined RTFWAPI_EXPORT
# define RTFWPROC __declspec(dllexport)
#else
# define RTFWPROC __declspec(dllimport)
#endif

// This enumeration represents the various ways the RTX64 subsystem can be configurated to start.
typedef enum _RTFW_START_MODE
{
    RTSM_MANUAL = 0,
    RTSM_AUTOMATIC,
    RTSM_MAX,           // Must be last!
} RTFW_START_MODE, * PRTFW_START_MODE;

// This structure represents the core configuration parameters of the RTX64 subsystem.
typedef struct _RTFW_SUBSYSTEM_CONFIGURATION
{
    size_t Size;
    RTFW_START_MODE StartMode;
    unsigned int HALTimerPeriod;
    uint64_t DefaultTimeQuantum;
    bool FreeThreadStackOnTerminateThread;
} RTFW_SUBSYSTEM_CONFIGURATION, * PRTFW_SUBSYSTEM_CONFIGURATION;

// This value can be assigned to member DefaultTimeQuantum in structure RTFW_SUBSYSTEM_CONFIGURATION to
// disable the use of time quantum scheduling.
#define RTFW_DISABLE_TIME_QUANTUM 0

// This function gets the core RTX64 subsystem configuration parameters.
RTFWPROC bool RtfwGetSubsystemConfiguration(RTFW_SUBSYSTEM_CONFIGURATION * pSubsystemConfiguration);

// This function sets the core RTX64 subsystem configuration parameters.
RTFWPROC bool RtfwSetSubsystemConfiguration(const RTFW_SUBSYSTEM_CONFIGURATION * pSubsystemConfiguration, DWORD * pWarning);

// This enumeration represents the possible exception handling modes used by the RTX64 subsystem.
typedef enum _RTFW_EXCEPTION_HANDLING
{
    RTEH_SEH_TERMINATE_PROCESS = 0, // Use SEH.  Terminate process on unhandled exception.
    RTEH_SEH_FREEZE_PROCESS,        // Use SEH.  Freeze process on unhandled exception.
    RTEH_NO_SEH_FREEZE_PROCESS,     // Don't use SEH.  Freeze process on exception.
    RTEH_NO_SEH_TERMINATE_PROCESS,  // Don't use SEH.  Terminate process on exception.
    RTEH_NO_SEH_HALT_SYSTEM,        // Don't use SEH.  Halt system and dump memory on exception.
    RTEH_MAX,                       // Must be last!
} RTFW_EXCEPTION_HANDLING, * PRTFW_EXCEPTION_HANDLING;

// This function gets the currently configured exception handling mode for the RTX64 subsystem.
RTFWPROC bool RtfwGetExceptionHandling(RTFW_EXCEPTION_HANDLING * pExceptionHandling);

// This function sets the currently configured exception handling mode for the RTX64 subsystem.
RTFWPROC bool RtfwSetExceptionHandling(const RTFW_EXCEPTION_HANDLING exceptionHandling);

// This structure represents the Local Memory configuration parameters for the RTX64 subsystem.
typedef struct _RTFW_LOCAL_MEMORY_CONFIGURATION
{
    size_t Size;
    bool UseLocalMemory;
    bool AutoExpandPool;
    bool AutoShrinkPool;
    unsigned int InitialSize;
    unsigned int ExpansionSize;
} RTFW_LOCAL_MEMORY_CONFIGURATION, * PRTFW_LOCAL_MEMORY_CONFIGURATION;

// This structure represents the RTX64 Server Console configuration parameters.
typedef struct _RTFW_CONSOLE_CONFIGURATION_A
{
	size_t Size;
	bool DisplayConsole;
	unsigned int DisplayBufferSize;
	bool LogOutputToFile;
	bool StartMinimized;
	bool SuppressWarnings;
	CHAR LogPathName[MAX_PATH];
} RTFW_CONSOLE_CONFIGURATION_A, * PRTFW_CONSOLE_CONFIGURATION_A;

// This structure represents the RTX64 Server Console configuration parameters.
typedef struct _RTFW_CONSOLE_CONFIGURATION_W
{
	size_t Size;
	bool DisplayConsole;
	unsigned int DisplayBufferSize;
	bool LogOutputToFile;
	bool StartMinimized;
	bool SuppressWarnings;
	WCHAR LogPathName[MAX_PATH];
} RTFW_CONSOLE_CONFIGURATION_W, * PRTFW_CONSOLE_CONFIGURATION_W;

#ifdef UNICODE
// This structure represents the RTX64 Server Console configuration parameters.
typedef RTFW_CONSOLE_CONFIGURATION_W RTFW_CONSOLE_CONFIGURATION;

// This structure represents the RTX64 Server Console configuration parameters.
typedef PRTFW_CONSOLE_CONFIGURATION_W PRTFW_CONSOLE_CONFIGURATION;
#else
// This structure represents the RTX64 Server Console configuration parameters.
typedef RTFW_CONSOLE_CONFIGURATION_A RTFW_CONSOLE_CONFIGURATION;

// This structure represents the RTX64 Server Console configuration parameters.
typedef PRTFW_CONSOLE_CONFIGURATION_A PRTFW_CONSOLE_CONFIGURATION;
#endif // UNICODE

// This function gets the RTX64 Server Console configuration parameters.
RTFWPROC bool RtfwGetConsoleConfigurationW(RTFW_CONSOLE_CONFIGURATION_W * pConsoleConfiguration);

// This function wets the RTX64 Server Console configuration parameters.
RTFWPROC bool RtfwSetConsoleConfigurationW(const RTFW_CONSOLE_CONFIGURATION_W * pConsoleConfiguration);

// This function gets the RTX64 Server Console configuration parameters.
RTFWPROC bool RtfwGetConsoleConfigurationA(RTFW_CONSOLE_CONFIGURATION_A * pConsoleConfiguration);

// This function wets the RTX64 Server Console configuration parameters.
RTFWPROC bool RtfwSetConsoleConfigurationA(const RTFW_CONSOLE_CONFIGURATION_A * pConsoleConfiguration);

#ifdef UNICODE
// This function gets the RTX64 Server Console configuration parameters.
# define RtfwGetConsoleConfiguration	RtfwGetConsoleConfigurationW

// This function sets the RTX64 Server Console configuration parameters.
# define RtfwSetConsoleConfiguration	RtfwSetConsoleConfigurationW
#else
// This function gets the RTX64 Server Console configuration parameters.
# define RtfwGetConsoleConfiguration	RtfwGetConsoleConfigurationA

// This function sets the RTX64 Server Console configuration parameters.
# define RtfwSetConsoleConfiguration	RtfwSetConsoleConfigurationA
#endif

// This enumeration represents the various states of the RTX64 subsystem.  See API
// RtfwGetSubsystemStatus.
typedef enum _RTFW_SUBSYSTEM_STATUS
{
    RTSS_TRANSITIONING = 0,  // The RTX64 subsystem is starting or stopping.
    RTSS_STOPPED,            // The RTX64 subsystem is stopped.
    RTSS_STARTED,            // The RTX64 subsystem is started.
    RTSS_NOT_CONFIGURED,     // The RTX64 subsystem has not been configured, so it cannot be started.
    RTSS_MAX
} RTFW_SUBSYSTEM_STATUS, * PRTFW_SUBSYSTEM_STATUS;

// This function returns the current status of the RTX64 subsystem.
RTFWPROC bool RtfwGetSubsystemStatus(RTFW_SUBSYSTEM_STATUS * pStatus);

// This function starts the RTX64 subsystem.  If the RTX64 TCP/IP stack is configured to start with the
// subsystem, it also starts.
RTFWPROC bool RtfwStartSubsystem();

// This function stops the RTX64 subsystem and the RTX64 TCP/IP stack (if it is running).
RTFWPROC bool RtfwStopSubsystem();

// This function starts the RTX64 TCP/IP stack.
RTFWPROC bool RtfwStartNetwork();

// This function stops the RTX64 TCP/IP stack.
RTFWPROC bool RtfwStopNetwork(unsigned int timeout);

// This function gets the RTX64 subsystem's Local Memory configuration parameters.
RTFWPROC bool RtfwGetLocalMemoryConfiguration(RTFW_LOCAL_MEMORY_CONFIGURATION * pLocalMemoryConfiguration);

// This function sets the RTX64 subsystem's Local Memory configuration parameters.
RTFWPROC bool RtfwSetLocalMemoryConfiguration(const RTFW_LOCAL_MEMORY_CONFIGURATION * pLocalMemoryConfiguration);

// This structure represents the RTX64 subsystem's watchdog configuration parameters.
typedef struct _RTFW_WATCHDOG_CONFIGURATION
{
	size_t Size;
	bool WatchdogEnabled;
	unsigned int WatchdogTimeout;
} RTFW_WATCHDOG_CONFIGURATION, * PRTFW_WATCHDOG_CONFIGURATION;

// This function gets the RTX64 subsystem's watchdog configuration parameters.
RTFWPROC bool RtfwGetWatchdogConfiguration(RTFW_WATCHDOG_CONFIGURATION * pWatchdogConfiguration);

// This function sets the RTX64 subsystem's watchdog configuration parameters.
RTFWPROC bool RtfwSetWatchdogConfiguration(const RTFW_WATCHDOG_CONFIGURATION * pWatchdogConfiguration);

// This function gets the RTX64 subsystem's search path configuration.
RTFWPROC bool RtfwGetSearchPathA(CHAR * searchPath, size_t * pCountCharacters);

// This function gets the RTX64 subsystem's search path configuration.
RTFWPROC bool RtfwGetSearchPathW(WCHAR * searchPath, size_t * pCountCharacters);

// This function sets the RTX64 subsystem's search path configuration.  If any pathname in the
// search path exceeds MAX_PATH - 9 characters (not counting the optional trailing '\'), this fails
// and sets the last error value to ERROR_INVALID_PARAMETER.
RTFWPROC bool RtfwSetSearchPathA(const CHAR * searchPath);

// This function sets the RTX64 subsystem's search path configuration.  If any pathname in the
// search path exceeds MAX_PATH - 9 characters (not counting the optional trailing '\'), this fails
// and sets the last error value to ERROR_INVALID_PARAMETER.
RTFWPROC bool RtfwSetSearchPathW(const WCHAR * searchPath);

#ifdef UNICODE
// This function gets the RTX64 subsystem's search path configuration.
# define RtfwGetSearchPath RtfwGetSearchPathW

// This function sets the RTX64 subsystem's search path configuration.  If any pathname in the
// search path exceeds MAX_PATH - 9 characters (not counting the optional trailing '\'), this fails
// and sets the last error value to ERROR_INVALID_PARAMETER.
# define RtfwSetSearchPath RtfwSetSearchPathW
#else
// This function gets the RTX64 subsystem's search path configuration.
# define RtfwGetSearchPath RtfwGetSearchPathA

// This function sets the RTX64 subsystem's search path configuration.  If any pathname in the
// search path exceeds MAX_PATH - 9 characters (not counting the optional trailing '\'), this fails
// and sets the last error value to ERROR_INVALID_PARAMETER.
# define RtfwSetSearchPath RtfwSetSearchPathA
#endif

// This enumeration represents the RTX64 subsystem's possible priority inversion protocols.
typedef enum _RTFW_PRIORITY_INVERSION_PROTOCOL
{
	RTPI_NONE = 0,
	RTPI_TIERED_DEMOTION,
	RTPI_MAX
} RTFW_PRIORITY_INVERSION_PROTOCOL, * PRTFW_PRIORITY_INVERSION_PROTOCOL;

// This function gets the RTX64 subsystem's priority inversion protocol.
RTFWPROC bool RtfwGetPriorityInversionProtocol(RTFW_PRIORITY_INVERSION_PROTOCOL * pProtocol);

// This function sets the RTX64 subsystem's priority inversion protocol.
RTFWPROC bool RtfwSetPriorityInversionProtocol(RTFW_PRIORITY_INVERSION_PROTOCOL protocol);

// This function gets the RTX64 subsystem's Windows idle detection configuration.
RTFWPROC bool RtfwGetWindowsIdleDetection(bool * pDisableIdleDetection);

// This function sets the RTX64 subsystem's Windows idle detection configuration.
RTFWPROC bool RtfwSetWindowsIdleDetection(bool disableIdleDetection);

// This function gets the RTX64 processor configuration.
RTFWPROC bool RtfwGetProcessorConfiguration(DWORD * pWindowsProcessorCount,
                                            DWORD * pRequestedRTSSProcessorCount);

// This function sets the RTX64 processor configuration.  This API requires administrator
// privileges, otherwise it fails.  After this API succeeds, Windows must be restarted
// before the RTX64 subsystem can be used.
RTFWPROC bool RtfwSetProcessorConfiguration(DWORD requestedWindowsProcessorCount,
                                            DWORD requestedRTSSProcessorCount,
                                            DWORD * pWarning);

// This enumeration represents the possible restart states of the RTX64 subsystem.  See API
// RtfwRestartRequired.
typedef enum _RTFW_RESTART_TYPE
{
    // No restart is required.
	RTRT_NONE = 0,

    // The RTX64 TCP/IP stack must be restarted for configuration changes to take effect.
    RTRT_NETWORK = 5,

    // The RTX64 subsystem must be restarted.
	RTRT_SUBSYSTEM = 10,

    // Windows must be restarted.
	RTRT_MACHINE = 20
} RTFW_RESTART_TYPE;

// This function returns an enumerator indicating whether the RTX64 TCP/IP stack, the
// RTX64 subsystem, or Windows needs to be restarted due to a configuration change.
RTFWPROC bool RtfwRestartRequired(RTFW_RESTART_TYPE * restartType, unsigned int reserved);

// This structure represents the configuration of the RTX64 TCP/IP stack.
typedef struct _RTFW_NETWORK_CONFIGURATION
{
	size_t Size;
	unsigned int Memory;
	bool Verbose;
	bool AutoStart;
	unsigned int IdealProcessor;
	unsigned int MaxSockets;
	unsigned int MaxARPEntries;
	unsigned int IPReassemblyTimeout;
	unsigned int MTU;
	unsigned int TickInterval;
	DWORD TimerIdealProcessor;
	unsigned int TimerPriority;
} RTFW_NETWORK_CONFIGURATION, * PRTFW_NETWORK_CONFIGURATION;

// This function gets the current status of the RTX64 TCP/IP stack.
RTFWPROC bool RtfwGetNetworkStatus(RTFW_SUBSYSTEM_STATUS * pStatus, unsigned int reserved);

// This function gets the current configuration of the RTX64 TCP/IP stack.
RTFWPROC bool RtfwGetNetworkConfiguration(RTFW_NETWORK_CONFIGURATION * pNetworkConfiguration, unsigned int reserved);

// This function sets the current configuration of the RTX64 TCP/IP stack.
RTFWPROC bool RtfwSetNetworkConfiguration(const RTFW_NETWORK_CONFIGURATION * pNetworkConfiguration, unsigned int reserved);

// These constants are used as array sizes for various strings in this header file.  These values include
// space for the terminating null character.
#define RTFW_IPV4ADDRESS_LENGTH 16
#define RTFW_MAX_INTERFACE_NAME_CHARS 14
#define RTFW_MAX_INTERFACE_INSTANCEID_CHARS 255
#define RTFW_MAX_INTERFACE_PCIBUSLOCATION_CHARS 12
#define RTFW_MAX_INTERFACE_IPV6ADDRESS_CHARS 46
#define RTFW_MAX_DEV_GUID_CHARS 40
#define RTFW_MAX_DEV_HARDWAREID_CHARS 256
#define RTFW_MAX_DEV_INSTANCEID_CHARS 256
#define RTFW_MAX_DEV_PCIBUSLOCATION_CHARS 100
#define RTFW_MAX_DEV_DESCRIPTION_CHARS 256

// This constant is the maximum number of IPv4 configurations per RTX64 network interface.
#define RTFW_MAX_IPS_PER_INTERFACE MAX_IPS_PER_INTERFACE

// This structure represents one IPv4 configuration of an RTX64 network interface.
typedef struct _RTFW_IPV4_CONFIGURATIONW
{
	WCHAR IPv4Address[RTFW_IPV4ADDRESS_LENGTH];
	WCHAR Netmask[RTFW_IPV4ADDRESS_LENGTH];
} RTFW_IPV4_CONFIGURATIONW, * PRTFW_IPV4_CONFIGURATIONW;

// This structure represents one IPv4 configuration of an RTX64 network interface.
typedef struct _RTFW_IPV4_CONFIGURATIONA
{
	CHAR IPv4Address[RTFW_IPV4ADDRESS_LENGTH];
	CHAR Netmask[RTFW_IPV4ADDRESS_LENGTH];
} RTFW_IPV4_CONFIGURATIONA, * PRTFW_IPV4_CONFIGURATIONA;

#ifdef UNICODE
// This structure represents one IPv4 configuration of an RTX64 network interface.
typedef RTFW_IPV4_CONFIGURATIONW RTFW_IPV4_CONFIGURATION;
#else
// This structure represents one IPv4 configuration of an RTX64 network interface.
typedef RTFW_IPV4_CONFIGURATIONA RTFW_IPV4_CONFIGURATION;
#endif

// This enumeration represents the possible interrupt types for devices owned by RTX64.
typedef enum _RTFW_INTERRUPT_TYPE
{
	RTIN_RESERVED = 0,
	RTIN_LINEBASED,
	RTIN_MSI,
	RTIN_MSIX,
	RTIN_MAX
} RTFW_INTERRUPT_TYPE, * PRTFW_INTERRUPT_TYPE;

// This structure represents the configuration parameters of an RTX64 network interface.
typedef struct _RTFW_NETWORK_INTERFACE_A
{
	size_t Size;
	CHAR FriendlyName[RTFW_MAX_INTERFACE_NAME_CHARS];
	CHAR Driver[MAX_PATH];
	bool Enabled;
	CHAR FilterDriver[MAX_PATH];
	bool FilterDriverEnabled;
	CHAR DeviceInstanceID[RTFW_MAX_INTERFACE_INSTANCEID_CHARS];
	DWORD ReceiveIdealProcessor;
	DWORD InterruptIdealProcessor;
	bool LinkStatus;
	unsigned int LinkStatusPriority;
	DWORD LinkStatusIdealProcessor;
	CHAR Gateway[RTFW_IPV4ADDRESS_LENGTH];
	CHAR PCIBusLocation[RTFW_MAX_INTERFACE_PCIBUSLOCATION_CHARS];
	unsigned int InterruptPriority;
	unsigned int MTU;
	unsigned int CountIPv4Configurations;
	RTFW_IPV4_CONFIGURATIONA IPv4Configurations[RTFW_MAX_IPS_PER_INTERFACE];
	CHAR IPv6Address[RTFW_MAX_INTERFACE_IPV6ADDRESS_CHARS];
	unsigned int IPv6Prefix;
	unsigned int ReceivePriority;
	RTFW_INTERRUPT_TYPE InterruptType;
	unsigned int NumberReceiveBuffers;
	unsigned int NumberTransmitBuffers;
} RTFW_NETWORK_INTERFACEA, * PRTFW_NETWORK_INTERFACEA;

// This structure represents the configuration parameters of an RTX64 network interface.
typedef struct _RTFW_NETWORK_INTERFACE_W
{
	size_t Size;
	WCHAR FriendlyName[RTFW_MAX_INTERFACE_NAME_CHARS];
	WCHAR Driver[MAX_PATH];
	bool Enabled;
	WCHAR FilterDriver[MAX_PATH];
	bool FilterDriverEnabled;
	WCHAR DeviceInstanceID[RTFW_MAX_INTERFACE_INSTANCEID_CHARS];
	DWORD ReceiveIdealProcessor;
	DWORD InterruptIdealProcessor;
	bool LinkStatus;
	unsigned int LinkStatusPriority;
	DWORD LinkStatusIdealProcessor;
	WCHAR Gateway[RTFW_IPV4ADDRESS_LENGTH];
	WCHAR PCIBusLocation[RTFW_MAX_INTERFACE_PCIBUSLOCATION_CHARS];
	unsigned int InterruptPriority;
	unsigned int MTU;
	unsigned int CountIPv4Configurations;
	RTFW_IPV4_CONFIGURATIONW IPv4Configurations[RTFW_MAX_IPS_PER_INTERFACE];
	WCHAR IPv6Address[RTFW_MAX_INTERFACE_IPV6ADDRESS_CHARS];
	unsigned int IPv6Prefix;
	unsigned int ReceivePriority;
	RTFW_INTERRUPT_TYPE InterruptType;
	unsigned int NumberReceiveBuffers;
	unsigned int NumberTransmitBuffers;
} RTFW_NETWORK_INTERFACEW, * PRTFW_NETWORK_INTERFACEW;

#ifdef UNICODE
// This structure represents the configuration parameters of an RTX64 network interface.
typedef RTFW_NETWORK_INTERFACEW RTFW_NETWORK_INTERFACE;
#else
// This structure represents the configuration parameters of an RTX64 network interface.
typedef RTFW_NETWORK_INTERFACEA RTFW_NETWORK_INTERFACE;
#endif

// This function creates a new RTX64 network interface.
RTFWPROC bool RtfwCreateNetworkInterfaceA(const RTFW_NETWORK_INTERFACEA * pInterface, unsigned int reserved);

// This function creates a new RTX64 network interface.
RTFWPROC bool RtfwCreateNetworkInterfaceW(const RTFW_NETWORK_INTERFACEW * pInterface, unsigned int reserved);

#ifdef UNICODE
// This function creates a new RTX64 network interface.
# define RtfwCreateNetworkInterface RtfwCreateNetworkInterfaceW
#else
// This function creates a new RTX64 network interface.
# define RtfwCreateNetworkInterface RtfwCreateNetworkInterfaceA
#endif

// This function modifies an RTX64 network interface.
RTFWPROC bool RtfwModifyNetworkInterfaceA(const CHAR * friendlyName,
                                          const RTFW_NETWORK_INTERFACEA * pInterface,
                                          unsigned int reserved);

// This function modifies an RTX64 network interface.
RTFWPROC bool RtfwModifyNetworkInterfaceW(const WCHAR * friendlyName,
                                          const RTFW_NETWORK_INTERFACEW * pInterface,
                                          unsigned int reserved);

#ifdef UNICODE
// This function modifies an RTX64 network interface.
# define RtfwModifyNetworkInterface RtfwModifyNetworkInterfaceW
#else
// This function modifies an RTX64 network interface.
# define RtfwModifyNetworkInterface RtfwModifyNetworkInterfaceA
#endif

// This function deletes an RTX64 network interface.
RTFWPROC bool RtfwDeleteNetworkInterfaceA(const CHAR * friendlyName, unsigned int reserved);

// This function deletes an RTX64 network interface.
RTFWPROC bool RtfwDeleteNetworkInterfaceW(const WCHAR * friendlyName, unsigned int reserved);

#ifdef UNICODE
// This function deletes an RTX64 network interface.
# define RtfwDeleteNetworkInterface RtfwDeleteNetworkInterfaceW
#else
// This function deletes an RTX64 network interface.
# define RtfwDeleteNetworkInterface RtfwDeleteNetworkInterfaceA
#endif

// This function gets the configuration parameters for the RTX64 network interface having the
// specified friendly name.
RTFWPROC bool RtfwGetNetworkInterfaceByNameA(const CHAR * friendlyName,
                                             RTFW_NETWORK_INTERFACEA * pInterface,
                                             unsigned int reserved);

// This function gets the configuration parameters for the RTX64 network interface having the
// specified friendly name.
RTFWPROC bool RtfwGetNetworkInterfaceByNameW(const WCHAR * friendlyName,
                                             RTFW_NETWORK_INTERFACEW * pInterface, 
                                             unsigned int reserved);

#ifdef UNICODE
// This function gets the configuration parameters for the RTX64 network interface having the
// specified friendly name.
# define RtfwGetNetworkInterfaceByName RtfwGetNetworkInterfaceByNameW
#else
// This function gets the configuration parameters for the RTX64 network interface having the
// specified friendly name.
# define RtfwGetNetworkInterfaceByName RtfwGetNetworkInterfaceByNameA
#endif

// This function gets the configuration parameters for all RTX64 network interfaces.
RTFWPROC bool RtfwGetAllNetworkInterfacesA(RTFW_NETWORK_INTERFACEA * pInterfaces,
                                           size_t * pCountNetworkInterfaces,
                                           unsigned int reserved);

// This function gets the configuration parameters for all RTX64 network interfaces.
RTFWPROC bool RtfwGetAllNetworkInterfacesW(RTFW_NETWORK_INTERFACEW * pInterfaces,
                                           size_t * pCountNetworkInterfaces,
                                           unsigned int reserved);

#ifdef UNICODE
// This function gets the configuration parameters for all RTX64 network interfaces.
# define RtfwGetAllNetworkInterfaces RtfwGetAllNetworkInterfacesW
#else
// This function gets the configuration parameters for all RTX64 network interfaces.
# define RtfwGetAllNetworkInterfaces RtfwGetAllNetworkInterfacesA
#endif

// This constant represents the first RTSS processor, regardless of its actual processor number.
#define RTFW_FIRST_RTSS_PROCESSOR 0

// This structure represents the properties of a device owned by RTX64.
typedef struct _RTFW_DEVICE_A
{
    size_t Size;
    char ClassGUID[RTFW_MAX_DEV_GUID_CHARS];
    char HardwareID[RTFW_MAX_DEV_HARDWAREID_CHARS];
    char InstanceID[RTFW_MAX_DEV_INSTANCEID_CHARS];
    char PCIBusLocation[RTFW_MAX_DEV_PCIBUSLOCATION_CHARS];
    char Description[RTFW_MAX_DEV_DESCRIPTION_CHARS];
} RTFW_DEVICEA, * PRTFW_DEVICEA;

// This structure represents the properties of a device owned by RTX64.
typedef struct _RTFW_DEVICE_W
{
    size_t Size;
    wchar_t ClassGUID[RTFW_MAX_DEV_GUID_CHARS];
    wchar_t HardwareID[RTFW_MAX_DEV_HARDWAREID_CHARS];
    wchar_t InstanceID[RTFW_MAX_DEV_INSTANCEID_CHARS];
    wchar_t PCIBusLocation[RTFW_MAX_DEV_PCIBUSLOCATION_CHARS];
    wchar_t Description[RTFW_MAX_DEV_DESCRIPTION_CHARS];
} RTFW_DEVICEW, * PRTFW_DEVICEW;

// This function gets the properties of all devices owned by RTX64.
RTFWPROC bool RtfwGetRTX64DevicesA(RTFW_DEVICEA * pRTX64Devices, size_t * pCountDevices);

// This function gets the properties of all devices owned by RTX64.
RTFWPROC bool RtfwGetRTX64DevicesW(RTFW_DEVICEW * pRTX64Devices, size_t * pCountDevices);

#ifdef UNICODE
// This function gets the properties of all devices owned by RTX64.
# define RtfwGetRTX64Devices RtfwGetRTX64DevicesW

// This structure represents the properties of a device owned by RTX64.
typedef RTFW_DEVICEW RTFW_DEVICE;
#else
// This function gets the properties of all devices owned by RTX64.
# define RtfwGetRTX64Devices RtfwGetRTX64DevicesA

// This structure represents the properties of a device owned by RTX64.
typedef RTFW_DEVICEA RTFW_DEVICE;
#endif

// This enumeration represents the state of the RTX64 Monitoring Framework.
typedef enum _RTFW_MONITOR_STATUS
{
    RTMS_DISABLED = 0,  // Monitoring is disabled (and thus also stopped).
    RTMS_ENABLED,       // Monitoring is enabled (but not started).
    RTMS_STARTED,       // Monitoring is started (and thus also enabled).
	RTMS_MAX			// Monitoring maximum value
} RTFW_MONITOR_STATUS, * PRTFW_MONITOR_STATUS;

// Returns the current status of the RTX64 Monitoring Framework.  If the RTX64 subsystem is stopped,
// the status is RTMS_DISABLED.
RTFWPROC bool RtfwGetMonitorStatus(RTFW_MONITOR_STATUS * pMonitorStatus);

// This structure represents the configuration of the RTX64 Monitoring Framework.  See APIs
// RtfwGetMonitorConfiguration and RtfwSetMonitorConfiguration.
typedef struct _RTFW_MONITOR_CONFIGURATIONA
{
    size_t Size;                       // Must be set to sizeof(RTFW_MONITOR_CONFIGURATION).
    uint64_t MTBSize;                  // The size (in megabytes) of the Monitor Transport Buffer.
    bool AutoEnable;                   // Whether or not to automatically enable monitoring when the subysstem starts.
    bool AutoStart;                    // Whether or not to automatically start monitoring when the subysstem starts.
    bool AutoRotateLogFiles;           // Whether or not to automatically rotate monitoring log files.
    uint64_t MaximumSessionSize;       // The maximum size (in megabytes) of a monitoring session folder.
    char SessionFolder[MAX_PATH];      // The absolute pathname of the folder that will hold monitoring session folders.
} RTFW_MONITOR_CONFIGURATIONA, * PRTFW_MONITOR_CONFIGURATIONA;

// This structure represents the configuration of the RTX64 Monitoring Framework.  See APIs
// RtfwGetMonitorConfiguration and RtfwSetMonitorConfiguration.
typedef struct _RTFW_MONITOR_CONFIGURATIONW
{
    size_t Size;                          // Must be set to sizeof(RTFW_MONITOR_CONFIGURATION).
    uint64_t MTBSize;                     // The size (in megabytes) of the Monitor Transport Buffer.
    bool AutoEnable;                      // Whether or not to automatically enable monitoring when the subysstem starts.
    bool AutoStart;                       // Whether or not to automatically start monitoring when the subysstem starts.
    bool AutoRotateLogFiles;              // Whether or not to automatically rotate monitoring log files.
    uint64_t MaximumSessionSize;          // The maximum size (in megabytes) of a monitoring session folder.
    wchar_t SessionFolder[MAX_PATH];      // The absolute pathname of the folder that will hold monitoring session folders.
} RTFW_MONITOR_CONFIGURATIONW, * PRTFW_MONITOR_CONFIGURATIONW;

// Returns the RTX64 Monitoring Framework configuration.
RTFWPROC bool RtfwGetMonitorConfigurationA(RTFW_MONITOR_CONFIGURATIONA * pMonitorConfiguration);

// Returns the RTX64 Monitoring Framework configuration.
RTFWPROC bool RtfwGetMonitorConfigurationW(RTFW_MONITOR_CONFIGURATIONW * pMonitorConfiguration);

// Sets the RTX64 Monitoring Framework configuration.
RTFWPROC bool RtfwSetMonitorConfigurationA(const RTFW_MONITOR_CONFIGURATIONA * pMonitorConfiguration);

// Sets the RTX64 Monitoring Framework configuration.
RTFWPROC bool RtfwSetMonitorConfigurationW(const RTFW_MONITOR_CONFIGURATIONW * pMonitorConfiguration);

#ifdef UNICODE
// Gets the RTX64 Monitoring Framework configuration.
# define RtfwGetMonitorConfiguration RtfwGetMonitorConfigurationW

// Sets the RTX64 Monitoring Framework configuration.
# define RtfwSetMonitorConfiguration RtfwSetMonitorConfigurationW

// This structure represents the configuration of the RTX64 Monitoring Framework.  See APIs
// RtfwGetMonitorConfiguration and RtfwSetMonitorConfiguration.
typedef RTFW_MONITOR_CONFIGURATIONW RTFW_MONITOR_CONFIGURATION;
#else
// Gets the RTX64 Monitoring Framework configuration.
# define RtfwGetMonitorConfiguration RtfwGetMonitorConfigurationA

// Sets the RTX64 Monitoring Framework configuration.
# define RtfwSetMonitorConfiguration RtfwSetMonitorConfigurationA

// This structure represents the configuration of the RTX64 Monitoring Framework.  See APIs
// RtfwGetMonitorConfiguration and RtfwSetMonitorConfiguration.
typedef RTFW_MONITOR_CONFIGURATIONA  RTFW_MONITOR_CONFIGURATION;
#endif

// This function returns the current enabled/disabled state of all monitoring events in
// the Persistent Event Set.  This does not affect the Transient Event set.
RTFWPROC bool RtfwGetMonitorEventStates(bool * pEventStates, size_t * pCountElements);

// This function enables/disables monitoring events in the Persistent Event Set.
// This does not affect the Transient Event set.
RTFWPROC bool RtfwSetMonitorEventStates(const bool * pEventStates, size_t countElements);

// This function enables/disables all monitoring events in the Persistent Event Set.
// This does not affect the Transient Event set.  If parameter enableAll is true, all
// monitoring events are enabled, otherwise all monitoring events are disabled (except
// MF_EVENT_KIND_MARKER and MF_EVENT_KIND_DATALOST, which can never be disabled).
RTFWPROC bool RtfwSetAllMonitorEventStates(bool enableAll);

// This function deletes all monitoring session folders located in the session parent
// folder configured by RtfwSetMonitorConfiguration.
RTFWPROC bool RtfwDeleteAllMonitorSessions();

// This function deletes the specified monitoring session folder.  Parameter sessionFolder must
// be a simple folder name (containing no '\' characters) that is the name of a monitoring
// session folder in the session parent folder configured by RtfwSetMonitorConfiguration.
RTFWPROC bool RtfwDeleteMonitorSessionA(const char * sessionFolder);

// This function deletes the specified monitoring session folder.  Parameter sessionFolder must
// be a simple folder name (containing no '\' characters) that is the name of a monitoring
// session folder in the session parent folder configured by RtfwSetMonitorConfiguration.
RTFWPROC bool RtfwDeleteMonitorSessionW(const wchar_t * sessionFolder);

#ifdef UNICODE
// This function deletes the specified monitoring session folder.  Parameter sessionFolder must
// be a simple folder name (containing no '\' characters) that is the name of a monitoring
// session folder in the session parent folder configured by RtfwSetMonitorConfiguration.
# define RtfwDeleteMonitorSession RtfwDeleteMonitorSessionW
#else
// This function deletes the specified monitoring session folder.  Parameter sessionFolder must
// be a simple folder name (containing no '\' characters) that is the name of a monitoring
// session folder in the session parent folder configured by RtfwSetMonitorConfiguration.
# define RtfwDeleteMonitorSession RtfwDeleteMonitorSessionA
#endif

// This function enables the RTX64 Monitoring Framework.
RTFWPROC bool RtfwEnableMonitoring();

// This function disables the RTX64 Monitoring Framework.
RTFWPROC bool RtfwDisableMonitoring();

// This function starts the RTX64 Monitoring Framework.  It must be enabled with
// RtfwEnableMonitoring before it is started.
RTFWPROC bool RtfwStartMonitoring();

// This function stops the RTX64 Monitoring Framework.
RTFWPROC bool RtfwStopMonitoring();

#ifdef __cplusplus
}  // End extern "C" block.
#endif
