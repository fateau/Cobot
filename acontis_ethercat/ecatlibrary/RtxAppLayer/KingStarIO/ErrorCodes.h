// Copyright (c) 2015 - 2018 IntervalZero, Inc.  All rights reserved.
//
// This file defines macros that represent custom Win32 error codes that RTX64 real-time APIs can
// return.  Custom error codes all have bit 29 set.  See:
//
//    https://msdn.microsoft.com/en-us/library/ms680627.aspx
//
// which says:
//
//    Bit 29 is reserved for application-defined error codes; no system error code has this bit set.
//    If you are defining an error code for your application, set this bit to indicate that the
//    error code has been defined by your application and to ensure that your error code does not
//    conflict with any system-defined error codes.
//
// RTX64 real-time APIs can still return normal Win32 error codes, such as ERROR_FILE_NOT_FOUND (2).
// These error codes extend the range of possible error codes to include codes specific to RTX64.

#pragma once

// Custom error codes must have bit 29 set.  Microsoft guarantees that no regular Win32 error code
// will ever have bin 29 set.  IMPORTANT: Do not change the name of this macro!
#define RT_CUSTOM_ERROR(VALUE)  ((VALUE) | (1 << 29))

// This macro has been deprecated.
// Custom error codes must have bit 29 set.  Microsoft guarantees that no regular Win32 error code
// will ever have bin 29 set.  IMPORTANT: Do not change the name of this macro!
#define RTX64_CUSTOM_ERROR(VALUE)  ((VALUE) | (1 << 29))


// Description: No RTX64 runtime license is available. Error Deprecated. 
// This error code has been deprecated.
#define RTX64_NO_RUNTIME_LICENSE                        RTX64_CUSTOM_ERROR(1)

// Description: The RTX64 runtime license has expired. 
// This error code has been deprecated.
#define RTX64_EXPIRED_RUNTIME_LICENSE                   RTX64_CUSTOM_ERROR(2)

// Description: No RTX64 SDK license was available when this application (or an RTDLL on which it
// depends) was built. 
// This error code has been deprecated.
#define RTX64_MODULE_NO_SDK_LICENSE                     RTX64_CUSTOM_ERROR(3)

// Description: This application (or an RTDLL on which it depends) was built with an evaluation
// RTX64 SDK license. It cannot be executed when the RTX64 runtime license is a retail license. 
// This error code has been deprecated.
#define RTX64_MODULE_EVAL_SDK_LICENSE                   RTX64_CUSTOM_ERROR(4)

// Description: This application (or an RTDLL on which it depends) is missing important RTX64
// license information.
// This error code has been deprecated.
#define RTX64_MODULE_LICENSE_INFO_MISSING               RTX64_CUSTOM_ERROR(5)

// Description: This application (or an RTDLL on which it depends) contains corrupted license
// information.
// This error code has been deprecated. 
#define RTX64_MODULE_LICENSE_INFO_CORRUPT               RTX64_CUSTOM_ERROR(6)

// Description: One or more object files in this application (or an RTDLL on which it depends) were
// built using an RTX64 SDK version that is incompatible with the current RTX64 runtime.
// This error code has been deprecated.
#define RTX64_MODULE_UNSUPPORTED_SDK                    RTX64_CUSTOM_ERROR(7)

// Description: The system clock appears to have been set to a time in the past. This prevents RTX64
// from properly validating licenses.  Please set the system time to the correct time.
// This error code has been deprecated.
#define RTX64_CLOCK_REWIND                              RTX64_CUSTOM_ERROR(8)

// Description: Length parameter is invalid
// This error code has been deprecated.
#define RTX64_GETLICENSES_INVALID_LENGTH_PARAMETER      RTX64_CUSTOM_ERROR(9)

// Description: Results buffer passed length is too small
// This error code has been deprecated.
#define RTX64_GETLICENSES_RESULTS_BUFFER_TOO_SMALL      RTX64_CUSTOM_ERROR(10)

// Description: Unable to access system registry
// This error code has been deprecated.
#define RTX64_GETLICENSES_REGISTRY_ACCESS_FAILURE       RTX64_CUSTOM_ERROR(11)

// Description: Expected registry value does not exist
// This error code has been deprecated.
#define RTX64_GETLICENSES_REGISTRY_VALUE_ACCESS_FAILURE RTX64_CUSTOM_ERROR(12)

// Description: License info data in registry does not match to its hash value stored at last update
// This error code has been deprecated.
#define RTX64_GETLICENSES_HASHSUM_DOES_NOT_MATCH        RTX64_CUSTOM_ERROR(13)

// Description: Version of license info value in registry does not match to the one of the currently
// installed system.
// This error code has been deprecated.
#define RTX64_GETLICENSES_VERSION_DOES_NOT_MATCH        RTX64_CUSTOM_ERROR(14)

// Description: Hashnum cannot be computed 
// This error code has been deprecated.
#define RTX64_GETLICENSES_HASHSUM_COMPUTE_FAILED        RTX64_CUSTOM_ERROR(15)

// Description: Clock rewind detected
// This error code has been deprecated.
#define RTX64_GETLICENSES_CLOCK_REWIND_DETECTED         RTX64_CUSTOM_ERROR(16)

// Description: License not found
// This error code has been deprecated.
#define RTX64_GETLICENSES_NOT_FOUND                     RTX64_CUSTOM_ERROR(17)

// Description: Internal Error
// This error code has been deprecated.
#define RTX64_GETLICENSES_INTERNAL_ERROR                RTX64_CUSTOM_ERROR(18)

// Description: Unknown Error
// This error code has been deprecated.
#define RTX64_GETLICENSES_UNKNOWN_ERROR                 RTX64_CUSTOM_ERROR(19)

// Description: Call success
// This error code has been deprecated.
#define RTX64_GETLICENSES_SUCCESS                       RTX64_CUSTOM_ERROR(20)

// Description: Could not access registry key to read data
// This error code has been deprecated.
#define RTX64_GETDONGLES_REG_KEY_ACCESS_ERROR           RTX64_CUSTOM_ERROR(21)

// Description: Could not access registry key to read data.
// This error code has been deprecated.
#define RTX64_GETDONGLES_REG_VALUE_READ_ERROR           RTX64_CUSTOM_ERROR(22)

// Description: Invalid paraemters were provided.
// This error code has been deprecated.
#define RTX64_GETDONGLES_INVALID_PARAMETERS             RTX64_CUSTOM_ERROR(23)

// Description: Invalid parameters were provided.
// This error code has been deprecated.
#define RTX64_GETDONGLES_BUFFER_TOO_SMALL               RTX64_CUSTOM_ERROR(24)

// Description: Failed to allocate memory.
// This error code has been deprecated.
#define RTX64_GETDONGLES_MEM_ALLOC_FAILED               RTX64_CUSTOM_ERROR(25)

// Description: The number of configured RTSS cores is greater than number of licensed cores.
// This error code has been deprecated.
#define RTX64_TOO_MANY_RTSS_CORES_CONFIGURED            RTX64_CUSTOM_ERROR(26)

// ================================================================================
// The above error codes are deprecated.  Do not use them in new code.
// ================================================================================

// IMPORTANT: Do not change or move the comment on the next line.
// ERROR-CODES-START-HERE

// Description: No RTX64 runtime license is available. 
#define RT_ERROR_NO_RUNTIME_LICENSE                         RT_CUSTOM_ERROR(1)

// Description: The RTX64 runtime license has expired.
#define RT_ERROR_EXPIRED_RUNTIME_LICENSE                    RT_CUSTOM_ERROR(2)

// Description: No valid license was found for the given product feature code.
#define RT_ERROR_NO_LICENSE                                 RT_CUSTOM_ERROR(3)

// Description: No RTX64 SDK license was available when this application (or an RTDLL on which it
// depends) was built. You may be able to resolve this error by rebuilding the application.
#define RT_ERROR_MODULE_NO_SDK_LICENSE                      RT_CUSTOM_ERROR(4)

// Description: This application (or an RTDLL on which it depends) was built with an evaluation
// RTX64 SDK license. It cannot be executed when the RTX64 runtime license is a retail license. 
// You may be able to resolve this error by rebuilding the application.
#define RT_ERROR_MODULE_EVAL_SDK_LICENSE                    RT_CUSTOM_ERROR(5)

// Description: This application (or an RTDLL on which it depends) is missing important RTX64
// license information. You may be able to resolve this error by rebuilding the application.
#define RT_ERROR_MODULE_LICENSE_INFO_MISSING                RT_CUSTOM_ERROR(6)

// Description: This application (or an RTDLL on which it depends) contains corrupted license
// information. You may be able to resolve this error by rebuilding the application.
#define RT_ERROR_MODULE_LICENSE_INFO_CORRUPT                RT_CUSTOM_ERROR(7)

// Description: One or more object files in this application (or an RTDLL on which it depends) were
// built using an RTX64 SDK version that is incompatible with the current RTX64 runtime.
// You may be able to resolve this error by rebuilding the application.
#define RT_ERROR_MODULE_UNSUPPORTED_SDK                     RT_CUSTOM_ERROR(8)

// Description: License data is corrupt.
#define RT_ERROR_CORRUPT_LICENSE_DATA                       RT_CUSTOM_ERROR(9)

// Description: RTX64 Subsystem was unable to communicate with required services.
#define RT_ERROR_SERVICE_ACCESS_FAILURE                     RT_CUSTOM_ERROR(10)

// Description: Internal error 1.  Please contact IntervalZero support for more information. 
#define RT_ERROR_INTERNAL_1                                 RT_CUSTOM_ERROR(11)

// Description: Internal error 2.  Please contact IntervalZero support for more information. 
#define RT_ERROR_INTERNAL_2                                 RT_CUSTOM_ERROR(12)

// Description: Internal error 3.  Please contact IntervalZero support for more information. 
#define RT_ERROR_INTERNAL_3                                 RT_CUSTOM_ERROR(13)

// Description: Internal error 4.  Please contact IntervalZero support for more information. 
#define RT_ERROR_INTERNAL_4                                 RT_CUSTOM_ERROR(14)

// Description: Internal error 5.  Please contact IntervalZero support for more information. 
#define RT_ERROR_INTERNAL_5                                 RT_CUSTOM_ERROR(15)

// Description: Internal error 6.  Please contact IntervalZero support for more information. 
#define RT_ERROR_INTERNAL_6                                 RT_CUSTOM_ERROR(16)

// Description: Internal error 7.  Please contact IntervalZero support for more information. 
#define RT_ERROR_INTERNAL_7                                 RT_CUSTOM_ERROR(17)

// Description: Internal error 8.  Please contact IntervalZero support for more information. 
#define RT_ERROR_INTERNAL_8                                 RT_CUSTOM_ERROR(18)

// Description: Internal error 9.  Please contact IntervalZero support for more information. 
#define RT_ERROR_INTERNAL_9                                 RT_CUSTOM_ERROR(19)

// Description: Internal error 10.  Please contact IntervalZero support for more information. 
#define RT_ERROR_INTERNAL_10                                RT_CUSTOM_ERROR(20)

// Description: Internal error 11.  Please contact IntervalZero support for more information. 
#define RT_ERROR_INTERNAL_11                                RT_CUSTOM_ERROR(21)

// Description: Internal error 12.  Please contact IntervalZero support for more information. 
#define RT_ERROR_INTERNAL_12                                RT_CUSTOM_ERROR(22)

// Description: Internal error 13.  Please contact IntervalZero support for more information. 
#define RT_ERROR_INTERNAL_13                                RT_CUSTOM_ERROR(23)

// Description: Attempt to load a library that is not an RTDLL. RTSS applications and RTDLL
// libraries cannot link implicitly or explicitly to Windows dynamic link libraries (DLLs). Please
// make sure your project's linker inputs do not contain references to kernel32.lib, user32.lib, or
// any other Windows DLL import library.
#define RT_ERROR_INVALID_BINARY_FORMAT                      RT_CUSTOM_ERROR(24)

// Description: The RTX64 subsystem needs to be restarted for a configuration change to take effect.
#define RT_ERROR_RESTART_SUBSYSTEM                          RT_CUSTOM_ERROR(25)

// Description: The RTX64 subsystem is not configured, but an operation was attempted that requires
// it to be configured.
#define RT_ERROR_SUBSYSTEM_NOT_CONFIGURED                   RT_CUSTOM_ERROR(26)

// Description: A structure passed to an API is too small.
#define RT_ERROR_STRUCTURE_TOO_SMALL                        RT_CUSTOM_ERROR(27)

// Description: Internal error 14.  Please contact IntervalZero support for more information. 
#define RT_ERROR_INTERNAL_14                                RT_CUSTOM_ERROR(28)

// Description: The RTX64 TCP/IP stack has no license, but an operation was attempted that requires
// it to be licensed.
#define RT_ERROR_NO_STACK_LICENSE                           RT_CUSTOM_ERROR(29)

// Description: An operation failed to acquire the mutex that serializes starting/stopping the RTX64
// subsystem.
#define RT_ERROR_SUBSYSTEM_STARTSTOP_LOCK_FAILURE           RT_CUSTOM_ERROR(30)

// Description: A timeout occurred while attempting to start the RTX64 subsystem.
#define RT_ERROR_TIMEOUT_STARTING_SUBSYSTEM                 RT_CUSTOM_ERROR(31)

// Description: A timeout occurred while attempting to start the RTX64 subsystem.
#define RT_ERROR_TIMEOUT_STOPPING_SUBSYSTEM                 RT_CUSTOM_ERROR(32)

// Description: A timeout occurred while attempting to start the RTX64 TCP/IP stack.
#define RT_ERROR_TIMEOUT_STARTING_NETWORK                   RT_CUSTOM_ERROR(33)

// Description: Internal error 15.  Please contact IntervalZero support for more information. 
#define RT_ERROR_INTERNAL_15                                RT_CUSTOM_ERROR(34)

// Description: Internal error 16.  Please contact IntervalZero support for more information. 
#define RT_ERROR_INTERNAL_16                                RT_CUSTOM_ERROR(35)

// Description: The RTX64 subsystem is not started but an operation was attempted that requires it
// to be started.
#define RT_ERROR_SUBSYSTEM_NOT_STARTED                      RT_CUSTOM_ERROR(36)

// Description: The subsystem is not stopped but an operation was attempted that requires it to be
// stopped.
#define RT_ERROR_SUBSYSTEM_NOT_STOPPED                      RT_CUSTOM_ERROR(37)

// Description: The subsystem cannot be stopped, because one or more real-time processes are
// running.
#define RT_ERROR_RTSS_PROCESSES_RUNNING                     RT_CUSTOM_ERROR(38)

// Description: The subsystem cannot be stopped, because one or more real-time proxy processes are
// running.
#define RT_ERROR_PROXY_PROCESSES_RUNNING                    RT_CUSTOM_ERROR(39)

// Description: An operation failed to acquire the mutex that serializes starting/stopping the RTX64
// TCP/IP stack.
#define RT_ERROR_NETWORK_STARTSTOP_LOCK_FAILURE             RT_CUSTOM_ERROR(40)

// Description: The RTX64 TCP/IP stack is not started but an operation was attempted that requires
// it to be started.
#define RT_ERROR_NETWORK_NOT_STARTED                        RT_CUSTOM_ERROR(41)

// Description: The RTX64 TCP/IP stack is not stopped but an operation was attempted that requires
// it to be stopped.
#define RT_ERROR_NETWORK_NOT_STOPPED                        RT_CUSTOM_ERROR(42)

// Description: Internal error 17.  Please contact IntervalZero support for more information.
#define RT_ERROR_INTERNAL_17                                RT_CUSTOM_ERROR(43)

// Description: An operation was attempted that requires the RTX64 TCP/IP stack to be installed, but
// it is not installed.
#define RT_ERROR_NETWORK_NOT_INSTALLED                      RT_CUSTOM_ERROR(44)

// Description: Internal error 18.  Please contact IntervalZero support for more information.
#define RT_ERROR_INTERNAL_18                                RT_CUSTOM_ERROR(45)

// Description: The RTX64 TCP/IP stack cannot be stopped, because one or more stack client processes
// are still running.
#define RT_ERROR_NETWORK_CLIENTS_EXIST                      RT_CUSTOM_ERROR(46)

// Description: The subsystem is bus starting or stopping.
#define RT_ERROR_SUBSYSTEM_START_STOP_IN_PROGRESS           RT_CUSTOM_ERROR(47)

// Description: The subsystem is bus starting or stopping.
#define RT_ERROR_RESTART_NETWORK                            RT_CUSTOM_ERROR(48)

// Description: There are no interface slots left. The maximum number of interfaces is 99.
#define RT_ERROR_TOO_MANY_NETWORK_INTERFACES                RT_CUSTOM_ERROR(49)

// Description: Monitoring is already enabled.
#define RT_ERROR_MONITORING_ALREADY_ENABLED                 RT_CUSTOM_ERROR(50)

// Description: Monitoring is already disabled.
#define RT_ERROR_MONITORING_ALREADY_DISABLED                RT_CUSTOM_ERROR(51)

// Description: Monitoring is already started.
#define RT_ERROR_MONITORING_ALREADY_STARTED                 RT_CUSTOM_ERROR(52)

// Description: Monitoring is already stopped.
#define RT_ERROR_MONITORING_ALREADY_STOPPED                 RT_CUSTOM_ERROR(53)

// Description: Monitoring cannot be disabled, because it is started.
#define RT_ERROR_MONITORING_IS_STARTED                      RT_CUSTOM_ERROR(54)

// Description: Monitoring cannot be disabled, because it is started.
#define RT_ERROR_MONITORING_NOT_ENABLED                     RT_CUSTOM_ERROR(55)

// Description: Windows needs to be restarted.
#define RT_ERROR_RESTART_WINDOWS                            RT_CUSTOM_ERROR(56)

// Description: A timeout occurred while attempting to stop the RTX64 TCP/IP stack.
#define RT_ERROR_NETWORK_STOP_TIMEOUT                       RT_CUSTOM_ERROR(57)

// Description: Internal error 19.  Please contact IntervalZero support for more information.
#define RT_ERROR_INTERNAL_19                                RT_CUSTOM_ERROR(58)

// Description: Internal error 20.  Please contact IntervalZero support for more information.
#define RT_ERROR_INTERNAL_20                                RT_CUSTOM_ERROR(59)

// Description: Internal error 21.  Please contact IntervalZero support for more information.
#define RT_ERROR_INTERNAL_21                                RT_CUSTOM_ERROR(60)

// Description: An API succeeded but needed to modify configured processor numbers to
// prevent invalid process numbers.
#define RT_WARNING_INCONSISTENT_PROCESSOR_CONFIGURATION     RT_CUSTOM_ERROR(61)

// Description: An API succeeded but needed to modify configured timers or timeout values to
// maintain RTX64 subsystem consistency.
#define RT_WARNING_TIMER_VALUES_ROUNDED                     RT_CUSTOM_ERROR(62)

// Description: Internal error 22.  Please contact IntervalZero support for more information.
#define RT_ERROR_INTERNAL_22                                RT_CUSTOM_ERROR(63)

// Description: Internal error 23.  Please contact IntervalZero support for more information.
#define RT_ERROR_INTERNAL_23                                RT_CUSTOM_ERROR(64)

// Description: Internal error 24.  Please contact IntervalZero support for more information.
#define RT_ERROR_INTERNAL_24                                RT_CUSTOM_ERROR(65)

// Description: Network interface configuration friendly name is invalid.
#define RT_ERROR_FRIENDLY_NAME_INVALID                      RT_CUSTOM_ERROR(66)

// Description: Network interface configuration driver name is invalid.
#define RT_ERROR_DRIVER_INVALID                             RT_CUSTOM_ERROR(67)

// Description: Network interface configuration filter driver name is invalid.
#define RT_ERROR_FILTER_DRIVER_INVALID                      RT_CUSTOM_ERROR(68)

// Description: Network interface configuration instance ID is invalid.
#define RT_ERROR_DEVICE_INSTANCE_ID_INVALID                 RT_CUSTOM_ERROR(69)

// Description: Network interface configuration gateway is invalid.
#define RT_ERROR_GATEWAY_INVALID                            RT_CUSTOM_ERROR(70)

// Description: Network interface configuration PCI BUS location is invalid.
#define RT_ERROR_PCI_BUS_LOCATION_INVALID                   RT_CUSTOM_ERROR(71)

// Description: Network interface configuration IPV6 address is invalid.
#define RT_ERROR_IPV6_ADDRESS_INVALID                       RT_CUSTOM_ERROR(72)

// Description: Network interface configuration number of IPv4 configurations is too small.
#define RT_ERROR_NUMBER_OF_IPV4_CONFIGURATIONS_TOO_SMALL    RT_CUSTOM_ERROR(73)

// Description: Network interface configuration number of IPv4 configurations is too high.
#define RT_ERROR_NUMBER_OF_IPV4_CONFIGURATIONS_TOO_HIGH     RT_CUSTOM_ERROR(74)

// Description: Network interface configuration IPv4 address is invalid.
#define RT_ERROR_IPV4_ADDRESS_INVALID                       RT_CUSTOM_ERROR(75)

// Description: Network interface configuration netmask is invalid.
#define RT_ERROR_NETMASK_INVALID                            RT_CUSTOM_ERROR(76)

// Description: Network interface configuration IPv6 prefix is invalid.
#define RT_ERROR_IPV6_PREFIX_INVALID                        RT_CUSTOM_ERROR(77)

// Description: Network interface configuration IPv6 interrupt priority is invalid.
#define RT_ERROR_INTERRUPT_PRIORITY_INVALID                 RT_CUSTOM_ERROR(78)

// Description: Network interface configuration MTU is invalid.
#define RT_ERROR_MTU_INVALID                                RT_CUSTOM_ERROR(79)

// Description: Network interface configuration ReceiveIdealProcessor is invalid.
#define RT_ERROR_RECEIVEIDEALPROCESSOR_INVALID              RT_CUSTOM_ERROR(80)

// Description: Network interface configuration InterruptIdealProcessor is invalid.
#define RT_ERROR_INTERRUPTIDEALPROCESSOR_INVALID            RT_CUSTOM_ERROR(81)

// Description: Network interface configuration LinkStatusIdealProcessor is invalid.
#define RT_ERROR_LINKSTATUSIDEALPROCESSOR_INVALID           RT_CUSTOM_ERROR(82)

// Description: Network interface configuration link status priority is invalid.
#define RT_ERROR_LINK_STATUS_PRIORITY_INVALID               RT_CUSTOM_ERROR(83)

// Description: Network interface configuration receive priority is invalid.
#define RT_ERROR_RECEIVE_PRIORITY_INVALID                   RT_CUSTOM_ERROR(84)

// Description: Network interface configuration interrupt type is invalid.
#define RT_ERROR_INTERRUPT_TYPE_INVALID                     RT_CUSTOM_ERROR(85)

// Description: Network interface configuration number of receive buffers is invalid.
#define RT_ERROR_NUMBER_OF_RECEIVE_BUFFERS_INVALID          RT_CUSTOM_ERROR(86)

// Description: Network interface configuration number of transmit buffers is invalid.
#define RT_ERROR_NUMBER_OF_TRANSMIT_BUFFERS_INVALID         RT_CUSTOM_ERROR(87)

// Description: Internal error 25.  Please contact IntervalZero support for more information.
#define RT_ERROR_INTERNAL_25                                RT_CUSTOM_ERROR(88)

// Description: Network interface configuration link status property is invalid.
#define RT_ERROR_LINK_STATUS_INVALID						RT_CUSTOM_ERROR(89)

// Description: A folder was specified more than once in the search path.
#define RT_ERROR_DUPLICATE_SEARCH_PATH						RT_CUSTOM_ERROR(90)

// Description: The specified interrupt type is incorrect.
#define RT_ERROR_INCORRECT_INTERRUPT_TYPE					RT_CUSTOM_ERROR(91)

// Description: Failed to attach to the interrupt.
#define RT_ERROR_FAILED_TO_ATTACH_INTERRUPT					RT_CUSTOM_ERROR(92)

// Description: Error translating the PCI bus address. 
#define RT_ERROR_TRANSLATING_PCI_BUS_ADDRESS				RT_CUSTOM_ERROR(93)

// Description: Error mapping memory registers.
#define RT_ERROR_MAPPING_REGISTERS							RT_CUSTOM_ERROR(94)

// Description: The adapter’s MAC address does not match the configuration.
#define RT_ERROR_MAC_ADDRESS_MISMATCH						RT_CUSTOM_ERROR(95)

// Description: Cannot start a new instance of the NAL since it is already loaded.
#define RT_NAL_PROCESS_ALREADY_LOADED						RT_CUSTOM_ERROR(96)

// Description: The specified interrupt priority is invalid.
#define RT_INVALID_INTERRUPT_PRIORITY						RT_CUSTOM_ERROR(97)

