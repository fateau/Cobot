#if defined UNDER_RTSS && defined _AMD64_
/*
*              ZZZZZZZZ    Copyright (c) 2017 IntervalZero, Inc.  All rights reserved.
*            ZZZZ   ZZZZZ
*         O ZZZZ      ZZZ O      Module Name:
*       OII ZZZ        ZZ IOII
*     OIII  ZZZZ      ZZZ  IIII     coe64.h
*    IOIII   ZZZZZ  ZZZZZ   IOII
*    III      ZZZZZZZZZ      IOI   Abstract:
*   III                       III		
*  IIIO     ZZZZZZZZZZZZZ     OIII       
*  IIIO     ZZZZZZZZZZZZZZ     III       
*  III      ZZZ                III   
*  OIII     ZZZ  ZZZZ          III		 
*   IIO     ZZZZZZZZZ         IOIO		 
*   IIII    ZZZZZZ     ZZ     III		
*    IIII   ZZZZ     ZZZZ    III     
*     I0II         ZZZZZZ  OIIOI  Author:
*      IOIO      ZZZZ  ZZ IIII         InterValZero
*        OO       Z    ZZ III
*                      ZZ      Environment:
*           ZZZZZZZZZZZZZ
*           ZZZZZZZZZZZZZ        x64 and  King-Star Motion application 
*                         
*     Revision History:
*		No Revision History
*/
#ifndef _COEKS_H
#define _COEKS_H 0


#include <windows.h>
#include <rtapi.h>

#ifdef UNDER_RTSS
#include <rtssapi.h>
#endif // UNDER_RTSS

#ifdef __cplusplus
extern "C" {
#endif

#ifdef KINGSTAR_ETHERCAT
#define coe64_API __declspec(dllexport)
#else
#define coe64_API __declspec(dllimport)
#endif // KINGSTAR_ETHERCAT


/*-TYPEDEFS------------------------------------------------------------------*/
typedef struct
{
	char				Name[64];
	unsigned int	    dwVendorId;
	unsigned int		dwProductCode;
	unsigned int		dwRevisionNumber;
	unsigned int		dwSerialNumber;
	unsigned int		dwSlaveID;
	unsigned short      wAliasAddress;
	unsigned short      wPhysAddress;
	unsigned short		wSlaveState;
	unsigned char*		pPdoInp;
	unsigned char*		pPdoOut;
	unsigned int		dwInpLength;
	unsigned int		dwInpOff;
	unsigned int		dwOutLength;
	unsigned int		dwOutOff;
	unsigned int		dwPitch;
	unsigned int		dwEncoderRes;
	unsigned int		dwCycleTime;
	unsigned int		dwInputVariables;
	unsigned int		dwOutputVariables;
	unsigned int		dwVariableIndexOffset;
} AXIS_ECAT;

typedef struct
{
	char				Name[64];
	unsigned int		dwVendorId;
	unsigned int		dwProductCode;
	unsigned int		dwRevisionNumber;
	unsigned int		dwSerialNumber;
	unsigned int		dwSlaveID;
	unsigned short      wAliasAddress;
	unsigned short      wPhysAddress;
	unsigned short		wSlaveState;
	long				bUnit; // 0 Bit (Use readbits), 1 Word (use readword)
	unsigned int		dwInpLength;
	unsigned char*		pInp;
	unsigned int		dwInpOff;
	unsigned int		dwOutLength;
	unsigned char*		pOut;
	unsigned int		dwOutOff;
	unsigned int		dwCycleTime;
} IO_ECAT;

#define NOTIFY_GENERIC							0x00000000
#define NOTIFY_ERROR							0x00010000
#define NOTIFY_OTHER							0x00100000
typedef enum _T_Notification_Code
{
	Notification_ETH_LINK_CONNECTED         = (NOTIFY_GENERIC | 101),     /* Ethernet link (cable) connected */
	Error_ETH_LINK_NOT_CONNECTED			= (NOTIFY_ERROR | 102),       /* Ethernet link (cable) not connected */
	Error_RED_LINEBRK						= (NOTIFY_ERROR | 103),       /* Redundancy: line break detected */
	Notification_RED_LINEFIXED				= (NOTIFY_ERROR | 104),       /* Redundancy: line is repaired */
	Error_LINE_CROSSED						= (NOTIFY_ERROR | 105),       /* Crossed lines detected */
	Error_NO_SLAVE							= (NOTIFY_ERROR | 106),      
	Error_UNSUPPORTED_SLAVE					= (NOTIFY_ERROR | 107),      
	Error_SLAVE_NO_ANSWER					= (NOTIFY_ERROR | 108),      
	Error_License_RTX						= (NOTIFY_ERROR | 201),
	Error_Library_NotStarted				= (NOTIFY_ERROR | 301),
	Error_Library_StartFail					= (NOTIFY_ERROR | 303),
	Error_Library_StopFail					= (NOTIFY_ERROR | 304),
	Error_Cycle_Jitter						= (NOTIFY_ERROR | 402),
	Error_Cycle_WKC							= (NOTIFY_ERROR | 403),       /* cyclic command: working counter error */
	Error_Cycle_WatchDog					= (NOTIFY_ERROR | 404),
	Notification_STATECHANGED_INIT			= (NOTIFY_GENERIC | 501),     /* EtherCAT operational state change */
	Notification_STATECHANGED_PREOP			= (NOTIFY_GENERIC | 502),     /* EtherCAT operational state change */
	Notification_STATECHANGED_SAFEOP		= (NOTIFY_GENERIC | 503),     /* EtherCAT operational state change */
	Notification_STATECHANGED_OP			= (NOTIFY_GENERIC | 504),     /* EtherCAT operational state change */
	Error_STATECHANGED_INIT					= (NOTIFY_ERROR | 505),     /* EtherCAT operational state change */
	Error_STATECHANGED_PREOP				= (NOTIFY_ERROR | 506),     /* EtherCAT operational state change */
	Error_STATECHANGED_SAFEOP				= (NOTIFY_ERROR | 507),     /* EtherCAT operational state change */
	Error_STATECHANGED_OP					= (NOTIFY_ERROR | 508),     /* EtherCAT operational state change */
	Notification_UPDATE_SLAVE_STATE_DONE    = (NOTIFY_GENERIC | 509),    /* "EC_IOCTL_INITIATE_UPDATE_ALL_SLAVE_STATE" done */
	Notification_ALL_DEVICES_OPERATIONAL	= (NOTIFY_GENERIC | 510),       /* All slave devices are in operational state */
	Error_NOT_ALL_DEVICES_OPERATIONAL		= (NOTIFY_ERROR | 507),       /* Not all slave devices are in operational state when receiving cyclic frames */
	Error_STATUS_SLAVE						= (NOTIFY_ERROR | 508),       /* At least one slave is in error state when receiving cyclic frames (BRD AL-STATUS) */
	Error_SLAVE_UNEXPECTED_STATE			= (NOTIFY_ERROR | 509),       /* slave in unexpected state */
	Notification_DC_RDY                     = (NOTIFY_GENERIC | 801),     /* Distributed clocks initialized */
	Error_FRAME_Loss						= (NOTIFY_ERROR | 902),       /* Got no response on a sent Ethernet frame */
} T_Notification_Code;

typedef struct {
int			NotifCode;
char		NotifMsg[64];
} NotificationData;

typedef enum
{
	ecTimeout = -4,
	ecWrongEnvironment,
	ecWrongParameter,
	ecFailed,
	ecNoError = 0,
	ecBusy = 1,
	ecVariableUnavailable = 200,
	ecDefinitionIncoherent,
	ecMultipleNic = 300,
	ecNoNic,
	ecNoSlave,
	ecUnknownSlave,
	ecNoMemory,
	ecNoBaseDat,
	ecIncorrectFormat,
	ecWrongDeviceCount,
	ecOther
} EC_Error;

typedef enum
{
	sdoNoError = 0,
	sdoToggleBit = 100,
	sdoTimeout = 101,
	sdoCommandSpecifier = 102,
	sdoOutOfMemory = 103,
	sdoUnsupportedAccess = 104,
	sdoWriteOnly = 105,
	sdoReadOnly = 106,
	sdoSubindexReadOnly = 107,
	sdoNoCompleteAccess = 108,
	sdoObjectTooLong = 109,
	sdoObjectInPdo = 110,
	sdoObjectNotExist = 111,
	sdoNoPdoMapping = 112,
	sdoPdoLengthExceeded = 113,
	sdoParameterIncompatible = 114,
	sdoInternalIncompatible = 115,
	sdoHardwareError = 116,
	sdoLengthIncorrect = 117,
	sdoLengthTooHigh = 118,
	sdoLengthTooLow = 119,
	sdoSubindexNotExist = 120,
	sdoValueOutOfRange = 121,
	sdoValueTooHigh = 122,
	sdoValueTooLow = 123,
	sdoMaxBelowMin = 124,
	sdoGeneralError = 125,
	sdoCannotTransfer = 126,
	sdoCannotTransferLocal = 127,
	sdoWrongState = 128,
	sdoDictionaryNotAvailable = 129
} SDO_Error;

typedef enum
{
	kseStopped = 1,
	kseCreated,
	kseWaitNic,
	kseScanBus,
	kseLoadTemplate,
	kseAutoConfig,
	kseLoadEni,
	kseToInit,
	kseToPreOp,
	kseScanMdp,
	kseInitMemory,
	kseStartDc,
	kseToSafeOp,
	kseSyncDc,
	kseToOp,
	kseWaitData,
	kseStarted,
	kseStopping
} KSE_State;


typedef int (__stdcall *AppCallback)(void* Context, int MasterState);

typedef struct {
	int		BufferLength;
	int		DeviceCount;
	int		AxisCount;
	int		IoCount;
	char* 	AxisNames;
	char* 	IoNames;
	int*	InputLengths;
	int*	OutputLengths;
} ScannedNetwork;

coe64_API int RTAPI EcatScanBus(ScannedNetwork* Network);
coe64_API int RTAPI RtEcatCreateMaster();
coe64_API int RTAPI RtEcatSetNIC(char driver[16], int instance);
coe64_API int RTAPI RtEcatStartMaster(AppCallback call, int* slaves, void* context = 0);
coe64_API int RTAPI RtEcatStopMaster();
coe64_API int RTAPI RtEcatRestartMaster();
coe64_API int RTAPI RtEcatGetMasterState();
coe64_API int RTAPI RtEcatRestartSlave(int SlaveId);
coe64_API int RTAPI RtEcatGetNotification(NotificationData *data, bool *NotifLeft);
coe64_API int RTAPI RtEcatSetAutoRestart (bool active);
coe64_API int RTAPI RtEcatSetAutoRepair (bool active);
coe64_API int RTAPI RtEcatSetMOP (bool active);
coe64_API int RTAPI RtEcatSetTimer(int period);
coe64_API int RTAPI RtEcatSetDC(bool active);
coe64_API int RTAPI RtEcatSetThreadPriority(int priority);
coe64_API int RTAPI RtEcatSetMode (int mode);
coe64_API int RTAPI RtEcatSetExtEncoder (bool active);
coe64_API int RTAPI RtEcatSetDI (bool active);
coe64_API int RTAPI RtEcatSetDO (bool active);
coe64_API int RTAPI RtEcatSetEncoderIndex(bool active);
coe64_API int RTAPI RtEcatSetTouchProbe(bool active);
coe64_API int RTAPI RtEcatActiveAutoCfg (bool active);
coe64_API int RTAPI RtEcatSetVerbosity (int verbosity);
coe64_API int RTAPI RtEcatSetAffinity (int affinity);
coe64_API int RTAPI RtEcatSetActualVelocity (bool velocityActVal);
coe64_API int RTAPI RtEcatSetActualTorque (bool torqueActVal);
coe64_API int RTAPI RtEcatSetActualCurrent (bool torqueActVal);
coe64_API int RTAPI RtEcatSetTorqueOffset (bool torqueoffset);
coe64_API int RTAPI RtEcatSetProfilePosition (bool active);
coe64_API int RTAPI RtEcatSetHomingMode (bool active);


coe64_API int RTAPI ReadBits (unsigned char* src, unsigned char* dst, long offs, int length);
coe64_API int RTAPI WriteBits (unsigned char* dst, unsigned char* src, long offs, int length);
coe64_API int RTAPI ReadWord (unsigned char* ptr, long offs, unsigned short* val);
coe64_API int RTAPI WriteWord (unsigned char* ptr, unsigned short val, long offs);
coe64_API int RTAPI ReadDWord (unsigned char* ptr, long offs, unsigned long* val);
coe64_API int RTAPI WriteDWord (unsigned char* ptr, unsigned long val, long offs);
coe64_API int RTAPI ReadStatusWord (AXIS_ECAT* axis, unsigned short* wStatus);
coe64_API int RTAPI ReadActualPosition (AXIS_ECAT* axis, unsigned long* dwPosition);
coe64_API int RTAPI ReadActualTorque (AXIS_ECAT* axis, unsigned short* dwTorque);
coe64_API int RTAPI ReadActualCurrent(AXIS_ECAT* axis, unsigned short* wCurrent);
coe64_API int RTAPI ReadActualVelocity (AXIS_ECAT* axis, unsigned long* dwVelocity);
coe64_API int RTAPI ReadInternalEncoder (AXIS_ECAT* axis, unsigned long* dwPosition);
coe64_API int RTAPI ReadDI (AXIS_ECAT* axis, unsigned long* dwDI);
coe64_API int RTAPI ReadModeOP (AXIS_ECAT* axis, unsigned char* bVal);
coe64_API int RTAPI ReadPositionDemand (AXIS_ECAT* axis, unsigned long* dwPosition);
coe64_API int RTAPI ReadIndex(AXIS_ECAT* axis, unsigned char* bIndex);
coe64_API int RTAPI ReadTouchProbeStatus(AXIS_ECAT* axis, unsigned short* wStatus);
coe64_API int RTAPI ReadControlWord (AXIS_ECAT* axis, unsigned short* wControl);
coe64_API int RTAPI WriteControlWord (AXIS_ECAT* axis, unsigned short val);
coe64_API int RTAPI WriteModeOP (AXIS_ECAT* axis, unsigned char val);
coe64_API int RTAPI ReadDO(AXIS_ECAT* axis, unsigned long* dwDO);
coe64_API int RTAPI ReadCustomInput1(AXIS_ECAT* axis, unsigned long* dwValue);
coe64_API int RTAPI ReadCustomInput2(AXIS_ECAT* axis, unsigned long* dwValue);
coe64_API int RTAPI WriteTouchProbeControl(AXIS_ECAT* axis, unsigned short val);
coe64_API int RTAPI WriteDO(AXIS_ECAT* axis, unsigned long dwDO);
coe64_API int RTAPI WriteCustomOutput1(AXIS_ECAT* axis, long dwValue);
coe64_API int RTAPI WriteCustomOutput2(AXIS_ECAT* axis, long dwValue);
coe64_API int RTAPI WriteTargetPosition (AXIS_ECAT* axis, long val);
coe64_API int RTAPI WriteTargetVelocity (AXIS_ECAT* axis, long val);
coe64_API int RTAPI WriteTargetTorque (AXIS_ECAT* axis, unsigned short val);
coe64_API int RTAPI WriteTorqueOffset (AXIS_ECAT* axis, short dwTorqueOffset);
coe64_API int RTAPI WriteProfileVelocity (AXIS_ECAT* axis, long val);
coe64_API int RTAPI WriteProfileAcceleration (AXIS_ECAT* axis, long val);
coe64_API int RTAPI WriteProfileDeceleration (AXIS_ECAT* axis, long val);
coe64_API int RTAPI WriteHomingMethod (AXIS_ECAT* axis, char val);
coe64_API int RTAPI WriteHomingSwitchSpeed (AXIS_ECAT* axis, long val);
coe64_API int RTAPI WriteHomingZeroSpeed (AXIS_ECAT* axis, long val);
coe64_API int RTAPI WriteHomingAcceleration (AXIS_ECAT* axis, long val);
coe64_API int RTAPI WriteQuickStopDeceleration (AXIS_ECAT* axis, long val);
coe64_API int RTAPI ReadPot(AXIS_ECAT* axis, BOOL* bPot);
coe64_API int RTAPI ReadNot(AXIS_ECAT* axis, BOOL* bNot);
coe64_API int RTAPI CoeSdoDownload ( unsigned long dwSlaveId, unsigned short wObIndex, unsigned char byObSubIndex, unsigned char* pbyData, unsigned long dwDataLen,  unsigned long dwTimeout, unsigned long dwFlags );
coe64_API int RTAPI CoeSdoUpload ( unsigned long dwSlaveId, unsigned short wObIndex, unsigned char byObSubIndex, unsigned char* pbyData, unsigned long dwDataLen, unsigned long* pdwOutDataLen, unsigned long dwTimeout, unsigned long dwFlags );
coe64_API int RTAPI WriteSlaveRegister(int bFixedAddress, unsigned short wSlaveAddress, unsigned short wRegisterOffset, void* pvData, unsigned short wLen, unsigned long dwTimeout) ;
coe64_API int RTAPI ReadSlaveRegister(int bFixedAddress, unsigned short wSlaveAddress, unsigned short wRegisterOffset, void* pvData, unsigned short wLen, unsigned long dwTimeout);

#ifdef __cplusplus
}
#endif
#endif


#endif //end if UNDER_RTSS && _AMD64_