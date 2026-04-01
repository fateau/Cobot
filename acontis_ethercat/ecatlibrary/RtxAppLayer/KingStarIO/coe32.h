#if defined UNDER_RTSS &&  !defined _AMD64_
#pragma once

/*
*              ZZZZZZZZ    Copyright (c) 2013 IntervalZero, Inc.  All rights reserved.
*            ZZZZ   ZZZZZ
*         O ZZZZ      ZZZ O      Module Name:
*       OII ZZZ        ZZ IOII
*     OIII  ZZZZ      ZZZ  IIII     coe32.cpp
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
*           ZZZZZZZZZZZZZ        x86 and  King-Star IO application 
*                         
*     Revision History:
*		No Revision History
*/


#include <windows.h>
#include <rtapi.h>
#include <tchar.h>

#define NO_ERRORS		0
#define ERROR_OCCURED	-1
//-TYPEDEFS------------------------------------------------------------------
#ifndef COE_H
#define COE_H

typedef enum _EC_T_STATE
{
    eEcatState_UNKNOWN  = 0,                        //< unknown 
    eEcatState_INIT     = 1,                        //< init 
    eEcatState_PREOP    = 2,                        //< pre-operational 
    eEcatState_SAFEOP   = 4,                        //< safe operational 
    eEcatState_OP       = 8,                        //< operational 
} EC_T_STATE;

typedef struct  {
int			NotifCode;
char		NotifMsg[64];
} NotificationData;

typedef struct
{
    unsigned int	    dwVendorId;
    unsigned int		dwProductCode;
	unsigned int		dwRevisionNumber;
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
} AXIS_ECAT;

typedef struct
{
    unsigned int		dwVendorId;
    unsigned int		dwProductCode;
	unsigned int		dwRevisionNumber;
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
} IO_ECAT;

#define NOTIFY_GENERIC							0x00000000
#define NOTIFY_ERROR							0x00010000
#define NOTIFY_OTHER							0x00100000
typedef enum _T_Notification_Code
{
	Notification_ETH_LINK_CONNECTED         = (NOTIFY_GENERIC | 101),     // Ethernet link (cable) connected 
	Error_ETH_LINK_NOT_CONNECTED			= (NOTIFY_ERROR | 102),       // Ethernet link (cable) not connected 
	Error_RED_LINEBRK						= (NOTIFY_ERROR | 103),       // Redundancy: line break detected 
	Notification_RED_LINEFIXED				= (NOTIFY_ERROR | 104),       // Redundancy: line is repaired 
	Error_LINE_CROSSED						= (NOTIFY_ERROR | 105),       // Crossed lines detected 
	Error_NO_SLAVE							= (NOTIFY_ERROR | 106),      
	Error_UNSUPPORTED_SLAVE					= (NOTIFY_ERROR | 107),      
	Error_SLAVE_NO_ANSWER					= (NOTIFY_ERROR | 108),      
	Error_License_RTX						= (NOTIFY_ERROR | 201),
	Error_License_Solo						= (NOTIFY_ERROR | 202),
	Error_Library_NotStarted				= (NOTIFY_ERROR | 301),
	Error_Library_IncorrectMode				= (NOTIFY_ERROR | 302),
	Error_Library_StartFail					= (NOTIFY_ERROR | 303),
	Error_Library_StopFail					= (NOTIFY_ERROR | 304),
	Error_Cycle_Other						= (NOTIFY_ERROR | 401),
	Error_Cycle_Jitter						= (NOTIFY_ERROR | 402),
	Error_Cycle_WKC							= (NOTIFY_ERROR | 403),       // cyclic command: working counter error 
	Error_Cycle_WatchDog					= (NOTIFY_ERROR | 404),
	Notification_STATECHANGED_INIT			= (NOTIFY_GENERIC | 501),     // EtherCAT operational state change 
	Notification_STATECHANGED_PREOP			= (NOTIFY_GENERIC | 502),     // EtherCAT operational state change 
	Notification_STATECHANGED_SAFEOP		= (NOTIFY_GENERIC | 503),     // EtherCAT operational state change 
	Notification_STATECHANGED_OP			= (NOTIFY_GENERIC | 504),     // EtherCAT operational state change 
	Error_STATECHANGED_INIT					= (NOTIFY_ERROR | 505),     // EtherCAT operational state change 
	Error_STATECHANGED_PREOP				= (NOTIFY_ERROR | 506),     // EtherCAT operational state change 
	Error_STATECHANGED_SAFEOP				= (NOTIFY_ERROR | 507),     // EtherCAT operational state change 
	Error_STATECHANGED_OP					= (NOTIFY_ERROR | 508),     // EtherCAT operational state change 
	Notification_UPDATE_SLAVE_STATE_DONE    = (NOTIFY_GENERIC | 509),    // "EC_IOCTL_INITIATE_UPDATE_ALL_SLAVE_STATE" done 
	Notification_ALL_DEVICES_OPERATIONAL	= (NOTIFY_GENERIC | 510),       // All slave devices are in operational state 
	Error_NOT_ALL_DEVICES_OPERATIONAL		= (NOTIFY_ERROR | 507),       // Not all slave devices are in operational state when receiving cyclic frames 
	Error_STATUS_SLAVE						= (NOTIFY_ERROR | 508),       // At least one slave is in error state when receiving cyclic frames (BRD AL-STATUS) 
	Error_SLAVE_UNEXPECTED_STATE			= (NOTIFY_ERROR | 509),       // slave in unexpected state 
	Error_MasterCore_NoResponse				= (NOTIFY_ERROR | 601),
	Error_MasterCore_DeInitFail				= (NOTIFY_ERROR | 602),
	Notification_SB_END                     = (NOTIFY_GENERIC | 701),     // ScanBus finished  
	Error_SB_MISMATCH						= (NOTIFY_ERROR | 702),     // ScanBus mismatch 
	Error_SB_ERROR							= (NOTIFY_ERROR | 702),		  // Bus could not be scanned 
	Notification_DC_RDY                     = (NOTIFY_GENERIC | 801),     // Distributed clocks initialized 
	Notification_DC_SLV_SYNC                = (NOTIFY_GENERIC | 802),     // DC Slave Synchronization deviation notification 
	Notification_DCL_RDY                    = (NOTIFY_GENERIC | 803),     // DCL initialized 
	Notification_DCM_SYNC                   = (NOTIFY_GENERIC | 804),     // DCM InSync 
	Error_FRAME_RESPONSE					= (NOTIFY_ERROR | 901),       // Got no response on a sent Ethernet frame 
	Error_FRAME_Loss						= (NOTIFY_ERROR | 902),       // Got no response on a sent Ethernet frame 
	Error_INITCMD_MASTER_WKC				= (NOTIFY_ERROR | 1001),       // master init command: working counter error 
	Error_INITCMD_MASTER_RESPONSE			= (NOTIFY_ERROR | 1002),       // Got no response on a sent ecat master init command 
	Error_INITCMD_SLAVE_WKC					= (NOTIFY_ERROR | 1003),       // slave init command: working counter error 
	Error_INITCMD_SLAVE_RESPONSE			= (NOTIFY_ERROR | 1004),       // Got no response on a sent ecat init command from slave 
	Error_INITCMD_MBSLAVE_TIMEOUT			= (NOTIFY_ERROR | 1005),       // Timeout when waiting for mailbox init command response 
	Error_MB_SLAVE_COE_SDO_ABORT			= (NOTIFY_ERROR | 1101),       // COE mbox SDO abort 
	Error_MB_COEXRCV_WKC					= (NOTIFY_ERROR | 1102),       // COE mbox receive: working counter error 
	Error_MB_COEXSND_WKC					= (NOTIFY_ERROR | 1103),       // COE mbox send: working counter error 
	Error_MB_RCV_INVALID_DATA				= (NOTIFY_ERROR | 1104),       // invalid mail box data received 
	
	Notification_Other						= 0xffffffff
} T_Notification_Code;

typedef int (__stdcall *AppCallback)(void* Context, EC_T_STATE MasterState);

#endif

int PreloadAPI();
int RtEcatCreateMaster (int master);
int RtEcatSetNIC(char driver[16], bool polling, int instance);
int RtEcatStartMaster (AppCallback call, int* slaves, void* context = 0);
int RtEcatRestartMaster ();
int RtEcatStopMaster ();
int RtEcatGetNotification(NotificationData *data, bool *NotifLeft);
int RtEcatSetTimer(int period);
int RtEcatSetDelay(int delay);
int RtEcatActiveAutoCfg(bool active);
int RtEcatSetMode (int mode);
int RtEcatSetMOP (bool active);
int RtEcatSetExtEncoder (bool active);
int RtEcatSetTorqueActVal (bool active);
int RtEcatSetVelocityActVal (bool active);
int RtEcatSetTorqueOffset (bool active);
int RtEcatSetProfilePosition (bool active);
int RtEcatSetHomingMode (bool active);
int RtEcatSetDC (bool active); //for DC
int RtEcatSetDIO (bool active);
int RtEcatSetDI (bool active);
int RtEcatSetDO (bool active);
int RtEcatSetEncoderIndex (bool active);
int RtEcatActiveDelay (bool active);
int RtEcatSetVerbosity (int verbosity);
int ReadBits (unsigned char* src, unsigned char* dst, unsigned long offs, int length);
int WriteBits (unsigned char* dst, unsigned char* src, unsigned long offs, int length);
int ReadWord (unsigned short* ptr, unsigned long offs, unsigned short* val);
int WriteWord (unsigned short* ptr, unsigned short val, unsigned long offs);
int ReadDWord (unsigned long* ptr, unsigned long offs, unsigned long* val);
int WriteDWord (unsigned long* ptr, unsigned long val, unsigned long offs);
int ReadStatusWord (AXIS_ECAT* axis, unsigned short* wStatus);
int ReadActualPosition (AXIS_ECAT* axis, unsigned long* dwPosition);
int ReadInternalEncoder (AXIS_ECAT* axis, unsigned long* dwPosition);
int ReadActualVelocity (AXIS_ECAT* axis, unsigned long* dwVelocity);
int ReadActualTorque (AXIS_ECAT* axis, unsigned long* dwTorque);
int ReadDI (AXIS_ECAT* axis, unsigned long* dwDI);
int ReadIndex (AXIS_ECAT* axis, unsigned char* bIndex);
int ReadPositionDemand (AXIS_ECAT* axis, unsigned long* dwPosition);
int ReadControlWord (AXIS_ECAT* axis, unsigned short* wControl);
int WriteControlWord (AXIS_ECAT* axis, unsigned short val);
int WriteModeOP (AXIS_ECAT* axis, unsigned char val);
int ReadDO (AXIS_ECAT* axis, unsigned long* dwDO);
int WriteDO (AXIS_ECAT* axis, unsigned long dwDO);
int WriteTargetPosition (AXIS_ECAT* axis, long val);
int WriteTargetVelocity (AXIS_ECAT* axis, long val);
int WriteTargetTorque (AXIS_ECAT* axis, long val);
int WriteTorqueOffset (AXIS_ECAT* axis, long dwTorqueOffset);
int WriteProfileVelocity (AXIS_ECAT* axis, long val);
int WriteProfileAcceleration (AXIS_ECAT* axis, long val);
int WriteProfileDeceleration (AXIS_ECAT* axis, long val);
int WriteHomingMethod (AXIS_ECAT* axis, char val);
int WriteHomingSwitchSpeed (AXIS_ECAT* axis, long val);
int WriteHomingZeroSpeed (AXIS_ECAT* axis, long val);
int WriteHomingAcceleration (AXIS_ECAT* axis, long val);
int WriteQuickStopDeceleration (AXIS_ECAT* axis, long val);
int CoeSdoDownload ( unsigned long dwSlaveId, unsigned short wObIndex, unsigned char byObSubIndex, unsigned char* pbyData, unsigned long dwDataLen,  unsigned long dwTimeout, unsigned long dwFlags );
int CoeSdoUpload ( unsigned long dwSlaveId, unsigned short wObIndex, unsigned char byObSubIndex, unsigned char* pbyData, unsigned long dwDataLen, unsigned long* pdwOutDataLen, unsigned long dwTimeout, unsigned long dwFlags );

#endif