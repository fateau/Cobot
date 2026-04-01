#if defined UNDER_RTSS && !defined _AMD64_ 
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


#include "coe32.h"

typedef struct tagRtxEcatAPI
{
	int (__cdecl *pfnRtEcatCreateMaster)(int master);
	int (__cdecl *pfnRtEcatSetNIC)(char driver[16], bool polling, int instance);
	int (__cdecl *pfnRtEcatStartMaster)(AppCallback call, int* slaves, void* context);
	int (__cdecl *pfnRtEcatRestartMaster)();
	int (__cdecl *pfnRtEcatStopMaster)();
	int (__cdecl *pfnRtEcatGetNotification)(NotificationData *data, bool *NotifLeft);
	int (__cdecl *pfnRtEcatSetMOP)(bool active);
	int (__cdecl *pfnRtEcatSetTimer)(int period);
	int (__cdecl *pfnRtEcatSetDelay)(bool active);
	int (__cdecl *pfnRtEcatSetDC)(bool active);
	int (__cdecl *pfnRtEcatSetThreadPriority)(int priority);
	int (__cdecl *pfnRtEcatSetMode)(int mode);
	int (__cdecl *pfnRtEcatSetExtEncoder)(bool active);
	int (__cdecl *pfnRtEcatSetDIO)(bool active);
	int (__cdecl *pfnRtEcatSetDI)(bool active);
	int (__cdecl *pfnRtEcatSetDO)(bool active);
	int (__cdecl *pfnRtEcatSetEncoderIndex)(bool active);
	int (__cdecl *pfnRtEcatActiveDelay)(bool active);
	int (__cdecl *pfnRtRtEcatActiveAutoCfg)(bool active);
	int (__cdecl *pfnRtEcatActiveAutoCfg)(bool active);
	int (__cdecl *pfnRtEcatSetVerbosity)(int verbosity);
	int (__cdecl *pfnRtEcatSetAffinity)(int affinity);
	int (__cdecl *pfnRtEcatSetTorqueActVal)(bool active);
	int (__cdecl *pfnRtEcatSetVelocityActVal)(bool active);
	int (__cdecl *pfnRtEcatSetTorqueOffset)(bool active);
	int (__cdecl *pfnRtEcatSetProfilePosition)(bool active);
	int (__cdecl *pfnRtEcatSetHomingMode)(bool active);
	int (__cdecl *pfnReadBits)(unsigned char* src, unsigned char* dst, unsigned long offs, int length);
	int (__cdecl *pfnWriteBits)(unsigned char* dst, unsigned char* src, unsigned long offs, int length);
	int (__cdecl *pfnReadWord)(unsigned short* ptr, unsigned long offs, unsigned short* val);
	int (__cdecl *pfnWriteWord)(unsigned short* ptr, unsigned short val, unsigned long offs);
	int (__cdecl *pfnReadDWord)(unsigned long* ptr, unsigned long offs, unsigned long* val);
	int (__cdecl *pfnWriteDWord)(unsigned long* ptr, unsigned long val, unsigned long offs);
	int (__cdecl *pfnReadStatusWord)(AXIS_ECAT* axis, unsigned short* wStatus);
	int (__cdecl *pfnReadActualPosition)(AXIS_ECAT* axis, unsigned long* dwPosition);
	int (__cdecl *pfnReadActualTorque)(AXIS_ECAT* axis, unsigned long* dwTorque);
	int (__cdecl *pfnReadActualVelocity)(AXIS_ECAT* axis, unsigned long* dwVelocity);
	int (__cdecl *pfnReadInternalEncoder)(AXIS_ECAT* axis, unsigned long* dwPosition);
	int (__cdecl *pfnReadDI)(AXIS_ECAT* axis, unsigned long* dwDI);
	int (__cdecl *pfnReadIndex)(AXIS_ECAT* axis, unsigned char* bIndex);
	int (__cdecl *pfnReadPositionDemand)(AXIS_ECAT* axis, unsigned long* dwPosition);
	int (__cdecl *pfnReadControlWord)(AXIS_ECAT* axis, unsigned short* wControl);
	int (__cdecl *pfnWriteControlWord)(AXIS_ECAT* axis, unsigned short val);
	int (__cdecl *pfnWriteModeOP)(AXIS_ECAT* axis, unsigned char val);
	int (__cdecl *pfnReadDO)(AXIS_ECAT* axis, unsigned long* dwDO);
	int (__cdecl *pfnWriteDO)(AXIS_ECAT* axis, unsigned long dwDO);
	int (__cdecl *pfnWriteTargetPosition)(AXIS_ECAT* axis, long val);
	int (__cdecl *pfnWriteTargetVelocity)(AXIS_ECAT* axis, long val);
	int (__cdecl *pfnWriteTargetTorque)(AXIS_ECAT* axis, long val);
	int (__cdecl *pfnWriteTorqueOffset)(AXIS_ECAT* axis, long dwTorqueOffset);
	int (__cdecl *pfnWriteProfileVelocity)(AXIS_ECAT* axis, long val);
	int (__cdecl *pfnWriteProfileAcceleration)(AXIS_ECAT* axis, long val);
	int (__cdecl *pfnWriteProfileDeceleration)(AXIS_ECAT* axis, long val);
	int (__cdecl *pfnWriteHomingMethod)(AXIS_ECAT* axis, char val);
	int (__cdecl *pfnWriteHomingSwitchSpeed)(AXIS_ECAT* axis, long val);
	int (__cdecl *pfnWriteHomingZeroSpeed)(AXIS_ECAT* axis, long val);
	int (__cdecl *pfnWriteHomingAcceleration)(AXIS_ECAT* axis, long val);
	int (__cdecl *pfnWriteQuickStopDeceleration)(AXIS_ECAT* axis, long val);
	int (__cdecl *pfnCoeSdoDownload)(unsigned long dwSlaveId, unsigned short wObIndex, unsigned char byObSubIndex, unsigned char* pbyData, unsigned long dwDataLen,  unsigned long dwTimeout, unsigned long dwFlags );
	int (__cdecl *pfnCoeSdoUpload)(unsigned long dwSlaveId, unsigned short wObIndex, unsigned char byObSubIndex, unsigned char* pbyData, unsigned long dwDataLen, unsigned long* pdwOutDataLen, unsigned long dwTimeout, unsigned long dwFlags );
	int (__cdecl *pfnWriteSlaveRegister)(int bFixedAddress, unsigned short wSlaveAddress, unsigned short wRegisterOffset, void* pvData, unsigned short wLen, unsigned long dwTimeout);
	int (__cdecl *pfnReadSlaveRegister)(int bFixedAddress, unsigned short wSlaveAddress, unsigned short wRegisterOffset, void* pvData, unsigned short wLen, unsigned long dwTimeout);
} RtxEcatAPI;

LPCTSTR lpLibFileName = "coe32aco.rtdll";
//LPCTSTR lpLibFileName = "RtxEcatLayer.rtdll";
 
static RtxEcatAPI s_Api = {0};

HMODULE s_hRtxEcatLib = NULL;

bool GetFunctionPtr(void** ppfn, const char* pszName)
{
    s_hRtxEcatLib = LoadLibrary(lpLibFileName);
    if (s_hRtxEcatLib == NULL )
    {
        RtPrintf("Can't load rt library %s, Get last Error = %d\n", lpLibFileName, GetLastError());
        return false;
    }
    *ppfn = GetProcAddress(s_hRtxEcatLib, (LPSTR)pszName);
    if(NULL == *ppfn)
    {
        RtPrintf( "Can't get procedure %s address\n", pszName);
        return false;
    }
    return true;
}

#define CHECK_FUNCTION_PTR(pfn, name)           \
    if ((pfn) == NULL)                     \
    GetFunctionPtr((void**)&(pfn), #name);      \
    if ((pfn) == NULL)                     \
    return -1;

#define GET_FUNCTION_PTR(pfn, name)           \
    GetFunctionPtr((void**)&(pfn), #name);      \
    if ((pfn) == NULL)                     \
    	printf("Can't find '%s'\n",#name);

int PreloadAPI() {
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatSetTimer, RtEcatSetTimer);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatSetDelay, RtEcatSetDelay);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatSetMode, RtEcatSetMode);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatSetMOP, RtEcatSetMOP);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatSetExtEncoder, RtEcatSetExtEncoder);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatSetTorqueActVal, RtEcatSetTorqueActVal);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatSetVelocityActVal, RtEcatSetVelocityActVal);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatSetTorqueOffset, RtEcatSetTorqueOffset);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatSetDIO, RtEcatSetDIO);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatSetDI, RtEcatSetDI);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatSetDO, RtEcatSetDO);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatSetDC, RtEcatSetDC);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatSetEncoderIndex, RtEcatSetEncoderIndex);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatActiveDelay, RtEcatActiveDelay);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatActiveAutoCfg, RtEcatActiveAutoCfg);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatSetVerbosity, RtEcatSetVerbosity);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatSetNIC, RtEcatSetNIC);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatStartMaster, RtEcatStartMaster);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatRestartMaster, RtEcatRestartMaster);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatStopMaster, RtEcatStopMaster);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatGetNotification, RtEcatGetNotification);
    CHECK_FUNCTION_PTR(s_Api.pfnReadBits, ReadBits);
    CHECK_FUNCTION_PTR(s_Api.pfnWriteBits, WriteBits);
    CHECK_FUNCTION_PTR(s_Api.pfnReadWord, ReadWord);
    CHECK_FUNCTION_PTR(s_Api.pfnWriteWord, WriteWord);
    CHECK_FUNCTION_PTR(s_Api.pfnReadDWord, ReadDWord);
    CHECK_FUNCTION_PTR(s_Api.pfnWriteDWord, WriteDWord);
    CHECK_FUNCTION_PTR(s_Api.pfnReadStatusWord, ReadStatusWord);
    CHECK_FUNCTION_PTR(s_Api.pfnReadActualPosition, ReadActualPosition);
    CHECK_FUNCTION_PTR(s_Api.pfnReadInternalEncoder, ReadInternalEncoder);
    CHECK_FUNCTION_PTR(s_Api.pfnReadActualVelocity, ReadActualVelocity);
    CHECK_FUNCTION_PTR(s_Api.pfnReadActualTorque, ReadActualTorque);
    CHECK_FUNCTION_PTR(s_Api.pfnReadDI, ReadDI);
    CHECK_FUNCTION_PTR(s_Api.pfnReadIndex, ReadIndex);
    CHECK_FUNCTION_PTR(s_Api.pfnReadControlWord, ReadControlWord);
    CHECK_FUNCTION_PTR(s_Api.pfnWriteControlWord, WriteControlWord);
    CHECK_FUNCTION_PTR(s_Api.pfnWriteTorqueOffset, WriteTorqueOffset);
    CHECK_FUNCTION_PTR(s_Api.pfnWriteModeOP, WriteModeOP);
    CHECK_FUNCTION_PTR(s_Api.pfnReadDO, ReadDO);
    CHECK_FUNCTION_PTR(s_Api.pfnWriteDO, WriteDO);
    CHECK_FUNCTION_PTR(s_Api.pfnWriteTargetPosition, WriteTargetPosition);
    CHECK_FUNCTION_PTR(s_Api.pfnWriteTargetVelocity, WriteTargetVelocity);
    CHECK_FUNCTION_PTR(s_Api.pfnWriteTargetTorque, WriteTargetTorque);
    CHECK_FUNCTION_PTR(s_Api.pfnCoeSdoDownload, CoeSdoDownload);
    CHECK_FUNCTION_PTR(s_Api.pfnCoeSdoUpload, CoeSdoUpload);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatSetProfilePosition, RtEcatSetProfilePosition);
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatSetHomingMode, RtEcatSetHomingMode);
    CHECK_FUNCTION_PTR(s_Api.pfnReadPositionDemand, ReadPositionDemand);
    CHECK_FUNCTION_PTR(s_Api.pfnWriteProfileVelocity, WriteProfileVelocity);
    CHECK_FUNCTION_PTR(s_Api.pfnWriteProfileAcceleration, WriteProfileAcceleration);
    CHECK_FUNCTION_PTR(s_Api.pfnWriteProfileDeceleration, WriteProfileDeceleration);
    CHECK_FUNCTION_PTR(s_Api.pfnWriteHomingMethod, WriteHomingMethod);
    CHECK_FUNCTION_PTR(s_Api.pfnWriteHomingSwitchSpeed, WriteHomingSwitchSpeed);
    CHECK_FUNCTION_PTR(s_Api.pfnWriteHomingZeroSpeed, WriteHomingZeroSpeed);
    CHECK_FUNCTION_PTR(s_Api.pfnWriteHomingAcceleration, WriteHomingAcceleration);
    CHECK_FUNCTION_PTR(s_Api.pfnWriteQuickStopDeceleration, WriteQuickStopDeceleration);
	return 0;
}

int UnloadAPI() {
    s_Api.pfnRtEcatSetTimer = NULL;
    s_Api.pfnRtEcatSetDelay = NULL;
    s_Api.pfnRtEcatSetMode = NULL;
    s_Api.pfnRtEcatSetMOP = NULL;
    s_Api.pfnRtEcatSetExtEncoder = NULL;
    s_Api.pfnRtEcatSetTorqueActVal = NULL;
    s_Api.pfnRtEcatSetVelocityActVal = NULL;
    s_Api.pfnRtEcatSetTorqueOffset = NULL;
    s_Api.pfnRtEcatSetDIO = NULL;
    s_Api.pfnRtEcatSetDI = NULL;
    s_Api.pfnRtEcatSetDO = NULL;
    s_Api.pfnRtEcatSetDC = NULL;
    s_Api.pfnRtEcatSetEncoderIndex = NULL;
    s_Api.pfnRtEcatActiveDelay = NULL;
    s_Api.pfnRtEcatActiveAutoCfg = NULL;
    s_Api.pfnRtEcatSetVerbosity = NULL;
	s_Api.pfnRtEcatCreateMaster = NULL;
    s_Api.pfnRtEcatSetNIC = NULL;
    s_Api.pfnRtEcatStartMaster = NULL;
    s_Api.pfnRtEcatRestartMaster = NULL;
    s_Api.pfnRtEcatStopMaster = NULL;
    s_Api.pfnRtEcatGetNotification = NULL;
    s_Api.pfnReadBits = NULL;
    s_Api.pfnWriteBits = NULL;
    s_Api.pfnReadWord = NULL;
    s_Api.pfnWriteWord = NULL;
    s_Api.pfnReadDWord = NULL;
    s_Api.pfnWriteDWord = NULL;
    s_Api.pfnReadStatusWord = NULL;
    s_Api.pfnReadActualPosition = NULL;
    s_Api.pfnReadInternalEncoder = NULL;
    s_Api.pfnReadActualVelocity = NULL;
    s_Api.pfnReadActualTorque = NULL;
    s_Api.pfnReadDI = NULL;
    s_Api.pfnReadIndex = NULL;
    s_Api.pfnReadControlWord = NULL;
    s_Api.pfnWriteControlWord = NULL;
    s_Api.pfnWriteTorqueOffset = NULL;
    s_Api.pfnWriteModeOP = NULL;
    s_Api.pfnReadDO = NULL;
    s_Api.pfnWriteDO = NULL;
    s_Api.pfnWriteTargetPosition = NULL;
    s_Api.pfnWriteTargetVelocity = NULL;
    s_Api.pfnWriteTargetTorque = NULL;
    s_Api.pfnCoeSdoDownload = NULL;
    s_Api.pfnCoeSdoUpload = NULL;
    s_Api.pfnRtEcatSetProfilePosition = NULL;
    s_Api.pfnRtEcatSetHomingMode = NULL;
    s_Api.pfnReadPositionDemand = NULL;
    s_Api.pfnWriteProfileVelocity = NULL;
    s_Api.pfnWriteProfileAcceleration = NULL;
    s_Api.pfnWriteProfileDeceleration = NULL;
    s_Api.pfnWriteHomingMethod = NULL;
    s_Api.pfnWriteHomingSwitchSpeed = NULL;
    s_Api.pfnWriteHomingZeroSpeed = NULL;
    s_Api.pfnWriteHomingAcceleration = NULL;
    s_Api.pfnWriteQuickStopDeceleration = NULL;
	return 0;
}

int RtEcatCreateMaster(int master) {
	PreloadAPI();
    CHECK_FUNCTION_PTR(s_Api.pfnRtEcatCreateMaster, RtEcatCreateMaster);
	int Master = master;
	int nret = (*s_Api.pfnRtEcatCreateMaster)(Master);
	return nret;
}
int RtEcatSetNIC(char driver[16], bool polling, int instance) {
	int nret = 0;
	bool Polling = polling;
	int Instance = instance;
	char Driver[16] = {0};
	strcpy_s(Driver, driver);
    if (s_Api.pfnRtEcatSetNIC != NULL)
		nret = (*s_Api.pfnRtEcatSetNIC)(Driver, Polling, Instance);
	else
		nret = -1;
	return nret;
}
int RtEcatStartMaster(AppCallback call, int* slaves, void* context) {
	int nret = 0;
	AppCallback Call = call;
	int* Slaves = slaves;
	void* Context = context;
    if (s_Api.pfnRtEcatStartMaster != NULL)
		nret = (*s_Api.pfnRtEcatStartMaster)(Call, Slaves, Context);
	else
		nret = -1;
	return nret;
}
int RtEcatRestartMaster() {
	int nret = 0;
 	if (s_hRtxEcatLib != NULL) {
		CHECK_FUNCTION_PTR(s_Api.pfnRtEcatRestartMaster, RtEcatRestartMaster);
		nret = (*s_Api.pfnRtEcatRestartMaster)();
		Sleep(2000);
		FreeLibrary(s_hRtxEcatLib);
	}
	s_hRtxEcatLib = NULL;
	UnloadAPI();
	return nret;
}
int RtEcatStopMaster() {
	int nret = 0;
 	if (s_hRtxEcatLib != NULL) {
		CHECK_FUNCTION_PTR(s_Api.pfnRtEcatStopMaster, RtEcatStopMaster);
		nret = (*s_Api.pfnRtEcatStopMaster)();
		Sleep(2000);
		FreeLibrary(s_hRtxEcatLib);
	}
	s_hRtxEcatLib = NULL;
	UnloadAPI();
	return nret;
}
int RtEcatGetNotification(NotificationData *data, bool *notifLeft) {
	int nret = 0;
	NotificationData* Data = data;
	bool* NotifLeft = notifLeft;
    if (s_Api.pfnRtEcatGetNotification != NULL)
		nret = (*s_Api.pfnRtEcatGetNotification)(Data, NotifLeft);
	else
		nret = -1;
	return nret;
}
int RtEcatSetMOP(bool active) {
	int nret = 0;
	bool Active = active;
    if (s_Api.pfnRtEcatSetMOP != NULL)
		nret = (*s_Api.pfnRtEcatSetMOP)(Active);
	else
		nret = -1;
	return nret;
}
int RtEcatSetTimer(int period) {
	int nret = 0;
	int Period = period;
	if (s_Api.pfnRtEcatSetTimer != NULL)
		nret = (*s_Api.pfnRtEcatSetTimer)(Period);
	else
		nret = -1;
	return nret;
}
int RtEcatSetDelay(int delay) {
	int nret = 0;
	int Delay = delay;
	if (s_Api.pfnRtEcatSetTimer != NULL)
		nret = (*s_Api.pfnRtEcatSetDelay)(Delay);
	else
		nret = -1;
	return nret;
}
int RtEcatSetDC(bool active) {
	int nret = 0;
	bool Active = active;
    if (s_Api.pfnRtEcatSetDC != NULL)
		nret = (*s_Api.pfnRtEcatSetDC)(Active);
	else
		nret = -1;
	return nret;
}
int RtEcatActiveAutoCfg(bool active) {
	int nret = 0;
	bool Active = active;
    if (s_Api.pfnRtEcatSetDC != NULL)
		nret = (*s_Api.pfnRtEcatActiveAutoCfg)(Active);
	else
		nret = -1;
	return nret;
}
int RtEcatSetThreadPriority(int priority) {
	int nret = 0;
	int Priority = priority;
    if (s_Api.pfnRtEcatSetThreadPriority != NULL)
		nret = (*s_Api.pfnRtEcatSetThreadPriority)(Priority);
	else
		nret = -1;
	return nret;
}
int RtEcatSetMode(int mode) {
	int nret = 0;
	int Mode = mode;
    if (s_Api.pfnRtEcatSetMode != NULL)
		nret = (*s_Api.pfnRtEcatSetMode)(Mode);
	else
		nret = -1;
	return nret;
}
int RtEcatSetExtEncoder(bool active) {
	int nret = 0;
	bool Active = active;
    if (s_Api.pfnRtEcatSetExtEncoder != NULL)
		nret = (*s_Api.pfnRtEcatSetExtEncoder)(Active);
	else
		nret = -1;
	return nret;
}
int RtEcatSetDIO(bool active) {
	int nret = 0;
	bool Active = active;
    if (s_Api.pfnRtEcatSetDIO != NULL)
		nret = (*s_Api.pfnRtEcatSetDIO)(Active);
	else
		nret = -1;
	return nret;
}
int RtEcatSetDI(bool active) {
	int nret = 0;
	bool Active = active;
    if (s_Api.pfnRtEcatSetDI != NULL)
		nret = (*s_Api.pfnRtEcatSetDI)(Active);
	else
		nret = -1;
	return nret;
}
int RtEcatSetDO(bool active) {
	int nret = 0;
	bool Active = active;
    if (s_Api.pfnRtEcatSetDO != NULL)
		nret = (*s_Api.pfnRtEcatSetDO)(Active);
	else
		nret = -1;
	return nret;
}
int RtEcatSetEncoderIndex(bool active) {
	int nret = 0;
	bool Active = active;
    if (s_Api.pfnRtEcatSetEncoderIndex != NULL)
		nret = (*s_Api.pfnRtEcatSetEncoderIndex)(Active);
	else
		nret = -1;
	return nret;
}
int RtEcatActiveDelay(bool active) {
	int nret = 0;
	bool Active = active;
    if (s_Api.pfnRtEcatActiveDelay != NULL)
		nret = (*s_Api.pfnRtEcatActiveDelay)(Active);
	else
		nret = -1;
	return nret;
}
int RtEcatSetVerbosity(int verbosity) {
	int nret = 0;
	int Verbosity = verbosity;
    if (s_Api.pfnRtEcatSetVerbosity != NULL)
		nret = (*s_Api.pfnRtEcatSetVerbosity)(Verbosity);
	else
		nret = -1;
	return nret;
}
int RtEcatSetAffinity(int affinity) {
	int nret = 0;
	int Affinity = affinity;
    if (s_Api.pfnRtEcatSetAffinity != NULL)
		nret = (*s_Api.pfnRtEcatSetAffinity)(Affinity);
	else
		nret = -1;
	return nret;
}
int RtEcatSetTorqueActVal(bool active) {
	int nret = 0;
	bool Active = active;
    if (s_Api.pfnRtEcatSetTorqueActVal != NULL)
		nret = (*s_Api.pfnRtEcatSetTorqueActVal)(Active);
	else
		nret = -1;
	return nret;
}
int RtEcatSetVelocityActVal(bool active) {
	int nret = 0;
	bool Active = active;
    if (s_Api.pfnRtEcatSetVelocityActVal != NULL)
		nret = (*s_Api.pfnRtEcatSetVelocityActVal)(Active);
	else
		nret = -1;
	return nret;
}
int RtEcatSetTorqueOffset(bool active) {
	int nret = 0;
	bool Active = active;
    if (s_Api.pfnRtEcatSetTorqueOffset != NULL)
		nret = (*s_Api.pfnRtEcatSetTorqueOffset)(Active);
	else
		nret = -1;
	return nret;
}
int RtEcatSetProfilePosition(bool active) {
	int nret = 0;
	bool Active = active;
    if (s_Api.pfnRtEcatSetProfilePosition != NULL)
		nret = (*s_Api.pfnRtEcatSetProfilePosition)(Active);
	else
		nret = -1;
	return nret;
}
int RtEcatSetHomingMode(bool active) {
	int nret = 0;
	bool Active = active;
    if (s_Api.pfnRtEcatSetHomingMode != NULL)
		nret = (*s_Api.pfnRtEcatSetHomingMode)(Active);
	else
		nret = -1;
	return nret;
}
int ReadBits(unsigned char* src, unsigned char* dst, unsigned long offs, int length) {
	int nret = 0;
	unsigned char* Src = src;
	unsigned char* Dst = dst;
	unsigned long Offs = offs;
	int Length = length;
    if (s_Api.pfnReadBits != NULL)
		nret = (*s_Api.pfnReadBits)(Src, Dst, Offs, Length);
	else
		nret = -1;
	return nret;
}
int WriteBits(unsigned char* dst, unsigned char* src, unsigned long offs, int length) {
	int nret = 0;
	unsigned char* Src = src;
	unsigned char* Dst = dst;
	unsigned long Offs = offs;
	int Length = length;
    if (s_Api.pfnWriteBits != NULL)
		nret = (*s_Api.pfnWriteBits)(Dst, Src, Offs, Length);
	else
		nret = -1;
	return nret;
}
int ReadWord(unsigned short* ptr, unsigned long offs, unsigned short* val) {
	int nret = 0;
	unsigned short* Ptr = ptr;
	unsigned long Offs = offs;
	unsigned short* Val = val;
    if (s_Api.pfnReadWord != NULL)
		nret = (*s_Api.pfnReadWord)(Ptr, Offs, Val);
	else
		nret = -1;
	return nret;
}
int WriteWord(unsigned short* ptr, unsigned short val, unsigned long offs) {
	int nret = 0;
	unsigned short* Ptr = ptr;
	unsigned long Offs = offs;
	unsigned short Val = val;
    if (s_Api.pfnWriteWord != NULL)
		nret = (*s_Api.pfnWriteWord)(Ptr, Val, Offs);
	else
		nret = -1;
	return nret;
}
int ReadDWord(unsigned long* ptr, unsigned long offs, unsigned long* val) {
	int nret = 0;
	unsigned long* Ptr = ptr;
	unsigned long Offs = offs;
	unsigned long* Val = val;
    if (s_Api.pfnReadDWord != NULL)
		nret = (*s_Api.pfnReadDWord)(Ptr, Offs, Val);
	else
		nret = -1;
	return nret;
}
int WriteDWord(unsigned long* ptr, unsigned long val, unsigned long offs) {
	int nret = 0;
	unsigned long* Ptr = ptr;
	unsigned long Offs = offs;
	unsigned long Val = val;
    if (s_Api.pfnWriteDWord != NULL)
		nret = (*s_Api.pfnWriteDWord)(Ptr, Val, Offs);
	else
		nret = -1;
	return nret;
}
int ReadStatusWord(AXIS_ECAT* axis, unsigned short* wStatus) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	unsigned short Val = *wStatus;
    if (s_Api.pfnReadStatusWord != NULL)
		nret = (*s_Api.pfnReadStatusWord)(Axis, &Val);
	else
		nret = -1;
	*wStatus = Val;
	return nret;
}
int ReadActualPosition(AXIS_ECAT* axis, unsigned long* dwPosition) {
	int nret = 0;
	unsigned long Val = *dwPosition;
 	AXIS_ECAT* Axis = axis;
   if (s_Api.pfnReadActualPosition != NULL)
		nret = (*s_Api.pfnReadActualPosition)(Axis, &Val);
	else
		nret = -1;
	*dwPosition = Val;
	return nret;
}
int ReadPositionDemand(AXIS_ECAT* axis, unsigned long* dwPosition) {
	int nret = 0;
	unsigned long Val = *dwPosition;
 	AXIS_ECAT* Axis = axis;
   if (s_Api.pfnReadPositionDemand != NULL)
		nret = (*s_Api.pfnReadPositionDemand)(Axis, &Val);
	else
		nret = -1;
	*dwPosition = Val;
	return nret;
}
int ReadActualTorque(AXIS_ECAT* axis, unsigned long* dwTorque) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	unsigned long Val = *dwTorque;
    if (s_Api.pfnReadActualTorque != NULL)
		nret = (*s_Api.pfnReadActualTorque)(Axis, &Val);
	else
		nret = -1;
	*dwTorque = Val;
	return nret;
}
int ReadActualVelocity(AXIS_ECAT* axis, unsigned long* dwVelocity) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	unsigned long Val = *dwVelocity;
    if (s_Api.pfnReadActualVelocity != NULL)
		nret = (*s_Api.pfnReadActualVelocity)(Axis, &Val);
	else
		nret = -1;
	*dwVelocity = Val;
	return nret;
}
int ReadInternalEncoder(AXIS_ECAT* axis, unsigned long* dwPosition) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	unsigned long Val = *dwPosition;
    if (s_Api.pfnReadInternalEncoder != NULL)
		nret = (*s_Api.pfnReadInternalEncoder)(Axis, &Val);
	else
		nret = -1;
	*dwPosition = Val;
	return nret;
}
int ReadDI(AXIS_ECAT* axis, unsigned long* dwDI) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	unsigned long Val = *dwDI;
    if (s_Api.pfnReadDI != NULL)
		nret = (*s_Api.pfnReadDI)(Axis, &Val);
	else
		nret = -1;
	*dwDI = Val;
	return nret;
}
int ReadIndex(AXIS_ECAT* axis, unsigned char* bIndex) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	unsigned char Val = *bIndex;
    if (s_Api.pfnReadIndex != NULL)
		nret = (*s_Api.pfnReadIndex)(Axis, &Val);
	else
		nret = -1;
	*bIndex = Val;
	return nret;
}
int ReadControlWord(AXIS_ECAT* axis, unsigned short* wControl) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	unsigned short Val = *wControl;
    if (s_Api.pfnReadControlWord != NULL)
		nret = (*s_Api.pfnReadControlWord)(Axis, &Val);
	else
		nret = -1;
	*wControl = Val;
	return nret;
}
int WriteControlWord(AXIS_ECAT* axis, unsigned short val) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	unsigned short Val = val;
    if (s_Api.pfnWriteControlWord != NULL)
		nret = (*s_Api.pfnWriteControlWord)(Axis, Val);
	else
		nret = -1;
	return nret;
}
int WriteModeOP(AXIS_ECAT* axis, unsigned char val) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	unsigned char Val = val;
    if (s_Api.pfnWriteModeOP != NULL)
		nret = (*s_Api.pfnWriteModeOP)(Axis, Val);
	else
		nret = -1;
	return nret;
}
int ReadDO(AXIS_ECAT* axis, unsigned long* dwDO) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	unsigned long Val = *dwDO;
    if (s_Api.pfnReadDO != NULL)
		nret = (*s_Api.pfnReadDO)(Axis, &Val);
	else
		nret = -1;
	*dwDO = Val;
	return nret;
}
int WriteDO(AXIS_ECAT* axis, unsigned long dwDO) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	unsigned long Val = dwDO;
    if (s_Api.pfnWriteDO != NULL)
		nret = (*s_Api.pfnWriteDO)(Axis, Val);
	else
		nret = -1;
	return nret;
}
int WriteTargetPosition(AXIS_ECAT* axis, long val) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	long Val = val;
    if (s_Api.pfnWriteTargetPosition != NULL)
		nret = (*s_Api.pfnWriteTargetPosition)(Axis, Val);
	else
		nret = -1;
	return nret;
}
int WriteTargetVelocity(AXIS_ECAT* axis, long val) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	long Val = val;
    if (s_Api.pfnWriteTargetVelocity != NULL)
		nret = (*s_Api.pfnWriteTargetVelocity)(Axis, Val);
	else
		nret = -1;
	return nret;
}
int WriteTargetTorque(AXIS_ECAT* axis, long val) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	long Val = val;
    if (s_Api.pfnWriteTargetTorque != NULL)
		nret = (*s_Api.pfnWriteTargetTorque)(Axis, Val);
	else
		nret = -1;
	return nret;
}
int WriteTorqueOffset(AXIS_ECAT* axis, long dwTorqueOffset) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	long Val = dwTorqueOffset;
    if (s_Api.pfnWriteTorqueOffset != NULL)
		nret = (*s_Api.pfnWriteTorqueOffset)(Axis, Val);
	else
		nret = -1;
	return nret;
}
int WriteProfileVelocity(AXIS_ECAT* axis, long val) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	long Val = val;
    if (s_Api.pfnWriteProfileVelocity != NULL)
		nret = (*s_Api.pfnWriteProfileVelocity)(Axis, Val);
	else
		nret = -1;
	return nret;
}
int WriteProfileAcceleration(AXIS_ECAT* axis, long val) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	long Val = val;
    if (s_Api.pfnWriteProfileAcceleration != NULL)
		nret = (*s_Api.pfnWriteProfileAcceleration)(Axis, Val);
	else
		nret = -1;
	return nret;
}
int WriteProfileDeceleration(AXIS_ECAT* axis, long val) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	long Val = val;
    if (s_Api.pfnWriteProfileDeceleration != NULL)
		nret = (*s_Api.pfnWriteProfileDeceleration)(Axis, Val);
	else
		nret = -1;
	return nret;
}
int WriteHomingSwitchSpeed(AXIS_ECAT* axis, long val) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	long Val = val;
    if (s_Api.pfnWriteHomingSwitchSpeed != NULL)
		nret = (*s_Api.pfnWriteHomingSwitchSpeed)(Axis, Val);
	else
		nret = -1;
	return nret;
}
int WriteHomingZeroSpeed(AXIS_ECAT* axis, long val) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	long Val = val;
    if (s_Api.pfnWriteHomingZeroSpeed != NULL)
		nret = (*s_Api.pfnWriteHomingZeroSpeed)(Axis, Val);
	else
		nret = -1;
	return nret;
}
int WriteHomingAcceleration(AXIS_ECAT* axis, long val) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	long Val = val;
    if (s_Api.pfnWriteHomingAcceleration != NULL)
		nret = (*s_Api.pfnWriteHomingAcceleration)(Axis, Val);
	else
		nret = -1;
	return nret;
}
int WriteQuickStopDeceleration(AXIS_ECAT* axis, long val) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	long Val = val;
    if (s_Api.pfnWriteQuickStopDeceleration != NULL)
		nret = (*s_Api.pfnWriteQuickStopDeceleration)(Axis, Val);
	else
		nret = -1;
	return nret;
}
int WriteHomingMethod(AXIS_ECAT* axis, char val) {
	int nret = 0;
	AXIS_ECAT* Axis = axis;
	short Val = val;
    if (s_Api.pfnWriteHomingMethod != NULL)
		nret = (*s_Api.pfnWriteHomingMethod)(Axis, Val);
	else
		nret = -1;
	return nret;
}
int CoeSdoDownload(unsigned long dwSlaveId, unsigned short wObIndex, unsigned char byObSubIndex, unsigned char* pbyData, unsigned long dwDataLen,  unsigned long dwTimeout, unsigned long dwFlags ) {
	int nret = 0;
    if (s_Api.pfnCoeSdoDownload != NULL)
		nret = (*s_Api.pfnCoeSdoDownload)(dwSlaveId, wObIndex, byObSubIndex, pbyData, dwDataLen, dwTimeout, dwFlags);
	else
		nret = -1;
	return nret;
}
int CoeSdoUpload(unsigned long dwSlaveId, unsigned short wObIndex, unsigned char byObSubIndex, unsigned char* pbyData, unsigned long dwDataLen, unsigned long* pdwOutDataLen, unsigned long dwTimeout, unsigned long dwFlags ) {
	int nret = 0;
    if (s_Api.pfnCoeSdoUpload != NULL)
		nret = (*s_Api.pfnCoeSdoUpload)(dwSlaveId, wObIndex, byObSubIndex, pbyData, dwDataLen, pdwOutDataLen, dwTimeout, dwFlags);
	else
		nret = -1;
	return nret;
}
int WriteSlaveRegister(int bFixedAddress, unsigned short wSlaveAddress, unsigned short wRegisterOffset, void* pvData, unsigned short wLen, unsigned long dwTimeout) {
	int nret = 0;
    if (s_Api.pfnWriteSlaveRegister != NULL)
		nret = (*s_Api.pfnWriteSlaveRegister)(bFixedAddress, wSlaveAddress, wRegisterOffset, pvData, wLen, dwTimeout);
	else
		nret = -1;
	return nret;
}
int ReadSlaveRegister(int bFixedAddress, unsigned short wSlaveAddress, unsigned short wRegisterOffset, void* pvData, unsigned short wLen, unsigned long dwTimeout) {
	int nret = 0;
    if (s_Api.pfnReadSlaveRegister != NULL)
		nret = (*s_Api.pfnReadSlaveRegister)(bFixedAddress, wSlaveAddress, wRegisterOffset, pvData, wLen, dwTimeout);
	else
		nret = -1;
	return nret;
}
#endif