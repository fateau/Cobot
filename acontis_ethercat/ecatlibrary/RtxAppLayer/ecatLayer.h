#include <windows.h>
#include <Rtapi.h>
#include <tchar.h>
/*-----------------------------------------------------------------------------
 * EcatLayer.h              header file
 * Copyright                IntervalZero
 * Response                 Yves de la Broise
 * Description              Description of the EtherCAT API functions
 *---------------------------------------------------------------------------*/
#ifdef RTX_EXPORTS
#define RtxEcatLayer_API extern "C" __declspec(dllexport)
#else
#define RtxEcatLayer_API extern "C" __declspec(dllimport)
#endif

#define NO_ERRORS		0
#define ERROR_OCCURED	-1


/*-TYPEDEFS------------------------------------------------------------------*/
#ifndef ECAT_LAYER
#define ECAT_LAYER

typedef enum _EC_T_STATE
{
    eEcatState_UNKNOWN  = 0,  /*< unknown */
    eEcatState_INIT     = 1,  /*< init */
    eEcatState_PREOP    = 2,  /*< pre-operational */
    eEcatState_SAFEOP   = 4,  /*< safe operational */
    eEcatState_OP       = 8,  /*< operational */
    eEcatState_BOOTSTRAP = 3, /*< BootStrap */
} EC_T_STATE;


typedef struct
{
    unsigned int	    dwVendorId;
    unsigned int		dwProductCode;
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


typedef int (__stdcall *AppCallback)(void* Context, EC_T_STATE MasterState);

#endif
/*-----------------------------------------------------------------------------
 * Settings Functions
 * These functions must be called after EcatCreateMaster but before EcatStartMaster.
 * EtherCAT protocol settings cannot be modified at run-time so calling these functions after starting the master is useless.
 * These functions are not mandatory they all have default values.
 *---------------------------------------------------------------------------*/
/********************************************************************************/
/** Function to change the EtherCAT cycle.
* The default period is 1000 micro-seconds. The minimum period is 100 micro-seconds.
* If you plan to use a period bellow 500 micro-seconds it is recommanded to test because it might not work depending on your network topology.
* Due to EthherCAT specificity the cyclic timer is inside the EcatLayer. The callback function is called every cycle.
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI RtEcatSetTimer(int period);
/** Function to change the EtherCAT Handle Thread Priority.
* The default priority is 127.
* This will only change the priority of the thread for the callback you registered.
* The priority for the timer thread controling the EtherCAT port cannot be changed to make sure the EhterCAT link is stable.
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI RtEcatSetThreadPriority(int priority);
/********************************************************************************/
/** Function to change the Motor control mode.
* The possible modes are 0 for Position control and 1 for Velocity control.
* The default mode is Position control mode 0.
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI RtEcatSetMode (int mode);
/********************************************************************************/
/** Function to change the Redundancy.
* The default is desactivated.
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI RtEcatSetRedundancy (bool active);
RtxEcatLayer_API int RTAPI RtEcatSetDC (bool active);
RtxEcatLayer_API int RTAPI RtEcatSetMOP (bool active);
RtxEcatLayer_API int RTAPI RtEcatSetExtEncoder (bool active);
RtxEcatLayer_API int RTAPI RtEcatSetDIO (bool active);
RtxEcatLayer_API int RTAPI RtEcatSetEncoderIndex (bool active);
/********************************************************************************/
/** Function to set NIC used for redundancy.
* IN driver: Only with Acontis master. String giving the driver to use for this NIC.
*			 Possible drivers are "-i8255x", "-i8254x", "-rtl8139" and "-rtl8169".
* IN polling: Only with Acontis master. The message mode of the NIC card Message (polling) or Interrupt line.
*			  All recent NIC should be in polling mode.
* IN instance: Instance of the card. Starts at one. With Koenig master driver, polling mode and instance are set in the RtTcpIp.ini file.
*			   In the ini the cards start at rtnd0 which corresponds to instance 1.
*			   When using Acontis master the NIC should not be set in the RtTcpIp.ini otherwise it will be controlled by the IntervalZero driver and not available for the Acontis driver.
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI RtEcatSetRedNIC(char driver[16], bool polling, int instance);
/********************************************************************************/
/** Function to change the Verbosity.
* Verbosity level goes from 0 to 3 to control the number of messages displayed.
* The default value is 2.
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI RtEcatSetVerbosity (int verbosity);
/********************************************************************************/
/** Function to change the Affinity.
* The default Affinity is the one from the application.
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI RtEcatSetAffinity (int affinity);

/*-----------------------------------------------------------------------------
 * Run Functions
 * These functions must be called in this order for the EtherCAT master to work properly.
 *---------------------------------------------------------------------------*/
/********************************************************************************/
/** Function to initialize the EtherCAT master.
* IN master: 0 for the Acontis master, 1 for the Koenig master
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI RtEcatCreateMaster (int master);
/********************************************************************************/
/** Function to set NIC used by the master.
* IN driver: Only with Acontis master. String giving the driver to use for this NIC.
*			 Possible drivers are "-i8255x", "-i8254x", "-rtl8139" and "-rtl8169".
* IN polling: Only with Acontis master. The message mode of the NIC card Message (polling) or Interrupt line.
*			  All recent NIC should be in polling mode.
* IN instance: Instance of the card. Starts at one. With Koenig master driver, polling mode and instance are set in the RtTcpIp.ini file.
*			   In the ini the cards start at rtnd0 which corresponds to instance 1.
*			   When using Acontis master the NIC should not be set in the RtTcpIp.ini otherwise it will be controlled by the IntervalZero driver and not available for the Acontis driver.
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI RtEcatSetNIC(char driver[16], bool polling, int instance);
/********************************************************************************/
/** Function to start the EtherCAT master.
* IN call: pointer to the cyclic callback function. This function will be called every cycle.
* 		   The cyclic function should be very short to avoid interfering with the EtherCAT cycle.
*		   This function should only be used to read and write the slave variables.
*		   There is a timeout on this function to protect the cycle so if it is too long the cycle might be updated with incorrect values.
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI RtEcatStartMaster (AppCallback call, int* slaves, void* context = 0);
RtxEcatLayer_API int RTAPI RtEcatRegisterCallback( AppCallback call, int timeSpan, int* index, int* slaves, void* context = 0);
/********************************************************************************/
/** Function to stop the EtherCAT master.
* Please set a delay after calling this function to give it time to unload the master.
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI RtEcatStopMaster ();
RtxEcatLayer_API int RTAPI RtEcatUnregisterCallback(int callback);
RtxEcatLayer_API int RTAPI RtEcatGetNotification(NotificationData *data, bool *NotifLeft);

/*-----------------------------------------------------------------------------
 * Variable Functions
 * These functions should be used inside the callback function to update the slave values.
 *---------------------------------------------------------------------------*/
/********************************************************************************/
/** Function to Read Variables.
* This function is meant to read digital I/O.
* IN src: pointer to the Input or Ouput buffer. The buffer addresses are pasted in the IO_ECAT structure.
* OUT dst: point to a buffer that will receive the value. The buffer length should be at least as long as the length parameter.
* IN offs: offset of the slave variable inside the buffer.
* IN length: length of the variable in bit
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI ReadBits (unsigned char* src, unsigned char* dst, unsigned long offs, int length);
/********************************************************************************/
/** Function to Write Variables.
* This function is meant to read digital I/O.
* IN dst: pointer to the Input or Ouput buffer. The buffer addresses are pasted in the IO_ECAT structure.
* IN src: point to a buffer that contains the value. The buffer length should be at least as long as the length parameter.
* IN offs: offset of the slave variable inside the buffer.
* IN length: length of the variable in bit
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI WriteBits (unsigned char* dst, unsigned char* src, unsigned long offs, int length);
/********************************************************************************/
/** Function to Read Variables.
* IN ptr: pointer to the variable. The pPdoIn or pPdoOut in the MOTOR_ECAT structure.
*		  With IO_ECAT structure if the length is in bytes use the buffer addresses with the offset.
*		  Example: pInp[dwInpOff]
* OUT val: pointer to the variable that will recieve the value read
*
* \return error code
*/
RtxEcatLayer_API int RTAPI ReadWord (unsigned short* ptr, unsigned long offs, unsigned short* val);
/********************************************************************************/
/** Function to Write Variables.
* IN ptr: pointer to the variable. The pPdoOut in the MOTOR_ECAT structure.
*		  With IO_ECAT structure if the length is in bytes use the buffer addresses with the offset.
*		  Example: pOut[dwOutOff]
* IN val: value to be writen.
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI WriteWord (unsigned short* ptr, unsigned short val, unsigned long offs);
/********************************************************************************/
/** Function to Read Variables.
* IN ptr: pointer to the variable. The pPdoIn or pPdoOut in the MOTOR_ECAT structure.
*		  With IO_ECAT structure if the length is in bytes use the buffer addresses with the offset.
*		  Example: pInp[dwInpOff]
* OUT val: pointer to the variable that will recieve the value read
*
* \return error code
*/
RtxEcatLayer_API int RTAPI ReadDWord (unsigned long* ptr, unsigned long offs, unsigned long* val);
/********************************************************************************/
/** Function to Write Variables.
* IN ptr: pointer to the variable. The pPdoOut in the MOTOR_ECAT structure.
*		  With IO_ECAT structure if the length is in bytes use the buffer addresses with the offset.
*		  Example: pOut[dwOutOff]
* OUT val: value to be writen.
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI WriteDWord (unsigned long* ptr, unsigned long val, unsigned long offs);
/********************************************************************************/
/** Function to Read The motor status word.
* This function is made to simplify the control of motors.
* The proper offset is applied by this function to access the status word variable with different motors.
* IN axis: pointer to the MOTOR_ECAT structure of the motor to control.
* OUT wStatus: pointer to the variable that will recieve the value read
*
* \return error code
*/
RtxEcatLayer_API int RTAPI ReadStatusWord (AXIS_ECAT* axis, unsigned short* wStatus);
/********************************************************************************/
/** Function to Read The motor actual position.
* This function is made to simplify the control of motors.
* The proper offset is applied by this function to access the actual position variable with different motors.
* IN axis: pointer to the MOTOR_ECAT structure of the motor to control.
* OUT dwPosition: pointer to the variable that will recieve the value read
*
* \return error code
*/
RtxEcatLayer_API int RTAPI ReadActualPosition (AXIS_ECAT* axis, unsigned long* dwPosition);
RtxEcatLayer_API int RTAPI ReadInternalEncoder (AXIS_ECAT* axis, unsigned long* dwPosition);
RtxEcatLayer_API int RTAPI ReadDI (AXIS_ECAT* axis, unsigned long* dwDI);
RtxEcatLayer_API int RTAPI ReadIndex (AXIS_ECAT* axis, unsigned char* bIndex);
/********************************************************************************/
/** Function to Read The motor control word.
* This function is made to simplify the control of motors.
* The proper offset is applied by this function to access the control word variable with different motors.
* IN axis: pointer to the MOTOR_ECAT structure of the motor to control.
* OUT wControl: pointer to the variable that will recieve the value read
*
* \return error code
*/
RtxEcatLayer_API int RTAPI ReadControlWord (AXIS_ECAT* axis, unsigned short* wControl);
/********************************************************************************/
/** Function to Write The motor control word.
* This function is made to simplify the control of motors.
* The proper offset is applied by this function to access the control word variable with different motors.
* IN axis: pointer to the MOTOR_ECAT structure of the motor to control.
* IN val: value to be writen.
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI WriteControlWord (AXIS_ECAT* axis, unsigned short val);
RtxEcatLayer_API int RTAPI WriteModeOP (AXIS_ECAT* axis, unsigned char val);
RtxEcatLayer_API int RTAPI ReadDO (AXIS_ECAT* axis, unsigned long* dwDO);
RtxEcatLayer_API int RTAPI WriteDO (AXIS_ECAT* axis, unsigned long dwDO);
/********************************************************************************/
/** Function to Write The motor target position.
* This function is made to simplify the control of motors.
* The proper offset is applied by this function to access the target position variable with different motors.
* IN axis: pointer to the MOTOR_ECAT structure of the motor to control.
* IN val: value to be writen.
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI WriteTargetPosition (AXIS_ECAT* axis, long val);
/********************************************************************************/
/** Function to Write The motor target velocity.
* This function is made to simplify the control of motors.
* The proper offset is applied by this function to access the target velocity variable with different motors.
* IN axis: pointer to the MOTOR_ECAT structure of the motor to control.
* IN val: value to be writen.
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI WriteTargetVelocity (AXIS_ECAT* axis, long val);
RtxEcatLayer_API int RTAPI WriteTargetTorque (AXIS_ECAT* axis, long val);
/********************************************************************************/
/** Function to Write SDO variables.
* This function can be used to access slave variables that are not mapped to the cyclic data.
* IN dwSlaveId: Id of the slave to access.
* IN wObIndex: Object index.
* IN byObSubIndex: Object sub index.
* IN pbyData: Data to be transferred.
* IN dwDataLen: Data size of pbyData.
* IN dwTimeout: Timeout in milliseconds. The function will block at most for this time.
* IN dwFlags: Mailbox Flags. Bit 0: if 1 Complete Access.
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI CoeSdoDownload ( unsigned long dwSlaveId, unsigned short wObIndex, unsigned char byObSubIndex, unsigned char* pbyData, unsigned long dwDataLen,  unsigned long dwTimeout, unsigned long dwFlags );
/********************************************************************************/
/** Function to Read SDO variables.
* This function can be used to access slave variables that are not mapped to the cyclic data.
* IN dwSlaveId: Id of the slave to access.
* IN wObIndex: Object index.
* IN byObSubIndex: Object sub index.
* IN pbyData: Data buffer to upload data to.
* IN dwDataLen: Data size of pbyData.
* OUT pdwOutDataLen: Pointer returning size of data uploaded from slave.
* IN dwTimeout: Timeout in milliseconds. The function will block at most for this time.
* IN dwFlags: Mailbox Flags. Bit 0: if 1 Complete Access.
* 
* \return error code
*/
RtxEcatLayer_API int RTAPI CoeSdoUpload ( unsigned long dwSlaveId, unsigned short wObIndex, unsigned char byObSubIndex, unsigned char* pbyData, unsigned long dwDataLen, unsigned long* pdwOutDataLen, unsigned long dwTimeout, unsigned long dwFlags );



