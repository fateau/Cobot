#include "ShmAPI.h"
#include "RtxAppLayer/HelpFunctions.h"

#include <stdio.h>
#include <string.h> // For strncpy, strcpy

// Linux/POSIX headers for shared memory and sleep
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/stat.h>

SHMData* shm;

int shm_fd = -1; // Use file descriptor for shared memory in Linux
const char* SHM_NAME = "/DataSpace";

extern DLLEXPORT int init()
{
	int shm_size = sizeof(SHMData);
	shm_fd = shm_open(SHM_NAME, O_RDWR, 0666);
	if (shm_fd == -1) return -1;
	shm = (SHMData*) mmap(0, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

	if(shm == MAP_FAILED) return -1;

	// reset Robot/Motor Declaring table (HMI will fill these)
	for(int r = 0; r < MAX_ROBOT_NUM; r++) 
		shm->robotDeclareTable[r].motorNum = -1;	
	
	for(int m = 0; m < MAX_MOTOR_NUM; m++) 
		shm->ecatMappingTable[m].robotInd = -1;

	// Do NOT overwrite RTXState, ecatState, stopRTX, etc.
	// Those are controlled by ecatApp.

	return 1;
}
extern DLLEXPORT int resetValue()	//called when init() and everyTime RTX is connected.
{
	shm->project = eProject::BASIC;

	int r,m,b;
	shm->stopRTX = false;
	shm->RTXMode = eRTXMode::CSP;
	shm->masterId = 0;
	// reset Motors
	for(r = 0; r < MAX_ROBOT_NUM; r++) {
		for(m = 0; m < MAX_MOTOR_PER_ROBOT; m++) {
			shm->robots[r].motors[m].isNeedServoOn = false;  
			shm->robots[r].motors[m].isNeedServoOff = false; 
			shm->robots[r].motors[m].isServoOn = false;
		}
    }

	// reset Script
	shm->isScriptRunning = false;
	shm->stopScript = true;

	for(r = 0; r < MAX_ROBOT_NUM; r++) {
		shm->scriptData[r].isHMICmdTerminate = true;
		shm->scriptData[r].nowLineId = 0;
		shm->scriptData[r].nowFuncId = -1;
		shm->scriptData[r].scriptRawCmd.info.myMasterId = -1; 
		shm->robots[r].spec.maxTCPVel = 1e4;
		for(m=0; m < MAX_MOTOR_PER_ROBOT; m++)
		{
			shm->robots[r].spec.maxAxisVel[m] = 1e4;
			shm->robots[r].spec.maxAxisTorq[m] = 3e4; //單位 : 0.1%。 因為short的範圍是-3萬多 ~3萬多 。所以設一個不超過3萬做為上限 //add by yunyu 20250505
		}
	}

	// reset IO 
	for(m = 0; m < MAX_IO_NUM; m++) 
		for(b = 0; b < MAX_OUT_BYTE_PER_IO; b++)
			shm->ioModules[m].outBytesCmd[b] = 0; //每次連線時，IO-OUT 皆為OFF。
	

	return 1;
}
extern DLLEXPORT int closeShm()
{
	if (shm != MAP_FAILED) munmap(shm, sizeof(SHMData));
	if (shm_fd != -1) ::close(shm_fd);
	// Do NOT shm_unlink here — ecatApp owns the shm lifecycle.
	// Unlinking would destroy the segment while ecatApp is still running.

	return 1;
}

// ======= Start and Terminate .rtss Process =====
extern DLLEXPORT int Set_MasterId(int masterId)
{
	shm->masterId = masterId;
	return 1;
}
extern DLLEXPORT eRTXState::e Get_RTXState()
{
	return shm->RTXState;
}
extern DLLEXPORT int Get_EcatState(int& ecatState)
{
	ecatState = shm->ecatState;
	return 1;
}
extern DLLEXPORT int StopRTXProcess()
{
	/* Send servo-off to all motors before stopping */
	for (int r = 0; r < MAX_ROBOT_NUM; r++) {
		for (int m = 0; m < MAX_MOTOR_PER_ROBOT; m++) {
			if (shm->robots[r].motors[m].isServoOn) {
				shm->robots[r].motors[m].isNeedServoOff = true;
			}
		}
	}
	usleep(500 * 1000);	//wait for motor to servoOff. (500 ms)
	shm->stopRTX = 1;
	return 1;
}
extern DLLEXPORT int Get_SlaveInfo(int& motorNum, int& ioNum)
{
	motorNum = shm->slaveMotorNum;
	ioNum	 = shm->slaveIONum;
	return 1;
}
extern DLLEXPORT void Set_RTXMode(eRTXMode::e mode)
{
	shm->RTXMode = mode;
}
extern DLLEXPORT eRTXMode::e Get_RTXMode()
{
	return shm->RTXMode;
}
extern DLLEXPORT void Set_NIC(int NIC)
{
	shm->NIC = NIC;
}
extern DLLEXPORT eError::e Get_ErrorCode(char* out_msg)
{
	strncpy(out_msg, shm->errorMsg, 256);
	return shm->errorCode;
}

// ============== Motor Control ===============
extern DLLEXPORT int Set_ServoOn(int r, int m, char onOff)
{
	if( r<0 || r> MAX_ROBOT_NUM) return -1;
	if( m<0 || m> MAX_MOTOR_PER_ROBOT) return -1;

	if(onOff == TRUE)
		shm->robots[r].motors[m].isNeedServoOn = true;
	else
		shm->robots[r].motors[m].isNeedServoOff = true;
	return 1;
}
extern DLLEXPORT char Get_IsServoOn(int r, int m)
{
	if( r<0 || r> MAX_ROBOT_NUM) return 0;
	if( m<0 || m> MAX_MOTOR_PER_ROBOT) return 0;
	return shm->robots[r].motors[m].isServoOn;
}
extern DLLEXPORT unsigned short Get_StatusWord(int r, int m)
{
	if( r<0 || r> MAX_ROBOT_NUM) return 0;
	if( m<0 || m> MAX_MOTOR_PER_ROBOT) return 0;
	return shm->robots[r].motors[m].statusWord;
}
extern DLLEXPORT void Set_IsApplyToRealMotor(bool isApplyToRealMotor)
{
	shm->isApplyToRealMotor = isApplyToRealMotor;
}
extern DLLEXPORT char Get_IsApplyToRealMotor()
{
	return shm->isApplyToRealMotor;
}

// ============== Robot Control ===============
extern DLLEXPORT void Get_AxisDeg(int r, double* axisDeg)
{
	for(int i = 0; i<shm->robotDeclareTable[r].motorNum; i++) 
		axisDeg[i] = shm->robots[r].axisDegNow[i];
	
}
extern DLLEXPORT void Get_Pose(int r, double* poseAtWorld, double* poseAtRoot, double* poseAtBase)
{
	RobotData* rData = &(shm->robots[r]);
	for(int i = 0; i<shm->robotDeclareTable[r].gestureNum; i++) {
		poseAtWorld[i] = rData->poseAtWorld[i];
		poseAtRoot [i] = rData->poseAtRoot[i];
		poseAtBase [i] = rData->poseAtBase[i];
	}
}
extern DLLEXPORT void Get_Velocity(int r, double* velocity)
{
	for(int i = 0; i<shm->robotDeclareTable[r].motorNum; i++) {
		velocity[i] = shm->robots[r].velocity[i];
	}
}
extern DLLEXPORT void Get_Torque(int r, double* torque)
{
	for(int i = 0; i<shm->robotDeclareTable[r].motorNum; i++) 
		torque[i] = shm->robots[r].torque[i];
	
}
//add by yunyu 20250505
extern DLLEXPORT void Get_EstimateTorq(int r, int* estimatetorque)
{
		for(int i = 0; i<shm->robotDeclareTable[r].motorNum; i++) 
			estimatetorque[i] = shm->robots[r].estimatetorque[i];
}
extern DLLEXPORT void Set_MasteringData(int r, int m, double* masteringData)
{
	// Always write mastering data to shared memory (Linux port fix)
	RobotSpec* spec = &(shm->robots[r].spec);
	for(int i = 0; i< MAX_MOTOR_PER_ROBOT; i++) 
		spec->masteringData[i] = masteringData[i];

	shm->homingData.rId = r;
	shm->homingData.mId = m;
}
extern DLLEXPORT void Get_MasteringData(int r, double* masteringData)
{
	RobotSpec* spec = &(shm->robots[r].spec);
	for(int i = 0; i< MAX_MOTOR_PER_ROBOT; i++) 
		masteringData[i] = spec->masteringData[i];
	
}
extern DLLEXPORT void Set_ReductionRatio(int r, double* ratio)
{
	RobotData* robot = &(shm->robots[r]);
	for(int i = 0; i< MAX_MOTOR_PER_ROBOT; i++) {
		robot->spec.reductionRatio[i] = ratio[i];
	}
}
extern DLLEXPORT void Set_DHtable(int r, double* a, double* alpha, double* d, double* thInit, double* thShift,
								  double* posLimit, double* negLimit, eJointType::e* jointType)
{
	DHtable* DH = &(shm->robots[r].spec.DH);
	for(int i = 0; i< MAX_MOTOR_PER_ROBOT; i++) {
		DH->a[i]					= a[i];
		DH->alphaDeg[i]				= alpha[i];
		DH->d[i]					= d[i];
		DH->thetasInit[i]			= thInit[i];
		DH->thetaShiftDeg[i]		= thShift[i];
		DH->axisPositiveLimit[i]	= posLimit[i];
		DH->axisNegativeLimit[i]	= negLimit[i];
		DH->jointType[i]			= jointType[i];
	}
}

extern DLLEXPORT void Set_MaxVelArray(int r, char isAxis, double* vel)
{
	RobotSpec* spec = &(shm->robots[r].spec);
	if(isAxis) {
		for(int i = 0; i< MAX_MOTOR_PER_ROBOT; i++) 
			spec->maxAxisVel[i] = vel[i];
	}
	else 
		spec->maxTCPVel = vel[0];
}
extern DLLEXPORT void Set_MaxVel(int r, int m, char isAxis, double vel)
{
	RobotSpec* spec = &(shm->robots[r].spec);
	if(isAxis)
		spec->maxAxisVel[m] = vel;
	else
		spec->maxTCPVel = vel;		// XYZ(mm) 和 R(deg) 共用一個最大速度
}

// ============== Process Command ============
extern DLLEXPORT int Set_CmdSource(eCmdSource::e source)
{
	shm->cmdSource = source;
	return 1;
}
extern DLLEXPORT int Set_CmdFormat(eCmdFormat::e format)
{
	shm->cmdFormat = format;
	return 1;
}

// ============== Jog ===============
extern DLLEXPORT void Set_JogAcc(double acc)
{
	shm->jogCmd.acc = acc;
	return;
}
extern DLLEXPORT void Set_JogCmdSingle(int r, int m, double vel, double dist, bool isLimitDist)
{
	shm->jogCmd.r	 = r;
	shm->jogCmd.ind	 = m;
	shm->jogCmd.dist = dist;
	shm->jogCmd.vel  = vel;
	shm->jogCmd.isUserPressJOG = true;
	shm->jogCmd.isLimitDist = isLimitDist;
	
	return;
}
extern DLLEXPORT void Set_UserStopPressJog()
{
	shm->jogCmd.isUserPressJOG = false;
	shm->jogCmd.vel = 0;
	return;
}
extern DLLEXPORT void Set_JogFrame(eFrame::e frame)
{
	shm->jogFrame = frame;
	return;
}

// ============== HG Cobot ============ //PMC Modified 11410
extern DLLEXPORT void Set_HG6Cobot(int r, HGParams HGParams)
{
	shm->hgParams[r] = HGParams;
}
extern DLLEXPORT bool Set_CollisionMode(int r,bool CollisionMode)
{
	return shm->CollisionMode[r] = CollisionMode;
}
extern DLLEXPORT int Set_MaxTolerateTime(int r, int MaxTolerateTime)
{
	return shm->hgParams[r].MaxTolerateTime = MaxTolerateTime;
}
extern DLLEXPORT int Get_CollisionState(int r)
{
		return shm->CollisionState[r];
}
extern DLLEXPORT int Set_CollisionState(int r,int CollisionState)
{
		return shm->CollisionState[r] = CollisionState;
}

extern DLLEXPORT void Set_URDF(int r, double* x, double* y, double* z, double* Rx, double* Ry,
	double* Rz, double* weight, double* center_x, double* center_y, double* center_z)
{
	URDFtable* URDF = &(shm->robots[r].spec.URDF);
	for (int i = 0; i < MAX_MOTOR_PER_ROBOT; i++) {
		URDF->x[i] = x[i];
		URDF->y[i] = y[i];
		URDF->z[i] = z[i];
		URDF->Rx[i] = Rx[i];
		URDF->Ry[i] = Ry[i];
		URDF->Rz[i] = Rz[i];
		URDF->weight[i] = weight[i];
		URDF->center_x[i] = center_x[i];
		URDF->center_y[i] = center_y[i];
		URDF->center_z[i] = center_z[i];
	}
}

// ============== PID Control ============ 	//PMC Modified 11412 //1229
extern DLLEXPORT void Set_PID(int r, int m, double kpp, double kvp, double kvi)
{
	if (r < 0 || r >= MAX_ROBOT_NUM) return;
	if (m < 0 || m >= MAX_MOTOR_PER_ROBOT) return;

	shm->robots[r].motors[m].cmdKpp = kpp;
	shm->robots[r].motors[m].cmdKvp = kvp;
	shm->robots[r].motors[m].cmdKvi = kvi;

	shm->robots[r].motors[m].isNeedSetPID = true;
}
extern DLLEXPORT void Get_PID(int r, int m, double* kpp, double* kvp, double* kvi)
{
	if (r < 0 || r >= MAX_ROBOT_NUM) return;
	if (m < 0 || m >= MAX_MOTOR_PER_ROBOT) return;

	*kpp = shm->robots[r].motors[m].Kpp;
	*kvp = shm->robots[r].motors[m].Kvp;
	*kvi = shm->robots[r].motors[m].Kvi;

	shm->robots[r].motors[m].isNeedReadPID = true;
}
extern DLLEXPORT void Save_MotorParams(int r, int m)
{
	if (r < 0 || r >= MAX_ROBOT_NUM) return;
	if (m < 0 || m >= MAX_MOTOR_PER_ROBOT) return;

	shm->robots[r].motors[m].isNeedSaveParams = true;
}

// ============== Script (Compiling) =======
extern DLLEXPORT void Clear_SyncTable()
{
	for(int s = 0; s < MAX_SYNC_NUM; s++) 
		for(int r = 0; r < MAX_ROBOT_NUM; r++)
			shm->syncTable[s][r] = eSyncState::NONE;
}
extern DLLEXPORT void Set_SyncTable(int r, int syncId)
{
	shm->syncTable[syncId][r] = eSyncState::NOT_YET;
}

// ============== Script ===============
extern DLLEXPORT void StopScript(char isSlowStop)
{
	shm->isSlowStop = isSlowStop & 1;
	shm->stopScript = true;
}
extern DLLEXPORT void Set_IsHMIScriptTerminate(int r, char isTerminate)
{
	shm->scriptData[r].isHMICmdTerminate = isTerminate & 1;
}
extern DLLEXPORT char Get_IsScriptRunning(){
	return (char)shm->isScriptRunning;
}
extern DLLEXPORT int  Get_NowLineId(int r)
{
	return shm->scriptData[r].nowLineId;
}
extern DLLEXPORT int  Get_NowFuncId(int r)
{
	return shm->scriptData[r].nowFuncId;
}

extern DLLEXPORT int  Set_VGain(double vGain)
{
	shm->vGain = vGain;
	return 1;
}


void setCmdMoveRaw_GeneralParam(CmdMoveRaw* cmdMR, MoveData mv)
{
	cmdMR->mvData = mv;
}
extern DLLEXPORT int Set_ScriptRawCmd_Index(int r, int lineId, int funcId)
{
	ScriptRawCmd* rCmd = &(shm->scriptData[r].scriptRawCmd);
	rCmd->info.lineId		= lineId;
	rCmd->info.funcId		= funcId;
	return 1;
}
extern DLLEXPORT int Set_ScriptRawCmd_Move(
						int r, eCmdFormat::e format, eMovePathType::e path, eFrame::e frame,
						char isAbs, MoveData mv,  
						double* axisPose, char* axisPoseMask, int scriptBaseIndex, int scriptToolIndex 
						)
{
	CmdMoveRaw* cmdMR = &(shm->scriptData[r].scriptRawCmd.cmdMoveRaw);
	shm->scriptData[r].scriptRawCmd.info.type = eScriptCmdType::MOVE;

	cmdMR->mvData = mv;

	cmdMR->isAbs = isAbs;
	cmdMR->format = format;
	cmdMR->frame = frame;
	cmdMR->pathType = path;
	cmdMR->scriptBaseIndex = scriptBaseIndex;
	cmdMR->scriptToolIndex = scriptToolIndex;


	// set Axis & Pose
	if(format == eCmdFormat::AXIS) {
		for(int i = 0; i< MAX_MOTOR_PER_ROBOT; i++) {				
			cmdMR->axis[i] = axisPose[i];
			cmdMR->axisMask[i] = axisPoseMask[i];
		}
		
	}
	else if(format == eCmdFormat::POSE) {
		for(int i = 0; i< MAX_REDUNDANCY; i++) {				
			cmdMR->Pose[i] = axisPose[i];
			cmdMR->PoseMask[i] = axisPoseMask[i];
		}
		
	}

	return 1;
}

extern DLLEXPORT int Set_ScriptRawCmd_MoveRecordPoint(
						int r, eCmdFormat::e format, eMovePathType::e path, 
						double* axis, double* pose, MoveData mv, 
						int ptBase, int scriptBaseIndex, int scriptToolIndex )
{
	CmdMoveRaw* cmdMR	= &(shm->scriptData[r].scriptRawCmd.cmdMoveRaw);
	shm->scriptData[r].scriptRawCmd.info.type = eScriptCmdType::MOVE;
	
	setCmdMoveRaw_GeneralParam(cmdMR, mv);

	cmdMR->isAbs = true;
	cmdMR->format = format;
	cmdMR->pathType = path;
	cmdMR->scriptBaseIndex = scriptBaseIndex;
	cmdMR->scriptToolIndex = scriptToolIndex;
	cmdMR->pointBaseIndex = ptBase;

	// set Axis & Pose
	CopyArray(cmdMR->axis, axis, MAX_MOTOR_PER_ROBOT);
	CopyArray(cmdMR->Pose, pose, MAX_REDUNDANCY);
	
	return 1;
}

extern DLLEXPORT int Set_ScriptRawCmd_MoveCircle(
						int r, MoveData mv, 
						double* poseMid, double* poseEnd, double theta, int ptBaseMid, int ptBaseEnd,
						int scriptBaseIndex, int scriptToolIndex )
{
	CmdMoveRaw* cmdMR		= &(shm->scriptData[r].scriptRawCmd.cmdMoveRaw);
	shm->scriptData[r].scriptRawCmd.info.type = eScriptCmdType::MOVE;
	
	setCmdMoveRaw_GeneralParam(cmdMR, mv);

	cmdMR->theta = theta;
	cmdMR->isAbs = 1;
	cmdMR->format = eCmdFormat::PT_TCP;
	cmdMR->pathType = eMovePathType::CIRCLE;
	cmdMR->scriptBaseIndex = scriptBaseIndex;
	cmdMR->scriptToolIndex = scriptToolIndex;

	cmdMR->pointBaseIndex  = ptBaseMid;
	cmdMR->pointBaseIndex2 = ptBaseEnd;

	CopyArray(cmdMR->Pose,  poseMid,  MAX_REDUNDANCY);
	CopyArray(cmdMR->Pose2, poseEnd, MAX_REDUNDANCY);

	return 1;
}

extern DLLEXPORT int Set_ScriptRawCmd_IO(int r, int ioCmdNum, IOData* ioDatas)
{	
	ScriptRawCmd* rCmd	= &(shm->scriptData[r].scriptRawCmd);
	rCmd->info.type = eScriptCmdType::IO_OUT;
	rCmd->cmdIo.ioCmdNum = ioCmdNum;

	for(int i = 0; i<ioCmdNum; i++)
	{
		rCmd->cmdIo.io[i] = ioDatas[i];
	}

	return 1;
}
extern DLLEXPORT int Set_ScriptRawCmd_Delay(int r, int time)
{	
	shm->scriptData[r].scriptRawCmd.info.type = eScriptCmdType::DELAY;

	shm->scriptData[r].scriptRawCmd.cmdDelay.time = time;
	return 1;
}
extern DLLEXPORT int Set_ScriptRawCmd_Sync(int r, int syncId, eSyncType::e type)
{	
	shm->scriptData[r].scriptRawCmd.info.type = eScriptCmdType::SYNC;
	shm->scriptData[r].scriptRawCmd.cmdSync.syncId = syncId;
	shm->scriptData[r].scriptRawCmd.cmdSync.type = type;
	return 1;
}
extern DLLEXPORT int Set_ScriptRawCmd_HMISync(int r)
{	
	shm->scriptData[r].scriptRawCmd.info.type = eScriptCmdType::HMI_SYNC;
	return 1;
}
extern DLLEXPORT int Set_ScriptRawCmd_MS(int r, int myMasterId)
{	
	shm->scriptData[r].scriptRawCmd.info.type = eScriptCmdType::MS;
	shm->scriptData[r].scriptRawCmd.info.myMasterId = myMasterId;
	return 1;
}

// ============== IO ===============
extern DLLEXPORT int Get_IOInfo(int ioInd, int& ioInpByteNum, int& ioOutByteNum)
{
	
	ioInpByteNum = shm->ioModules[ioInd].inpByteNum;
	ioOutByteNum = shm->ioModules[ioInd].outByteNum;

	return 1;
}
extern DLLEXPORT int Get_IOBytes(int ind, BYTE* ioInBytes, BYTE* ioOutBytes)
{
	for(int i = 0; i< shm->ioModules[ind].inpByteNum; i++) {
		ioInBytes[i] = shm->ioModules[ind].inpBytes[i];
	}

	for(int i = 0; i< shm->ioModules[ind].outByteNum; i++) {
		ioOutBytes[i] = shm->ioModules[ind].outBytes[i];
	}
	return 1;
}
extern DLLEXPORT int Set_IO_OUTByte(int mind, int bInd, BYTE ioOutByte)
{
	shm->ioModules[mind].outBytesCmd[bInd] = ioOutByte;	
	return 1;
}
extern DLLEXPORT int Set_IO_INByte(int mind, int bInd, BYTE ioInByte)
{
	#if defined WIN32_SIMULATE || defined LINUX_SIMULATE
	shm->ioModules[mind].inpBytes[bInd] = ioInByte;
	#endif
	return 1;
}

extern DLLEXPORT void Set_IOBit(int mId, int allBitId, bool isOut, bool value)
{
	int byteId = allBitId / 8;
    int bitId  = allBitId % 8;
	
	// 將指定的IO bit 值設為value
	if(isOut) {
		BYTE* number = &(shm->ioModules[mId].outBytesCmd[byteId]);
		*number = (*number & ~(1UL << bitId)) | (value << bitId);
	}
	else{
		#if defined WIN32_SIMULATE || defined LINUX_SIMULATE
		BYTE* number = &(shm->ioModules[mId].inpBytes[byteId]);
		*number = (*number & ~(1UL << bitId)) | (value << bitId);
		#endif
	}	
}
extern DLLEXPORT void Set_IOBit_Toggle(int mId, int allBitId, bool isOut)
{
	int byteId = allBitId / 8;
    int bitId  = allBitId % 8;

	// 將指定的IO bit 值反轉
	if(isOut)
		shm->ioModules[mId].outBytesCmd[byteId] ^= 1 << bitId;	
	else{
		#if defined WIN32_SIMULATE || defined LINUX_SIMULATE
		shm->ioModules[mId].inpBytes[byteId] ^= 1 << bitId;
		#endif
	}
}


// ============== RobotDeclare ===============
extern DLLEXPORT int Set_RobotDeclare(int ind, int motorNum,double refRoot[6], eRobotType::e type)
{
	if(ind < 0 || ind >= MAX_ROBOT_NUM) return -1;

	RobotDeclareData* data = &(shm->robotDeclareTable[ind]);
	data->motorNum	= motorNum;
	data->robotType = type;

	for(int i = 0; i < 6; i++)
		data->refRoot[i]	= refRoot[i];

	// set gestureNum;
	data->gestureNum = 6;

	return 1;
}
extern DLLEXPORT int Set_EcatMapping(int ind, int robotInd, int axisInd)
{
	if(ind < 0 || ind >= MAX_MOTOR_NUM) return -1;

	shm->ecatMappingTable[ind].robotInd = robotInd;
	shm->ecatMappingTable[ind].axisInd = axisInd;
	return 1;
}

// ============== DataBase ===============

extern DLLEXPORT void Set_ToolBase(eToolBaseType::e type, int ind, double* Pose)
{
	if(ind < 0 || ind >= MAX_TOOL_NUM) return;

	if(type == eToolBaseType::TOOL) 
		CopyArray(shm->tools[ind].Pose, Pose, 6);
	else 
		CopyArray(shm->bases[ind].Pose, Pose, 6);
	
}
extern DLLEXPORT void Set_JogToolInd(int rId, int toolId)
{
	shm->jogToolIds[rId] = toolId;
}
extern DLLEXPORT void Set_JogBaseInd(int rId, int baseId)
{
	shm->jogBaseIds[rId] = baseId;
}

// === for Scurve === 
extern DLLEXPORT void Set_CurveType(int onScurve)
{
	shm->onScurve = onScurve;
}
extern DLLEXPORT void Set_Jerk(double Jerk)	
{
	shm->Jerk = Jerk;
}

// === Record ==== 
extern DLLEXPORT void  Set_BinPath(char *path)
{
	// Use strncpy for safety to prevent buffer overflows
	strncpy(shm->logPath, path, sizeof(shm->logPath) - 1);
	shm->logPath[sizeof(shm->logPath) - 1] = '\0'; // Ensure null-termination
}

// === Cross-process HMI signaling ====
extern DLLEXPORT void Set_HMICommandReady()
{
	shm->hmiCommandReady = true;
}
