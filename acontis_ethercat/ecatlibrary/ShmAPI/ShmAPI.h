#if defined KING_STAR_IO
#include <Rtapi.h>
#endif

#include "RtxAppLayer/Shm.h"

#if defined(_WIN32) || defined(_WIN64)
    // Windows (MSVC)
    #define DLLEXPORT __declspec(dllexport)
    #define DLLIMPORT __declspec(dllimport)
#elif defined(__linux__) || defined(__APPLE__)
    // Linux, macOS (GCC, Clang)
    #define DLLEXPORT __attribute__((visibility("default")))
    #define DLLIMPORT
#endif

#ifdef __cplusplus
	extern "C" 
	{
#endif

// ======= Start and Terminate SharedMemory ======
extern DLLEXPORT int  init();
extern DLLEXPORT int  closeShm();
extern DLLEXPORT int  resetValue();

// ======= Start and Terminate .rtss Process =====
extern DLLEXPORT int  Set_MasterId(int masterId);
extern DLLEXPORT eRTXState::e Get_RTXState();
extern DLLEXPORT int  Get_EcatState(int& ecatState);
extern DLLEXPORT int  StopRTXProcess();
extern DLLEXPORT int  Get_SlaveInfo(int& motorNum, int& ioNum);
extern DLLEXPORT void Set_RTXMode(eRTXMode::e mode);
extern DLLEXPORT eRTXMode::e Get_RTXMode(); 
extern DLLEXPORT void Set_NIC(int NIC);
extern DLLEXPORT eError::e Get_ErrorCode(char* out_msg);

// ============== Motor Control ===============
extern DLLEXPORT int Set_ServoOn(int r, int m, char onOff);
extern DLLEXPORT char Get_IsServoOn(int r, int m);
extern DLLEXPORT unsigned short Get_StatusWord(int r, int m);
extern DLLEXPORT void Set_IsApplyToRealMotor(bool isApplyToRealMotor);
extern DLLEXPORT char Get_IsApplyToRealMotor();

// ============== Robot Control ===============
extern DLLEXPORT void Get_AxisDeg(int r, double* axisDeg);
extern DLLEXPORT void Get_Pose(int r, double* poseAtWorld, double* poseAtRoot, double* poseAtBase);
extern DLLEXPORT void Get_Velocity(int r, double* velocity);
extern DLLEXPORT void Get_Torque(int r, double* torque);
extern DLLEXPORT void Get_EstimateTorq(int r, int* estimatetorque); //add by yunyu 20250505

extern DLLEXPORT void Get_MasteringData(int r, double* masteringData);
extern DLLEXPORT void Set_MasteringData(int r, int m,double* masteringData); //0706 IFang
extern DLLEXPORT void Set_ReductionRatio(int r, double* ratio);
extern DLLEXPORT void Set_DHtable(int r, double* a, double* alpha, double* d, double* thInit, double* thShift, 
								  double* posLimit, double* negLimit, eJointType::e* jointType);

extern DLLEXPORT void Set_MaxVelArray(int r, char isAxis, double* vel);
extern DLLEXPORT void Set_MaxVel(int r, int m, char isAxis, double vel);

//extern DLLEXPORT void Set_MaxTorqArray(int r, short Torq); 
//extern DLLEXPORT void Set_MaxTorq(int r, int m, short torq);

// ============== Process Command ============
extern DLLEXPORT int Set_CmdSource(eCmdSource::e source);
extern DLLEXPORT int Set_CmdFormat(eCmdFormat::e format);

// ============== Jog ===============
extern DLLEXPORT void Set_JogAcc(double acc);
extern DLLEXPORT void Set_JogCmdSingle(int r, int m, double vel, double dist, bool isLimitDist);
extern DLLEXPORT void Set_UserStopPressJog();
extern DLLEXPORT void Set_JogFrame(eFrame::e frame);

// ============== HG Cobot ============ //add by yunyu 20250505
extern DLLEXPORT void Set_HG6Cobot(int r, HGParams HGParams);
extern DLLEXPORT bool Set_CollisionMode(int r,bool CollisionMode);
extern DLLEXPORT int Set_MaxTolerateTime(int r, int MaxTolerateTime);
extern DLLEXPORT int Get_CollisionState(int r);
extern DLLEXPORT int Set_CollisionState(int r,int CollisionState);
extern DLLEXPORT void Set_URDF(int r, double* x, double* y, double* z, double* Rx, double* Ry,
	double* Rz, double* weight, double* center_x, double* center_y,  double* center_z); //PMC Modified 11412//1209

// ============== PID Control ============ 	//PMC Modified 11412 //1229
extern DLLEXPORT void Set_PID(int r, int m, double kpp, double kvp, double kvi);
extern DLLEXPORT void Get_PID(int r, int m, double* kpp, double* kvp, double* kvi);
extern DLLEXPORT void Save_MotorParams(int r, int m);

// ============== Script (Compiling) =======
extern DLLEXPORT void Clear_SyncTable();
extern DLLEXPORT void Set_SyncTable(int r, int syncId);

// ============== Script ===============
extern DLLEXPORT void StopScript(char isSlowStop);
extern DLLEXPORT void Set_IsHMIScriptTerminate(int r, char isTerminate);
extern DLLEXPORT char Get_IsScriptRunning();
extern DLLEXPORT int  Get_NowLineId(int r);
extern DLLEXPORT int  Get_NowFuncId(int r);
extern DLLEXPORT int  Set_VGain(double vGain);


extern DLLEXPORT int  Set_ScriptRawCmd_Index(int r, int lineId, int funcId);
extern DLLEXPORT int  Set_ScriptRawCmd_Move(
						int r, eCmdFormat::e format, eMovePathType::e path, eFrame::e frame,
						char isAbs, MoveData mv, 
						double* axisPose, char* axisPoseMask, int scriptBaseIndex, int scriptToolIndex);
extern DLLEXPORT int  Set_ScriptRawCmd_MoveRecordPoint(
						int r, eCmdFormat::e format, eMovePathType::e path, 
						double* axis, double* pose, MoveData mv, 
						int ptBase, int scriptBaseIndex, int scriptToolIndex 
						);
extern DLLEXPORT int  Set_ScriptRawCmd_MoveCircle(
						int r, MoveData mv, 
						double* poseMid, double* poseEnd, double theta, int ptBaseMid, int ptBaseEnd,
						int scriptBaseIndex, int scriptToolIndex
						);
extern DLLEXPORT int  Set_ScriptRawCmd_IO(int r, int ioCmdNum, IOData* ioDatas);
extern DLLEXPORT int  Set_ScriptRawCmd_Delay(int r, int time);
extern DLLEXPORT int  Set_ScriptRawCmd_Sync(int r, int syncId, eSyncType::e type);
extern DLLEXPORT int  Set_ScriptRawCmd_HMISync(int r);
extern DLLEXPORT int  Set_ScriptRawCmd_MS(int r, int myMasterId);
// ============== IO ===============
extern DLLEXPORT int  Get_IOInfo(int ioInd, int& ioInByteNums, int& ioOutByteNums);
extern DLLEXPORT int  Get_IOBytes(int ind, BYTE* ioInBytes, BYTE* ioOutBytes);
extern DLLEXPORT int  Set_IO_OUTByte(int mind, int bInd, BYTE ioOutByte);
extern DLLEXPORT int  Set_IO_INByte(int mind, int bInd, BYTE ioInByte);
extern DLLEXPORT void  Set_IOBit(int mId, int allBitId, bool isOut, bool value);
extern DLLEXPORT void  Set_IOBit_Toggle(int mId, int allBitId, bool isOut);  

// ============== RobotDeclare ============
extern DLLEXPORT int  Set_RobotDeclare(int ind, int motorNum,double* refRoot, eRobotType::e type);
extern DLLEXPORT int  Set_EcatMapping(int ind, int robotInd, int axisInd);

// ============== DataBase ===============
extern DLLEXPORT void Set_ToolBase(eToolBaseType::e type, int ind, double* pose);
extern DLLEXPORT void Set_JogToolInd(int rId, int toolId);
extern DLLEXPORT void Set_JogBaseInd(int rId, int baseId);

// === for Scurve === 
extern DLLEXPORT void Set_CurveType(int onScurve);
extern DLLEXPORT void Set_Jerk(double Jerk);

// === Record ====
extern DLLEXPORT void  Set_BinPath(char *path);

// === Cross-process HMI signaling ====
extern DLLEXPORT void  Set_HMICommandReady();

#ifdef __cplusplus
	}
#endif
