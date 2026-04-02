#pragma once

#include "Def.h"

struct BaseData
{
	double Pose[6]; //XYZABC
};
struct ToolData
{
	double Pose[6]; //XYZABC
};
struct IOModuleData
{
	int		inpByteNum;
	int		outByteNum;
	BYTE	inpBytes[MAX_INP_BYTE_PER_IO];		// Ecat偵測到的值
	BYTE	outBytes[MAX_OUT_BYTE_PER_IO];		// Ecat偵測到的值
	BYTE	outBytesCmd[MAX_OUT_BYTE_PER_IO];	// 從人機來下給Ecat的指令
};
struct MotorData
{
	BOOL	isNeedServoOn;
	BOOL	isNeedServoOff;
	BOOL	isServoOn;
	//PMC Modified 11412 //1229
	double	Kpp, Kvp, Kvi;        // 用來儲存從驅動器讀回來的數值 (給 HMI 顯示用)
	double	cmdKpp, cmdKvp, cmdKvi; // HMI 設定下來的數值 (暫存用)

	bool	isNeedSetPID;           // 旗標：通知 RTX 執行寫入 PID
	bool	isNeedSaveParams;       // 旗標：通知 RTX 執行 Save to Flash
	bool	isNeedReadPID;          // 旗標：通知 RTX 執行讀取 PID
};
struct RobotSpec
{
	double	reductionRatio	[MAX_MOTOR_PER_ROBOT];	// 減速比 = gearRatio * 皮帶比 * 導程
	double  masteringData	[MAX_MOTOR_PER_ROBOT];
	DHtable DH;
	double  maxAxisVel		[MAX_MOTOR_PER_ROBOT];
	double  maxTCPVel;								// XYZ(mm) 和 R(deg) 共用一個最大速度
	short	maxAxisTorq		[MAX_MOTOR_PER_ROBOT];  //add by yunyu 20250505
	URDFtable URDF;  //PMC Modified 11412//1209
};
struct RobotData
{
	RobotSpec spec;
	MotorData motors	[MAX_MOTOR_PER_ROBOT];	

	double	axisDegNow	[MAX_MOTOR_PER_ROBOT];
	double	velocity	[MAX_MOTOR_PER_ROBOT];
	short	torque		[MAX_MOTOR_PER_ROBOT];
	int 	estimatetorque	[MAX_MOTOR_PER_ROBOT]; //add by yunyu 20250505
	double	poseAtWorld	[MAX_REDUNDANCY];		//在世界座標下的姿態
	double	poseAtRoot	[MAX_REDUNDANCY];		//在RootBase下的姿態
	double	poseAtBase	[MAX_REDUNDANCY];		//在目前Base下的姿態
};
struct ScriptData
{
	bool			isHMICmdTerminate;
	int				nowLineId; 
	int				nowFuncId;
	ScriptRawCmd	scriptRawCmd;
};
struct HomingData  
{
	int				rId;
	int				mId;
};
struct SHMData
{
	eError::e		errorCode;
	char			errorMsg[ ERROR_MSG_LEN];

	eRTXState::e	RTXState;
	eRTXMode::e		RTXMode;
	eProject::e		project;
	int				NIC;		// Network Card #
	int				ecatState;
	int				masterId;
	int				slaveMotorNum;
	int				slaveIONum;
	BOOL			stopRTX;
	bool			isApplyToRealMotor;
	bool			isNeedResetDegree;	// for WIN32_SIMULATE

    //PMC Modified 11410	
	bool			CollisionMode[MAX_ROBOT_NUM];
	int             CollisionState[MAX_ROBOT_NUM];  

	RobotData		robots[MAX_ROBOT_NUM];
	BaseData		bases[MAX_BASE_NUM];
	ToolData		tools[MAX_TOOL_NUM];
		
	eCmdSource::e	cmdSource;	//Jog or Script
	eCmdFormat::e	cmdFormat;	//Axis,Pose, or Point

	// ===== Jog ======
	JogCmd			jogCmd;
	int				jogBaseIds[MAX_ROBOT_NUM];
	int				jogToolIds[MAX_ROBOT_NUM];
	eFrame::e		jogFrame;

	// ===== Script ====
	ScriptData		scriptData[MAX_ROBOT_NUM];
	bool			isScriptRunning;
	bool			isSlowStop;
	bool			stopScript;
	double			vGain;
	eSyncState::e	syncTable[MAX_SYNC_NUM][MAX_ROBOT_NUM + 1]; // last Column for "ALL"

	// ===== HandGuide ==== //PMC Modified 11410
	struct HGParams		hgParams[MAX_ROBOT_NUM];

	// ===== IO =======
	IOModuleData		ioModules		[MAX_IO_NUM];
	
	// ====== Robot Declaration ========
	RobotDeclareData	robotDeclareTable[MAX_ROBOT_NUM];
	EcatMappingData		ecatMappingTable[MAX_MOTOR_NUM];

	// == Scurve ===== 
	int 		Jerk;	 
	int         onScurve;

	// == Record ===
	char logPath[256];

	//== Homing Mode===
	HomingData			homingData;
};