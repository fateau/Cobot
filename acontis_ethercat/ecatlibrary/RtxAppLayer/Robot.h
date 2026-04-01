#pragma once

#include "Def.h"


class MySystem;
class Jog;
class Motor;
class Script;
class Kinematics;
class HG6Cobot; //add by yunyu 20250423
struct RobotSpec; //add by yunyu 20250423
struct RobotData; //add by yunyu 20250423

class Robot
{
public:
	
	//-------- member --------
	MySystem* sys;			//point to it's parent.
	
	int				rId;
	eRobotType::e	robotType;
	int				motorNum;
	int				gestureNum;
	Jog*			jog;
	Script*			script;
	HG6Cobot*		HGCobot; //add by yunyu 20250423 
	Kinematics*		kinMC;		//for MyCallBack;
	Kinematics*		kinIntp;	//for IntpThread;		(checking before put in intpQ)
	Kinematics*		kinPath;	//for PlanningThread;	(used in raw2Path)
	Motor*			motors[MAX_MOTOR_PER_ROBOT];

	double	poseAtWorld	[MAX_REDUNDANCY];
	double	poseAtRoot	[MAX_REDUNDANCY];
	double	poseAtBase	[MAX_REDUNDANCY];
	double	nextPose	[MAX_REDUNDANCY];
	double	axisDegNow	[MAX_MOTOR_PER_ROBOT];   
	double	axisDegCmd	[MAX_MOTOR_PER_ROBOT];   
	double	axisDegCmdPrev[MAX_MOTOR_PER_ROBOT]; //used for error detection process.(ex angle limit)
	double	axisDegCmdPrevAcc[MAX_MOTOR_PER_ROBOT]; //used for error detection process.(ex angle limit)
	double  axisVelNow	[MAX_MOTOR_PER_ROBOT];
	double  axisVelPrev	[MAX_MOTOR_PER_ROBOT];
	double  axisVelCmd[MAX_MOTOR_PER_ROBOT]; //PMC Modified 11411 //1114
	double  axisVelCmdPrev[MAX_MOTOR_PER_ROBOT]; //PMC Modified 11411 //1114
	double  axisAccNow	[MAX_MOTOR_PER_ROBOT];
	short   axisTorqNow	[MAX_MOTOR_PER_ROBOT];
	int		estimateTorq[MAX_MOTOR_PER_ROBOT]; //add by yunyu 20250505
	int		gravityTorq[MAX_MOTOR_PER_ROBOT];  //PMC Modified 11412//1229
	int		nextTorq	[MAX_MOTOR_PER_ROBOT]; //modify by yunyu 20250505
	int		errorCount	[MAX_MOTOR_PER_ROBOT]; //add by yunyu 20250505
	
	double error[MAX_REDUNDANCY];

	bool			isNeedMove;
	int				isAtSyncId;		// 表示劇本正停在sync#的地方。預設值為-1
	
	double	world2Base_T[4][4];
	double	base2World_T[4][4];
	double	world2Root_T[4][4];	
	double	root2World_T[4][4];
	double	world2Ref_T[4][4];
	double	ref2Root_T[4][4];

	//-------- function --------
	Robot::Robot(MySystem* sys, int rId, RobotDeclareData* data);
	~Robot(void);

	//馬達控制
	void StartServoOnOff(); 
	void isServoOn();
	int  updateData();					//read Axis And Pose, update Matrices.
	void resetCmd();
	void resetCmdPrev();
	void writeNextAxisForAllMotors();
	void writeNextTorqForAllMotors();
	bool isAxisJump();

	//運動學
	bool isReachAngleLimit_next();
	double* getThetaInit(); //add by yunyu 20250423
	double* getRefRoot(); //PMC Modified 11410
	void getaxisAccNow(); //PMC Modified 11411 //1114

private:
	double* masteringData;	//指向 shm 的 robotData.spec.masteringData
	RobotData* rData; //add by yunyu 20250505
	RobotSpec* spec; //add by yunyu 20250423
	double	refRoot[7];		//ref T root. 參考點到基座的轉置矩陣

	double  nowBase[6];
	int		nowBaseId;
	

	//歸零補償
	void masteringDataAdd(double* outCompDeg, double* inRawDeg);
	void masteringDataRemove(double* outRawDeg, double* inCompDeg);

	//add by yunyu 20250505
	//協作功能計算及判斷
	void calculateTorq();
	void updateCollisionState();

	void setSimulationData();
};

